#!/usr/bin/env python3
# Reference decoder for the WAVELET (stream_type=3) parser test. Builds the SAME
# synthetic per-octave wavelet packets as wavelet_parse_test.cpp and decodes them
# EXACTLY the way mz-unified-wavelet/remote/net.py does:
#   * UnifiedSink._handle_wavelet  -> per-OCTAVE SEQ continuity (the loss check)
#   * receive_wavelet              -> reassemble the multirate scalogram surface
#     by HOLDING each octave's last value between its (rate-aligned) updates.
# Emits the same machine-readable summary keys so run_wavelet_test.sh can diff
# the C++ plugin reassembly logic against this net.py-style reference byte-for-byte.
#
# Header (docs/unified-packet-format.md, WAVELET section):
#   w0 MAGIC=0xCAFEBABE | w1 TYPE_VER (stream_type=3 | version<<8)
#   w2/w3 = 64-bit timestamp
#   w4 = SEQ (PER-OCTAVE; +1 per packet of THIS octave -- the loss check)
#   w5 = AUX0 = octave[3:0] | n_octaves[7:4] | n_voices[11:8] | overrun[24]
#   w6 = AUX1 = n_channels[7:0] | lane_start[23:8]
#   w7 = RSVD
#   w8.. = n_channels*n_voices complex bins (re,im int32), lane-major/voice-minor.
import struct

UNIFIED_MAGIC = 0xCAFEBABE
UNIFIED_VERSION = 1
COMMON_HEADER_WORDS = 8
STREAM_TYPE_WAVELET = 3


def build_wavelet(seq, ts, octave, n_octaves, n_voices, n_channels, lane_start,
                  overrun, first_re, first_im):
    """One per-octave wavelet datagram, to the spec. Bins are filled so each
    (local channel c, voice v) carries a deterministic value:
        re = first_re + (c*n_voices + v)
        im = first_im - (c*n_voices + v)
    (lane-major, voice-minor) so the decoder can check ordering, not just count."""
    hdr = [0] * COMMON_HEADER_WORDS
    hdr[0] = UNIFIED_MAGIC
    hdr[1] = STREAM_TYPE_WAVELET | (UNIFIED_VERSION << 8)
    hdr[2] = ts & 0xFFFFFFFF
    hdr[3] = (ts >> 32) & 0xFFFFFFFF
    hdr[4] = seq & 0xFFFFFFFF
    hdr[5] = ((octave & 0xF) | ((n_octaves & 0xF) << 4) |
              ((n_voices & 0xF) << 8) | ((overrun & 1) << 24))
    hdr[6] = (n_channels & 0xFF) | ((lane_start & 0xFFFF) << 8)
    hdr[7] = 0
    bins = []
    for c in range(n_channels):
        for v in range(n_voices):
            k = c * n_voices + v
            bins.append(first_re + k)   # re
            bins.append(first_im - k)   # im
    return struct.pack("<8I", *hdr) + struct.pack(f"<{len(bins)}i", *bins)


class Decoder:
    """Mirrors net.py: per-octave SEQ loss check + a held multirate surface."""

    def __init__(self):
        self.wav_pkts = 0
        self.wav_seq_gaps = 0          # gap EVENTS, summed over octaves
        self.wav_lost = 0              # total octave packets implied missing
        self.last_seq = {}             # octave -> last SEQ (per-octave loss check)
        self.surface = {}              # octave -> list[(re,im)] per (lane,voice), HELD
        self.n_oct = 0
        self.n_voc = 0
        self.last_lane_start = 0
        self.last_n_chan = 0
        self.last_overrun = 0
        self.last_ts = 0

    def demux(self, data):
        if len(data) < COMMON_HEADER_WORDS * 4:
            return
        magic, type_ver = struct.unpack("<II", data[:8])
        if magic != UNIFIED_MAGIC:
            return
        if (type_ver & 0xFF) != STREAM_TYPE_WAVELET:
            return
        self.wavelet(data)

    def wavelet(self, data):
        HDR = COMMON_HEADER_WORDS * 4
        ts_lo, ts_hi, seq, aux0, aux1 = struct.unpack("<IIIII", data[8:28])
        ts = ts_lo | (ts_hi << 32)
        octave = aux0 & 0xF
        n_oct = (aux0 >> 4) & 0xF
        n_voc = (aux0 >> 8) & 0xF
        overrun = (aux0 >> 24) & 1
        n_chan = aux1 & 0xFF
        lane_start = (aux1 >> 8) & 0xFFFF
        payload = data[HDR:]
        nints = len(payload) // 4
        expect = n_chan * n_voc * 2
        if n_voc == 0 or nints < expect:
            return                      # truncated / wrong cfg -- drop
        vals = struct.unpack(f"<{expect}i", payload[:expect * 4])

        # per-OCTAVE SEQ continuity = the loss check
        ls = self.last_seq.get(octave)
        if ls is not None and seq != (ls + 1) & 0xFFFFFFFF:
            missing = (seq - (ls + 1)) & 0xFFFFFFFF
            self.wav_seq_gaps += 1
            self.wav_lost += missing
        self.last_seq[octave] = seq
        self.wav_pkts += 1

        # HOLD: place this octave's block into the surface (lane-major/voice-minor)
        self.surface[octave] = [(vals[2 * i], vals[2 * i + 1])
                                for i in range(n_chan * n_voc)]
        self.n_oct = n_oct
        self.n_voc = n_voc
        self.last_lane_start = lane_start
        self.last_n_chan = n_chan
        self.last_overrun = overrun
        self.last_ts = ts

    def surface_checksum(self):
        """Order-sensitive checksum of the HELD multirate surface (all octaves,
        each at its last-held value). Walk octaves ascending, then (lane,voice)
        in payload order, folding re,im in -- catches any mis-ordering / wrong
        hold."""
        acc = 0
        for o in sorted(self.surface.keys()):
            for (re, im) in self.surface[o]:
                acc = (acc * 1000003 + (re & 0xFFFFFFFF)) & 0xFFFFFFFFFFFFFFFF
                acc = (acc * 1000003 + (im & 0xFFFFFFFF)) & 0xFFFFFFFFFFFFFFFF
        return acc


def main():
    # 3 octaves, V=4, K=5 lanes. Octave 0 updates every step (fast), octave 1
    # every 2 steps, octave 2 every 4 steps -- the rate-aligned multirate
    # cadence (octave o at 3 kHz / 2^o). We DELIBERATELY DROP one octave-0 packet
    # (skip a SEQ) and one octave-1 packet to exercise per-octave gap detection.
    d = Decoder()
    n_oct, n_voc, n_chan, lane_start = 3, 4, 5, 0
    seqs = {0: 500, 1: 9000, 2: 77000}
    ts = 0
    NSTEPS = 40
    for step in range(NSTEPS):
        # octave 0 every step
        if step == 7:
            seqs[0] += 1            # DROP one octave-0 packet (skip a SEQ)
        d.demux(build_wavelet(seqs[0], ts, 0, n_oct, n_voc, n_chan, lane_start,
                              0, 0x1000 + step, 0x2000 + step))
        seqs[0] += 1
        # octave 1 every 2 steps
        if step % 2 == 0:
            if step == 10:
                seqs[1] += 1       # DROP one octave-1 packet
            d.demux(build_wavelet(seqs[1], ts, 1, n_oct, n_voc, n_chan, lane_start,
                                  0, 0x3000 + step, 0x4000 + step))
            seqs[1] += 1
        # octave 2 every 4 steps; last one sets overrun
        if step % 4 == 0:
            ov = 1 if step == NSTEPS - 4 else 0
            d.demux(build_wavelet(seqs[2], ts, 2, n_oct, n_voc, n_chan, lane_start,
                                  ov, 0x5000 + step, 0x6000 + step))
            seqs[2] += 1
        ts += 1

    print(f"wav_pkts={d.wav_pkts}")
    print(f"wav_seq_gaps={d.wav_seq_gaps}")
    print(f"wav_lost={d.wav_lost}")
    print(f"wav_n_octaves={d.n_oct}")
    print(f"wav_n_voices={d.n_voc}")
    print(f"wav_last_lane_start={d.last_lane_start}")
    print(f"wav_last_n_chan={d.last_n_chan}")
    print(f"wav_last_overrun={d.last_overrun}")
    print(f"wav_last_ts={d.last_ts}")
    print(f"wav_octaves_held={sorted(d.surface.keys())}")
    print(f"wav_surface_checksum={d.surface_checksum()}")


if __name__ == "__main__":
    main()
