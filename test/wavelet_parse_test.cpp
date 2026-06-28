// ===========================================================================
// Standalone WAVELET (stream_type=3) parser test (no JUCE / no Open Ephys).
//
// Re-implements the EXACT decode + reassembly logic that ephys-socket's
// Source/IntanInterface.cpp (demuxDatagram / processWaveletDatagram) and
// Source/IntanSocket.cpp (the held multirate scalogram surface in
// processWaveletPacket) use for the UNIFIED octave-split wavelet format, and
// asserts it against synthetic per-octave packets built to the spec in
// mz-unified-wavelet docs/unified-packet-format.md (WAVELET section).
//
// Goal: prove the plugin's per-OCTAVE SEQ-gap detection AND the rate-aligned
// multirate surface reassembly (hold each octave between its updates) match the
// firmware byte-for-byte -- the same bytes net.py's UnifiedSink._handle_wavelet
// + receive_wavelet decode. The companion ref_decode_wavelet.py decodes the SAME
// synthetic packets the net.py way; run_wavelet_test.sh diffs the two summaries.
//
// Build:  g++ -std=c++17 -O2 wavelet_parse_test.cpp -o wavelet_parse_test
// Run:    ./wavelet_parse_test      (emits a machine-readable summary)
// ===========================================================================
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include <map>
#include <string>

// ---- constants copied verbatim from IntanInterface.cpp -------------------
static constexpr uint32_t UNIFIED_MAGIC       = 0xCAFEBABE;
static constexpr uint8_t  UNIFIED_VERSION     = 1;
static constexpr size_t   COMMON_HEADER_WORDS = 8;
static constexpr uint8_t  STREAM_TYPE_WAVELET = 3;
static constexpr int      kMaxWaveletOctaves  = 16;

static inline uint32_t unpackU32LE(const uint8_t* b) {
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
           ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}
static inline void packU32LE(std::vector<uint8_t>& v, uint32_t x) {
    v.push_back(x & 0xFF); v.push_back((x >> 8) & 0xFF);
    v.push_back((x >> 16) & 0xFF); v.push_back((x >> 24) & 0xFF);
}
static inline void packI32LE(std::vector<uint8_t>& v, int32_t x) {
    packU32LE(v, (uint32_t)x);
}

// One decoded octave packet handed to the surface (mirror of WaveletPacket).
struct WaveletPacket {
    uint64_t       timestamp;
    uint32_t       sequence;
    uint8_t        octave, nOctaves, nVoices, nChannels;
    bool           overrun;
    uint16_t       laneStart;
    const int32_t* bins;
    size_t         binCount;   // nChannels*nVoices*2
};

// ---- decode state: IntanInterface per-octave SEQ check + IntanSocket's held
//      multirate surface (the reassembly the canvas reads) -----------------
struct Decoder {
    // per-octave SEQ loss tracking (IntanInterface::processWaveletDatagram)
    bool     wavHaveSeq[kMaxWaveletOctaves] = { false };
    uint32_t wavLastSeq[kMaxWaveletOctaves] = { 0 };
    uint64_t wavPkts = 0, wavSeqGaps = 0, wavLost = 0;

    // held multirate surface (IntanSocket): octave -> last-held block of
    // (re,im) per (lane,voice), in payload order. Held between octave updates.
    std::map<int, std::vector<std::pair<int32_t,int32_t>>> surface;
    int      nOct = 0, nVoc = 0;
    uint16_t lastLaneStart = 0; uint8_t lastNChan = 0; bool lastOverrun = false;
    uint64_t lastTs = 0;

    void demux(const uint8_t* data, size_t len) {
        if (len < COMMON_HEADER_WORDS * 4) return;
        uint32_t magic   = unpackU32LE(data);
        uint32_t typeVer = unpackU32LE(data + 4);
        if (magic != UNIFIED_MAGIC) return;
        if ((uint8_t)(typeVer & 0xFF) != STREAM_TYPE_WAVELET) return;
        wavelet(data, len);
    }

    // IntanInterface::processWaveletDatagram (decode + per-octave SEQ check),
    // then IntanSocket's held-surface placement.
    void wavelet(const uint8_t* data, size_t len) {
        const size_t HDR = COMMON_HEADER_WORDS * 4;
        uint64_t ts   = (uint64_t)unpackU32LE(data + 8) |
                        ((uint64_t)unpackU32LE(data + 12) << 32);
        uint32_t seq  = unpackU32LE(data + 16);
        uint32_t aux0 = unpackU32LE(data + 20);
        uint32_t aux1 = unpackU32LE(data + 24);
        uint8_t  octave    = (uint8_t)(aux0 & 0xF);
        uint8_t  nOctaves  = (uint8_t)((aux0 >> 4) & 0xF);
        uint8_t  nVoices   = (uint8_t)((aux0 >> 8) & 0xF);
        bool     overrun   = ((aux0 >> 24) & 0x1) != 0;
        uint8_t  nChannels = (uint8_t)(aux1 & 0xFF);
        uint16_t laneStart = (uint16_t)((aux1 >> 8) & 0xFFFF);

        size_t binsInt32 = (size_t)nChannels * (size_t)nVoices * 2;
        if (nVoices == 0 || len - HDR < binsInt32 * sizeof(int32_t)) return;

        // per-OCTAVE SEQ continuity = the loss check
        if (octave < kMaxWaveletOctaves) {
            if (wavHaveSeq[octave]) {
                uint32_t exp = wavLastSeq[octave] + 1;
                if (seq != exp) { uint32_t miss = seq - exp; wavSeqGaps++; wavLost += miss; }
            }
            wavLastSeq[octave] = seq; wavHaveSeq[octave] = true;
        }
        wavPkts++;

        const int32_t* bins = reinterpret_cast<const int32_t*>(data + HDR);
        WaveletPacket pkt{ ts, seq, octave, nOctaves, nVoices, nChannels,
                           overrun, laneStart, bins, binsInt32 };
        placeIntoSurface(pkt);
    }

    // The reassembly the canvas relies on: HOLD each octave between updates.
    void placeIntoSurface(const WaveletPacket& p) {
        std::vector<std::pair<int32_t,int32_t>> blk;
        blk.reserve((size_t)p.nChannels * p.nVoices);
        for (size_t i = 0; i < (size_t)p.nChannels * p.nVoices; ++i)
            blk.emplace_back(p.bins[2 * i + 0], p.bins[2 * i + 1]);
        surface[p.octave] = std::move(blk);
        nOct = p.nOctaves; nVoc = p.nVoices;
        lastLaneStart = p.laneStart; lastNChan = p.nChannels;
        lastOverrun = p.overrun; lastTs = p.timestamp;
    }

    // Order-sensitive checksum of the HELD surface (octaves ascending, payload
    // order). MUST fold identically to ref_decode_wavelet.surface_checksum().
    uint64_t surfaceChecksum() const {
        uint64_t acc = 0;
        for (const auto& kv : surface)            // std::map iterates ascending
            for (const auto& ri : kv.second) {
                acc = acc * 1000003ull + (uint32_t)ri.first;
                acc = acc * 1000003ull + (uint32_t)ri.second;
            }
        return acc;
    }
};

// ---- synthetic per-octave packet builder (to the spec) -------------------
static std::vector<uint8_t> buildWavelet(uint32_t seq, uint64_t ts, uint8_t octave,
                                         uint8_t nOctaves, uint8_t nVoices,
                                         uint8_t nChannels, uint16_t laneStart,
                                         uint8_t overrun, int32_t firstRe, int32_t firstIm) {
    std::vector<uint8_t> v;
    packU32LE(v, UNIFIED_MAGIC);                                              // w0
    packU32LE(v, (uint32_t)STREAM_TYPE_WAVELET | (UNIFIED_VERSION << 8));      // w1
    packU32LE(v, (uint32_t)(ts & 0xFFFFFFFF));                                // w2
    packU32LE(v, (uint32_t)(ts >> 32));                                      // w3
    packU32LE(v, seq);                                                       // w4
    packU32LE(v, ((uint32_t)(octave & 0xF) | ((uint32_t)(nOctaves & 0xF) << 4) |
                  ((uint32_t)(nVoices & 0xF) << 8) | ((uint32_t)(overrun & 1) << 24))); // w5 AUX0
    packU32LE(v, (uint32_t)(nChannels & 0xFF) | ((uint32_t)(laneStart & 0xFFFF) << 8)); // w6 AUX1
    packU32LE(v, 0);                                                         // w7 RSVD
    for (int c = 0; c < (int)nChannels; ++c)
        for (int vv = 0; vv < (int)nVoices; ++vv) {
            int k = c * (int)nVoices + vv;
            packI32LE(v, firstRe + k);   // re
            packI32LE(v, firstIm - k);   // im
        }
    return v;
}

int main() {
    Decoder d;
    const uint8_t nOct = 3, nVoc = 4, nChan = 5; const uint16_t laneStart = 0;
    std::map<int,uint32_t> seqs{ {0, 500}, {1, 9000}, {2, 77000} };
    uint64_t ts = 0;
    const int NSTEPS = 40;
    for (int step = 0; step < NSTEPS; ++step) {
        // octave 0 every step; DROP one at step 7
        if (step == 7) seqs[0]++;
        { auto p = buildWavelet(seqs[0], ts, 0, nOct, nVoc, nChan, laneStart, 0,
                                0x1000 + step, 0x2000 + step);
          d.demux(p.data(), p.size()); seqs[0]++; }
        // octave 1 every 2 steps; DROP one at step 10
        if (step % 2 == 0) {
            if (step == 10) seqs[1]++;
            auto p = buildWavelet(seqs[1], ts, 1, nOct, nVoc, nChan, laneStart, 0,
                                  0x3000 + step, 0x4000 + step);
            d.demux(p.data(), p.size()); seqs[1]++;
        }
        // octave 2 every 4 steps; the second-last sets overrun
        if (step % 4 == 0) {
            uint8_t ov = (step == NSTEPS - 4) ? 1 : 0;
            auto p = buildWavelet(seqs[2], ts, 2, nOct, nVoc, nChan, laneStart, ov,
                                  0x5000 + step, 0x6000 + step);
            d.demux(p.data(), p.size()); seqs[2]++;
        }
        ts++;
    }

    printf("wav_pkts=%llu\n",          (unsigned long long)d.wavPkts);
    printf("wav_seq_gaps=%llu\n",      (unsigned long long)d.wavSeqGaps);
    printf("wav_lost=%llu\n",          (unsigned long long)d.wavLost);
    printf("wav_n_octaves=%d\n",       d.nOct);
    printf("wav_n_voices=%d\n",        d.nVoc);
    printf("wav_last_lane_start=%u\n", (unsigned)d.lastLaneStart);
    printf("wav_last_n_chan=%u\n",     (unsigned)d.lastNChan);
    printf("wav_last_overrun=%d\n",    d.lastOverrun ? 1 : 0);
    printf("wav_last_ts=%llu\n",       (unsigned long long)d.lastTs);
    // octaves held, e.g. "[0, 1, 2]" to match Python list repr
    std::string oct = "[";
    bool first = true;
    for (const auto& kv : d.surface) {
        if (!first) oct += ", ";
        oct += std::to_string(kv.first); first = false;
    }
    oct += "]";
    printf("wav_octaves_held=%s\n", oct.c_str());
    printf("wav_surface_checksum=%llu\n", (unsigned long long)d.surfaceChecksum());
    return 0;
}
