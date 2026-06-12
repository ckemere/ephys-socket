#!/usr/bin/env python3
"""
Analyze /tmp/intan_raw.bin written by ephys-socket during the first 500 packets.

File format
-----------
Bytes 0-15  : file header  [magic=0x494E5441, version=1, words_per_packet, channel_enable]
Bytes 16+   : packets      [words_per_packet x uint32_le per packet]

Packet layout (words_per_packet words, little-endian uint32)
  [0]  magic low   0xDEADBEEF
  [1]  magic high  0xCAFEBABE
  [2]  timestamp low  (64-bit TS = [3]<<32 | [2])
  [3]  timestamp high
  [4]  digital_in (bits 7:0)
  [5]  reserved
  [6]  reserved (future analog 0-3)
  [7]  reserved
  [8]  reserved (future analog 4-7)
  [9]  reserved
  [10..] data words

Data word layout (with all 4 streams enabled, channel_enable=0x0F):
  word[10 + 2*c]     = CIPO0 cycle c : bits[15:0]=regular, bits[31:16]=DDR
  word[10 + 2*c + 1] = CIPO1 cycle c : bits[15:0]=regular, bits[31:16]=DDR
  c = 0..34  (35 acquisition cycles per packet)

SPI pipeline delay: result of CONVERT ch_k arrives in cycle (k+2) % 35.
Aux ins arrive at cycles 34, 0, 1.
"""

import sys
import struct
import numpy as np
import matplotlib.pyplot as plt

HEADER_WORDS = 10
FILE_PATH = "/tmp/intan_raw.bin"

# ── load file ─────────────────────────────────────────────────────────────────
with open(FILE_PATH, "rb") as f:
    raw = f.read()

magic, version, words_per_packet, channel_enable = struct.unpack_from("<IIII", raw, 0)
assert magic == 0x494E5441, f"Bad magic 0x{magic:08X}"
print(f"File header: version={version}  words_per_packet={words_per_packet}"
      f"  channel_enable=0x{channel_enable:X}")

payload = raw[16:]
bytes_per_packet = words_per_packet * 4
n_packets = len(payload) // bytes_per_packet
print(f"Packets found: {n_packets}")

data = np.frombuffer(payload[:n_packets * bytes_per_packet],
                     dtype="<u4").reshape(n_packets, words_per_packet)

# ── verify magic ──────────────────────────────────────────────────────────────
magic_ok = (data[:, 0] == 0xDEADBEEF) & (data[:, 1] == 0xCAFEBABE)
print(f"Magic OK: {magic_ok.sum()} / {n_packets} packets")

# ── timestamps ────────────────────────────────────────────────────────────────
ts = data[:, 2].astype(np.uint64) | (data[:, 3].astype(np.uint64) << 32)
ts_gaps = np.diff(ts.astype(np.int64))
print(f"Timestamp gaps  min={ts_gaps.min()}  max={ts_gaps.max()}"
      f"  (expected all=1)")

# ── extract raw data words ────────────────────────────────────────────────────
dwords = data[:, HEADER_WORDS:]          # shape (n_packets, n_data_words)
n_data_words = dwords.shape[1]
print(f"Data words per packet: {n_data_words}  (expected {words_per_packet - HEADER_WORDS})")

# Split each 32-bit word into two uint16 halves (lo = even flat index, hi = odd)
lo = (dwords & 0xFFFF).astype(np.uint16)
hi = (dwords >> 16).astype(np.uint16)

# Signed offset-binary → two's complement
lo_s = (lo.astype(np.int32) - 32768).astype(np.int16)
hi_s = (hi.astype(np.int32) - 32768).astype(np.int16)

# ── stream layout (needed by all figures) ─────────────────────────────────────
n_streams = bin(channel_enable & 0xF).count('1')
stream_names = [["CIPO0_reg", "CIPO0_DDR", "CIPO1_reg", "CIPO1_DDR"][b]
                for b in range(4) if channel_enable & (1 << b)]
n_data_samples = 35 * n_streams
print(f"nStreams={n_streams}  streams={stream_names}")

# ── figure 1: first 8 raw uint16 samples vs packet index ─────────────────────
# The first 4 data words = 8 samples covering streams/cycles 0
# word[0] lo = CIPO0_reg cycle 0,  word[0] hi = CIPO0_DDR cycle 0
# word[1] lo = CIPO1_reg cycle 0,  word[1] hi = CIPO1_DDR cycle 0
# ...
# Build flat-index labels matching actual stream layout
# flat index f → word f//2, lo if even, hi if odd
# With nStreams streams: flat f = cycle*(nStreams) + stream_slot
_flat_labels = []
for f in range(8):
    cycle_f = f // n_streams
    slot_f  = f %  n_streams
    sname   = stream_names[slot_f] if slot_f < len(stream_names) else "?"
    half    = "lo" if f % 2 == 0 else "hi"
    _flat_labels.append((f"word[{f//2}] {half}", f"{sname}  cycle {cycle_f}"))

fig, axes = plt.subplots(4, 2, figsize=(14, 10), sharex=True)
fig.suptitle(
    f"First 8 raw 16-bit samples (first 4 data words) — all packets\n"
    f"channel_enable=0x{channel_enable:X}  nStreams={n_streams}",
    fontsize=11)

labels = _flat_labels

pkt = np.arange(n_packets)
for idx, (short_lbl, long_lbl) in enumerate(labels):
    ax = axes[idx // 2, idx % 2]
    wi = idx // 2
    is_hi = (idx % 2) == 1
    raw_vals = hi[:, wi] if is_hi else lo[:, wi]
    signed_vals = hi_s[:, wi] if is_hi else lo_s[:, wi]
    ax.plot(pkt, raw_vals, label="uint16", lw=0.8, alpha=0.7)
    ax.plot(pkt, signed_vals.astype(np.int32) + 32768, label="int16+32768", lw=0.8,
            alpha=0.7, linestyle="--")
    ax.set_title(f"{short_lbl}  ({long_lbl})", fontsize=9)
    ax.set_ylabel("ADC count")
    ax.legend(fontsize=7, loc="upper right")
axes[-1, 0].set_xlabel("Packet index")
axes[-1, 1].set_xlabel("Packet index")
plt.tight_layout()

# ── figure 2: all 35 cycles for CIPO0 regular, first packet ──────────────────
# Helps see the pipeline delay and channel ordering
fig2, ax2 = plt.subplots(figsize=(12, 4))
fig2.suptitle("CIPO0 regular — all 35 cycles, packet 0  (uint16)", fontsize=11)
# CIPO0 regular = lo of even data words [0,2,4,...,68]
cipo0_reg_p0 = lo[0, 0::2]   # every other word starting at 0, lower half
ax2.stem(np.arange(len(cipo0_reg_p0)), cipo0_reg_p0.astype(int),
         markerfmt="C0o", linefmt="C0-", basefmt="k-")
ax2.axhline(32768, color="r", linestyle="--", lw=0.8, label="0x8000 (0V)")
ax2.set_xlabel("Cycle index c  (channel k → cycle k+2)")
ax2.set_ylabel("uint16 ADC value")
ax2.legend()
ax2.set_xticks(np.arange(35))
plt.tight_layout()

# ── figure 3: de-interleaved channels 0..7 over all packets ──────────────────
# Mirrors updateBuffer() exactly:
#   flat index = cycle * nStreams + stream_slot
#   wordIdx    = flat // 2
#   lo half if flat is even, hi half if flat is odd
def sample_at(dwords_row, flat_idx, n_data_words, n_data_samples):
    if flat_idx < 0 or flat_idx >= n_data_samples:
        return 0
    word_idx = flat_idx // 2
    if word_idx >= n_data_words:
        return 0
    w = int(dwords_row[word_idx])
    return (w & 0xFFFF) if (flat_idx % 2 == 0) else ((w >> 16) & 0xFFFF)

n_data_samples = 35 * n_streams
n_data_words_actual = dwords.shape[1]

fig3, axes3 = plt.subplots(16, 1, figsize=(14, 24), sharex=True)
fig3.suptitle(
    f"Channels 0–15 after de-interleave + pipeline correction\n"
    f"channel_enable=0x{channel_enable:X}  streams={stream_names}",
    fontsize=10)
for k in range(16):
    cycle = (k + 2) % 35
    flat  = cycle * n_streams + 0   # stream slot 0 = first enabled stream
    word_idx = flat // 2
    if word_idx >= n_data_words_actual:
        axes3[k].text(0.5, 0.5, "out of range", transform=axes3[k].transAxes, ha='center')
        continue
    raw = np.array([sample_at(dwords[p], flat, n_data_words_actual, n_data_samples)
                    for p in range(n_packets)], dtype=np.uint16)
    vals = raw.astype(np.int32) - 32768
    axes3[k].plot(pkt, vals, lw=0.6)
    axes3[k].set_ylabel(f"CH{k+1} (i16)", fontsize=7)
    axes3[k].axhline(0, color="r", lw=0.5, linestyle="--")
axes3[-1].set_xlabel("Packet index  (= time sample)")
plt.tight_layout()

plt.show()
