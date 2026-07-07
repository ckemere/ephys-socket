# Unified single-port packet consumer (branch `claude/unified-lfp`)

The plugin now consumes the **unified single-port packet format**: broadband
**and** LFP both arrive on **one UDP port** (default **5000**), demuxed host-side
by a `stream_type` field in a common header. This matches the MicroZed firmware
on branch `claude/unified-ports` byte-for-byte. The authoritative spec is
`mz-unified-ports/docs/unified-packet-format.md` (the *as-implemented* sections),
and the host reference is `mz-unified-ports/remote/net.py` (`UnifiedSink` +
`DataValidator` + `receive_lfp`).

## The wire format the plugin decodes

**Common header — 8 × 32-bit little-endian words (32 bytes), on every packet:**

| word | name | contents |
|------|------|----------|
| 0 | `MAGIC` | `0xCAFEBABE` |
| 1 | `TYPE_VER` | `stream_type[7:0]` · `version[15:8]` (=1) · `flags[31:16]` |
| 2 / 3 | timestamp | 64-bit master timestamp (lo, hi) |
| 4 | `SEQ` | per-stream sequence, **+1 per packet of that stream** (the loss check) |
| 5 | `AUX0` | stream-specific |
| 6 | `AUX1` | stream-specific |
| 7 | `RSVD` | 0 |

`stream_type`: **1 = BROADBAND, 2 = LFP** (3 = WAVELET, reserved/ignored here).
The host demuxes on `TYPE_VER & 0xFF`.

**BROADBAND (type 1)** — 14 header words = the 8 common + a 6-word sub-block,
then the data words (byte-identical to the legacy stream):
- `AUX0` (w5) = `channel_enable[7:0]` · `num_data_words[23:8]`
- `AUX1` (w6) = `digital_in[7:0]` · `aux_flags[15:8]` · `echo0[31:16]`
- w8 (sub-block) = prev-packet slot-1/2 aux echoes (low 16 = slot-1 accel echo)
- w9..12 = 8 external-ADC breadcrumbs (currently 0); w13 reserved
- w14.. = DATA

**LFP (type 2)** — exactly the 8 common header words, then the decimated
offset-binary int16 samples:
- `AUX0` (w5) = `lane_mask[7:0]` · `decim_R[15:8]` · `num_taps[23:16]` · `overrun[24]`
- `AUX1` (w6) = `num_samples` (= `popcount(lane_mask) * 32`)
- w8.. = `num_samples` offset-binary 16-bit samples (subtract `0x8000` for signed)

## What changed in the plugin

- **One UDP socket on port 5000, promiscuous drain.** `Source/IntanInterface.cpp`
  replaced the two per-stream listeners (broadband on 5000 + a separate **LFP
  socket on 5001**) with a single unified listener: `udpRecvThread()` binds 5000
  with a **16 MB `SO_RCVBUF`** and does the minimum on the hot path (`recvfrom`
  → ring), so broadband is **never** blocked while a slow consumer runs;
  `udpDemuxThread()` pops the ring, peeks `stream_type`, and routes broadband →
  data callback, LFP → LFP callback. Mirrors net.py's `UnifiedSink`
  recv→ring/demux split. A host-side ring overflow is counted (`ringDrops_`),
  never silently hidden.
- **Per-stream SEQ loss detection.** Both streams track header word 4
  independently. A broadband gap increments `seqGaps_`/`seqLostPackets_` (and the
  existing `timestampErrors_` reception counter, surfaced in the STATUS console
  line) **and prints a `[LOSS]` warning** — broadband is archival, gaps must be
  visible. LFP gaps increment `lfpSeqGaps_`/`lfpLostFrames_` and print their own
  `[LOSS]` line. (Replaces the old timestamp-delta heuristic as the authoritative
  loss signal, matching net.py's `DataValidator` SEQ check.)
- **Broadband header is 14 words, not 10.** `PACKET_HEADER_WORDS` 10 → 14;
  `calculatePacketSize` and the default packet size follow. `Source/IntanSocket.cpp`
  `updateBuffer()` skips 14 header words; the aux/TTL fields moved with the
  header: `digital_in` + `aux_flags` + `echo0` are now in `AUX1` (**word 6**, was
  word 4) and the slot-1 accelerometer echo is in the sub-block (**word 8** low
  16 bits, was word 5).
- **Cable detection offset stays at 8 words.** `scoreChannel` slices data from a
  fixed `CABLE_TEST_DATA_OFFSET_WORDS = 8` (NOT `PACKET_HEADER_WORDS`), exactly
  matching net.py `_score_channel`'s `data_words = packet[8:]` — the detection
  lock is calibrated to 8 with the +2 SPI-pipeline delay applied inside the
  scorer.
- **UI / docs:** the LFP button tooltip, `LfpFrame`/`setLfpEnabled` doc comments,
  and status comments no longer say "port 5001" — LFP is on the unified port,
  `stream_type=2`. The editable UDP-port box already defaulted to 5000.

## Validation

A standalone parser harness lives in `test/`. It re-implements the plugin's
exact decode logic (no JUCE / no Open Ephys), feeds synthetic broadband + LFP
packets built to the spec (including deliberately dropped packets to exercise
gap detection), and diffs its summary against `ref_decode.py`, which decodes the
**same** bytes the net.py way.

```bash
cd test && ./run_test.sh      # builds with g++, runs both, diffs -> PASS/FAIL
```

It checks: demux by `stream_type`, broadband + LFP SEQ-gap counts, the moved
header field offsets (digital_in @ AUX1/w6, slot-1 echo @ sub-block/w8), LFP
`AUX0`/`AUX1` decode, and offset-binary sample conversion. All match net.py
byte-for-byte.

The three changed source files also pass a header-checked syntax compile
against the real JUCE/Open Ephys headers (the same way prior `ephys-socket`
branches were validated — header-checked, not GUI-built, since this environment
has no board and no `installed_libs` commonlib):

```bash
G=../plugin-GUI    # adjust to your plugin-GUI checkout
g++ -std=c++17 -fsyntax-only -Wall -Wextra Source/IntanInterface.cpp
g++ -std=c++17 -fsyntax-only -DOEPLUGIN -DJUCE_DISABLE_NATIVE_FILECHOOSERS=1 \
    -I$G/JuceLibraryCode -I$G/JuceLibraryCode/modules -I$G/Plugins/Headers \
    Source/IntanSocket.cpp
g++ -std=c++17 -fsyntax-only -DOEPLUGIN -DJUCE_DISABLE_NATIVE_FILECHOOSERS=1 \
    -I$G/JuceLibraryCode -I$G/JuceLibraryCode/modules -I$G/Plugins/Headers \
    Source/IntanSocketEditor.cpp
```

### Remaining build steps for the user (full GUI build)

A complete plugin build needs the Open Ephys GUI built first (so `installed_libs`
and the GUI `Build/` libraries exist) — see `docs/building.md`. Once the GUI is
built next to this repo:

```bash
cd Build
cmake ..                       # add -DGUI_BASE_DIR=/path/to/plugin-GUI if not the sibling
cmake --build . --config Debug # or Release; match the GUI build type
cmake --build . --config Debug --target install
```

Then launch the GUI, drop in the **Intan Socket** source, set the device IP,
CONNECT, and (optionally) enable the firmware LFP engine via `remote/net.py`
`configure_lfp(...)` + `lfp_on`, then reconnect to publish the second
(LFP) DataStream. Broadband and LFP both arrive on UDP 5000 now.

## Mismatch risk vs the firmware

Low, and the parser test pins the byte layout. The remaining assumptions, all
matching `claude/unified-ports`:
- One packet per datagram (the demux re-chunks defensively if several coalesce,
  as net.py does).
- The broadband sub-block is exactly 6 words (header total 14). If the firmware
  ever changes the sub-block size, `PACKET_HEADER_WORDS` and the
  `IntanSocket.cpp` word-6/word-8 field reads must move together.
- Cable detection keeps net.py's empirical offset-8 lock; it is exercised only
  with a physical chip attached.
