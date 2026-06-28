# Wavelet (DWT) scalogram consumer — UNIFIED octave-split stream

This branch (`claude/unified-wavelet`, off `claude/unified-lfp`) adds the
**WAVELET scalogram consumer** (`stream_type = 3`) on top of the existing unified
single-port broadband + LFP demux, **without touching the broadband/LFP data
path**. The wavelet engine streams on the SAME UDP port (5000); the host demuxes
by `stream_type`.

## The wire format (authoritative — matches the firmware byte-for-byte)

Source of truth: `mz-unified-wavelet/docs/unified-packet-format.md` (WAVELET
section) and the real host decoder `mz-unified-wavelet/remote/net.py`
(`UnifiedSink._handle_wavelet` + `receive_wavelet`).

**ONE datagram = ONE octave**, emitted **rate-aligned**: octave `o` updates at
`3 kHz / 2^o` (octave 0 at 3 kHz, octave 7 at ~23 Hz). The host **reassembles**
the full scalogram surface by **holding each octave's last value between its
updates** — a multirate surface. Common 8-word LE header + payload:

| word | field | contents |
|------|-------|----------|
| w0 | MAGIC | `0xCAFEBABE` |
| w1 | TYPE_VER | `stream_type=3` \| `version<<8` \| `flags<<16` |
| w2/w3 | TS | 64-bit master timestamp |
| w4 | SEQ | **per-OCTAVE** sequence (+1 per packet of THIS octave) — the loss check |
| w5 | AUX0 | `octave[3:0]` \| `n_octaves[7:4]` \| `n_voices[11:8]` \| `overrun[24]` |
| w6 | AUX1 | `n_channels[7:0]` \| `lane_start[23:8]` |
| w7 | RSVD | 0 |
| w8.. | payload | `n_channels × n_voices` complex bins (re,im int32 each), **lane-major then voice-minor**, FOR THIS OCTAVE ONLY |

Bin (local channel `c`, voice `v`): `re = bins[(c*n_voices+v)*2+0]`,
`im = …+1`. The absolute scalogram lane is `lane_start + c`; the absolute scale
row is `octave*n_voices + v` (bin 0 = highest frequency). One octave packet fits
one datagram by construction (`n_channels·n_voices ≤ 180`; default K=40, V=4).

## What was added (files changed)

- **`Source/IntanInterface.{h,cpp}`** — the demux. `demuxDatagram()` gains a
  `STREAM_TYPE_WAVELET` branch → `processWaveletDatagram()`, which decodes the
  header exactly the way net.py does, verifies **per-octave SEQ continuity**
  (each octave streams at its own rate, so each carries its OWN SEQ; a gap is
  logged as `[IntanInterface][LOSS] Wavelet octN SEQ gap …`, mirroring the
  broadband/LFP `[LOSS]` logging — loss is never hidden), and dispatches one
  `WaveletPacket` (octave + metadata + bins pointer) via a new
  `WaveletDataCallback`. Per-octave last-seq is tracked in `wavLastSeq_[octave]`,
  exactly like net.py's `UnifiedSink._wav_last_seq[octave]`.
- **`Source/IntanSocket.{h,cpp}`** — the reassembly. `processWaveletPacket()`
  converts each octave's complex bins to magnitude and stores them in a HELD
  per-octave surface (`waveletHeld_[octave]`); on each **octave-0** (fastest)
  update it flattens the whole held surface into one full scalogram column
  (`K × nscales`, `nscales = n_octaves·n_voices`) and parks it in a ring for the
  canvas. Slow octaves are held between updates — the truthful multirate
  representation (matches net.py `receive_wavelet`). The canvas reads it via
  `drainWaveletSince()` / `isWaveletRunning()`. **This path never touches the
  broadband or LFP `sourceBuffers`, so it cannot regress those streams.**
- **`Source/DwtCanvas.{h,cpp}`** — the heatmap visualizer (x=time, y=scale/freq,
  color=magnitude, viridis, per-scale or global log normalization). Adapted from
  the prior `claude/dwt-visualizer` canvas (which consumed the OLD full-surface
  format) to the NEW octave-split/rate-aligned reassembly — the reassembly moved
  into `IntanSocket`, so the canvas just renders the full columns it drains.
  Frequency axis comes from the Morse bank center-freq table (`fs=3000`),
  matching net.py `design_wavelet_bank()`.
- **`Source/IntanSocketEditor.{h,cpp}`** — converted from `GenericEditor` to
  `VisualizerEditor` so the scalogram canvas is hosted on the editor's tab/window
  button (`createNewCanvas()` returns a `DwtCanvas`). All existing broadband/LFP
  buttons (CONNECT/RESCAN/debug/LFP-enable/aux) are untouched.
- **`test/`** — a standalone parser test (below).

## Validation — standalone parser test (no board, no GUI)

`test/wavelet_parse_test.cpp` re-implements the EXACT plugin decode + multirate
surface reassembly and asserts it against synthetic per-octave packets:
**3 octaves at different rates** (octave 0 every step, octave 1 every 2, octave 2
every 4 — the rate-aligned cadence), with a **deliberate dropped packet** on
octave 0 AND on octave 1. `test/ref_decode_wavelet.py` decodes the SAME bytes the
**net.py** way (`UnifiedSink._handle_wavelet` + `receive_wavelet`).

```bash
cd test && ./run_wavelet_test.sh
```

It diffs the two summaries and confirms they match byte-for-byte:
per-octave SEQ-gap count (= 2, the two deliberate drops), packets received (70),
octaves held `[0, 1, 2]`, geometry (`n_octaves`/`n_voices`/`lane_start`/
`n_channels`/`overrun`/`ts`), and an **order-sensitive checksum of the held
surface** (proves lane-major/voice-minor ordering AND the hold-between-updates
semantics are identical). The broadband+LFP regression test (`./run_test.sh`)
still passes too.

## Syntax check (header-checked against real JUCE/OE headers)

All four changed source files pass a header-checked syntax compile against the
real JUCE/Open Ephys headers (same validation level as prior `ephys-socket`
branches — header-checked, not GUI-built, since this environment has no board,
no `cmake`, and no `installed_libs` commonlib):

```bash
G=../plugin-GUI    # adjust to your plugin-GUI checkout
g++ -std=c++17 -fsyntax-only -Wall -Wextra Source/IntanInterface.cpp
for f in Source/IntanSocket.cpp Source/DwtCanvas.cpp Source/IntanSocketEditor.cpp; do
  g++ -std=c++17 -fsyntax-only -DOEPLUGIN -DJUCE_DISABLE_NATIVE_FILECHOOSERS=1 \
      -I$G/JuceLibraryCode -I$G/JuceLibraryCode/modules -I$G/Plugins/Headers "$f"
done
```

## Remaining build steps for the user (full GUI build)

A complete plugin build needs the Open Ephys GUI built first (so `installed_libs`
and the GUI `Build/` libraries exist) — see `docs/building.md`. Once the GUI is
built next to this repo:

```bash
cd Build
cmake ..                       # add -DGUI_BASE_DIR=/path/to/plugin-GUI if not the sibling
cmake --build . --config Debug # or Release; match the GUI build type
cmake --build . --config Debug --target install
```

Then launch the GUI, drop in the **Intan Socket** source, set the device IP, and
CONNECT. To see the scalogram:

1. Configure + enable the firmware wavelet engine out of band via
   `remote/net.py` (`configure_wavelet(sock, channels=…, V=4, n_octaves=8)` then
   `wavelet_enable(sock, True)`). The board then streams `stream_type=3` octave
   packets on UDP 5000.
2. Click the editor's tab/window button to open the **DWT** scalogram canvas.
   Pick a lane; the heatmap scrolls (newest at the right), log-frequency axis,
   per-scale normalization by default.

Broadband and LFP continue to work exactly as before — the wavelet consumer is
additive and isolated.

## Mismatch risk vs the firmware

- **Header decode is byte-for-byte net.py** (asserted by the parser test): same
  word offsets, same AUX0/AUX1 bitfields, same lane-major/voice-minor payload
  order, same per-octave SEQ semantics.
- **Endianness:** the plugin reads the payload via `reinterpret_cast<const
  int32_t*>` (the header is a whole number of 32-bit words, so it's aligned). The
  board and host are both little-endian; the parser test cross-checks this
  against `struct.unpack('<i')` in the reference. On a (hypothetical) big-endian
  host this would need byte-swapping — out of scope for the supported platforms.
- **Geometry bounds:** `octave` is a 4-bit field (0..15); the plugin sizes its
  per-octave arrays to 16 and ignores anything out of range. `n_voices` /
  `n_channels` are taken from the packet each frame; a reconfigure (changed K /
  V / n_octaves) resets the held surface so stale geometry is never mixed.
- **Center-freq axis** (`DwtCanvas`) mirrors net.py `design_wavelet_bank()`
  (`fs=3000`, `fc_top=0.34`). If the firmware's bank constants change, update
  `kWavFs`/`kWavFcTop` in `DwtCanvas.cpp` to match — this is display-only and
  does not affect the data or loss accounting.
