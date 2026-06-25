# DWT (wavelet) scalogram viewer

The MicroZed Tier-3 **wavelet** engine (repo `mz-tier3-wavelet`) runs a
constant-Q Morse wavelet bank on the LFP→3 kHz datapath and emits **one UDP
datagram per scalogram column on port 5004** (the rate-limited monitor stream).
This plugin binds 5004, decodes each column, parks it in a ring buffer, and the
**DWT** tab on the editor renders a scrolling time-frequency heat-map
(scalogram) for one selectable lane.

This viewer was adapted from the STFT spectrogram viewer
(`origin/claude/stft-spectrogram`, `Source/StftCanvas.*`) — it is the same kind
of thing (a scrolling time-frequency heat-map), retargeted at the wavelet
stream + packet format, with a **log** frequency axis driven by the wavelet
center-frequency table and **per-scale** color normalization (the 1/f roll-off
otherwise buries the high-frequency rows).

## Quickstart

1. CONNECT the plugin to the board. (The plugin binds UDP 5004 on connect; with
   no enabled engine the listener silently consumes nothing.)
2. Enable the wavelet engine out-of-band via `remote/net.py` (in the
   `mz-tier3-wavelet` repo): `configure_wavelet(sock, ...)` then
   `wavelet_enable(sock, True)`. (This plugin only *consumes* the monitor
   stream; the engine config + enable live in the wavelet firmware / net.py.)
3. Click the editor's window/tab button (added by `VisualizerEditor`, labeled
   **DWT**) to open the scalogram canvas.
4. Pick a lane from the **Lane** dropdown (0..K-1; K can be 32).
5. Choose color normalization: **Per-scale** (each frequency row self-
   normalizes — recommended, copes with 1/f) or **Global**.

## Axes

- **Y = scale / center frequency (log).** Labels come from the wavelet bank's
  center-frequency table (Morse bank, fs = 3000). Scale index
  `s = octave*n_voices + voice`; **bin 0 is the highest frequency** (top of the
  image), the last bin the lowest. At the default 8 octaves × 4 voices the bins
  span roughly **5 Hz … 1020 Hz**.
- **X = time** (scrolling; newest column on the right). One image column = one
  wavelet datagram = one decimated time step.
- **color = magnitude** = `sqrt(re² + im²)` from the complex int32 payload,
  log-compressed over a −40 dB floor.

## Wire format (UDP 5004) — for reference

The authoritative decoder is `remote/net.py::receive_wavelet` +
`design_wavelet_bank` in the `mz-tier3-wavelet` repo. All 32-bit little-endian:

| word(s) | field |
|---------|-------|
| 0–1 | magic `0xCAFEBABE_5CA70900` ("SCALOG") |
| 2–3 | 64-bit timestamp (word 2 == frame seq, word 3 = 0) |
| 4 | `[7:0]` n_octaves · `[15:8]` n_voices · `[23:16]` K · `[24]` overrun |
| 5 | frame sequence (mod 2³⁰) — for drop detection |
| 6 | `[15:0]` nscales (= n_octaves·n_voices) · `[31:16]` n_taps |
| 7 | gain word (per-octave 4-bit shifts, low octave first) |
| 8… | payload: per lane (lane-major), nscales complex **int32** as (re, im) |

One packet = one scalogram **column** (all K lanes × nscales complex bins at one
time step). Lane L, scale s: `re = samples[L*nscales*2 + s*2]`,
`im = samples[L*nscales*2 + s*2 + 1]`. The packet header is fully
self-describing, so the viewer adapts to whatever K / n_octaves / n_voices the
engine is built for — no reconnect needed when those change (the canvas picks
them up on the next column).

## Files

| File | Role |
|------|------|
| `Source/DwtCanvas.{h,cpp}` | the `Visualizer` subclass: scalogram render, lane + norm selectors, log freq axis |
| `Source/IntanInterface.{h,cpp}` | UDP 5004 listener thread + `WaveletFrame` decode + `setWaveletDataCallback` |
| `Source/IntanSocket.{h,cpp}` | `WaveletColumn` ring + `drainWaveletSince()` + `processWaveletFrame()` |
| `Source/IntanSocketEditor.{h,cpp}` | now a `VisualizerEditor`; `createNewCanvas()` returns the `DwtCanvas` |

`CMakeLists.txt` globs `Source/*.cpp` / `*.h`, so the new files need **no CMake
edit**.

## Building the plugin (must be done on a host with the OE GUI SDK)

This plugin cannot be built/linked in the headless dev container used to write
it — it needs the Open Ephys GUI SDK present and **built** (an `installed_libs/`
+ a Debug/Release `Build/` of plugin-GUI). On a normal OE plugin dev machine
(see the repo README), the usual flow is:

```bash
# GUI_BASE_DIR must point at a *built* plugin-GUI checkout
export GUI_BASE_DIR=/path/to/plugin-GUI
cd ephys-socket
mkdir -p build && cd build
cmake -G "Unix Makefiles" ..        # or the Xcode/VS generator per your platform
make -j                              # produces the plugin .so/.dll/.bundle
# copy the artifact into the GUI's plugins folder, or use the install target
```

On macOS the maintainer's reference build is a **Debug** build with the Xcode
generator (matching the rest of this repo's plugins). The new files were
syntax-checked (`g++ -fsyntax-only`) against the real OE GUI + JUCE headers;
the **link** step was not run here because the GUI SDK in the dev container was
not compiled.

## Headless validation (no GUI, no board)

`remote/dwt_plot.py` (in the `mz-tier3-wavelet` repo) is the standalone twin of
this viewer: it binds UDP 5004, decodes the exact same packet format, and
renders a scrolling scalogram PNG/window. It de-risks the decode the OE viewer
reuses. Test it with the bundled synthetic sender:

```bash
cd mz-tier3-wavelet/remote
python3 dwt_plot.py --selftest --png dwt_scalogram.png      # no socket; craft+decode+render
# or end-to-end over a real socket:
python3 dwt_plot.py --port 5004 --frames 400 --png live.png &
python3 dwt_test_sender.py --port 5004 --frames 400
```
