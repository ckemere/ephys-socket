# LFP 3 kHz update (R=15 → R=10)

Branch: `lfp-3khz`. Tracks the firmware change moving LFP decimation from
**R=15 (2 kHz)** to **R=10 (3 kHz)**. Broadband (30 kHz) is untouched.

## TL;DR

The published LFP `DataStream` rate is **already derived from the packet**, so
the plugin auto-tracks whatever rate the firmware runs. The only thing that was
hardcoded to R=15 was the *default configuration* the plugin pushes to firmware
when the LFP engine has never been configured since boot. That default is now
R=10. No timestamp/alignment offset change was needed.

## How the rate is determined (verified, not changed)

The LFP UDP packet is self-describing. Header word 4 packs the config:
`[7:0] lane_mask | [15:8] decim_R | [23:16] num_taps | [24] overrun`
(see `Source/IntanInterface.cpp` `processLfpDatagram`, ~line 1043/1120).

That `decim_R` flows through unchanged at every layer:

- `DeviceStatus.lfpDecimR` → `IntanSocket::lfp_decim_R`
  (`IntanSocket.cpp` connect path ~line 235, and `setLfpEnabled` ~line 1398).
- Published DataStream rate: `lfpSampleRate = SAMPLE_RATE / (float)lfp_decim_R`
  (`IntanSocket.cpp` ~line 471). With R=10 firmware this is exactly 3000 Hz —
  **no edit required for the runtime rate.**
- Timestamps: each LFP frame carries `timestamp = frame_seq * decim_R` in
  **broadband (30 kHz) ticks** (`IntanSocket.cpp::processLfpFrame` ~line 681,
  `IntanInterface.h` `LfpFrame::timestamp`). Because the alignment is expressed
  in broadband ticks and driven by the packet's own R, it tracks any R
  automatically — there is **no separate FIR group-delay constant in the
  plugin** to update. The FIR group delay lives in firmware (where the filter
  runs); the plugin just consumes the timestamps the firmware emits.

=> The rate is **derived-from-packet**, not hardcoded. There is no `3000.0f`
literal anywhere in the rate path.

## What actually changed

1. `Source/IntanInterface.h` — `LfpDefaults::DECIM_R`: `15` → `10`.
   Added a comment block explaining that (a) this is only the default the
   plugin *configures* (the received rate is always derived from the packet),
   and (b) the Nyquist constraint `CUTOFF_HZ < FS/(2*DECIM_R)`: at R=10 the
   output Nyquist is 1500 Hz, so the existing 600 Hz windowed-sinc FIR is still
   valid and the coefficient set is unchanged.

2. `Source/IntanSocket.h` — doc comments on `setLfpEnabled` /
   `configureLfpDefaults` updated from "decim 15 (2 kHz)" / "decim_R=15" to
   "decim 10 (3 kHz)" / "decim_R=10".

3. `docs/lfp.md` — default table, the `configure_lfp(...)` example, the
   parameter notes, and the STATUS-dump example updated to R=10 / 3 kHz. Added
   a note clarifying that the plugin reads `decim_R` per-packet and does not
   hardcode the 3 kHz figure into the stream.

4. `docs/notes.md` — `configure_lfp(... decim_R=15 ...)` → `decim_R=10`.

No changes to the broadband path. No changes to the FIR designer
(`lfpDesignLowpass`) — it takes `fs=30000` and `cutoff=600` independent of R,
and 600 Hz remains below the 1500 Hz output Nyquist at R=10.

## Build / validation status

- **Compile sanity:** verified the new `LfpDefaults` math with a standalone
  `g++ -std=c++17` static-assert check: R=10 ⇒ 3000 Hz, and 600 Hz < 1500 Hz
  Nyquist. PASS.
- **Full plugin build NOT run.** This Linux box has no `cmake` and the
  Open Ephys GUI's linkable `open-ephys` library / `installed_libs/include`
  are not built (only stale CMake config under `../plugin-GUI/Build`).
  A real build needs the Open Ephys GUI built with `GUI_BASE_DIR` pointing at
  it. That, plus on-board validation, is **pending** and is for the user to do.

## What the user should check / do

- **Firmware first.** The plugin auto-follows R, but the firmware must actually
  decimate at R=10. If you instead reconfigure LFP out-of-band via
  `remote/net.py configure_lfp(..., decim_R=10, ...)`, then **disconnect and
  reconnect** the plugin so `updateSettings` re-reads the rate.
- **First-enable default.** If the engine was never configured since boot and
  you click the editor's LFP button, the plugin now configures R=10 (3 kHz).
- **Confirm rate in OE:** the second DataStream ("IntanLFP") should appear at
  3000 Hz; the STATUS dump should read `decim=10 (3000 Hz)`.
- **Build the plugin** against your Open Ephys GUI checkout and confirm it
  compiles + loads, then validate against a connected board (timestamps should
  still line up with broadband, since alignment is packet-driven).
