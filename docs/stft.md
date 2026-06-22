# STFT spectrogram viewer

Firmware ≥ 1.3 (MicroZed) runs a Tier-2 STFT engine on the PL that
windows + transforms the LFP datapath and emits **one jumbo UDP datagram
per STFT pass on port 5003**. The plugin receives those, parks them in a
ring buffer, and the new `STFT` tab on the editor renders a scrolling
heat-map spectrogram for one selectable lane.

## Quickstart

1. Make sure your host NIC is **jumbo-capable** (MTU 9000). At the
   default `N = 64, K = 32` the datagram is ~8.5 KB, which won't
   fragment cleanly without jumbo support.
2. CONNECT the plugin to the board.
3. Click **LFP** — STFT taps the LFP datapath, so LFP must be enabled.
4. Click **STFT** — applies net.py defaults (N=64, hop=1, Hann window,
   identity channel selector) on first enable, then turns the engine
   on. Label flips to `STFT: ON` (blue).
5. Click the editor's **Tab** button (added by `VisualizerEditor`) to
   open the spectrogram canvas.
6. Pick a lane from the **Lane** dropdown (0..K-1). Auto color range
   adapts in a few seconds.

## Defaults (mirror remote/net.py configure_stft)

| Param | Value | Meaning |
|---|---|---|
| `nfft_log2` | 6 | N = 64 (33 bins, the Hermitian half) |
| `hop` | 1 | one STFT pass per LFP frame |
| `K` | engine build param | read from `get_status`, not host-settable |
| channels | identity | lane i → source channel i |
| window | Hann, signed Q15 | bit-identical port of `design_stft_window` |

At default LFP (`decim_R = 15`, Fs_lfp = 2 kHz), the 33 STFT bins span
**0..1000 Hz at 31.25 Hz spacing**; one image column = one LFP frame
(0.5 ms).

## Sanity tests (built-in playback, no electrodes needed)

The board has a synthetic-playback path that pushes a host-designed
waveform through the real LFP→STFT chain. From `remote/net.py`:

```
set_debug 1
start
playback chirp 20 800        # diagonal sweep, fades above ~600 Hz (LFP roll-off)
playback ripple              # 150-250 Hz blinks at known times
playback tone 200            # steady horizontal line at 200 Hz
```

Each is a free check that the frequency axis is right.

## Wire format (for reference)

Per UDP datagram, all 32-bit little-endian:

| word(s) | field                                                                        |
|---------|------------------------------------------------------------------------------|
| 0–1     | magic `0xCAFEBABE_5DEC7A00`                                                  |
| 2–3     | 64-bit timestamp ≈ LFP-frame index                                           |
| 4       | `[7:0]` nfft_log2 · `[15:8]` K (lanes) · `[31:24]` flags (bit0 = overflow)   |
| 5       | frame sequence number (mod 2³⁰) — for drop detection                         |
| 6       | `[15:0]` nbins (= N/2+1) · `[31:16]` hop                                     |
| 7       | reserved                                                                     |
| 8…      | payload: per lane (lane-major), nbins complex float32 as (re, im) pairs      |

Lane L, bin b: `re = samples[L * nbins * 2 + b * 2]`,
`im = samples[L * nbins * 2 + b * 2 + 1]`. Power = `re² + im²`; the
canvas plots `10·log10(power)` in dB.

The plugin always binds port 5003 on connect; without an enabled engine
the listener silently consumes nothing.

## Configurability (future)

The first cut hard-codes the net.py defaults so the button "just works".
Filter / window / channel-selector updates still go through
`remote/net.py configure_stft(...)` — same protocol, different values.
A reconnect is needed for the canvas to pick up a new `K` or `nbins`
(it adapts on the next frame).
