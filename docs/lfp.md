# LFP / DSP engine — external configure recipe

The plugin **receives** the LFP UDP stream and publishes a second
`DataStream`, but it does **not** design or upload filter coefficients.
That stays in an external tool so the filter library can evolve
independently of the OE plugin (and so a tighter loop is possible while
tuning).

The reference tool is `remote/net.py` in the MicroZedIntanInterface
repository. It already implements:

- `design_lfp_lowpass(num_taps, cutoff_hz, fs=30000)` — windowed-sinc
  (Hamming) low-pass FIR, unity DC gain, quantized to Q1.17 (18-bit signed).
- `configure_lfp(sock, lane_mask, decim_R, num_taps, cutoff_hz)` — disables
  the engine, sets channels + params, designs + uploads the filter.
- `lfp_enable(sock, on)` — starts / stops UDP emission on port 5001.

## Quickstart

From the MicroZed repo's `remote/` directory:

```python
from net import (tcp_connect, configure_lfp, lfp_enable)

sock = tcp_connect('192.168.18.10')          # board IP
configure_lfp(
    sock,
    lane_mask=0x0F,    # which broadband streams to LFP-filter
    decim_R=15,        # 30000 / 15 = 2000 Hz output
    num_taps=128,      # FIR length
    cutoff_hz=600.0    # passband cutoff
)
lfp_enable(sock, True)
```

Then **disconnect and reconnect** the ephys-socket plugin so OE picks up
the new LFP stream (the channel count + sample rate are fixed at
`updateSettings` time, which only re-runs on reconnect).

## What you actually need to set

| Parameter | Meaning | Notes |
|---|---|---|
| `lane_mask` | 8-bit, same convention as broadband (bits 0..3 port A, 4..7 port B) | Bit set ⇒ that lane's 32 amplifier channels get LFP-filtered |
| `decim_R` | broadband_rate / output_rate | Powers of 2 keep the FIR aliasing math clean; `decim_R=15` → 2 kHz is the lab default |
| `num_taps` | active FIR length | Bounded by the firmware's FIR engine (see protocol.md); 128 is plenty for the 2 kHz LP |
| `cutoff_hz` | passband edge of the LP design | Must be `< fs / (2 × decim_R)` (Nyquist of the output rate) |

## Validating that data is flowing

- **From the plugin**: click STATUS — the device-status dump now ends with
  an LFP block (`ENABLED  mask=0x0F (4 streams / 128 ch)  decim=15
  (2000 Hz)  taps=128`). The `Packets sent` counter should be advancing
  if streaming is live.
- **In OE**: a second `DataStream` ("IntanLFP") appears in the signal
  chain after reconnect, with the configured channel count + sample rate.
- **Standalone**: `receive_lfp()` in `net.py` is a reference receiver
  that prints per-frame metadata and the first few sample values.

## Sample format on the wire

Per-frame payload is **offset-binary 16-bit** (mid-scale `0x8000` = 0 µV),
identical to broadband. The plugin subtracts `0x8000` and applies the
neural `bitVolts = 0.195` to publish microvolts, matching the broadband
neural stream's scaling.

## Disabling

```python
lfp_enable(sock, False)
```

The plugin will still publish the second stream on the next reconnect
(it's bound to `lfp_enabled` at connect time). Disconnect, run
`lfp_enable(False)`, reconnect — and the second `DataStream` is gone.
