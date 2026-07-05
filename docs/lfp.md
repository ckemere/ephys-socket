# LFP / DSP engine — configure recipe

The plugin **receives** the LFP UDP stream (unified port 5000, demuxed by
`stream_type = 2`) and publishes a second `DataStream`. It ships **one set
of defaults** matching `remote/net.py`'s `configure_lfp("cic")` — the CIC
comp-FIR halfband for the shipped PL datapath. The editor's `LFP` button
applies them on first enable; CONNECT also force-reapplies them if the
firmware is holding an incompatible config from a previous session. Custom
filter kernels still go through `remote/net.py`.

## Quickstart — defaults (no external tool needed)

1. CONNECT the plugin to a board. If the firmware's LFP config is stale
   or incompatible with the shipped `USE_CIC=1` PL, the plugin reapplies
   the defaults automatically (you'll see
   `LFP: ... not CIC-compatible -- reapplying defaults at connect`).
2. Click **LFP**. If the engine has never been configured since boot,
   the plugin applies the defaults below and enables.
3. A second `DataStream` `IntanLFP` appears in the signal chain.

Default values (same as `net.py` `configure_lfp("cic")`):

| Param | Value | Meaning |
|---|---|---|
| `lane_mask` | mirrors broadband `channel_enable` | driven by the PL, not the host — filter exactly the broadband-enabled lanes |
| `decim_R` | `10` | 30 kHz / 10 = **3 kHz** output sample rate (CIC^4(/5) + halfband(/2) = /10, hardwired) |
| `num_taps` | `43` | halfband comp-FIR length (must be ≤ `HB_RING = 64`) |
| `cutoff_hz` | `1300` | CIC-droop-compensated LP, unity DC gain, Q1.17 |

The C++ designer (`IntanInterface::lfpDesignCicCompFir`) is a port of
`design_cic_comp_fir()` in net.py — same params → same coefficients.

## External configure — custom values

The reference tool is `remote/net.py` in the MicroZedIntanInterface repo.
It implements:

- `design_cic_comp_fir(num_taps, fc, ...)` — the droop-compensated comp-FIR
  for the CIC datapath, quantized to Q1.17.
- `configure_lfp(sock, datapath="cic", num_taps=None, cutoff_hz=1300.0)` —
  disables the engine, sets params, designs + uploads the filter.
- `lfp_enable(sock, on)` — starts / stops UDP emission on port 5000
  (unified with broadband, demuxed host-side by `stream_type = 2`).

From the MicroZed repo's `remote/` directory:

```python
from net import (tcp_connect, configure_lfp, lfp_enable)

sock = tcp_connect('192.168.18.10')
configure_lfp(sock, datapath="cic")     # decim_R=10, taps=43, CIC comp-FIR
lfp_enable(sock, True)
```

Then **disconnect and reconnect** the ephys-socket plugin so OE picks up
the new LFP stream (channel count + sample rate are captured at
`updateSettings` time, which only re-runs on reconnect). If your custom
config keeps `decim_R = 10` and `num_taps ≤ 64`, CONNECT will honor it as-
is; if not, CONNECT will overwrite it with the plugin defaults.

## What you actually need to set

| Parameter | Meaning | Notes |
|---|---|---|
| `num_taps` | halfband comp-FIR length | Must be ≤ `HB_RING = 64`; the 7-bit tap-count field wraps silently otherwise and scrambles the filter |
| `cutoff_hz` | passband edge (fc into `design_cic_comp_fir`) | Design is at the CIC output rate `fs_in / R_cic = 6 kHz`; keep fc well under 3 kHz |

`decim_R` is **hardwired** at /10 in the PL (`USE_CIC=1`) — it's advisory
metadata on the wire only. Any host-side value that doesn't match 10 is
mislabelled and the plugin will treat it as incompatible.

## Validating that data is flowing

- **From the plugin**: click STATUS — the device-status dump ends with an
  LFP block (`ENABLED  mask=0xFF (8 streams / 256 ch)  decim=10 (3000 Hz)
  taps=43`). The `Packets sent` counter should be advancing.
- **In OE**: a second `DataStream` (`IntanLFP`) appears in the signal
  chain after reconnect, with the configured channel count + 3 kHz sample
  rate.
- **Standalone**: `receive_lfp()` in `net.py` prints per-frame metadata
  and the first few sample values.
- **Loss check**: per-stream SEQ continuity — if you see
  `[LOSS] LFP SEQ gap` lines in the plugin's debug log, the config is
  bad (or the firmware wedged); the plugin now auto-reapplies defaults
  at CONNECT to prevent this, but if it still fires, something upstream
  is off.

## Sample format on the wire

Per-frame payload is **offset-binary 16-bit** (mid-scale `0x8000` = 0 µV),
identical to broadband. The plugin subtracts `0x8000` and applies the
neural `bitVolts = 0.195` to publish microvolts.

## Disabling

Click **LFP** again to toggle it off, or from `net.py`:

```python
lfp_enable(sock, False)
```

The plugin drops the second `DataStream` on the next reconnect.
