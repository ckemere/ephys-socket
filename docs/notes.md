# Notes

Implementation details for the MicroZed Intan Socket plugin: editor
controls, the aux sequencer / accelerometer parse mode, and how acquired
samples are scaled and recorded.

## Editor controls

| Control | When | What |
|---|---|---|
| CONNECT / DISCONNECT | idle | open/close the TCP+UDP connection |
| RESCAN | idle | chip auto-detection (phase sweep, chip ID, channel mask) |
| DBG 1P / DBG 2P | idle | device-generated sine on port A only (134 ch) or both ports (268 ch) |
| **STATUS** | **any time** | print the full device status to the GUI console, including the aux-sequencer block (enabled/fast-settle/digout/DSP flags, active banks, per-slot indices, last injected-command result) |
| **SETTLE** | **any time** | toggle amplifier fast settle (RHD Reg-0 D5). The PL injects `WRITE(0,0xFE)` / `WRITE(0,0xDE)` into the slot-0 aux position on the transition packet. Datasheet pulse guidance is ~2.5/f_H (≈250 µs at 7.5 kHz upper cutoff) — toggle off promptly |
| **AUX SEQ** | **any time** | switch the 3 aux COPI positions to the banked sequencer programs (see below). Toggling while acquiring uploads the **standby** banks and swaps them atomically at a packet boundary — exercising the live double-buffer path |
| TTL Settle | any time | fast settle follows the selected `digital_in` pin (TTL1–8 → pins 0–7); "-" disables. Combined (OR) with the SETTLE button's software level |

STATUS, SETTLE, AUX SEQ, and TTL Settle are deliberately usable **during
acquisition** — they exist to validate the firmware's runtime-control paths.
SETTLE and TTL Settle auto-enable the aux sequencer if it is off (the override
layer only reaches the chip while the sequencer is enabled).

On **CONNECT**, the editor pulls authoritative state from the device's
`get_status` response — chip indicators, debug-mode label, aux-sequencer
state. Reconnecting after a successful RESCAN restores the prior indicators
and channel count without re-running detection. A fresh-boot firmware
(channel\_enable = 0) is seeded with `0x0F` so the signal chain isn't
zero-channel; chip indicators stay dark to prompt the user to RESCAN.

## Aux sequencer mode and accelerometer de-interleaving

Firmware `aux-seq-v2` can source the three aux COPI cycles (32–34) from
programmable, double-buffered command banks. AUX SEQ loads the default
programs (mirroring `remote/net.py`):

- **slot 0** (cycle 32, real-time): a `WRITE(3, …)` carrier, rewritten every
  packet by the PL's Reg-3 shadow (digout mirror); also the fast-settle
  injection point
- **slot 1** (cycle 33, ADC): accelerometer sweep `CONVERT(32) → (33) → (34)`,
  looping — **one axis per packet**, i.e. 10 kHz per axis
- **slot 2** (cycle 34, housekeeping): supply voltage, temperature, chip ID,
  and the `INTAN` ROM string, looping

Packets are **self-describing**: header word 4 carries
`{echo_slot0[15:0], aux_flags[7:0], digital_in[7:0]}` and word 5 carries
`{echo_slot2[31:16], echo_slot1[15:0]}` — the *originating command* for each
aux result in the packet. Because of the chip's 2-command SPI pipeline, the
result of slot 0's command is at data word 34 of the *same* packet, and the
results of the *previous* packet's slot-1/2 commands are at data words 0 and 1.

`IntanSocket::updateBuffer()` uses the per-packet flags (not local state) to
pick the parse mode, so it stays correct through live bank swaps:

- sequencer **off**: legacy format — all three aux inputs converted every
  packet (results at cycles 34/0/1), passed straight to the AUX channels
- sequencer **on**: data word 0 holds one accelerometer axis per packet,
  identified by the slot-1 echo; the plugin de-interleaves by echo into the
  3 AUX channels with sample-and-hold (each axis updates at 10 kHz, buffered
  at the 30 kHz stream rate). The `echo_valid` flag gates the first packet
  after a start (its word-0/1 results belong to the previous run).

The AUX channel count and packet size are identical in both modes, so no
signal-chain rebuild is needed when toggling.

Status responses are 122 bytes on firmware ≥ 1.1.0.0; the plugin also accepts
the 98-byte aux-seq-v2 form and the 86-byte pre-aux form (aux features then
report "not supported"). The buffer is sized to the largest known form so a
firmware response never sits unread in the TCP queue.

The authoritative protocol documentation lives in the MicroZedIntanInterface
repo: `docs/command-bank-design.md` (design), `docs/NIGHT_LOG-2026-06-11.md`
(implementation + verification), and `firmware/include/main.h` (register map).

## Channel scaling and data storage

Scaling matches the OpenEphys **acquisition-board plugin** exactly. Every
sample is published into the GUI data buffer as:

```
buffer_value = (raw_adc_count - 32768) * bitVolts
```

and the **same `bitVolts`** is declared on the channel:

| Channel type | bitVolts | buffer units | units label |
|---|---|---|---|
| ELECTRODE (neural) | `0.195` | µV | `uV` |
| AUX (auxin1/2/3, e.g. accelerometer) | `0.0000374` | — | `mV` |

The `- 32768` decodes the chip's offset-binary samples to a signed value
around zero; it is a constant, reversible representation choice (the same one
the acquisition board makes), **not** a baseline subtraction or detrend — no
acquired information is altered or lost.

### What this means for recordings

The Binary record engine converts each float back to `int16` as
`int16 = buffer_value / bitVolts` (clamped to ±32767). Because the buffer value
is `(raw - 32768) * bitVolts` and the channel's `bitVolts` is that same factor,
the division cancels exactly:

```
int16_on_disk = (raw - 32768)
```

So the recording stores the **exact signed ADC count** (`raw - 32768`, range
−32768…+32767 — precisely the int16 range, so it never clips), and the
`bit_volts` field in `structure.oebin` lets any reader reconstruct the physical
value as `int16 * bit_volts`. The stored data is a lossless representation of
the raw acquired counts.

> Note: this differs from earlier revisions of this plugin, which wrote raw
> counts into the buffer while declaring a `bitVolts` ≠ 1. That displayed
> amplitudes ~5× off and made `int16 = raw / bitVolts` overflow the int16
> range, clipping large transients in the recording. The current scaling fixes
> both, at the cost of changing the recorded values relative to those
> revisions — re-derive any amplitude thresholds tuned against the old output.
