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
| **SETTLE** | **any time** | toggle amplifier fast settle (RHD Reg-0 D5) **and** DSP reset (CONVERT bit-H force). The PL injects `WRITE(0,0xFE)` / `WRITE(0,0xDE)` into the slot-0 aux position on the transition packet and forces bit-H on every channel CONVERT while the level is high. Datasheet pulse guidance is ~2.5/f_H (≈250 µs at 7.5 kHz upper cutoff) — toggle off promptly |
| **AUX SEQ** | **any time** | switch the 3 aux COPI positions to the banked sequencer programs (see below). Toggling while acquiring uploads the **standby** banks and swaps them atomically at a packet boundary — exercising the live double-buffer path |
| TTL Settle | any time | both amplifier fast settle and DSP reset follow the selected `digital_in` pin (TTL1–8 → pins 0–7); "-" disables. Combined (OR) with the SETTLE button's software level. Combo is enabled only while connected and resets to "-" on disconnect, so reconnecting into a fresh-booted board never silently re-enables a TTL trigger |

STATUS, SETTLE, AUX SEQ, and TTL Settle are deliberately usable **during
acquisition** — they exist to validate the firmware's runtime-control paths.
SETTLE and TTL Settle auto-enable the aux sequencer if it is off (the override
layer only reaches the chip while the sequencer is enabled).

On **CONNECT**, the editor pulls authoritative state from the device's
`get_status` response — chip indicators, debug-mode label, aux-sequencer
state, **and LFP engine state**. Reconnecting after a successful RESCAN
(or after configuring LFP out-of-band) restores everything without
re-running detection / re-uploading anything. A fresh-boot firmware
(channel\_enable = 0) is seeded with `0x0F` so the signal chain isn't
zero-channel; chip indicators stay dark to prompt the user to RESCAN.

## LFP / DSP engine (second DataStream)

Firmware ≥ 1.2 exposes a parallel LFP engine that low-pass filters and
decimates the amplifier streams through a host-programmed FIR. It emits LFP
frames on the **same unified UDP port as broadband** (`0x6800`), tagged
`stream_type = 2` in the common header; the plugin demuxes the two streams off
that single socket. Whether a second `DataStream` is published depends on the
firmware reporting `lfp_enabled` in the status response.

### Why one port, not one-port-per-stream

LFP originally streamed on its **own** UDP port (5001; broadband on 5000). The
firmware could do that essentially for free — the PL assembles each *complete*
wire packet (header + samples) in its output BRAM and the PS just DMAs it
straight into a `PBUF_REF` and sends it, so choosing a different destination
port per stream costs the board nothing. The failure mode was entirely on the
**receiver** side.

If a per-stream port has **no socket draining it** — the LFP viewer is closed,
or a tool binds 5001 only transiently — then *because nothing is listening*, the
host **OS kernel** answers every datagram to that dead port with an **ICMP "port
unreachable."** At the LFP frame rate that is a multi-kHz inbound flood back at
the board, and the board's **fully-polled** network stack (NO_SYS lwIP, no
interrupts) must receive and process every one of those ICMP replies. That stole
time from the 30 kHz acquisition loop (recv→transmit spikes of 40–60 µs against a
~33 µs budget) and drove catch-up bursts that exhausted the TX descriptors →
**dropped broadband packets**. A controlled test isolated it precisely:
broadband-only was pristine (~27 µs max, 0 over-budget, 0 drops); LFP-on with
5001 *undrained* spiked to ~63 µs with ~250 over-budget packets and ~12 drops;
LFP-on with 5001 *drained* was identical to broadband-only. So the firmware was
never the bottleneck — an unlistened UDP port was.

You *can* patch that host-side (keep a sink draining every stream port for the
whole session), but it is a fragile rule to remember, especially in a GUI plugin
where a viewer can be closed mid-run. Folding all streams onto **one** port
removes the failure mode structurally: there is only ever one socket, broadband
keeps it drained continuously, and LFP frames on that same port are simply
demuxed — or harmlessly ignored if LFP isn't being consumed. Nothing ever lands
on a dead port, so the host never emits ICMP and the board never sees the storm.
That is why the design is a single unified port with a `stream_type` tag rather
than a port per stream.

**Configuration is out-of-band**: the filter design lives outside the
plugin. Use `remote/net.py` from the MicroZedIntanInterface repo:

```python
configure_lfp(sock, lane_mask=0x0F, decim_R=10, num_taps=128, cutoff_hz=600.0)
lfp_enable(sock, True)
```

…then **reconnect the plugin** so it picks up the new config in
`updateSettings`. See [docs/lfp.md](lfp.md) for the full external-tool
recipe.

When LFP is enabled, the plugin publishes:

- a second `DataStream` named `IntanLFP`, sample rate
  `30000 / lfp_decim_R` Hz
- one `ContinuousChannel` per amplifier channel on every enabled LFP lane
  (the same 8-bit lane mask convention as broadband), named
  `LFP_A_CH1..`, `LFP_B_CH1..`, with the same `0.195 µV/LSB` scaling as
  broadband neural

The LFP frame's timestamp (`frame_seq × decim_R`) is in broadband ticks,
so downstream nodes can correlate the two streams.

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

Status response sizes grow over time. Current firmware sends **160 bytes**;
the plugin accepts every prior size and decodes optional fields based on
what the device actually sent:

| Size | Firmware | Added |
|---|---|---|
| 86 | pre-aux | base config + UDP info |
| 98 | aux-seq-v2 | aux sequencer block (`auxSeqEnabled`, `fastSettleActive`, ...) |
| 122 | fw 1.1.0.0 | DMA / perf instrumentation (skipped by the plugin) |
| 126 | `65d5fb5` | **`aux_ctrl`** — CTRL_REG_22 readback: SW level, GPIO_EN, pin select for each of fast settle / DSP reset / digout, plus Reg-3 static |
| 148 | `7fb41dc` | **`rhd_reg[22]`** — RHD chip register shadow (commanded state of regs 0..21) |
| 160 | `0e99881` | **LFP engine config + status** — `lfp_enabled`, `lfp_lane_mask`, `lfp_decim_R`, `lfp_num_taps`, `lfp_packets_sent`, `lfp_overrun` |

The plugin sizes its buffer to the largest known form (160) so an unread
suffix never sits in the TCP queue corrupting the next command's ACK. The
`aux_ctrl` field is what lets the editor pull the TTL Settle combo from
the device on connect instead of pushing -- a reconnect after a prior
configuration restores the prior pin for free.

The authoritative protocol documentation lives in the MicroZedIntanInterface
repo: `docs/command-bank-design.md` (design), `docs/NIGHT_LOG-2026-06-11.md`
(implementation + verification), and `firmware/include/main.h` (register map).

## Channel scaling and data storage

Every sample is published into the GUI data buffer as:

```
buffer_value = (raw_adc_count - 32768) * bitVolts
```

and the **same `bitVolts`** is declared on the channel:

| Channel type | bitVolts | buffer units | units label | nominal range |
|---|---|---|---|---|
| ELECTRODE (neural) | `0.195` | µV | `uV` | viewer default |
| AUX (auxin1/2/3, e.g. accelerometer) | `1.0` | raw signed ADC count | `a.u.` | ±32768 |

The `- 32768` decodes the chip's offset-binary samples to a signed value
around zero; it is a constant, reversible representation choice, **not** a
baseline subtraction or detrend — no acquired information is altered or lost.

For neural channels, `bitVolts = 0.195` µV/LSB gives a buffer value in
microvolts, matching the OpenEphys acquisition-board plugin.

For aux channels, `bitVolts = 1.0` deliberately leaves the value as the raw
signed ADC count and the channel is labelled `a.u.` (arbitrary units). The
LFP viewer's range selector then offers the natural ±32768 window plus
zoom-in subdivisions for the small-amplitude regime where typical
accelerometer / supply-rail signals live. A reader who wants physical units
can apply the chip's data-sheet conversion: 1 LSB ≈ 2.45 V / 65536 = 37.4 µV
at the RHD aux input.

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
`bit_volts` field in `structure.oebin` lets any reader reconstruct a physical
value as `int16 * bit_volts`. For aux channels that physical value is the same
signed integer (bitVolts = 1.0); for neural channels it's microvolts.

The stored data is a lossless representation of the raw acquired counts.

> Note: this differs from earlier revisions of this plugin, which wrote raw
> counts into the buffer while declaring a `bitVolts` ≠ 1. That displayed
> amplitudes ~5× off (neural) or ~1000× off (aux, V/LSB silently labelled mV)
> and made `int16 = raw / bitVolts` overflow the int16 range, clipping large
> transients in the recording. The current scaling fixes both, at the cost of
> changing the recorded values relative to those revisions — re-derive any
> amplitude thresholds tuned against the old output.
