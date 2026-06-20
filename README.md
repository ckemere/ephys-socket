# Intan Socket (MicroZed fork of Ephys Socket)

An Open Ephys GUI `DataThread` plugin for the **MicroZed/Zynq-7020 Intan
acquisition system** ([MicroZedIntanInterface](https://github.com/kemerelab/MicroZedIntanInterface)).
Forked from the original [Ephys Socket](https://github.com/open-ephys-plugins/ephys-socket)
by Jonathan Newman ([@jonnew](https://github.com/jonnew)); this fork replaces the
generic matrix-over-TCP source with the MicroZed device protocol:

- **TCP control** (port 6000): start/stop, configuration, chip auto-detection,
  and the aux-sequencer commands. `IntanInterface.{h,cpp}` is a
  standalone C++ client for this protocol (no JUCE dependency) — it is the
  third consumer of the register/packet contract, after the firmware and
  `remote/net.py`.
- **UDP data** (port 5000): one packet per ~30 kHz sample; 10 header words +
  up to 140 data words depending on the channel-enable mask.

<p align="center">
  <img src="ephys-socket.png" width="80%" />
</p>

The board firmware, the `remote/net.py` reference client, and this plugin are
the **three consumers of the same register/packet contract** — see the
MicroZed repo when changing the protocol.

## Usage

1. **Flash the board** with a current MicroZed image (`blobs/BOOT.bin` from the
   MicroZedIntanInterface repo) and put it on the network. Default device IP is
   `192.168.18.10`; put your host on the same subnet (port 6000 TCP control,
   5000 UDP data).
2. **Install the plugin** (see [docs/building.md](docs/building.md)) so
   OpenEphys finds it in its `plugins` directory.
3. In OpenEphys, open the **Processor List → Sources** and drag **Intan Socket**
   in as the signal-chain source.
4. In the editor, set **Device IP** (and TCP/UDP ports if non-default), then
   click **CONNECT**. The editor mirrors the device's current state —
   chip indicators, DBG button, aux flags — so reconnecting after a successful
   RESCAN restores everything for free.
5. Click **RESCAN** to auto-detect connected chips on both port A and port B
   in parallel (one sweep over 16 cable-phase values, ce=0xFF) and pick the
   optimal phase per port. The channel count updates to match.
6. Press **play** to stream. Neural channels appear as `A_CH1…` / `B_CH1…`,
   aux inputs as `A_AUX0_1…` / `B_AUX0_1…` (per port × CIPO line). Click
   **STATUS** at any time to dump full device state to the console (View →
   **Console**, or Shift+C in a Release build).
7. To exercise the run-time features: **SETTLE** toggles amplifier fast
   settle and DSP reset together; the **TTL Settle** dropdown makes both
   follow a digital-input pin; **AUX SEQ** switches the aux slots to the
   banked accelerometer/housekeeping programs (and, toggled while streaming,
   performs a live bank swap).

A connected **headstage accelerometer** (auxin1/2/3) shows up on the AUX
channels — select the **AUX** channel type in the LFP viewer's range selector
to see it at the right scale.

## Further reading

- [docs/notes.md](docs/notes.md) — editor controls reference, aux-sequencer /
  accelerometer parse mode, channel scaling, recordings storage, and how the
  second LFP `DataStream` is published.
- [docs/lfp.md](docs/lfp.md) — external recipe for configuring the LFP/DSP
  engine (filter design + upload + enable) from `remote/net.py`.
- [docs/building.md](docs/building.md) — Windows / Linux / macOS build
  instructions.

## Attribution

Original Ephys Socket plugin by Jonathan Newman ([@jonnew](https://github.com/jonnew)).
MicroZed Intan fork by the Kemere Lab.
