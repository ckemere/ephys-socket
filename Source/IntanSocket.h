#ifndef __INTANSOCKETH__
#define __INTANSOCKETH__

#include <DataThreadHeaders.h>
#include "IntanInterface.h"
#include <memory>
#include <mutex>
#include <queue>
#include <deque>
#include <vector>
#include <atomic>

namespace IntanSocketNode
{
class IntanSocket : public DataThread
{
public:
    /** Default parameters */
    const String DEFAULT_DEVICE_IP = "192.168.18.10";
    const int DEFAULT_TCP_PORT = 6000;
    const int DEFAULT_UDP_PORT = 5000;
    const float SAMPLE_RATE = 30000.0f; // Fixed for now
    // Scaling matches the OpenEphys acquisition-board plugin exactly
    // (Source/devices/oni/AcqBoardONI.cpp getBitVolts + sample conversion):
    // each sample is published as (raw_count - 32768) * bitVolts, and the
    // SAME bitVolts is declared on the channel. See processDataPacket() and
    // the storage note in README.md.
    const float DEFAULT_DATA_SCALE = 0.195f;       // ELECTRODE bitVolts (Intan amplifier LSB, µV/bit)
    // AUX channels: bitVolts = 1.0 -> values published as the raw signed
    // ADC count (raw - 32768). Units are "a.u." so the LFP viewer doesn't try
    // to interpret the numeric range as a physical unit. Full range is the
    // signed-16-bit window centred on 0; smaller ranges in the range selector
    // zoom into the typical accelerometer / supply-rail signal. Reader-side
    // conversion: 1 LSB = 2.45V / 65536 = 37.4 µV at the RHD aux input.
    const float DEFAULT_AUX_DATA_SCALE = 1.0f;
    
    /** Parameter limits */
    const int MIN_PORT = 1024;
    const int MAX_PORT = 65535;
    const float MIN_DATA_SCALE = 0.0f;
    const float MAX_DATA_SCALE = 10.0f;

    /** Network parameters */
    String device_ip;
    int tcp_port;
    int udp_port;
    float data_scale;
    float aux_data_scale;

    uint8_t channel_enable_mask;
    int num_channels;

    // LFP/DSP engine state, mirrored from the device at connect-time. The
    // second DataStream / sourceBuffer is built from these values; if
    // lfp_enabled is false at connect, no second stream is published (the
    // user enables it from net.py / external tool and reconnects).
    bool    lfp_enabled = false;
    uint8_t lfp_lane_mask = 0;
    uint8_t lfp_decim_R = 0;
    uint8_t lfp_num_taps = 0;
    int     lfp_num_channels = 0;   // popcount(lfp_lane_mask) * 32

    /** Constructor */
    IntanSocket(SourceNode* sn);

    /** Destructor */
    ~IntanSocket();

    /** Creates custom editor */
    std::unique_ptr<GenericEditor> createEditor(SourceNode* sn);

    /** Create the DataThread object*/
    static DataThread* createDataThread(SourceNode* sn);

    /** Registers the parameters for the DataThread */
    void registerParameters() override;

    /** Returns true if connected to device */
    bool foundInputSource() override;

    /** Sets info about available channels */
    void updateSettings(OwnedArray<ContinuousChannel>* continuousChannels,
                       OwnedArray<EventChannel>* eventChannels,
                       OwnedArray<SpikeChannel>* spikeChannels,
                       OwnedArray<DataStream>* sourceStreams,
                       OwnedArray<DeviceInfo>* devices,
                       OwnedArray<ConfigurationObject>* configurationObjects) override;

    /** Handles parameter value changes */
    void parameterValueChanged(Parameter* parameter) override;

    /** Disconnects from device */
    void disconnectDevice();

    /** Attempts to connect to device */
    bool connectDevice(bool printOutput = true);

    /** Returns if any errors occurred */
    bool errorFlag();
    
    /** Run auto-detection of connected chips */
    bool runAutoDetection(IntanInterface::AutoDetectionResult& result, bool verbose = false);
    
    /** Apply auto-detection configuration */
    bool applyDetectionConfig(const IntanInterface::AutoDetectionResult& result);

    /** Debug mode is useful for testing. **/
    /** Enable/disable debug mode. When enabling, mask selects single-port
        (0x0F → 4 streams, 134 channels) or dual-port (0xFF → 8 streams,
        268 channels) synthetic data. Ignored when disabling. */
    void setDebugMode(bool enable, uint8_t mask = 0xFF);
    bool isDebugMode() const { return debugMode; }

    // ------------------------------------------------------------------
    // Aux command sequencer tooling (firmware aux-seq-v2)
    // ------------------------------------------------------------------

    /** Print the full device status (incl. aux sequencer state) to the
        console. Safe to call during acquisition. */
    void printDeviceStatus();

    /** Software-triggered amplifier fast settle (RHD Reg-0 D5). Enables the
        aux sequencer automatically if needed. Safe during acquisition. */
    void setManualFastSettle(bool active);
    bool isFastSettleOn() const { return fastSettleSw; }

    /** Follow a digital_in pin for fast settle (-1 = off). Combined (OR)
        with the software level in the PL. */
    void setFastSettleTTLPin(int pin);

    /** The device's CURRENT TTL fast-settle pin selection (-1 = off, 0..7 =
        digital_in pin). Reads back from the cached aux_ctrl status field
        (firmware 65d5fb5+). Returns the locally tracked fastSettleTTL value
        on older firmware that doesn't surface it. */
    int getDeviceTTLFastSettlePin() const { return fastSettleTTL; }

    /** Switch the aux COPI slots to the banked sequencer programs
        (accelerometer sweep on slot 1 -> one axis per packet, de-interleaved
        here via the packet command echo). Toggling while acquiring uploads
        the STANDBY banks and swaps them live -- this exercises the
        double-buffer + atomic-swap path. */
    bool setAuxSequencerMode(bool enable);
    bool isAuxSequencerMode() const { return auxSeqMode; }

    /** Enable / disable the firmware's LFP/DSP engine. LFP frames arrive on the
        SAME unified UDP port as broadband (stream_type=2). When enabling and the firmware has no
        configuration yet (lane_mask = 0 or decim_R = 0), this first
        applies a default configure -- same values as remote/net.py's
        configure_lfp(): 0x0F lane mask, decim 15 (2 kHz output), 128-tap
        Hamming-windowed sinc with 600 Hz cutoff. Filter UPDATES still go
        through the external tool. Caller is expected to invoke
        CoreServices::updateSignalChain afterwards so the stream count
        change actually takes effect in OE. */
    bool setLfpEnabled(bool enable);
    bool isLfpEnabled() const { return lfp_enabled; }

    /** Apply the net.py default LFP configuration (lane_mask=0x0F,
        decim_R=15, num_taps=128, cutoff=600 Hz) -- DOES NOT enable.
        Used by setLfpEnabled(true) when the firmware is fresh. */
    bool configureLfpDefaults();

    // ------------------------------------------------------------------
    // WAVELET (DWT) scalogram consumer API (used by the DwtCanvas viewer).
    //
    // The wavelet engine (mz-unified-wavelet firmware) emits ONE UNIFIED
    // datagram PER OCTAVE, rate-aligned: octave o updates at 3 kHz / 2^o
    // (octave 0 at 3 kHz, the slowest at ~23 Hz). IntanInterface decodes each
    // octave packet (stream_type=3) and calls processWaveletPacket(), which
    // places that octave's block into a HELD multirate surface (each octave is
    // held at its last value between its updates -- the truthful multirate
    // scalogram) and, on each octave-0 (fastest) update, parks a synthesized
    // full COLUMN (all K lanes x nscales, where nscales = n_octaves*n_voices)
    // into waveletRing for the canvas. The canvas polls drainWaveletSince().
    //
    // This consumer is OPTIONAL and self-contained: it never touches the
    // broadband or LFP data path, so it cannot regress those streams. It
    // does nothing until the firmware's wavelet engine is enabled (configured
    // out of band via remote/net.py configure_wavelet + wavelet_enable).
    // ------------------------------------------------------------------

    /** One reassembled scalogram column: all K lanes x nscales magnitudes (the
        held multirate surface sampled at one time step). Lane-major: lane L,
        scale s (s = octave*n_voices + voice; bin 0 = highest freq) lives at
        mags[L * nscales + s]. */
    struct WaveletColumn {
        uint32_t seq;                  // octave-0 SEQ at synthesis time (monotone)
        uint64_t timestamp;
        std::vector<float> mags;       // K * nscales magnitudes (sqrt(re^2+im^2))
    };

    /** Snapshot of all wavelet columns newer than the caller's last-seen seq.
        The caller sets `sinceSeq`; we return columns with seq != that and the
        current surface geometry so the canvas can size its image / axes. */
    struct WaveletDrain {
        std::vector<WaveletColumn> columns;
        int K = 0;           // streamed lanes
        int nscales = 0;     // n_octaves * n_voices
        int nOct = 0;
        int nVoc = 0;
        uint64_t dropped = 0;  // ring overflow (host-side) since session start
    };
    WaveletDrain drainWaveletSince(uint32_t sinceSeq);

    /** Whether wavelet octave packets have been arriving this session. */
    bool isWaveletRunning() const { return waveletColumnsReceived.load() > 0; }

private:
    const int bufferSizeInSeconds = 10;
    
    /** Receives data from IntanInterface and pushes to DataBuffer */
    bool updateBuffer() override;

    bool isReady() override;

    /** Initializes device and starts acquisition */
    bool startAcquisition() override;

    /** Stops acquisition */
    bool stopAcquisition() override;

    /** Converts Intan data packet to Open Ephys format */
    void processDataPacket(const uint32_t* data, size_t wordCount, uint64_t timestamp);

    /** Pushes one LFP frame (from the IntanInterface LFP listener) into
        sourceBuffers[1]. Each frame is one decimated sample per channel
        across `popcount(lane_mask) * 32` LFP channels. */
    void processLfpFrame(const IntanInterface::LfpFrame& frame);

    /** Place ONE wavelet octave packet (from the IntanInterface wavelet
        listener) into the held multirate surface; on each octave-0 update,
        synthesize and park a full scalogram column for the canvas. Runs on the
        demux thread -- kept cheap, and entirely off the broadband/LFP path. */
    void processWaveletPacket(const IntanInterface::WaveletPacket& pkt);

    /** Number of enabled 16-bit data streams in the 8-bit mask.
        Bits 0-3 = port A (A_CIPO0_REG, A_CIPO0_DDR, A_CIPO1_REG, A_CIPO1_DDR);
        bits 4-7 = port B (B_CIPO0_REG, B_CIPO0_DDR, B_CIPO1_REG, B_CIPO1_DDR). */
    static int countStreams(uint8_t mask) {
        int n = 0;
        for (int b = 0; b < 8; ++b)
            n += ((mask >> b) & 1);
        return n;
    }

    /** Number of aux banks. Only the four "regular" streams carry aux inputs
        (the DDR streams just resample them):
          bit 0 = A_CIPO0_REG -> aux bank 0
          bit 2 = A_CIPO1_REG -> aux bank 1
          bit 4 = B_CIPO0_REG -> aux bank 2
          bit 6 = B_CIPO1_REG -> aux bank 3 */
    static int countAuxBanks(uint8_t mask) {
        return ((mask & 0b00000001) != 0) + ((mask & 0b00000100) != 0)
             + ((mask & 0b00010000) != 0) + ((mask & 0b01000000) != 0);
    }

    /** Total Open Ephys channels for a mask: 32 amplifiers per stream
        plus 3 aux inputs per aux bank. */
    int calculateNumChannels(uint8_t mask) {
        return countStreams(mask) * 32 + countAuxBanks(mask) * 3;
    };
    
    /** Our IntanInterface library instance */
    std::unique_ptr<IntanInterface> intanInterface;
    
    /** Thread-safe queue for data packets */
    struct DataPacket {
        std::vector<uint32_t> data;
        uint64_t timestamp;
    };
    std::queue<DataPacket> dataQueue;
    std::mutex queueMutex;
    
    /** Buffers for conversion */
    std::vector<float> convbuf;
    std::vector<float> lfpConvBuf;
    
    /** Sample counter */
    int64 totalSamples;
    uint64 eventState;
    
    /** Error tracking */
    std::atomic<bool> hasError;

    /** Debug mode */
    bool debugMode;

    /** Aux sequencer tooling state */
    bool auxSeqMode = false;       // banked aux programs active
    bool fastSettleSw = false;     // software fast-settle level
    int fastSettleTTL = -1;        // digital_in pin for fast settle (-1 = off)

    /** Push the combined fast-settle config to the device */
    bool pushFastSettleConfig();

    /** Sample-and-hold accelerometer state for sequencer-mode de-interleave:
        [aux bank: 0=A_CIPO0, 1=A_CIPO1, 2=B_CIPO0, 3=B_CIPO1][axis 0..2],
        raw offset-binary samples */
    uint16_t lastAccel[4][3] = {{0x8000, 0x8000, 0x8000},
                                {0x8000, 0x8000, 0x8000},
                                {0x8000, 0x8000, 0x8000},
                                {0x8000, 0x8000, 0x8000}};

    // ------------------------------------------------------------------
    // WAVELET held multirate surface + canvas ring (off the broadband/LFP path).
    //   waveletSurface_[octave] holds that octave's last block of magnitudes,
    //   K lanes x n_voices each (sqrt(re^2+im^2)), held between octave updates.
    //   On each octave-0 update we flatten the whole surface into a full column
    //   (K x nscales) and push it into waveletRing for the canvas.
    // Producer = demux thread (processWaveletPacket); consumer = visualizer
    // thread (drainWaveletSince). Guarded by waveletMu_.
    // ------------------------------------------------------------------
    static constexpr int kWaveletMaxOctaves = 16;
    static constexpr size_t kWaveletRingCapacity = 4096;  // ~1.4 s at 3 kHz oct0
    mutable std::mutex waveletMu_;
    // per-octave held magnitudes: [octave] -> K*n_voices magnitudes (lane-major)
    std::vector<float> waveletHeld_[kWaveletMaxOctaves];
    std::deque<WaveletColumn> waveletRing_;
    int      waveletK_     = 0;     // streamed lanes (from the latest packet)
    int      waveletNOct_  = 0;     // n_octaves (full surface height = nOct*nVoc)
    int      waveletNVoc_  = 0;     // n_voices
    uint64_t waveletDroppedTotal_ = 0;     // host-side ring overflow
    std::atomic<uint64_t> waveletColumnsReceived{0};

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(IntanSocket);
};
} // namespace IntanSocketNode
#endif
