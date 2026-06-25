#ifndef __INTANSOCKETH__
#define __INTANSOCKETH__

#include <DataThreadHeaders.h>
#include "IntanInterface.h"
#include <memory>
#include <mutex>
#include <queue>
#include <deque>
#include <atomic>
#include <vector>

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

    /** Enable / disable the firmware's LFP/DSP engine + the second
        UDP stream (port 5001). When enabling and the firmware has no
        configuration yet (lane_mask = 0 or decim_R = 0), this first
        applies a default configure -- same values as remote/net.py's
        configure_lfp(): 0x0F lane mask, decim 10 (3 kHz output), 128-tap
        Hamming-windowed sinc with 600 Hz cutoff. Filter UPDATES still go
        through the external tool. Caller is expected to invoke
        CoreServices::updateSignalChain afterwards so the stream count
        change actually takes effect in OE. */
    bool setLfpEnabled(bool enable);
    bool isLfpEnabled() const { return lfp_enabled; }

    /** Apply the net.py default LFP configuration (lane_mask=0x0F,
        decim_R=10, num_taps=128, cutoff=600 Hz) -- DOES NOT enable.
        Used by setLfpEnabled(true) when the firmware is fresh. */
    bool configureLfpDefaults();

    // ------------------------------------------------------------------
    // Wavelet (DWT) scalogram consumer API (used by the DwtCanvas viewer).
    // The wavelet engine (mz-tier3-wavelet firmware) emits one UDP datagram
    // per scalogram COLUMN on port 5004; IntanInterface decodes it and calls
    // processWaveletFrame(), which parks the column in waveletRing. The canvas
    // drains the ring each refresh. The engine config + enable live in the
    // wavelet firmware / remote/net.py (configure_wavelet, wavelet_enable);
    // this plugin only consumes the monitor stream. The packet header is fully
    // self-describing (n_oct/n_voc/K/nscales), so the viewer works regardless
    // of whether get_status surfaces wavelet config.
    // ------------------------------------------------------------------
    // One column = one wavelet datagram. Magnitudes are stored (sqrt(re^2+im^2)
    // computed in the canvas from the raw int32 payload kept here).
public:
    struct WaveletColumn {
        uint32_t seq;
        uint64_t timestamp;
        uint8_t  K;
        uint16_t nscales;
        std::vector<int32_t> samples;   // size = K * nscales * 2 (re,im per bin)
    };

    /** Snapshot of all wavelet columns with seq > `sinceSeq`. The caller sets
        the returned newest column's seq as `sinceSeq` for the next call.
        Capped at the ring size; older drops are reported via `dropped`. */
    struct WaveletDrain {
        std::vector<WaveletColumn> columns;
        uint64_t dropped;             // total drops since session start
        int K;
        int nscales;
        int nOct;
        int nVoc;
    };
    WaveletDrain drainWaveletSince(uint32_t sinceSeq);

    /** Whether wavelet data has been arriving (any column since session start). */
    bool isWaveletRunning() const { return waveletColumnsReceived.load() > 0; }

private:

    /** Park one wavelet scalogram column (from the IntanInterface wavelet
        listener) into waveletRing for the visualizer. Runs on the listener
        thread; keep it cheap. */
    void processWaveletFrame(const IntanInterface::WaveletFrame& frame);

    // Wavelet ring buffer (producer = listener thread, consumer = visualizer).
    // 2048 columns at K=32, nscales=32 is ~16 MB.
    static constexpr int WAVELET_RING_CAPACITY = 2048;
    std::deque<WaveletColumn> waveletRing;
    mutable std::mutex waveletRingMu;
    uint64_t waveletDroppedTotal = 0;
    uint8_t  waveletK_ = 0;
    uint16_t waveletNscales_ = 0;
    uint8_t  waveletNOct_ = 0;
    uint8_t  waveletNVoc_ = 0;
    std::atomic<uint64_t> waveletColumnsReceived{0};
    std::atomic<uint64_t> waveletSeqGaps{0};
    std::atomic<uint32_t> waveletLastSeq{0xFFFFFFFFu};

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

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(IntanSocket);
};
} // namespace IntanSocketNode
#endif
