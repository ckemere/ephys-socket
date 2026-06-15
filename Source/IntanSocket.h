#ifndef __INTANSOCKETH__
#define __INTANSOCKETH__

#include <DataThreadHeaders.h>
#include "IntanInterface.h"
#include <memory>
#include <mutex>
#include <queue>

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
    const float DEFAULT_AUX_DATA_SCALE = 0.0000374f; // AUX bitVolts (Intan aux-ADC LSB)
    
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
    void setDebugMode(bool enable);
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

    /** Switch the aux COPI slots to the banked sequencer programs
        (accelerometer sweep on slot 1 -> one axis per packet, de-interleaved
        here via the packet command echo). Toggling while acquiring uploads
        the STANDBY banks and swaps them live -- this exercises the
        double-buffer + atomic-swap path. */
    bool setAuxSequencerMode(bool enable);
    bool isAuxSequencerMode() const { return auxSeqMode; }

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
