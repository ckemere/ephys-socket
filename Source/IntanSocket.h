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
    const float DEFAULT_DATA_SCALE = 0.195f;    // Intan µV per bit
    
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

    int num_channels;
    uint8_t channel_enable_mask;

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
    
    /** Calculate number of channels from enable mask */
    int calculateNumChannels(uint8_t mask);

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
    Array<int64> sampleNumbers;
    Array<double> timestamps;
    Array<uint64> ttlEventWords;
    
    /** Sample counter */
    int64 totalSamples;
    uint64 eventState;
    
    /** Error tracking */
    std::atomic<bool> hasError;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(IntanSocket);
};
} // namespace IntanSocketNode
#endif
