#ifndef INTAN_INTERFACE_H
#define INTAN_INTERFACE_H

#include <cstdint>
#include <string>
#include <functional>
#include <memory>
#include <vector>
#include <sstream>

/**
 * @brief C++ API for Zynq-based Intan neural recording interface
 * 
 * This class provides a high-level interface to the Zynq FPGA system that
 * acquires data from Intan chips via SPI and streams it over UDP.
 * 
 * Thread Safety: All public methods are thread-safe.
 * 
 * Example usage:
 * @code
 *   IntanInterface intan("192.168.18.10");
 *   
 *   if (intan.foundInputSource() && intan.isReady()) {
  *       // Run auto-detection
 *       IntanInterface::AutoDetectionResult result;
 *       if (intan.runAutoDetection(result, true)) {
 *           std::cout << result.getSummary() << std::endl;
 *           
 *           if (result.success) {
 *               intan.applyDetectionConfig(result);
 *           }
 *       }
 *       
*       intan.setDataCallback([](const uint32_t* data, size_t words, uint64_t ts) {
 *           // Process data packet
 *       });
 *       
 *       intan.loadInitSequence();
 *       intan.loadConvertSequence();
 *       intan.startAcquisition();
 *       
 *       // ... acquisition runs in background ...
 *       
 *       intan.stopAcquisition();
 *   }
 * @endcode
 */
class IntanInterface {
public:
    /**
     * @brief Chip type identification
     */
    enum class ChipType {
        NONE,
        RHD2132,  // 32-channel, no DDR
        RHD2164   // 64-channel, with DDR
    };

    /**
     * @brief Device status information
     */
    struct DeviceStatus {
        // Version and identification
        uint16_t protocolVersion;
        uint16_t deviceType;
        uint32_t firmwareVersion;
        
        // PL Hardware Status
        uint64_t timestamp;
        uint32_t packetsSent;
        uint32_t bramWriteAddr;
        uint16_t fifoCount;
        uint8_t stateCounter;
        uint8_t cycleCounter;
        bool transmissionActive;
        bool loopLimitReached;
        
        // PS Software Status
        uint32_t packetsReceived;
        uint32_t errorCount;
        uint32_t udpPacketsSent;
        uint32_t udpSendErrors;
        uint32_t psReadAddr;
        uint32_t packetSizeWords;
        bool streamEnabled;
        
        // Current Configuration
        uint32_t loopCount;
        uint8_t phase0;
        uint8_t phase1;
        uint8_t channelEnable;
        bool debugMode;
        
        // UDP Stream Information
        std::string udpDestIp;
        uint16_t udpDestPort;
        uint32_t udpBytesSent;
        
        // Helper methods
        std::string getFirmwareVersionString() const;
        std::string getChannelEnableString() const;
    };
    
    /**
     * @brief Channel enable bit masks
     */
    enum ChannelMask : uint8_t {
        CHANNEL_CIPO0_REGULAR = 0x01,
        CHANNEL_CIPO0_DDR     = 0x02,
        CHANNEL_CIPO1_REGULAR = 0x04,
        CHANNEL_CIPO1_DDR     = 0x08,
        CHANNEL_ALL           = 0x0F,
        CHANNEL_NONE          = 0x00
    };
    
    /**
     * @brief Statistics about data reception
     */
    struct ReceptionStats {
        uint64_t totalPackets;
        uint64_t totalErrors;
        uint64_t magicErrors;
        uint64_t timestampErrors;
        uint64_t sizeErrors;
        double averageRate;      // packets/second
        double instantRate;      // packets/second (last 5 seconds)
        double dataRateMbps;     // data rate in Mbps
        uint64_t lastTimestamp;
    };
    
    /**
     * @brief Result from testing a single phase setting
     */
    struct PhaseTestResult {
        uint8_t phase;
        double cipo0Score;
        double cipo1Score;
        bool cipo0HasDdr;
        bool cipo1HasDdr;
        ChipType cipo0ChipType;
        ChipType cipo1ChipType;
    };
    
    /**
     * @brief Complete auto-detection results
     */
    struct AutoDetectionResult {
        bool success;
        bool chipsDetected;
        
        // Best configuration
        uint8_t bestPhase0;
        uint8_t bestPhase1;
        uint8_t optimalChannelMask;
        
        // CIPO0 detection
        bool cipo0Detected;
        ChipType cipo0ChipType;
        bool cipo0HasDdr;
        double cipo0Score;
        
        // CIPO1 detection
        bool cipo1Detected;
        ChipType cipo1ChipType;
        bool cipo1HasDdr;
        double cipo1Score;
        
        // All phase test results
        std::vector<PhaseTestResult> allPhaseResults;
        
        /**
         * @brief Get chip type as string
         */
        static std::string chipTypeToString(ChipType type);
        
        /**
         * @brief Get human-readable summary
         */
        std::string getSummary() const;
        
        /**
         * @brief Get detailed channel summary
         */
        std::string getChannelSummary() const;
    };
        
    /**
     * @brief Callback type for receiving data packets
     * 
     * @param data Pointer to 32-bit word array
     * @param wordCount Number of 32-bit words in packet
     * @param timestamp 64-bit timestamp from device
     */
    using DataCallback = std::function<void(const uint32_t* data, size_t wordCount, uint64_t timestamp)>;
    
    /**
     * @brief Callback type for error notifications
     * 
     * @param errorMessage Human-readable error description
     */
    using ErrorCallback = std::function<void(const std::string& errorMessage)>;
    
    /**
     * @brief Constructor - discovers and initializes connection
     * 
     * @param deviceIp IP address of Zynq device (default: 192.168.18.10)
     * @param tcpPort TCP command port (default: 6000)
     * @param udpPort UDP data port (default: 5000)
     */
    explicit IntanInterface(const std::string& deviceIp = "192.168.18.10",
                           uint16_t tcpPort = 6000,
                           uint16_t udpPort = 5000);
    
    /**
     * @brief Destructor - cleanly shuts down threads and connections
     */
    ~IntanInterface();
    
    // Prevent copying
    IntanInterface(const IntanInterface&) = delete;
    IntanInterface& operator=(const IntanInterface&) = delete;
    
    // ========================================================================
    // CONNECTION MANAGEMENT
    // ========================================================================
    
    /**
     * @brief Check if connection to device has been established
     * 
     * @return true if TCP connection is active and device responds
     */
    bool foundInputSource() const;
    
    /**
     * @brief Check if system is ready to start acquisition
     * 
     * Verifies that:
     * - TCP connection is established
     * - UDP listener is running
     * - Device is not currently transmitting
     * - No critical errors
     * 
     * @return true if ready to start
     */
    bool isReady() const;
    
    /**
     * @brief Get detailed device status
     * 
     * @param status Output parameter filled with current status
     * @return true if status was successfully retrieved
     */
    bool getStatus(DeviceStatus& status) const;
    
    /**
     * @brief Get local reception statistics
     * 
     * @param stats Output parameter filled with reception stats
     */
    void getReceptionStats(ReceptionStats& stats) const;
    
    // ========================================================================
    // AUTO-DETECTION
    // ========================================================================
    
    /**
     * @brief Run automated chip detection and cable phase optimization
     * 
     * This function:
     * 1. Initializes the Intan chips
     * 2. Loads the cable test sequence
     * 3. Tests all 16 phase delay settings (0-15)
     * 4. Scores each phase based on INTAN pattern detection
     * 5. Identifies chip types (RHD2132 vs RHD2164)
     * 6. Determines optimal channel enable mask
     * 
     * The device must be in a stopped state (not transmitting) before calling.
     * 
     * @param result Output parameter filled with detection results
     * @param verbose If true, prints progress information to stdout
     * @return true if detection completed (check result.success for chip detection)
     */
    bool runAutoDetection(AutoDetectionResult& result, bool verbose = false);
    
    /**
     * @brief Apply configuration from auto-detection results
     * 
     * Sets the phase delays and channel enable mask based on detection results.
     * Also loads the normal conversion sequence (not cable test).
     * Does not start acquisition - call startAcquisition() separately.
     * 
     * @param result Detection results from runAutoDetection()
     * @return true if configuration was applied successfully
     */
    bool applyDetectionConfig(const AutoDetectionResult& result);
    
    // ========================================================================
    // ACQUISITION CONTROL
    // ========================================================================
    
    /**
     * @brief Start continuous data acquisition
     * 
     * Sends START command to device. Data will be streamed via UDP
     * and delivered through the registered data callback.
     * 
     * @return true if command succeeded
     */
    bool startAcquisition();
    
    /**
     * @brief Stop continuous data acquisition
     * 
     * Sends STOP command to device. Waits briefly for in-flight
     * packets to be received.
     * 
     * @return true if command succeeded
     */
    bool stopAcquisition();
    
    /**
     * @brief Reset timestamp counter on device
     * 
     * Resets the PL timestamp counter to zero. Should be called
     * when transmission is stopped.
     * 
     * @return true if command succeeded
     */
    bool resetTimestamp();
    
    // ========================================================================
    // CONFIGURATION
    // ========================================================================
    
    /**
     * @brief Set loop count (number of packets to acquire)
     * 
     * @param count Number of packets (0 = infinite)
     * @return true if command succeeded
     */
    bool setLoopCount(uint32_t count);
    
    /**
     * @brief Set CIPO phase delay compensation
     * 
     * Sets the phase delay for sampling CIPO data to compensate
     * for cable delays. Valid range: 0-15 for each phase.
     * 
     * @param phase0 Phase delay for CIPO0 (0-15)
     * @param phase1 Phase delay for CIPO1 (0-15)
     * @return true if command succeeded
     */
    bool setPhaseSelect(uint8_t phase0, uint8_t phase1);
    
    /**
     * @brief Enable or disable debug mode
     * 
     * In debug mode, the device generates synthetic sine wave data
     * instead of reading from the Intan chips.
     * 
     * @param enable true to enable debug mode
     * @return true if command succeeded
     */
    bool setDebugMode(bool enable);
    
    /**
     * @brief Set which data channels are enabled
     * 
     * Use ChannelMask enum values. Each bit enables one 16-bit channel:
     * - Bit 0: CIPO0 regular rate
     * - Bit 1: CIPO0 double data rate
     * - Bit 2: CIPO1 regular rate
     * - Bit 3: CIPO1 double data rate
     * 
     * @param channelMask Bitmask of enabled channels (0x0-0xF)
     * @return true if command succeeded
     */
    bool setChannelEnable(uint8_t channelMask);
    
    /**
     * @brief Configure UDP destination
     * 
     * Changes where the device sends UDP data packets. By default,
     * the constructor auto-configures this to the local machine.
     * 
     * @param ipAddress Destination IP address (e.g., "192.168.18.100")
     * @param port Destination UDP port
     * @return true if command succeeded
     */
    bool setUdpDestination(const std::string& ipAddress, uint16_t port);
    
    // ========================================================================
    // COPI COMMAND SEQUENCES
    // ========================================================================
    
    /**
     * @brief Load standard conversion sequence
     * 
     * Loads the COPI command sequence for normal data acquisition
     * from Intan channels 0-31.
     * 
     * @return true if command succeeded
     */
    bool loadConvertSequence();
    
    /**
     * @brief Load initialization sequence
     * 
     * Loads the COPI command sequence to initialize Intan chips.
     * Should be called once before first data acquisition.
     * 
     * @return true if command succeeded
     */
    bool loadInitSequence();
    
    /**
     * @brief Load cable length test sequence
     * 
     * Loads a sequence that reads the "INTAN" string from ROM
     * to test cable delays and optimize phase selection.
     * 
     * @return true if command succeeded
     */
    bool loadCableTestSequence();
    
    /**
     * @brief Run automated cable length test
     * 
     * Runs a full automated test sequence:
     * 1. Initialize chips
     * 2. Test all 16 phase delay settings
     * 3. Capture packets for analysis
     * 
     * Results can be examined via the data callback to find
     * optimal phase settings.
     * 
     * @return true if test started successfully
     */
    bool runFullCableTest();
    
    // ========================================================================
    // CALLBACKS
    // ========================================================================
    
    /**
     * @brief Register callback for data packets
     * 
     * The callback is invoked from the UDP listener thread whenever
     * a valid packet is received. The callback should be fast to avoid
     * blocking packet reception.
     * 
     * @param callback Function to call with each packet
     */
    void setDataCallback(DataCallback callback);
    
    /**
     * @brief Register callback for error notifications
     * 
     * Called when errors occur (connection loss, packet errors, etc.)
     * 
     * @param callback Function to call with error messages
     */
    void setErrorCallback(ErrorCallback callback);
    
    // ========================================================================
    // UTILITY
    // ========================================================================
    
    /**
     * @brief Get the expected packet size in words
     * 
     * Packet size depends on channel enable setting:
     * - Header: 10 words (magic + timestamp + digital + 2x analog)
     * - Data: variable based on enabled channels
     * 
     * @param channelMask Channel enable mask (0x0-0xF)
     * @return Expected packet size in 32-bit words
     */
    static uint32_t calculatePacketSize(uint8_t channelMask);
    
    /**
     * @brief Get device IP address
     */
    std::string getDeviceIp() const;
    
    /**
     * @brief Get TCP port
     */
    uint16_t getTcpPort() const;
    
    /**
     * @brief Get UDP port
     */
    uint16_t getUdpPort() const;

private:
    // Forward declaration of implementation class
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};

#endif // INTAN_INTERFACE_H
