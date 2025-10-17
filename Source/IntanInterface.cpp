#include "IntanInterface.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <vector>
#include <queue>

// Platform-specific networking
#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    using socklen_t = int;
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
    #define closesocket close
    using SOCKET = int;
#endif

// ============================================================================
// PROTOCOL CONSTANTS
// ============================================================================

namespace {
    constexpr uint32_t CMD_MAGIC = 0xDEADBEEF;
    constexpr uint32_t PACKET_MAGIC_LOW = 0xDEADBEEF;
    constexpr uint32_t PACKET_MAGIC_HIGH = 0xCAFEBABE;
    constexpr size_t CMD_PACKET_SIZE = 20;
    constexpr size_t ACK_PACKET_SIZE = 3;
    constexpr size_t STATUS_RESPONSE_SIZE = 86;
    
    // Command IDs
    enum CommandId : uint32_t {
        CMD_START = 0x01,
        CMD_STOP = 0x02,
        CMD_RESET_TIMESTAMP = 0x03,
        CMD_SET_LOOP_COUNT = 0x10,
        CMD_SET_PHASE = 0x11,
        CMD_SET_DEBUG_MODE = 0x12,
        CMD_SET_CHANNEL_ENABLE = 0x13,
        CMD_LOAD_CONVERT = 0x20,
        CMD_LOAD_INIT = 0x21,
        CMD_LOAD_CABLE_TEST = 0x22,
        CMD_FULL_CABLE_TEST = 0x30,
        CMD_GET_STATUS = 0x40,
        CMD_SET_UDP_DEST = 0x50
    };
    
    // ACK status codes
    constexpr uint8_t ACK_SUCCESS = 0x06;
    constexpr uint8_t ACK_ERROR = 0x15;
    
    // Packet header
    constexpr size_t PACKET_HEADER_WORDS = 4;
    
    // Auto-detection constants
    constexpr uint16_t INTAN_PATTERN[] = {0x0049, 0x004E, 0x0054, 0x0041, 0x004E};  // 'I','N','T','A','N'
    constexpr size_t INTAN_PATTERN_SIZE = 5;
    
    constexpr uint16_t CHIP_ID_RHD2164 = 4;  // 64-channel with DDR
    constexpr uint16_t CHIP_ID_RHD2132 = 1;  // 32-channel without DDR
    constexpr uint16_t CHIP_ID_RHD2216 = 2;  // 16-channel
    
    constexpr uint16_t MISO_REG_DDR = 0x35;     // Regular word when DDR available
    constexpr uint16_t MISO_DDR_DDR = 0x3A;     // DDR word when DDR available
    constexpr uint16_t MISO_NO_DDR = 0x00;      // When no DDR
    
    constexpr size_t CABLE_TEST_PACKET_SIZE_WORDS = 74;
    constexpr double DETECTION_THRESHOLD = 60.0;
    constexpr int NUM_PHASES_TO_TEST = 16;
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

namespace {
    std::string ipToString(uint32_t ip) {
        struct in_addr addr;
        addr.s_addr = ip;
        return inet_ntoa(addr);
    }
    
    uint32_t stringToIp(const std::string& ipStr) {
        struct in_addr addr;
        if (inet_pton(AF_INET, ipStr.c_str(), &addr) == 1) {
            return addr.s_addr;
        }
        return 0;
    }
    
    void packU32LE(uint8_t* buf, uint32_t val) {
        buf[0] = val & 0xFF;
        buf[1] = (val >> 8) & 0xFF;
        buf[2] = (val >> 16) & 0xFF;
        buf[3] = (val >> 24) & 0xFF;
    }
    
    uint32_t unpackU32LE(const uint8_t* buf) {
        return buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
    }
    
    uint16_t unpackU16BE(const uint8_t* buf) {
        return (buf[0] << 8) | buf[1];
    }
}

// ============================================================================
// IMPLEMENTATION CLASS
// ============================================================================

class IntanInterface::Impl {
public:
    Impl(const std::string& deviceIp, uint16_t tcpPort, uint16_t udpPort)
        : deviceIp_(deviceIp)
        , tcpPort_(tcpPort)
        , udpPort_(udpPort)
        , tcpSocket_(INVALID_SOCKET)
        , udpSocket_(INVALID_SOCKET)
        , running_(false)
        , connected_(false)
        , currentChannelEnable_(0x0F)
        , expectedPacketSizeWords_(74)
        , totalPackets_(0)
        , totalErrors_(0)
        , magicErrors_(0)
        , timestampErrors_(0)
        , sizeErrors_(0)
        , lastTimestamp_(0)
        , lastStatsTime_(std::chrono::steady_clock::now())
        , lastPacketCount_(0)
        , captureForDetection_(false)
        , maxDetectionQueueSize_(20)
    {
#ifdef _WIN32
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            throw std::runtime_error("Failed to initialize Winsock");
        }
#endif
        
        // Connect TCP
        if (!connectTcp()) {
            throw std::runtime_error("Failed to establish TCP connection to " + deviceIp_);
        }
        
        // Auto-configure UDP destination
        autoConfigureUdp();
        
        // Start UDP listener thread
        running_ = true;
        udpThread_ = std::thread(&Impl::udpListenerThread, this);
        
        // Get initial status to sync channel enable
        DeviceStatus status;
        if (getStatusInternal(status)) {
            updateChannelEnable(status.channelEnable);
        }
    }
    
    ~Impl() {
        shutdown();
    }
    
    void shutdown() {
        running_ = false;
        
        if (udpThread_.joinable()) {
            udpThread_.join();
        }
        
        if (tcpSocket_ != INVALID_SOCKET) {
            closesocket(tcpSocket_);
            tcpSocket_ = INVALID_SOCKET;
        }
        
        if (udpSocket_ != INVALID_SOCKET) {
            closesocket(udpSocket_);
            udpSocket_ = INVALID_SOCKET;
        }
        
#ifdef _WIN32
        WSACleanup();
#endif
    }
    
    bool isConnected() const {
        return connected_.load();
    }
    
    bool isReady() const {
        if (!connected_.load() || !running_.load()) {
            return false;
        }
        
        // Check device status
        DeviceStatus status;
        if (!getStatusInternal(status)) {
            return false;
        }
        
        // Device should not be transmitting
        return !status.transmissionActive;
    }
    
    bool sendCommand(CommandId cmdId, uint32_t param1 = 0, uint32_t param2 = 0,
                     uint8_t* responseData = nullptr, size_t* responseLen = nullptr) {
        std::lock_guard<std::mutex> lock(tcpMutex_);
        
        if (tcpSocket_ == INVALID_SOCKET) {
            return false;
        }
        
        // Build command packet
        uint8_t cmd[CMD_PACKET_SIZE];
        static uint32_t ackId = 1;
        
        packU32LE(&cmd[0], CMD_MAGIC);
        packU32LE(&cmd[4], cmdId);
        packU32LE(&cmd[8], ackId);
        packU32LE(&cmd[12], param1);
        packU32LE(&cmd[16], param2);
        
        // Send command
        if (send(tcpSocket_, reinterpret_cast<char*>(cmd), CMD_PACKET_SIZE, 0) != CMD_PACKET_SIZE) {
            reportError("Failed to send TCP command");
            return false;
        }
        
        // Receive response
        uint8_t response[5];
        int received = recv(tcpSocket_, reinterpret_cast<char*>(response), 5, 0);
        
        if (received < 3) {
            reportError("Failed to receive command response");
            return false;
        }
        
        // Validate ACK
        uint16_t recvAckId = unpackU16BE(&response[0]);
        uint8_t status = response[2];
        
        if (recvAckId != ackId) {
            reportError("ACK ID mismatch");
            return false;
        }
        
        if (status != ACK_SUCCESS) {
            reportError("Command failed on device");
            return false;
        }
        
        ackId++;
        
        // Check for data response
        if (received == 5 && responseData && responseLen) {
            uint16_t dataLen = unpackU16BE(&response[3]);
            if (dataLen > 0 && dataLen <= *responseLen) {
                int dataReceived = recv(tcpSocket_, reinterpret_cast<char*>(responseData), dataLen, 0);
                if (dataReceived == dataLen) {
                    *responseLen = dataLen;
                    return true;
                }
            }
        }
        
        return true;
    }
    
    bool getStatusInternal(DeviceStatus& status) const {
        // Need to cast away const for mutex, but logically this is a const operation
        auto* self = const_cast<Impl*>(this);
        
        uint8_t data[STATUS_RESPONSE_SIZE];
        size_t dataLen = STATUS_RESPONSE_SIZE;
        
        if (!self->sendCommand(CMD_GET_STATUS, 0, 0, data, &dataLen)) {
            return false;
        }
        
        if (dataLen != STATUS_RESPONSE_SIZE) {
            return false;
        }
        
        // Parse response structure
        const uint8_t* p = data;
        
        status.protocolVersion = unpackU32LE(p) & 0xFFFF; p += 2;
        status.deviceType = unpackU32LE(p) & 0xFFFF; p += 2;
        status.firmwareVersion = unpackU32LE(p); p += 4;
        
        status.timestamp = static_cast<uint64_t>(unpackU32LE(p)) | 
                          (static_cast<uint64_t>(unpackU32LE(p + 4)) << 32); p += 8;
        status.packetsSent = unpackU32LE(p); p += 4;
        status.bramWriteAddr = unpackU32LE(p); p += 4;
        status.fifoCount = unpackU32LE(p) & 0xFFFF; p += 2;
        status.stateCounter = *p++; 
        status.cycleCounter = *p++;
        uint8_t flagsPl = *p++;
        p++; // reserved
        
        status.transmissionActive = (flagsPl & 0x01) != 0;
        status.loopLimitReached = (flagsPl & 0x02) != 0;
        
        status.packetsReceived = unpackU32LE(p); p += 4;
        status.errorCount = unpackU32LE(p); p += 4;
        status.udpPacketsSent = unpackU32LE(p); p += 4;
        status.udpSendErrors = unpackU32LE(p); p += 4;
        status.psReadAddr = unpackU32LE(p); p += 4;
        status.packetSizeWords = unpackU32LE(p); p += 4;
        uint8_t flagsPs = *p++;
        p += 3; // reserved
        
        status.streamEnabled = (flagsPs & 0x01) != 0;
        
        status.loopCount = unpackU32LE(p); p += 4;
        status.phase0 = *p++;
        status.phase1 = *p++;
        status.channelEnable = *p++;
        status.debugMode = *p++;
        p += 8; // reserved
        
        uint32_t udpDestIp = unpackU32LE(p); p += 4;
        status.udpDestIp = ipToString(udpDestIp);
        status.udpDestPort = unpackU32LE(p) & 0xFFFF; p += 2;
        p += 2; // format
        status.udpBytesSent = unpackU32LE(p);
        
        return true;
    }
    
    void setDataCallback(DataCallback callback) {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        dataCallback_ = callback;
    }
    
    void setErrorCallback(ErrorCallback callback) {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        errorCallback_ = callback;
    }
    
    void getReceptionStats(ReceptionStats& stats) const {
        std::lock_guard<std::mutex> lock(statsMutex_);
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - startTime_).count();
        auto recentElapsed = std::chrono::duration<double>(now - lastStatsTime_).count();
        
        stats.totalPackets = totalPackets_;
        stats.totalErrors = totalErrors_;
        stats.magicErrors = magicErrors_;
        stats.timestampErrors = timestampErrors_;
        stats.sizeErrors = sizeErrors_;
        stats.lastTimestamp = lastTimestamp_;
        
        stats.averageRate = (elapsed > 0) ? (totalPackets_ / elapsed) : 0.0;
        stats.instantRate = (recentElapsed > 0) ? 
            ((totalPackets_ - lastPacketCount_) / recentElapsed) : 0.0;
        
        size_t packetBytes = expectedPacketSizeWords_ * 4;
        stats.dataRateMbps = stats.averageRate * packetBytes * 8.0 / 1e6;
    }
    
    void updateChannelEnable(uint8_t channelEnable) {
        currentChannelEnable_ = channelEnable;
        expectedPacketSizeWords_ = IntanInterface::calculatePacketSize(channelEnable);
    }
    
    std::string getDeviceIp() const { return deviceIp_; }
    uint16_t getTcpPort() const { return tcpPort_; }
    uint16_t getUdpPort() const { return udpPort_; }
    
    // ========================================================================
    // AUTO-DETECTION METHODS
    // ========================================================================
    
    bool runAutoDetection(AutoDetectionResult& result, bool verbose) {
        // Initialize result
        result.success = false;
        result.chipsDetected = false;
        result.bestPhase0 = 0;
        result.bestPhase1 = 0;
        result.optimalChannelMask = 0;
        result.cipo0Detected = false;
        result.cipo1Detected = false;
        result.cipo0HasDdr = false;
        result.cipo1HasDdr = false;
        result.cipo0ChipType = ChipType::NONE;
        result.cipo1ChipType = ChipType::NONE;
        result.cipo0Score = 0.0;
        result.cipo1Score = 0.0;
        result.allPhaseResults.clear();
        
        if (verbose) {
            std::cout << "[Detection] Starting automated chip detection..." << std::endl;
        }
        
        // Check device is ready (not transmitting)
        DeviceStatus status;
        if (!getStatusInternal(status)) {
            if (verbose) {
                std::cout << "[Detection] ERROR: Cannot read device status" << std::endl;
            }
            return false;
        }
        
        if (status.transmissionActive) {
            if (verbose) {
                std::cout << "[Detection] ERROR: Device is transmitting. Stop acquisition first." << std::endl;
            }
            return false;
        }
        
        // Initialize chips and load cable test sequence
        if (!initializeForDetection(verbose)) {
            return false;
        }
        
        // Test all phases
        double bestScore = -1000.0;
        
        for (int phase = 0; phase < NUM_PHASES_TO_TEST; ++phase) {
            if (verbose) {
                std::cout << "[Detection] Testing phase " << phase << "..." << std::endl;
            }
            
            PhaseTestResult phaseResult = testPhase(phase, verbose);
            result.allPhaseResults.push_back(phaseResult);
            
            // Check if either channel is valid (score > threshold)
            bool cipo0Valid = phaseResult.cipo0Score > DETECTION_THRESHOLD;
            bool cipo1Valid = phaseResult.cipo1Score > DETECTION_THRESHOLD;
            
            if (cipo0Valid || cipo1Valid) {
                double totalScore = phaseResult.cipo0Score + phaseResult.cipo1Score;
                
                if (verbose && totalScore > 0) {
                    std::cout << "  Phase " << phase 
                             << ": CIPO0=" << phaseResult.cipo0Score
                             << ", CIPO1=" << phaseResult.cipo1Score << std::endl;
                }
                
                if (totalScore > bestScore) {
                    bestScore = totalScore;
                    result.bestPhase0 = phase;
                    result.bestPhase1 = phase;  // Use same phase for both
                    result.cipo0Detected = cipo0Valid;
                    result.cipo1Detected = cipo1Valid;
                    result.cipo0HasDdr = phaseResult.cipo0HasDdr;
                    result.cipo1HasDdr = phaseResult.cipo1HasDdr;
                    result.cipo0ChipType = phaseResult.cipo0ChipType;
                    result.cipo1ChipType = phaseResult.cipo1ChipType;
                    result.cipo0Score = phaseResult.cipo0Score;
                    result.cipo1Score = phaseResult.cipo1Score;
                }
            }
        }
        
        // Calculate optimal channel mask
        result.chipsDetected = result.cipo0Detected || result.cipo1Detected;
        result.success = result.chipsDetected;
        
        if (result.success) {
            result.optimalChannelMask = 0;
            
            if (result.cipo0Detected) {
                result.optimalChannelMask |= 0x01;  // CIPO0 regular
                if (result.cipo0HasDdr) {
                    result.optimalChannelMask |= 0x02;  // CIPO0 DDR
                }
            }
            
            if (result.cipo1Detected) {
                result.optimalChannelMask |= 0x04;  // CIPO1 regular
                if (result.cipo1HasDdr) {
                    result.optimalChannelMask |= 0x08;  // CIPO1 DDR
                }
            }
            
            if (verbose) {
                std::cout << "[Detection] Complete! Best phase: " 
                         << static_cast<int>(result.bestPhase0) << std::endl;
                std::cout << "[Detection] " << result.getChannelSummary() << std::endl;
            }
        } else {
            if (verbose) {
                std::cout << "[Detection] No chips detected" << std::endl;
            }
        }
        
        return true;
    }
    
    private:
    bool connectTcp() {
        tcpSocket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (tcpSocket_ == INVALID_SOCKET) {
            return false;
        }
        
        struct sockaddr_in serverAddr;
        std::memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(tcpPort_);
        
        if (inet_pton(AF_INET, deviceIp_.c_str(), &serverAddr.sin_addr) != 1) {
            closesocket(tcpSocket_);
            tcpSocket_ = INVALID_SOCKET;
            return false;
        }
        
        if (connect(tcpSocket_, reinterpret_cast<struct sockaddr*>(&serverAddr),
                   sizeof(serverAddr)) == SOCKET_ERROR) {
            closesocket(tcpSocket_);
            tcpSocket_ = INVALID_SOCKET;
            return false;
        }
        
        connected_ = true;
        return true;
    }
    
    void autoConfigureUdp() {
        // Get local IP that can reach device
        SOCKET tempSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (tempSock == INVALID_SOCKET) {
            return;
        }
        
        struct sockaddr_in serverAddr;
        std::memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(tcpPort_);
        inet_pton(AF_INET, deviceIp_.c_str(), &serverAddr.sin_addr);
        
        if (connect(tempSock, reinterpret_cast<struct sockaddr*>(&serverAddr),
                   sizeof(serverAddr)) == 0) {
            struct sockaddr_in localAddr;
            socklen_t addrLen = sizeof(localAddr);
            if (getsockname(tempSock, reinterpret_cast<struct sockaddr*>(&localAddr),
                           &addrLen) == 0) {
                char localIp[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &localAddr.sin_addr, localIp, sizeof(localIp));
                
                // Configure device to send UDP to us
                uint32_t ipInt = stringToIp(localIp);
                sendCommand(CMD_SET_UDP_DEST, ntohl(ipInt), udpPort_);
            }
        }
        
        closesocket(tempSock);
    }
    
    void udpListenerThread() {
        // Create UDP socket
        udpSocket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (udpSocket_ == INVALID_SOCKET) {
            reportError("Failed to create UDP socket");
            return;
        }
        
        // Bind to UDP port
        struct sockaddr_in bindAddr;
        std::memset(&bindAddr, 0, sizeof(bindAddr));
        bindAddr.sin_family = AF_INET;
        bindAddr.sin_addr.s_addr = INADDR_ANY;
        bindAddr.sin_port = htons(udpPort_);
        
        if (bind(udpSocket_, reinterpret_cast<struct sockaddr*>(&bindAddr),
                sizeof(bindAddr)) == SOCKET_ERROR) {
            reportError("Failed to bind UDP socket");
            closesocket(udpSocket_);
            udpSocket_ = INVALID_SOCKET;
            return;
        }
        
        // Set timeout for clean shutdown
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(udpSocket_, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<char*>(&tv), sizeof(tv));
        
        std::vector<uint8_t> buffer(4096);
        startTime_ = std::chrono::steady_clock::now();
        
        while (running_.load()) {
            struct sockaddr_in senderAddr;
            socklen_t senderLen = sizeof(senderAddr);
            
            int received = recvfrom(udpSocket_, reinterpret_cast<char*>(buffer.data()),
                                   buffer.size(), 0,
                                   reinterpret_cast<struct sockaddr*>(&senderAddr),
                                   &senderLen);
            
            if (received > 0) {
                processUdpData(buffer.data(), received);
            }
        }
    }
    
    void processUdpData(const uint8_t* data, size_t len) {
        size_t expectedBytes = expectedPacketSizeWords_ * 4;
        
        // Process multiple packets if present
        for (size_t offset = 0; offset + expectedBytes <= len; offset += expectedBytes) {
            validateAndDispatchPacket(data + offset, expectedBytes);
        }
    }
    
    void validateAndDispatchPacket(const uint8_t* data, size_t len) {
        std::lock_guard<std::mutex> lock(statsMutex_);
        
        totalPackets_++;
        
        size_t expectedBytes = expectedPacketSizeWords_ * 4;
        
        // Check size
        if (len != expectedBytes) {
            sizeErrors_++;
            totalErrors_++;
            return;
        }
        
        // Unpack as 32-bit words
        std::vector<uint32_t> words(expectedPacketSizeWords_);
        for (size_t i = 0; i < expectedPacketSizeWords_; ++i) {
            words[i] = unpackU32LE(&data[i * 4]);
        }
        
        // Validate magic
        uint64_t magic = (static_cast<uint64_t>(words[1]) << 32) | words[0];
        uint64_t expectedMagic = (static_cast<uint64_t>(PACKET_MAGIC_HIGH) << 32) | 
                                PACKET_MAGIC_LOW;
        
        if (magic != expectedMagic) {
            magicErrors_++;
            totalErrors_++;
            return;
        }
        
        // Extract timestamp
        uint64_t timestamp = (static_cast<uint64_t>(words[3]) << 32) | words[2];
        
        // Check for timestamp gaps
        if (lastTimestamp_ != 0 && timestamp != lastTimestamp_ + 1) {
            timestampErrors_++;
            totalErrors_++;
        }
        
        lastTimestamp_ = timestamp;
        
        // Update stats periodically
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - lastStatsTime_).count() >= 5.0) {
            lastStatsTime_ = now;
            lastPacketCount_ = totalPackets_;
        }
        
        // If capturing for detection, save packet
        if (captureForDetection_.load()) {
            std::lock_guard<std::mutex> detLock(detectionMutex_);
            if (detectionPacketQueue_.size() < maxDetectionQueueSize_) {
                detectionPacketQueue_.push(words);
            }
        }

        // Dispatch to callback (without holding stats lock)
        DataCallback callback;
        {
            std::lock_guard<std::mutex> cbLock(callbackMutex_);
            callback = dataCallback_;
        }
        
        if (callback) {
            // Don't hold statsMutex while calling user callback
            statsMutex_.unlock();
            callback(words.data(), words.size(), timestamp);
            statsMutex_.lock();
        }
    }
    
    void reportError(const std::string& message) const {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        if (errorCallback_) {
            errorCallback_(message);
        }
    }
    
    // ========================================================================
    // AUTO-DETECTION HELPER METHODS
    // ========================================================================
    
    bool initializeForDetection(bool verbose) {
        // Set loop count to 1 for single-packet acquisitions
        if (!sendCommand(CMD_SET_LOOP_COUNT, 1)) {
            if (verbose) {
                std::cout << "[Detection] ERROR: Failed to set loop count" << std::endl;
            }
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Set channel enable to all channels for cable test
        if (!sendCommand(CMD_SET_CHANNEL_ENABLE, 0x0F)) {
            if (verbose) {
                std::cout << "[Detection] ERROR: Failed to set channel enable" << std::endl;
            }
            return false;
        }
        updateChannelEnable(0x0F);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Load and run initialization sequence
        if (verbose) {
            std::cout << "[Detection] Running initialization sequence..." << std::endl;
        }
        
        if (!sendCommand(CMD_LOAD_INIT)) {
            if (verbose) {
                std::cout << "[Detection] ERROR: Failed to load init sequence" << std::endl;
            }
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Run initialization (acquire 1 packet)
        if (!sendCommand(CMD_START)) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!sendCommand(CMD_STOP)) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Load cable test sequence
        if (verbose) {
            std::cout << "[Detection] Loading cable test sequence..." << std::endl;
        }
        
        if (!sendCommand(CMD_LOAD_CABLE_TEST)) {
            if (verbose) {
                std::cout << "[Detection] ERROR: Failed to load cable test sequence" << std::endl;
            }
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        return true;
    }
    
    PhaseTestResult testPhase(uint8_t phase, bool verbose) {
        PhaseTestResult result;
        result.phase = phase;
        result.cipo0Score = 0.0;
        result.cipo1Score = 0.0;
        result.cipo0HasDdr = false;
        result.cipo1HasDdr = false;
        result.cipo0ChipType = ChipType::NONE;
        result.cipo1ChipType = ChipType::NONE;
        
        // Set phase
        if (!sendCommand(CMD_SET_PHASE, phase, phase)) {
            return result;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Start capture
        startDetectionCapture();
        
        // Acquire packet
        if (!sendCommand(CMD_START)) {
            stopDetectionCapture();
            return result;
        }
        
        // Wait and capture packet
        std::vector<uint32_t> packet;
        bool captured = capturePacketForDetection(packet, 2.0);
        
        // Stop acquisition
        sendCommand(CMD_STOP);
        stopDetectionCapture();
        
        if (!captured) {
            if (verbose) {
                std::cout << "  Phase " << phase << ": No packet received" << std::endl;
            }
            return result;
        }
        
        // Score both channels
        auto [cipo0Score, cipo0Type] = scoreChannel(packet, 0, verbose);
        auto [cipo1Score, cipo1Type] = scoreChannel(packet, 1, verbose);
        
        result.cipo0Score = cipo0Score;
        result.cipo1Score = cipo1Score;
        result.cipo0ChipType = cipo0Type;
        result.cipo1ChipType = cipo1Type;
        result.cipo0HasDdr = (cipo0Type == ChipType::RHD2164);
        result.cipo1HasDdr = (cipo1Type == ChipType::RHD2164);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        return result;
    }
    
    std::pair<double, ChipType> scoreChannel(
        const std::vector<uint32_t>& packet, int channel, bool verbose) {
        
        if (packet.size() < CABLE_TEST_PACKET_SIZE_WORDS) {
            return {0.0, ChipType::NONE};
        }
        
        double score = 0.0;
        ChipType chipType = ChipType::NONE;
        
        // Extract data words (skip 4-word header)
        std::vector<uint32_t> dataWords(packet.begin() + 4, packet.end());
        
        if (dataWords.size() < 35) {
            return {0.0, ChipType::NONE};
        }
        
        // Extract this channel's words (every other word starting at channel offset)
        std::vector<uint32_t> channelWords;
        for (size_t i = channel; i < dataWords.size() && i < 70; i += 2) {
            channelWords.push_back(dataWords[i]);
        }
        
        if (channelWords.size() < 9) {
            return {0.0, ChipType::NONE};
        }
        
        // Extract regular and DDR streams from 32-bit words
        std::vector<uint16_t> regular, ddr;
        for (uint32_t word : channelWords) {
            regular.push_back(word & 0xFFFF);           // Lower 16 bits
            ddr.push_back((word >> 16) & 0xFFFF);       // Upper 16 bits
        }
        
        // Score INTAN pattern (indices 2-6 due to 2-cycle pipeline delay)
        std::vector<uint16_t> intanFound;
        for (size_t i = 0; i < INTAN_PATTERN_SIZE; ++i) {
            size_t idx = i + 2;  // Pipeline delay
            if (idx < regular.size()) {
                intanFound.push_back(regular[idx]);
                if (regular[idx] == INTAN_PATTERN[i]) {
                    score += 10.0;
                }
            }
        }
        
        // Check chip ID at index 7
        if (regular.size() > 7 && ddr.size() > 7) {
            uint16_t chipIdReg = regular[7];
            uint16_t chipIdDdr = ddr[7];
            
            // Both regular and DDR should read same chip ID
            if (chipIdReg == CHIP_ID_RHD2164 && chipIdDdr == CHIP_ID_RHD2164) {
                chipType = ChipType::RHD2164;
                score += 10.0;
            } else if (chipIdReg == CHIP_ID_RHD2132) {
                chipType = ChipType::RHD2132;
                score += 10.0;
            } else if (chipIdReg == CHIP_ID_RHD2216) {
                chipType = ChipType::RHD2132;  // Treat as RHD2132 (no DDR)
                score += 10.0;
            }
        }
        
        // Check MISO register at index 8
        if (regular.size() > 8 && ddr.size() > 8) {
            uint16_t misoReg = regular[8];
            uint16_t misoDdr = ddr[8];
            
            if (chipType == ChipType::RHD2164) {
                // RHD2164 should have specific MISO values
                if (misoReg == MISO_REG_DDR && misoDdr == MISO_DDR_DDR) {
                    score += 10.0;
                }
            } else if (chipType == ChipType::RHD2132) {
                // RHD2132 should have zero MISO
                if (misoReg == MISO_NO_DDR) {
                    score += 10.0;
                }
            }
        }
        
        // Verbose output
        if (verbose && score > DETECTION_THRESHOLD) {
            std::string patternStr;
            for (uint16_t val : intanFound) {
                if (val >= 0x20 && val <= 0x7E) {
                    patternStr += static_cast<char>(val);
                } else {
                    patternStr += '?';
                }
            }
            
            std::string chipStr = (chipType == ChipType::RHD2164) ? "RHD2164" : 
                                 (chipType == ChipType::RHD2132) ? "RHD2132" : "Unknown";
            
            std::cout << "    CIPO" << channel << ": '" << patternStr 
                     << "' (" << chipStr << ")" << std::endl;
        }
        
        return {score, chipType};
    }
    
    void startDetectionCapture() {
        std::lock_guard<std::mutex> lock(detectionMutex_);
        // Clear queue
        while (!detectionPacketQueue_.empty()) {
            detectionPacketQueue_.pop();
        }
        captureForDetection_ = true;
    }
    
    void stopDetectionCapture() {
        captureForDetection_ = false;
    }
    
    bool capturePacketForDetection(
        std::vector<uint32_t>& packet, double timeoutSec) {
        
        auto deadline = std::chrono::steady_clock::now() + 
                       std::chrono::milliseconds(static_cast<int>(timeoutSec * 1000));
        
        while (std::chrono::steady_clock::now() < deadline) {
            {
                std::lock_guard<std::mutex> lock(detectionMutex_);
                if (!detectionPacketQueue_.empty()) {
                    packet = detectionPacketQueue_.front();
                    detectionPacketQueue_.pop();
                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        return false;
    }
    
    // Configuration
    std::string deviceIp_;
    uint16_t tcpPort_;
    uint16_t udpPort_;
    
    // Sockets
    SOCKET tcpSocket_;
    SOCKET udpSocket_;
    
    // Threading
    std::thread udpThread_;
    std::atomic<bool> running_;
    std::atomic<bool> connected_;
    mutable std::mutex tcpMutex_;
    mutable std::mutex callbackMutex_;
    mutable std::mutex statsMutex_;
    
    // State
    uint8_t currentChannelEnable_;
    size_t expectedPacketSizeWords_;
    
    // Statistics
    uint64_t totalPackets_;
    uint64_t totalErrors_;
    uint64_t magicErrors_;
    uint64_t timestampErrors_;
    uint64_t sizeErrors_;
    uint64_t lastTimestamp_;
    std::chrono::steady_clock::time_point startTime_;
    std::chrono::steady_clock::time_point lastStatsTime_;
    uint64_t lastPacketCount_;
    
    // Callbacks
    DataCallback dataCallback_;
    ErrorCallback errorCallback_;
    
    // Auto-detection
    std::mutex detectionMutex_;
    std::queue<std::vector<uint32_t>> detectionPacketQueue_;
    std::atomic<bool> captureForDetection_;
    size_t maxDetectionQueueSize_;
};

// ============================================================================
// PUBLIC INTERFACE IMPLEMENTATION
// ============================================================================

IntanInterface::IntanInterface(const std::string& deviceIp, uint16_t tcpPort, uint16_t udpPort)
    : pImpl_(std::make_unique<Impl>(deviceIp, tcpPort, udpPort))
{
}

IntanInterface::~IntanInterface() = default;

bool IntanInterface::foundInputSource() const {
    return pImpl_->isConnected();
}

bool IntanInterface::isReady() const {
    return pImpl_->isReady();
}

bool IntanInterface::getStatus(DeviceStatus& status) const {
    return pImpl_->getStatusInternal(status);
}

void IntanInterface::getReceptionStats(ReceptionStats& stats) const {
    pImpl_->getReceptionStats(stats);
}

bool IntanInterface::startAcquisition() {
    return pImpl_->sendCommand(CMD_START);
}

bool IntanInterface::stopAcquisition() {
    return pImpl_->sendCommand(CMD_STOP);
}

bool IntanInterface::resetTimestamp() {
    return pImpl_->sendCommand(CMD_RESET_TIMESTAMP);
}

bool IntanInterface::setLoopCount(uint32_t count) {
    return pImpl_->sendCommand(CMD_SET_LOOP_COUNT, count);
}

bool IntanInterface::setPhaseSelect(uint8_t phase0, uint8_t phase1) {
    return pImpl_->sendCommand(CMD_SET_PHASE, phase0, phase1);
}

bool IntanInterface::setDebugMode(bool enable) {
    return pImpl_->sendCommand(CMD_SET_DEBUG_MODE, enable ? 1 : 0);
}

bool IntanInterface::setChannelEnable(uint8_t channelMask) {
    if (pImpl_->sendCommand(CMD_SET_CHANNEL_ENABLE, channelMask)) {
        pImpl_->updateChannelEnable(channelMask);
        return true;
    }
    return false;
}

bool IntanInterface::setUdpDestination(const std::string& ipAddress, uint16_t port) {
    uint32_t ipInt = stringToIp(ipAddress);
    if (ipInt == 0) {
        return false;
    }
    return pImpl_->sendCommand(CMD_SET_UDP_DEST, ntohl(ipInt), port);
}

bool IntanInterface::loadConvertSequence() {
    return pImpl_->sendCommand(CMD_LOAD_CONVERT);
}

bool IntanInterface::loadInitSequence() {
    return pImpl_->sendCommand(CMD_LOAD_INIT);
}

bool IntanInterface::loadCableTestSequence() {
    return pImpl_->sendCommand(CMD_LOAD_CABLE_TEST);
}

bool IntanInterface::runFullCableTest() {
    return pImpl_->sendCommand(CMD_FULL_CABLE_TEST);
}

void IntanInterface::setDataCallback(DataCallback callback) {
    pImpl_->setDataCallback(callback);
}

void IntanInterface::setErrorCallback(ErrorCallback callback) {
    pImpl_->setErrorCallback(callback);
}

bool IntanInterface::runAutoDetection(AutoDetectionResult& result, bool verbose) {
    return pImpl_->runAutoDetection(result, verbose);
}

bool IntanInterface::applyDetectionConfig(const AutoDetectionResult& result) {
    if (!result.success) {
        return false;
    }
    
    // Set phase delays
    if (!setPhaseSelect(result.bestPhase0, result.bestPhase1)) {
        return false;
    }
    
    // Set channel enable mask
    if (!setChannelEnable(result.optimalChannelMask)) {
        return false;
    }
    
    // Load normal conversion sequence (not cable test)
    if (!loadConvertSequence()) {
        return false;
    }
    
    return true;
}

uint32_t IntanInterface::calculatePacketSize(uint8_t channelMask) {
    int numChannels = 0;
    for (int i = 0; i < 4; ++i) {
        if (channelMask & (1 << i)) {
            numChannels++;
        }
    }
    
    if (numChannels == 0) {
        return 74; // Default to maximum
    }
    
    uint32_t total16bitWords = 35 * numChannels;
    uint32_t data32bitWords = (total16bitWords + 1) / 2;
    
    return PACKET_HEADER_WORDS + data32bitWords;
}

std::string IntanInterface::getDeviceIp() const {
    return pImpl_->getDeviceIp();
}

uint16_t IntanInterface::getTcpPort() const {
    return pImpl_->getTcpPort();
}

uint16_t IntanInterface::getUdpPort() const {
    return pImpl_->getUdpPort();
}

// ============================================================================
// HELPER METHODS FOR STATUS
// ============================================================================

std::string IntanInterface::DeviceStatus::getFirmwareVersionString() const {
    std::ostringstream oss;
    oss << ((firmwareVersion >> 24) & 0xFF) << "."
        << ((firmwareVersion >> 16) & 0xFF) << "."
        << ((firmwareVersion >> 8) & 0xFF) << "."
        << (firmwareVersion & 0xFF);
    return oss.str();
}

std::string IntanInterface::DeviceStatus::getChannelEnableString() const {
    std::ostringstream oss;
    bool first = true;
    
    if (channelEnable & CHANNEL_CIPO0_REGULAR) {
        if (!first) oss << ", ";
        oss << "CIPO0_REG";
        first = false;
    }
    if (channelEnable & CHANNEL_CIPO0_DDR) {
        if (!first) oss << ", ";
        oss << "CIPO0_DDR";
        first = false;
    }
    if (channelEnable & CHANNEL_CIPO1_REGULAR) {
        if (!first) oss << ", ";
        oss << "CIPO1_REG";
        first = false;
    }
    if (channelEnable & CHANNEL_CIPO1_DDR) {
        if (!first) oss << ", ";
        oss << "CIPO1_DDR";
        first = false;
    }
    
    return first ? "NONE" : oss.str();
}

// ============================================================================
// HELPER METHODS FOR AUTO-DETECTION RESULT
// ============================================================================

std::string IntanInterface::AutoDetectionResult::chipTypeToString(ChipType type) {
    switch (type) {
        case ChipType::RHD2132: return "RHD2132";
        case ChipType::RHD2164: return "RHD2164";
        case ChipType::NONE:
        default: return "None";
    }
}

std::string IntanInterface::AutoDetectionResult::getSummary() const {
    if (!success) {
        return "Detection failed. Check connections and try manual configuration.";
    }
    
    if (!chipsDetected) {
        return "No Intan chips detected. Verify:\n"
               "  - SPI cable connections\n"
               "  - Chip power supply\n"
               "  - Cable integrity";
    }
    
    std::ostringstream oss;
    oss << "Chips detected!\n";
    oss << "  Best Phase: " << static_cast<int>(bestPhase0);
    if (bestPhase0 != bestPhase1) {
        oss << " (Phase0), " << static_cast<int>(bestPhase1) << " (Phase1)";
    }
    oss << "\n";
    oss << "  Channel Mask: 0x" << std::hex << std::uppercase 
        << static_cast<int>(optimalChannelMask) << std::dec << "\n";
    
    if (cipo0Detected) {
        oss << "  CIPO0: " << chipTypeToString(cipo0ChipType) << "\n";
    }
    
    if (cipo1Detected) {
        oss << "  CIPO1: " << chipTypeToString(cipo1ChipType) << "\n";
    }
    
    double bestScore = cipo0Score + cipo1Score;
    std::string confidence = (bestScore > 100) ? "High" : "Medium";
    oss << "  Detection Confidence: " << confidence;
    
    return oss.str();
}

std::string IntanInterface::AutoDetectionResult::getChannelSummary() const {
    if (!chipsDetected) {
        return "No chips detected";
    }
    
    std::vector<std::string> channels;
    if (cipo0Detected) {
        std::string desc = "CIPO0 (" + chipTypeToString(cipo0ChipType) + ")";
        channels.push_back(desc);
    }
    
    if (cipo1Detected) {
        std::string desc = "CIPO1 (" + chipTypeToString(cipo1ChipType) + ")";
        channels.push_back(desc);
    }
    
    if (channels.empty()) {
        return "No active channels";
    }
    
    std::ostringstream oss;
    oss << "Active channels: ";
    for (size_t i = 0; i < channels.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << channels[i];
    }
    return oss.str();
}

