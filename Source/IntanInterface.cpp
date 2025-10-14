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
                sendCommand(CMD_SET_UDP_DEST, (ipInt), udpPort_);
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
    return pImpl_->sendCommand(CMD_SET_UDP_DEST, (ipInt), port);
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
