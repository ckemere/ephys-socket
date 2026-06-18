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
#include <cerrno>
#include <array>

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
    // Firmware status response has grown over time:
    //   86  bytes  pre-aux firmware
    //   98  bytes  aux-seq-v2 (aux sequencer block)
    //  122  bytes  + DMA/perf instrumentation (fw 1.1.0.0)
    //  126  bytes  + aux_ctrl (CTRL_REG_22 readback)        -- 65d5fb5
    //  148  bytes  + rhd_reg[22] mirror (commanded chip cfg) -- 7fb41dc
    // The buffer must be sized to the largest known form, or extra bytes will
    // sit unread in the TCP queue and corrupt the next command's ACK. The
    // parser accepts any size >= STATUS_RESPONSE_SIZE_LEGACY and decodes
    // optional fields based on what the device actually sent.
    constexpr size_t STATUS_RESPONSE_SIZE = 148;
    constexpr size_t STATUS_RESPONSE_SIZE_RHD = 148;
    constexpr size_t STATUS_RESPONSE_SIZE_AUXCTRL = 126;
    constexpr size_t STATUS_RESPONSE_SIZE_PERF = 122;
    constexpr size_t STATUS_RESPONSE_SIZE_AUX = 98;
    constexpr size_t STATUS_RESPONSE_SIZE_LEGACY = 86;

    // CTRL_REG_AUX_CTRL bit layout (mirrors firmware/include/main.h).
    constexpr uint32_t AUX_CTRL_SEQ_EN              = 1u << 0;
    constexpr uint32_t AUX_CTRL_FS_SW               = 1u << 4;
    constexpr uint32_t AUX_CTRL_FS_GPIO_EN          = 1u << 5;
    constexpr int      AUX_CTRL_FS_GPIO_SEL_SHIFT   = 6;
    constexpr uint32_t AUX_CTRL_DSP_SW              = 1u << 9;
    constexpr uint32_t AUX_CTRL_DSP_GPIO_EN         = 1u << 10;
    constexpr int      AUX_CTRL_DSP_GPIO_SEL_SHIFT  = 11;
    constexpr uint32_t AUX_CTRL_DIGOUT_SW           = 1u << 14;
    constexpr uint32_t AUX_CTRL_DIGOUT_GPIO_EN      = 1u << 15;
    constexpr int      AUX_CTRL_DIGOUT_GPIO_SEL_SHIFT = 16;
    constexpr int      AUX_CTRL_REG3_STATIC_SHIFT   = 24;

    // Command IDs
    enum CommandId : uint32_t {
        CMD_START = 0x01,
        CMD_STOP = 0x02,
        CMD_RESET_TIMESTAMP = 0x03,
        CMD_SET_LOOP_COUNT = 0x10,
        CMD_SET_PHASE = 0x11,
        CMD_SET_DEBUG_MODE = 0x12,
        CMD_SET_CHANNEL_ENABLE = 0x13,
        CMD_SET_PHASE_B = 0x14,      // port B (second cable) CIPO phase (phase2, phase3)
        CMD_LOAD_CONVERT = 0x20,
        CMD_LOAD_INIT = 0x21,
        CMD_LOAD_CABLE_TEST = 0x22,
        CMD_FULL_CABLE_TEST = 0x30,
        CMD_GET_STATUS = 0x40,
        CMD_SET_UDP_DEST = 0x50,
        // Aux command sequencer / override layer (firmware aux-seq-v2;
        // mirrors firmware/src-core0/network.c + remote/net.py)
        CMD_AUX_WRITE_WORD = 0x70,   // p1 = slot | bank<<8 | is_len<<16; p2 = addr<<16 | data
        CMD_AUX_BANK_SELECT = 0x71,  // p1 = slot; p2 = bank (confirmed before ACK)
        CMD_AUX_SEQ_EN = 0x72,       // p1 = 0/1
        CMD_READ_REGISTER = 0x73,    // p1 = reg -> 4-byte {cipo1,cipo0} response
        CMD_WRITE_REGISTER = 0x74,   // p1 = reg; p2 = value -> 4-byte echo response
        CMD_SET_FAST_SETTLE = 0x75,  // p1 = amp: sw|gpio_en<<1|pin<<4; p2 = dsp: same layout
        CMD_SET_DIGOUT = 0x76        // p1 = sw|gpio_en<<1|pin<<4; p2 = reg3_static byte
    };
    
    // ACK status codes
    constexpr uint8_t ACK_SUCCESS = 0x06;
    constexpr uint8_t ACK_ERROR = 0x15;
    
    // Packet header
    constexpr size_t PACKET_HEADER_WORDS = 10;
    
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

    // Firmware (ab14c19) holds a one-packet guard band between the PL write
    // frontier and the PS read frontier (cross-clock read-during-write fix).
    // Worse: handle_enable_streaming() resyncs ps_read_address to the MOST
    // RECENT magic in BRAM, so a finite loop_count that stops before that
    // resync leaves ps_read at the last packet -- packets_available = 0 and
    // no UDP is ever emitted. Use loop_count=0 (infinite); the PL keeps writing,
    // PS drains continuously, and CMD_STOP halts it after we've captured.
    // RTL: `loop_limit_reached <= (loop_count != 0) && (counter >= loop_count)`,
    // so 0 is the "no limit" sentinel.
    constexpr uint32_t DETECTION_LOOP_COUNT = 0;
    // Cable test now uses ce=0xFF (both ports, all 8 streams) so port A and
    // port B are tested in parallel; packet has 10 header + 140 data = 150 words.
    constexpr uint8_t  DETECTION_CHANNEL_ENABLE = 0xFF;
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
        
        // Give the listener a moment to bind so the device's reply about
        // udp_dest reflects post-autoConfigureUdp state.
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Get initial status to sync channel enable
        DeviceStatus status;
        if (getStatusInternal(status)) {
            updateChannelEnable(status.channelEnable);
            std::cout << "[IntanInterface] firmware reports udp_dest "
                      << status.udpDestIp << ":" << status.udpDestPort
                      << ", channel_enable=0x" << std::hex
                      << static_cast<int>(status.channelEnable) << std::dec
                      << ", fw " << status.getFirmwareVersionString()
                      << std::endl;
        } else {
            std::cout << "[IntanInterface] WARN: initial getStatus failed"
                      << std::endl;
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
            } else if (dataLen > 0) {
                // Firmware is sending more bytes than the caller's buffer can
                // hold (newer firmware revision). Drain them so the next command's
                // ACK header isn't corrupted, then report failure for this call.
                uint8_t scratch[256];
                uint16_t remaining = dataLen;
                while (remaining > 0) {
                    int chunk = remaining > sizeof(scratch) ? (int)sizeof(scratch) : remaining;
                    int got = recv(tcpSocket_, reinterpret_cast<char*>(scratch), chunk, 0);
                    if (got <= 0) break;
                    remaining -= (uint16_t)got;
                }
                *responseLen = 0;
                return false;
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

        // Accept any of the known sizes (see STATUS_RESPONSE_SIZE comment).
        if (dataLen != STATUS_RESPONSE_SIZE_RHD &&
            dataLen != STATUS_RESPONSE_SIZE_AUXCTRL &&
            dataLen != STATUS_RESPONSE_SIZE_PERF &&
            dataLen != STATUS_RESPONSE_SIZE_AUX &&
            dataLen != STATUS_RESPONSE_SIZE_LEGACY) {
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
        status.udpBytesSent = unpackU32LE(p); p += 4;

        // Aux sequencer status block (aux-seq-v2 firmware and later)
        status.hasAuxStatus = (dataLen >= STATUS_RESPONSE_SIZE_AUX);
        status.auxSeqEnabled = false;
        status.fastSettleActive = false;
        status.digoutState = false;
        status.dspResetActive = false;
        status.auxBankActive = 0;
        status.auxIndex[0] = status.auxIndex[1] = status.auxIndex[2] = 0;
        status.auxReadResult = 0;
        if (status.hasAuxStatus) {
            status.auxReadResult = unpackU32LE(p); p += 4;
            status.auxBankActive = *p++;
            uint8_t auxFlags = *p++;
            status.auxSeqEnabled    = (auxFlags & 0x01) != 0;
            status.fastSettleActive = (auxFlags & 0x02) != 0;
            status.digoutState      = (auxFlags & 0x04) != 0;
            status.dspResetActive   = (auxFlags & 0x08) != 0;
            status.auxIndex[0] = *p++;
            status.auxIndex[1] = *p++;
            status.auxIndex[2] = *p++;
            p += 3;  // 3 reserved bytes
        }

        // DMA / perf instrumentation block (firmware 1.1.0.0+, status >= 122)
        // -- we don't expose the fields in DeviceStatus yet, just skip past.
        if (dataLen >= STATUS_RESPONSE_SIZE_PERF) {
            p += 24;
        }

        // CTRL_REG_22 (aux_ctrl) readback (firmware 65d5fb5+, status >= 126).
        // Decode the fast-settle / DSP / digout config so the editor can sync
        // its TTL combo to whatever the device is actually in.
        status.hasAuxCtrl = false;
        status.auxCtrlRaw = 0;
        status.fsSwLevel = status.fsGpioEn = false;     status.fsGpioPin = 0;
        status.dspSwLevel = status.dspGpioEn = false;   status.dspGpioPin = 0;
        status.digoutSwLevel = status.digoutGpioEn = false; status.digoutGpioPin = 0;
        status.reg3Static = 0;
        if (dataLen >= STATUS_RESPONSE_SIZE_AUXCTRL) {
            uint32_t auxCtrl = unpackU32LE(p); p += 4;
            status.hasAuxCtrl = true;
            status.auxCtrlRaw = auxCtrl;
            status.fsSwLevel     = (auxCtrl & AUX_CTRL_FS_SW) != 0;
            status.fsGpioEn      = (auxCtrl & AUX_CTRL_FS_GPIO_EN) != 0;
            status.fsGpioPin     = (auxCtrl >> AUX_CTRL_FS_GPIO_SEL_SHIFT) & 0x7;
            status.dspSwLevel    = (auxCtrl & AUX_CTRL_DSP_SW) != 0;
            status.dspGpioEn     = (auxCtrl & AUX_CTRL_DSP_GPIO_EN) != 0;
            status.dspGpioPin    = (auxCtrl >> AUX_CTRL_DSP_GPIO_SEL_SHIFT) & 0x7;
            status.digoutSwLevel = (auxCtrl & AUX_CTRL_DIGOUT_SW) != 0;
            status.digoutGpioEn  = (auxCtrl & AUX_CTRL_DIGOUT_GPIO_EN) != 0;
            status.digoutGpioPin = (auxCtrl >> AUX_CTRL_DIGOUT_GPIO_SEL_SHIFT) & 0x7;
            status.reg3Static    = (auxCtrl >> AUX_CTRL_REG3_STATIC_SHIFT) & 0xFF;
        }

        // RHD chip register mirror (firmware 7fb41dc+, status == 148).
        status.hasRhdRegMirror = false;
        std::memset(status.rhdReg, 0, sizeof(status.rhdReg));
        if (dataLen >= STATUS_RESPONSE_SIZE_RHD) {
            std::memcpy(status.rhdReg, p, 22);
            p += 22;
            status.hasRhdRegMirror = true;
        }

        return true;
    }

    // ========================================================================
    // AUX COMMAND SEQUENCER
    // ========================================================================

    bool auxUploadBank(int slot, int bank, const std::vector<uint16_t>& commands,
                       int loopIndex) {
        if (commands.empty() || commands.size() > 64 ||
            loopIndex < 0 || loopIndex >= (int)commands.size()) {
            reportError("auxUploadBank: bad program size/loop index");
            return false;
        }
        uint32_t p1 = (slot & 3) | ((bank & 1) << 8);
        for (size_t i = 0; i < commands.size(); ++i) {
            uint32_t p2 = ((uint32_t)i << 16) | commands[i];
            if (!sendCommand(CMD_AUX_WRITE_WORD, p1, p2)) {
                reportError("auxUploadBank: word upload failed");
                return false;
            }
        }
        // Length record: loop index in [5:0], end index in [13:8]
        uint32_t lengthData = (loopIndex & 0x3F) |
                              (((uint32_t)(commands.size() - 1) & 0x3F) << 8);
        return sendCommand(CMD_AUX_WRITE_WORD, p1 | (1u << 16), lengthData);
    }

    bool auxBankSelect(int slot, int bank) {
        // Firmware polls bank_active (up to ~50 ms) before ACKing
        return sendCommand(CMD_AUX_BANK_SELECT, slot & 3, bank & 1);
    }

    bool auxSeqEnable(bool enable) {
        return sendCommand(CMD_AUX_SEQ_EN, enable ? 1 : 0);
    }

    bool setFastSettle(bool softwareLevel, bool gpioEnable, uint8_t gpioPin,
                       bool dspReset) {
        uint32_t p1 = (softwareLevel ? 1 : 0) | (gpioEnable ? 2 : 0)
                    | ((uint32_t)(gpioPin & 7) << 4);
        // When dspReset is requested, mirror the *entire* fast-settle config
        // into the DSP fields (same layout: bit0 = SW, bit1 = GPIO_EN, [6:4] =
        // pin sel). That way the DSP reset (CONVERT bit-H force) follows the
        // exact same trigger as the amplifier fast settle -- both fire together
        // on the SW level or the selected digital_in pin.
        uint32_t p2 = dspReset ? p1 : 0;
        return sendCommand(CMD_SET_FAST_SETTLE, p1, p2);
    }

    bool readRegister(uint8_t reg, uint16_t& cipo0Value, uint16_t& cipo1Value) {
        uint8_t data[4];
        size_t dataLen = sizeof(data);
        if (!sendCommand(CMD_READ_REGISTER, reg & 0x3F, 0, data, &dataLen))
            return false;
        if (dataLen != 4)
            return false;
        uint32_t result = unpackU32LE(data);
        cipo0Value = result & 0xFFFF;
        cipo1Value = (result >> 16) & 0xFFFF;
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

    // Per-CIPO-lane best across the sweep. Used by runAutoDetection and
    // testPhaseParallel; declared once at class scope so both can share it.
    struct LaneBest {
        const char* label;     // "A.CIPO0", "A.CIPO1", "B.CIPO0", "B.CIPO1"
        int      stride;       // lane stride in data-word units
        int      offset;       // lane offset within each cycle's words
        double   bestScore;
        ChipType bestType;
        bool     bestHasDdr;
        uint8_t  bestPhase;
        bool     detected;
    };

    // One sweep over 16 phase values, both port-A and port-B phases ganged to
    // the same value, channel_enable = 0xFF so every packet carries CIPO0/CIPO1
    // for BOTH cables in the same lane positions. Each phase yields a single
    // packet; we score all four CIPO lanes from it and track per-port best.
    bool runAutoDetection(AutoDetectionResult& result, bool verbose) {
        result = AutoDetectionResult{};
        result.cipo0ChipType = ChipType::NONE;
        result.cipo1ChipType = ChipType::NONE;
        result.portBCipo0ChipType = ChipType::NONE;
        result.portBCipo1ChipType = ChipType::NONE;

        if (verbose) {
            std::cout << "[Detection] Starting automated chip detection (port A + port B, parallel)..." << std::endl;
        }

        DeviceStatus status;
        if (!getStatusInternal(status)) {
            if (verbose) std::cout << "[Detection] ERROR: Cannot read device status" << std::endl;
            return false;
        }
        if (status.transmissionActive) {
            if (verbose) std::cout << "[Detection] ERROR: Device is transmitting. Stop acquisition first." << std::endl;
            return false;
        }

        // initializeForDetection sets channel_enable=0xFF and loop_count=DETECTION_LOOP_COUNT
        if (!initializeForDetection(verbose)) {
            return false;
        }

        // Per-lane bookkeeping. Same struct as the method-level LaneBest below
        // so testPhaseParallel can update bestScore/bestType in place.
        LaneBest lanes[4] = {
            { "A.CIPO0", 4, 0, 0.0, ChipType::NONE, false, 0, false },
            { "A.CIPO1", 4, 1, 0.0, ChipType::NONE, false, 0, false },
            { "B.CIPO0", 4, 2, 0.0, ChipType::NONE, false, 0, false },
            { "B.CIPO1", 4, 3, 0.0, ChipType::NONE, false, 0, false },
        };

        for (int phase = 0; phase < NUM_PHASES_TO_TEST; ++phase) {
            std::array<double, 4>   scores{0,0,0,0};
            std::array<ChipType, 4> types{ChipType::NONE, ChipType::NONE, ChipType::NONE, ChipType::NONE};

            bool ok = testPhaseParallel(static_cast<uint8_t>(phase), lanes,
                                        scores, types, verbose);
            if (!ok) {
                continue;  // already logged "No packet received" in testPhaseParallel
            }

            // Per-lane track the best phase (independent across lanes; same value
            // of phase, but different CIPOs can prefer different phases when
            // cables differ in length).
            for (int i = 0; i < 4; ++i) {
                if (scores[i] > DETECTION_THRESHOLD && scores[i] > lanes[i].bestScore) {
                    lanes[i].bestScore   = scores[i];
                    lanes[i].bestType    = types[i];
                    lanes[i].bestHasDdr  = (types[i] == ChipType::RHD2164);
                    lanes[i].bestPhase   = static_cast<uint8_t>(phase);
                    lanes[i].detected    = true;
                }
            }

            if (verbose && (scores[0] + scores[1] + scores[2] + scores[3]) > 0) {
                std::cout << "  Phase " << phase << ": "
                          << "A.CIPO0=" << scores[0]
                          << " A.CIPO1=" << scores[1]
                          << " B.CIPO0=" << scores[2]
                          << " B.CIPO1=" << scores[3] << std::endl;
            }

            // Record the phase result for callers that want the raw matrix.
            PhaseTestResult pr;
            pr.phase = static_cast<uint8_t>(phase);
            pr.cipo0Score = scores[0]; pr.cipo1Score = scores[1];
            pr.cipo0HasDdr = (types[0] == ChipType::RHD2164);
            pr.cipo1HasDdr = (types[1] == ChipType::RHD2164);
            pr.cipo0ChipType = types[0]; pr.cipo1ChipType = types[1];
            result.allPhaseResults.push_back(pr);
        }

        // Port A: cipo0 + cipo1
        result.cipo0Detected     = lanes[0].detected;
        result.cipo0ChipType     = lanes[0].bestType;
        result.cipo0HasDdr       = lanes[0].bestHasDdr;
        result.cipo0Score        = lanes[0].bestScore;
        result.cipo1Detected     = lanes[1].detected;
        result.cipo1ChipType     = lanes[1].bestType;
        result.cipo1HasDdr       = lanes[1].bestHasDdr;
        result.cipo1Score        = lanes[1].bestScore;

        // Port B: cipo0 + cipo1
        result.portBCipo0Detected = lanes[2].detected;
        result.portBCipo0ChipType = lanes[2].bestType;
        result.portBCipo0HasDdr   = lanes[2].bestHasDdr;
        result.portBCipo1Detected = lanes[3].detected;
        result.portBCipo1ChipType = lanes[3].bestType;
        result.portBCipo1HasDdr   = lanes[3].bestHasDdr;

        // Pick one phase per port (the better of the port's two CIPO lanes).
        auto pickPhase = [](const LaneBest& a, const LaneBest& b) -> uint8_t {
            if (a.detected && b.detected) {
                return (a.bestScore >= b.bestScore) ? a.bestPhase : b.bestPhase;
            }
            if (a.detected) return a.bestPhase;
            if (b.detected) return b.bestPhase;
            return 0;
        };
        result.bestPhase0 = pickPhase(lanes[0], lanes[1]);
        result.bestPhase1 = pickPhase(lanes[2], lanes[3]);

        result.chipsDetected = result.cipo0Detected || result.cipo1Detected ||
                               result.portBCipo0Detected || result.portBCipo1Detected;
        result.success = result.chipsDetected;

        result.optimalChannelMask = 0;
        if (result.cipo0Detected) { result.optimalChannelMask |= 0x01; if (result.cipo0HasDdr) result.optimalChannelMask |= 0x02; }
        if (result.cipo1Detected) { result.optimalChannelMask |= 0x04; if (result.cipo1HasDdr) result.optimalChannelMask |= 0x08; }
        if (result.portBCipo0Detected) { result.optimalChannelMask |= 0x10; if (result.portBCipo0HasDdr) result.optimalChannelMask |= 0x20; }
        if (result.portBCipo1Detected) { result.optimalChannelMask |= 0x40; if (result.portBCipo1HasDdr) result.optimalChannelMask |= 0x80; }

        if (verbose) {
            if (result.success) {
                std::cout << "[Detection] Complete. Port A phase=" << (int)result.bestPhase0
                          << " port B phase=" << (int)result.bestPhase1
                          << " mask=0x" << std::hex << (int)result.optimalChannelMask
                          << std::dec << std::endl;
                std::cout << "[Detection] " << result.getChannelSummary() << std::endl;
            } else {
                std::cout << "[Detection] No chips detected on either port" << std::endl;
            }
        }
        return true;
    }

    // Set both phase pairs (port A: CMD_SET_PHASE; port B: CMD_SET_PHASE_B) to
    // `phase`, briefly run the cable-test sequence with loop_count buffered, and
    // score all four CIPO lanes out of the captured packet.
    bool testPhaseParallel(uint8_t phase,
                           LaneBest (&lanes)[4],
                           std::array<double, 4>& scoresOut,
                           std::array<ChipType, 4>& typesOut,
                           bool verbose) {
        // Stop, set both ports' phases, start, capture one packet, stop.
        sendCommand(CMD_STOP);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        if (!sendCommand(CMD_SET_PHASE,   phase, phase) ||
            !sendCommand(CMD_SET_PHASE_B, phase, phase)) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        RxSnapshot before = rxSnapshot();
        startDetectionCapture();

        if (!sendCommand(CMD_START)) {
            stopDetectionCapture();
            return false;
        }

        std::vector<uint32_t> packet;
        bool captured = capturePacketForDetection(packet, 1.0);

        sendCommand(CMD_STOP);
        stopDetectionCapture();

        if (!captured) {
            if (verbose) {
                RxSnapshot after = rxSnapshot();
                std::cout << "  Phase " << static_cast<int>(phase)
                          << ": No packet received"
                          << "  (rx delta total=" << (after.total - before.total)
                          << " magicErr=" << (after.magic - before.magic)
                          << " sizeErr=" << (after.size - before.size)
                          << ", expectedBytes=" << (expectedPacketSizeWords_ * 4)
                          << ")" << std::endl;
            }
            return false;
        }

        for (int i = 0; i < 4; ++i) {
            auto pr = scoreChannel(packet, lanes[i].stride, lanes[i].offset,
                                   lanes[i].label, verbose);
            scoresOut[i] = pr.first;
            typesOut[i]  = pr.second;
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
            std::cout << "[IntanInterface] autoConfigureUdp: socket() failed" << std::endl;
            return;
        }

        struct sockaddr_in serverAddr;
        std::memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(tcpPort_);
        inet_pton(AF_INET, deviceIp_.c_str(), &serverAddr.sin_addr);

        bool ok = false;
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
                ok = sendCommand(CMD_SET_UDP_DEST, ntohl(ipInt), udpPort_);
                std::cout << "[IntanInterface] autoConfigureUdp: telling device to send UDP to "
                          << localIp << ":" << udpPort_
                          << (ok ? "  (ACKed)" : "  (NACKed!)") << std::endl;
            } else {
                std::cout << "[IntanInterface] autoConfigureUdp: getsockname failed" << std::endl;
            }
        } else {
            std::cout << "[IntanInterface] autoConfigureUdp: connect() to "
                      << deviceIp_ << ":" << tcpPort_ << " failed" << std::endl;
        }

        closesocket(tempSock);
    }
    
    void udpListenerThread() {
        // Create UDP socket
        udpSocket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (udpSocket_ == INVALID_SOCKET) {
            std::cout << "[IntanInterface] UDP listener: socket() failed" << std::endl;
            reportError("Failed to create UDP socket");
            return;
        }

        // Allow rebinding the port even if a previous instance left it in
        // TIME_WAIT or another local process (e.g. net.py) holds it.
        int reuse = 1;
        setsockopt(udpSocket_, SOL_SOCKET, SO_REUSEADDR,
                   reinterpret_cast<char*>(&reuse), sizeof(reuse));
#ifdef SO_REUSEPORT
        setsockopt(udpSocket_, SOL_SOCKET, SO_REUSEPORT,
                   reinterpret_cast<char*>(&reuse), sizeof(reuse));
#endif

        // Bind to UDP port
        struct sockaddr_in bindAddr;
        std::memset(&bindAddr, 0, sizeof(bindAddr));
        bindAddr.sin_family = AF_INET;
        bindAddr.sin_addr.s_addr = INADDR_ANY;
        bindAddr.sin_port = htons(udpPort_);

        if (bind(udpSocket_, reinterpret_cast<struct sockaddr*>(&bindAddr),
                sizeof(bindAddr)) == SOCKET_ERROR) {
            std::cout << "[IntanInterface] UDP listener: bind to port "
                      << udpPort_ << " FAILED (errno=" << errno << ")" << std::endl;
            reportError("Failed to bind UDP socket");
            closesocket(udpSocket_);
            udpSocket_ = INVALID_SOCKET;
            return;
        }

        std::cout << "[IntanInterface] UDP listener: bound on port "
                  << udpPort_ << std::endl;
        
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
        // Disable debug mode to read real chip data
        if (!sendCommand(CMD_STOP)) {
            if (verbose) {
                std::cout << "[Detection] ERROR: Failed to disable debug mode" << std::endl;
            }
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));


        if (!sendCommand(CMD_SET_DEBUG_MODE, 0)) {
            if (verbose) {
                std::cout << "[Detection] ERROR: Failed to disable debug mode" << std::endl;
            }
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Loop count >=2 is required: firmware ab14c19 holds a one-packet guard
        // band in packets_available(), so loop_count==1 -> zero UDP emitted.
        if (!sendCommand(CMD_SET_LOOP_COUNT, DETECTION_LOOP_COUNT)) {
            if (verbose) {
                std::cout << "[Detection] ERROR: Failed to set loop count" << std::endl;
            }
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Channel enable = 0xFF: both ports' CIPO0+CIPO1 land in one packet, so
        // a single phase sweep tests A and B in parallel.
        if (!sendCommand(CMD_SET_CHANNEL_ENABLE, DETECTION_CHANNEL_ENABLE)) {
            if (verbose) {
                std::cout << "[Detection] ERROR: Failed to set channel enable" << std::endl;
            }
            return false;
        }
        updateChannelEnable(DETECTION_CHANNEL_ENABLE);
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
    
    // Snapshot reception counters for diagnostic comparisons.
    struct RxSnapshot {
        uint64_t total;
        uint64_t magic;
        uint64_t size;
    };
    RxSnapshot rxSnapshot() const {
        std::lock_guard<std::mutex> lock(statsMutex_);
        return RxSnapshot{ totalPackets_, magicErrors_, sizeErrors_ };
    }

    // Score one CIPO lane out of a cable-test packet.
    //   stride / offset = pick lane out of the packed data: one 32-bit word per
    //                     enabled stream-pair, per cycle, in bit order.
    //                     ce=0x0F (port A only):  stride=2  offset=0(A0) 1(A1)
    //                     ce=0xF0 (port B only):  stride=2  offset=0(B0) 1(B1)
    //                     ce=0xFF (both ports):   stride=4
    //                                              offset=0(A0) 1(A1) 2(B0) 3(B1)
    //   label is used only for verbose output (e.g. "A.CIPO0").
    std::pair<double, ChipType> scoreChannel(
        const std::vector<uint32_t>& packet, int stride, int offset,
        const std::string& label, bool verbose) {

        if (packet.size() < PACKET_HEADER_WORDS + 9 * static_cast<size_t>(stride)) {
            return {0.0, ChipType::NONE};
        }

        double score = 0.0;
        ChipType chipType = ChipType::NONE;

        // Extract data words (skip header)
        std::vector<uint32_t> dataWords(packet.begin() + PACKET_HEADER_WORDS, packet.end());

        if (dataWords.size() < 35) {
            return {0.0, ChipType::NONE};
        }

        // Extract this lane's words from the tightly-packed data block.
        std::vector<uint32_t> channelWords;
        for (size_t i = offset; i < dataWords.size(); i += stride) {
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
            
            std::cout << "    " << label << ": '" << patternStr
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

bool IntanInterface::setPhaseSelectB(uint8_t phase2, uint8_t phase3) {
    return pImpl_->sendCommand(CMD_SET_PHASE_B, phase2, phase3);
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

bool IntanInterface::auxUploadBank(int slot, int bank,
                                   const std::vector<uint16_t>& commands,
                                   int loopIndex) {
    return pImpl_->auxUploadBank(slot, bank, commands, loopIndex);
}

bool IntanInterface::auxBankSelect(int slot, int bank) {
    return pImpl_->auxBankSelect(slot, bank);
}

bool IntanInterface::auxSeqEnable(bool enable) {
    return pImpl_->auxSeqEnable(enable);
}

bool IntanInterface::setFastSettle(bool softwareLevel, bool gpioEnable,
                                   uint8_t gpioPin, bool dspReset) {
    return pImpl_->setFastSettle(softwareLevel, gpioEnable, gpioPin, dspReset);
}

bool IntanInterface::readRegister(uint8_t reg, uint16_t& cipo0Value,
                                  uint16_t& cipo1Value) {
    return pImpl_->readRegister(reg, cipo0Value, cipo1Value);
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

    // Port A phase: bestPhase0 applies to both port-A CIPO lines (cipo0/cipo1).
    if (!setPhaseSelect(result.bestPhase0, result.bestPhase0)) {
        return false;
    }

    // Port B phase: bestPhase1 applies to both port-B CIPO lines (phase2/phase3).
    // setPhaseSelectB is a no-op on firmware without dual-port support; harmless
    // if no chips were found there.
    if (!setPhaseSelectB(result.bestPhase1, result.bestPhase1)) {
        return false;
    }

    // Combined 8-bit channel enable mask (port A in [3:0], port B in [7:4]).
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
    for (int i = 0; i < 8; ++i) {
        if (channelMask & (1 << i)) {
            numChannels++;
        }
    }
    
    if (numChannels == 0) {
        return 80; // Default to maximum
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

std::string IntanInterface::DeviceStatus::getSummary() const {
    std::ostringstream oss;
    oss << "=== INTAN DEVICE STATUS ===\n";
    oss << "Firmware: v" << getFirmwareVersionString()
        << "  (protocol " << protocolVersion << ", device 0x"
        << std::hex << deviceType << std::dec << ")\n";
    oss << "--- PL Hardware ---\n";
    oss << "Timestamp: " << timestamp
        << "  Packets sent: " << packetsSent << "\n";
    oss << "Transmission: " << (transmissionActive ? "ACTIVE" : "stopped")
        << "  Loop limit: " << (loopLimitReached ? "reached" : "no")
        << "  State/Cycle: " << (int)stateCounter << "/" << (int)cycleCounter << "\n";
    oss << "BRAM write addr: " << bramWriteAddr
        << "  FIFO count: " << fifoCount << "\n";
    oss << "--- PS Software ---\n";
    oss << "Packets received: " << packetsReceived
        << "  Errors: " << errorCount << "\n";
    oss << "UDP sent: " << udpPacketsSent
        << "  UDP errors: " << udpSendErrors
        << "  Packet size: " << packetSizeWords << " words\n";
    oss << "--- Configuration ---\n";
    oss << "Channels: 0x" << std::hex << (int)channelEnable << std::dec
        << " (" << getChannelEnableString() << ")"
        << "  Phase: " << (int)phase0 << "/" << (int)phase1
        << "  Debug: " << (debugMode ? "ON" : "off") << "\n";
    oss << "UDP dest: " << udpDestIp << ":" << udpDestPort << "\n";
    oss << "--- Aux Sequencer ---\n";
    if (!hasAuxStatus) {
        oss << "(not supported by this firmware -- 86-byte status)\n";
    } else {
        oss << "Enabled: " << (auxSeqEnabled ? "YES" : "no")
            << "  Fast settle: " << (fastSettleActive ? "ACTIVE" : "off")
            << "  Digout: " << (digoutState ? "1" : "0")
            << "  DSP reset: " << (dspResetActive ? "ACTIVE" : "off") << "\n";
        oss << "Active banks: slot0=" << ((auxBankActive >> 0) & 1)
            << " slot1=" << ((auxBankActive >> 1) & 1)
            << " slot2=" << ((auxBankActive >> 2) & 1)
            << "  Indices: " << (int)auxIndex[0] << "/"
            << (int)auxIndex[1] << "/" << (int)auxIndex[2] << "\n";
        oss << "Last inject result: 0x" << std::hex << std::setw(8)
            << std::setfill('0') << auxReadResult << std::dec << "\n";
    }
    oss << "===========================";
    return oss.str();
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
    oss << "  Port A phase: " << static_cast<int>(bestPhase0) << "\n";
    oss << "  Port B phase: " << static_cast<int>(bestPhase1) << "\n";
    oss << "  Channel Mask: 0x" << std::hex << std::uppercase
        << static_cast<int>(optimalChannelMask) << std::dec << "\n";

    if (cipo0Detected) {
        oss << "  A.CIPO0: " << chipTypeToString(cipo0ChipType) << "\n";
    }
    if (cipo1Detected) {
        oss << "  A.CIPO1: " << chipTypeToString(cipo1ChipType) << "\n";
    }
    if (portBCipo0Detected) {
        oss << "  B.CIPO0: " << chipTypeToString(portBCipo0ChipType) << "\n";
    }
    if (portBCipo1Detected) {
        oss << "  B.CIPO1: " << chipTypeToString(portBCipo1ChipType) << "\n";
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
        channels.push_back("A.CIPO0 (" + chipTypeToString(cipo0ChipType) + ")");
    }
    if (cipo1Detected) {
        channels.push_back("A.CIPO1 (" + chipTypeToString(cipo1ChipType) + ")");
    }
    if (portBCipo0Detected) {
        channels.push_back("B.CIPO0 (" + chipTypeToString(portBCipo0ChipType) + ")");
    }
    if (portBCipo1Detected) {
        channels.push_back("B.CIPO1 (" + chipTypeToString(portBCipo1ChipType) + ")");
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

