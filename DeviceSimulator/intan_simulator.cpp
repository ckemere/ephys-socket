#include <iostream>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <cstring>
#include <cmath>
#include <queue>
#include <condition_variable>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    typedef int socklen_t;
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #define SOCKET int
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
    #define closesocket close
#endif

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

constexpr uint32_t MAGIC_NUMBER_LOW = 0xDEADBEEF;
constexpr uint32_t MAGIC_NUMBER_HIGH = 0xCAFEBABE;
constexpr uint32_t BRAM_SIZE_WORDS = 16384;
constexpr uint32_t PACKET_HEADER_WORDS = 4;
constexpr uint32_t MAX_PACKET_DATA_WORDS = 70;
constexpr uint32_t MAX_WORDS_PER_PACKET = PACKET_HEADER_WORDS + MAX_PACKET_DATA_WORDS;

constexpr uint16_t TCP_PORT = 6000;
constexpr uint16_t UDP_PORT = 5000;
constexpr uint32_t SAMPLING_RATE_HZ = 30000;

// Command IDs
constexpr uint32_t CMD_MAGIC = 0xDEADBEEF;
constexpr uint32_t CMD_START = 0x01;
constexpr uint32_t CMD_STOP = 0x02;
constexpr uint32_t CMD_RESET_TIMESTAMP = 0x03;
constexpr uint32_t CMD_SET_LOOP_COUNT = 0x10;
constexpr uint32_t CMD_SET_PHASE = 0x11;
constexpr uint32_t CMD_SET_DEBUG_MODE = 0x12;
constexpr uint32_t CMD_SET_CHANNEL_ENABLE = 0x13;
constexpr uint32_t CMD_GET_STATUS = 0x40;
constexpr uint32_t CMD_SET_UDP_DEST = 0x50;

constexpr uint8_t ACK_SUCCESS = 0x06;
constexpr uint8_t ACK_ERROR = 0x15;

// ============================================================================
// UTILITY CLASSES
// ============================================================================

class NetworkInit {
public:
    NetworkInit() {
#ifdef _WIN32
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            throw std::runtime_error("WSAStartup failed");
        }
#endif
    }
    
    ~NetworkInit() {
#ifdef _WIN32
        WSACleanup();
#endif
    }
};

// ============================================================================
// REGISTER INTERFACE
// ============================================================================

class RegisterInterface {
private:
    std::mutex mutex_;
    
    // Control registers
    bool enable_transmission_ = false;
    bool reset_timestamp_ = false;
    bool debug_mode_ = false;
    uint32_t loop_count_ = 0;
    uint8_t phase0_ = 0;
    uint8_t phase1_ = 0;
    uint8_t channel_enable_ = 0x0F;  // All channels enabled by default
    
    // Status registers
    std::atomic<uint64_t> timestamp_{0};
    std::atomic<uint32_t> packets_sent_{0};
    std::atomic<uint32_t> bram_write_addr_{0};
    std::atomic<bool> transmission_active_{false};
    std::atomic<bool> loop_limit_reached_{false};
    
public:
    // Control register access
    bool get_enable_transmission() {
        std::lock_guard<std::mutex> lock(mutex_);
        return enable_transmission_;
    }
    
    void set_enable_transmission(bool enable) {
        std::lock_guard<std::mutex> lock(mutex_);
        enable_transmission_ = enable;
    }
    
    bool get_reset_timestamp() {
        std::lock_guard<std::mutex> lock(mutex_);
        return reset_timestamp_;
    }
    
    void set_reset_timestamp(bool reset) {
        std::lock_guard<std::mutex> lock(mutex_);
        reset_timestamp_ = reset;
    }
    
    bool get_debug_mode() {
        std::lock_guard<std::mutex> lock(mutex_);
        return debug_mode_;
    }
    
    void set_debug_mode(bool debug) {
        std::lock_guard<std::mutex> lock(mutex_);
        debug_mode_ = debug;
    }
    
    uint32_t get_loop_count() {
        std::lock_guard<std::mutex> lock(mutex_);
        return loop_count_;
    }
    
    void set_loop_count(uint32_t count) {
        std::lock_guard<std::mutex> lock(mutex_);
        loop_count_ = count;
    }
    
    void get_phase_select(uint8_t& phase0, uint8_t& phase1) {
        std::lock_guard<std::mutex> lock(mutex_);
        phase0 = phase0_;
        phase1 = phase1_;
    }
    
    void set_phase_select(uint8_t phase0, uint8_t phase1) {
        std::lock_guard<std::mutex> lock(mutex_);
        phase0_ = phase0 & 0x0F;
        phase1_ = phase1 & 0x0F;
    }
    
    uint8_t get_channel_enable() {
        std::lock_guard<std::mutex> lock(mutex_);
        return channel_enable_;
    }
    
    void set_channel_enable(uint8_t enable) {
        std::lock_guard<std::mutex> lock(mutex_);
        channel_enable_ = enable & 0x0F;
    }
    
    // Status register access
    uint64_t get_timestamp() const { return timestamp_.load(); }
    void set_timestamp(uint64_t ts) { timestamp_.store(ts); }
    void increment_timestamp() { timestamp_.fetch_add(1); }
    
    uint32_t get_packets_sent() const { return packets_sent_.load(); }
    void increment_packets_sent() { packets_sent_.fetch_add(1); }
    
    uint32_t get_bram_write_addr() const { return bram_write_addr_.load(); }
    void set_bram_write_addr(uint32_t addr) { bram_write_addr_.store(addr); }
    
    bool get_transmission_active() const { return transmission_active_.load(); }
    void set_transmission_active(bool active) { transmission_active_.store(active); }
    
    bool get_loop_limit_reached() const { return loop_limit_reached_.load(); }
    void set_loop_limit_reached(bool reached) { loop_limit_reached_.store(reached); }
};

// ============================================================================
// SINE WAVE GENERATOR (for debug mode)
// ============================================================================

class SineGenerator {
private:
    static constexpr size_t LUT_SIZE = 512;
    std::vector<int16_t> sine_lut_;
    uint32_t phase_index_ = 0;
    
public:
    SineGenerator() {
        sine_lut_.resize(LUT_SIZE);
        for (size_t i = 0; i < LUT_SIZE; i++) {
            double angle = 2.0 * M_PI * i / LUT_SIZE;
            sine_lut_[i] = static_cast<int16_t>(32767.0 * std::sin(angle));
        }
    }
    
    void generate_packet_data(std::vector<uint32_t>& data, size_t num_cycles) {
        for (size_t cycle = 0; cycle < num_cycles; cycle++) {
            uint32_t base_phase = (phase_index_ + cycle) & 0x1FF;
            
            // CIPO0 regular: 1× frequency (58.6 Hz at 30kHz sampling)
            int16_t cipo0_reg = sine_lut_[base_phase];
            
            // CIPO0 DDR: 2× frequency
            int16_t cipo0_ddr = sine_lut_[(base_phase << 1) & 0x1FF];
            
            // CIPO1 regular: 4× frequency
            int16_t cipo1_reg = sine_lut_[(base_phase << 2) & 0x1FF];
            
            // CIPO1 DDR: 8× frequency
            int16_t cipo1_ddr = sine_lut_[(base_phase << 3) & 0x1FF];
            
            // Pack into 32-bit words
            uint32_t cipo0_word = (static_cast<uint16_t>(cipo0_ddr) << 16) | 
                                  static_cast<uint16_t>(cipo0_reg);
            uint32_t cipo1_word = (static_cast<uint16_t>(cipo1_ddr) << 16) | 
                                  static_cast<uint16_t>(cipo1_reg);
            
            data.push_back(cipo0_word);
            data.push_back(cipo1_word);
        }
        
        phase_index_ = (phase_index_ + 1) & 0x1FF;
    }
};

// ============================================================================
// DATA GENERATOR (simulates PL)
// ============================================================================

class DataGenerator {
private:
    RegisterInterface& registers_;
    SineGenerator sine_gen_;
    uint32_t loop_counter_ = 0;
    std::atomic<bool> running_{false};
    
    uint32_t calculate_data_words(uint8_t channel_enable) {
        int num_channels = 0;
        if (channel_enable & 0x01) num_channels++;
        if (channel_enable & 0x02) num_channels++;
        if (channel_enable & 0x04) num_channels++;
        if (channel_enable & 0x08) num_channels++;
        
        if (num_channels == 0) return 70;
        
        uint32_t total_16bit_words = 35 * num_channels;
        return (total_16bit_words + 1) / 2;
    }
    
    void apply_channel_mask(std::vector<uint32_t>& full_data, 
                           std::vector<uint32_t>& masked_data,
                           uint8_t channel_enable) {
        masked_data.clear();
        
        for (size_t i = 0; i < full_data.size(); i += 2) {
            uint32_t cipo0_word = full_data[i];
            uint32_t cipo1_word = full_data[i + 1];
            
            // Extract 16-bit segments
            uint16_t cipo0_reg = cipo0_word & 0xFFFF;
            uint16_t cipo0_ddr = (cipo0_word >> 16) & 0xFFFF;
            uint16_t cipo1_reg = cipo1_word & 0xFFFF;
            uint16_t cipo1_ddr = (cipo1_word >> 16) & 0xFFFF;
            
            // Apply channel mask
            std::vector<uint16_t> enabled_segments;
            if (channel_enable & 0x01) enabled_segments.push_back(cipo0_reg);
            if (channel_enable & 0x02) enabled_segments.push_back(cipo0_ddr);
            if (channel_enable & 0x04) enabled_segments.push_back(cipo1_reg);
            if (channel_enable & 0x08) enabled_segments.push_back(cipo1_ddr);
            
            // Pack into 32-bit words
            for (size_t j = 0; j < enabled_segments.size(); j += 2) {
                uint32_t word = enabled_segments[j];
                if (j + 1 < enabled_segments.size()) {
                    word |= (static_cast<uint32_t>(enabled_segments[j + 1]) << 16);
                }
                masked_data.push_back(word);
            }
        }
    }
    
public:
    explicit DataGenerator(RegisterInterface& regs) : registers_(regs) {}
    
    void generate_packet(std::vector<uint32_t>& packet) {
        packet.clear();
        
        // Header: magic number
        packet.push_back(MAGIC_NUMBER_LOW);
        packet.push_back(MAGIC_NUMBER_HIGH);
        
        // Header: timestamp
        uint64_t ts = registers_.get_timestamp();
        packet.push_back(static_cast<uint32_t>(ts & 0xFFFFFFFF));
        packet.push_back(static_cast<uint32_t>(ts >> 32));
        
        // Generate sine wave data (same for both real and debug mode)
        // This makes the simulator predictable and easy to validate
        std::vector<uint32_t> full_data;
        std::vector<uint32_t> masked_data;
        
        sine_gen_.generate_packet_data(full_data, 35);
        
        // Apply channel mask
        uint8_t channel_enable = registers_.get_channel_enable();
        apply_channel_mask(full_data, masked_data, channel_enable);
        
        // Add data to packet
        packet.insert(packet.end(), masked_data.begin(), masked_data.end());
    }
    
    bool should_continue_transmission() {
        uint32_t loop_count = registers_.get_loop_count();
        if (loop_count == 0) return true;
        return loop_counter_ < loop_count;
    }
    
    void reset_loop_counter() {
        loop_counter_ = 0;
    }
    
    void increment_loop_counter() {
        loop_counter_++;
    }
    
    uint32_t get_loop_counter() const {
        return loop_counter_;
    }
};

// ============================================================================
// BRAM SIMULATOR
// ============================================================================

class BRAMSimulator {
private:
    std::vector<uint32_t> memory_;
    std::mutex mutex_;
    uint32_t write_ptr_ = 0;
    uint32_t read_ptr_ = 0;
    
public:
    BRAMSimulator() {
        memory_.resize(BRAM_SIZE_WORDS, 0);
    }
    
    bool write_packet(const std::vector<uint32_t>& packet) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Check if there's enough space
        uint32_t available = (read_ptr_ <= write_ptr_) ? 
            (BRAM_SIZE_WORDS - write_ptr_ + read_ptr_) :
            (read_ptr_ - write_ptr_);
            
        if (available < packet.size()) {
            return false;  // BRAM full
        }
        
        // Write packet
        for (uint32_t word : packet) {
            memory_[write_ptr_] = word;
            write_ptr_ = (write_ptr_ + 1) % BRAM_SIZE_WORDS;
        }
        
        return true;
    }
    
    uint32_t get_write_address() const {
        return write_ptr_;
    }
    
    bool read_packet(std::vector<uint32_t>& packet, size_t expected_size) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        uint32_t available = (write_ptr_ >= read_ptr_) ?
            (write_ptr_ - read_ptr_) :
            (BRAM_SIZE_WORDS - read_ptr_ + write_ptr_);
            
        if (available < expected_size) {
            return false;  // Not enough data
        }
        
        packet.clear();
        for (size_t i = 0; i < expected_size; i++) {
            packet.push_back(memory_[read_ptr_]);
            read_ptr_ = (read_ptr_ + 1) % BRAM_SIZE_WORDS;
        }
        
        return true;
    }
    
    uint32_t get_read_address() const {
        return read_ptr_;
    }
};

// ============================================================================
// UDP STREAMER
// ============================================================================

class UDPStreamer {
private:
    SOCKET sock_ = INVALID_SOCKET;
    sockaddr_in dest_addr_;
    std::atomic<uint32_t> packets_sent_{0};
    std::atomic<uint32_t> send_errors_{0};
    std::mutex mutex_;
    
public:
    UDPStreamer() {
        sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock_ == INVALID_SOCKET) {
            throw std::runtime_error("Failed to create UDP socket");
        }
        
        // Default destination
        set_destination("127.0.0.1", UDP_PORT);
    }
    
    ~UDPStreamer() {
        if (sock_ != INVALID_SOCKET) {
            closesocket(sock_);
        }
    }
    
    void set_destination(const std::string& ip, uint16_t port) {
        std::lock_guard<std::mutex> lock(mutex_);
        memset(&dest_addr_, 0, sizeof(dest_addr_));
        dest_addr_.sin_family = AF_INET;
        dest_addr_.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &dest_addr_.sin_addr);
        
        std::cout << "[UDP] Destination set to " << ip << ":" << port << std::endl;
    }
    
    bool send_packet(const std::vector<uint32_t>& packet) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        int result = sendto(sock_, 
                          reinterpret_cast<const char*>(packet.data()),
                          packet.size() * sizeof(uint32_t),
                          0,
                          reinterpret_cast<const sockaddr*>(&dest_addr_),
                          sizeof(dest_addr_));
        
        if (result == SOCKET_ERROR) {
            send_errors_.fetch_add(1);
            return false;
        }
        
        packets_sent_.fetch_add(1);
        return true;
    }
    
    uint32_t get_packets_sent() const { return packets_sent_.load(); }
    uint32_t get_send_errors() const { return send_errors_.load(); }
};

// ============================================================================
// TCP CONTROL SERVER
// ============================================================================

class TCPControlServer {
private:
    RegisterInterface& registers_;
    UDPStreamer& udp_streamer_;
    SOCKET listen_sock_ = INVALID_SOCKET;
    std::atomic<bool> running_{false};
    
    struct CommandPacket {
        uint32_t magic;
        uint32_t cmd_id;
        uint32_t ack_id;
        uint32_t param1;
        uint32_t param2;
    };
    
    void send_ack(SOCKET client_sock, uint32_t ack_id, uint8_t status) {
        uint8_t response[3];
        response[0] = (ack_id >> 8) & 0xFF;
        response[1] = ack_id & 0xFF;
        response[2] = status;
        send(client_sock, reinterpret_cast<char*>(response), 3, 0);
    }
    
    void send_response(SOCKET client_sock, uint32_t ack_id, uint8_t status,
                      const void* data, uint16_t data_len) {
        uint8_t header[5];
        header[0] = (ack_id >> 8) & 0xFF;
        header[1] = ack_id & 0xFF;
        header[2] = status;
        header[3] = (data_len >> 8) & 0xFF;
        header[4] = data_len & 0xFF;
        
        send(client_sock, reinterpret_cast<char*>(header), 5, 0);
        if (data && data_len > 0) {
            send(client_sock, reinterpret_cast<const char*>(data), data_len, 0);
        }
    }
    
    void handle_command(SOCKET client_sock, const CommandPacket& cmd) {
        uint8_t status = ACK_SUCCESS;
        
        switch (cmd.cmd_id) {
            case CMD_START:
                std::cout << "[TCP] Command: START" << std::endl;
                registers_.set_enable_transmission(true);
                break;
                
            case CMD_STOP:
                std::cout << "[TCP] Command: STOP" << std::endl;
                registers_.set_enable_transmission(false);
                break;
                
            case CMD_RESET_TIMESTAMP:
                std::cout << "[TCP] Command: RESET_TIMESTAMP" << std::endl;
                registers_.set_timestamp(0);
                break;
                
            case CMD_SET_LOOP_COUNT:
                std::cout << "[TCP] Command: SET_LOOP_COUNT " << cmd.param1 << std::endl;
                registers_.set_loop_count(cmd.param1);
                break;
                
            case CMD_SET_PHASE:
                std::cout << "[TCP] Command: SET_PHASE " 
                         << (cmd.param1 & 0xFF) << " " << (cmd.param2 & 0xFF) << std::endl;
                registers_.set_phase_select(cmd.param1 & 0xFF, cmd.param2 & 0xFF);
                break;
                
            case CMD_SET_DEBUG_MODE:
                std::cout << "[TCP] Command: SET_DEBUG_MODE " << cmd.param1 << std::endl;
                registers_.set_debug_mode(cmd.param1 != 0);
                break;
                
            case CMD_SET_CHANNEL_ENABLE:
                std::cout << "[TCP] Command: SET_CHANNEL_ENABLE 0x" 
                         << std::hex << (cmd.param1 & 0x0F) << std::dec << std::endl;
                registers_.set_channel_enable(cmd.param1 & 0x0F);
                break;

                            case 0x20: // CMD_LOAD_CONVERT
                std::cout << "[TCP] Command: LOAD_CONVERT" << std::endl;
                // Simulator note: COPI command sequences not needed for synthetic data
                break;
                
            case 0x21: // CMD_LOAD_INIT
                std::cout << "[TCP] Command: LOAD_INIT" << std::endl;
                // Simulator note: COPI command sequences not needed for synthetic data
                break;
                
            case 0x22: // CMD_LOAD_CABLE_TEST
                std::cout << "[TCP] Command: LOAD_CABLE_TEST" << std::endl;
                // Simulator note: COPI command sequences not needed for synthetic data
                break;
                
            case 0x30: // CMD_FULL_CABLE_TEST
                std::cout << "[TCP] Command: FULL_CABLE_TEST" << std::endl;
                // Simulator note: Cable test not applicable - using synthetic data
                // In real hardware, this runs init + 16 phase tests
                break;
                
            case CMD_SET_UDP_DEST: {
                uint32_t ip_net = ntohl(cmd.param1);
                uint16_t port = cmd.param2 & 0xFFFF;
                
                char ip_str[INET_ADDRSTRLEN];
                struct in_addr addr;
                addr.s_addr = htonl(ip_net);
                inet_ntop(AF_INET, &addr, ip_str, INET_ADDRSTRLEN);
                
                std::cout << "[TCP] Command: SET_UDP_DEST " << ip_str << ":" << port << std::endl;
                udp_streamer_.set_destination(ip_str, port);
                break;
            }
                
            case CMD_GET_STATUS: {
                std::cout << "[TCP] Command: GET_STATUS" << std::endl;
                
                // Build status response (86 bytes)
                uint8_t status_data[86];
                memset(status_data, 0, sizeof(status_data));
                
                // Version info (8 bytes)
                uint16_t version = 1;
                uint16_t device_type = 0x1000;
                uint32_t fw_version = 0x01000000;
                memcpy(&status_data[0], &version, 2);
                memcpy(&status_data[2], &device_type, 2);
                memcpy(&status_data[4], &fw_version, 4);
                
                // PL status (22 bytes starting at offset 8)
                uint64_t timestamp = registers_.get_timestamp();
                uint32_t packets_sent = registers_.get_packets_sent();
                uint32_t bram_addr = registers_.get_bram_write_addr();
                uint16_t fifo_count = 0;
                uint8_t state_counter = 0;
                uint8_t cycle_counter = 0;
                uint8_t flags_pl = (registers_.get_transmission_active() ? 0x01 : 0) |
                                   (registers_.get_loop_limit_reached() ? 0x02 : 0);
                
                memcpy(&status_data[8], &timestamp, 8);
                memcpy(&status_data[16], &packets_sent, 4);
                memcpy(&status_data[20], &bram_addr, 4);
                memcpy(&status_data[24], &fifo_count, 2);
                status_data[26] = state_counter;
                status_data[27] = cycle_counter;
                status_data[28] = flags_pl;
                
                // PS status (28 bytes starting at offset 30)
                uint32_t udp_sent = udp_streamer_.get_packets_sent();
                uint32_t udp_errors = udp_streamer_.get_send_errors();
                memcpy(&status_data[38], &udp_sent, 4);
                memcpy(&status_data[42], &udp_errors, 4);
                
                // Configuration (16 bytes starting at offset 58)
                uint32_t loop_count = registers_.get_loop_count();
                uint8_t phase0, phase1;
                registers_.get_phase_select(phase0, phase1);
                uint8_t ch_enable = registers_.get_channel_enable();
                uint8_t debug_mode = registers_.get_debug_mode() ? 1 : 0;
                
                memcpy(&status_data[58], &loop_count, 4);
                status_data[62] = phase0;
                status_data[63] = phase1;
                status_data[64] = ch_enable;
                status_data[65] = debug_mode;
                
                send_response(client_sock, cmd.ack_id, ACK_SUCCESS, status_data, 86);
                return;
            }
                
            default:
                std::cout << "[TCP] Unknown command: 0x" << std::hex << cmd.cmd_id << std::dec << std::endl;
                status = ACK_ERROR;
                break;
        }
        
        send_ack(client_sock, cmd.ack_id, status);
    }
    
    void handle_client(SOCKET client_sock) {
        std::cout << "[TCP] Client connected" << std::endl;
        
        uint8_t buffer[20];
        while (running_.load()) {
            int bytes_received = recv(client_sock, reinterpret_cast<char*>(buffer), 20, 0);
            
            if (bytes_received != 20) break;
            
            CommandPacket cmd;
            memcpy(&cmd, buffer, sizeof(CommandPacket));
            
            if (cmd.magic == CMD_MAGIC) {
                handle_command(client_sock, cmd);
            }
        }
        
        closesocket(client_sock);
        std::cout << "[TCP] Client disconnected" << std::endl;
    }
    
public:
    TCPControlServer(RegisterInterface& regs, UDPStreamer& udp) 
        : registers_(regs), udp_streamer_(udp) {}
    
    ~TCPControlServer() {
        stop();
    }
    
    void start() {
        listen_sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (listen_sock_ == INVALID_SOCKET) {
            throw std::runtime_error("Failed to create TCP socket");
        }
        
        int opt = 1;
        setsockopt(listen_sock_, SOL_SOCKET, SO_REUSEADDR, 
                  reinterpret_cast<char*>(&opt), sizeof(opt));
        
        sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(TCP_PORT);
        
        if (bind(listen_sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
            throw std::runtime_error("Failed to bind TCP socket");
        }
        
        if (listen(listen_sock_, 5) == SOCKET_ERROR) {
            throw std::runtime_error("Failed to listen on TCP socket");
        }
        
        running_.store(true);
        std::cout << "[TCP] Server started on port " << TCP_PORT << std::endl;
        
        // Accept connections in separate threads
        std::thread accept_thread([this]() {
            while (running_.load()) {
                sockaddr_in client_addr;
                socklen_t client_len = sizeof(client_addr);
                
                SOCKET client_sock = accept(listen_sock_, 
                    reinterpret_cast<sockaddr*>(&client_addr), &client_len);
                
                if (client_sock != INVALID_SOCKET) {
                    std::thread(&TCPControlServer::handle_client, this, client_sock).detach();
                }
            }
        });
        accept_thread.detach();
    }
    
    void stop() {
        running_.store(false);
        if (listen_sock_ != INVALID_SOCKET) {
            closesocket(listen_sock_);
            listen_sock_ = INVALID_SOCKET;
        }
    }
};

// ============================================================================
// MAIN SIMULATOR
// ============================================================================

class IntanSimulator {
private:
    NetworkInit net_init_;
    RegisterInterface registers_;
    DataGenerator data_gen_;
    BRAMSimulator bram_;
    UDPStreamer udp_streamer_;
    TCPControlServer tcp_server_;
    
    std::atomic<bool> running_{false};
    std::thread generation_thread_;
    std::thread streaming_thread_;
    
    void generation_loop() {
        using namespace std::chrono;
        
        auto next_cycle_time = steady_clock::now();
        const auto cycle_interval = microseconds(1000000 / SAMPLING_RATE_HZ);
        
        while (running_.load()) {
            bool should_generate = registers_.get_enable_transmission();
            
            // Handle reset timestamp - only when transmission is disabled
            if (!should_generate && registers_.get_reset_timestamp()) {
                registers_.set_timestamp(0);
                registers_.set_reset_timestamp(false);
                data_gen_.reset_loop_counter();
                registers_.set_loop_limit_reached(false);
                std::cout << "[PL] Timestamp reset to 0" << std::endl;
            }
            
            // Generate packet if transmission is enabled
            if (should_generate && data_gen_.should_continue_transmission()) {
                std::vector<uint32_t> packet;
                data_gen_.generate_packet(packet);
                
                if (bram_.write_packet(packet)) {
                    registers_.increment_packets_sent();
                    registers_.set_bram_write_addr(bram_.get_write_address());
                    data_gen_.increment_loop_counter();
                    
                    // Check loop limit
                    if (!data_gen_.should_continue_transmission()) {
                        registers_.set_loop_limit_reached(true);
                        registers_.set_enable_transmission(false);
                        std::cout << "[PL] Loop limit reached, stopping transmission" << std::endl;
                    }
                }
                
                registers_.set_transmission_active(true);
            } else {
                registers_.set_transmission_active(false);
            }
            
            // CRITICAL: Timestamp ALWAYS increments at 30 kHz, regardless of transmission state
            // This matches the hardware behavior where the timestamp counter is free-running
            registers_.increment_timestamp();
            
            // Wait for next cycle time
            next_cycle_time += cycle_interval;
            std::this_thread::sleep_until(next_cycle_time);
        }
    }
    
    void streaming_loop() {
        while (running_.load()) {
            uint8_t channel_enable = registers_.get_channel_enable();
            uint32_t data_words = calculate_data_words(channel_enable);
            uint32_t packet_size = PACKET_HEADER_WORDS + data_words;
            
            std::vector<uint32_t> packet;
            if (bram_.read_packet(packet, packet_size)) {
                udp_streamer_.send_packet(packet);
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }
    }
    
    uint32_t calculate_data_words(uint8_t channel_enable) {
        int num_channels = 0;
        if (channel_enable & 0x01) num_channels++;
        if (channel_enable & 0x02) num_channels++;
        if (channel_enable & 0x04) num_channels++;
        if (channel_enable & 0x08) num_channels++;
        
        if (num_channels == 0) return 70;
        
        uint32_t total_16bit_words = 35 * num_channels;
        return (total_16bit_words + 1) / 2;
    }
    
public:
    IntanSimulator() 
        : data_gen_(registers_),
          tcp_server_(registers_, udp_streamer_) {}
    
    void start() {
        running_.store(true);
        
        tcp_server_.start();
        
        generation_thread_ = std::thread(&IntanSimulator::generation_loop, this);
        streaming_thread_ = std::thread(&IntanSimulator::streaming_loop, this);
        
        std::cout << "\n=== Intan Interface Simulator ===\n";
        std::cout << "TCP Control: port " << TCP_PORT << "\n";
        std::cout << "UDP Stream: port " << UDP_PORT << "\n";
        std::cout << "Sampling Rate: " << SAMPLING_RATE_HZ << " Hz\n";
        std::cout << "Press Ctrl+C to stop\n" << std::endl;
    }
    
    void stop() {
        running_.store(false);
        
        if (generation_thread_.joinable()) generation_thread_.join();
        if (streaming_thread_.joinable()) streaming_thread_.join();
        
        tcp_server_.stop();
        
        std::cout << "\nSimulator stopped" << std::endl;
    }
    
    void wait() {
        while (running_.load()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
};

// ============================================================================
// MAIN
// ============================================================================

std::atomic<bool> keep_running{true};

void signal_handler(int) {
    keep_running.store(false);
}

int main() {
    try {
        IntanSimulator simulator;
        
        simulator.start();
        
        // Wait for Ctrl+C
        while (keep_running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        simulator.stop();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
