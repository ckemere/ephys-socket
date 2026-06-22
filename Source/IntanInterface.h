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

        // Aux command sequencer status (firmware >= aux-seq-v2; 98-byte
        // status response). hasAuxStatus is false when talking to older
        // 86-byte firmware -- the fields below are then all zero.
        bool hasAuxStatus;
        bool auxSeqEnabled;       // per-packet latched aux_seq_en
        bool fastSettleActive;    // live RHD Reg-0 D5 state
        bool digoutState;         // live auxout mirror bit
        bool dspResetActive;      // CONVERT bit-H forcing active
        uint8_t auxBankActive;    // [2:0] active bank per slot
        uint8_t auxIndex[3];      // per-slot sequence index
        uint32_t auxReadResult;   // last injected command's response {cipo1,cipo0}

        // CTRL_REG_22 readback (firmware 65d5fb5+, status >= 126 bytes). When
        // hasAuxCtrl is true, the host can read back the FULL fast-settle /
        // DSP / digout configuration -- including the GPIO pin select -- and
        // so the editor can sync its TTL combo to whatever the device is in.
        bool hasAuxCtrl;
        uint32_t auxCtrlRaw;       // raw CTRL_REG_22 (for re-decoding if needed)
        bool fsSwLevel;            // amp fast settle: software level bit
        bool fsGpioEn;             // amp fast settle: GPIO trigger enable
        uint8_t fsGpioPin;         // amp fast settle: digital_in pin select [2:0]
        bool dspSwLevel;           // DSP reset: software level bit
        bool dspGpioEn;            // DSP reset: GPIO trigger enable
        uint8_t dspGpioPin;        // DSP reset: digital_in pin select [2:0]
        bool digoutSwLevel;        // digout: software level bit
        bool digoutGpioEn;         // digout: GPIO trigger enable
        uint8_t digoutGpioPin;     // digout: digital_in pin select [2:0]
        uint8_t reg3Static;        // RHD Reg-3 static (host-owned bits D7..D1)

        // RHD chip register shadow (firmware 7fb41dc+, status >= 148 bytes).
        // Commanded state of regs 0..21, seeded from the init sequence and
        // updated on WRITE_REGISTER. Regs 0 and 3 are owned by the PL
        // override layer -- their LIVE values are in aux_ctrl/aux_flags, the
        // shadow keeps the init base for those two. hasRhdRegMirror is false
        // on older firmware; rhdReg[] is then zero.
        bool hasRhdRegMirror;
        uint8_t rhdReg[22];

        // LFP/DSP engine config + status (firmware 0e99881+, status == 160).
        // The engine emits a SEPARATE UDP stream on port 5001 (broadband on
        // 5000 is untouched). Decimated samples = popcount(lane_mask) x 32
        // amplifier channels per frame, at 30000/decim_R Hz. Configure with
        // CMD_LFP_SET_CHANNELS / CMD_LFP_SET_PARAMS / CMD_LFP_WRITE_COEF,
        // then CMD_LFP_ENABLE 1. hasLfpStatus is false on older firmware.
        bool hasLfpStatus;
        bool lfpEnabled;
        uint8_t lfpLaneMask;       // which of the 8 stream bits are LFP-filtered
        uint8_t lfpDecimR;         // decimation factor (broadband sample / output sample)
        uint8_t lfpNumTaps;        // active FIR length
        uint32_t lfpPacketsSent;   // LFP UDP frames emitted (firmware-side counter)
        bool lfpOverrun;           // sticky compute-overrun flag

        // STFT/Tier-2 engine config + status (firmware fw>=1.3, status >= 184).
        // The engine emits ONE jumbo UDP datagram per STFT frame on port 5003
        // (independent of broadband + LFP). K lanes per frame, each with
        // (N/2+1) complex float32 bins (Hermitian half). hasStftStatus is
        // false on older firmware.
        bool hasStftStatus;
        bool stftEnabled;
        uint8_t  stftNfftLog2;     // log2(N); N = 1 << stftNfftLog2
        uint8_t  stftK;            // analyzed lanes (build param, matches the PL)
        bool     stftOverflow;     // sticky pass-overflow flag
        uint16_t stftHop;          // LFP frames between STFT passes
        uint32_t stftFrameSeq;     // completed STFT passes (firmware counter)
        uint32_t stftPacketsSent;  // STFT UDP packets emitted

        // Helper methods
        std::string getFirmwareVersionString() const;
        std::string getChannelEnableString() const;

        /** Multi-line human-readable dump of the full status (for console) */
        std::string getSummary() const;
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
        
        // CIPO1 detection (port A)
        bool cipo1Detected;
        ChipType cipo1ChipType;
        bool cipo1HasDdr;
        double cipo1Score;

        // Port B detection (populated by Phase 2 per-port RESCAN; false/NONE until then)
        bool portBCipo0Detected = false;
        ChipType portBCipo0ChipType = ChipType::NONE;
        bool portBCipo0HasDdr = false;
        bool portBCipo1Detected = false;
        ChipType portBCipo1ChipType = ChipType::NONE;
        bool portBCipo1HasDdr = false;

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
     * @brief LFP frame metadata (decoded once per UDP packet on port 5001).
     */
    struct LfpFrame {
        uint64_t timestamp;        // frame_seq * decim_R (aligns with broadband ts)
        uint32_t frameSequence;    // monotonic LFP frame counter (for drop detection)
        uint8_t  laneMask;         // streams whose 32 amp channels are in this frame
        uint8_t  decimR;
        uint8_t  numTaps;
        bool     overrun;          // sticky compute-overrun flag
        const uint16_t* samples;   // popcount(laneMask)*32 offset-binary 16-bit samples
                                   //   layout: per enabled lane (low->high bit), 32 ch
        size_t   sampleCount;      // == popcount(laneMask) * 32
    };

    /**
     * @brief Callback type for receiving an LFP frame (UDP port 5001).
     * Invoked from the LFP listener thread.
     */
    using LfpDataCallback = std::function<void(const LfpFrame&)>;

    /**
     * @brief STFT frame metadata (decoded once per UDP packet on port 5003).
     *
     * Wire layout (32-bit LE words):
     *   0-1 magic {0x5DEC7A00, 0xCAFEBABE}
     *   2-3 64-bit timestamp (~LFP frame index of this pass)
     *   4   [7:0] nfft_log2 | [15:8] K | [31:24] flags (bit0 = overflow)
     *   5   32-bit frame sequence (mod 2^30)
     *   6   [15:0] nbins (= N/2+1) | [31:16] hop
     *   7   reserved
     *   8+  per lane (lane-major), nbins complex float32 (re, im)
     */
    struct StftFrame {
        uint64_t timestamp;
        uint32_t frameSequence;
        uint8_t  nfftLog2;       // N = 1 << nfftLog2
        uint8_t  K;              // analyzed lanes (one per "channel" in the lane-major payload)
        uint16_t hop;            // LFP frames between passes (time-axis spacing)
        uint16_t nbins;          // N/2 + 1 (the Hermitian half that's actually sent)
        bool     overflow;
        // Payload pointer + bin/lane stride. The view is valid only for the
        // callback's lifetime; copy whatever you need. Layout (lane-major):
        //   bins[lane L, bin b].re = samples[L * nbins * 2 + b * 2 + 0]
        //   bins[lane L, bin b].im = samples[L * nbins * 2 + b * 2 + 1]
        const float* samples;
        size_t   sampleCount;    // == K * nbins * 2
    };

    /**
     * @brief Callback type for receiving an STFT frame (UDP port 5003).
     * Invoked from the STFT listener thread.
     */
    using StftDataCallback = std::function<void(const StftFrame&)>;

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
     * @brief Set CIPO phase delay compensation for port B (second cable)
     *
     * Port B has independent phase delays. On firmware without dual-port
     * support, this returns false and has no effect.
     *
     * @param phase2 Phase delay for port B CIPO0 (0-15)
     * @param phase3 Phase delay for port B CIPO1 (0-15)
     * @return true if command succeeded
     */
    bool setPhaseSelectB(uint8_t phase2, uint8_t phase3);

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
    // AUX COMMAND SEQUENCER / OVERRIDE LAYER (firmware aux-seq-v2)
    // ========================================================================
    //
    // The PL can source the 3 aux COPI positions (cycles 32..34) from
    // programmable, double-buffered command banks instead of the static
    // table. Banks are uploadable DURING acquisition; a bank select swaps
    // atomically at a packet boundary and the firmware confirms the swap
    // before ACKing. When the sequencer is enabled, packets self-describe
    // their aux contents via a command echo in header words 4/5 (see
    // MicroZedIntanInterface docs/command-bank-design.md).

    /** RHD SPI command encoders (datasheet bit layouts) */
    static uint16_t rhdConvert(uint8_t channel, bool dspResetBit = false) {
        return static_cast<uint16_t>(((channel & 0x3F) << 8) | (dspResetBit ? 1 : 0));
    }
    static uint16_t rhdWrite(uint8_t reg, uint8_t value) {
        return static_cast<uint16_t>(0x8000 | ((reg & 0x3F) << 8) | value);
    }
    static uint16_t rhdRead(uint8_t reg) {
        return static_cast<uint16_t>(0xC000 | ((reg & 0x3F) << 8));
    }

    /**
     * @brief Upload a command program (and its length record) into one bank.
     *
     * @param slot 0..2 (slot 0 -> COPI cycle 32, real-time control;
     *             slot 1 -> cycle 33, ADC/accelerometer;
     *             slot 2 -> cycle 34, config/housekeeping)
     * @param bank 0 or 1 (write the STANDBY bank while the other plays)
     * @param commands 1..64 RHD command words
     * @param loopIndex index the program wraps back to (entries before it
     *                  play once -- a run-once preamble)
     * @return true if all words were acknowledged
     */
    bool auxUploadBank(int slot, int bank, const std::vector<uint16_t>& commands,
                       int loopIndex = 0);

    /**
     * @brief Atomically swap a slot to a bank (live, at a packet boundary).
     * The firmware polls bank_active and only ACKs once the swap landed.
     */
    bool auxBankSelect(int slot, int bank);

    /** @brief Enable/disable the aux command sequencer (default off). */
    bool auxSeqEnable(bool enable);

    /**
     * @brief Configure amplifier fast settle (RHD Reg-0 D5).
     *
     * On a level change the PL injects WRITE(0,0xFE)/WRITE(0,0xDE) into the
     * slot-0 command of the transition packet. Requires the aux sequencer
     * to be enabled. Software level and GPIO trigger may be combined (OR).
     *
     * @param softwareLevel software fast-settle level
     * @param gpioEnable    follow a digital_in pin as well
     * @param gpioPin       which digital_in pin (0-7)
     * @param dspReset      also force the CONVERT bit-H (DSP HPF reset)
     *                      while settling is requested in software
     */
    bool setFastSettle(bool softwareLevel, bool gpioEnable = false,
                       uint8_t gpioPin = 0, bool dspReset = false);

    /**
     * @brief Read an RHD register at runtime (injected via slot 2's
     * sequencer position for exactly one packet; the loaded program is not
     * perturbed). Requires streaming + sequencer enabled.
     *
     * @param reg RHD register address (0-63)
     * @param cipo0Value chip on CIPO0's response
     * @param cipo1Value chip on CIPO1's response
     */
    bool readRegister(uint8_t reg, uint16_t& cipo0Value, uint16_t& cipo1Value);

    // ========================================================================
    // LFP / DSP ENGINE (firmware >= 1.2; second UDP stream on port 5001)
    // ========================================================================
    //
    // The engine LP-filters and decimates the amplifier streams (the same 8
    // bit lanes as the broadband stream), emitting a parallel UDP datagram
    // per output frame on port 5001 -- broadband on 5000 is untouched.
    //
    // Configure order: lfpEnable(false) -> lfpSetChannels(mask)
    //   -> lfpSetParams(decim_R, num_taps) -> lfpUploadCoefs(coefs)
    //   -> lfpEnable(true).
    // (Same sequence as remote/net.py's configure_lfp().)

    /** Enable / disable the engine (also starts/stops UDP emission). */
    bool lfpEnable(bool on);

    /** Pick which of the 8 broadband lanes to LFP-filter. */
    bool lfpSetChannels(uint8_t laneMask);

    /** Set decimation factor (broadband_rate / output_rate) and FIR length. */
    bool lfpSetParams(uint8_t decimR, uint8_t numTaps);

    /** Upload one FIR tap (Q1.17 signed, 18-bit). The first call of a fresh
     *  upload should pass `clearFirst=true` to reset the coefficient pointer. */
    bool lfpWriteCoef(bool clearFirst, int32_t coef18);

    /** Convenience: upload a full coefficient set in order (first call clears). */
    bool lfpUploadCoefs(const std::vector<int32_t>& coefs);

    /** Default LFP-engine parameters (mirror remote/net.py configure_lfp). */
    struct LfpDefaults {
        static constexpr uint8_t LANE_MASK = 0x0F;     // port A, all 4 streams
        static constexpr uint8_t DECIM_R   = 15;       // 30000 / 15 = 2000 Hz
        static constexpr uint8_t NUM_TAPS  = 128;
        static constexpr double  CUTOFF_HZ = 600.0;
        static constexpr double  FS        = 30000.0;
        static constexpr int     COEF_FRAC = 17;       // Q1.17
    };

    /** Design a windowed-sinc (Hamming) low-pass FIR with unity DC gain,
     *  quantized to Q1.17 signed (18-bit). Exactly mirrors net.py's
     *  design_lfp_lowpass(); use for the default configure path. */
    static std::vector<int32_t> lfpDesignLowpass(int numTaps,
                                                 double cutoffHz,
                                                 double fs = LfpDefaults::FS);

    // ========================================================================
    // STFT / Tier-2 ENGINE (firmware >= 1.3; jumbo UDP on port 5003)
    // ========================================================================
    //
    // The STFT engine takes K LFP lanes (one source channel per lane), windows
    // each with a Q15 Hann taper, and emits a sequence of (N/2+1)-bin complex
    // float32 spectra over UDP. Configure order: stftEnable(false) ->
    // stftSetParams(nfft_log2, hop) -> stftUploadChannels(...) ->
    // stftUploadWindow(...) -> stftEnable(true). The host designs the window;
    // stftDesignHann() is the reference design used by net.py.

    struct StftDefaults {
        static constexpr uint8_t  NFFT_LOG2 = 6;     // N = 64
        static constexpr uint16_t HOP       = 1;     // every LFP frame
        static constexpr int      Q15_FULL  = 32767;
    };

    /** Enable / disable the engine (also starts/stops UDP emission). */
    bool stftEnable(bool on);

    /** Set FFT length (log2) and pass-to-pass hop. */
    bool stftSetParams(uint8_t nfftLog2, uint16_t hop);

    /** Push one lane's source-channel selection. `clearFirst=true` on the
     *  first call of a fresh upload resets the lane pointer. */
    bool stftSetChannelEntry(bool clearFirst, uint8_t sourceChannel);

    /** Convenience: upload K lane selections in one go (first call clears). */
    bool stftUploadChannels(const std::vector<uint8_t>& channels);

    /** Push one window tap (Q15 signed). `clearFirst=true` resets pointer. */
    bool stftWriteWindowTap(bool clearFirst, int16_t coefQ15);

    /** Convenience: upload N window taps in one go (first call clears). */
    bool stftUploadWindow(const std::vector<int16_t>& taps);

    /** Hann window, signed Q15. Exactly mirrors net.py's design_stft_window. */
    static std::vector<int16_t> stftDesignHann(int n);

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
     * @brief Register callback for LFP frames (UDP port 5001).
     *
     * Invoked from the LFP listener thread for every well-formed frame.
     * The pointer / count in LfpFrame are valid only for the duration of
     * the callback; copy what you need.
     */
    void setLfpDataCallback(LfpDataCallback callback);

    /**
     * @brief Register callback for STFT frames (UDP port 5003).
     *
     * Invoked from the STFT listener thread for every well-formed jumbo
     * datagram. The samples pointer is valid only for the duration of the
     * callback; copy what you need.
     */
    void setStftDataCallback(StftDataCallback callback);

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
