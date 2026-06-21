#include "IntanSocket.h"
#include "IntanSocketEditor.h"

#include <sstream>

using namespace IntanSocketNode;

DataThread* IntanSocket::createDataThread(SourceNode* sn)
{
    return new IntanSocket(sn);
}

IntanSocket::IntanSocket(SourceNode* sn) 
    : DataThread(sn)
{
    device_ip = DEFAULT_DEVICE_IP;
    tcp_port = DEFAULT_TCP_PORT;
    udp_port = DEFAULT_UDP_PORT;
    data_scale = DEFAULT_DATA_SCALE;
    aux_data_scale = DEFAULT_AUX_DATA_SCALE;
    channel_enable_mask = 0x0F;  // All channels enabled by default
    totalSamples = 0;
    eventState = 0;
    hasError = false;
    debugMode = false;

    // Create IntanInterface (will throw if can't connect, so wrap in try)
    intanInterface = nullptr;
    
    // Pre-allocate buffers for 4 channels (maximum)
    sourceBuffers.add(new DataBuffer(4, SAMPLE_RATE * bufferSizeInSeconds));
}

std::unique_ptr<GenericEditor> IntanSocket::createEditor(SourceNode* sn)
{
    std::unique_ptr<IntanSocketEditor> editor = std::make_unique<IntanSocketEditor>(sn, this);
    return editor;
}

IntanSocket::~IntanSocket()
{
    if (intanInterface && intanInterface->foundInputSource())
    {
        intanInterface->stopAcquisition();
    }
}

void IntanSocket::registerParameters()
{
    addStringParameter(Parameter::PROCESSOR_SCOPE, 
                      "device_ip", 
                      "Device IP", 
                      "IP address of Intan device", 
                      DEFAULT_DEVICE_IP,
                      true);
    
    addIntParameter(Parameter::PROCESSOR_SCOPE,
                   "tcp_port",
                   "TCP Port",
                   "TCP command port",
                   DEFAULT_TCP_PORT,
                   MIN_PORT,
                   MAX_PORT);
    
    addIntParameter(Parameter::PROCESSOR_SCOPE,
                   "udp_port",
                   "UDP Port",
                   "UDP data port",
                   DEFAULT_UDP_PORT,
                   MIN_PORT,
                   MAX_PORT);
    
    addFloatParameter(Parameter::PROCESSOR_SCOPE, 
                     "data_scale", 
                     "Scale", 
                     "Data scale (µV per bit)", 
                     "", 
                     DEFAULT_DATA_SCALE, 
                     MIN_DATA_SCALE, 
                     MAX_DATA_SCALE, 
                     0.01f);
    addFloatParameter(Parameter::PROCESSOR_SCOPE, 
                     "aux_data_scale",
                     "Scale",
                     "Aux data scale (uV per bit)",
                     "", 
                     DEFAULT_AUX_DATA_SCALE, 
                     MIN_DATA_SCALE, 
                     MAX_DATA_SCALE, 
                     0.01f);
}

void IntanSocket::disconnectDevice()
{
    if (intanInterface)
    {
        intanInterface->stopAcquisition();
        intanInterface.reset();
    }
    
    getParameter("device_ip")->setEnabled(true);

    if (sn->getEditor() != nullptr)
        static_cast<IntanSocketEditor*>(sn->getEditor())->disconnected();
}

bool IntanSocket::connectDevice(bool printOutput)
{
    try
    {
        // Create IntanInterface with configured ports
        intanInterface = std::make_unique<IntanInterface>(
            device_ip.toStdString(),
            tcp_port,   // Use parameter value
            udp_port    // Use parameter value
        );
        
        if (!intanInterface->foundInputSource())
        {
            if (printOutput)
            {
                LOGE("Failed to connect to Intan device at ", device_ip);
                CoreServices::sendStatusMessage("Intan: Connection failed.");
            }
            intanInterface.reset();
            return false;
        }
        
        if (!intanInterface->isReady())
        {
            if (printOutput)
            {
                LOGE("Intan device not ready");
                CoreServices::sendStatusMessage("Intan: Device not ready.");
            }
            intanInterface.reset();
            return false;
        }

        // Set up data callback
        intanInterface->setDataCallback(
            [this](const uint32_t* data, size_t words, uint64_t timestamp) {
                processDataPacket(data, words, timestamp);
            }
        );

        // LFP callback: each frame is one decimated sample across all enabled
        // LFP channels. Always wired -- silently does nothing until the LFP
        // engine is enabled in the firmware.
        intanInterface->setLfpDataCallback(
            [this](const IntanInterface::LfpFrame& f) {
                processLfpFrame(f);
            }
        );

        // Set up error callback
        intanInterface->setErrorCallback(
            [this, printOutput](const std::string& error) {
                if (printOutput)
                {
                    LOGE("Intan error: ", error.c_str());
                }
                hasError = true;
            }
        );

        // Pull authoritative state from the device. The firmware retains the
        // channel-enable mask, debug mode, phase delays, and aux-sequencer
        // state across plugin disconnects (they live in PL registers, no NVM
        // but they survive the lifetime of the firmware). Adopt whatever the
        // device is in and let the editor mirror it. After any successful
        // RESCAN, reconnecting restores the prior chip indicators and channel
        // count without re-running detection.
        //
        // Exception: if the firmware just booted, channel_enable=0 (no streams
        // configured yet). Publishing a 0-channel DataStream crashes downstream
        // plugins on the next updateSignalChain (LFP Viewer dereferences a
        // null stream in saveParameters). Seed the firmware with 0x0F so the
        // signal chain has *something* valid -- the chip indicators still stay
        // dark because no real RESCAN has happened, so the user is prompted to
        // RESCAN in the obvious way.
        IntanInterface::DeviceStatus status;
        if (!intanInterface->getStatus(status))
        {
            // The board responded to the constructor's getStatus but not this
            // one -- likely a half-up TCP stack right after boot. With the
            // recv timeout in place we don't hang, but we DO need to refuse
            // the connection cleanly so the user can retry.
            if (printOutput)
            {
                LOGE("Intan: status read failed -- board may be still booting. "
                     "Wait until the ethernet activity LED is steady and try "
                     "CONNECT again.");
                CoreServices::sendStatusMessage("Intan: not ready, retry CONNECT");
            }
            intanInterface.reset();
            return false;
        }

        if (status.channelEnable == 0)
        {
            if (!intanInterface->setChannelEnable(0x0F))
            {
                if (printOutput)
                    LOGE("Intan: setChannelEnable failed during initial seed");
                intanInterface.reset();
                return false;
            }
            Thread::sleep(10);
            if (!intanInterface->getStatus(status))
            {
                if (printOutput)
                    LOGE("Intan: status re-read failed after channel-enable seed");
                intanInterface.reset();
                return false;
            }
        }

        channel_enable_mask = status.channelEnable;
        num_channels = calculateNumChannels(channel_enable_mask);
        debugMode = (status.debugMode != 0);

        // Aux sequencer state (already persisted across reconnect)
        auxSeqMode   = status.hasAuxStatus && status.auxSeqEnabled;

        // LFP/DSP engine state -- mirror from device. If LFP is enabled in
        // the firmware (configured + started via the external tool), we'll
        // publish a SECOND DataStream in updateSettings sized for the active
        // lane mask + decimation rate. If disabled, no LFP stream.
        if (status.hasLfpStatus && status.lfpEnabled
            && status.lfpLaneMask != 0 && status.lfpDecimR != 0)
        {
            lfp_enabled  = true;
            lfp_lane_mask = status.lfpLaneMask;
            lfp_decim_R  = status.lfpDecimR;
            lfp_num_taps = status.lfpNumTaps;
            int popcount = 0;
            for (int b = 0; b < 8; ++b)
                popcount += ((lfp_lane_mask >> b) & 1);
            lfp_num_channels = popcount * 32;
        }
        else
        {
            lfp_enabled = false;
            lfp_lane_mask = 0;
            lfp_decim_R = 0;
            lfp_num_taps = 0;
            lfp_num_channels = 0;
        }

        // Fast-settle / TTL state: prefer the new aux_ctrl readback
        // (firmware 65d5fb5+) which surfaces the actual SW level and TTL
        // pin select. On older firmware, fall back to the live fs_active
        // bit (SW state isn't directly observable) and assume no TTL pin.
        if (status.hasAuxCtrl)
        {
            fastSettleSw  = status.fsSwLevel;
            fastSettleTTL = status.fsGpioEn ? (int)status.fsGpioPin : -1;
        }
        else
        {
            fastSettleSw  = status.hasAuxStatus && status.fastSettleActive;
            fastSettleTTL = -1;
        }

        if (printOutput)
        {
            LOGC("Connected to Intan device - mask 0x",
                 String::toHexString((int)channel_enable_mask),
                 " (", num_channels, " channels), debug=",
                 debugMode ? "ON" : "OFF");
            LOGC("Firmware: ", status.getFirmwareVersionString().c_str());
            CoreServices::sendStatusMessage("Intan: Connected successfully.");
        }

        getParameter("device_ip")->setEnabled(false);

        if (sn->getEditor() != nullptr)
        {
            auto* editor = static_cast<IntanSocketEditor*>(sn->getEditor());
            editor->connected();
            // Mirror the device's persisted state into the UI (chip indicators
            // from channel_enable, DBG button label from debug_mode). On a fresh
            // boot mask=0 -> no chips shown -> user clicks RESCAN. On reconnect
            // after a previous RESCAN, the prior indicators come back for free.
            editor->syncFromDeviceState(channel_enable_mask, debugMode);
        }

        return true;
    }
    catch (const std::exception& e)
    {
        if (printOutput)
        {
            LOGE("Exception connecting to Intan: ", e.what());
            CoreServices::sendStatusMessage("Intan: Connection error.");
        }
        intanInterface.reset();
        return false;
    }
}

bool IntanSocket::errorFlag()
{
    return hasError.load();
}

void IntanSocket::updateSettings(OwnedArray<ContinuousChannel>* continuousChannels,
                                 OwnedArray<EventChannel>* eventChannels,
                                 OwnedArray<SpikeChannel>* spikeChannels,
                                 OwnedArray<DataStream>* sourceStreams,
                                 OwnedArray<DeviceInfo>* devices,
                                 OwnedArray<ConfigurationObject>* configurationObjects)
{
    continuousChannels->clear();
    eventChannels->clear();
    devices->clear();
    spikeChannels->clear();
    configurationObjects->clear();
    sourceStreams->clear();

    bool generatesTimestamps = true;

    DataStream::Settings dataStreamSettings{
        "IntanStream",
        "Data from Intan neural interface",
        "intan.data",
        SAMPLE_RATE,
        generatesTimestamps
    };

    DataStream* stream = new DataStream (dataStreamSettings);

    sourceStreams->add(stream);
    
    // ------------------------------------------------------------------
    // Build the channel layout from the active stream mask (8-bit, dual-port).
    //
    // Bits 0-3 = port A: bit0=A_CIPO0_REG, bit1=A_CIPO0_DDR,
    //                    bit2=A_CIPO1_REG, bit3=A_CIPO1_DDR
    // Bits 4-7 = port B: bit4=B_CIPO0_REG, bit5=B_CIPO0_DDR,
    //                    bit6=B_CIPO1_REG, bit7=B_CIPO1_DDR
    //
    // The PL emits per acquisition cycle the enabled 16-bit segments in
    // bit order (0 to 7). Each stream carries 32 amplifier channels; only
    // the four "regular" streams additionally carry 3 aux inputs (the DDR
    // streams just resample the same aux, so their aux samples are dropped).
    //
    // Channel order here MUST match the fill order in updateBuffer():
    //   [stream0 CH1..32][stream1 CH1..32]...  then  [aux per regular stream]
    //
    // For single-port 0x0F this is byte-identical to the previous layout:
    //   A_CH1..A_CH128  then  A_AUX0_1..A_AUX0_3, A_AUX1_1..A_AUX1_3
    // (prefix "A_" added to names, but count and order unchanged)
    // ------------------------------------------------------------------
    int n_streams = countStreams(channel_enable_mask);
    int n_aux_banks = countAuxBanks(channel_enable_mask);

    if (n_streams == 0)
        LOGE("No channels enabled!");

    int n_neural_channels = n_streams * 32;
    num_channels = n_neural_channels + n_aux_banks * 3;

    // Resize buffer to exactly the number of channels we publish
    sourceBuffers[0]->resize(num_channels > 0 ? num_channels : 1,
                             SAMPLE_RATE * bufferSizeInSeconds);

    // Neural channels, grouped per stream (de-interleaved).
    // Streams for bits 0-3 get port prefix "A_", bits 4-7 get "B_".
    // Numbering is sequential within each port (CH1..CH32 per stream,
    // continuing across streams of the same port).
    int portA_neuralIdx = 0;  // running counter for port A channels
    int portB_neuralIdx = 0;  // running counter for port B channels
    for (int b = 0; b < 8; ++b)
    {
        if ((channel_enable_mask & (1 << b)) == 0)
            continue;

        String portPrefix = (b < 4) ? "A_" : "B_";

        for (int k = 0; k < 32; ++k)
        {
            int chanNum;
            if (b < 4)
                chanNum = ++portA_neuralIdx;
            else
                chanNum = ++portB_neuralIdx;

            ContinuousChannel::Settings chanSettings{
                ContinuousChannel::Type::ELECTRODE,
                portPrefix + "CH" + String(chanNum),
                "Intan neural data channel",
                "intan.continuous.ephys",
                data_scale,
                stream
            };

            continuousChannels->add(new ContinuousChannel(chanSettings));
            continuousChannels->getLast()->setUnits("uV");
        }
    }

    // Aux channels: only for the "regular" streams, 3 per bank.
    // Bank mapping:
    //   bit 0 (A_CIPO0_REG) -> bank 0 -> A_AUX0_1..A_AUX0_3
    //   bit 2 (A_CIPO1_REG) -> bank 1 -> A_AUX1_1..A_AUX1_3
    //   bit 4 (B_CIPO0_REG) -> bank 2 -> B_AUX0_1..B_AUX0_3
    //   bit 6 (B_CIPO1_REG) -> bank 3 -> B_AUX1_1..B_AUX1_3
    // Bit positions of the four regular streams: 0, 2, 4, 6
    static const int auxRegularBits[4] = {0, 2, 4, 6};
    static const char* auxBankNames[4] = {"A_AUX0", "A_AUX1", "B_AUX0", "B_AUX1"};

    for (int bankIdx = 0; bankIdx < 4; ++bankIdx)
    {
        int b = auxRegularBits[bankIdx];
        if ((channel_enable_mask & (1 << b)) == 0)
            continue;

        for (int a = 0; a < 3; ++a)
        {
            ContinuousChannel::Settings channelSettings {
                ContinuousChannel::AUX,
                String(auxBankNames[bankIdx]) + "_" + String(a + 1),
                "Intan aux input channel",
                "intan.continuous.aux",
                aux_data_scale,
                stream
            };

            continuousChannels->add (new ContinuousChannel (channelSettings));
            // Aux samples are published as raw signed ADC counts (bitVolts=1.0);
            // expose them in arbitrary units with a range that covers the full
            // signed-16-bit window. Accelerometer signals sit well inside this,
            // and the LFP viewer's range selector zooms in from there.
            continuousChannels->getLast()->setUnits ("a.u.");
            continuousChannels->getLast()->inputRange = {-32768.0f, 32767.0f};
        }
    }

    // ============================================================
    // TTL EVENT CHANNELS
    // ============================================================
    EventChannel::Settings settings {
        EventChannel::Type::TTL,
        "Acquisition Board TTL Input",
        "Events on digital input lines of an Open Ephys Acquisition Board",
        "acq-board.rhythm.events",
        stream,
        8
    };

    eventChannels->add (new EventChannel (settings));

    LOGC("Configured ", n_neural_channels, " channels");

    // ------------------------------------------------------------------
    // SECOND DATASTREAM: decimated LFP band (firmware LFP engine).
    // Created only when the engine is enabled in the firmware at connect-
    // time. Sample rate = SAMPLE_RATE / lfp_decim_R; channel count = popcount
    // of the LFP lane mask * 32 (amplifier channels only -- no aux). The
    // user configures + enables the engine via an out-of-band tool (e.g.
    // net.py configure_lfp), then reconnects the plugin to publish the
    // stream into Open Ephys.
    //
    // (Pattern follows the Neuropixels plugin's AP / LFP stream pairing:
    //  one DataStream per band, parallel sourceBuffers index.)
    // ------------------------------------------------------------------
    if (lfp_enabled && lfp_num_channels > 0 && lfp_decim_R > 0)
    {
        float lfpSampleRate = SAMPLE_RATE / (float)lfp_decim_R;

        DataStream::Settings lfpSettings{
            "IntanLFP",
            "Decimated LFP band from Intan neural interface",
            "intan.data.lfp",
            lfpSampleRate,
            generatesTimestamps
        };
        DataStream* lfpStream = new DataStream(lfpSettings);
        sourceStreams->add(lfpStream);

        // sourceBuffers is owned by the plugin (not auto-managed by OE) and
        // the constructor only creates [0] for the broadband stream. Add the
        // LFP buffer on first connect with LFP enabled; resize on subsequent
        // reconnects when the channel count / rate might have changed.
        int lfpBufferSamples = (int)(lfpSampleRate * bufferSizeInSeconds);
        if (sourceBuffers.size() < 2) {
            sourceBuffers.add(new DataBuffer(lfp_num_channels, lfpBufferSamples));
        } else {
            sourceBuffers[1]->resize(lfp_num_channels, lfpBufferSamples);
        }

        // Channel naming mirrors the broadband layout but with an LFP_
        // prefix: LFP_A_CH1.., LFP_B_CH1.. Lane order follows the same
        // bit-order packing the firmware uses (low->high bit, A then B).
        int portA_idx = 0;
        int portB_idx = 0;
        for (int b = 0; b < 8; ++b)
        {
            if ((lfp_lane_mask & (1 << b)) == 0)
                continue;
            String portPrefix = (b < 4) ? "LFP_A_" : "LFP_B_";
            for (int k = 0; k < 32; ++k)
            {
                int chanNum = (b < 4) ? ++portA_idx : ++portB_idx;
                ContinuousChannel::Settings ls{
                    ContinuousChannel::Type::ELECTRODE,
                    portPrefix + "CH" + String(chanNum),
                    "Intan LFP-band neural data channel",
                    "intan.continuous.lfp",
                    data_scale,                  // same 0.195 µV/LSB as broadband
                    lfpStream
                };
                continuousChannels->add(new ContinuousChannel(ls));
                continuousChannels->getLast()->setUnits("uV");
            }
        }

        LOGC("Configured LFP stream: ", lfp_num_channels, " channels @ ",
             (int)lfpSampleRate, " Hz (mask=0x",
             String::toHexString((int)lfp_lane_mask),
             ", decim=", (int)lfp_decim_R,
             ", taps=", (int)lfp_num_taps, ")");
    }
}

bool IntanSocket::foundInputSource()
{
    return intanInterface && intanInterface->foundInputSource();
}

bool IntanSocket::isReady()
{
    return intanInterface && intanInterface->isReady();
}

void IntanSocket::parameterValueChanged(Parameter* parameter)
{
    if (parameter->getName() == "device_ip")
    {
        device_ip = parameter->getValueAsString();
    }
    else if (parameter->getName() == "tcp_port")
    {
        tcp_port = (int)parameter->getValue();
    }
    else if (parameter->getName() == "udp_port")
    {
        udp_port = (int)parameter->getValue();
    }
    else if (parameter->getName() == "data_scale")
    {
        data_scale = (float)parameter->getValue();
    }
    else if (parameter->getName() == "aux_data_scale")
    {
        aux_data_scale = (float)parameter->getValue();
    }
}

bool IntanSocket::startAcquisition()
{
    if (!intanInterface || !intanInterface->isReady())
    {
        LOGE("Cannot start acquisition - device not ready");
        return false;
    }
    
    // Resize buffers - ONE time sample per packet across all channels
    convbuf.resize(num_channels);      // one time sample per channel

    totalSamples = 0;
    eventState = 0;
    hasError = false;
    
    // Clear any old data
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        while (!dataQueue.empty())
            dataQueue.pop();
    }
    
    // Initialize device (sends INIT sequence, then CONVERT sequence)
    if (!intanInterface->loadInitSequence())
    {
        LOGE("Failed to load init sequence");
        return false;
    }
    Thread::sleep(100);  // Wait for initialization
    
    if (!intanInterface->loadConvertSequence())
    {
        LOGE("Failed to load convert sequence");
        return false;
    }
    
    // Thread::sleep(100);  // Wait for initialization
    
    // if (!intanInterface->setDebugMode(true))
    // {
    //     LOGE("Failed to set debug mode.");
    //     return false;
    // }
 

    if (!intanInterface->setLoopCount(0)) // Loop count 0 for infinite streaming
    {
        LOGE("Failed to set loop count to infinite");
        return false;
    }
    Thread::sleep(10);    

    // Start acquisition on device
    if (!intanInterface->startAcquisition())
    {
        LOGE("Failed to start acquisition on device");
        return false;
    }
    
    LOGC("Intan acquisition started");
    startThread();
    
    return true;
}

bool IntanSocket::stopAcquisition()
{
    if (isThreadRunning())
    {
        signalThreadShouldExit();
    }
    
    if (intanInterface)
    {
        intanInterface->stopAcquisition();
    }

    sourceBuffers[0]->clear();
    
    LOGC("Intan acquisition stopped");
    return true;
}

void IntanSocket::processDataPacket(const uint32_t* data, size_t wordCount, uint64_t timestamp)
{
    // Called from IntanInterface's UDP thread
    // Queue the packet for processing in updateBuffer()
    
    std::lock_guard<std::mutex> lock(queueMutex);
    
    DataPacket packet;
    packet.data.assign(data, data + wordCount);
    packet.timestamp = timestamp;
    
    dataQueue.push(packet);
}

void IntanSocket::processLfpFrame(const IntanInterface::LfpFrame& frame)
{
    // Called from IntanInterface's LFP listener thread. If no second
    // DataStream was published (LFP wasn't enabled at connect time), there's
    // no sourceBuffers[1] to push into -- silently drop.
    if (!lfp_enabled || lfp_num_channels <= 0) return;
    if (sourceBuffers.size() < 2) return;
    if ((int)frame.sampleCount != lfp_num_channels) return;  // mask/cfg drift

    // Convert offset-binary uint16 -> signed float in uV, matching broadband
    // scaling. One time sample across all channels per frame.
    if ((int)lfpConvBuf.size() != lfp_num_channels)
        lfpConvBuf.resize(lfp_num_channels);

    for (int ch = 0; ch < lfp_num_channels; ++ch)
        lfpConvBuf[ch] = (float)((int)frame.samples[ch] - 32768) * data_scale;

    // Use the frame's timestamp (= frame_seq * decim_R, in broadband ticks --
    // aligns with the broadband stream). One TTL event word per sample;
    // we don't have a per-frame digital_in latch on the LFP path, so keep
    // it constant at eventState (no transitions on this stream).
    int64 lfpSampleNumber = (int64)frame.frameSequence;
    double lfpTimestamp = (double)frame.timestamp;

    sourceBuffers[1]->addToBuffer(lfpConvBuf.data(),
                                  &lfpSampleNumber,
                                  &lfpTimestamp,
                                  &eventState,
                                  1);  // ONE time sample
}

bool IntanSocket::updateBuffer()
{
    if (hasError)
    {
        return false;
    }
    
    // Get packet from queue
    DataPacket packet;
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        if (dataQueue.empty())
            return true;
        
        packet = dataQueue.front();
        dataQueue.pop();
    }

    int64 timestamp = (static_cast<uint64_t>(packet.data.data()[3]) << 32) | packet.data.data()[2];

    // Skip header (first 10 words: magic + timestamp)
    const uint32_t* dataWords = packet.data.data() + 10;
    size_t numDataWords = packet.data.size() - 10;

    // (Per-second PACKET DEBUG dump removed -- it was log spam; totalSamples
    // is still incremented at the end of this function as a sample counter,
    // preserving the upstream fix that made it actually count.)

    // Each packet contains ONE time sample for every channel.
    //
    // The data is a tight stream of 16-bit samples (two per 32-bit word, low
    // half first). For each acquisition cycle c (0..34) the PL appends the
    // enabled streams in bit order: bit0=CIPO0 reg, bit1=CIPO0 DDR,
    // bit2=CIPO1 reg, bit3=CIPO1 DDR. So a stream's sample for cycle c sits at
    // flat index  c * nStreams + slot,  where slot is its rank among the
    // enabled streams (== its bit-order rank).
    //
    // Two corrections vs. a naive copy:
    //   1. De-interleave: group each stream's 35 cycles back together.
    //   2. Undo the 2-cycle SPI pipeline delay: the result of COPI command i
    //      arrives in cycle (i+2) mod 35. The convert sequence issues
    //      amplifier ch 0..31 then 3 aux reads, so amplifier ch k lands in
    //      cycle (k+2), and aux 0/1/2 land in cycles 34/0/1.
    // Amplifier output is offset binary (baseline 0x8000), converted to signed
    // counts. Aux is physically unsigned but is centered the same way for
    // display (see the aux loop below).

    convbuf.resize(num_channels);

    // Scan all 8 bits: bits 0-3 = port A, bits 4-7 = port B.
    int streamBits[8];
    int nStreams = 0;
    for (int b = 0; b < 8; ++b)
    {
        if (channel_enable_mask & (1 << b))
            streamBits[nStreams++] = b;
    }

    // One-shot diagnostic: print the actual runtime sizes the first packet of
    // each acquisition. If numDataWords < 140 or nStreams != 8 at 0xFF, the
    // packet size pipeline is broken somewhere — do NOT remove until verified.
    {
        static uint32_t lastReportedMask = 0;
        if (channel_enable_mask != lastReportedMask)
        {
            lastReportedMask = channel_enable_mask;
            LOGC("[ephys-socket diag] mask=0x", String::toHexString((int)channel_enable_mask),
                 " nStreams=", nStreams,
                 " packet.data.size=", (int)packet.data.size(),
                 " numDataWords=", (int)numDataWords,
                 " num_channels=", num_channels);
        }
    }

    // Re-scan (the loop above was moved up so we can report nStreams);
    // this second pass is a no-op but kept for clarity of the original flow.
    nStreams = 0;
    for (int b = 0; b < 8; ++b)
        if (channel_enable_mask & (1 << b))
            streamBits[nStreams++] = b;

    const int nDataSamples = 35 * nStreams;

    // Fetch one 16-bit sample by flat index into the de-interleaved data.
    auto sampleAt = [&](int flatIdx) -> uint16_t {
        if (flatIdx < 0 || flatIdx >= nDataSamples)
            return 0x8000;  // midscale -> 0 after offset-binary conversion
        int wordIdx = flatIdx / 2;
        if ((size_t)wordIdx >= numDataWords)
            return 0x8000;
        if ((flatIdx & 1) == 0)
            return (uint16_t)(dataWords[wordIdx] & 0xFFFF);       // low 16 bits
        return (uint16_t)((dataWords[wordIdx] >> 16) & 0xFFFF);   // high 16 bits
    };

    int outCh = 0;

    // Neural channels: de-interleaved, de-skewed, and converted exactly as the
    // acquisition-board plugin does -- (raw_offset_binary - 32768) * 0.195 --
    // so the buffer carries true microvolts. With bitVolts = data_scale = 0.195
    // the record node stores (raw - 32768): the exact signed ADC count,
    // full-range and lossless (see storage note in README.md).
    for (int s = 0; s < nStreams; ++s)
    {
        for (int k = 0; k < 32; ++k)
        {
            int cycle = (k + 2) % 35;
            int flat = cycle * nStreams + s;
            convbuf[outCh++] = (float)((int)sampleAt(flat) - 32768) * data_scale;
        }
    }

    // Aux channels: only the "regular" streams carry aux inputs.
    // Regular stream bit positions and their aux bank indices:
    //   bit 0 (A_CIPO0_REG) -> aux bank 0
    //   bit 2 (A_CIPO1_REG) -> aux bank 1
    //   bit 4 (B_CIPO0_REG) -> aux bank 2
    //   bit 6 (B_CIPO1_REG) -> aux bank 3
    // (DDR streams at bits 1, 3, 5, 7 just resample the same aux -- drop them.)
    //
    // Converted exactly as the acquisition-board plugin does:
    //   (raw - 32768) * 0.0000374
    // The record node therefore stores (raw - 32768): the exact signed ADC
    // count, lossless. The midscale subtraction is a constant, reversible
    // representation choice, not a baseline/detrend.
    //
    // TWO FORMATS, distinguished PER PACKET by the aux flags in header word 4
    // (the packet is self-describing -- stays correct through live bank swaps
    // and sequencer enable/disable -- firmware aux-seq-v2):
    //
    //  * Legacy (sequencer off, flags==0): the static table converts all
    //    three aux inputs every packet; results sit at cycles 34/0/1.
    //
    //  * Aux-sequencer mode (flags bit0 set): slot 1 sweeps ONE accelerometer
    //    axis per packet (CONVERT 32 -> 33 -> 34 looping, 10 kHz per axis).
    //    Its result arrives at cycle 0 of the FOLLOWING packet, and that
    //    packet's header word 5 [15:0] echoes the originating command, which
    //    identifies the axis. De-interleave by echo with sample-and-hold so
    //    the 3 output channels stay at the full 30 kHz buffer rate.
    //    (Cycle 34 = slot 0's Reg-3 write echo and cycle 1 = slot 2's
    //    housekeeping result -- neither is accelerometer data in this mode.)
    //
    // Port B uses the SAME header echo (words 4/5) as port A.
    {
        uint32_t hdr4 = packet.data.data()[4];
        uint32_t hdr5 = packet.data.data()[5];
        uint8_t auxFlags = (hdr4 >> 8) & 0xFF;
        bool seqActive = (auxFlags & 0x01) != 0;
        bool echoValid = (auxFlags & 0x10) != 0;

        // Mapping from stream bit position to aux bank index.
        // Regular streams are at even bit positions 0, 2, 4, 6.
        //   bit 0 -> bank 0, bit 2 -> bank 1, bit 4 -> bank 2, bit 6 -> bank 3
        // DDR streams (odd bits) do not carry independent aux data.
        auto auxBankForBit = [](int b) -> int {
            // b must be 0, 2, 4, or 6
            return b / 2;
        };

        if (!seqActive)
        {
            const int auxCycle[3] = {34, 0, 1};
            for (int s = 0; s < nStreams; ++s)
            {
                int b = streamBits[s];
                if ((b & 1) != 0)
                    continue;  // skip DDR streams (odd bits)
                for (int a = 0; a < 3; ++a)
                {
                    int flat = auxCycle[a] * nStreams + s;
                    convbuf[outCh++] = (float)((int)sampleAt(flat) - 32768) * aux_data_scale;
                }
            }
        }
        else
        {
            uint16_t echo1 = hdr5 & 0xFFFF;          // slot-1 cmd answered @ cycle 0
            bool isConvert = (echo1 & 0xC000) == 0;
            int convCh = (echo1 >> 8) & 0x3F;

            for (int s = 0; s < nStreams; ++s)
            {
                int b = streamBits[s];
                if ((b & 1) != 0)
                    continue;  // skip DDR streams (odd bits)
                int bank = auxBankForBit(b);

                if (echoValid && isConvert && convCh >= 32 && convCh <= 34)
                    lastAccel[bank][convCh - 32] = sampleAt(0 * nStreams + s);

                for (int a = 0; a < 3; ++a)
                    convbuf[outCh++] = (float)((int)lastAccel[bank][a] - 32768) * aux_data_scale;
            }
        }
    }

    // Safety net: zero any channels we somehow didn't fill.
    for (; outCh < num_channels; ++outCh)
        convbuf[outCh] = 0.0f;
    
    uint64 ttlEventWord = (static_cast<uint64_t>(packet.data.data()[5]) << 32) | packet.data.data()[4];
    ttlEventWord = ttlEventWord & 0x00000000000000FF; // digital input is least significant 8 bits


    double ts;
    
    sourceBuffers[0]->addToBuffer(convbuf.data(),
                                   &timestamp,
                                   &ts,
                                   &ttlEventWord,
                                   1);  // ONE time sample

    totalSamples++;

    return true;
}

bool IntanSocket::runAutoDetection(IntanInterface::AutoDetectionResult& result, bool verbose)
{
    if (!intanInterface || !intanInterface->isReady())
    {
        LOGE("Cannot run auto-detection - device not ready");
        return false;
    }
    
    // Run detection
    bool success = intanInterface->runAutoDetection(result, verbose);
    
    if (success && result.success)
    {
        LOGC("Auto-detection complete: ", result.getChannelSummary().c_str());
    }
    
    return success;
}

bool IntanSocket::applyDetectionConfig(const IntanInterface::AutoDetectionResult& result)
{
    if (!intanInterface || !result.success)
    {
        return false;
    }
    
    // Apply configuration from detection
    if (!intanInterface->applyDetectionConfig(result))
    {
        LOGE("Failed to apply detection configuration");
        return false;
    }
    
    // Update local channel enable state
    channel_enable_mask = result.optimalChannelMask;
    num_channels = calculateNumChannels(channel_enable_mask);
    
    LOGC("Applied detection config - ", num_channels, " channels enabled");
    
    return true;
}
void IntanSocket::setDebugMode(bool enable, uint8_t mask)
{
    debugMode = enable;

    if (debugMode)
    {
        // ====================================================================
        // ENABLE DEBUG MODE
        // ====================================================================

        bool dualPort = (mask & 0xF0) != 0;
        LOGC("Enabling debug mode - mask 0x", String::toHexString((int)mask),
             dualPort ? " (dual-port 268ch)" : " (single-port 134ch)");

        // Check if we have a connection to the hardware
        if (!intanInterface || !intanInterface->isReady())
        {
            LOGE("Cannot enable debug mode - device not ready");
            CoreServices::sendStatusMessage("Intan: Debug mode failed - device not connected");
            debugMode = false;
            return;
        }

        // Step 1: Send hardware debug mode enable command (0x12 SET_DEBUG_MODE)
        if (!intanInterface->setDebugMode(true))
        {
            LOGE("Failed to send debug mode enable command to hardware");
            CoreServices::sendStatusMessage("Intan: Failed to enable hardware debug mode");
            debugMode = false;
            return;
        }

        Thread::sleep(50);  // Let hardware switch to debug mode

        // Step 2: Apply the requested channel-enable mask.
        if (!intanInterface->setChannelEnable(mask))
        {
            LOGE("Failed to set channel enable for debug mode");
            CoreServices::sendStatusMessage("Intan: Failed to configure channels");
            debugMode = false;
            return;
        }

        Thread::sleep(50);  // Let channel config take effect

        // Step 5: Update local configuration
        channel_enable_mask = mask;
        num_channels = calculateNumChannels(channel_enable_mask);

        LOGC("Debug mode enabled - mask 0x", String::toHexString((int)mask),
             ", channels=", num_channels);

        // Step 6: Update the chip display in the editor
        if (sn->getEditor() != nullptr)
        {
            IntanSocketEditor* editor = static_cast<IntanSocketEditor*>(sn->getEditor());

            // Fake detection result mirroring the requested mask. Port A is
            // lit whenever any of the low-nibble bits are on; port B only
            // when the high nibble is on.
            IntanInterface::AutoDetectionResult debugResult;
            debugResult.success = true;
            debugResult.chipsDetected = true;
            debugResult.cipo0Detected = (mask & 0x01) != 0;
            debugResult.cipo1Detected = (mask & 0x04) != 0;
            debugResult.cipo0ChipType = debugResult.cipo0Detected ? IntanInterface::ChipType::RHD2164 : IntanInterface::ChipType::NONE;
            debugResult.cipo1ChipType = debugResult.cipo1Detected ? IntanInterface::ChipType::RHD2164 : IntanInterface::ChipType::NONE;
            debugResult.cipo0HasDdr = false;
            debugResult.cipo1HasDdr = false;
            debugResult.portBCipo0Detected = (mask & 0x10) != 0;
            debugResult.portBCipo1Detected = (mask & 0x40) != 0;
            debugResult.portBCipo0ChipType = debugResult.portBCipo0Detected ? IntanInterface::ChipType::RHD2164 : IntanInterface::ChipType::NONE;
            debugResult.portBCipo1ChipType = debugResult.portBCipo1Detected ? IntanInterface::ChipType::RHD2164 : IntanInterface::ChipType::NONE;
            debugResult.portBCipo0HasDdr = false;
            debugResult.portBCipo1HasDdr = false;
            debugResult.optimalChannelMask = mask;
            debugResult.bestPhase0 = 0;
            debugResult.bestPhase1 = 0;

            editor->updateChipDetection(debugResult);
        }

        // Step 7: Update the signal chain to reflect new channel count
        CoreServices::updateSignalChain(sn->getEditor());
        CoreServices::sendStatusMessage(dualPort
            ? "Intan: Debug mode enabled (dual-port, 268 channels)"
            : "Intan: Debug mode enabled (single-port, 134 channels)");
    }
    else
    {
        // ====================================================================
        // DISABLE DEBUG MODE
        // ====================================================================
        
        LOGC("Disabling debug mode");
        
        // Check if we have a connection to the hardware
        if (!intanInterface || !intanInterface->isReady())
        {
            LOGD("Debug mode disabled (device not connected)");
            debugMode = false;
            return;
        }
        
        // Step 1: Send hardware debug mode disable command (0x12 SET_DEBUG_MODE with param1=0)
        if (!intanInterface->setDebugMode(false))
        {
            LOGE("Failed to send debug mode disable command to hardware");
            // Continue anyway - we want to update the UI
        }
        
        Thread::sleep(50);
        
        // Step 2: Reset to default channel configuration
        // Note: You may want to restore the previous channel enable state
        // For now, we'll set it to a reasonable default (all channels)
        if (!intanInterface->setChannelEnable(0x0F))
        {
            LOGE("Failed to reset channel enable");
        }
        
        Thread::sleep(50);
        
        // Step 4: Clear the chip displays
        if (sn->getEditor() != nullptr)
        {
            IntanSocketEditor* editor = static_cast<IntanSocketEditor*>(sn->getEditor());
            
            // Clear the chip displays - user should run RESCAN to detect real chips
            IntanInterface::AutoDetectionResult clearResult;
            clearResult.success = false;
            clearResult.chipsDetected = false;
            clearResult.cipo0Detected = false;
            clearResult.cipo1Detected = false;
            clearResult.cipo0ChipType = IntanInterface::ChipType::NONE;
            clearResult.cipo1ChipType = IntanInterface::ChipType::NONE;
            
            editor->updateChipDetection(clearResult);
        }
        
        LOGC("Debug mode disabled - use RESCAN to detect actual chips");
        CoreServices::sendStatusMessage("Intan: Debug mode disabled - run RESCAN");
    }
}
// ============================================================================
// AUX COMMAND SEQUENCER TOOLING (firmware aux-seq-v2)
// ============================================================================

void IntanSocket::printDeviceStatus()
{
    if (!intanInterface || !intanInterface->foundInputSource())
    {
        LOGE("Cannot print status - device not connected");
        CoreServices::sendStatusMessage("Intan: not connected");
        return;
    }

    IntanInterface::DeviceStatus status;
    if (!intanInterface->getStatus(status))
    {
        LOGE("Failed to read device status");
        CoreServices::sendStatusMessage("Intan: status read failed");
        return;
    }

    // Emit line-by-line so each line gets the console prefix
    std::istringstream iss(status.getSummary());
    std::string line;
    while (std::getline(iss, line))
        LOGC(line);

    // Plugin-side reception stats. These tell us whether UDP packets are being
    // lost (timestampErrors), arriving with wrong size (sizeErrors), or with
    // a bad magic header (magicErrors). A growing timestampErrors with a stable
    // packetsReceived from the device == packet drops between firmware and host
    // (network or BRAM ring overrun). Growing PL fifoCount or PS errorCount in
    // the device status == firmware can't keep up with PL writes.
    IntanInterface::ReceptionStats rx;
    intanInterface->getReceptionStats(rx);
    LOGC("--- Plugin Reception ---");
    LOGC("Packets recv: ", (int64)rx.totalPackets,
         "  totalErr: ", (int64)rx.totalErrors,
         "  magicErr: ", (int64)rx.magicErrors,
         "  tsErr: ", (int64)rx.timestampErrors,
         "  sizeErr: ", (int64)rx.sizeErrors);
    LOGC("Rate: ", rx.instantRate, " pkt/s (", rx.dataRateMbps, " Mbps)");

    CoreServices::sendStatusMessage("Intan: status printed to console");
}

bool IntanSocket::pushFastSettleConfig()
{
    if (!intanInterface)
        return false;
    bool gpioEn = fastSettleTTL >= 0;
    // Default: amplifier fast settle (RHD Reg-0 D5) and DSP reset (CONVERT
    // bit-H) follow the SAME trigger -- both the SETTLE button's software
    // level and the TTL Settle pin. Configurability for independent DSP
    // triggering can be added later.
    return intanInterface->setFastSettle(fastSettleSw, gpioEn,
                                         gpioEn ? (uint8_t)fastSettleTTL : 0,
                                         true /* DSP follows fast settle */);
}

void IntanSocket::setManualFastSettle(bool active)
{
    if (!intanInterface || !intanInterface->foundInputSource())
    {
        LOGE("Fast settle: device not connected");
        return;
    }

    // The override layer only reaches the chip while the sequencer is on
    if (active && !auxSeqMode)
    {
        LOGC("Fast settle requires the aux sequencer - enabling aux mode first");
        if (!setAuxSequencerMode(true))
            return;
    }

    fastSettleSw = active;
    if (pushFastSettleConfig())
    {
        LOGC("Fast settle ", active ? "ON" : "OFF",
             " (RHD Reg-0 D5 via slot-0 injection)");
        CoreServices::sendStatusMessage(active ? "Intan: FAST SETTLE ON"
                                               : "Intan: fast settle off");
    }
    else
    {
        LOGE("Fast settle command failed");
    }
}

void IntanSocket::setFastSettleTTLPin(int pin)
{
    if (!intanInterface || !intanInterface->foundInputSource())
        return;

    if (pin >= 0 && !auxSeqMode)
    {
        LOGC("TTL fast settle requires the aux sequencer - enabling aux mode first");
        if (!setAuxSequencerMode(true))
            return;
    }

    fastSettleTTL = (pin >= 0 && pin <= 7) ? pin : -1;
    if (pushFastSettleConfig())
    {
        // (LOGC expands to a statement with its own ';' -- keep braces)
        if (fastSettleTTL >= 0)
        {
            LOGC("Fast settle following digital_in[", fastSettleTTL, "]");
        }
        else
        {
            LOGC("TTL fast settle disabled");
        }
    }
}

bool IntanSocket::setAuxSequencerMode(bool enable)
{
    if (!intanInterface || !intanInterface->foundInputSource())
    {
        LOGE("Aux sequencer: device not connected");
        return false;
    }

    if (!enable)
    {
        // Firmware clears the live fast-settle/digout sources and waits one
        // packet before dropping the enable (so the chip is never left
        // clamped); mirror the local state.
        fastSettleSw = false;
        if (!intanInterface->auxSeqEnable(false))
        {
            LOGE("Failed to disable aux sequencer");
            return false;
        }
        auxSeqMode = false;
        LOGC("Aux sequencer disabled - legacy aux format (all 3 axes per packet)");
        CoreServices::sendStatusMessage("Intan: aux sequencer off");
        return true;
    }

    IntanInterface::DeviceStatus status;
    if (!intanInterface->getStatus(status))
    {
        LOGE("Aux sequencer: status read failed");
        return false;
    }
    if (!status.hasAuxStatus)
    {
        LOGE("This firmware predates the aux sequencer (86-byte status) - "
             "update BOOT.bin to the aux-seq-v2 build");
        CoreServices::sendStatusMessage("Intan: firmware lacks aux sequencer");
        return false;
    }

    // Target banks: if the sequencer is already running, write the STANDBY
    // bank of each slot and swap - this exercises the live double-buffer +
    // atomic packet-boundary swap. Otherwise start on bank 0.
    int target[3];
    for (int s = 0; s < 3; ++s)
        target[s] = status.auxSeqEnabled ? (((status.auxBankActive >> s) & 1) ^ 1) : 0;

    // Default slot programs (mirrors remote/net.py):
    //   slot 0 (cycle 32, real-time): Reg-3 carrier - rewritten by the
    //           override shadow every packet (digout mirror / fast settle home)
    //   slot 1 (cycle 33, ADC): accelerometer sweep, one axis per packet
    //   slot 2 (cycle 34, housekeeping): supply, temp, chip ID, 'INTAN' ROM
    std::vector<uint16_t> slot0 = { IntanInterface::rhdWrite(3, 0x02) };
    std::vector<uint16_t> slot1 = { IntanInterface::rhdConvert(32),
                                    IntanInterface::rhdConvert(33),
                                    IntanInterface::rhdConvert(34) };
    std::vector<uint16_t> slot2 = { IntanInterface::rhdConvert(48),
                                    IntanInterface::rhdConvert(49),
                                    IntanInterface::rhdRead(63),
                                    IntanInterface::rhdRead(62),
                                    IntanInterface::rhdRead(40),
                                    IntanInterface::rhdRead(41),
                                    IntanInterface::rhdRead(42),
                                    IntanInterface::rhdRead(43),
                                    IntanInterface::rhdRead(44) };

    if (!intanInterface->auxUploadBank(0, target[0], slot0, 0) ||
        !intanInterface->auxUploadBank(1, target[1], slot1, 0) ||
        !intanInterface->auxUploadBank(2, target[2], slot2, 0))
    {
        LOGE("Aux bank upload failed");
        return false;
    }

    for (int s = 0; s < 3; ++s)
    {
        if (!intanInterface->auxBankSelect(s, target[s]))
        {
            LOGE("Aux bank select failed (slot ", s, ", bank ", target[s], ")");
            return false;
        }
    }

    if (!status.auxSeqEnabled && !intanInterface->auxSeqEnable(true))
    {
        LOGE("Failed to enable aux sequencer");
        return false;
    }

    // Reset the de-interleave state for all 4 aux banks to midscale
    for (int b = 0; b < 4; ++b)
        for (int a = 0; a < 3; ++a)
            lastAccel[b][a] = 0x8000;

    auxSeqMode = true;
    if (status.auxSeqEnabled)
    {
        LOGC("Aux banks reloaded LIVE via standby-bank swap (slots now on banks ",
             target[0], "/", target[1], "/", target[2], ")");
    }
    else
    {
        LOGC("Aux sequencer enabled - accel de-interleave mode (10 kHz/axis)");
    }
    CoreServices::sendStatusMessage("Intan: aux sequencer active");
    return true;
}
