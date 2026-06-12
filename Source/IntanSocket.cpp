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

        // CRITICAL: Configure channel enable BEFORE getting status
        // Firmware defaults to 0x0, we want all 4 channels
        if (!intanInterface->setChannelEnable(0x0F)) {
            LOGE("Failed to set channel enable");
            intanInterface.reset();
            return false;
        }
        
        Thread::sleep(100);  // Let it take effect
        
        // Set up data callback
        intanInterface->setDataCallback(
            [this](const uint32_t* data, size_t words, uint64_t timestamp) {
                processDataPacket(data, words, timestamp);
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
        
        // Get device status to determine channel configuration
        IntanInterface::DeviceStatus status;
        if (intanInterface->getStatus(status))
        {
            channel_enable_mask = status.channelEnable;
            num_channels = calculateNumChannels(channel_enable_mask);

            // Sync local aux-sequencer state with the device (it persists
            // across plugin reconnects)
            auxSeqMode = status.hasAuxStatus && status.auxSeqEnabled;
            fastSettleSw = status.hasAuxStatus && status.fastSettleActive;

            if (printOutput)
            {
                LOGC("Connected to Intan device - ", num_channels, " channels active");
                LOGC("Firmware: ", status.getFirmwareVersionString().c_str());
                CoreServices::sendStatusMessage("Intan: Connected successfully.");
            }
        }
        
        getParameter("device_ip")->setEnabled(false);

        if (sn->getEditor() != nullptr)
            static_cast<IntanSocketEditor*>(sn->getEditor())->connected();

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
    // Build the channel layout from the active stream mask.
    //
    // The PL emits, per acquisition cycle, the enabled 16-bit segments in
    // bit order: bit0=CIPO0 regular, bit1=CIPO0 DDR, bit2=CIPO1 regular,
    // bit3=CIPO1 DDR. Each stream carries 32 amplifier channels; only the
    // two "regular" streams additionally carry the 3 aux inputs (the DDR
    // stream just resamples the same aux, so its aux samples are dropped).
    //
    // Channel order here MUST match the fill order in updateBuffer():
    //   [stream0 CH1..32][stream1 CH1..32]...  then  [aux per regular stream]
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

    // Neural channels, grouped per stream (de-interleaved), sequential CH numbering
    int neuralIdx = 0;
    for (int b = 0; b < 4; ++b)
    {
        if ((channel_enable_mask & (1 << b)) == 0)
            continue;

        for (int k = 0; k < 32; ++k)
        {
            ContinuousChannel::Settings chanSettings{
                ContinuousChannel::Type::ELECTRODE,
                "CH" + String(neuralIdx + 1),  // CH1, CH2, ... CH128
                "Intan neural data channel",
                "intan.continuous.ephys",
                data_scale,
                stream
            };

            continuousChannels->add(new ContinuousChannel(chanSettings));
            continuousChannels->getLast()->setUnits("uV");
            neuralIdx++;
        }
    }

    // Aux channels, only for the regular streams (CIPO0 regular = bit0,
    // CIPO1 regular = bit2), 3 per bank, clearly identified by CIPO line.
    for (int b = 0; b < 4; ++b)
    {
        if ((channel_enable_mask & (1 << b)) == 0)
            continue;
        if (b != 0 && b != 2)
            continue;  // only regular streams carry aux

        int cipoNum = (b == 0) ? 0 : 1;
        for (int a = 0; a < 3; ++a)
        {
            ContinuousChannel::Settings channelSettings {
                ContinuousChannel::AUX,
                "AUX" + String(cipoNum) + "_" + String(a + 1),  // AUX0_1..AUX1_3
                "Intan aux input channel",
                "intan.continuous.aux",
                aux_data_scale,
                stream
            };

            continuousChannels->add (new ContinuousChannel (channelSettings));
            continuousChannels->getLast()->setUnits ("uV");
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

    // Periodic logging (every 30000 samples = once per second at 30kHz)
    static int64 logCounter = 0;
    bool shouldLog = (totalSamples % 29000 == 0);
    
    if (shouldLog) {
        LOGC("=== PACKET DEBUG (sample ", totalSamples, ") ===");
        LOGC("Total packet words: ", packet.data.size());
        LOGC("Data words (after header): ", numDataWords);
        LOGC("num_channels: ", num_channels);
        LOGC("channel_enable_mask: 0x", String::toHexString(channel_enable_mask));
        
        // Show first few raw data words
        LOGC("First 8 data words (hex):");
        for (size_t i = 0; i < std::min(size_t(8), numDataWords); ++i) {
            LOGC("  Word ", i, ": 0x", String::toHexString(dataWords[i]));
        }
    }

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

    int streamBits[4];
    int nStreams = 0;
    for (int b = 0; b < 4; ++b)
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

    // Neural channels: de-interleaved, de-skewed, offset-binary -> signed.
    for (int s = 0; s < nStreams; ++s)
    {
        for (int k = 0; k < 32; ++k)
        {
            int cycle = (k + 2) % 35;
            int flat = cycle * nStreams + s;
            convbuf[outCh++] = (float)((int)sampleAt(flat) - 32768);
        }
    }

    // Aux channels: only the regular streams (bit 0 / bit 2). Aux (e.g. a
    // headstage accelerometer on auxin1/2/3) is physically unsigned, but we
    // subtract mid-scale and use the same uV scaling as the amplifiers purely
    // so the trace is centered and visible in the viewer. Absolute aux
    // calibration (true LSB ~37.4 uV) is not applied.
    //
    // TWO FORMATS, distinguished PER PACKET by the aux flags in header word 4
    // (the packet is self-describing, so this stays correct through live
    // bank swaps and sequencer enable/disable -- firmware aux-seq-v2):
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
    {
        uint32_t hdr4 = packet.data.data()[4];
        uint32_t hdr5 = packet.data.data()[5];
        uint8_t auxFlags = (hdr4 >> 8) & 0xFF;
        bool seqActive = (auxFlags & 0x01) != 0;
        bool echoValid = (auxFlags & 0x10) != 0;

        if (!seqActive)
        {
            const int auxCycle[3] = {34, 0, 1};
            for (int s = 0; s < nStreams; ++s)
            {
                int b = streamBits[s];
                if (b != 0 && b != 2)
                    continue;
                for (int a = 0; a < 3; ++a)
                {
                    int flat = auxCycle[a] * nStreams + s;
                    convbuf[outCh++] = (float)((int)sampleAt(flat) - 32768);
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
                if (b != 0 && b != 2)
                    continue;
                int bank = (b == 0) ? 0 : 1;

                if (echoValid && isConvert && convCh >= 32 && convCh <= 34)
                    lastAccel[bank][convCh - 32] = sampleAt(0 * nStreams + s);

                for (int a = 0; a < 3; ++a)
                    convbuf[outCh++] = (float)((int)lastAccel[bank][a] - 32768);
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
void IntanSocket::setDebugMode(bool enable)
{
    debugMode = enable;
    
    if (debugMode)
    {
        // ====================================================================
        // ENABLE DEBUG MODE
        // ====================================================================
        
        LOGC("Enabling debug mode - simulating 128 channels (2x64)");
        
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
        
        // Step 2: Set channel enable to all channels (0x0F)
        // This enables all 4 channel groups for maximum channel count
        if (!intanInterface->setChannelEnable(0x0F))
        {
            LOGE("Failed to set channel enable for debug mode");
            CoreServices::sendStatusMessage("Intan: Failed to configure channels");
            debugMode = false;
            return;
        }
        
        Thread::sleep(50);  // Let channel config take effect
        
        // Step 5: Update local configuration
        channel_enable_mask = 0x0F;
        num_channels = calculateNumChannels(channel_enable_mask);  // 4×32 + 2×3 aux
        
        LOGC("Debug mode enabled successfully - hardware configured for synthetic data");
        LOGC("Channel mask: 0x0F, Channels: ", num_channels);
        
        // Step 6: Update the chip display in the editor
        if (sn->getEditor() != nullptr)
        {
            IntanSocketEditor* editor = static_cast<IntanSocketEditor*>(sn->getEditor());
            
            // Create a fake detection result showing 2x RHD2164 chips (64 channels each)
            IntanInterface::AutoDetectionResult debugResult;
            debugResult.success = true;
            debugResult.chipsDetected = true;
            debugResult.cipo0Detected = true;
            debugResult.cipo1Detected = true;
            debugResult.cipo0ChipType = IntanInterface::ChipType::RHD2164;
            debugResult.cipo1ChipType = IntanInterface::ChipType::RHD2164;
            debugResult.cipo0HasDdr = false;
            debugResult.cipo1HasDdr = false;
            debugResult.optimalChannelMask = 0x0F;
            debugResult.bestPhase0 = 0;
            debugResult.bestPhase1 = 0;
            
            editor->updateChipDetection(debugResult);
        }
        
        // Step 7: Update the signal chain to reflect new channel count
        CoreServices::updateSignalChain(sn->getEditor());
        CoreServices::sendStatusMessage("Intan: Debug mode enabled (128 channels)");
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

    CoreServices::sendStatusMessage("Intan: status printed to console");
}

bool IntanSocket::pushFastSettleConfig()
{
    if (!intanInterface)
        return false;
    bool gpioEn = fastSettleTTL >= 0;
    return intanInterface->setFastSettle(fastSettleSw, gpioEn,
                                         gpioEn ? (uint8_t)fastSettleTTL : 0,
                                         false /* no DSP reset by default */);
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

    // Reset the de-interleave state to midscale until real samples arrive
    for (int b = 0; b < 2; ++b)
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
