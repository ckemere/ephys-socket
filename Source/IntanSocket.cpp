#include "IntanSocket.h"
#include "IntanSocketEditor.h"

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
                     "Data scale (mV per bit)", 
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
            int num_channels = calculateNumChannels(channel_enable_mask);
            
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
    
    // Calculate total channels based on channel_enable_mask
    // Each bit enables 35 channels (32 + 3 aux):
    // Bit 0: channels 0-34
    // Bit 1: channels 35-69
    // Bit 2: channels 70-104
    // Bit 3: channels 105-140
    
    // The 2164 DDR channel just resamples the aux input, so max is two
    int n_aux_banks = ((channel_enable_mask & 0b0001) != 0) + ((channel_enable_mask & 0b0100) != 0); 
    int n_data_banks = n_aux_banks + ((channel_enable_mask & 0b1000) != 0) + ((channel_enable_mask & 0b0010) != 0);
    
    int n_neural_channels = n_data_banks * 32;
    if (n_neural_channels == 0)
    {
        LOGE("No channels enabled!");
        n_neural_channels = 32; // Fallback
        n_aux_banks = 3;
    }
    
    // Resize buffer
    num_channels = n_data_banks * 35;
    sourceBuffers[0]->resize(num_channels, SAMPLE_RATE * bufferSizeInSeconds);

    // Create channels - simple sequential numbering
    for (int ch = 0; ch < n_neural_channels; ++ch)
    {
        ContinuousChannel::Settings chanSettings{
            ContinuousChannel::Type::ELECTRODE,
            "CH" + String(ch + 1),  // CH1, CH2, CH3, ... CH128
            "Intan neural data channel",
            "intan.continuous.ephys",
            data_scale,
            stream
        };
        
        continuousChannels->add(new ContinuousChannel(chanSettings));
        continuousChannels->getLast()->setUnits ("uV");

    }

    for (int a = 0; a < n_aux_banks; a++)
    {
        for (int ch = 0; ch < 3; ch++)
        {
            ContinuousChannel::Settings channelSettings {
                ContinuousChannel::AUX,
                "AUX" + String(a + 1) + "_CH" + String (ch + 1),
                "Intan aux input channel",
                "intan.continuous.aux",
                data_scale,
                stream
            };

            continuousChannels->add (new ContinuousChannel (channelSettings));
            continuousChannels->getLast()->setUnits ("mV");
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
}

bool IntanSocket::startAcquisition()
{
    if (!intanInterface || !intanInterface->isReady())
    {
        LOGE("Cannot start acquisition - device not ready");
        return false;
    }
    
    // Resize buffers - ONE time sample per packet across all channels
    convbuf.resize(num_channels);      // 140 channels × 1 sample
    
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

    // Each packet contains ONE time sample for each channel
    // Data is packed as 16-bit signed integers in 32-bit words
    
    convbuf.resize(num_channels);
    
    for (int ch = 0; ch < num_channels; ++ch)
    {
        // Calculate which word and which half (low or high 16 bits)
        int wordIdx = ch / 2;
        int halfIdx = ch % 2;
        
        if (wordIdx < numDataWords)
        {
            uint16_t sample;
            if (halfIdx == 0)
                sample = (uint16_t)(dataWords[wordIdx] & 0xFFFF);  // Low 16 bits
            else
                sample = (uint16_t)((dataWords[wordIdx] >> 16) & 0xFFFF);  // High 16 bits
            
            convbuf[ch] = (float)(sample - 32768);
        }
        else
        {
            convbuf[ch] = 0.0f;
        }
    }
    
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
    int num_channels = calculateNumChannels(channel_enable_mask);
    
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
        num_channels = 140;  // 4 × 35 channels (will show as 128 in the UI)
        
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