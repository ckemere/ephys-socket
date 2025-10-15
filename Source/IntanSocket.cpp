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
    channel_enable_mask = 0x0F;  // All channels enabled by default
    num_channels = 4;
    totalSamples = 0;
    eventState = 0;
    hasError = false;

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

int IntanSocket::calculateNumChannels(uint8_t mask)
{
    int count = 0;
    for (int i = 0; i < 4; ++i)
    {
        if (mask & (1 << i))
            count++;
    }
    return (count > 0) ? count : 1;  // At least 1
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

    DataStream::Settings settings{
        "IntanStream",
        "Data from Intan neural interface",
        "intan.data",
        SAMPLE_RATE
    };

    sourceStreams->add(new DataStream(settings));
    
    // Calculate total channels based on channel_enable_mask
    // Each bit enables 35 channels:
    // Bit 0: channels 0-34
    // Bit 1: channels 35-69
    // Bit 2: channels 70-104
    // Bit 3: channels 105-140
    
    num_channels = 0;
    for (int i = 0; i < 4; ++i)
    {
        if (channel_enable_mask & (1 << i))
            num_channels += 35;
    }
    
    if (num_channels == 0)
    {
        LOGE("No channels enabled!");
        num_channels = 35; // Fallback
    }
    
    // Resize buffer
    sourceBuffers[0]->resize(num_channels, SAMPLE_RATE * bufferSizeInSeconds);

    // Create channels - simple sequential numbering
    for (int ch = 0; ch < num_channels; ++ch)
    {
        ContinuousChannel::Settings chanSettings{
            ContinuousChannel::Type::ELECTRODE,
            "CH" + String(ch + 1),  // CH1, CH2, CH3, ... CH128
            "Intan neural data channel",
            "intan.continuous",
            data_scale,
            sourceStreams->getFirst()
        };
        
        continuousChannels->add(new ContinuousChannel(chanSettings));
    }

    EventChannel::Settings eventSettings{
        EventChannel::Type::TTL,
        "Events",
        "Events from Intan interface",
        "intan.events",
        sourceStreams->getFirst(),
        1
    };

    eventChannels->add(new EventChannel(eventSettings));
    
    LOGC("Configured ", num_channels, " channels");
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
    

    // CRITICAL: Set channel enable explicitly (firmware defaults to 0x0)
    if (!intanInterface->setChannelEnable(0x0F)) {
        LOGE("Failed to set channel enable");
        return false;
    }
    Thread::sleep(50);
    
    // Update num_channels to match what we just set
    channel_enable_mask = 0x0F;
    num_channels = calculateNumChannels(0x0F);  // Should be 140
    
    // Resize buffers - ONE time sample per packet across all channels
    convbuf.resize(num_channels);      // 140 channels × 1 sample
    sampleNumbers.resize(1);           // 1 time sample
    timestamps.clear();
    timestamps.insertMultiple(0, 0.0, 1);  // 1 timestamp
    ttlEventWords.resize(1);           // 1 TTL word
    
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
    
    Thread::sleep(100);  // Wait for initialization
    
    if (!intanInterface->setDebugMode(true))
    {
        LOGE("Failed to set debug mode.");
        return false;
    }

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
    
    // Skip header (first 4 words: magic + timestamp)
    const uint32_t* dataWords = packet.data.data() + 4;
    size_t numDataWords = packet.data.size() - 4;
    
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
            int16_t sample;
            if (halfIdx == 0)
                sample = (int16_t)(dataWords[wordIdx] & 0xFFFF);  // Low 16 bits
            else
                sample = (int16_t)((dataWords[wordIdx] >> 16) & 0xFFFF);  // High 16 bits
            
            convbuf[ch] = (float)sample;
        }
        else
        {
            convbuf[ch] = 0.0f;
        }
    }
    
    // Add ONE sample (across all channels) to the buffer
    sampleNumbers.set(0, totalSamples++);
    ttlEventWords.set(0, eventState);
    timestamps.set(0, 0.0);
    
    sourceBuffers[0]->addToBuffer(convbuf.data(),
                                   sampleNumbers.getRawDataPointer(),
                                   timestamps.getRawDataPointer(),
                                   ttlEventWords.getRawDataPointer(),
                                   1);  // ONE time sample
    
    return true;
}
