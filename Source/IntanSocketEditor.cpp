#include "IntanSocketEditor.h"
#include "IntanSocket.h"

using namespace IntanSocketNode;

// ============================================================================
// ChipInterface Implementation
// ============================================================================

ChipInterface::ChipInterface(IntanSocket* node_, int cipoIndex_)
    : node(node_)
    , cipoIndex(cipoIndex_)
    , isDetected(false)
    , chipType(IntanInterface::ChipType::NONE)
{
    label = (cipoIndex == 0) ? "0" : "1";
}

void ChipInterface::updateChipStatus(bool detected, IntanInterface::ChipType type)
{
    isDetected = detected;
    chipType = type;
    repaint();
}

void ChipInterface::paint(Graphics& g)
{
    // Background box
    g.setColour(findColour(ThemeColours::componentBackground).darker(0.2f));
    g.fillRoundedRectangle(5, 0, getWidth() - 10, getHeight(), 4.0f);
    
    // Label
    g.setColour(findColour(ThemeColours::defaultText));
    g.setFont(FontOptions("Inter", "Regular", 15.0f));
    g.drawText(label, 10, 2, 20, 15, Justification::left, false);
    
    // Chip indicator box
    if (isDetected)
    {
        // Orange background for detected chip
        g.setColour(Colour(255, 145, 0));
        g.fillRoundedRectangle(23, 1, 40, 17, 3.0f);
        
        // Black text showing channel count
        g.setColour(Colours::black);
        g.setFont(FontOptions("Inter", "Bold", 12.0f));
        String chipLabel;
        if (chipType == IntanInterface::ChipType::RHD2164)
            chipLabel = "64";
        else if (chipType == IntanInterface::ChipType::RHD2132)
            chipLabel = "32";
        else
            chipLabel = "??";
            
        g.drawText(chipLabel, 23, 1, 40, 17, Justification::centred, false);
    }
    else
    {
        // Gray background for no chip
        g.setColour(Colour(80, 80, 80));
        g.fillRoundedRectangle(23, 1, 40, 17, 3.0f);
    }
}

// ============================================================================
// SampleRateInterface Implementation
// ============================================================================

SampleRateInterface::SampleRateInterface(IntanSocket* node_)
    : node(node_)
{
    rateLabel = std::make_unique<Label>("SampleRate", "30.0 kS/s");
    rateLabel->setBounds(0, 14, 80, 20);
    rateLabel->setJustificationType(Justification::centred);
    rateLabel->setColour(Label::backgroundColourId, Colour(60, 60, 60));
    rateLabel->setColour(Label::textColourId, Colours::white);
    addAndMakeVisible(rateLabel.get());
}

void SampleRateInterface::paint(Graphics& g)
{
    g.setColour(findColour(ThemeColours::defaultText));
    g.setFont(FontOptions("Inter", "Regular", 10.0f));
    g.drawText("Sample Rate", 0, 0, 80, 15, Justification::left, false);
}

// ============================================================================
// BandwidthInterface Implementation
// ============================================================================

BandwidthInterface::BandwidthInterface(IntanSocket* node_)
    : node(node_)
    , lowerBandwidth(1.0)
    , upperBandwidth(7500.0)
{
    lowerBandwidthLabel = std::make_unique<Label>("LowerBW", "1");
    lowerBandwidthLabel->setEditable(true, false, false);
    lowerBandwidthLabel->addListener(this);
    lowerBandwidthLabel->setBounds(25, 10, 50, 20);
    addAndMakeVisible(lowerBandwidthLabel.get());
    
    upperBandwidthLabel = std::make_unique<Label>("UpperBW", "7500");
    upperBandwidthLabel->setEditable(true, false, false);
    upperBandwidthLabel->addListener(this);
    upperBandwidthLabel->setBounds(25, 25, 50, 20);
    addAndMakeVisible(upperBandwidthLabel.get());
}

void BandwidthInterface::labelTextChanged(Label* label)
{
    // Placeholder - will be implemented when hardware supports bandwidth control
    if (label == lowerBandwidthLabel.get())
    {
        double value = label->getText().getDoubleValue();
        if (value >= 0.1 && value <= 500.0)
        {
            lowerBandwidth = value;
        }
        else
        {
            label->setText(String(lowerBandwidth), dontSendNotification);
            CoreServices::sendStatusMessage("Lower bandwidth must be between 0.1 and 500 Hz");
        }
    }
    else if (label == upperBandwidthLabel.get())
    {
        double value = label->getText().getDoubleValue();
        if (value >= 100.0 && value <= 20000.0)
        {
            upperBandwidth = value;
        }
        else
        {
            label->setText(String(upperBandwidth), dontSendNotification);
            CoreServices::sendStatusMessage("Upper bandwidth must be between 100 and 20000 Hz");
        }
    }
}

void BandwidthInterface::setLowerBandwidth(double value)
{
    lowerBandwidth = value;
    lowerBandwidthLabel->setText(String(value), dontSendNotification);
}

void BandwidthInterface::setUpperBandwidth(double value)
{
    upperBandwidth = value;
    upperBandwidthLabel->setText(String(value), dontSendNotification);
}

void BandwidthInterface::paint(Graphics& g)
{
    g.setColour(findColour(ThemeColours::defaultText));
    g.setFont(FontOptions("Inter", "Regular", 10.0f));
    g.drawText("Bandwidth", 0, 0, 200, 11, Justification::left, false);
    g.drawText("Low:", 0, 11, 200, 15, Justification::left, false);
    g.drawText("High:", 0, 26, 200, 15, Justification::left, false);
}

// ============================================================================
// IntanSocketEditor Implementation
// ============================================================================

IntanSocketEditor::IntanSocketEditor(GenericProcessor* parentNode, IntanSocket* socket)
    : GenericEditor(parentNode)
{
    node = socket;
    desiredWidth = 425;

    // Chip detection interfaces (CIPO0 and CIPO1)
    cipo0Interface = std::make_unique<ChipInterface>(node, 0);
    cipo0Interface->setBounds(3, 28, 70, 18);
    addAndMakeVisible(cipo0Interface.get());
    
    cipo1Interface = std::make_unique<ChipInterface>(node, 1);
    cipo1Interface->setBounds(3, 48, 70, 18);
    addAndMakeVisible(cipo1Interface.get());

    // Connect/Disconnect buttons
    connectButton = std::make_unique<UtilityButton>(stringConnect);
    connectButton->setFont(FontOptions("Small Text", 12, Font::bold));
    connectButton->setRadius(3.0f);
    connectButton->setBounds(6, 75, 65, 18);
    connectButton->addListener(this);
    addAndMakeVisible(connectButton.get());

    disconnectButton = std::make_unique<UtilityButton>(stringDisconnect);
    disconnectButton->setFont(FontOptions("Small Text", 12, Font::bold));
    disconnectButton->setRadius(3.0f);
    disconnectButton->setBounds(6, 75, 65, 18);
    disconnectButton->addListener(this);
    addAndMakeVisible(disconnectButton.get());
    disconnectButton->setVisible(false);
    
    // Rescan button
    rescanButton = std::make_unique<UtilityButton>("RESCAN");
    rescanButton->setRadius(3.0f);
    rescanButton->setBounds(6, 96, 65, 18);
    rescanButton->addListener(this);
    rescanButton->setTooltip("Auto-detect connected chips");
    addAndMakeVisible(rescanButton.get());
    rescanButton->setVisible(false);

    // Debug mode button
    debugModeButton = std::make_unique<UtilityButton>("DEBUG MODE");
    debugModeButton->setBounds(6, 117, 65, 18);
    debugModeButton->addListener(this);
    debugModeButton->setTooltip("Simulate 128 channels (2x64)");
    addAndMakeVisible(debugModeButton.get());
    debugModeButton->setVisible(false);

    debugModeActive = false;

    // ------------------------------------------------------------------
    // Aux sequencer test tooling (firmware aux-seq-v2). These three work
    // DURING acquisition -- that is their purpose.
    // ------------------------------------------------------------------
    statusButton = std::make_unique<UtilityButton>("STATUS");
    statusButton->setFont(FontOptions("Small Text", 12, Font::bold));
    statusButton->setRadius(3.0f);
    statusButton->setBounds(345, 28, 72, 18);
    statusButton->addListener(this);
    statusButton->setTooltip("Print full device status (incl. aux sequencer) to the console");
    addAndMakeVisible(statusButton.get());
    statusButton->setVisible(false);

    fastSettleButton = std::make_unique<UtilityButton>("SETTLE");
    fastSettleButton->setFont(FontOptions("Small Text", 12, Font::bold));
    fastSettleButton->setRadius(3.0f);
    fastSettleButton->setBounds(345, 53, 72, 18);
    fastSettleButton->addListener(this);
    fastSettleButton->setTooltip("Toggle amplifier fast settle (RHD Reg-0 D5) - hold for ~250 us per datasheet");
    addAndMakeVisible(fastSettleButton.get());
    fastSettleButton->setVisible(false);

    auxModeButton = std::make_unique<UtilityButton>("AUX SEQ");
    auxModeButton->setFont(FontOptions("Small Text", 12, Font::bold));
    auxModeButton->setRadius(3.0f);
    auxModeButton->setBounds(345, 78, 72, 18);
    auxModeButton->addListener(this);
    auxModeButton->setTooltip("Banked aux mode: accel sweep (1 axis/packet, echo de-interleave). "
                              "Toggling while acquiring exercises the live standby-bank swap");
    addAndMakeVisible(auxModeButton.get());
    auxModeButton->setVisible(false);

    fastSettleActive = false;
    auxModeActive = false;

    // Sample rate interface
    sampleRateInterface = std::make_unique<SampleRateInterface>(node);
    sampleRateInterface->setBounds(80, 22, 80, 50);
    addAndMakeVisible(sampleRateInterface.get());
    
    // Bandwidth interface
    bandwidthInterface = std::make_unique<BandwidthInterface>(node);
    bandwidthInterface->setBounds(80, 60, 80, 45);
    addAndMakeVisible(bandwidthInterface.get());
    
    // TTL Settle dropdown
    ttlSettleLabel = std::make_unique<Label>("TTL Settle", "TTL Settle");
    ttlSettleLabel->setFont(FontOptions("Inter", "Regular", 10.0f));
    ttlSettleLabel->setBounds(80, 100, 70, 20);
    addAndMakeVisible(ttlSettleLabel.get());
    
    ttlSettleCombo = std::make_unique<ComboBox>("TTLSettleCombo");
    ttlSettleCombo->setBounds(80, 115, 60, 18);
    ttlSettleCombo->addListener(this);
    ttlSettleCombo->addItem("-", 1);
    for (int k = 0; k < 8; k++)
    {
        ttlSettleCombo->addItem("TTL" + String(1 + k), 2 + k);
    }
    ttlSettleCombo->setSelectedId(1, dontSendNotification);
    addAndMakeVisible(ttlSettleCombo.get());
    
    // Add parameter editors for IP/ports on the right side
    addTextBoxParameterEditor(Parameter::PROCESSOR_SCOPE, "device_ip", 170, 25);
    addTextBoxParameterEditor(Parameter::PROCESSOR_SCOPE, "tcp_port", 170, 60);
    addTextBoxParameterEditor(Parameter::PROCESSOR_SCOPE, "udp_port", 255, 25);
    addTextBoxParameterEditor(Parameter::PROCESSOR_SCOPE, "data_scale", 255, 60);

    for (auto& ed : parameterEditors)
    {
        ed->setLayout(ParameterEditor::Layout::nameOnTop);
        ed->setBounds(ed->getX(), ed->getY(), 75, 30);
    }
}

void IntanSocketEditor::startAcquisition()
{
    rescanButton->setEnabledState(false);
    disconnectButton->setEnabled(false);
    disconnectButton->setAlpha(0.2f);
    debugModeButton->setEnabledState(false);
}

void IntanSocketEditor::stopAcquisition()
{
    if (node->errorFlag())
    {
        node->disconnectDevice();
    }
    
    rescanButton->setEnabledState(true);
    disconnectButton->setEnabled(true);
    disconnectButton->setAlpha(1.0f);
    debugModeButton->setEnabledState(true);
}

void IntanSocketEditor::buttonClicked(Button* button)
{
    if (button == connectButton.get() && !acquisitionIsActive)
    {
        node->connectDevice();
        CoreServices::updateSignalChain(this);
    }
    else if (button == disconnectButton.get() && !acquisitionIsActive)
    {
        node->disconnectDevice();
    }
    else if (button == rescanButton.get() && !acquisitionIsActive)
    {
        // Run auto-detection
        IntanInterface::AutoDetectionResult result;
        if (node->runAutoDetection(result, true))
        {
            updateChipDetection(result);
            
            if (result.success)
            {
                // Apply the configuration
                node->applyDetectionConfig(result);
                CoreServices::sendStatusMessage("Intan: Auto-detection successful");
                CoreServices::updateSignalChain(this);
            }
            else
            {
                CoreServices::sendStatusMessage("Intan: No chips detected");
            }
        }
    }
    else if (button == statusButton.get())
    {
        // Works during acquisition by design
        node->printDeviceStatus();
    }
    else if (button == fastSettleButton.get())
    {
        // Works during acquisition by design (that is the point of settle)
        fastSettleActive = !fastSettleActive;
        node->setManualFastSettle(fastSettleActive);
        refreshAuxButtons();
    }
    else if (button == auxModeButton.get())
    {
        // Works during acquisition by design: toggling while streaming
        // uploads the STANDBY banks and swaps them live.
        bool target = !auxModeActive;
        if (node->setAuxSequencerMode(target))
            auxModeActive = target;
        refreshAuxButtons();
    }
    else if (button == debugModeButton.get() && !acquisitionIsActive)
    {
        // Toggle the state
        debugModeActive = !debugModeActive;

        node->setDebugMode(debugModeActive);
        
        // Update button appearance based on state
        if (debugModeActive)
        {
            debugModeButton->setLabel("DEBUG: ON");
            debugModeButton->setColour(TextButton::buttonColourId, Colours::orange.darker(0.3f));
            CoreServices::sendStatusMessage("Intan: Debug mode enabled (128 channels)");
        }
        else
        {
            debugModeButton->setLabel("DEBUG MODE");
            debugModeButton->setColour(TextButton::buttonColourId, 
                                    findColour(TextButton::buttonColourId));
            CoreServices::sendStatusMessage("Intan: Debug mode disabled");
        }
    }

}

void IntanSocketEditor::comboBoxChanged(ComboBox* comboBox)
{
    if (comboBox == ttlSettleCombo.get())
    {
        // Id 1 = "-" (off); ids 2..9 = TTL1..TTL8 -> digital_in pins 0..7.
        // Fast settle then follows the selected pin level (sampled once per
        // packet in the PL; edge -> one injection packet each way).
        int selectedId = ttlSettleCombo->getSelectedId();
        node->setFastSettleTTLPin(selectedId >= 2 ? (selectedId - 2) : -1);
        refreshAuxButtons();
    }
}

void IntanSocketEditor::refreshAuxButtons()
{
    // Pull authoritative state from the node (it may have auto-enabled the
    // sequencer for fast settle, or synced state on reconnect)
    fastSettleActive = node->isFastSettleOn();
    auxModeActive = node->isAuxSequencerMode();

    if (fastSettleActive)
    {
        fastSettleButton->setLabel("SETTLE: ON");
        fastSettleButton->setColour(TextButton::buttonColourId, Colours::red.darker(0.2f));
    }
    else
    {
        fastSettleButton->setLabel("SETTLE");
        fastSettleButton->setColour(TextButton::buttonColourId,
                                    findColour(TextButton::buttonColourId));
    }

    if (auxModeActive)
    {
        auxModeButton->setLabel("AUX: ON");
        auxModeButton->setColour(TextButton::buttonColourId, Colours::orange.darker(0.3f));
    }
    else
    {
        auxModeButton->setLabel("AUX SEQ");
        auxModeButton->setColour(TextButton::buttonColourId,
                                 findColour(TextButton::buttonColourId));
    }
}

void IntanSocketEditor::connected()
{
    connectButton->setVisible(false);
    disconnectButton->setVisible(true);
    rescanButton->setVisible(true);
    debugModeButton->setVisible(true);
    statusButton->setVisible(true);
    fastSettleButton->setVisible(true);
    auxModeButton->setVisible(true);
    refreshAuxButtons();   // sync with device state (persists across reconnect)
}

void IntanSocketEditor::disconnected()
{
    connectButton->setVisible(true);
    disconnectButton->setVisible(false);
    rescanButton->setVisible(false);
    debugModeButton->setVisible(true);
    statusButton->setVisible(false);
    fastSettleButton->setVisible(false);
    auxModeButton->setVisible(false);

    // Reset chip displays
    cipo0Interface->updateChipStatus(false, IntanInterface::ChipType::NONE);
    cipo1Interface->updateChipStatus(false, IntanInterface::ChipType::NONE);
}

void IntanSocketEditor::updateChipDetection(const IntanInterface::AutoDetectionResult& result)
{
    cipo0Interface->updateChipStatus(result.cipo0Detected, result.cipo0ChipType);
    cipo1Interface->updateChipStatus(result.cipo1Detected, result.cipo1ChipType);
}
