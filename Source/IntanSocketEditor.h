#ifndef __IntanSocketEditorH__
#define __IntanSocketEditorH__

#ifdef _WIN32
#include <Windows.h>
#endif

#include <VisualizerEditorHeaders.h>
#include "IntanSocket.h"
#include "IntanInterface.h"

namespace IntanSocketNode
{
class IntanSocket;

/** UI component for bandwidth filter settings */
class BandwidthInterface : public Component,
                          public Label::Listener
{
public:
    BandwidthInterface(IntanSocket* node);
    ~BandwidthInterface() {}
    
    void paint(Graphics& g) override;
    void labelTextChanged(Label* label) override;
    
    void setLowerBandwidth(double value);
    void setUpperBandwidth(double value);
    double getLowerBandwidth() { return lowerBandwidth; }
    double getUpperBandwidth() { return upperBandwidth; }
    
private:
    IntanSocket* node;
    std::unique_ptr<Label> lowerBandwidthLabel;
    std::unique_ptr<Label> upperBandwidthLabel;
    double lowerBandwidth;
    double upperBandwidth;
};

/** UI component for sample rate selection */
class SampleRateInterface : public Component
{
public:
    SampleRateInterface(IntanSocket* node);
    ~SampleRateInterface() {}
    
    void paint(Graphics& g) override;
    
private:
    IntanSocket* node;
    std::unique_ptr<Label> rateLabel;
};

/** UI component for chip detection display */
class ChipInterface : public Component
{
public:
    ChipInterface(IntanSocket* node, int cipoIndex);
    ~ChipInterface() {}
    
    void paint(Graphics& g) override;
    void updateChipStatus(bool detected, IntanInterface::ChipType chipType);
    
private:
    IntanSocket* node;
    int cipoIndex;
    bool isDetected;
    IntanInterface::ChipType chipType;
    String label;
};

class IntanSocketEditor : public GenericEditor,
                          public Button::Listener,
                          public ComboBox::Listener
{
public:
    /** Constructor */
    IntanSocketEditor(GenericProcessor* parentNode, IntanSocket* node);

    /** Button listener callback */
    void buttonClicked(Button* button) override;
    
    /** ComboBox listener callback */
    void comboBoxChanged(ComboBox* comboBox) override;

    /** Called at start of acquisition */
    void startAcquisition() override;

    /** Called at end of acquisition */
    void stopAcquisition() override;

    /** Called when socket connects */
    void connected();

    /** Called when socket disconnects */
    void disconnected();
    
    /** Update chip detection display */
    void updateChipDetection(const IntanInterface::AutoDetectionResult& result);

private:
    // Buttons
    std::unique_ptr<UtilityButton> connectButton;
    std::unique_ptr<UtilityButton> disconnectButton;
    std::unique_ptr<UtilityButton> rescanButton;
    std::unique_ptr<UtilityButton> debugModeButton;
    bool debugModeActive;  // Track debug mode state
    
    // UI components
    std::unique_ptr<ChipInterface> cipo0Interface;
    std::unique_ptr<ChipInterface> cipo1Interface;
    std::unique_ptr<SampleRateInterface> sampleRateInterface;
    std::unique_ptr<BandwidthInterface> bandwidthInterface;
    
    // Dropdowns
    std::unique_ptr<ComboBox> ttlSettleCombo;
    std::unique_ptr<Label> ttlSettleLabel;

    String stringConnect = "CONNECT";
    String stringDisconnect = "DISCONNECT";

    // Parent node
    IntanSocket* node;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(IntanSocketEditor);
};

} // namespace IntanSocketNode

#endif
