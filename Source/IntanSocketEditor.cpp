#include "IntanSocketEditor.h"
#include "IntanSocket.h"

#include <iostream>
#include <string>

using namespace IntanSocketNode;

IntanSocketEditor::IntanSocketEditor (GenericProcessor* parentNode, IntanSocket* socket) : GenericEditor (parentNode)
{
    node = socket;

    desiredWidth = 180;

    // Add connect button
    connectButton = std::make_unique<UtilityButton> (stringConnect);
    connectButton->setFont (FontOptions ("Small Text", 12, Font::bold));
    connectButton->setRadius (3.0f);
    connectButton->setBounds (15, 35, 70, 20);
    connectButton->addListener (this);
    addAndMakeVisible (connectButton.get());

    disconnectButton = std::make_unique<UtilityButton> (stringDisconnect);
    disconnectButton->setFont (FontOptions ("Small Text", 12, Font::bold));
    disconnectButton->setRadius (3.0f);
    disconnectButton->setBounds (15, 35, 70, 20);
    disconnectButton->addListener (this);
    addAndMakeVisible (disconnectButton.get());
    disconnectButton->setVisible (false);
    
    addTextBoxParameterEditor(Parameter::PROCESSOR_SCOPE, "device_ip", 10, 60);
    addTextBoxParameterEditor(Parameter::PROCESSOR_SCOPE, "tcp_port", 10, 95);
    addTextBoxParameterEditor(Parameter::PROCESSOR_SCOPE, "udp_port", 95, 60);
    addTextBoxParameterEditor(Parameter::PROCESSOR_SCOPE, "data_scale", 95, 95);

    for (auto& ed : parameterEditors)
    {
        ed->setLayout (ParameterEditor::Layout::nameOnTop);
        ed->setBounds (ed->getX(), ed->getY(), 80, 30);
    }
}

void IntanSocketEditor::startAcquisition()
{
    disconnectButton->setEnabled (false);
    disconnectButton->setAlpha (0.2f);
}

void IntanSocketEditor::stopAcquisition()
{
    if (node->errorFlag())
    {
        node->disconnectDevice();
    }

    disconnectButton->setEnabled (true);
    disconnectButton->setAlpha (1.0f);
}

void IntanSocketEditor::buttonClicked (Button* button)
{
    if (button == connectButton.get() && ! acquisitionIsActive)
    {
        node->connectDevice();

        CoreServices::updateSignalChain (this);
    }
    else if (button == disconnectButton.get() && ! acquisitionIsActive)
    {
        node->disconnectDevice();
    }
}

void IntanSocketEditor::connected()
{
    connectButton->setVisible (false);
    disconnectButton->setVisible (true);
}

void IntanSocketEditor::disconnected()
{
    connectButton->setVisible (true);
    disconnectButton->setVisible (false);
}
