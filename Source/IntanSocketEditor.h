#ifndef __IntanSocketEditorH__
#define __IntanSocketEditorH__

#ifdef _WIN32
#include <Windows.h>
#endif

#include <VisualizerEditorHeaders.h>

#include "IntanSocket.h"

namespace IntanSocketNode
{
class IntanSocket;

class IntanSocketEditor : public GenericEditor,
                          public Button::Listener
{
public:
    /** Constructor */
    IntanSocketEditor (GenericProcessor* parentNode, IntanSocket* node);

    /** Button listener callback, called by button when pressed. */
    void buttonClicked (Button* button);

    /** Called by processor graph in beginning of the acquisition, disables editor completely. */
    void startAcquisition();

    /** Called by processor graph at the end of the acquisition, reenables editor completely. */
    void stopAcquisition();

    /** Called by the processor when the socket is connected. */
    void connected();

    /** Called by the processor when the socket is disconnected. */
    void disconnected();

private:
    // Button that connects/disconnects from/to server
    std::unique_ptr<UtilityButton> connectButton;
    std::unique_ptr<UtilityButton> disconnectButton;

    String stringConnect = "CONNECT";
    String stringDisconnect = "DISCONNECT";

    // Parent node
    IntanSocket* node;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (IntanSocketEditor);
};
} // namespace EphysSocketNode

#endif