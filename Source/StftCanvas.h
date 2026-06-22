/*
 * StftCanvas: scrolling heat-map spectrogram for the on-FPGA STFT stream
 * (one selectable lane, x = time, y = frequency, color = log power).
 *
 * Data flow: the firmware emits one jumbo UDP datagram per STFT pass on
 * port 5003; IntanInterface decodes the header and dispatches frames into
 * IntanSocket's STFT ring (raw complex float32, lane-major). The canvas
 * polls IntanSocket::drainStftSince() on each refresh and renders new
 * columns into a scrolling juce::Image.
 *
 * Frequency axis: bin b -> b * Fs_lfp / N Hz, where Fs_lfp = 30000 /
 * lfp_decim_R (the LFP rate feeding the STFT). At default LFP (R=15) and
 * STFT (N=64) the bins span 0..1000 Hz at ~31.25 Hz spacing.
 */
#ifndef __INTAN_STFT_CANVAS_H__
#define __INTAN_STFT_CANVAS_H__

#include <VisualizerWindowHeaders.h>
#include "IntanSocket.h"

namespace IntanSocketNode
{
class StftCanvas : public Visualizer,
                   public ComboBox::Listener
{
public:
    StftCanvas(GenericProcessor* p, IntanSocket* socket);
    ~StftCanvas() override = default;

    /** Called by Visualizer at refreshRate Hz while animating. */
    void refresh() override;
    void refreshState() override {}
    void resized() override;
    void paint(Graphics& g) override;

    void comboBoxChanged(ComboBox*) override;

private:
    IntanSocket* node = nullptr;

    std::unique_ptr<Label>    laneLabel;
    std::unique_ptr<ComboBox> laneCombo;
    std::unique_ptr<Label>    rangeModeLabel;
    std::unique_ptr<ComboBox> rangeModeCombo;     // 1 = Auto, 2 = Manual
    std::unique_ptr<Label>    infoLabel;

    int  selectedLane = 0;
    int  laneCount    = 0;      // K, from data
    int  nbinsKnown   = 0;
    int  hopKnown     = 0;
    int  lfpDecimR    = 15;
    bool autoRange    = true;
    float minDb = -20.0f;       // EMA-tracked when autoRange = true
    float maxDb =  40.0f;

    // Image: x = time (newest on the right), y = frequency (bin 0 at bottom).
    // Width is fixed; image height is nbins (small) and scaled in paint().
    static constexpr int IMAGE_WIDTH = 1024;
    int imageHeight = 33;
    std::unique_ptr<Image> spectrogram;

    uint32_t lastSeq  = 0xFFFFFFFFu;
    uint64_t framesSeen   = 0;
    uint64_t droppedTotal = 0;

    void rebuildLaneCombo(int K);
    void ensureImage(int nbins);

    // Map t in [0,1] -> a viridis-like RGB triple.
    static Colour viridis(float t);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(StftCanvas);
};
} // namespace IntanSocketNode

#endif
