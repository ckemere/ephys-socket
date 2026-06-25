/*
 * DwtCanvas: scrolling heat-map SCALOGRAM for the on-FPGA wavelet (DWT) stream
 * (one selectable lane, x = time, y = scale/frequency, color = magnitude).
 *
 * Adapted from the STFT spectrogram canvas (origin/claude/stft-spectrogram,
 * Source/StftCanvas.*). Same kind of thing -- a scrolling time-frequency
 * heat-map -- retargeted at the wavelet scalogram UDP stream on port 5004.
 *
 * Differences from the STFT canvas:
 *   - Payload is complex INT32 (not float32); magnitude = sqrt(re^2+im^2).
 *   - The frequency axis is LOG and comes from the wavelet center-freq table
 *     (Morse bank, fs=3000) -- bin s = octave*n_voices + voice; bin 0 is the
 *     highest band, the last bin the lowest (~2..1000 Hz over 8 octaves).
 *   - Color normalization is PER-SCALE (per row) by default so the 1/f roll-off
 *     doesn't bury the high-frequency rows (low freqs dominate the energy).
 *
 * Data flow: IntanInterface binds UDP 5004, decodes the header + payload, and
 * dispatches each column (one packet = one scalogram column, all K lanes x
 * nscales complex bins at one time step) into IntanSocket's wavelet ring. The
 * canvas polls IntanSocket::drainWaveletSince() each refresh and renders the
 * new columns into a scrolling juce::Image.
 */
#ifndef __INTAN_DWT_CANVAS_H__
#define __INTAN_DWT_CANVAS_H__

#include <VisualizerWindowHeaders.h>
#include "IntanSocket.h"
#include <vector>

namespace IntanSocketNode
{
class DwtCanvas : public Visualizer,
                  public ComboBox::Listener
{
public:
    DwtCanvas(GenericProcessor* p, IntanSocket* socket);
    ~DwtCanvas() override = default;

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
    std::unique_ptr<Label>    normModeLabel;
    std::unique_ptr<ComboBox> normCombo;          // 1 = Per-scale, 2 = Global
    std::unique_ptr<Label>    infoLabel;

    int  selectedLane = 0;
    int  laneCount    = 0;      // K, from data
    int  nscalesKnown = 0;
    int  nOctKnown    = 0;
    int  nVocKnown    = 0;
    bool perScaleNorm = true;   // per-row normalization vs global

    // Center frequency (Hz) for each scale bin. Rebuilt when nOct/nVoc change.
    // centers[s] for s = octave*n_voices + voice; bin 0 = highest freq.
    std::vector<float> centers;

    // Per-scale magnitude normalization reference (EMA-tracked) and global ref.
    std::vector<float> rowMax;
    float globMax = 1.0f;
    static constexpr float kFloorDb = -40.0f;     // log color floor

    // Image: x = time (newest on the right), y = scale (bin 0 at top = high f).
    static constexpr int IMAGE_WIDTH = 1024;
    int imageHeight = 32;
    std::unique_ptr<Image> scalogram;

    uint64_t framesSeen   = 0;
    uint64_t droppedTotal = 0;
    uint32_t lastSeq      = 0xFFFFFFFFu;

    void rebuildLaneCombo(int K);
    void ensureImage(int nscales);
    void rebuildCenters(int nOct, int nVoc);

    // magnitude at scale s -> [0,1] color coordinate (log, per-scale or global)
    float colorT(float mag, int s) const;

    // Map t in [0,1] -> a viridis-like RGB triple.
    static Colour viridis(float t);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(DwtCanvas);
};
} // namespace IntanSocketNode

#endif
