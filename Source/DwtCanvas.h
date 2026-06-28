/*
 * DwtCanvas: scrolling heat-map SCALOGRAM for the on-FPGA wavelet (DWT) stream
 * (one selectable lane, x = time, y = scale/frequency, color = magnitude).
 *
 * Adapted from the ephys-socket claude/dwt-visualizer canvas (which consumed
 * the OLD full-surface wavelet format: one datagram = one full column) to the
 * NEW UNIFIED octave-split / rate-aligned format (mz-unified-wavelet):
 *   - ONE datagram = ONE octave, emitted rate-aligned (octave o updates at
 *     3 kHz / 2^o). IntanInterface decodes each octave packet (stream_type=3)
 *     and IntanSocket::processWaveletPacket REASSEMBLES the full scalogram
 *     surface by HOLDING each octave's last value between its updates (a
 *     multirate surface), synthesizing one full column per octave-0 update.
 *   - The canvas just polls IntanSocket::drainWaveletSince() each refresh and
 *     renders the new full columns -- so this file's rendering is unchanged in
 *     spirit; the reassembly moved into IntanSocket (matching net.py).
 *
 * Rendering details (same as the prior canvas / remote/dwt_plot.py so all
 * viewers look alike):
 *   - Magnitude per bin already computed in IntanSocket (sqrt(re^2+im^2)).
 *   - The frequency axis is LOG, from the wavelet center-freq table (Morse
 *     bank, fs=3000): bin s = octave*n_voices + voice; bin 0 = highest band.
 *   - Color normalization is PER-SCALE (per row) by default so the 1/f roll-off
 *     doesn't bury the high-frequency rows.
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
