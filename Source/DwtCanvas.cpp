#include "DwtCanvas.h"
#include <cmath>
#include <limits>

using namespace IntanSocketNode;

namespace
{
// 6-anchor viridis approximation (perceptual-ish), identical anchors to the
// prior canvas + remote/dwt_plot.py so all viewers look the same.
struct ViridisAnchor { float t; uint8_t r, g, b; };
constexpr ViridisAnchor kViridis[] = {
    { 0.00f,  68,   1,  84 },
    { 0.20f,  72,  37, 118 },
    { 0.40f,  62,  73, 137 },
    { 0.60f,  31, 158, 137 },
    { 0.80f, 109, 205,  89 },
    { 1.00f, 253, 231,  37 },
};
constexpr int kViridisN = sizeof(kViridis) / sizeof(kViridis[0]);

// Wavelet bank constants -- MUST match remote/net.py design_wavelet_bank()
// and the firmware. fs = 3000 (the LFP->3 kHz rate feeding the DWT engine).
constexpr double kWavFs    = 3000.0;
constexpr double kWavFcTop = 0.34;     // top voice center, cycles/sample
}

Colour DwtCanvas::viridis(float t)
{
    if (t <= 0.0f) return Colour::fromRGB(kViridis[0].r, kViridis[0].g, kViridis[0].b);
    if (t >= 1.0f) return Colour::fromRGB(kViridis[kViridisN - 1].r,
                                          kViridis[kViridisN - 1].g,
                                          kViridis[kViridisN - 1].b);
    for (int i = 1; i < kViridisN; ++i)
    {
        if (t <= kViridis[i].t)
        {
            float t0 = kViridis[i - 1].t, t1 = kViridis[i].t;
            float a  = (t - t0) / (t1 - t0);
            uint8_t r = (uint8_t)std::lround(kViridis[i - 1].r * (1 - a) + kViridis[i].r * a);
            uint8_t g = (uint8_t)std::lround(kViridis[i - 1].g * (1 - a) + kViridis[i].g * a);
            uint8_t b = (uint8_t)std::lround(kViridis[i - 1].b * (1 - a) + kViridis[i].b * a);
            return Colour::fromRGB(r, g, b);
        }
    }
    return Colours::black;
}

DwtCanvas::DwtCanvas(GenericProcessor* p, IntanSocket* socket)
    : Visualizer(p), node(socket)
{
    refreshRate = 30;   // Hz; the Visualizer base calls refresh() at this rate

    laneLabel = std::make_unique<Label>("LaneL", "Lane");
    laneLabel->setFont(FontOptions("Inter", "Regular", 13.0f));
    addAndMakeVisible(laneLabel.get());

    laneCombo = std::make_unique<ComboBox>("Lane");
    laneCombo->addListener(this);
    laneCombo->addItem("0", 1);     // populated for real on first frame
    laneCombo->setSelectedId(1, dontSendNotification);
    addAndMakeVisible(laneCombo.get());

    normModeLabel = std::make_unique<Label>("NormL", "Norm");
    normModeLabel->setFont(FontOptions("Inter", "Regular", 13.0f));
    addAndMakeVisible(normModeLabel.get());

    normCombo = std::make_unique<ComboBox>("NormMode");
    normCombo->addListener(this);
    normCombo->addItem("Per-scale", 1);   // each frequency row self-normalizes (1/f safe)
    normCombo->addItem("Global", 2);      // one scale across all bins
    normCombo->setSelectedId(1, dontSendNotification);
    addAndMakeVisible(normCombo.get());

    infoLabel = std::make_unique<Label>("Info", "");
    infoLabel->setFont(FontOptions("Inter", "Regular", 12.0f));
    addAndMakeVisible(infoLabel.get());

    ensureImage(imageHeight);
}

void DwtCanvas::ensureImage(int nscales)
{
    if (nscales <= 0) nscales = 1;
    if (scalogram == nullptr || scalogram->getHeight() != nscales) {
        scalogram = std::make_unique<Image>(Image::RGB, IMAGE_WIDTH, nscales,
                                             true, SoftwareImageType());
        scalogram->clear(scalogram->getBounds(), Colours::black);
        imageHeight = nscales;
    }
}

// Center frequencies, mirroring net.py design_wavelet_bank(): octave o has
// effective rate fs/2^o; voice v center = fc_top * 2^(-v/V) * rate. Scale index
// s = o*V + v, so bin 0 is the highest frequency, the last bin the lowest.
void DwtCanvas::rebuildCenters(int nOct, int nVoc)
{
    centers.clear();
    if (nOct <= 0 || nVoc <= 0) return;
    centers.reserve((size_t)nOct * nVoc);
    for (int o = 0; o < nOct; ++o) {
        double fr = kWavFs / std::pow(2.0, (double)o);
        for (int v = 0; v < nVoc; ++v)
            centers.push_back((float)(kWavFcTop * std::pow(2.0, -(double)v / (double)nVoc) * fr));
    }
}

void DwtCanvas::rebuildLaneCombo(int K)
{
    if (K <= 0 || K == laneCount) return;
    laneCount = K;
    int prev = laneCombo->getSelectedId();
    laneCombo->clear(dontSendNotification);
    for (int i = 0; i < K; ++i)
        laneCombo->addItem(String(i), i + 1);
    if (prev >= 1 && prev <= K)
        laneCombo->setSelectedId(prev, dontSendNotification);
    else
        laneCombo->setSelectedId(1, dontSendNotification);
    selectedLane = laneCombo->getSelectedId() - 1;
}

void DwtCanvas::resized()
{
    int row = 8;
    laneLabel->setBounds(10, row, 40, 18);
    laneCombo->setBounds(55, row, 80, 18);
    normModeLabel->setBounds(150, row, 50, 18);
    normCombo->setBounds(205, row, 100, 18);
    infoLabel->setBounds(320, row, getWidth() - 330, 18);
}

void DwtCanvas::comboBoxChanged(ComboBox* cb)
{
    if (cb == laneCombo.get()) {
        selectedLane = laneCombo->getSelectedId() - 1;
        if (selectedLane < 0) selectedLane = 0;
        if (scalogram) scalogram->clear(scalogram->getBounds(), Colours::black);
    }
    else if (cb == normCombo.get()) {
        perScaleNorm = (normCombo->getSelectedId() == 1);
    }
}

float DwtCanvas::colorT(float mag, int s) const
{
    float ref;
    if (perScaleNorm)
        ref = (s >= 0 && s < (int)rowMax.size() && rowMax[s] > 1e-6f) ? rowMax[s] : 1.0f;
    else
        ref = (globMax > 1e-6f) ? globMax : 1.0f;
    if (mag <= 0.0f) return 0.0f;
    float ratio = mag / ref;
    if (ratio <= 0.0f) return 0.0f;
    float db = 20.0f * std::log10(ratio);
    float t = (db - kFloorDb) / (0.0f - kFloorDb);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return t;
}

void DwtCanvas::refresh()
{
    if (node == nullptr) return;

    IntanSocket::WaveletDrain d = node->drainWaveletSince(lastSeq);

    if (d.columns.empty())
    {
        String info;
        info << "lane=" << selectedLane << "/" << jmax(1, laneCount)
             << "  framesSeen=" << (int64)framesSeen
             << "  dropped=" << (int64)droppedTotal
             << (node->isWaveletRunning() ? "" : "  (no wavelet stream)");
        infoLabel->setText(info, dontSendNotification);
        repaint();
        return;
    }

    // Pick up config from the drain.
    if (d.K > 0)        rebuildLaneCombo(d.K);
    if (d.nscales > 0)  ensureImage(d.nscales);
    if (d.nOct != nOctKnown || d.nVoc != nVocKnown) {
        rebuildCenters(d.nOct, d.nVoc);
        rowMax.assign(jmax(1, d.nscales), 1.0f);
    }
    nscalesKnown = d.nscales;
    nOctKnown    = d.nOct;
    nVocKnown    = d.nVoc;

    droppedTotal = d.dropped;
    framesSeen  += (uint64_t)d.columns.size();

    int K = d.K, ns = d.nscales;
    if (K <= 0 || ns <= 0 || selectedLane >= K) {
        lastSeq = d.columns.back().seq;
        repaint();
        return;
    }
    if ((int)rowMax.size() != ns) rowMax.assign(ns, 1.0f);

    // NEW format: the column already holds magnitudes, lane-major:
    //   mags[L * ns + s]  (s = octave*n_voices + voice; bin 0 = highest freq)
    int laneOffset = selectedLane * ns;
    int nNew = (int)d.columns.size();

    int imgW = scalogram->getWidth();
    int imgH = scalogram->getHeight();

    // Shift the image left by nNew pixels (cap at imgW) so the freshest column
    // lands at x = imgW - 1.
    int shift = jmin(nNew, imgW);
    if (shift > 0 && shift < imgW)
        scalogram->moveImageSection(0, 0, shift, 0, imgW - shift, imgH);
    if (shift >= imgW)
        scalogram->clear(scalogram->getBounds(), Colours::black);

    // If more columns arrived than fit, render only the newest `shift`.
    int colOffset = nNew - shift;

    // First pass: gather each new column's magnitude row for the selected lane
    // and update the EMA normalization references.
    std::vector<std::vector<float>> magColumns;
    magColumns.reserve(shift);
    constexpr float alpha = 0.02f;
    for (int c = colOffset; c < nNew; ++c)
    {
        const auto& col = d.columns[c];
        if ((int)col.mags.size() < laneOffset + ns) {
            magColumns.emplace_back();
            continue;
        }
        std::vector<float> m(ns);
        float gmax = 0.0f;
        for (int s = 0; s < ns; ++s) {
            float mag = col.mags[laneOffset + s];
            m[s] = mag;
            if (mag > gmax) gmax = mag;
            rowMax[s] = std::max(mag, (1.0f - alpha) * rowMax[s] + alpha * mag);
        }
        globMax = std::max(gmax, (1.0f - alpha) * globMax + alpha * gmax);
        magColumns.emplace_back(std::move(m));
    }

    // Second pass: paint columns. Newest land at the right. y = 0 (top) is the
    // highest frequency (bin 0); higher bin index -> lower row -> lower freq.
    int startX = imgW - shift;
    for (int i = 0; i < shift; ++i)
    {
        int x = startX + i;
        const auto& m = magColumns[i];
        if ((int)m.size() != ns) continue;
        for (int s = 0; s < ns; ++s) {
            int y = s;   // bin 0 (highest freq) at the top row
            if (y >= imgH) break;
            scalogram->setPixelAt(x, y, viridis(colorT(m[s], s)));
        }
    }

    lastSeq = d.columns.back().seq;

    float fHi = centers.empty() ? 0.0f : centers.front();
    float fLo = centers.empty() ? 0.0f : centers.back();
    String info;
    info << "lane=" << selectedLane << "/" << laneCount
         << "  K=" << K << "  scales=" << ns
         << "  oct=" << nOctKnown << " V=" << nVocKnown
         << "  " << String(fLo, 1) << "-" << String(fHi, 1) << " Hz"
         << "  norm=" << (perScaleNorm ? "per-scale" : "global")
         << "  seen=" << (int64)framesSeen
         << "  drop=" << (int64)droppedTotal;
    infoLabel->setText(info, dontSendNotification);

    repaint();
}

void DwtCanvas::paint(Graphics& g)
{
    g.fillAll(findColour(ThemeColours::componentParentBackground));

    if (scalogram == nullptr) return;

    const int topMargin   = 36;
    const int leftMargin   = 66;   // frequency axis (Hz labels)
    const int rightMargin  = 12;
    const int botMargin    = 24;   // time axis

    int x = leftMargin;
    int y = topMargin;
    int w = getWidth()  - leftMargin - rightMargin;
    int h = getHeight() - topMargin  - botMargin;
    if (w <= 0 || h <= 0) return;

    g.drawImage(*scalogram, x, y, w, h,
                0, 0, scalogram->getWidth(), scalogram->getHeight());

    // Frequency axis: log scale, labels from the center-freq table. bin 0
    // (highest freq) is at the TOP. We place ~one label per octave row.
    g.setColour(findColour(ThemeColours::controlPanelText));
    g.setFont(FontOptions("Inter", "Regular", 11.0f));
    int ns = (int)centers.size();
    if (ns > 0)
    {
        int step = jmax(1, (nVocKnown > 0 ? nVocKnown : 4));  // ~1 tick/octave
        for (int s = 0; s < ns; s += step)
        {
            // center of scale-row s within the drawn image
            int yy = y + (int)(((float)s + 0.5f) / (float)ns * h);
            String lbl = String((int)std::lround(centers[s])) + " Hz";
            g.drawText(lbl, 2, yy - 7, leftMargin - 6, 14,
                       Justification::centredRight, false);
        }
    }
    else
    {
        // No data yet -- nominal log span 2..512 Hz.
        for (int t = 0; t <= 5; ++t) {
            float frac = (float)t / 5.0f;
            int yy = y + (int)(frac * h);
            double freq = 512.0 * std::pow(2.0, -frac * 8.0);    // 512 down to 2
            g.drawText(String((int)std::lround(freq)) + " Hz", 2, yy - 7,
                       leftMargin - 6, 14, Justification::centredRight, false);
        }
    }

    // Time axis label (the wavelet monitor stream's column rate is set by the
    // engine's octave-0 update rate, so we don't claim an exact dt -- just
    // orientation).
    g.drawText("time -> (newest at right)", x, y + h + 4, w, 14,
               Justification::centredLeft, false);
}
