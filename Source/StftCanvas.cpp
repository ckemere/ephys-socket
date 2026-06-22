#include "StftCanvas.h"
#include <cmath>

using namespace IntanSocketNode;

namespace
{
// 6-anchor viridis approximation (perceptual-ish). Good enough for a heat
// map; refine later if needed.
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
}

Colour StftCanvas::viridis(float t)
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

StftCanvas::StftCanvas(GenericProcessor* p, IntanSocket* socket)
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

    rangeModeLabel = std::make_unique<Label>("RangeL", "Range");
    rangeModeLabel->setFont(FontOptions("Inter", "Regular", 13.0f));
    addAndMakeVisible(rangeModeLabel.get());

    rangeModeCombo = std::make_unique<ComboBox>("RangeMode");
    rangeModeCombo->addListener(this);
    rangeModeCombo->addItem("Auto", 1);
    rangeModeCombo->addItem("Manual", 2);
    rangeModeCombo->setSelectedId(1, dontSendNotification);
    addAndMakeVisible(rangeModeCombo.get());

    infoLabel = std::make_unique<Label>("Info", "");
    infoLabel->setFont(FontOptions("Inter", "Regular", 12.0f));
    addAndMakeVisible(infoLabel.get());

    ensureImage(imageHeight);
}

void StftCanvas::ensureImage(int nbins)
{
    if (spectrogram == nullptr || spectrogram->getHeight() != nbins) {
        spectrogram = std::make_unique<Image>(Image::RGB, IMAGE_WIDTH, nbins,
                                              true, SoftwareImageType());
        spectrogram->clear(spectrogram->getBounds(), Colours::black);
        imageHeight = nbins;
    }
}

void StftCanvas::rebuildLaneCombo(int K)
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

void StftCanvas::resized()
{
    int row = 8;
    laneLabel->setBounds(10, row, 40, 18);
    laneCombo->setBounds(55, row, 80, 18);
    rangeModeLabel->setBounds(150, row, 50, 18);
    rangeModeCombo->setBounds(205, row, 90, 18);
    infoLabel->setBounds(310, row, getWidth() - 320, 18);
}

void StftCanvas::comboBoxChanged(ComboBox* cb)
{
    if (cb == laneCombo.get()) {
        selectedLane = laneCombo->getSelectedId() - 1;
        if (selectedLane < 0) selectedLane = 0;
        if (spectrogram) spectrogram->clear(spectrogram->getBounds(), Colours::black);
    }
    else if (cb == rangeModeCombo.get()) {
        autoRange = (rangeModeCombo->getSelectedId() == 1);
    }
}

void StftCanvas::refresh()
{
    if (node == nullptr) return;

    IntanSocket::StftDrain d = node->drainStftSince(lastSeq);
    if ((int)d.columns.size() == 0)
    {
        // Just keep the info text live.
        String info;
        info << "lane=" << selectedLane << "/" << jmax(1, laneCount)
             << "  framesSeen=" << (int64)framesSeen
             << "  dropped=" << (int64)droppedTotal;
        infoLabel->setText(info, dontSendNotification);
        repaint();
        return;
    }

    // Pick up config from the drain (may have just been set up).
    if (d.K > 0)      rebuildLaneCombo(d.K);
    if (d.nbins > 0)  ensureImage(d.nbins);
    nbinsKnown = d.nbins;
    hopKnown   = d.hop;
    lfpDecimR  = d.lfpDecimR;

    droppedTotal = d.dropped;
    framesSeen  += (uint64_t)d.columns.size();

    int K = d.K, nbins = d.nbins;
    if (K <= 0 || nbins <= 0 || selectedLane >= K) {
        // Bad config -- skip.
        lastSeq = d.columns.back().seq;
        return;
    }

    // Compute dB power for the selected lane across the new columns.
    // Lane L offset (lane-major payload): L * nbins * 2.
    int laneOffset = selectedLane * nbins * 2;
    int nNew = (int)d.columns.size();

    // Track auto-range min/max across just these new columns.
    float batchMin =  std::numeric_limits<float>::infinity();
    float batchMax = -std::numeric_limits<float>::infinity();

    // Working buffer for per-column dB values (reused across columns).
    static thread_local std::vector<float> dbCol;
    dbCol.resize(nbins);

    int imgW = spectrogram->getWidth();
    int imgH = spectrogram->getHeight();

    // Shift the image left by nNew pixels (cap at imgW) so the freshest
    // column lands at x = imgW - 1.
    int shift = jmin(nNew, imgW);
    if (shift > 0 && shift < imgW)
        spectrogram->moveImageSection(0, 0, shift, 0, imgW - shift, imgH);
    if (shift >= imgW)
        spectrogram->clear(spectrogram->getBounds(), Colours::black);

    // Render newest `shift` columns: if more columns arrived than fit, drop
    // the oldest ones (visualizer is best-effort).
    int colOffset = nNew - shift;

    // First pass: compute all dB columns + batch range
    std::vector<std::vector<float>> dbColumns;
    dbColumns.reserve(shift);
    for (int c = colOffset; c < nNew; ++c)
    {
        const auto& col = d.columns[c];
        if ((int)col.samples.size() < laneOffset + nbins * 2) {
            dbColumns.emplace_back();
            continue;
        }
        std::vector<float> v(nbins);
        for (int b = 0; b < nbins; ++b) {
            float re = col.samples[laneOffset + b * 2 + 0];
            float im = col.samples[laneOffset + b * 2 + 1];
            float p  = re * re + im * im;
            float db = p > 0.0f ? 10.0f * std::log10(p) : -120.0f;
            v[b] = db;
            if (db < batchMin) batchMin = db;
            if (db > batchMax) batchMax = db;
        }
        dbColumns.emplace_back(std::move(v));
    }

    // Track auto-range with a slow EMA so the scale is stable.
    if (autoRange && std::isfinite(batchMin) && std::isfinite(batchMax) &&
        batchMax > batchMin)
    {
        constexpr float alpha = 0.05f;
        minDb = (1 - alpha) * minDb + alpha * batchMin;
        maxDb = (1 - alpha) * maxDb + alpha * batchMax;
        if (maxDb - minDb < 1.0f) maxDb = minDb + 1.0f;
    }

    float visMin = minDb, visMax = maxDb;
    float invRange = 1.0f / (visMax - visMin);

    // Second pass: paint columns. New columns land at the right.
    int startX = imgW - shift;
    for (int i = 0; i < shift; ++i)
    {
        int x = startX + i;
        const auto& v = dbColumns[i];
        if ((int)v.size() != nbins) continue;
        for (int b = 0; b < nbins; ++b) {
            // y = 0 -> top of image = highest frequency. So mirror b.
            int y = imgH - 1 - b;
            float t = (v[b] - visMin) * invRange;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            spectrogram->setPixelAt(x, y, viridis(t));
        }
    }

    lastSeq = d.columns.back().seq;

    String info;
    info << "lane=" << selectedLane << "/" << laneCount
         << "  K=" << K << "  nbins=" << nbins
         << "  hop=" << hopKnown
         << "  Fs_lfp=" << (int)(30000.0 / jmax(1, lfpDecimR))
         << " Hz"
         << "  range=[" << String(visMin, 1) << ", " << String(visMax, 1) << "] dB"
         << "  framesSeen=" << (int64)framesSeen
         << "  dropped=" << (int64)droppedTotal;
    infoLabel->setText(info, dontSendNotification);

    repaint();
}

void StftCanvas::paint(Graphics& g)
{
    g.fillAll(findColour(ThemeColours::componentParentBackground));

    if (spectrogram == nullptr) return;

    // Heat-map area: leaves room for the top control row + an axis margin.
    const int topMargin   = 36;
    const int leftMargin  = 60;   // frequency axis
    const int rightMargin = 12;
    const int botMargin   = 24;   // time axis

    int x = leftMargin;
    int y = topMargin;
    int w = getWidth()  - leftMargin - rightMargin;
    int h = getHeight() - topMargin  - botMargin;
    if (w <= 0 || h <= 0) return;

    g.drawImage(*spectrogram, x, y, w, h,
                0, 0, spectrogram->getWidth(), spectrogram->getHeight());

    // Frequency axis: bin -> b * Fs_lfp / N. Use the data's known nbins.
    g.setColour(findColour(ThemeColours::controlPanelText));
    g.setFont(FontOptions("Inter", "Regular", 11.0f));
    int nbins = (nbinsKnown > 0 ? nbinsKnown : spectrogram->getHeight());
    int N = (nbins > 1) ? (2 * (nbins - 1)) : 64;
    double fs_lfp = 30000.0 / (lfpDecimR > 0 ? lfpDecimR : 15);
    double fmax = (double)(nbins - 1) * fs_lfp / (double)N;
    // 5 tick marks on freq axis.
    for (int t = 0; t <= 5; ++t)
    {
        float frac = (float)t / 5.0f;
        int yy = y + (int)((1.0f - frac) * h);
        double freq = frac * fmax;
        String s = String((int)std::lround(freq)) + " Hz";
        g.drawText(s, 2, yy - 7, leftMargin - 6, 14, Justification::centredRight, false);
    }

    // Time axis: x = imgW - 1 is "now", scrolls left. Each pixel = hop LFP
    // samples = hop/fs_lfp seconds. The visible window is w pixels * (image
    // pixel width / data pixel) -- the image is drawn at our width, so each
    // SCREEN x corresponds to (IMAGE_WIDTH / w) image pixels. We label in
    // seconds-from-now, increasing left.
    int hop = (hopKnown > 0 ? hopKnown : 1);
    double secsPerImagePx = (double)hop / fs_lfp;
    double imagePxPerScreenPx = (double)spectrogram->getWidth() / (double)w;
    double secsPerScreenPx = secsPerImagePx * imagePxPerScreenPx;
    double totalSpan = secsPerScreenPx * w;
    for (int t = 0; t <= 5; ++t)
    {
        float frac = (float)t / 5.0f;        // 0 = oldest (left), 1 = newest (right)
        int xx = x + (int)(frac * w);
        double secsFromNow = (1.0 - frac) * totalSpan;
        String s = "-" + String(secsFromNow, 2) + " s";
        if (t == 5) s = "now";
        g.drawText(s, xx - 30, y + h + 4, 60, 14, Justification::centred, false);
    }
}
