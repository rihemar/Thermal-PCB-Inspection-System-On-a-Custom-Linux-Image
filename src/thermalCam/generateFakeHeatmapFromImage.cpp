#include "generateFakeHeatmapFromImage.hpp"
#include <algorithm>
#include <cmath>

namespace {
    // Clamp helper (pre-C++17-friendly, though std::clamp also works in C++17+)
    template <typename T>
    T clampVal(T v, T lo, T hi) { return std::max(lo, std::min(v, hi)); }
}

generateFakeHeatmapFromImage::generateFakeHeatmapFromImage(double rgbHFovDeg,
                                                   double thermalHFovDeg,
                                                   cv::Size thermalRes,
                                                   float ambientTempC,
                                                   float maxHotspotTempC,
                                                   double noiseStdDevC)
    : rgbHFovDeg_(rgbHFovDeg),
      thermalHFovDeg_(thermalHFovDeg),
      thermalRes_(thermalRes),
      ambientTempC_(ambientTempC),
      maxHotspotTempC_(maxHotspotTempC),
      noiseStdDevC_(noiseStdDevC)
{
}

void generateFakeHeatmapFromImage::setFovs(double rgbHFovDeg, double thermalHFovDeg)
{
    rgbHFovDeg_ = rgbHFovDeg;
    thermalHFovDeg_ = thermalHFovDeg;
}

void generateFakeHeatmapFromImage::setThermalResolution(cv::Size res)
{
    thermalRes_ = res;
}

void generateFakeHeatmapFromImage::setTemperatureRange(float ambientTempC, float maxHotspotTempC)
{
    ambientTempC_ = ambientTempC;
    maxHotspotTempC_ = maxHotspotTempC;
}

void generateFakeHeatmapFromImage::setNoiseStdDev(double stdDevC)
{
    noiseStdDevC_ = stdDevC;
}

// ---------------------------------------------------------------------------
// Step 1: FOV reconciliation
// ---------------------------------------------------------------------------
// We model each camera as a simple pinhole with a given horizontal FOV.
// The ratio of "how much physical width is seen" scales with tan(FOV/2).
// If the thermal sensor's FOV is narrower than the RGB camera's, it sees a
// cropped central region of the RGB frame. If it's wider (typical for a
// 110 deg thermal lens vs. a normal RGB lens), it sees MORE than the RGB
// frame captured -- since we don't have real pixel data for that extra
// area, we extrapolate it by replicating/blurring the border region. This
// keeps the class fully self-contained without requiring extra source
// images, while still visibly reflecting the FOV mismatch.
cv::Mat generateFakeHeatmapFromImage::computeFovAdjustedRegion(const cv::Mat& rgbImage) const
{
    CV_Assert(!rgbImage.empty());

    const double rgbHalfTan = std::tan((rgbHFovDeg_ * CV_PI / 180.0) / 2.0);
    const double thermalHalfTan = std::tan((thermalHFovDeg_ * CV_PI / 180.0) / 2.0);
    const double ratio = thermalHalfTan / rgbHalfTan; // >1 => thermal sees a wider scene

    const int w = rgbImage.cols;
    const int h = rgbImage.rows;

    if (std::abs(ratio - 1.0) < 1e-6) {
        return rgbImage.clone();
    }

    if (ratio < 1.0) {
        // Thermal FOV narrower than RGB FOV -> crop the central region.
        int cropW = clampVal(static_cast<int>(std::round(w * ratio)), 1, w);
        int cropH = clampVal(static_cast<int>(std::round(h * ratio)), 1, h);
        cv::Rect roi((w - cropW) / 2, (h - cropH) / 2, cropW, cropH);
        return rgbImage(roi).clone();
    }

    // ratio > 1: thermal FOV wider than RGB FOV -> we need to "see" beyond
    // the edges of the RGB frame. We don't have real data there, so we
    // extrapolate by mirror-padding (keeps edge continuity more plausible
    // than plain replication for a PCB's fairly uniform background) and
    // then lightly blur just the padded border so it doesn't look like an
    // obvious hard-mirrored seam.
    int padW = static_cast<int>(std::round((w * (ratio - 1.0)) / 2.0));
    int padH = static_cast<int>(std::round((h * (ratio - 1.0)) / 2.0));
    padW = std::max(padW, 0);
    padH = std::max(padH, 0);

    cv::Mat padded;
    cv::copyMakeBorder(rgbImage, padded, padH, padH, padW, padW, cv::BORDER_REFLECT101);

    if (padW > 0 || padH > 0) {
        cv::Mat blurred;
        cv::GaussianBlur(padded, blurred, cv::Size(9, 9), 0);
        // Blend only near the border area so the original PCB stays sharp
        // while the extrapolated surround looks softly out-of-view.
        cv::Mat mask = cv::Mat::zeros(padded.size(), CV_8UC1);
        cv::rectangle(mask, cv::Rect(padW, padH, w, h), cv::Scalar(255), cv::FILLED);
        cv::Mat maskInv;
        cv::bitwise_not(mask, maskInv);
        cv::Mat result = padded.clone();
        blurred.copyTo(result, maskInv);
        return result;
    }

    return padded;
}

// ---------------------------------------------------------------------------
// Step 2: fake "temperature" estimation from image content
// ---------------------------------------------------------------------------
// Heuristic, not real physics: we treat brighter regions and copper/metal
// colored regions (solder pads, traces, exposed copper pours) as running
// warmer, since on real boards current-carrying copper and dense component
// clusters tend to dominate a thermal image. Dark solder-mask background is
// treated as closer to ambient.
cv::Mat generateFakeHeatmapFromImage::estimateTemperatureField(const cv::Mat& regionBgr) const
{
    CV_Assert(!regionBgr.empty());

    cv::Mat bgr;
    if (regionBgr.channels() == 1) {
        cv::cvtColor(regionBgr, bgr, cv::COLOR_GRAY2BGR);
    } else if (regionBgr.channels() == 4) {
        cv::cvtColor(regionBgr, bgr, cv::COLOR_BGRA2BGR);
    } else {
        bgr = regionBgr;
    }

    // 1) Base intensity contribution (0..1)
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::Mat grayF;
    gray.convertTo(grayF, CV_32F, 1.0 / 255.0);

    // 2) Copper / metallic-pad detection via HSV thresholding.
    //    Typical exposed copper/solder look: low saturation-to-mid, yellow/
    //    orange hues, and fairly high value (shiny). This is a coarse
    //    heuristic, tuned to be permissive rather than exact.
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsvCh;
    cv::split(hsv, hsvCh);
    cv::Mat hue = hsvCh[0], sat = hsvCh[1], val = hsvCh[2];

    cv::Mat copperMask;
    cv::inRange(hsv, cv::Scalar(5, 40, 90), cv::Scalar(35, 255, 255), copperMask); // yellow/orange, bright
    cv::Mat moduleMask;
    cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 70), moduleMask); // any hue/sat, dark = chip bodies
    cv::Mat silverMask;
    cv::inRange(hsv, cv::Scalar(0, 0, 150), cv::Scalar(180, 40, 255), silverMask); // low-sat, bright (solder/pads)
    cv::Mat metalMask;
    cv::bitwise_or(copperMask, silverMask, metalMask);

    cv::Mat moduleMaskClosed;
cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
cv::morphologyEx(moduleMask, moduleMaskClosed, cv::MORPH_CLOSE, kernel);

cv::Mat moduleMaskFiltered = cv::Mat::zeros(moduleMaskClosed.size(), CV_8UC1);
cv::Mat labels, stats, centroids;
int nLabels = cv::connectedComponentsWithStats(moduleMaskClosed, labels, stats, centroids);
const int minChipArea = 500;
for (int i = 1; i < nLabels; ++i) {
    if (stats.at<int>(i, cv::CC_STAT_AREA) >= minChipArea) {
        moduleMaskFiltered.setTo(255, (labels == i));
    }
}

cv::Mat FinalMask;
cv::bitwise_or(metalMask, moduleMaskFiltered, FinalMask);
cv::Mat FinalMaskF;
FinalMask.convertTo(FinalMaskF, CV_32F, 1.0 / 255.0);
cv::GaussianBlur(FinalMaskF, FinalMaskF, cv::Size(7, 7), 0);

cv::Mat combined = cv::max(grayF * 0.4f, FinalMaskF);
cv::normalize(combined, combined, 0.0, 1.0, cv::NORM_MINMAX);
cv::GaussianBlur(combined, combined, cv::Size(5, 5), 0);
    // cv::Mat FinalMask;
    // cv::bitwise_or(metalMask, moduleMask, FinalMask);

    // cv::Mat FinalMaskF;
    // FinalMask.convertTo(FinalMaskF, CV_32F, 1.0 / 255.0);

    // // Slightly smooth the metal mask so "hot" regions have soft gradients
    // // rather than hard binary edges, like real heat diffusion would.
    // cv::GaussianBlur(FinalMaskF, FinalMaskF, cv::Size(7, 7), 0);

    // // 3) Combine: weighted sum of raw intensity + metallic/hotspot boost.
    // cv::Mat combined = grayF * 0.4f + FinalMaskF * 0.6f;
    // cv::normalize(combined, combined, 0.0, 1.0, cv::NORM_MINMAX);

    // // Slight smoothing to emulate lateral heat spread on the board.
    // cv::GaussianBlur(combined, combined, cv::Size(5, 5), 0);

    // 4) Map normalized "activity" to a temperature range.
    cv::Mat tempField = ambientTempC_ + combined * (maxHotspotTempC_ - ambientTempC_);

    return tempField; // CV_32FC1, same size as regionBgr
}

// ---------------------------------------------------------------------------
// Step 3 + 4: downsample to sensor resolution + add sensor noise
// ------------------------------------f---------------------------------------
cv::Mat generateFakeHeatmapFromImage::integrateToSensorResolution(const cv::Mat& temperatureField32f) const
{
    CV_Assert(temperatureField32f.type() == CV_32FC1);

    cv::Mat lowRes;
    // INTER_AREA performs a local-area average, which is a reasonable stand-in
    // for how a real thermal pixel integrates radiation over its IFOV.
    cv::resize(temperatureField32f, lowRes, thermalRes_, 0, 0, cv::INTER_AREA);

    if (noiseStdDevC_ > 0.0) {
        cv::Mat noise(lowRes.size(), CV_32FC1);
        cv::randn(noise, 0.0, noiseStdDevC_);
        lowRes += noise;
    }

    return lowRes;
}

// ---------------------------------------------------------------------------
// Public pipeline
// ---------------------------------------------------------------------------
cv::Mat generateFakeHeatmapFromImage::generateHeatmap(const cv::Mat& rgbImage)
{
    CV_Assert(!rgbImage.empty());

    cv::Mat fovRegion = computeFovAdjustedRegion(rgbImage);
    cv::Mat tempField = estimateTemperatureField(fovRegion);
    cv::Mat heatmap = integrateToSensorResolution(tempField);
    return heatmap; // CV_32FC1, size == thermalRes_, values in degrees C
}

cv::Mat generateFakeHeatmapFromImage::getHeatmap(const cv::Mat& rgbImage)
{
    return generateHeatmap(rgbImage);
}

cv::Mat generateFakeHeatmapFromImage::getColorizedHeatmap(const cv::Mat& rawHeatmapC,
                                                       cv::Size displaySize,
                                                       int colormap,
                                                       int interpolation) const
{
    CV_Assert(rawHeatmapC.type() == CV_32FC1);

    cv::Mat normalized;
    cv::normalize(rawHeatmapC, normalized, 0, 255, cv::NORM_MINMAX);
    cv::Mat u8;
    normalized.convertTo(u8, CV_8UC1);

    cv::Mat colorSmall;
    cv::applyColorMap(u8, colorSmall, colormap);

    cv::Mat colorBig;
    cv::resize(colorSmall, colorBig, displaySize, 0, 0, interpolation);
    return colorBig; // CV_8UC3 BGR
}