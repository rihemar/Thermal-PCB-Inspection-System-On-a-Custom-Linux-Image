#pragma once

#include <opencv2/opencv.hpp>

// ---------------------------------------------------------------------------
// generateFakeHeatmapFromImage
//
// Takes a normal RGB image of a PCB and synthesizes a *fake* low-resolution
// thermal heatmap (default 16x12, e.g. MLX90640-like) as if it had been
// captured by a thermal sensor with a DIFFERENT field of view (default 110
// degrees horizontal) than the RGB camera that took the input image.
//
// Because the two "cameras" don't share a FOV, the class:
//   1. Computes the sub-region (or padded/extrapolated region) of the RGB
//      frame that a 110 deg thermal sensor would actually observe, given
//      the RGB camera's own (assumed) FOV.
//   2. Converts that region into a synthetic per-pixel temperature field
//      based on image intensity + copper/trace detection (fake physics:
//      brighter / metallic-looking areas run hotter, e.g. current-carrying
//      copper pours and components).
//   3. Integrates (area-averages) that high-res temperature field down to
//      the thermal sensor's native resolution -- similar to what a real
//      microbolometer pixel does when it integrates radiation over its
//      instantaneous field of view.
//   4. Adds sensor noise (NETD-like Gaussian noise) for realism.
//
// This is purely a data-simulation utility (e.g. for testing thermal
// processing pipelines without real hardware) -- it does not read real
// thermal sensor data.
// ---------------------------------------------------------------------------
class generateFakeHeatmapFromImage
{
public:
    // rgbHFovDeg      : horizontal FOV of the RGB camera that captured the input image
    // thermalHFovDeg  : horizontal FOV of the (fake) thermal sensor, e.g. 110 deg
    // thermalRes      : native resolution of the fake thermal sensor (default 16x12)
    // ambientTempC    : baseline "cool" temperature assigned to dark/background regions
    // maxHotspotTempC : temperature assigned to the brightest / most "active" regions
    // noiseStdDevC    : standard deviation (deg C) of simulated sensor noise
    generateFakeHeatmapFromImage(double rgbHFovDeg = 55.0,
                             double thermalHFovDeg = 55.0,
                             cv::Size thermalRes = cv::Size(16, 12),
                             float ambientTempC = 25.0f,
                             float maxHotspotTempC = 85.0f,
                             double noiseStdDevC = 0.6);

    // Main entry point: takes an RGB (or BGR, as OpenCV normally loads) PCB
    // image and returns the fake thermal heatmap as a cv::Mat of type
    // CV_32FC1, size == thermalRes (16x12 by default), values in degrees C.
    cv::Mat getHeatmap(const cv::Mat& rgbImage);

    // Same as getHeatmap(), kept as a separate named step in case callers
    // want to be explicit about the generation stage.
    cv::Mat generateHeatmap(const cv::Mat& rgbImage);

    // Utility: turns a raw CV_32FC1 temperature map into a human-viewable
    // colorized, upscaled cv::Mat (CV_8UC3, BGR) for display/debugging.
    cv::Mat getColorizedHeatmap(const cv::Mat& rawHeatmapC,
                                 cv::Size displaySize = cv::Size(320, 240),
                                 int colormap = cv::COLORMAP_INFERNO,
                                 int interpolation = cv::INTER_CUBIC) const;

    // Accessors / tuning knobs
    void setFovs(double rgbHFovDeg, double thermalHFovDeg);
    void setThermalResolution(cv::Size res);
    void setTemperatureRange(float ambientTempC, float maxHotspotTempC);
    void setNoiseStdDev(double stdDevC);

    cv::Size thermalResolution() const { return thermalRes_; }
    double   thermalHFovDeg()   const { return thermalHFovDeg_; }
    double   rgbHFovDeg()       const { return rgbHFovDeg_; }

private:
    // Step 1: given the RGB image and the two FOVs, returns the sub-region
    // (crop) or padded/extrapolated version of rgbImage that represents
    // what the thermal sensor's 110 deg FOV would actually frame.
    cv::Mat computeFovAdjustedRegion(const cv::Mat& rgbImage) const;

    // Step 2: converts a BGR image region into a dense (same-size) fake
    // temperature field (CV_32FC1) using intensity + copper-color detection.
    cv::Mat estimateTemperatureField(const cv::Mat& regionBgr) const;

    // Step 3+4: area-averages temperatureField down to thermalRes_ and
    // adds Gaussian sensor noise.
    cv::Mat integrateToSensorResolution(const cv::Mat& temperatureField32f) const;

    double rgbHFovDeg_;
    double thermalHFovDeg_;
    cv::Size thermalRes_;
    float ambientTempC_;
    float maxHotspotTempC_;
    double noiseStdDevC_;
};