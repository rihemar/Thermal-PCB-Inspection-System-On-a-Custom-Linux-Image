#pragma once
#include <opencv2/opencv.hpp>

// Choice of guide signal extracted from the RGB frame:
//   RAW    - plain normalized grayscale intensity. Cheapest, works
//            reasonably when RGB brightness correlates with thermal
//            boundaries.
//   SOBEL  - gradient magnitude. Usually the best default for
//            cross-modal fusion: it isolates structural edges and
//            ignores color/texture that has no thermal correlate.
//   CANNY  - binary edges converted to a continuous field via an
//            inverted, normalized distance transform (raw Canny output
//            is binary and doesn't work directly in a bilateral range
//            kernel).
enum class GuideMode { RAW, SOBEL, CANNY };

// Static class implementing Joint Bilateral Upsampling for fusing a
// low-resolution thermal frame (e.g. MLX90641, 16x12) with a
// higher-resolution RGB guide frame that has ALREADY been registered/
// aligned to the thermal camera's field of view (see Registration).
//
// All methods are static -- call them directly, no instance needed.
//
// Typical usage:
//   cv::Mat aligned  = Registration::AlignToThermal(rgbFrame, H, cv::Size(640, 480));
//   cv::Mat enhanced = JointBilateralUpsample::Upsample(thermalFrame, aligned);
class JointBilateralUpsample {
public:
    // Build a [0,1]-normalized single-channel guide image from an RGB
    // (or already-grayscale) frame, per the chosen GuideMode.
    static cv::Mat PrepareGuide(const cv::Mat& rgbFrame, GuideMode mode = GuideMode::SOBEL);

    // Run one joint bilateral filtering pass: `input` (CV_32F, low- or
    // intermediate-resolution) is smoothed using edge weights drawn from
    // `guide` (CV_32F, same size as input, values in [0,1]). Result
    // written to `output` (CV_32F, same size).
    static void Filter(const cv::Mat& input, const cv::Mat& guide, cv::Mat& output,
                        int windowSize = 5, float sigmaSpectral = 0.15f, double sigmaSpatial = 2.0);

    // Full pipeline: takes a low-res float thermal frame (CV_32F, °C or
    // any consistent unit) and a full-resolution RGB guide frame that is
    // ALREADY ALIGNED to the thermal camera's FOV, and returns a
    // full-resolution enhanced thermal map (CV_32F, same size as guide).
    // Internally does iterative doubling + a final direct step so it
    // works for large, non-power-of-two upsampling factors (e.g. the
    // MLX90641's 16x12 -> 640x480 is a ~40x factor).
    static cv::Mat Upsample(const cv::Mat& thermalLowRes, const cv::Mat& rgbGuideAligned,
                             GuideMode mode = GuideMode::SOBEL,
                             int windowSize = 5, float sigmaSpectral = 0.15f, double sigmaSpatial = 2.0);

    // Convenience: colorize a float temperature map to a viewable BGR
    // image for display/saving. minC/maxC set the color scale range.
    static cv::Mat Colorize(const cv::Mat& tempsFloat, double minC, double maxC,
                             int colormap = cv::COLORMAP_INFERNO);
// Overlays Canny edges from the original PCB image onto a colorized
// heatmap, alpha-blended. originalRgb is resized to match colorizedHeatmap
// if the sizes differ (e.g. heatmap was upsampled to a different target
// resolution than the raw input image).
static cv::Mat OverlayEdges(const cv::Mat& colorizedHeatmap,
                             const cv::Mat& originalRgb,
                             double alpha = 0.6,
                             int cannyLow = 50,
                             int cannyHigh = 150,
                             cv::Scalar edgeColor = cv::Scalar(255, 255, 255));
private:
    static cv::Mat CreateGaussianKernel(int windowSize, double sigma);
};