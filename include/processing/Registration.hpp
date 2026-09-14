#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// Static utility class for aligning an RGB guide frame to the MLX90641
// thermal camera's coordinate frame. Two alignment paths are provided:
//
//   1) Homography-based (ComputeHomography + AlignToThermal): accurate,
//      needs a one-time calibration with point correspondences visible
//      in both modalities (e.g. a checkerboard heated/backlit so it
//      shows up in both the RGB and thermal images).
//
//   2) Crop+scale-based (AlignByCropScale): a quick fallback for a fixed
//      rigid mount where you've manually measured the overlapping
//      region once and don't need per-pixel calibration accuracy.
//
// All methods are static -- call them directly, no instance needed.
class Registration {
public:
    // Compute a homography mapping RGB-frame points -> thermal-frame
    // points from matched correspondences. Needs >= 4 non-collinear
    // point pairs. Typically a one-time calibration step.
    static cv::Mat ComputeHomography(const std::vector<cv::Point2f>& rgbPoints,
                                      const std::vector<cv::Point2f>& thermalPoints);

    // Warp an RGB frame into the thermal camera's pixel grid using a
    // previously computed homography. `outputSize` is the resolution you
    // want the aligned guide at (this can be larger than the thermal
    // sensor's native resolution -- that's the point, it's the upsample
    // target size fed into JointBilateralUpsample).
    static cv::Mat AlignToThermal(const cv::Mat& rgbFrame,
                                   const cv::Mat& homography,
                                   const cv::Size& outputSize);

    // Fallback alignment for a fixed rigid mount: crop the sub-region of
    // the RGB frame that overlaps the thermal FOV, then resize it to
    // outputSize. Use this once you've manually determined roiInRgb
    // (by eye, a ruler, or a single calibration shot) and don't want to
    // do full homography calibration.
    static cv::Mat AlignByCropScale(const cv::Mat& rgbFrame,
                                     const cv::Rect& roiInRgb,
                                     const cv::Size& outputSize);

    // Persist/reload a computed homography so calibration only needs to
    // be done once per physical camera rig, not on every run.
    static bool SaveHomography(const cv::Mat& homography, const std::string& path);
    static cv::Mat LoadHomography(const std::string& path);
};