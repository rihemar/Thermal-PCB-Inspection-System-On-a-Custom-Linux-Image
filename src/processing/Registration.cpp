#include "Registration.hpp"
#include <iostream>

cv::Mat Registration::ComputeHomography(const std::vector<cv::Point2f>& rgbPoints,
                                         const std::vector<cv::Point2f>& thermalPoints) {
    if (rgbPoints.size() < 4 || rgbPoints.size() != thermalPoints.size()) {
        std::cerr << "Registration::ComputeHomography needs >= 4 matched point pairs\n";
        return cv::Mat();
    }
    // RANSAC gives robustness against a few mis-clicked/mis-detected
    // correspondences during calibration.
    cv::Mat H = cv::findHomography(rgbPoints, thermalPoints, cv::RANSAC);
    return H;
}
 cv::Mat Registration::AlignToThermal(const cv::Mat& rgbFrame,
                                      const cv::Mat& homography,
                                      const cv::Size& outputSize) {
    if (homography.empty()) {
        std::cerr << "Registration::AlignToThermal called with empty homography\n";
        return cv::Mat();
    }
    cv::Mat aligned;
    cv::warpPerspective(rgbFrame, aligned, homography, outputSize,
                         cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    return aligned;
}

cv::Mat Registration::AlignByCropScale(const cv::Mat& rgbFrame,
                                        const cv::Rect& roiInRgb,
                                        const cv::Size& outputSize) {
    cv::Rect safeRoi = roiInRgb & cv::Rect(0, 0, rgbFrame.cols, rgbFrame.rows);
    if (safeRoi.width <= 0 || safeRoi.height <= 0) {
        std::cerr << "Registration::AlignByCropScale: ROI does not overlap frame\n";
        return cv::Mat();
    }
    cv::Mat cropped = rgbFrame(safeRoi);
    cv::Mat aligned;
    cv::resize(cropped, aligned, outputSize, 0, 0, cv::INTER_LINEAR);
    return aligned;
}

bool Registration::SaveHomography(const cv::Mat& homography, const std::string& path) {
    if (homography.empty()) return false;
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) return false;
    fs << "homography" << homography;
    fs.release();
    return true;
}

cv::Mat Registration::LoadHomography(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    cv::Mat H;
    if (!fs.isOpened()) {
        std::cerr << "Registration::LoadHomography: could not open " << path << "\n";
        return H;
    }
    fs["homography"] >> H;
    fs.release();
    return H;
}