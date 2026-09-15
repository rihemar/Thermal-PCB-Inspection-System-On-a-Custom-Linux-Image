#include "ChessboardCalibration.hpp"
#include <iostream>
#include <algorithm>

ChessboardCalibration::ChessboardCalibration(cv::Size boardSize, float squareSizeMm)
    : m_boardSize(boardSize), m_squareSizeMm(squareSizeMm) {}

cv::Mat ChessboardCalibration::NormalizeThermalForDetection(const cv::Mat& thermalImage) {
    cv::Mat gray;

    if (thermalImage.type() == CV_32F || thermalImage.type() == CV_64F) {
        cv::normalize(thermalImage, gray, 0, 255, cv::NORM_MINMAX, CV_8U);
    } else if (thermalImage.channels() == 3) {
        cv::cvtColor(thermalImage, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = thermalImage.clone();
        if (gray.type() != CV_8U) {
            cv::normalize(gray, gray, 0, 255, cv::NORM_MINMAX, CV_8U);
        }
    }

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    cv::Mat enhanced;
    clahe->apply(gray, enhanced);

    return enhanced;
}

std::vector<ChessboardCalibration::Corner> ChessboardCalibration::DetectCornersHarris(
    const cv::Mat& grayFloat, float threshRatio) {

    cv::Mat harrisResponse;
    cv::cornerHarris(grayFloat, harrisResponse, 2, 3, 0.04);

    cv::Mat dilated;
    cv::dilate(harrisResponse, dilated, cv::Mat());

    double minVal, maxVal;
    cv::minMaxLoc(harrisResponse, &minVal, &maxVal);
    float thresh = static_cast<float>(maxVal) * threshRatio;

    std::vector<Corner> corners;
    for (int r = 0; r < harrisResponse.rows; ++r) {
        for (int c = 0; c < harrisResponse.cols; ++c) {
            float val = harrisResponse.at<float>(r, c);
            float dilatedVal = dilated.at<float>(r, c);
            if (val > thresh && val == dilatedVal) {
                corners.push_back({(float)c, (float)r, val});
            }
        }
    }
    return corners;
}

std::vector<ChessboardCalibration::Corner> ChessboardCalibration::SuppressNearbyDuplicates(
    const std::vector<Corner>& corners, float minDist) {

    std::vector<Corner> sorted = corners;
    std::sort(sorted.begin(), sorted.end(), [](const Corner& a, const Corner& b) {
        return a.response > b.response;
    });

    std::vector<Corner> kept;
    for (const auto& c : sorted) {
        bool tooClose = false;
        for (const auto& k : kept) {
            float dx = c.x - k.x;
            float dy = c.y - k.y;
            if (dx * dx + dy * dy < minDist * minDist) {
                tooClose = true;
                break;
            }
        }
        if (!tooClose) kept.push_back(c);
    }
    return kept;
}

bool ChessboardCalibration::SortIntoGrid(std::vector<Corner>& corners) const {
    int expected = m_boardSize.width * m_boardSize.height;
    if ((int)corners.size() < expected) {
        std::cerr << "ChessboardCalibration: not enough corners detected: "
                  << corners.size() << " / expected " << expected << "\n";
        return false;
    }

    std::sort(corners.begin(), corners.end(), [](const Corner& a, const Corner& b) {
        return a.y < b.y;
    });

    if ((int)corners.size() > expected) {
        corners.resize(expected);
    }

    std::vector<Corner> gridSorted;
    for (int row = 0; row < m_boardSize.height; ++row) {
        int start = row * m_boardSize.width;
        int end = start + m_boardSize.width;
        if (end > (int)corners.size()) return false;

        std::vector<Corner> rowCorners(corners.begin() + start, corners.begin() + end);
        std::sort(rowCorners.begin(), rowCorners.end(), [](const Corner& a, const Corner& b) {
            return a.x < b.x;
        });
        gridSorted.insert(gridSorted.end(), rowCorners.begin(), rowCorners.end());
    }

    corners = gridSorted;
    return true;
}

bool ChessboardCalibration::DetectChessboardManual(const cv::Mat& gray,
                                                    std::vector<cv::Point2f>& outCorners) const {
    cv::Mat grayFloat;
    gray.convertTo(grayFloat, CV_32F, 1.0 / 255.0);

    std::vector<Corner> raw = DetectCornersHarris(grayFloat);
    std::vector<Corner> filtered = SuppressNearbyDuplicates(raw, 15.0f);

    if (!SortIntoGrid(filtered)) {
        return false;
    }

    outCorners.clear();
    for (const auto& c : filtered) {
        outCorners.push_back(cv::Point2f(c.x, c.y));
    }

    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01);
    cv::cornerSubPix(gray, outCorners, cv::Size(11, 11), cv::Size(-1, -1), criteria);

    return true;
}

ChessboardCalibration::PairResult ChessboardCalibration::DetectPair(const cv::Mat& rgbImage,
                                                                     const cv::Mat& thermalImage) const {
    PairResult result;

    cv::Mat rgbGray;
    if (rgbImage.channels() == 3) {
        cv::cvtColor(rgbImage, rgbGray, cv::COLOR_BGR2GRAY);
    } else {
        rgbGray = rgbImage;
    }

    cv::Mat thermalGray = NormalizeThermalForDetection(thermalImage);

    std::vector<cv::Point2f> rgbCorners, thermalCorners;
    bool foundRgb = DetectChessboardManual(rgbGray, rgbCorners);
    bool foundThermal = DetectChessboardManual(thermalGray, thermalCorners);

    if (!foundRgb || !foundThermal) {
        std::cerr << "ChessboardCalibration: chessboard not found in "
                  << (!foundRgb ? "RGB" : "")
                  << ((!foundRgb && !foundThermal) ? " and " : "")
                  << (!foundThermal ? "thermal" : "")
                  << " image.\n";
        result.success = false;
        return result;
    }

    auto isReversed = [](const std::vector<cv::Point2f>& a, const std::vector<cv::Point2f>& b) {
        cv::Point2f dirA = a.back() - a.front();
        cv::Point2f dirB = b.back() - b.front();
        return dirA.dot(dirB) < 0.f;
    };
    if (isReversed(rgbCorners, thermalCorners)) {
        std::reverse(thermalCorners.begin(), thermalCorners.end());
    }

    result.rgbCorners = std::move(rgbCorners);
    result.thermalCorners = std::move(thermalCorners);
    result.success = true;
    return result;
}

cv::Mat ChessboardCalibration::ComputeHomography(const cv::Mat& rgbImage, const cv::Mat& thermalImage,
                                                  double ransacReprojThreshold) const {
    PairResult pair = DetectPair(rgbImage, thermalImage);
    if (!pair.success) return cv::Mat();
    return ComputeHomography(pair, ransacReprojThreshold);
}

cv::Mat ChessboardCalibration::ComputeHomography(const PairResult& pair, double ransacReprojThreshold) {
    if (!pair.success || pair.thermalCorners.size() < 4) {
        std::cerr << "ChessboardCalibration: need at least 4 correspondences.\n";
        return cv::Mat();
    }

    cv::Mat H = cv::findHomography(pair.thermalCorners, pair.rgbCorners, cv::RANSAC, ransacReprojThreshold);
    if (H.empty()) {
        std::cerr << "ChessboardCalibration: findHomography failed to converge.\n";
    }
    return H;
}

double ChessboardCalibration::ComputeMeanReprojectionError(const cv::Mat& H, const PairResult& pair) {
    if (H.empty() || pair.thermalCorners.empty()) return -1.0;

    std::vector<cv::Point2f> projected;
    cv::perspectiveTransform(pair.thermalCorners, projected, H);

    double totalError = 0.0;
    for (size_t i = 0; i < projected.size(); ++i) {
        totalError += cv::norm(projected[i] - pair.rgbCorners[i]);
    }
    return totalError / static_cast<double>(projected.size());
}

bool ChessboardCalibration::SaveHomography(const std::string& path, const cv::Mat& H) {
    if (H.empty()) {
        std::cerr << "ChessboardCalibration::SaveHomography: homography is empty, not saving.\n";
        return false;
    }
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "ChessboardCalibration::SaveHomography: could not open " << path << " for writing.\n";
        return false;
    }
    fs << "homography_thermal_to_rgb" << H;
    fs.release();
    return true;
}

cv::Mat ChessboardCalibration::LoadHomography(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "ChessboardCalibration::LoadHomography: could not open " << path << " for reading.\n";
        return cv::Mat();
    }
    cv::Mat H;
    fs["homography_thermal_to_rgb"] >> H;
    fs.release();
    return H;
}