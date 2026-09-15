#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// Computes a homography mapping a single flat chessboard between an RGB
// camera and a thermal camera (the board must be thermally visible too,
// e.g. heated/cooled or made of two different-emissivity materials).
class ChessboardCalibration {
public:
    struct PairResult {
        bool success = false;
        std::vector<cv::Point2f> rgbCorners;
        std::vector<cv::Point2f> thermalCorners;
    };

    // boardSize = number of INNER corners (cols, rows), e.g. cv::Size(9,6)
    // for a 10x7 square chessboard.
    ChessboardCalibration(cv::Size boardSize, float squareSizeMm = 1.0f);

    // Detect chessboard corners in one RGB/thermal image pair, using a
    // manual Harris-corner + non-max-suppression + grid-sort pipeline
    // instead of cv::findChessboardCorners.
    PairResult DetectPair(const cv::Mat& rgbImage, const cv::Mat& thermalImage) const;

    // Computes homography mapping THERMAL pixel coords -> RGB pixel coords
    // (H * thermalPoint ≈ rgbPoint) from a single detected pair.
    // Returns empty Mat on detection or fit failure.
    cv::Mat ComputeHomography(const cv::Mat& rgbImage, const cv::Mat& thermalImage,
                               double ransacReprojThreshold = 3.0) const;

    // Same as above, but from a PairResult you already have (avoids
    // re-running detection if you already called DetectPair yourself).
    static cv::Mat ComputeHomography(const PairResult& pair, double ransacReprojThreshold = 3.0);

    // Reprojection error diagnostic: mean pixel distance between
    // H * thermalCorners and rgbCorners.
    static double ComputeMeanReprojectionError(const cv::Mat& H, const PairResult& pair);

    static bool SaveHomography(const std::string& path, const cv::Mat& H);
    static cv::Mat LoadHomography(const std::string& path);

private:
    cv::Size m_boardSize;
    float m_squareSizeMm;

    struct Corner {
        float x, y;
        float response;
    };

    static cv::Mat NormalizeThermalForDetection(const cv::Mat& thermalImage);
    static std::vector<Corner> DetectCornersHarris(const cv::Mat& grayFloat, float threshRatio = 0.05f);
    static std::vector<Corner> SuppressNearbyDuplicates(const std::vector<Corner>& corners, float minDist);
    bool SortIntoGrid(std::vector<Corner>& corners) const;
    bool DetectChessboardManual(const cv::Mat& gray, std::vector<cv::Point2f>& outCorners) const;
};