#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>

struct Corner {
    float x, y;
    float response;
};

std::vector<Corner> DetectCornersHarris(const cv::Mat& gray, float threshRatio = 0.05f) {
    cv::Mat harrisResponse;
    cv::cornerHarris(gray, harrisResponse, 2, 3, 0.04);

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

std::vector<Corner> SuppressNearbyDuplicates(const std::vector<Corner>& corners, float minDist) {
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

bool SortIntoGrid(std::vector<Corner>& corners, cv::Size boardSize) {
    int expected = boardSize.width * boardSize.height;
    if ((int)corners.size() < expected) {
        std::cerr << "Not enough corners detected: " << corners.size()
                  << " / expected " << expected << "\n";
        return false;
    }

    std::sort(corners.begin(), corners.end(), [](const Corner& a, const Corner& b) {
        return a.y < b.y;
    });

    if ((int)corners.size() > expected) {
        corners.resize(expected);
    }

    std::vector<Corner> gridSorted;
    for (int row = 0; row < boardSize.height; ++row) {
        int start = row * boardSize.width;
        int end = start + boardSize.width;
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

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <image>\n";
        return 1;
    }

    cv::Mat img = cv::imread(argv[1], cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR);
    if (img.empty()) {
        std::cerr << "Failed to load image\n";
        return 1;
    }

    cv::Mat gray;
    if (img.type() == CV_32F || img.type() == CV_64F) {
        cv::normalize(img, gray, 0, 255, cv::NORM_MINMAX, CV_8U);
    } else if (img.channels() == 3) {
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = img;
    }

    cv::Mat grayFloat;
    gray.convertTo(grayFloat, CV_32F, 1.0 / 255.0);

    std::vector<Corner> raw = DetectCornersHarris(grayFloat);
    std::cout << "Raw Harris candidates: " << raw.size() << "\n";

    std::vector<Corner> filtered = SuppressNearbyDuplicates(raw, 15.0f);
    std::cout << "After NMS: " << filtered.size() << "\n";

    cv::Size boardSize(8, 8);
    bool ok = SortIntoGrid(filtered, boardSize);
    std::cout << "Grid sort success: " << ok << "\n";

    if (ok) {
        std::vector<cv::Point2f> points;
        for (const auto& c : filtered) points.push_back(cv::Point2f(c.x, c.y));

        cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01);
        cv::cornerSubPix(gray, points, cv::Size(11, 11), cv::Size(-1, -1), criteria);

        cv::Mat vis;
        cv::cvtColor(gray, vis, cv::COLOR_GRAY2BGR);
        cv::drawChessboardCorners(vis, boardSize, points, true);
        cv::imwrite("manual_corners.png", vis);
        std::cout << "Saved manual_corners.png\n";
    } else {
        cv::Mat vis;
        cv::cvtColor(gray, vis, cv::COLOR_GRAY2BGR);
        for (const auto& c : filtered) {
            cv::circle(vis, cv::Point(c.x, c.y), 4, cv::Scalar(0, 0, 255), 2);
        }
        cv::imwrite("manual_corners_unsorted.png", vis);
        std::cout << "Saved manual_corners_unsorted.png (grid sort failed, raw points shown)\n";
    }

    return 0;
}