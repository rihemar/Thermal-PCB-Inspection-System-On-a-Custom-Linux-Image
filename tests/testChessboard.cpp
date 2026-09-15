#include <opencv2/opencv.hpp>
#include <iostream>

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

    std::cout << "Size: " << img.size() << ", type: " << img.type() << ", channels: " << img.channels() << "\n";

    cv::Mat gray;
    if (img.type() == CV_32F || img.type() == CV_64F) {
        cv::normalize(img, gray, 0, 255, cv::NORM_MINMAX, CV_8U);
    } else if (img.channels() == 3) {
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = img;
    }

    cv::imwrite("debug_gray.png", gray);

    cv::Size boardSize(9, 6);

    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray, boardSize, corners);

    std::cout << "found: " << found << "\n";
    std::cout << "corners: " << corners.size() << "\n";

    cv::Mat vis;
    cv::cvtColor(gray, vis, cv::COLOR_GRAY2BGR);
    cv::drawChessboardCorners(vis, boardSize, corners, found);
    cv::imwrite("debug_corners.png", vis);

    return 0;
}