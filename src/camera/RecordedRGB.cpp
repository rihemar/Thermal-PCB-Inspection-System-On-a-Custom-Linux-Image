#include "RecordedRGB.hpp"

        RecordedRGB::RecordedRGB(int width, int height)
        : width_(width), height_(height)
        {}

RecordedRGB::~RecordedRGB() {}

   bool RecordedRGB::captureFrame(cv::Mat& frame) {
            frame = cv::imread("./pictures/raspberrypi.jpg");
            return !frame.empty();
        }