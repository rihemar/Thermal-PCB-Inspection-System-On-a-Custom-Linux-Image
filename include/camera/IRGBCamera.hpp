#pragma once
#include <opencv2/opencv.hpp>

class IRGBCamera {
public:
    virtual bool initialize()=0;

    virtual bool captureFrame(cv::Mat& frame) = 0;

    virtual ~IRGBCamera() = default;

    // virtual cv::Mat displayFrame();
};