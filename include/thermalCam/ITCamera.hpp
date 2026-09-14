#pragma once
#include <opencv2/opencv.hpp>

class ITCamera {
public:
    virtual bool initialize() = 0;

    virtual bool captureFrame(cv::Mat& frame) = 0;

    virtual ~ITCamera() = default;

    virtual cv::Mat displayFrame(float minTemp=0.0, float maxTemp=50.0)=0;
};