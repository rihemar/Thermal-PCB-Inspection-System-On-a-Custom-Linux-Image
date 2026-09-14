#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include "IRGBCamera.hpp"

class WebcamCamera : public IRGBCamera {
    private:
        cv::VideoCapture cap_;
        int deviceId_;
        int width_;
        int height_;
    public:
    
        cv::Mat frame;
        explicit WebcamCamera(int deviceId, int width, int height);

        WebcamCamera();

        ~WebcamCamera();

        bool initialize() override;

        bool captureFrame(cv::Mat& frame) override;

};