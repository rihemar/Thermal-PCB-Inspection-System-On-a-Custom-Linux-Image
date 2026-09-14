#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include "IRGBCamera.hpp"

class RecordedRGB : public IRGBCamera {
    private:
        cv::VideoCapture cap_;
        int width_;
        int height_;
        
    public:
    
        cv::Mat frame;

        RecordedRGB(int width = 640, int height = 480);

        bool initialize() override {
            return true;
        };

        ~RecordedRGB();

        bool captureFrame(cv::Mat& frame) override;

};