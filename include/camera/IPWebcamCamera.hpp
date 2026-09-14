#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include "IRGBCamera.hpp"

class IPWebcamCamera : public IRGBCamera {
    private:
        std::string url = "http://192.168.100.158:8080/video";
        cv::VideoCapture cap_;
        int deviceId_;
        int width_ = 640;
        int height_= 480;

    public:
    
        cv::Mat frame;
        explicit IPWebcamCamera(std::string url = "http://192.168.100.158:8080/video", int width = 640, int height= 480);

        ~IPWebcamCamera();

        bool initialize() override;

        bool captureFrame(cv::Mat& frame) override;

};