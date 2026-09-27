#pragma once
#include <vector>
#include <cstdint>
#include <chrono>
#include <random>
#include <opencv2/opencv.hpp>
#include "ITCamera.hpp"
#include "ThermalCamera.hpp"


class fakeThermalCamera : public ITCamera {
    private:
        static uint64_t currentTimestampMs();
        std::random_device rd_;
        std::mt19937 gen_;
        std::normal_distribution<float> distrib_;
        ThermalFrame thermalframe;
        int width_ = 32;
        int height_ = 24;
        int nbPixels_ = 0;

        int display_width = 750;
        int display_height = 600;
        cv::Mat lastFrame_;         // canonical storage now — CV_32F, width_ x height_
        uint64_t lastTimestamp_ = 0;

    public:
        fakeThermalCamera();
        bool initialize() override;
        bool captureFrame(cv::Mat &frame) override;
        bool captureFrameSpecificHotspot(cv::Mat &frame, cv::Point hotspotPos,
                                         float hotspotTemp = 40.0f,
                                         float hotspotRadius = 3.0f);
        cv::Mat displayFrame(float minTemp = 0.0f, float maxTemp = 50.0f) override;
        cv::Mat createCustomThermalLUT();

        // Convert on demand — only pays the copy cost when someone actually needs
        // the vector form (e.g. HotspotDetector). Nothing calls this implicitly.
        ThermalFrame toThermalFrame() const;

        cv::Point2f ScaleCoordinates(const cv::Point& originalPoint) const;

        ~fakeThermalCamera();
  
};




// class fakeThermalCamera : public ITCamera {


// public:

//     fakeThermalCamera();
//     bool initialize();
//     bool captureFrame(cv::Mat& frame) override;
//     bool captureFrameSpecificHotspot(cv::Mat& frame , cv::Point hotspotPos,
//                                                 float hotspotTemp = 40.0f,
//                                                 float hotspotRadius = 3.0f);
//     ~fakeThermalCamera() override = default;
//     cv::Mat displayFrame (float minTemp = 0.0f,float maxTemp = 50.0f) override;
// };