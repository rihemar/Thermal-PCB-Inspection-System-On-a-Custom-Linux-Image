#pragma once
#include <vector>
#include <cstdint>
#include <chrono>
#include <random>
#include <opencv2/opencv.hpp>
#include "ITCamera.hpp"
#include "fakeThermalCamera.hpp"


class RecordedThermal : public ITCamera {
    private:
        ThermalFrame thermalframe;
        int width_ = 32;
        int height_ = 24;
        int nbPixels_ = 0;

        int display_width = 750;
        int display_height = 600;
        cv::Mat lastFrame_;         // canonical storage now — CV_32F, width_ x height_
        uint64_t lastTimestamp_ = 0;

    public:
        RecordedThermal();
        bool initialize() override;
        bool captureFrame(cv::Mat &frame) override;

        ~RecordedThermal();
        cv::Point2f ScaleCoordinates(const cv::Point& originalPoint) const;
    cv::Mat displayFrame(float minTemp=0.0f, float maxTemp=50.0f);
    cv::Mat createCustomThermalLUT();
static double ComputeAdaptiveThreshold(const cv::Mat& frame, float aboveAverageFraction = 0.25f);

};
