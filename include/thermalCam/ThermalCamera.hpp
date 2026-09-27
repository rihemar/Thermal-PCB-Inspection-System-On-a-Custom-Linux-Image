
#ifndef THERMAL_CAMERA_HPP
#define THERMAL_CAMERA_HPP

#include <cstdint>
#include <opencv2/opencv.hpp>
#include "ITCamera.hpp"

#include "MLX90641_API.hpp"

struct ThermalFrame
{
    std::vector<float> temperatures;
    int width;
    int height;
    uint64_t timestamp;
};

class ThermalCamera : public ITCamera
{
private:

    static constexpr int WIDTH = 16;
    static constexpr int HEIGHT = 12;
    static constexpr int PIXELS = WIDTH * HEIGHT;

    uint8_t slave_addr = 0x033;
    uint8_t TA_SHIFT = 5;
    float emissivity = 0.95;

    uint16_t eeprom[832];
    uint16_t frame[834];

    paramsMLX90641 params;

    float temperatures[PIXELS];

    float tr;

    int width;
    int height;

public:

    explicit ThermalCamera(uint8_t slaveAddr = 0x33);

    bool initialize() override;

    bool captureFrame(cv::Mat& frame) override;

public:
    // ... existing declarations ...
    cv::Mat displayFrame(float minTemp = 0.0, float maxTemp = 50.0) override;

private:
    cv::Mat lastFrame;   // last captured CV_32FC1 temperature map, cached for displayFrame()
};

#endif
