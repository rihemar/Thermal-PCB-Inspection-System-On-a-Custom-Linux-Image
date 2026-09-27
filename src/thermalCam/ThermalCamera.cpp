
#include "ThermalCamera.hpp"

#include <opencv2/opencv.hpp>
#include <cstdint>
#include <cstring>

// MLX90641 driver
#include "MLX90641_API.hpp"

// ---------------------------------------------------------
// Constructor
// ---------------------------------------------------------

ThermalCamera::ThermalCamera(uint8_t slaveAddr)
    : slave_addr(slaveAddr)
{
    // MLX90641 native resolution:
    // 16 x 12 = 192 thermal pixels
    width = 16;
    height = 12;

    emissivity = 1.0f;
    tr = 0.0f;

    std::memset(eeprom, 0, sizeof(eeprom));
    std::memset(frame, 0, sizeof(frame));
    std::memset(temperatures, 0, sizeof(temperatures));
}

// ---------------------------------------------------------
// Initialize sensor
// ---------------------------------------------------------

bool ThermalCamera::initialize()
{
    // Read EEPROM calibration data
    int status = MLX90641_DumpEE(slave_addr, eeprom);

    if (status != 0)
    {
        return false;
    }

    // Extract calibration parameters
    status = MLX90641_ExtractParameters(eeprom, &params);

    if (status != 0)
    {
        return false;
    }

    return true;
}

// ---------------------------------------------------------
// Capture one thermal frame
// ---------------------------------------------------------

bool ThermalCamera::captureFrame(cv::Mat& frameOutput)
{
    // Get raw frame from MLX90641
    int status = MLX90641_GetFrameData(slave_addr, frame);

    if (status != 0)
    {
        return false;
    }

    // Calculate reflected temperature
    tr = MLX90641_GetTa(frame, &params) - TA_SHIFT;

    // Convert raw sensor data into actual temperatures
    MLX90641_CalculateTo(
        frame,
        &params,
        emissivity,
        tr,
        temperatures
    );

    // Create OpenCV matrix containing float temperatures
    frameOutput = cv::Mat(
        height,
        width,
        CV_32FC1
    );

    // Copy the 192 temperature values into OpenCV
    std::memcpy(
        frameOutput.data,
        temperatures,
        sizeof(float) * width * height
    );
    frameOutput.copyTo(lastFrame);   // add this line right before `return true;`


    return true;
}


// ---------------------------------------------------------
// Render the last captured frame as a false-color image
// ---------------------------------------------------------

cv::Mat ThermalCamera::displayFrame(float minTemp, float maxTemp)
{
    if (lastFrame.empty())
    {
        // Nothing captured yet — return a blank placeholder at native resolution
        return cv::Mat::zeros(height, width, CV_8UC3);
    }

    // Normalize the temperature map into 8-bit range using the given bounds
    cv::Mat clamped;
    cv::min(lastFrame, maxTemp, clamped);
    cv::max(clamped, minTemp, clamped);

    cv::Mat normalized;
    clamped.convertTo(normalized, CV_8UC1, 255.0 / (maxTemp - minTemp),
                       -minTemp * 255.0 / (maxTemp - minTemp));

    cv::Mat colorized;
    cv::applyColorMap(normalized, colorized, cv::COLORMAP_JET);

    // Upscale from the native 16x12 sensor grid so it's actually visible
    cv::Mat resized;
    cv::resize(colorized, resized, cv::Size(width * 20, height * 20), 0, 0, cv::INTER_LINEAR);

    return resized;
}