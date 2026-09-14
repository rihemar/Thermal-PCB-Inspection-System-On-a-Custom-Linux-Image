#include "RecordedThermal.hpp"

// RecordedThermal.cpp
double RecordedThermal::ComputeAdaptiveThreshold(const cv::Mat& frame, float aboveAverageFraction)
{
    cv::Mat gray;
    if (frame.channels() > 1) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame;
    }

    double meanVal = cv::mean(gray)[0];
    // e.g. aboveAverageFraction = 0.25 -> threshold sits 25% above the mean,
    // so only pixels notably brighter than "typical" for this frame count
    // as hot, regardless of the image's absolute brightness scale.
    return meanVal * (1.0 + aboveAverageFraction);
}

// bool RecordedThermal::captureFrame(cv::Mat& frame)
// {
//     const std::string path = "./pictures/thermal_raspberrypi12.jpg";

//     cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);

//     if (image.empty())
//     {
//         std::cerr << "ERROR: Could not open thermal image: "
//                   << path << std::endl;
//         return false;
//     }

//     // Resize to the camera's expected resolution
//     cv::resize(
//         image,
//         frame,
//         cv::Size(width_, height_),
//         0,
//         0,
//         cv::INTER_LINEAR
//     );

//     // Keep last frame synchronized
//     lastFrame_ = frame;
//     return !frame.empty();
// }

    bool RecordedThermal::initialize() {
        return 1;
    }

    // Drop-in replacement for RecordedThermal::captureFrame — the LUT is
// built and used entirely inside this function, no separate
// createCustomThermalLUT() / DecodeTemperatureFromLUT() calls needed.

bool RecordedThermal::captureFrame(cv::Mat& frame)
{
    const std::string path = "./pictures/thermal_raspberrypi12.jpg";

    // Load in COLOR: this is a false-color image, and the palette IS the
    // temperature encoding. Converting to grayscale first destroys it.
    cv::Mat colorImage = cv::imread(path, cv::IMREAD_COLOR);

    if (colorImage.empty())
    {
        std::cerr << "ERROR: Could not open thermal image: "
                  << path << std::endl;
        return false;
    }

    // --- LUT defined right here, matching createCustomThermalLUT's
    // checkpoints, built once (static) and reused across calls ---
    static const cv::Mat lut = [] {
        cv::Mat table(256, 1, CV_8UC3);

        std::vector<std::pair<int, cv::Vec3b>> colorMap = {
            {  0, cv::Vec3b(203, 192, 255)}, // Dark Purple (coldest)
            { 32, cv::Vec3b(255,  55, 174)}, // Pink / Light Pink
            { 64, cv::Vec3b(255,   0,   0)}, // Blue
            { 96, cv::Vec3b(255, 200,   0)}, // Cyan
            {128, cv::Vec3b(  0, 255,   0)}, // Green
            {160, cv::Vec3b(  0, 200, 255)}, // Yellow
            {192, cv::Vec3b(  0, 128, 255)}, // Orange
            {224, cv::Vec3b(  0,   0, 255)}, // Bright Red
            {255, cv::Vec3b(  0,   0,   0)}, // Black (hottest)
        };

        for (size_t i = 0; i < colorMap.size() - 1; ++i)
        {
            int idxStart = colorMap[i].first;
            int idxEnd   = colorMap[i + 1].first;
            cv::Vec3b colorStart = colorMap[i].second;
            cv::Vec3b colorEnd   = colorMap[i + 1].second;
            int range = idxEnd - idxStart;

            for (int j = idxStart; j <= idxEnd; ++j)
            {
                float t = static_cast<float>(j - idxStart) / range;
                uchar b = static_cast<uchar>((1.0f - t) * colorStart[0] + t * colorEnd[0]);
                uchar g = static_cast<uchar>((1.0f - t) * colorStart[1] + t * colorEnd[1]);
                uchar r = static_cast<uchar>((1.0f - t) * colorStart[2] + t * colorEnd[2]);
                table.at<cv::Vec3b>(j, 0) = cv::Vec3b(b, g, r);
            }
        }
        return table;
    }();

    // --- decode: nearest-palette-match per pixel, straight to a float
    // temperature Mat, no intermediate grayscale step ---
    const float minTemp = 10; // add these members to the class if
    const float maxTemp = 50; // not already present (see note below)

    cv::Mat temperature(colorImage.size(), CV_32F);
    for (int y = 0; y < colorImage.rows; ++y)
    {
        for (int x = 0; x < colorImage.cols; ++x)
        {
            cv::Vec3b pixel = colorImage.at<cv::Vec3b>(y, x);

            int bestIndex = 0;
            int bestDist = INT_MAX;
            for (int i = 0; i < 256; ++i)
            {
                cv::Vec3b lutColor = lut.at<cv::Vec3b>(i, 0);
                int db = static_cast<int>(pixel[0]) - lutColor[0];
                int dg = static_cast<int>(pixel[1]) - lutColor[1];
                int dr = static_cast<int>(pixel[2]) - lutColor[2];
                int dist = db * db + dg * dg + dr * dr;

                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestIndex = i;
                    if (dist == 0) break; // exact match, stop early
                }
            }

            float t = static_cast<float>(bestIndex) / 255.0f;
            temperature.at<float>(y, x) = minTemp + t * (maxTemp - minTemp);
        }
    }

    // Resize AFTER decoding, not before: resizing the color JPEG first
    // would bilinearly blend palette colors into values that don't match
    // any palette entry, corrupting the decode. Resizing the decoded
    // float temperatures afterward is valid since temperature is
    // continuous.
    cv::resize(
        temperature,
        frame,
        cv::Size(width_, height_),
        0,
        0,
        cv::INTER_LINEAR
    );

    // Keep last frame synchronized
    lastFrame_ = frame;
    return !frame.empty();
}

cv::Point2f RecordedThermal::ScaleCoordinates(const cv::Point& originalPoint) const
{

    float scaleX = static_cast<float>(display_width) / static_cast<float>(width_);
    float scaleY = static_cast<float>(display_height) / static_cast<float>(height_);

    return cv::Point2f(originalPoint.x * scaleX, originalPoint.y * scaleY);
}


cv::Mat RecordedThermal::displayFrame(float minTemp, float maxTemp)
{
    if (lastFrame_.empty())
        return cv::Mat();

    cv::Mat resized, clamped, normalized, colorized;

    cv::resize(
        lastFrame_,
        resized,
        cv::Size(display_width, display_height),
        0, 0,
        cv::INTER_LINEAR
    );

    cv::min(resized, maxTemp, clamped);
    cv::max(clamped, minTemp, clamped);

    clamped.convertTo(
        normalized,
        CV_8U,
        255.0 / (maxTemp - minTemp),
        -minTemp * 255.0 / (maxTemp - minTemp)
    );

    static cv::Mat customLut = createCustomThermalLUT();

    colorized.create(normalized.size(), CV_8UC3);

    for (int y = 0; y < normalized.rows; ++y)
    {
        for (int x = 0; x < normalized.cols; ++x)
        {
            uchar index = normalized.at<uchar>(y, x);

            colorized.at<cv::Vec3b>(y, x) =
                customLut.at<cv::Vec3b>(index, 0);
        }
    }

    return colorized;
}

RecordedThermal::RecordedThermal()
{
}

RecordedThermal::~RecordedThermal()
{
}

// Helper function to create a 256-entry custom multi-color LUT
cv::Mat RecordedThermal::createCustomThermalLUT() {
    // 1. Create a 256x1 matrix of type CV_8UC3 (256 rows, 1 col, 3 channels)
    cv::Mat lut(256, 1, CV_8UC3);

    // 2. Define key color checkpoints (BGR format: Blue, Green, Red)
    std::vector<std::pair<int, cv::Vec3b>> colorMap = {
        {  0, cv::Vec3b(203, 192, 255)}, // Index 0   : Dark Purple (Coldest)
        { 32, cv::Vec3b(255, 55, 174)}, // Index 32  : Pink / Light Pink
        { 64, cv::Vec3b(255,   0,   0)}, // Index 64  : Blue
        { 96, cv::Vec3b(255, 200,   0)}, // Index 96  : Cyan
        {128, cv::Vec3b(  0, 255,   0)}, // Index 128 : Green
        {160, cv::Vec3b(  0, 200, 255)}, // Index 160 : Yellow
        {192, cv::Vec3b(  0, 128, 255)}, // Index 192 : Orange
        {224, cv::Vec3b(  0,   0, 255)}, // Index 224 : Bright Red
        {255, cv::Vec3b(0, 0, 0)}  // Index 255 : White (Hottest)
    };

    // 3. Interpolate linearly between color checkpoints
    for (size_t i = 0; i < colorMap.size() - 1; ++i) {
        int idxStart = colorMap[i].first;
        int idxEnd   = colorMap[i+1].first;

        cv::Vec3b colorStart = colorMap[i].second;
        cv::Vec3b colorEnd   = colorMap[i+1].second;

        int range = idxEnd - idxStart;
        for (int j = idxStart; j <= idxEnd; ++j) {
            float t = static_cast<float>(j - idxStart) / range;
            
            uchar b = static_cast<uchar>((1.0f - t) * colorStart[0] + t * colorEnd[0]);
            uchar g = static_cast<uchar>((1.0f - t) * colorStart[1] + t * colorEnd[1]);
            uchar r = static_cast<uchar>((1.0f - t) * colorStart[2] + t * colorEnd[2]);

            // Access row 'j', col 0
            lut.at<cv::Vec3b>(j, 0) = cv::Vec3b(b, g, r);
        }
    }

    return lut;
}