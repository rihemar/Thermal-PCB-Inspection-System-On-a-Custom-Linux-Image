#include <fakeThermalCamera.hpp>


// constructor simplifies to:
fakeThermalCamera::fakeThermalCamera()
    : gen_(rd_()), distrib_(25.0f, 10.0f)
{
    nbPixels_ = width_ * height_;
    // no thermalframe init needed anymore
}

bool fakeThermalCamera::initialize() {
        // No hardware to open for a simulated sensor.
        return true;
    }


uint64_t fakeThermalCamera::currentTimestampMs() {
        using namespace std::chrono;
        return static_cast<uint64_t>(
            duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count());
    }

// fakeThermalCamera.cpp
bool fakeThermalCamera::captureFrame(cv::Mat& frame) {
    const int rawWidth = 8, rawHeight = 6, rawPixels = rawWidth * rawHeight;

    std::vector<float> raw(rawPixels);
    for (auto& v : raw) v = distrib_(gen_);

    cv::Mat rawMat(rawHeight, rawWidth, CV_32F, raw.data());
    cv::resize(rawMat, frame, cv::Size(width_, height_), 0, 0, cv::INTER_LINEAR);

    if (!frame.isContinuous()) frame = frame.clone();

    lastFrame_ = frame;                 // just a Mat header copy (shared data), no pixel copy
    lastTimestamp_ = currentTimestampMs();

    return !frame.empty();
}


bool fakeThermalCamera::captureFrameSpecificHotspot(cv::Mat& frame, cv::Point hotspotPos,
                                                    float hotspotTemp,
                                                    float hotspotRadius)
{
    const int rawWidth = 8;
    const int rawHeight = 6;
    const int rawPixels = rawWidth * rawHeight;

    hotspotPos.x = std::clamp(hotspotPos.x, 0, rawWidth - 1);
    hotspotPos.y = std::clamp(hotspotPos.y, 0, rawHeight - 1);

    std::vector<float> raw(rawPixels);

    for (int y = 0; y < rawHeight; ++y) {
        for (int x = 0; x < rawWidth; ++x) {
            float ambient = distrib_(gen_);

            float dx = static_cast<float>(x - hotspotPos.x);
            float dy = static_cast<float>(y - hotspotPos.y);
            float distSq = dx * dx + dy * dy;
            float gaussian = std::exp(-distSq / (2.0f * hotspotRadius * hotspotRadius));

            raw[y * rawWidth + x] = ambient + (hotspotTemp - ambient) * gaussian;
        }
    }

    cv::Mat rawMat(rawHeight, rawWidth, CV_32F, raw.data());
    cv::resize(rawMat, frame, cv::Size(width_, height_), 0, 0, cv::INTER_LINEAR);

    if (!frame.isContinuous()) {
        frame = frame.clone();
    }

    lastFrame_ = frame;                       // <-- the fix: keep this in sync too
    lastTimestamp_ = currentTimestampMs();

    return !frame.empty();
}
    
fakeThermalCamera::~fakeThermalCamera(){}


cv::Point2f fakeThermalCamera::ScaleCoordinates(const cv::Point& originalPoint) const
{

    float scaleX = static_cast<float>(display_width) / static_cast<float>(width_);
    float scaleY = static_cast<float>(display_height) / static_cast<float>(height_);

    return cv::Point2f(originalPoint.x * scaleX, originalPoint.y * scaleY);
}

ThermalFrame fakeThermalCamera::toThermalFrame() const
        {
            ThermalFrame f;
            f.width = lastFrame_.cols;
            f.height = lastFrame_.rows;
            f.timestamp = lastTimestamp_;
            f.temperatures.assign(
                reinterpret_cast<float *>(lastFrame_.data),
                reinterpret_cast<float *>(lastFrame_.data) + (lastFrame_.cols * lastFrame_.rows));
            return f;
        }

// Helper function to create a 256-entry custom multi-color LUT
cv::Mat fakeThermalCamera::createCustomThermalLUT() {
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



// cv::Mat fakeThermalCamera::displayFrame(float minTemp, float maxTemp) {
//     if (lastFrame_.empty()) return cv::Mat();

//     cv::Mat resized, clamped, normalized, colorized;
//     cv::resize(lastFrame_, resized, cv::Size(display_width, display_height), 0, 0, cv::INTER_LINEAR);

//     cv::min(resized, maxTemp, clamped);
//     cv::max(clamped, minTemp, clamped);
//     clamped.convertTo(normalized, CV_8U, 255.0 / (maxTemp - minTemp),
//                       -minTemp * 255.0 / (maxTemp - minTemp));
    
//     static cv::Mat customLut = createCustomThermalLUT();
//     cv::LUT(normalized, customLut, colorized);
//     return colorized;
// }

cv::Mat fakeThermalCamera::displayFrame(float minTemp, float maxTemp)
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