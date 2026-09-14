#include "fakeThermalCamera.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>


struct Hotspot {
    cv::Point position;
    float temperature;
};


class HotspotDetector {
private:
    int threshold = 35;
public:
    HotspotDetector() {}

    Hotspot DetectAbsolute( const cv::Mat& frame) const;
    std::vector<Hotspot> DetectThreshold(const cv::Mat& frame,float threshold) const;
    std::vector<Hotspot> DetectThreshold(const cv::Mat& frame) const;

};