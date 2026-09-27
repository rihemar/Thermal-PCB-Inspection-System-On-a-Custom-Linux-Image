#include "fakeThermalCamera.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>


struct Hotspot {
    cv::Point position;
    float temperature;
};

struct HotRegion {
    cv::Rect boundingBox;
    cv::Point peakPosition;
    float peakTemperature;
    float borderTemperature; // sample just outside the box, for sanity-checking the drop-off
};

class HotspotDetector {
private:
    static const int threshold = 35;
    static const int coldnessMargin = 4;
    static const int noiseSupressionSigma = 4;
public:
    HotspotDetector() {}

    static Hotspot DetectAbsolute( const cv::Mat& frame);
    static HotRegion DetectThreshold (const cv::Mat& frame,float threshold=HotspotDetector::threshold);
    static HotRegion DetectRegion(const cv::Mat& frame, float coldnessMargin=HotspotDetector::coldnessMargin) ;
    static HotRegion DetectHottestRegionBox(const cv::Mat& frame, double noiseSuppressionSigma= HotspotDetector::noiseSupressionSigma);

};