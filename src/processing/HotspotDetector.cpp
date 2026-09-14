#include <cmath>
#include "HotspotDetector.hpp"

Hotspot HotspotDetector::DetectAbsolute( const cv::Mat& frame) const{
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;

    cv::minMaxLoc(frame, &minVal, &maxVal, &minLoc, &maxLoc);

    return Hotspot{ maxLoc, static_cast<float>(maxVal) }; // pixel coordinates of the hottest point
}



std::vector<Hotspot>  HotspotDetector::DetectThreshold(const cv::Mat& frame,float threshold) const {
    std::vector<Hotspot> hotspots;
    if (frame.empty()) {
        return hotspots;
    }

    cv::Mat mask;
    cv::compare(frame, threshold, mask, cv::CMP_GT);

    std::vector<cv::Point> points;
    cv::findNonZero(mask, points);

    hotspots.reserve(points.size());
    for (const cv::Point& p : points) {
        float temp = frame.at<float>(p.y, p.x); // row=y, col=x
        hotspots.push_back(Hotspot{ p, temp });
    }

    return hotspots;
}


std::vector<Hotspot> HotspotDetector::DetectThreshold(const cv::Mat& frame) const {
   return DetectThreshold(frame,this->threshold);

}

