#include <cmath>
#include "HotspotDetector.hpp"
#include "JointBilateralUpsample.hpp"

Hotspot HotspotDetector::DetectAbsolute( const cv::Mat& frame){
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;

    cv::minMaxLoc(frame, &minVal, &maxVal, &minLoc, &maxLoc);

    return Hotspot{ maxLoc, static_cast<float>(maxVal) }; // pixel coordinates of the hottest point
}

HotRegion HotspotDetector::DetectThreshold(const cv::Mat& frame, float threshold) {
    if (frame.empty()) {
        return HotRegion{ cv::Rect(), cv::Point(-1, -1), 0.0f, 0.0f };
    }
 
    cv::Mat mask;
    cv::compare(frame, threshold, mask, cv::CMP_GT);
 
    cv::Rect boundingBox = cv::boundingRect(mask);
    if (boundingBox.area() == 0) {
        return HotRegion{ cv::Rect(), cv::Point(-1, -1), 0.0f, 0.0f };
    }
 
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(frame, &minVal, &maxVal, &minLoc, &maxLoc, mask);
 
    float borderTemp = static_cast<float>(maxVal);
    int sampleRow = boundingBox.y + boundingBox.height / 2;
    if (boundingBox.x > 0) {
        borderTemp = frame.at<float>(sampleRow, boundingBox.x - 1);
    } else if (boundingBox.x + boundingBox.width < frame.cols) {
        borderTemp = frame.at<float>(sampleRow, boundingBox.x + boundingBox.width);
    }
 
    return HotRegion{ boundingBox, maxLoc, static_cast<float>(maxVal), borderTemp };
}

// ---------------------------------------------------------------------------
// The single hottest *region*: grow out from the global peak, stop where
// the temperature drops off sharply (i.e. the edges are "significantly
// colder"), and return the bounding box of what's left.
// ---------------------------------------------------------------------------
HotRegion HotspotDetector::DetectRegion(const cv::Mat& frame, float coldnessMargin)  {
    CV_Assert(!frame.empty());
    CV_Assert(frame.type() == CV_32FC1);
 
    // 1. Seed the region at the single hottest pixel.
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(frame, &minVal, &maxVal, &minLoc, &maxLoc);
    cv::imshow("",JointBilateralUpsample::Colorize(frame,minVal,maxVal));

 
    // 2. Flood-fill outward from the seed. loDiff/upDiff = coldnessMargin
    //    means a neighboring pixel only joins the region if it's within
    //    `coldnessMargin` degrees of the pixel it's growing from — so the
    //    fill naturally halts right at a sharp edge.
    cv::Mat filled = frame.clone(); // required non-const, untouched due to MASK_ONLY
    cv::Mat mask = cv::Mat::zeros(frame.rows + 2, frame.cols + 2, CV_8UC1);
 
    cv::Rect boundingBox;
    const int maskFillValue = 255;
    cv::floodFill(
        filled,
        mask,
        maxLoc,
        cv::Scalar(0),
        &boundingBox,
        cv::Scalar(coldnessMargin),
        cv::Scalar(coldnessMargin),
        4 | cv::FLOODFILL_MASK_ONLY | (maskFillValue << 8)
    );
 
    // 3. Sample just outside the box (if possible) to confirm the drop-off.
    float borderTemp = static_cast<float>(maxVal);
    int sampleRow = boundingBox.y + boundingBox.height / 2;
    if (boundingBox.x > 0) {
        borderTemp = frame.at<float>(sampleRow, boundingBox.x - 1);
    } else if (boundingBox.x + boundingBox.width < frame.cols) {
        borderTemp = frame.at<float>(sampleRow, boundingBox.x + boundingBox.width);
    }
 
    return HotRegion{ boundingBox, maxLoc, static_cast<float>(maxVal), borderTemp };
}
 
HotRegion HotspotDetector::DetectHottestRegionBox(const cv::Mat& frame, double noiseSuppressionSigma) {
    CV_Assert(!frame.empty());
    CV_Assert(frame.type() == CV_32FC1);
 
    // Suppress sensor noise so we don't get one "peak" per noisy pixel.
    cv::Mat smoothed;
    cv::GaussianBlur(frame, smoothed, cv::Size(0, 0), noiseSuppressionSigma);
 
    // Every local maximum (a pixel >= all its 3x3 neighbors) seeds its own
    // basin. connectedComponents labels 0 = "unknown", 1..N = distinct peaks
    // -- which is exactly the marker format cv::watershed expects.
    cv::Mat dilated;
    cv::dilate(smoothed, dilated, cv::Mat());
    cv::Mat localMaxMask = (smoothed >= dilated);
 
    cv::Mat markers;
    int numPeaks = cv::connectedComponents(localMaxMask, markers, 8, CV_32S);
 
    // Seed a background basin from the coldest ~10% of the frame, so a
    // lone hotspot is still bounded by something even with no other peaks.
    cv::Mat flat = smoothed.reshape(1, 1).clone();
    cv::Mat sortedFlat;
    cv::sort(flat, sortedFlat, cv::SORT_ASCENDING);
    float backgroundLevel = sortedFlat.at<float>(static_cast<int>(0.1 * sortedFlat.total()));
    int backgroundLabel = numPeaks;
    markers.setTo(backgroundLabel, smoothed <= backgroundLevel);
 
    // Watershed floods every basin downhill until they meet -- each
    // basin's edge is exactly where the terrain favors a neighboring
    // basin (or the background) instead. No manual threshold involved.
    double minVal, maxVal;
    cv::minMaxLoc(smoothed, &minVal, &maxVal);
    cv::Mat inverted = maxVal - smoothed;
    cv::Mat inverted8U, colorImage;
    inverted.convertTo(inverted8U, CV_8U, 255.0 / (maxVal - minVal));
    cv::cvtColor(inverted8U, colorImage, cv::COLOR_GRAY2BGR);
    cv::watershed(colorImage, markers);
 
    cv::Point maxLoc;
    cv::minMaxLoc(frame, nullptr, &maxVal, nullptr, &maxLoc);
    int peakLabel = markers.at<int>(maxLoc);
 
    cv::Rect boundingBox = cv::boundingRect(markers == peakLabel);
 
    float borderTemp = static_cast<float>(maxVal);
    int sampleRow = boundingBox.y + boundingBox.height / 2;
    if (boundingBox.x > 0) {
        borderTemp = frame.at<float>(sampleRow, boundingBox.x - 1);
    } else if (boundingBox.x + boundingBox.width < frame.cols) {
        borderTemp = frame.at<float>(sampleRow, boundingBox.x + boundingBox.width);
    }
 
    return HotRegion{ boundingBox, maxLoc, static_cast<float>(maxVal), borderTemp };
}
 