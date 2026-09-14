#include "PCBDetector.hpp"
#include <vector>
#include "utils.hpp"
PCBDetector::PCBDetector() {}
PCBDetector::~PCBDetector() {}

std::vector<cv::Point> PCBDetector::DetectPCB(const cv::Mat& rgbImage) const {
        cv::Mat gray = ToGrayscale(rgbImage);
        cv::Mat edges = DetectEdges(gray);
        std::vector<std::vector<cv::Point>> contours = FindContours(edges);
        return FindLargestQuadrilateral(contours);
    }


cv::Mat PCBDetector::ToGrayscale(const cv::Mat& rgbImage) const {
        cv::Mat gray;
        cv::cvtColor(rgbImage, gray, cv::COLOR_BGR2GRAY);
        // utils::show("gray",gray);
        return gray;
    }


 cv::Mat PCBDetector::DetectEdges(const cv::Mat& gray, double lowThresh, double highThresh) const {
        cv::Mat blurred, edges;

        // Blur first — Canny is sensitive to noise, and raw sensor/webcam
        // grain creates lots of tiny spurious edges without this.
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
        // utils::show("blur",blurred);
        cv::Canny(blurred, edges, lowThresh, highThresh);
        utils::show("canny",edges);
        return edges;
    }

      std::vector<cv::Point> PCBDetector::FindLargestQuadrilateral(
        const std::vector<std::vector<cv::Point>>& contours) const {
 
        if (contours.empty()) return {};
 
        // Gather every point from every external contour. Even if
        // dilation/closing left more than one blob, pooling all of them
        // guarantees the final shape still contains 100% of the edges.
        std::vector<cv::Point> allPoints;
        for (const auto& c : contours)
            allPoints.insert(allPoints.end(), c.begin(), c.end());
 
        if (allPoints.empty()) return {};
 
        // Convex hull of all edge points = smallest convex region that
        // still contains every single edge pixel. This directly encodes
        // "outside is no edges at all".
        std::vector<cv::Point> hull;
        cv::convexHull(allPoints, hull);
 
        // Reduce the hull down to 4 corners.
        double peri = cv::arcLength(hull, true);
        double epsilon = 0.02 * peri;
        std::vector<cv::Point> approx;
        cv::approxPolyDP(hull, approx, epsilon, true);
 
        // approxPolyDP doesn't always land exactly on 4 points on the
        // first try; nudge epsilon up/down until it does.
        int iterations = 0;
        while (approx.size() != 4 && iterations < 25) {
            epsilon *= (approx.size() > 4) ? 1.1 : 0.9;
            cv::approxPolyDP(hull, approx, epsilon, true);
            ++iterations;
        }
 
        if (approx.size() != 4) {
            // Fallback: minimum-area bounding rectangle of the hull.
            // It's still guaranteed to enclose every edge point, and is
            // always exactly 4 points.
            cv::RotatedRect rect = cv::minAreaRect(hull);
            cv::Point2f pts[4];
            rect.points(pts);
            approx.assign(pts, pts + 4);
        }
 
        return approx;
    }


    // std::vector<cv::Point> PCBDetector::FindLargestQuadrilateral(
    //     const std::vector<std::vector<cv::Point>>& contours) const
    // {
    //     std::vector<cv::Point> bestQuad;
    //     double bestArea = 0.0;

    //     for (const auto& contour : contours) {
    //         double perimeter = cv::arcLength(contour, true);

    //         std::vector<cv::Point> approx;
    //         // approxPolyDP simplifies a contour down to its dominant corners.
    //         // The epsilon (2% of perimeter here) controls how aggressively it
    //         // simplifies — too small and noisy contours won't reduce to 4
    //         // points; too large and real shapes get over-simplified.
    //         cv::approxPolyDP(contour, approx, 0.02 * perimeter, true);

    //         // A PCB, viewed from above (even at a slight angle), should
    //         // approximate a 4-sided convex shape.
    //         if (approx.size() == 4 && cv::isContourConvex(approx)) {
    //             double area = cv::contourArea(approx);
    //             if (area > bestArea) {
    //                 bestArea = area;
    //                 bestQuad = approx;
    //             }
    //         }
    //     }

    //     return bestQuad;
    // }


std::vector<std::vector<cv::Point>> PCBDetector::FindContours(const cv::Mat& edges) const {
        std::vector<std::vector<cv::Point>> contours;

        // RETR_EXTERNAL: we only want outer boundaries, not nested contours
        // inside the PCB (traces, components, silkscreen text, etc.).
        // CHAIN_APPROX_SIMPLE: compresses straight-line segments to just
        // their endpoints, which is exactly what we want before polygon fitting.
        cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        return contours;
    }