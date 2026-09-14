#include <opencv2/opencv.hpp>
#include <vector>

class PCBDetector {
public:

    PCBDetector(); // declare it here
    ~PCBDetector();
    // Runs the full pipeline and returns the 4 corner points of the largest
    // quadrilateral found, or an empty vector if none was found.
    std::vector<cv::Point> DetectPCB(const cv::Mat& rgbImage) const ;


    cv::Mat ToGrayscale(const cv::Mat& rgbImage) const;

    cv::Mat DetectEdges(const cv::Mat& gray, double lowThresh = 50.0, double highThresh = 100.0) const ;

    std::vector<std::vector<cv::Point>> FindContours(const cv::Mat& edges) const;

    std::vector<cv::Point> FindLargestQuadrilateral(
        const std::vector<std::vector<cv::Point>>& contours) const;
};