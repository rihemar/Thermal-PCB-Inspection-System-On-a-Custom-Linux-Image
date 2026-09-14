#include <opencv2/opencv.hpp>
class utils{
    private:
    const static int normal_display_width = 640;
    const static int normal_display_height = 480;
    public:
    static bool show(std::string title,cv::Mat frame){
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size(normal_display_width, normal_display_height), 0, 0, cv::INTER_LINEAR);
        cv::imshow(title,resized);
        return 1;
    }
};