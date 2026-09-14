#include "IPWebcamCamera.hpp"


IPWebcamCamera::IPWebcamCamera(std::string url, int width, int height)
            : url(url), width_(width), height_(height) {}


IPWebcamCamera::~IPWebcamCamera() {
            if (cap_.isOpened()) {
                cap_.release();
            }
        }


        bool IPWebcamCamera::initialize() {
            // On Windows you might prefer cv::CAP_DSHOW, on Linux cv::CAP_V4L2
            cap_.open(url, cv::CAP_FFMPEG);

            if (!cap_.isOpened()) {
                std::cerr << "IPWebcamCamera: failed to open device " << deviceId_ << std::endl;
                return false;
            }

            cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);

            return true;
        }

   bool IPWebcamCamera::captureFrame(cv::Mat& frame) {
            if (!cap_.isOpened()) {
                return false;
            }
            cap_ >> frame; // read() also works and is a bit clearer
            this->frame = frame;
            return !frame.empty();
        }