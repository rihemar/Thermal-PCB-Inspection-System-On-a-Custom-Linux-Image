#include "WebcamCamera.hpp"


WebcamCamera::WebcamCamera(int deviceId, int width, int height)
            : deviceId_(deviceId), width_(width), height_(height) {}

WebcamCamera::WebcamCamera(){
            deviceId_ = 0 ;
            width_ = 640;
            height_= 480;
            while(initialize()==false){
                deviceId_++;
            }
            if (cap_.isOpened()) {
                cap_.release();
            }

        }

WebcamCamera::~WebcamCamera() {
            if (cap_.isOpened()) {
                cap_.release();
            }
        }


        bool WebcamCamera::initialize() {
            // On Windows you might prefer cv::CAP_DSHOW, on Linux cv::CAP_V4L2
            cap_.open(deviceId_, cv::CAP_ANY);

            if (!cap_.isOpened()) {
                std::cerr << "WebcamCamera: failed to open device " << deviceId_ << std::endl;
                return false;
            }

            cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);

            return true;
        }

   bool WebcamCamera::captureFrame(cv::Mat& frame) {
            if (!cap_.isOpened()) {
                return false;
            }
            cap_ >> frame; // read() also works and is a bit clearer
            this->frame = frame;
            return !frame.empty();
        }