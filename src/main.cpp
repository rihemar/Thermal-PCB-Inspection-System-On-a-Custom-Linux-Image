#include "ITCamera.hpp"
#include "IRGBCamera.hpp"
#include "WebcamCamera.hpp"
#include "IPWebcamCamera.hpp"
#include "fakeThermalCamera.hpp"
#include "HotspotDetector.hpp"
#include "PCBDetector.hpp"
#include "utils.hpp"
#include "Registration.hpp"
#include "JointBilateralUpsample.hpp"
#include "RecordedRGB.hpp"
#include "RecordedThermal.hpp"
#include "generateFakeHeatmapFromImage.hpp"
#include <thread>
#include <chrono>
#include <memory>
#include <iostream>

int test() {
    // --- recorded sources instead of live cameras, so the pipeline can
    // be tested deterministically without hardware attached ---
    auto camera = std::make_unique<RecordedRGB>();
    if (!camera->initialize()) {
        return -1; // no leak — camera cleans itself up automatically
    }

    auto thermalCamera = std::make_unique<RecordedThermal>();
    if (!thermalCamera->initialize()) {
        return -1;
    }

    // --- detectors ---
    PCBDetector pcbDetector;
    HotspotDetector hotspotDetector;

    // --- registration: load once before the loop, not per frame ---
    cv::Mat homography = Registration::LoadHomography("rgb_to_thermal_homography.yml");
    bool useHomography = !homography.empty();
    // TODO: measure this ROI (in RGB pixel coords) for your actual rig
    // if you haven't run homography calibration yet.
    cv::Rect fallbackRoi(80, 40, 480, 360);

    cv::Mat rgbFrame, thermalFrame;
    std::vector<cv::Point> pcbContour;

    while (camera->captureFrame(rgbFrame) && thermalCamera->captureFrame(thermalFrame)) {

        // --- PCB detection on the recorded RGB stream ---
        pcbContour = pcbDetector.DetectPCB(rgbFrame);
        if (!pcbContour.empty()) {
            cv::polylines(rgbFrame, pcbContour, true, cv::Scalar(0, 255, 0), 2);
        }
        utils::show("PCB Detection", rgbFrame);

        // --- hotspot detection on the recorded thermal stream ---
        cv::Mat thermalDisplay = thermalCamera->displayFrame();
        std::vector<Hotspot> hotspots = hotspotDetector.DetectThreshold(thermalFrame, 55);
        std::cout << hotspots.size() << " points above threshold:" << std::endl;

        for (const Hotspot& h : hotspots) {
            cv::Point2f pf = thermalCamera->ScaleCoordinates(h.position);
            cv::Point displayPt(cvRound(pf.x), cvRound(pf.y));
            cv::circle(thermalDisplay, displayPt, 4, cv::Scalar(255, 255, 255), -1);
            cv::putText(thermalDisplay, std::to_string(h.temperature),
                        displayPt + cv::Point(10, 0), cv::FONT_HERSHEY_SIMPLEX,
                        0.5, cv::Scalar(255, 255, 255), 1);
        }
        cv::imshow("Thermal (raw)", thermalDisplay);

        // --- RGB-guided thermal fusion / upsampling ---
        cv::Size targetSize = rgbFrame.size(); // enhance thermal up to RGB resolution
        cv::Mat alignedGuide = useHomography
            ? Registration::AlignToThermal(rgbFrame, homography, targetSize)
            : Registration::AlignByCropScale(rgbFrame, fallbackRoi, targetSize);

        cv::Mat thermalFloat;
        if (thermalFrame.type() != CV_32F) {
            thermalFrame.convertTo(thermalFloat, CV_32F);
        } else {
            thermalFloat = thermalFrame;
        }

        cv::Mat enhanced = JointBilateralUpsample::Upsample(
            thermalFloat, alignedGuide,
            GuideMode::SOBEL,      // try GuideMode::RAW or GuideMode::CANNY too
            /*windowSize=*/5,
            /*sigmaSpectral=*/0.15f,
            /*sigmaSpatial=*/2.0);

        double minV, maxV;
        cv::minMaxLoc(enhanced, &minV, &maxV);
        cv::Mat enhancedDisplay = JointBilateralUpsample::Colorize(enhanced, minV, maxV);
        cv::imshow("Thermal (RGB-guided enhanced)", enhancedDisplay);

        if (cv::waitKey(1) == 27) break;

        // Recorded thermal playback likely runs at a lower/fixed rate
        // than the RGB recording; keep pacing so the fusion step isn't
        // hammered faster than a real thermal sensor would produce frames.
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}

int main(){
    cv::Mat frame = cv::imread("./pictures/raspberrypi.jpg");
    generateFakeHeatmapFromImage generatorr;
    cv::Mat heatmapgen = generatorr.getHeatmap(frame);
    std::cout << "Heatmap size: " << heatmapgen.cols << "x" << heatmapgen.rows
              << ", type: " << heatmapgen.type() << std::endl;
 
    double minT, maxT;
    cv::minMaxLoc(heatmapgen, &minT, &maxT);
    std::cout << "Min temp: " << minT << " C, Max temp: " << maxT << " C" << std::endl;
 
    cv::Mat colorized = generatorr.getColorizedHeatmap(heatmapgen, cv::Size(320, 240));
 
    cv::imwrite("fake_thermal_heatmap.png", colorized);
    // std::cout << "Wrote fake_thermal_heatmap.png" << std::endl;
    cv::Mat enhanced = JointBilateralUpsample::Upsample(
            heatmapgen, frame,
            GuideMode::RAW,      // try GuideMode::RAW or GuideMode::CANNY too
            /*windowSize=*/7, // original 5
            /*sigmaSpectral=*/0.05f, //original 0.15
            /*sigmaSpatial=*/8.0); // original 2.0

        double minV, maxV;
        cv::minMaxLoc(enhanced, &minV, &maxV);
        // minV = minV - 20;
        cv::Mat enhancedDisplay = JointBilateralUpsample::Colorize(enhanced, minV, maxV);
        cv::Mat withEdges = JointBilateralUpsample::OverlayEdges(enhancedDisplay, frame, 0.2);
        cv::imwrite("Thermal (RGB-guided enhanced).png", withEdges);


        // Recorded thermal playback likely runs at a lower/fixed rate
        // than the RGB recording; keep pacing so the fusion step isn't
        // hammered faster than a real thermal sensor would produce frames.    }
        return 1;
 
}