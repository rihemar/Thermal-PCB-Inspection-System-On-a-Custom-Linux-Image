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
#include "ChessboardCalibration.hpp"
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

int BilateralJointUpsampletest(){
    auto camera = std::make_unique<RecordedRGB>();
    if (!camera->initialize()) {
        return -1; // no leak — camera cleans itself up automatically
    }
    cv::Mat frame;

    while(camera->captureFrame(frame) ){
    // cv::Mat frame = cv::imread("./pictures/raspberrypi.jpg");
    generateFakeHeatmapFromImage generatorr(/* rgbHFovDeg =*/ 55.0,
                              /*thermalHFovDeg =*/ 55.0,
                              /*thermalRes =*/ cv::Size(16, 12),
                              /*ambientTempC =*/ 25.0f,
                              /*maxHotspotTempC =*/ 85.0f,
                              /*noiseStdDevC = */0.6);
    cv::Mat heatmapgen = generatorr.getHeatmap(frame);
    std::cout << "Heatmap size: " << heatmapgen.cols << "x" << heatmapgen.rows
              << ", type: " << heatmapgen.type() << std::endl;
 
    double minT, maxT;
    cv::minMaxLoc(heatmapgen, &minT, &maxT);
    std::cout << "Min temp: " << minT << " C, Max temp: " << maxT << " C" << std::endl;
 
    cv::Mat colorized = generatorr.getColorizedHeatmap(heatmapgen, cv::Size(320, 240));
 
    cv::imwrite("fake_thermal_heatmap.png", colorized);
    cv::imshow("fake_thermal_heatmap.png", colorized);
    // std::cout << "Wrote fake_thermal_heatmap.png" << std::endl;
    cv::Mat enhanced = JointBilateralUpsample::Upsample3D(
            heatmapgen, frame,
            GuideMode::RAW,      // try GuideMode::RAW or GuideMode::CANNY too
            /*windowSize=*/6, // original 5
            /*sigmaSpectral=*/0.05f, //original 0.15
            /*sigmaSpatial=*/8.0); // original 2.0

        double minV, maxV;
        cv::minMaxLoc(enhanced, &minV, &maxV);
        // cv::Mat thermalEnhanced = JointBilateralUpsample::EnhanceGradientContrast(enhanced, /*gain=*/0.1f);

        // minV = minV - 20;
        cv::Mat enhancedDisplay = JointBilateralUpsample::Colorize(enhanced, minV, maxV);
        cv::Mat withEdges = JointBilateralUpsample::OverlayEdges(enhancedDisplay, frame, 0.05);
        cv::imshow("Thermal (RGB-guided enhanced).png", withEdges);
        cv::imwrite("Thermal (RGB-guided enhanced).png", withEdges);
        if (cv::waitKey(1) == 27) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    }
        // Recorded thermal playback likely runs at a lower/fixed rate
        // than the RGB recording; keep pacing so the fusion step isn't
        // hammered faster than a real thermal sensor would produce frames.    }
        return 1;
 
}


int chessboard(){
    std::string chess1 = "./pictures/chess7.png";
    std::string chess2 = "./pictures/chess8.jpg";
    cv::Mat rgbImage = cv::imread(chess1, cv::IMREAD_COLOR);
    cv::Mat thermalImage = cv::imread(chess2, cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR);

    if (rgbImage.empty() || thermalImage.empty()) {
        std::cerr << "Failed to load images.\n";
        return 1;
    }

    cv::Size boardSize(10, 7);
    ChessboardCalibration calib(boardSize);

    auto pair = calib.DetectPair(rgbImage, thermalImage);
    if (!pair.success) {
        std::cerr << "Chessboard detection failed.\n";
        return 1;
    }

    cv::Mat H = ChessboardCalibration::ComputeHomography(pair);
    if (H.empty()) {
        std::cerr << "Homography computation failed.\n";
        return 1;
    }

    double meanErr = ChessboardCalibration::ComputeMeanReprojectionError(H,pair);

    std::cout << "H =\n" << H << "\n";
    std::cout << "Mean reprojection error: " << meanErr << " px\n";
    
    ChessboardCalibration::SaveHomography("homography.yml", H);
    cv::Mat output = Registration::AlignToThermal(thermalImage,H,cv::Size(320, 240));
    cv::imwrite("output.png", output);
    return 0;

}


int chess2(){
    std::string chess1 = "./pictures/chess7.png";
    std::string chess2 = "./pictures/chess7.png";
    cv::Mat rgbImage = cv::imread(chess1, cv::IMREAD_COLOR);
    cv::Mat thermalImage = cv::imread(chess2, cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR);

    if (rgbImage.empty() || thermalImage.empty()) {
        std::cerr << "Failed to load images.\n";
        return 1;
    }

    cv::Size boardSize(9, 6);
    ChessboardCalibration calib(boardSize);

    auto pair = calib.DetectPair(rgbImage, thermalImage);
    if (!pair.success) {
        std::cerr << "Chessboard detection failed.\n";
        return 1;
    }

    cv::Mat H = ChessboardCalibration::ComputeHomography(pair);
    if (H.empty()) {
        std::cerr << "Homography computation failed.\n";
        return 1;
    }

    double meanErr = ChessboardCalibration::ComputeMeanReprojectionError(H, pair);

    std::cout << "H =\n" << H << "\n";
    std::cout << "Mean reprojection error: " << meanErr << " px\n";

    ChessboardCalibration::SaveHomography("homography.yml", H);

    cv::Mat rgbCornersVis = rgbImage.clone();
    cv::drawChessboardCorners(rgbCornersVis, boardSize, pair.rgbCorners, true);
    cv::imwrite("rgb_corners.png", rgbCornersVis);

    cv::Mat thermalGray8u;
    if (thermalImage.type() == CV_32F || thermalImage.type() == CV_64F) {
        cv::normalize(thermalImage, thermalGray8u, 0, 255, cv::NORM_MINMAX, CV_8U);
    } else if (thermalImage.channels() == 3) {
        cv::cvtColor(thermalImage, thermalGray8u, cv::COLOR_BGR2GRAY);
    } else {
        thermalGray8u = thermalImage;
    }
    cv::Mat thermalCornersVis;
    cv::cvtColor(thermalGray8u, thermalCornersVis, cv::COLOR_GRAY2BGR);
    cv::drawChessboardCorners(thermalCornersVis, boardSize, pair.thermalCorners, true);
    cv::imwrite("thermal_corners.png", thermalCornersVis);

    cv::Mat thermalColor;
    cv::applyColorMap(thermalGray8u, thermalColor, cv::COLORMAP_JET);

    cv::Mat thermalWarped;
    cv::warpPerspective(thermalColor, thermalWarped, H, rgbImage.size());

    double alpha = 0.35;
    cv::Mat overlay;
    cv::addWeighted(rgbImage, 1.0 - alpha, thermalWarped, alpha, 0.0, overlay);
    cv::imwrite("overlay.png", overlay);

    std::cout << "Saved rgb_corners.png, thermal_corners.png, overlay.png\n";

    return 0;

}

int main(){
    chess2();
    return 1;
}