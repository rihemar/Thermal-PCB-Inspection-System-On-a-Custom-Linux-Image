// ThermalInspectorGUI.cpp
//
// Adds a tap-able on-screen UI (Calibrate / Detect buttons + a JET-colored
// threshold slider) on top of the existing RGB + thermal fusion pipeline.
//
// Sources:
//   RGB      -> IPWebcamCamera   (initialize() / captureFrame())
//   Thermal  -> ThermalCamera    (initialize() / captureFrame())
//
// Detection path mirrors BilateralJointUpsampletest() from main.cpp exactly:
//   JointBilateralUpsample::Upsample3D(thermalFrame, rgbFrame, GuideMode::RAW, ...)
//   -> HotspotDetector::DetectThreshold(enhanced, threshold)
//
// If your ThermalCamera header has a different filename, adjust the
// #include below accordingly — everything else only depends on
// initialize()/captureFrame() being present on both camera classes.

#include "IPWebcamCamera.hpp"
#include "ThermalCamera.hpp"          // <-- adjust filename if different in your project
#include "HotspotDetector.hpp"
#include "JointBilateralUpsample.hpp"
#include "ChessboardCalibration.hpp"

#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

namespace {

// ---------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------
constexpr int kWindowName_len = 0; // (unused, keeps clang-format happy)
const std::string kWindowName   = "Thermal Inspector";
const cv::Size    kBoardSize    = cv::Size(9, 6);     // chessboard inner corners
const std::string kHomographyPath = "rgb_to_thermal_homography.yml";

constexpr float kThresholdMinC = 30.0f;
constexpr float kThresholdMaxC = 120.0f;
constexpr float kThresholdInitC = 84.0f;

constexpr int   kPanelHeight   = 110;   // control-panel strip under the video
constexpr int   kButtonWidth   = 150;
constexpr int   kButtonHeight  = 44;
constexpr int   kMargin        = 20;

// Upsample params (same values as BilateralJointUpsampletest)
constexpr int   kWindowSize     = 6;
constexpr float kSigmaSpectral  = 0.05f;
constexpr double kSigmaSpatial  = 8.0;

// ---------------------------------------------------------------------
// UI primitives
// ---------------------------------------------------------------------
enum class UIMode { DETECT, CALIBRATE };

struct SliderUI {
    cv::Rect area;
    float minVal = 0.f, maxVal = 1.f, value = 0.f;
    bool dragging = false;

    float xToValue(int x) const {
        float t = (x - area.x) / static_cast<float>(area.width);
        t = std::min(1.0f, std::max(0.0f, t));
        return minVal + t * (maxVal - minVal);
    }
    int valueToX() const {
        float t = (value - minVal) / (maxVal - minVal);
        return area.x + static_cast<int>(t * area.width);
    }
    bool hitTest(int x, int y) const {
        cv::Rect grabZone(area.x, area.y - 10, area.width, area.height + 20);
        return grabZone.contains(cv::Point(x, y));
    }
};

struct ButtonUI {
    cv::Rect area;
    std::string label;
};

struct AppState {
    UIMode mode = UIMode::DETECT;
    SliderUI thresholdSlider;
    ButtonUI calibrateBtn;
    ButtonUI detectBtn;

    bool calibrationArmed = false;      // waiting for a good chessboard view
    bool calibrationJustSucceeded = false;
    std::chrono::steady_clock::time_point calibrationSuccessAt;

    std::string statusMessage;
    cv::Mat homography;                 // saved for later use if needed
};

AppState g_state;

// ---------------------------------------------------------------------
// Drawing helpers
// ---------------------------------------------------------------------
void drawColorGradientBar(cv::Mat& canvas, const cv::Rect& area) {
    cv::Mat gradient(1, std::max(2, area.width), CV_8UC1);
    for (int x = 0; x < gradient.cols; ++x) {
        gradient.at<uchar>(0, x) = static_cast<uchar>(255.0 * x / (gradient.cols - 1));
    }
    cv::Mat gradientColor;
    cv::applyColorMap(gradient, gradientColor, cv::COLORMAP_JET);
    cv::resize(gradientColor, gradientColor, cv::Size(area.width, area.height));
    gradientColor.copyTo(canvas(area));
    cv::rectangle(canvas, area, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
}

void drawSlider(cv::Mat& canvas, const SliderUI& slider) {
    drawColorGradientBar(canvas, slider.area);

    int knobX = slider.valueToX();
    int cy = slider.area.y + slider.area.height / 2;
    cv::line(canvas, {knobX, slider.area.y - 8}, {knobX, slider.area.y + slider.area.height + 8},
              cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    cv::circle(canvas, {knobX, cy}, 8, cv::Scalar(20, 20, 20), -1, cv::LINE_AA);
    cv::circle(canvas, {knobX, cy}, 8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    std::string label = cv::format("Hotspot Threshold: %.1f C", slider.value);
    cv::putText(canvas, label, {slider.area.x, slider.area.y - 14}, cv::FONT_HERSHEY_SIMPLEX,
                0.55, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

void drawButton(cv::Mat& canvas, const ButtonUI& btn, bool active) {
    cv::Scalar fill = active ? cv::Scalar(60, 160, 70) : cv::Scalar(65, 65, 65);
    cv::rectangle(canvas, btn.area, fill, -1, cv::LINE_AA);
    cv::rectangle(canvas, btn.area, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    int baseline = 0;
    cv::Size textSize = cv::getTextSize(btn.label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseline);
    cv::Point textOrg(btn.area.x + (btn.area.width - textSize.width) / 2,
                       btn.area.y + (btn.area.height + textSize.height) / 2);
    cv::putText(canvas, btn.label, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

void layoutUI(int frameWidth, int frameHeight) {
    int panelY = frameHeight;

    g_state.calibrateBtn.area = cv::Rect(kMargin, panelY + (kPanelHeight - kButtonHeight) / 2,
                                          kButtonWidth, kButtonHeight);
    g_state.calibrateBtn.label = "CALIBRATE";

    g_state.detectBtn.area = cv::Rect(kMargin * 2 + kButtonWidth,
                                       panelY + (kPanelHeight - kButtonHeight) / 2,
                                       kButtonWidth, kButtonHeight);
    g_state.detectBtn.label = "DETECT";

    int sliderX = kMargin * 3 + kButtonWidth * 2;
    int sliderWidth = frameWidth - sliderX - kMargin;
    g_state.thresholdSlider.area = cv::Rect(sliderX, panelY + kPanelHeight / 2,
                                             std::max(100, sliderWidth), 18);
    if (g_state.thresholdSlider.minVal == g_state.thresholdSlider.maxVal) {
        g_state.thresholdSlider.minVal = kThresholdMinC;
        g_state.thresholdSlider.maxVal = kThresholdMaxC;
        g_state.thresholdSlider.value = kThresholdInitC;
    }
}

void onMouse(int event, int x, int y, int /*flags*/, void* /*userdata*/) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        if (g_state.calibrateBtn.area.contains(cv::Point(x, y))) {
            g_state.mode = UIMode::CALIBRATE;
            g_state.calibrationArmed = true;
            g_state.calibrationJustSucceeded = false;
            g_state.statusMessage = "Calibrating... show the chessboard to both cameras";
        } else if (g_state.detectBtn.area.contains(cv::Point(x, y))) {
            g_state.mode = UIMode::DETECT;
            g_state.calibrationArmed = false;
            g_state.statusMessage.clear();
        } else if (g_state.thresholdSlider.hitTest(x, y)) {
            g_state.thresholdSlider.dragging = true;
            g_state.thresholdSlider.value = g_state.thresholdSlider.xToValue(x);
        }
    } else if (event == cv::EVENT_MOUSEMOVE) {
        if (g_state.thresholdSlider.dragging) {
            g_state.thresholdSlider.value = g_state.thresholdSlider.xToValue(x);
        }
    } else if (event == cv::EVENT_LBUTTONUP) {
        g_state.thresholdSlider.dragging = false;
    }
}

// ---------------------------------------------------------------------
// Detection path — mirrors BilateralJointUpsampletest() from main.cpp
// ---------------------------------------------------------------------
cv::Mat runDetection(const cv::Mat& thermalFrame, const cv::Mat& rgbFrame) {
    cv::Mat thermalFloat;
    if (thermalFrame.type() != CV_32F) {
        thermalFrame.convertTo(thermalFloat, CV_32F);
    } else {
        thermalFloat = thermalFrame;
    }

    cv::Mat enhanced = JointBilateralUpsample::Upsample3D(
        thermalFloat, rgbFrame,
        GuideMode::RAW,
        /*windowSize=*/kWindowSize,
        /*sigmaSpectral=*/kSigmaSpectral,
        /*sigmaSpatial=*/kSigmaSpatial);

    double minV, maxV;
    cv::minMaxLoc(enhanced, &minV, &maxV);
    cv::Mat display = JointBilateralUpsample::Colorize(enhanced, minV, maxV);

    HotRegion region = HotspotDetector::DetectThreshold(enhanced, g_state.thresholdSlider.value);
    if (region.boundingBox.area() > 0) {
        cv::rectangle(display, region.boundingBox, cv::Scalar(0, 255, 0), 2);
        cv::circle(display, region.peakPosition, 4, cv::Scalar(255, 255, 255), -1);
        cv::putText(display, cv::format("%.1f", region.peakTemperature),
                    region.peakPosition + cv::Point(8, -8), cv::FONT_HERSHEY_SIMPLEX,
                    0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    return JointBilateralUpsample::OverlayEdges(display, rgbFrame, 0.05);
}

// ---------------------------------------------------------------------
// Calibration path — single-shot chessboard capture, like chessboard()
// ---------------------------------------------------------------------
void runCalibrationAttempt(const cv::Mat& rgbFrame, const cv::Mat& thermalFrame,
                            ChessboardCalibration& calib) {
    cv::Mat thermal8u;
    if (thermalFrame.type() == CV_32F || thermalFrame.type() == CV_64F) {
        cv::normalize(thermalFrame, thermal8u, 0, 255, cv::NORM_MINMAX, CV_8U);
    } else if (thermalFrame.channels() == 3) {
        cv::cvtColor(thermalFrame, thermal8u, cv::COLOR_BGR2GRAY);
    } else {
        thermal8u = thermalFrame;
    }

    auto pair = calib.DetectPair(rgbFrame, thermal8u);
    if (!pair.success) {
        return; // keep waiting for a clean view — not an error
    }

    cv::Mat H = ChessboardCalibration::ComputeHomography(pair);
    if (H.empty()) {
        g_state.statusMessage = "Chessboard seen but homography failed, try again";
        return;
    }

    double meanErr = ChessboardCalibration::ComputeMeanReprojectionError(H, pair);
    ChessboardCalibration::SaveHomography(kHomographyPath, H);

    g_state.homography = H;
    g_state.statusMessage = cv::format("Calibrated (reproj err %.2f px) - saved %s",
                                        meanErr, kHomographyPath.c_str());
    g_state.calibrationArmed = false;
    g_state.calibrationJustSucceeded = true;
    g_state.calibrationSuccessAt = std::chrono::steady_clock::now();
}

} // namespace

// ---------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------
int runThermalInspectorGUI() {
    auto rgbCamera = std::make_unique<IPWebcamCamera>();
    if (!rgbCamera->initialize()) {
        std::cerr << "Failed to initialize IPWebcamCamera (RGB)\n";
        return -1;
    }

    auto thermalCamera = std::make_unique<ThermalCamera>();
    if (!thermalCamera->initialize()) {
        std::cerr << "Failed to initialize ThermalCamera\n";
        return -1;
    }

    ChessboardCalibration calib(kBoardSize);

    cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(kWindowName, onMouse);

    cv::Mat rgbFrame, thermalFrame;
    bool layoutDone = false;

    while (rgbCamera->captureFrame(rgbFrame) && thermalCamera->captureFrame(thermalFrame)) {
        if (!layoutDone) {
            layoutUI(rgbFrame.cols, rgbFrame.rows);
            layoutDone = true;
        }

        // Auto-return to Detect mode a moment after a successful calibration
        if (g_state.calibrationJustSucceeded) {
            auto elapsed = std::chrono::steady_clock::now() - g_state.calibrationSuccessAt;
            if (elapsed > std::chrono::seconds(2)) {
                g_state.calibrationJustSucceeded = false;
                g_state.mode = UIMode::DETECT;
                g_state.statusMessage.clear();
            }
        }

        cv::Mat contentFrame;
        if (g_state.mode == UIMode::DETECT) {
            contentFrame = runDetection(thermalFrame, rgbFrame);
        } else {
            contentFrame = rgbFrame.clone();
            if (g_state.calibrationArmed) {
                runCalibrationAttempt(rgbFrame, thermalFrame, calib);
            }
            cv::putText(contentFrame, "CALIBRATION MODE", {kMargin, 30},
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
        }

        // Compose full canvas: video content + control panel strip
        cv::Mat canvas(contentFrame.rows + kPanelHeight, contentFrame.cols, contentFrame.type(),
                        cv::Scalar(30, 30, 30));
        contentFrame.copyTo(canvas(cv::Rect(0, 0, contentFrame.cols, contentFrame.rows)));

        drawButton(canvas, g_state.calibrateBtn, g_state.mode == UIMode::CALIBRATE);
        drawButton(canvas, g_state.detectBtn, g_state.mode == UIMode::DETECT);
        drawSlider(canvas, g_state.thresholdSlider);

        if (!g_state.statusMessage.empty()) {
            cv::putText(canvas, g_state.statusMessage,
                        {kMargin, contentFrame.rows + kPanelHeight - 12},
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        }

        cv::imshow(kWindowName, canvas);
        int key = cv::waitKey(1);
        if (key == 27) break; // ESC quits
    }

    return 0;
}

// Standalone entry point. Remove/rename if you're calling
// runThermalInspectorGUI() from your existing main.cpp instead.
int main() {
    return runThermalInspectorGUI();
}