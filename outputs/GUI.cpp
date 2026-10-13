// // // ThermalInspectorGUI.cpp
// // //
// // // Adds a tap-able on-screen UI (Calibrate / Detect buttons + a JET-colored
// // // threshold slider) on top of the existing RGB + thermal fusion pipeline.
// // //
// // // Ubuntu:
// // //   RGB      -> IPWebcamCamera
// // //   Thermal  -> generateFakeHeatmapFromImage
// // //
// // // Detection path:
// // //   JointBilateralUpsample::Upsample3D(thermalFrame, rgbFrame, GuideMode::RAW, ...)
// // //   -> HotspotDetector::DetectThreshold(enhanced, threshold)

// // #include "IPWebcamCamera.hpp"
// // #include "generateFakeHeatmapFromImage.hpp"
// // #include "HotspotDetector.hpp"
// // #include "JointBilateralUpsample.hpp"
// // #include "ChessboardCalibration.hpp"
// // #include <thread>
// // #include <opencv2/opencv.hpp>
// // #include <chrono>
// // #include <iostream>
// // #include <memory>
// // #include <string>

// // namespace {

// // // ---------------------------------------------------------------------
// // // Tunables
// // // ---------------------------------------------------------------------

// // const std::string kWindowName = "Thermal Inspector";
// // const cv::Size kBoardSize = cv::Size(9, 6);
// // const std::string kHomographyPath = "rgb_to_thermal_homography.yml";

// // constexpr float kThresholdMinC = 30.0f;
// // constexpr float kThresholdMaxC = 120.0f;
// // constexpr float kThresholdInitC = 84.0f;

// // constexpr int kPanelHeight = 110;
// // constexpr int kButtonWidth = 150;
// // constexpr int kButtonHeight = 44;
// // constexpr int kMargin = 20;

// // // Same parameters as BilateralJointUpsampletest()
// // constexpr int kWindowSize = 6;
// // constexpr float kSigmaSpectral = 0.05f;
// // constexpr double kSigmaSpatial = 8.0;

// // // ---------------------------------------------------------------------
// // // UI primitives
// // // ---------------------------------------------------------------------

// // enum class UIMode {
// //     DETECT,
// //     CALIBRATE
// // };

// // struct SliderUI {
// //     cv::Rect area;
// //     float minVal = 0.f;
// //     float maxVal = 1.f;
// //     float value = 0.f;
// //     bool dragging = false;

// //     float xToValue(int x) const {
// //         float t = (x - area.x) / static_cast<float>(area.width);
// //         t = std::min(1.0f, std::max(0.0f, t));

// //         return minVal + t * (maxVal - minVal);
// //     }

// //     int valueToX() const {
// //         float t = (value - minVal) / (maxVal - minVal);

// //         return area.x + static_cast<int>(t * area.width);
// //     }

// //     bool hitTest(int x, int y) const {
// //         cv::Rect grabZone(
// //             area.x,
// //             area.y - 10,
// //             area.width,
// //             area.height + 20
// //         );

// //         return grabZone.contains(cv::Point(x, y));
// //     }
// // };

// // struct ButtonUI {
// //     cv::Rect area;
// //     std::string label;
// // };

// // struct AppState {
// //     UIMode mode = UIMode::DETECT;

// //     SliderUI thresholdSlider;

// //     ButtonUI calibrateBtn;
// //     ButtonUI detectBtn;

// //     bool calibrationArmed = false;
// //     bool calibrationJustSucceeded = false;

// //     std::chrono::steady_clock::time_point calibrationSuccessAt;

// //     std::string statusMessage;

// //     cv::Mat homography;
// // };

// // AppState g_state;

// // // ---------------------------------------------------------------------
// // // Drawing helpers
// // // ---------------------------------------------------------------------

// // void drawColorGradientBar(cv::Mat& canvas, const cv::Rect& area) {

// //     cv::Mat gradient(
// //         1,
// //         std::max(2, area.width),
// //         CV_8UC1
// //     );

// //     for (int x = 0; x < gradient.cols; ++x) {

// //         gradient.at<uchar>(0, x) =
// //             static_cast<uchar>(
// //                 255.0 * x / (gradient.cols - 1)
// //             );
// //     }

// //     cv::Mat gradientColor;

// //     cv::applyColorMap(
// //         gradient,
// //         gradientColor,
// //         cv::COLORMAP_JET
// //     );

// //     cv::resize(
// //         gradientColor,
// //         gradientColor,
// //         cv::Size(area.width, area.height)
// //     );

// //     gradientColor.copyTo(canvas(area));

// //     cv::rectangle(
// //         canvas,
// //         area,
// //         cv::Scalar(230, 230, 230),
// //         1,
// //         cv::LINE_AA
// //     );
// // }

// // void drawSlider(
// //     cv::Mat& canvas,
// //     const SliderUI& slider
// // ) {

// //     drawColorGradientBar(canvas, slider.area);

// //     int knobX = slider.valueToX();

// //     int cy =
// //         slider.area.y +
// //         slider.area.height / 2;

// //     cv::line(
// //         canvas,
// //         {knobX, slider.area.y - 8},
// //         {
// //             knobX,
// //             slider.area.y +
// //                 slider.area.height +
// //                 8
// //         },
// //         cv::Scalar(255, 255, 255),
// //         2,
// //         cv::LINE_AA
// //     );

// //     cv::circle(
// //         canvas,
// //         {knobX, cy},
// //         8,
// //         cv::Scalar(20, 20, 20),
// //         -1,
// //         cv::LINE_AA
// //     );

// //     cv::circle(
// //         canvas,
// //         {knobX, cy},
// //         8,
// //         cv::Scalar(255, 255, 255),
// //         2,
// //         cv::LINE_AA
// //     );

// //     std::string label =
// //         cv::format(
// //             "Hotspot Threshold: %.1f C",
// //             slider.value
// //         );

// //     cv::putText(
// //         canvas,
// //         label,
// //         {
// //             slider.area.x,
// //             slider.area.y - 14
// //         },
// //         cv::FONT_HERSHEY_SIMPLEX,
// //         0.55,
// //         cv::Scalar(255, 255, 255),
// //         1,
// //         cv::LINE_AA
// //     );
// // }

// // void drawButton(
// //     cv::Mat& canvas,
// //     const ButtonUI& btn,
// //     bool active
// // ) {

// //     cv::Scalar fill =
// //         active
// //             ? cv::Scalar(60, 160, 70)
// //             : cv::Scalar(65, 65, 65);

// //     cv::rectangle(
// //         canvas,
// //         btn.area,
// //         fill,
// //         -1,
// //         cv::LINE_AA
// //     );

// //     cv::rectangle(
// //         canvas,
// //         btn.area,
// //         cv::Scalar(255, 255, 255),
// //         1,
// //         cv::LINE_AA
// //     );

// //     int baseline = 0;

// //     cv::Size textSize =
// //         cv::getTextSize(
// //             btn.label,
// //             cv::FONT_HERSHEY_SIMPLEX,
// //             0.6,
// //             1,
// //             &baseline
// //         );

// //     cv::Point textOrg(
// //         btn.area.x +
// //             (btn.area.width - textSize.width) / 2,

// //         btn.area.y +
// //             (btn.area.height + textSize.height) / 2
// //     );

// //     cv::putText(
// //         canvas,
// //         btn.label,
// //         textOrg,
// //         cv::FONT_HERSHEY_SIMPLEX,
// //         0.6,
// //         cv::Scalar(255, 255, 255),
// //         1,
// //         cv::LINE_AA
// //     );
// // }

// // // ---------------------------------------------------------------------
// // // UI layout
// // // ---------------------------------------------------------------------

// // void layoutUI(
// //     int frameWidth,
// //     int frameHeight
// // ) {

// //     int panelY = frameHeight;

// //     g_state.calibrateBtn.area =
// //         cv::Rect(
// //             kMargin,
// //             panelY +
// //                 (kPanelHeight - kButtonHeight) / 2,
// //             kButtonWidth,
// //             kButtonHeight
// //         );

// //     g_state.calibrateBtn.label = "CALIBRATE";

// //     g_state.detectBtn.area =
// //         cv::Rect(
// //             kMargin * 2 + kButtonWidth,
// //             panelY +
// //                 (kPanelHeight - kButtonHeight) / 2,
// //             kButtonWidth,
// //             kButtonHeight
// //         );

// //     g_state.detectBtn.label = "DETECT";

// //     int sliderX =
// //         kMargin * 3 +
// //         kButtonWidth * 2;

// //     int sliderWidth =
// //         frameWidth -
// //         sliderX -
// //         kMargin;

// //     g_state.thresholdSlider.area =
// //         cv::Rect(
// //             sliderX,
// //             panelY + kPanelHeight / 2,
// //             std::max(100, sliderWidth),
// //             18
// //         );

// //     if (
// //         g_state.thresholdSlider.minVal ==
// //         g_state.thresholdSlider.maxVal
// //     ) {

// //         g_state.thresholdSlider.minVal =
// //             kThresholdMinC;

// //         g_state.thresholdSlider.maxVal =
// //             kThresholdMaxC;

// //         g_state.thresholdSlider.value =
// //             kThresholdInitC;
// //     }
// // }

// // // ---------------------------------------------------------------------
// // // Mouse callback
// // // ---------------------------------------------------------------------

// // void onMouse(
// //     int event,
// //     int x,
// //     int y,
// //     int /*flags*/,
// //     void* /*userdata*/
// // ) {

// //     if (event == cv::EVENT_LBUTTONDOWN) {

// //         if (
// //             g_state.calibrateBtn.area.contains(
// //                 cv::Point(x, y)
// //             )
// //         ) {

// //             g_state.mode = UIMode::CALIBRATE;

// //             g_state.calibrationArmed = true;

// //             g_state.calibrationJustSucceeded = false;

// //             g_state.statusMessage =
// //                 "Calibrating... show the chessboard to both cameras";
// //         }

// //         else if (
// //             g_state.detectBtn.area.contains(
// //                 cv::Point(x, y)
// //             )
// //         ) {

// //             g_state.mode = UIMode::DETECT;

// //             g_state.calibrationArmed = false;

// //             g_state.statusMessage.clear();
// //         }

// //         else if (
// //             g_state.thresholdSlider.hitTest(x, y)
// //         ) {

// //             g_state.thresholdSlider.dragging = true;

// //             g_state.thresholdSlider.value =
// //                 g_state.thresholdSlider.xToValue(x);
// //         }
// //     }

// //     else if (event == cv::EVENT_MOUSEMOVE) {

// //         if (g_state.thresholdSlider.dragging) {

// //             g_state.thresholdSlider.value =
// //                 g_state.thresholdSlider.xToValue(x);
// //         }
// //     }

// //     else if (event == cv::EVENT_LBUTTONUP) {

// //         g_state.thresholdSlider.dragging = false;
// //     }
// // }

// // // ---------------------------------------------------------------------
// // // Detection path
// // // ---------------------------------------------------------------------

// // cv::Mat runDetection(
// //     const cv::Mat& thermalFrame,
// //     const cv::Mat& rgbFrame
// // ) {

// //     cv::Mat thermalFloat;

// //     if (thermalFrame.type() != CV_32F) {

// //         thermalFrame.convertTo(
// //             thermalFloat,
// //             CV_32F
// //         );

// //     } else {

// //         thermalFloat = thermalFrame;
// //     }

// //     cv::Mat enhanced =
// //         JointBilateralUpsample::Upsample3D(
// //             thermalFloat,
// //             rgbFrame,
// //             GuideMode::RAW,
// //             kWindowSize,
// //             kSigmaSpectral,
// //             kSigmaSpatial
// //         );

// //     double minV;
// //     double maxV;

// //     cv::minMaxLoc(
// //         enhanced,
// //         &minV,
// //         &maxV
// //     );

// //     cv::Mat display =
// //         JointBilateralUpsample::Colorize(
// //             enhanced,
// //             minV,
// //             maxV
// //         );

// //     HotRegion region =
// //         HotspotDetector::DetectThreshold(
// //             enhanced,
// //             g_state.thresholdSlider.value
// //         );

// //     if (region.boundingBox.area() > 0) {

// //         cv::rectangle(
// //             display,
// //             region.boundingBox,
// //             cv::Scalar(0, 255, 0),
// //             2
// //         );

// //         cv::circle(
// //             display,
// //             region.peakPosition,
// //             4,
// //             cv::Scalar(255, 255, 255),
// //             -1
// //         );

// //         cv::putText(
// //             display,
// //             cv::format(
// //                 "%.1f",
// //                 region.peakTemperature
// //             ),
// //             region.peakPosition +
// //                 cv::Point(8, -8),
// //             cv::FONT_HERSHEY_SIMPLEX,
// //             0.5,
// //             cv::Scalar(255, 255, 255),
// //             1,
// //             cv::LINE_AA
// //         );
// //     }

// //     return JointBilateralUpsample::OverlayEdges(
// //         display,
// //         rgbFrame,
// //         0.05
// //     );
// // }

// // // ---------------------------------------------------------------------
// // // Calibration path
// // // ---------------------------------------------------------------------

// // void runCalibrationAttempt(
// //     const cv::Mat& rgbFrame,
// //     const cv::Mat& thermalFrame,
// //     ChessboardCalibration& calib
// // ) {

// //     cv::Mat thermal8u;

// //     if (
// //         thermalFrame.type() == CV_32F ||
// //         thermalFrame.type() == CV_64F
// //     ) {

// //         cv::normalize(
// //             thermalFrame,
// //             thermal8u,
// //             0,
// //             255,
// //             cv::NORM_MINMAX,
// //             CV_8U
// //         );

// //     }

// //     else if (thermalFrame.channels() == 3) {

// //         cv::cvtColor(
// //             thermalFrame,
// //             thermal8u,
// //             cv::COLOR_BGR2GRAY
// //         );

// //     }

// //     else {

// //         thermal8u = thermalFrame;
// //     }

// //     auto pair =
// //         calib.DetectPair(
// //             rgbFrame,
// //             thermal8u
// //         );

// //     if (!pair.success) {
// //         return;
// //     }

// //     cv::Mat H =
// //         ChessboardCalibration::ComputeHomography(
// //             pair
// //         );

// //     if (H.empty()) {

// //         g_state.statusMessage =
// //             "Chessboard seen but homography failed, try again";

// //         return;
// //     }

// //     double meanErr =
// //         ChessboardCalibration::ComputeMeanReprojectionError(
// //             H,
// //             pair
// //         );

// //     ChessboardCalibration::SaveHomography(
// //         kHomographyPath,
// //         H
// //     );

// //     g_state.homography = H;

// //     g_state.statusMessage =
// //         cv::format(
// //             "Calibrated (reproj err %.2f px) - saved %s",
// //             meanErr,
// //             kHomographyPath.c_str()
// //         );

// //     g_state.calibrationArmed = false;

// //     g_state.calibrationJustSucceeded = true;

// //     g_state.calibrationSuccessAt =
// //         std::chrono::steady_clock::now();
// // }

// // } // namespace

// // // ---------------------------------------------------------------------
// // // Entry point
// // // ---------------------------------------------------------------------

// // int runThermalInspectorGUI()
// // {
// //     // -------------------------------------------------------------
// //     // RGB camera
// //     // -------------------------------------------------------------

// //     auto rgbCamera =
// //         std::make_unique<IPWebcamCamera>();

// //     if (!rgbCamera->initialize()) {

// //         std::cerr
// //             << "Failed to initialize IPWebcamCamera (RGB)\n";

// //         return -1;
// //     }

// //     // -------------------------------------------------------------
// //     // Fake thermal generator
// //     //
// //     // Same configuration as BilateralJointUpsampletest()
// //     // in main.cpp.
// //     // -------------------------------------------------------------

// //     generateFakeHeatmapFromImage thermalGenerator(
// //         55.0,                // RGB HFOV
// //         55.0,                // Thermal HFOV
// //         cv::Size(16, 12),    // Thermal resolution
// //         25.0f,               // Ambient temperature
// //         85.0f,               // Maximum hotspot temperature
// //         0.6                  // Noise standard deviation
// //     );

// //     // -------------------------------------------------------------
// //     // Chessboard calibration
// //     // -------------------------------------------------------------

// //     ChessboardCalibration calib(kBoardSize);

// //     // -------------------------------------------------------------
// //     // Window / UI
// //     // -------------------------------------------------------------

// //     cv::namedWindow(
// //         kWindowName,
// //         cv::WINDOW_AUTOSIZE
// //     );

// //     cv::setMouseCallback(
// //         kWindowName,
// //         onMouse
// //     );

// //     // -------------------------------------------------------------
// //     // Main loop
// //     // -------------------------------------------------------------

// //     cv::Mat rgbFrame;
// //     cv::Mat thermalFrame;

// //     bool layoutDone = false;

// //     while (rgbCamera->captureFrame(rgbFrame)) {

// //         // ---------------------------------------------------------
// //         // Generate fake thermal frame from current RGB frame
// //         // ---------------------------------------------------------

// //         thermalFrame =
// //             thermalGenerator.getHeatmap(rgbFrame);

// //         // ---------------------------------------------------------
// //         // UI layout
// //         // ---------------------------------------------------------

// //         if (!layoutDone) {

// //             layoutUI(
// //                 rgbFrame.cols,
// //                 rgbFrame.rows
// //             );

// //             layoutDone = true;
// //         }

// //         // ---------------------------------------------------------
// //         // Auto-return to Detect mode after successful calibration
// //         // ---------------------------------------------------------

// //         if (g_state.calibrationJustSucceeded) {

// //             auto elapsed =
// //                 std::chrono::steady_clock::now() -
// //                 g_state.calibrationSuccessAt;

// //             if (
// //                 elapsed >
// //                 std::chrono::seconds(2)
// //             ) {

// //                 g_state.calibrationJustSucceeded =
// //                     false;

// //                 g_state.mode =
// //                     UIMode::DETECT;

// //                 g_state.statusMessage.clear();
// //             }
// //         }

// //         // ---------------------------------------------------------
// //         // Generate content
// //         // ---------------------------------------------------------

// //         cv::Mat contentFrame;

// //         if (g_state.mode == UIMode::DETECT) {

// //             contentFrame =
// //                 runDetection(
// //                     thermalFrame,
// //                     rgbFrame
// //                 );

// //         } else {

// //             contentFrame =
// //                 rgbFrame.clone();

// //             if (g_state.calibrationArmed) {

// //                 runCalibrationAttempt(
// //                     rgbFrame,
// //                     thermalFrame,
// //                     calib
// //                 );
// //             }

// //             cv::putText(
// //                 contentFrame,
// //                 "CALIBRATION MODE",
// //                 {kMargin, 30},
// //                 cv::FONT_HERSHEY_SIMPLEX,
// //                 0.8,
// //                 cv::Scalar(0, 255, 255),
// //                 2,
// //                 cv::LINE_AA
// //             );
// //         }

// //         // ---------------------------------------------------------
// //         // Compose full canvas
// //         // ---------------------------------------------------------

// //         cv::Mat canvas(
// //             contentFrame.rows + kPanelHeight,
// //             contentFrame.cols,
// //             contentFrame.type(),
// //             cv::Scalar(30, 30, 30)
// //         );

// //         contentFrame.copyTo(
// //             canvas(
// //                 cv::Rect(
// //                     0,
// //                     0,
// //                     contentFrame.cols,
// //                     contentFrame.rows
// //                 )
// //             )
// //         );

// //         // ---------------------------------------------------------
// //         // UI controls
// //         // ---------------------------------------------------------

// //         drawButton(
// //             canvas,
// //             g_state.calibrateBtn,
// //             g_state.mode == UIMode::CALIBRATE
// //         );

// //         drawButton(
// //             canvas,
// //             g_state.detectBtn,
// //             g_state.mode == UIMode::DETECT
// //         );

// //         drawSlider(
// //             canvas,
// //             g_state.thresholdSlider
// //         );

// //         // ---------------------------------------------------------
// //         // Status message
// //         // ---------------------------------------------------------

// //         if (!g_state.statusMessage.empty()) {

// //             cv::putText(
// //                 canvas,
// //                 g_state.statusMessage,
// //                 {
// //                     kMargin,
// //                     contentFrame.rows +
// //                         kPanelHeight -
// //                         12
// //                 },
// //                 cv::FONT_HERSHEY_SIMPLEX,
// //                 0.5,
// //                 cv::Scalar(200, 200, 200),
// //                 1,
// //                 cv::LINE_AA
// //             );
// //         }

// //         // ---------------------------------------------------------
// //         // Display
// //         // ---------------------------------------------------------

// //         cv::imshow(
// //             kWindowName,
// //             canvas
// //         );

// //         int key =
// //             cv::waitKey(1);

// //         if (key == 27) {
// //             break;
// //         }

// //         // Match the ~1 second pacing used in your test function.
// //         std::this_thread::sleep_for(
// //             std::chrono::milliseconds(1000)
// //         );
// //     }

// //     return 0;
// // }

// // // ---------------------------------------------------------------------
// // // Standalone entry point
// // // ---------------------------------------------------------------------

// // int main()
// // {
// //     return runThermalInspectorGUI();
// // }

// // GUI.cpp
// //
// // RGB source:
// //   PC / USB webcam through OpenCV VideoCapture
// //
// // Thermal source:
// //   generateFakeHeatmapFromImage
// //
// // Pipeline:
// //   PC Camera
// //      ↓
// //   RGB Frame
// //      ↓
// //   Fake 16x12 Thermal Frame
// //      ↓
// //   Joint Bilateral Upsampling
// //      ↓
// //   Hotspot Detection
// //      ↓
// //   GUI

// #include "generateFakeHeatmapFromImage.hpp"
// #include "HotspotDetector.hpp"
// #include "JointBilateralUpsample.hpp"
// #include "ChessboardCalibration.hpp"

// #include <opencv2/opencv.hpp>

// #include <chrono>
// #include <iostream>
// #include <memory>
// #include <string>
// #include <thread>
// #include <algorithm>

// namespace {

// // =====================================================================
// // Tunables
// // =====================================================================

// const std::string kWindowName = "Thermal Inspector";

// const cv::Size kBoardSize = cv::Size(9, 6);

// const std::string kHomographyPath =
//     "rgb_to_thermal_homography.yml";

// // Hotspot threshold slider
// constexpr float kThresholdMinC  = 30.0f;
// constexpr float kThresholdMaxC  = 120.0f;
// constexpr float kThresholdInitC = 84.0f;

// // GUI panel
// constexpr int kPanelHeight = 110;
// constexpr int kButtonWidth = 150;
// constexpr int kButtonHeight = 44;
// constexpr int kMargin = 20;

// // Joint bilateral upsampling parameters
// constexpr int kWindowSize = 6;
// constexpr float kSigmaSpectral = 0.05f;
// constexpr double kSigmaSpatial = 8.0;


// // =====================================================================
// // UI
// // =====================================================================

// enum class UIMode {
//     DETECT,
//     CALIBRATE
// };


// // ---------------------------------------------------------------------
// // Slider
// // ---------------------------------------------------------------------

// struct SliderUI {

//     cv::Rect area;

//     float minVal = 0.0f;
//     float maxVal = 1.0f;
//     float value = 0.0f;

//     bool dragging = false;


//     float xToValue(int x) const
//     {
//         float t =
//             (x - area.x) /
//             static_cast<float>(area.width);

//         t = std::min(
//             1.0f,
//             std::max(0.0f, t)
//         );

//         return minVal +
//                t * (maxVal - minVal);
//     }


//     int valueToX() const
//     {
//         float t =
//             (value - minVal) /
//             (maxVal - minVal);

//         return area.x +
//                static_cast<int>(
//                    t * area.width
//                );
//     }


//     bool hitTest(int x, int y) const
//     {
//         cv::Rect grabZone(
//             area.x,
//             area.y - 10,
//             area.width,
//             area.height + 20
//         );

//         return grabZone.contains(
//             cv::Point(x, y)
//         );
//     }
// };


// // ---------------------------------------------------------------------
// // Button
// // ---------------------------------------------------------------------

// struct ButtonUI {

//     cv::Rect area;

//     std::string label;
// };


// // ---------------------------------------------------------------------
// // Application state
// // ---------------------------------------------------------------------

// struct AppState {

//     UIMode mode = UIMode::DETECT;

//     SliderUI thresholdSlider;

//     ButtonUI calibrateBtn;
//     ButtonUI detectBtn;

//     bool calibrationArmed = false;

//     bool calibrationJustSucceeded = false;

//     std::chrono::steady_clock::time_point
//         calibrationSuccessAt;

//     std::string statusMessage;

//     cv::Mat homography;
// };


// AppState g_state;


// // =====================================================================
// // DRAWING
// // =====================================================================


// // ---------------------------------------------------------------------
// // Draw thermal color gradient
// // ---------------------------------------------------------------------

// void drawColorGradientBar(
//     cv::Mat& canvas,
//     const cv::Rect& area
// )
// {
//     cv::Mat gradient(
//         1,
//         std::max(2, area.width),
//         CV_8UC1
//     );


//     for (int x = 0; x < gradient.cols; ++x) {

//         gradient.at<uchar>(0, x) =
//             static_cast<uchar>(
//                 255.0 *
//                 x /
//                 (gradient.cols - 1)
//             );
//     }


//     cv::Mat gradientColor;

//     cv::applyColorMap(
//         gradient,
//         gradientColor,
//         cv::COLORMAP_JET
//     );


//     cv::resize(
//         gradientColor,
//         gradientColor,
//         cv::Size(
//             area.width,
//             area.height
//         )
//     );


//     gradientColor.copyTo(
//         canvas(area)
//     );


//     cv::rectangle(
//         canvas,
//         area,
//         cv::Scalar(
//             230,
//             230,
//             230
//         ),
//         1,
//         cv::LINE_AA
//     );
// }


// // ---------------------------------------------------------------------
// // Draw slider
// // ---------------------------------------------------------------------

// void drawSlider(
//     cv::Mat& canvas,
//     const SliderUI& slider
// )
// {
//     drawColorGradientBar(
//         canvas,
//         slider.area
//     );


//     int knobX =
//         slider.valueToX();


//     int cy =
//         slider.area.y +
//         slider.area.height / 2;


//     // Vertical indicator
//     cv::line(
//         canvas,
//         {
//             knobX,
//             slider.area.y - 8
//         },
//         {
//             knobX,
//             slider.area.y +
//                 slider.area.height +
//                 8
//         },
//         cv::Scalar(
//             255,
//             255,
//             255
//         ),
//         2,
//         cv::LINE_AA
//     );


//     // Knob
//     cv::circle(
//         canvas,
//         {
//             knobX,
//             cy
//         },
//         8,
//         cv::Scalar(
//             20,
//             20,
//             20
//         ),
//         -1,
//         cv::LINE_AA
//     );


//     cv::circle(
//         canvas,
//         {
//             knobX,
//             cy
//         },
//         8,
//         cv::Scalar(
//             255,
//             255,
//             255
//         ),
//         2,
//         cv::LINE_AA
//     );


//     std::string label =
//         cv::format(
//             "Hotspot Threshold: %.1f C",
//             slider.value
//         );


//     cv::putText(
//         canvas,
//         label,
//         {
//             slider.area.x,
//             slider.area.y - 14
//         },
//         cv::FONT_HERSHEY_SIMPLEX,
//         0.55,
//         cv::Scalar(
//             255,
//             255,
//             255
//         ),
//         1,
//         cv::LINE_AA
//     );
// }


// // ---------------------------------------------------------------------
// // Draw button
// // ---------------------------------------------------------------------

// void drawButton(
//     cv::Mat& canvas,
//     const ButtonUI& btn,
//     bool active
// )
// {
//     cv::Scalar fill =
//         active
//             ? cv::Scalar(
//                 60,
//                 160,
//                 70
//               )
//             : cv::Scalar(
//                 65,
//                 65,
//                 65
//               );


//     cv::rectangle(
//         canvas,
//         btn.area,
//         fill,
//         -1,
//         cv::LINE_AA
//     );


//     cv::rectangle(
//         canvas,
//         btn.area,
//         cv::Scalar(
//             255,
//             255,
//             255
//         ),
//         1,
//         cv::LINE_AA
//     );


//     int baseline = 0;


//     cv::Size textSize =
//         cv::getTextSize(
//             btn.label,
//             cv::FONT_HERSHEY_SIMPLEX,
//             0.6,
//             1,
//             &baseline
//         );


//     cv::Point textOrg(
//         btn.area.x +
//             (
//                 btn.area.width -
//                 textSize.width
//             ) / 2,

//         btn.area.y +
//             (
//                 btn.area.height +
//                 textSize.height
//             ) / 2
//     );


//     cv::putText(
//         canvas,
//         btn.label,
//         textOrg,
//         cv::FONT_HERSHEY_SIMPLEX,
//         0.6,
//         cv::Scalar(
//             255,
//             255,
//             255
//         ),
//         1,
//         cv::LINE_AA
//     );
// }


// // =====================================================================
// // UI LAYOUT
// // =====================================================================

// void layoutUI(
//     int frameWidth,
//     int frameHeight
// )
// {
//     int panelY =
//         frameHeight;


//     // -------------------------------------------------------------
//     // CALIBRATE button
//     // -------------------------------------------------------------

//     g_state.calibrateBtn.area =
//         cv::Rect(
//             kMargin,
//             panelY +
//                 (
//                     kPanelHeight -
//                     kButtonHeight
//                 ) / 2,

//             kButtonWidth,
//             kButtonHeight
//         );


//     g_state.calibrateBtn.label =
//         "CALIBRATE";


//     // -------------------------------------------------------------
//     // DETECT button
//     // -------------------------------------------------------------

//     g_state.detectBtn.area =
//         cv::Rect(
//             kMargin * 2 +
//                 kButtonWidth,

//             panelY +
//                 (
//                     kPanelHeight -
//                     kButtonHeight
//                 ) / 2,

//             kButtonWidth,
//             kButtonHeight
//         );


//     g_state.detectBtn.label =
//         "DETECT";


//     // -------------------------------------------------------------
//     // Threshold slider
//     // -------------------------------------------------------------

//     int sliderX =
//         kMargin * 3 +
//         kButtonWidth * 2;


//     int sliderWidth =
//         frameWidth -
//         sliderX -
//         kMargin;


//     g_state.thresholdSlider.area =
//         cv::Rect(
//             sliderX,
//             panelY +
//                 kPanelHeight / 2,
//             std::max(
//                 100,
//                 sliderWidth
//             ),
//             18
//         );


//     g_state.thresholdSlider.minVal =
//         kThresholdMinC;

//     g_state.thresholdSlider.maxVal =
//         kThresholdMaxC;

//     g_state.thresholdSlider.value =
//         kThresholdInitC;
// }


// // =====================================================================
// // MOUSE CALLBACK
// // =====================================================================

// void onMouse(
//     int event,
//     int x,
//     int y,
//     int /*flags*/,
//     void* /*userdata*/
// )
// {
//     // -------------------------------------------------------------
//     // Mouse button pressed
//     // -------------------------------------------------------------

//     if (event == cv::EVENT_LBUTTONDOWN) {

//         // ---------------------------------------------------------
//         // CALIBRATE
//         // ---------------------------------------------------------

//         if (
//             g_state.calibrateBtn.area.contains(
//                 cv::Point(x, y)
//             )
//         )
//         {
//             g_state.mode =
//                 UIMode::CALIBRATE;

//             g_state.calibrationArmed =
//                 true;

//             g_state.calibrationJustSucceeded =
//                 false;

//             g_state.statusMessage =
//                 "Calibrating... show the chessboard";
//         }


//         // ---------------------------------------------------------
//         // DETECT
//         // ---------------------------------------------------------

//         else if (
//             g_state.detectBtn.area.contains(
//                 cv::Point(x, y)
//             )
//         )
//         {
//             g_state.mode =
//                 UIMode::DETECT;

//             g_state.calibrationArmed =
//                 false;

//             g_state.statusMessage.clear();
//         }


//         // ---------------------------------------------------------
//         // Slider
//         // ---------------------------------------------------------

//         else if (
//             g_state.thresholdSlider.hitTest(
//                 x,
//                 y
//             )
//         )
//         {
//             g_state.thresholdSlider.dragging =
//                 true;

//             g_state.thresholdSlider.value =
//                 g_state.thresholdSlider.xToValue(
//                     x
//                 );
//         }
//     }


//     // -------------------------------------------------------------
//     // Mouse moving
//     // -------------------------------------------------------------

//     else if (
//         event == cv::EVENT_MOUSEMOVE
//     )
//     {
//         if (
//             g_state.thresholdSlider.dragging
//         )
//         {
//             g_state.thresholdSlider.value =
//                 g_state.thresholdSlider.xToValue(
//                     x
//                 );
//         }
//     }


//     // -------------------------------------------------------------
//     // Mouse released
//     // -------------------------------------------------------------

//     else if (
//         event == cv::EVENT_LBUTTONUP
//     )
//     {
//         g_state.thresholdSlider.dragging =
//             false;
//     }
// }


// // =====================================================================
// // DETECTION
// // =====================================================================

// cv::Mat runDetection(
//     const cv::Mat& thermalFrame,
//     const cv::Mat& rgbFrame
// )
// {
//     // -------------------------------------------------------------
//     // Make sure thermal data is float
//     // -------------------------------------------------------------

//     cv::Mat thermalFloat;


//     if (
//         thermalFrame.type() != CV_32F
//     )
//     {
//         thermalFrame.convertTo(
//             thermalFloat,
//             CV_32F
//         );
//     }
//     else
//     {
//         thermalFloat =
//             thermalFrame;
//     }


//     // -------------------------------------------------------------
//     // Joint bilateral upsampling
//     // -------------------------------------------------------------

//     cv::Mat enhanced =
//         JointBilateralUpsample::Upsample3D(
//             thermalFloat,
//             rgbFrame,
//             GuideMode::RAW,
//             kWindowSize,
//             kSigmaSpectral,
//             kSigmaSpatial
//         );


//     // -------------------------------------------------------------
//     // Find temperature range
//     // -------------------------------------------------------------

//     double minV;
//     double maxV;


//     cv::minMaxLoc(
//         enhanced,
//         &minV,
//         &maxV
//     );


//     // -------------------------------------------------------------
//     // Colorize thermal image
//     // -------------------------------------------------------------

//     cv::Mat display =
//         JointBilateralUpsample::Colorize(
//             enhanced,
//             minV,
//             maxV
//         );


//     // -------------------------------------------------------------
//     // Hotspot detection
//     // -------------------------------------------------------------

//     HotRegion region =
//         HotspotDetector::DetectThreshold(
//             enhanced,
//             g_state.thresholdSlider.value
//         );


//     // -------------------------------------------------------------
//     // Draw hotspot
//     // -------------------------------------------------------------

//     if (
//         region.boundingBox.area() > 0
//     )
//     {
//         cv::rectangle(
//             display,
//             region.boundingBox,
//             cv::Scalar(
//                 0,
//                 255,
//                 0
//             ),
//             2
//         );


//         cv::circle(
//             display,
//             region.peakPosition,
//             4,
//             cv::Scalar(
//                 255,
//                 255,
//                 255
//             ),
//             -1
//         );


//         cv::putText(
//             display,
//             cv::format(
//                 "%.1f C",
//                 region.peakTemperature
//             ),
//             region.peakPosition +
//                 cv::Point(
//                     8,
//                     -8
//                 ),
//             cv::FONT_HERSHEY_SIMPLEX,
//             0.5,
//             cv::Scalar(
//                 255,
//                 255,
//                 255
//             ),
//             1,
//             cv::LINE_AA
//         );
//     }


//     // -------------------------------------------------------------
//     // Overlay RGB edges
//     // -------------------------------------------------------------

//     return JointBilateralUpsample::OverlayEdges(
//         display,
//         rgbFrame,
//         0.05
//     );
// }


// // =====================================================================
// // CALIBRATION
// // =====================================================================

// void runCalibrationAttempt(
//     const cv::Mat& rgbFrame,
//     const cv::Mat& thermalFrame,
//     ChessboardCalibration& calib
// )
// {
//     // -------------------------------------------------------------
//     // Convert fake thermal frame to 8-bit
//     // -------------------------------------------------------------

//     cv::Mat thermal8u;


//     if (
//         thermalFrame.type() == CV_32F ||
//         thermalFrame.type() == CV_64F
//     )
//     {
//         cv::normalize(
//             thermalFrame,
//             thermal8u,
//             0,
//             255,
//             cv::NORM_MINMAX,
//             CV_8U
//         );
//     }

//     else if (
//         thermalFrame.channels() == 3
//     )
//     {
//         cv::cvtColor(
//             thermalFrame,
//             thermal8u,
//             cv::COLOR_BGR2GRAY
//         );
//     }

//     else
//     {
//         thermal8u =
//             thermalFrame;
//     }


//     // -------------------------------------------------------------
//     // Detect chessboard in RGB + thermal
//     // -------------------------------------------------------------

//     auto pair =
//         calib.DetectPair(
//             rgbFrame,
//             thermal8u
//         );


//     if (!pair.success) {
//         return;
//     }


//     // -------------------------------------------------------------
//     // Compute homography
//     // -------------------------------------------------------------

//     cv::Mat H =
//         ChessboardCalibration::ComputeHomography(
//             pair
//         );


//     if (H.empty())
//     {
//         g_state.statusMessage =
//             "Chessboard detected but homography failed";

//         return;
//     }


//     // -------------------------------------------------------------
//     // Calculate reprojection error
//     // -------------------------------------------------------------

//     double meanErr =
//         ChessboardCalibration::
//         ComputeMeanReprojectionError(
//             H,
//             pair
//         );


//     // -------------------------------------------------------------
//     // Save homography
//     // -------------------------------------------------------------

//     ChessboardCalibration::
//     SaveHomography(
//         kHomographyPath,
//         H
//     );


//     g_state.homography =
//         H;


//     g_state.statusMessage =
//         cv::format(
//             "Calibrated (reproj err %.2f px) - saved %s",
//             meanErr,
//             kHomographyPath.c_str()
//         );


//     g_state.calibrationArmed =
//         false;


//     g_state.calibrationJustSucceeded =
//         true;


//     g_state.calibrationSuccessAt =
//         std::chrono::steady_clock::now();
// }

// } // namespace


// // =====================================================================
// // MAIN GUI
// // =====================================================================

// int runThermalInspectorGUI()
// {
//     // =================================================================
//     // PC CAMERA
//     // =================================================================

//     cv::VideoCapture rgbCamera(0);


//     if (!rgbCamera.isOpened())
//     {
//         std::cerr
//             << "ERROR: Failed to open PC camera.\n";

//         std::cerr
//             << "Try changing camera index 0 to 1 or 2.\n";

//         return -1;
//     }


//     // -------------------------------------------------------------
//     // Camera resolution
//     // -------------------------------------------------------------

//     rgbCamera.set(
//         cv::CAP_PROP_FRAME_WIDTH,
//         1280
//     );

//     rgbCamera.set(
//         cv::CAP_PROP_FRAME_HEIGHT,
//         720
//     );


//     // =================================================================
//     // FAKE THERMAL GENERATOR
//     // =================================================================

//     generateFakeHeatmapFromImage thermalGenerator(
//         /* rgbHFovDeg       = */ 55.0,
//         /* thermalHFovDeg   = */ 55.0,
//         /* thermalRes       = */ cv::Size(16, 12),
//         /* ambientTempC     = */ 25.0f,
//         /* maxHotspotTempC  = */ 85.0f,
//         /* noiseStdDevC     = */ 0.6
//     );


//     // =================================================================
//     // CALIBRATION
//     // =================================================================

//     ChessboardCalibration calib(
//         kBoardSize
//     );


//     // =================================================================
//     // WINDOW
//     // =================================================================

//     cv::namedWindow(
//         kWindowName,
//         cv::WINDOW_AUTOSIZE
//     );


//     cv::setMouseCallback(
//         kWindowName,
//         onMouse
//     );


//     // =================================================================
//     // FRAMES
//     // =================================================================

//     cv::Mat rgbFrame;
//     cv::Mat thermalFrame;


//     bool layoutDone = false;


//     // =================================================================
//     // MAIN LOOP
//     // =================================================================

//     while (
//         rgbCamera.read(rgbFrame)
//     )
//     {
//         // -------------------------------------------------------------
//         // Make sure frame is valid
//         // -------------------------------------------------------------

//         if (
//             rgbFrame.empty()
//         )
//         {
//             continue;
//         }


//         // -------------------------------------------------------------
//         // Generate fake thermal image
//         //
//         // This is equivalent to:
//         //
//         // generatorr.getHeatmap(frame)
//         //
//         // from your main.cpp.
//         // -------------------------------------------------------------

//         thermalFrame =
//             thermalGenerator.getHeatmap(
//                 rgbFrame
//             );


//         // -------------------------------------------------------------
//         // Initialize GUI layout
//         // -------------------------------------------------------------

//         if (!layoutDone)
//         {
//             layoutUI(
//                 rgbFrame.cols,
//                 rgbFrame.rows
//             );

//             layoutDone = true;
//         }


//         // -------------------------------------------------------------
//         // After successful calibration,
//         // return to detection after 2 seconds.
//         // -------------------------------------------------------------

//         if (
//             g_state.calibrationJustSucceeded
//         )
//         {
//             auto elapsed =
//                 std::chrono::steady_clock::now() -
//                 g_state.calibrationSuccessAt;


//             if (
//                 elapsed >
//                 std::chrono::seconds(2)
//             )
//             {
//                 g_state.calibrationJustSucceeded =
//                     false;

//                 g_state.mode =
//                     UIMode::DETECT;

//                 g_state.statusMessage.clear();
//             }
//         }


//         // =================================================================
//         // CREATE CONTENT
//         // =================================================================

//         cv::Mat contentFrame;


//         // -------------------------------------------------------------
//         // DETECTION MODE
//         // -------------------------------------------------------------

//         if (
//             g_state.mode ==
//             UIMode::DETECT
//         )
//         {
//             contentFrame =
//                 runDetection(
//                     thermalFrame,
//                     rgbFrame
//                 );
//         }


//         // -------------------------------------------------------------
//         // CALIBRATION MODE
//         // -------------------------------------------------------------

//         else
//         {
//             contentFrame =
//                 rgbFrame.clone();


//             if (
//                 g_state.calibrationArmed
//             )
//             {
//                 runCalibrationAttempt(
//                     rgbFrame,
//                     thermalFrame,
//                     calib
//                 );
//             }


//             cv::putText(
//                 contentFrame,
//                 "CALIBRATION MODE",
//                 {
//                     kMargin,
//                     30
//                 },
//                 cv::FONT_HERSHEY_SIMPLEX,
//                 0.8,
//                 cv::Scalar(
//                     0,
//                     255,
//                     255
//                 ),
//                 2,
//                 cv::LINE_AA
//             );
//         }


//         // =================================================================
//         // CREATE FULL GUI CANVAS
//         // =================================================================

//         cv::Mat canvas(
//             contentFrame.rows +
//                 kPanelHeight,

//             contentFrame.cols,

//             contentFrame.type(),

//             cv::Scalar(
//                 30,
//                 30,
//                 30
//             )
//         );


//         // -------------------------------------------------------------
//         // Put image on top
//         // -------------------------------------------------------------

//         contentFrame.copyTo(
//             canvas(
//                 cv::Rect(
//                     0,
//                     0,
//                     contentFrame.cols,
//                     contentFrame.rows
//                 )
//             )
//         );


//         // =================================================================
//         // DRAW BUTTONS
//         // =================================================================

//         drawButton(
//             canvas,
//             g_state.calibrateBtn,
//             g_state.mode ==
//                 UIMode::CALIBRATE
//         );


//         drawButton(
//             canvas,
//             g_state.detectBtn,
//             g_state.mode ==
//                 UIMode::DETECT
//         );


//         // =================================================================
//         // DRAW SLIDER
//         // =================================================================

//         drawSlider(
//             canvas,
//             g_state.thresholdSlider
//         );


//         // =================================================================
//         // STATUS MESSAGE
//         // =================================================================

//         if (
//             !g_state.statusMessage.empty()
//         )
//         {
//             cv::putText(
//                 canvas,
//                 g_state.statusMessage,
//                 {
//                     kMargin,

//                     contentFrame.rows +
//                         kPanelHeight -
//                         12
//                 },
//                 cv::FONT_HERSHEY_SIMPLEX,
//                 0.5,
//                 cv::Scalar(
//                     200,
//                     200,
//                     200
//                 ),
//                 1,
//                 cv::LINE_AA
//             );
//         }


//         // =================================================================
//         // DISPLAY
//         // =================================================================

//         cv::imshow(
//             kWindowName,
//             canvas
//         );


//         // -------------------------------------------------------------
//         // Keyboard
//         // -------------------------------------------------------------

//         int key =
//             cv::waitKey(1);


//         // ESC → exit
//         if (
//             key == 27
//         )
//         {
//             break;
//         }
//     }


//     // =================================================================
//     // CLEANUP
//     // =================================================================

//     rgbCamera.release();

//     cv::destroyWindow(
//         kWindowName
//     );


//     return 0;
// }


// // =====================================================================
// // PROGRAM ENTRY POINT
// // =====================================================================

// int main()
// {
//     return runThermalInspectorGUI();
// }

// ================================================================
// GUI.cpp
//
// PC webcam
//      |
//      v
// RGB frame
//      |
//      +--------------------------+
//      |                          |
//      v                          v
// Fake thermal               RGB image
// 16 x 12                       |
//      |                         |
//      +----------+--------------+
//                 |
//                 v
//       Joint Bilateral Upsample
//                 |
//                 v
//          Hotspot Detection
//                 |
//                 v
//          Live Heatmap GUI
//
// Main menu:
//   1 -> Live Heatmap
//   2 -> Automated Calibration
//   3 -> Manual Calibration
//   Q -> Quit
//
// Inside every mode:
//   M / ESC -> Back to menu
//   Q       -> Quit
//
// Manual calibration:
//   SPACE -> Attempt calibration
// ================================================================


#include "generateFakeHeatmapFromImage.hpp"
#include "HotspotDetector.hpp"
#include "JointBilateralUpsample.hpp"
#include "ChessboardCalibration.hpp"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <iostream>
#include <string>
#include <algorithm>


// ================================================================
// GLOBAL CONFIGURATION
// ================================================================



const std::string kWindowName =
    "Thermal Inspector";

const cv::Size kBoardSize =
    cv::Size(9, 6);

const std::string kHomographyPath =
    "rgb_to_thermal_homography.yml";


// ---------------------------------------------------------------
// Temperature threshold
// ---------------------------------------------------------------

constexpr float kThresholdMinC  = 30.0f;
constexpr float kThresholdMaxC  = 120.0f;
constexpr float kThresholdInitC = 84.0f;


// ---------------------------------------------------------------
// GUI
// ---------------------------------------------------------------

constexpr int kPanelHeight = 110;

constexpr int kButtonWidth = 150;
constexpr int kButtonHeight = 44;

constexpr int kMargin = 20;


// ---------------------------------------------------------------
// Joint bilateral upsampling
// ---------------------------------------------------------------

constexpr int kWindowSize = 6;

constexpr float kSigmaSpectral = 0.05f;

constexpr double kSigmaSpatial = 8.0;


// ================================================================
// APPLICATION MODES
// ================================================================

enum class AppMode
{
    MENU,

    LIVE_HEATMAP,

    AUTOMATED_CALIBRATION,

    MANUAL_CALIBRATION
};


// ================================================================
// UI MODE
// ================================================================

enum class CalibrationStatus
{
    IDLE,

    SEARCHING,

    SUCCESS,

    FAILED
};


// ================================================================
// SLIDER
// ================================================================

struct SliderUI
{
    cv::Rect area;

    float minVal = 30.0f;
    float maxVal = 120.0f;
    float value = 84.0f;

    bool dragging = false;


    float xToValue(int x) const
    {
        float t =
            (x - area.x) /
            static_cast<float>(area.width);

        t =
            std::min(
                1.0f,
                std::max(
                    0.0f,
                    t
                )
            );

        return minVal +
               t * (maxVal - minVal);
    }


    int valueToX() const
    {
        float t =
            (value - minVal) /
            (maxVal - minVal);

        return area.x +
               static_cast<int>(
                   t * area.width
               );
    }


    bool hitTest(int x, int y) const
    {
        cv::Rect zone(
            area.x,
            area.y - 10,
            area.width,
            area.height + 20
        );

        return zone.contains(
            cv::Point(x, y)
        );
    }
};


// ================================================================
// BUTTON
// ================================================================

struct ButtonUI
{
    cv::Rect area;

    std::string label;
};


// ================================================================
// GLOBAL APPLICATION STATE
// ================================================================

struct AppState
{
    AppMode mode =
        AppMode::MENU;


    SliderUI thresholdSlider;


    ButtonUI calibrateBtn;

    ButtonUI detectBtn;


    bool calibrationArmed =
        false;


    CalibrationStatus calibrationStatus =
        CalibrationStatus::IDLE;


    std::string statusMessage;


    cv::Mat homography;
};


AppState g_state;


// ================================================================
// DRAW TEXT CENTERED
// ================================================================

void drawCenteredText(
    cv::Mat& image,
    const std::string& text,
    int y,
    double scale,
    cv::Scalar color,
    int thickness = 1
)
{
    int baseline = 0;

    cv::Size size =
        cv::getTextSize(
            text,
            cv::FONT_HERSHEY_SIMPLEX,
            scale,
            thickness,
            &baseline
        );

    int x =
        (image.cols - size.width) / 2;


    cv::putText(
        image,
        text,
        cv::Point(x, y),
        cv::FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv::LINE_AA
    );
}


// ================================================================
// MAIN MENU
// ================================================================

void drawMainMenu(
    cv::Mat& frame
)
{
    frame.setTo(
        cv::Scalar(
            25,
            25,
            25
        )
    );


    // ------------------------------------------------------------
    // Title
    // ------------------------------------------------------------

    drawCenteredText(
        frame,
        "THERMAL INSPECTOR",
        100,
        1.4,
        cv::Scalar(
            255,
            255,
            255
        ),
        2
    );


    drawCenteredText(
        frame,
        "Select operating mode",
        145,
        0.65,
        cv::Scalar(
            180,
            180,
            180
        )
    );


    // ------------------------------------------------------------
    // Menu options
    // ------------------------------------------------------------

    int boxWidth = 500;

    int boxHeight = 60;

    int x =
        (frame.cols - boxWidth) / 2;


    int startY = 190;

    int spacing = 80;


    // ------------------------------------------------------------
    // Live heatmap
    // ------------------------------------------------------------

    cv::Rect liveRect(
        x,
        startY,
        boxWidth,
        boxHeight
    );


    cv::rectangle(
        frame,
        liveRect,
        cv::Scalar(
            55,
            55,
            55
        ),
        -1,
        cv::LINE_AA
    );


    cv::rectangle(
        frame,
        liveRect,
        cv::Scalar(
            100,
            100,
            100
        ),
        1,
        cv::LINE_AA
    );


    drawCenteredText(
        frame,
        "[1]  LIVE HEATMAP",
        startY + 39,
        0.7,
        cv::Scalar(
            255,
            255,
            255
        )
    );


    // ------------------------------------------------------------
    // Automated calibration
    // ------------------------------------------------------------

    int autoY =
        startY + spacing;


    cv::Rect autoRect(
        x,
        autoY,
        boxWidth,
        boxHeight
    );


    cv::rectangle(
        frame,
        autoRect,
        cv::Scalar(
            55,
            55,
            55
        ),
        -1,
        cv::LINE_AA
    );


    cv::rectangle(
        frame,
        autoRect,
        cv::Scalar(
            100,
            100,
            100
        ),
        1,
        cv::LINE_AA
    );


    drawCenteredText(
        frame,
        "[2]  AUTOMATED CALIBRATION",
        autoY + 39,
        0.7,
        cv::Scalar(
            255,
            255,
            255
        )
    );


    // ------------------------------------------------------------
    // Manual calibration
    // ------------------------------------------------------------

    int manualY =
        startY + spacing * 2;


    cv::Rect manualRect(
        x,
        manualY,
        boxWidth,
        boxHeight
    );


    cv::rectangle(
        frame,
        manualRect,
        cv::Scalar(
            55,
            55,
            55
        ),
        -1,
        cv::LINE_AA
    );


    cv::rectangle(
        frame,
        manualRect,
        cv::Scalar(
            100,
            100,
            100
        ),
        1,
        cv::LINE_AA
    );


    drawCenteredText(
        frame,
        "[3]  MANUAL CALIBRATION",
        manualY + 39,
        0.7,
        cv::Scalar(
            255,
            255,
            255
        )
    );


    // ------------------------------------------------------------
    // Bottom instructions
    // ------------------------------------------------------------

    drawCenteredText(
        frame,
        "[Q] Quit",
        frame.rows - 45,
        0.55,
        cv::Scalar(
            160,
            160,
            160
        )
    );
}


// ================================================================
// COLOR GRADIENT
// ================================================================

void drawColorGradientBar(
    cv::Mat& canvas,
    const cv::Rect& area
)
{
    cv::Mat gradient(
        1,
        std::max(
            2,
            area.width
        ),
        CV_8UC1
    );


    for (
        int x = 0;
        x < gradient.cols;
        ++x
    )
    {
        gradient.at<uchar>(
            0,
            x
        ) =
            static_cast<uchar>(
                255.0 *
                x /
                (gradient.cols - 1)
            );
    }


    cv::Mat gradientColor;


    cv::applyColorMap(
        gradient,
        gradientColor,
        cv::COLORMAP_JET
    );


    cv::resize(
        gradientColor,
        gradientColor,
        cv::Size(
            area.width,
            area.height
        )
    );


    gradientColor.copyTo(
        canvas(area)
    );


    cv::rectangle(
        canvas,
        area,
        cv::Scalar(
            230,
            230,
            230
        ),
        1,
        cv::LINE_AA
    );
}


// ================================================================
// SLIDER
// ================================================================

void drawSlider(
    cv::Mat& canvas,
    const SliderUI& slider
)
{
    drawColorGradientBar(
        canvas,
        slider.area
    );


    int knobX =
        slider.valueToX();


    int cy =
        slider.area.y +
        slider.area.height / 2;


    cv::line(
        canvas,
        {
            knobX,
            slider.area.y - 8
        },
        {
            knobX,
            slider.area.y +
                slider.area.height +
                8
        },
        cv::Scalar(
            255,
            255,
            255
        ),
        2,
        cv::LINE_AA
    );


    cv::circle(
        canvas,
        {
            knobX,
            cy
        },
        8,
        cv::Scalar(
            20,
            20,
            20
        ),
        -1,
        cv::LINE_AA
    );


    cv::circle(
        canvas,
        {
            knobX,
            cy
        },
        8,
        cv::Scalar(
            255,
            255,
            255
        ),
        2,
        cv::LINE_AA
    );


    std::string label =
        cv::format(
            "Hotspot Threshold: %.1f C",
            slider.value
        );


    cv::putText(
        canvas,
        label,
        {
            slider.area.x,
            slider.area.y - 14
        },
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        cv::Scalar(
            255,
            255,
            255
        ),
        1,
        cv::LINE_AA
    );
}


// ================================================================
// BUTTON
// ================================================================

void drawButton(
    cv::Mat& canvas,
    const ButtonUI& btn,
    bool active
)
{
    cv::Scalar fill =
        active
        ? cv::Scalar(
            60,
            160,
            70
        )
        : cv::Scalar(
            65,
            65,
            65
        );


    cv::rectangle(
        canvas,
        btn.area,
        fill,
        -1,
        cv::LINE_AA
    );


    cv::rectangle(
        canvas,
        btn.area,
        cv::Scalar(
            255,
            255,
            255
        ),
        1,
        cv::LINE_AA
    );


    int baseline = 0;


    cv::Size textSize =
        cv::getTextSize(
            btn.label,
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            1,
            &baseline
        );


    cv::Point textOrg(
        btn.area.x +
            (
                btn.area.width -
                textSize.width
            ) / 2,

        btn.area.y +
            (
                btn.area.height +
                textSize.height
            ) / 2
    );


    cv::putText(
        canvas,
        btn.label,
        textOrg,
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(
            255,
            255,
            255
        ),
        1,
        cv::LINE_AA
    );
}


// ================================================================
// UI LAYOUT
// ================================================================

void layoutUI(
    int frameWidth,
    int frameHeight
)
{
    int panelY =
        frameHeight;


    // ------------------------------------------------------------
    // Calibrate button
    // ------------------------------------------------------------

    g_state.calibrateBtn.area =
        cv::Rect(
            kMargin,
            panelY +
                (
                    kPanelHeight -
                    kButtonHeight
                ) / 2,
            kButtonWidth,
            kButtonHeight
        );


    g_state.calibrateBtn.label =
        "CALIBRATE";


    // ------------------------------------------------------------
    // Detect button
    // ------------------------------------------------------------

    g_state.detectBtn.area =
        cv::Rect(
            kMargin * 2 +
                kButtonWidth,

            panelY +
                (
                    kPanelHeight -
                    kButtonHeight
                ) / 2,

            kButtonWidth,
            kButtonHeight
        );


    g_state.detectBtn.label =
        "DETECT";


    // ------------------------------------------------------------
    // Slider
    // ------------------------------------------------------------

    int sliderX =
        kMargin * 3 +
        kButtonWidth * 2;


    int sliderWidth =
        frameWidth -
        sliderX -
        kMargin;


    g_state.thresholdSlider.area =
        cv::Rect(
            sliderX,
            panelY +
                kPanelHeight / 2,
            std::max(
                100,
                sliderWidth
            ),
            18
        );


    g_state.thresholdSlider.minVal =
        kThresholdMinC;

    g_state.thresholdSlider.maxVal =
        kThresholdMaxC;

    g_state.thresholdSlider.value =
        kThresholdInitC;
}


// ================================================================
// MOUSE
// ================================================================

void onMouse(
    int event,
    int x,
    int y,
    int /*flags*/,
    void* /*userdata*/
)
{
    // ------------------------------------------------------------
    // Only process GUI mouse controls in live mode
    // ------------------------------------------------------------

    if (
        g_state.mode !=
        AppMode::LIVE_HEATMAP
    )
    {
        return;
    }


    // ------------------------------------------------------------
    // Button pressed
    // ------------------------------------------------------------

    if (
        event ==
        cv::EVENT_LBUTTONDOWN
    )
    {
        // --------------------------------------------------------
        // Calibrate
        // --------------------------------------------------------

        if (
            g_state.calibrateBtn.area.contains(
                cv::Point(
                    x,
                    y
                )
            )
        )
        {
            g_state.mode =
                AppMode::AUTOMATED_CALIBRATION;

            g_state.calibrationArmed =
                true;

            g_state.calibrationStatus =
                CalibrationStatus::SEARCHING;

            g_state.statusMessage =
                "Searching for chessboard...";
        }


        // --------------------------------------------------------
        // Detect
        // --------------------------------------------------------

        else if (
            g_state.detectBtn.area.contains(
                cv::Point(
                    x,
                    y
                )
            )
        )
        {
            g_state.mode =
                AppMode::LIVE_HEATMAP;

            g_state.statusMessage.clear();
        }


        // --------------------------------------------------------
        // Slider
        // --------------------------------------------------------

        else if (
            g_state.thresholdSlider.hitTest(
                x,
                y
            )
        )
        {
            g_state.thresholdSlider.dragging =
                true;

            g_state.thresholdSlider.value =
                g_state.thresholdSlider.xToValue(
                    x
                );
        }
    }


    // ------------------------------------------------------------
    // Slider movement
    // ------------------------------------------------------------

    else if (
        event ==
        cv::EVENT_MOUSEMOVE
    )
    {
        if (
            g_state.thresholdSlider.dragging
        )
        {
            g_state.thresholdSlider.value =
                g_state.thresholdSlider.xToValue(
                    x
                );
        }
    }


    // ------------------------------------------------------------
    // Slider released
    // ------------------------------------------------------------

    else if (
        event ==
        cv::EVENT_LBUTTONUP
    )
    {
        g_state.thresholdSlider.dragging =
            false;
    }
}


// ================================================================
// DETECTION
// ================================================================

cv::Mat runDetection(
    const cv::Mat& thermalFrame,
    const cv::Mat& rgbFrame
)
{
    cv::Mat thermalFloat;


    // ------------------------------------------------------------
    // Convert thermal data to float
    // ------------------------------------------------------------

    if (
        thermalFrame.type() != CV_32F
    )
    {
        thermalFrame.convertTo(
            thermalFloat,
            CV_32F
        );
    }
    else
    {
        thermalFloat =
            thermalFrame;
    }


    // ------------------------------------------------------------
    // Joint bilateral upsampling
    // ------------------------------------------------------------

    cv::Mat enhanced =
        JointBilateralUpsample::Upsample3D(
            thermalFloat,
            rgbFrame,
            GuideMode::RAW,
            kWindowSize,
            kSigmaSpectral,
            kSigmaSpatial
        );


    // ------------------------------------------------------------
    // Temperature range
    // ------------------------------------------------------------

    double minV;

    double maxV;


    cv::minMaxLoc(
        enhanced,
        &minV,
        &maxV
    );


    // ------------------------------------------------------------
    // Colorize
    // ------------------------------------------------------------

    cv::Mat display =
        JointBilateralUpsample::Colorize(
            enhanced,
            minV,
            maxV
        );


    // ------------------------------------------------------------
    // Hotspot detection
    // ------------------------------------------------------------

    HotRegion region =
        HotspotDetector::DetectThreshold(
            enhanced,
            g_state.thresholdSlider.value
        );


    // ------------------------------------------------------------
    // Draw hotspot
    // ------------------------------------------------------------

    if (
        region.boundingBox.area() > 0
    )
    {
        cv::rectangle(
            display,
            region.boundingBox,
            cv::Scalar(
                0,
                255,
                0
            ),
            2
        );


        cv::circle(
            display,
            region.peakPosition,
            4,
            cv::Scalar(
                255,
                255,
                255
            ),
            -1
        );


        cv::putText(
            display,
            cv::format(
                "%.1f C",
                region.peakTemperature
            ),
            region.peakPosition +
                cv::Point(
                    8,
                    -8
                ),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(
                255,
                255,
                255
            ),
            1,
            cv::LINE_AA
        );
    }


    // ------------------------------------------------------------
    // RGB edge overlay
    // ------------------------------------------------------------

    return JointBilateralUpsample::OverlayEdges(
        display,
        rgbFrame,
        0.05
    );
}


// ================================================================
// CALIBRATION
// ================================================================

bool attemptCalibration(
    const cv::Mat& rgbFrame,
    const cv::Mat& thermalFrame,
    ChessboardCalibration& calib
)
{
    // ------------------------------------------------------------
    // Convert thermal image to 8-bit
    // ------------------------------------------------------------

    cv::Mat thermal8u;


    if (
        thermalFrame.type() == CV_32F ||
        thermalFrame.type() == CV_64F
    )
    {
        cv::normalize(
            thermalFrame,
            thermal8u,
            0,
            255,
            cv::NORM_MINMAX,
            CV_8U
        );
    }

    else if (
        thermalFrame.channels() == 3
    )
    {
        cv::cvtColor(
            thermalFrame,
            thermal8u,
            cv::COLOR_BGR2GRAY
        );
    }

    else
    {
        thermal8u =
            thermalFrame;
    }


    // ------------------------------------------------------------
    // Detect chessboard in both images
    // ------------------------------------------------------------

    auto pair =
        calib.DetectPair(
            rgbFrame,
            thermal8u
        );


    if (!pair.success)
    {
        return false;
    }


    // ------------------------------------------------------------
    // Calculate homography
    // ------------------------------------------------------------

    cv::Mat H =
        ChessboardCalibration::
        ComputeHomography(
            pair
        );


    if (H.empty())
    {
        return false;
    }


    // ------------------------------------------------------------
    // Reprojection error
    // ------------------------------------------------------------

    double meanErr =
        ChessboardCalibration::
        ComputeMeanReprojectionError(
            H,
            pair
        );


    // ------------------------------------------------------------
    // Save
    // ------------------------------------------------------------

    ChessboardCalibration::
    SaveHomography(
        kHomographyPath,
        H
    );


    g_state.homography =
        H;


    g_state.statusMessage =
        cv::format(
            "Calibration successful - error %.2f px",
            meanErr
        );


    g_state.calibrationStatus =
        CalibrationStatus::SUCCESS;


    return true;
}


// ================================================================
// CALIBRATION DISPLAY
// ================================================================

void drawCalibrationInfo(
    cv::Mat& frame,
    bool automated
)
{
    // ------------------------------------------------------------
    // Header
    // ------------------------------------------------------------

    std::string title =
        automated
        ? "AUTOMATED CALIBRATION"
        : "MANUAL CALIBRATION";


    cv::putText(
        frame,
        title,
        cv::Point(
            20,
            40
        ),
        cv::FONT_HERSHEY_SIMPLEX,
        0.85,
        cv::Scalar(
            0,
            255,
            255
        ),
        2,
        cv::LINE_AA
    );


    // ------------------------------------------------------------
    // Instructions
    // ------------------------------------------------------------

    if (automated)
    {
        cv::putText(
            frame,
            "Place the chessboard in view",
            cv::Point(
                20,
                75
            ),
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            cv::Scalar(
                255,
                255,
                255
            ),
            1,
            cv::LINE_AA
        );


        cv::putText(
            frame,
            "System is continuously searching...",
            cv::Point(
                20,
                100
            ),
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            cv::Scalar(
                0,
                255,
                0
            ),
            1,
            cv::LINE_AA
        );
    }
    else
    {
        cv::putText(
            frame,
            "Place the chessboard in view",
            cv::Point(
                20,
                75
            ),
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            cv::Scalar(
                255,
                255,
                255
            ),
            1,
            cv::LINE_AA
        );


        cv::putText(
            frame,
            "Press SPACE to capture",
            cv::Point(
                20,
                100
            ),
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            cv::Scalar(
                0,
                255,
                255
            ),
            1,
            cv::LINE_AA
        );
    }


    // ------------------------------------------------------------
    // Back instruction
    // ------------------------------------------------------------

    cv::putText(
        frame,
        "[M / ESC] Back to menu    [Q] Quit",
        cv::Point(
            20,
            frame.rows - 20
        ),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(
            220,
            220,
            220
        ),
        1,
        cv::LINE_AA
    );


    // ------------------------------------------------------------
    // Status
    // ------------------------------------------------------------

    if (
        !g_state.statusMessage.empty()
    )
    {
        cv::putText(
            frame,
            g_state.statusMessage,
            cv::Point(
                20,
                frame.rows - 50
            ),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(
                255,
                255,
                255
            ),
            1,
            cv::LINE_AA
        );
    }
}


// ================================================================
// LIVE HEATMAP MODE
// ================================================================

void drawLiveModeInfo(
    cv::Mat& canvas
)
{
    cv::putText(
        canvas,
        "[M / ESC] Menu",
        cv::Point(
            20,
            canvas.rows - 15
        ),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(
            255,
            255,
            255
        ),
        1,
        cv::LINE_AA
    );


    cv::putText(
        canvas,
        "[Q] Quit",
        cv::Point(
            160,
            canvas.rows - 15
        ),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(
            255,
            255,
            255
        ),
        1,
        cv::LINE_AA
    );
}


// ================================================================
// MAIN
// ================================================================

int main()
{
    // ============================================================
    // OPEN PC CAMERA
    // ============================================================

    cv::VideoCapture camera(
        0
    );


    if (
        !camera.isOpened()
    )
    {
        std::cerr
            << "ERROR: Could not open PC camera.\n";

        std::cerr
            << "Try changing camera index 0 to 1 or 2.\n";

        return -1;
    }


    // ------------------------------------------------------------
    // Camera resolution
    // ------------------------------------------------------------

    camera.set(
        cv::CAP_PROP_FRAME_WIDTH,
        1280
    );


    camera.set(
        cv::CAP_PROP_FRAME_HEIGHT,
        720
    );


    // ============================================================
    // FAKE THERMAL GENERATOR
    // ============================================================

    generateFakeHeatmapFromImage thermalGenerator(
        /* RGB HFOV          */ 55.0,

        /* Thermal HFOV      */ 55.0,

        /* Thermal resolution */
        cv::Size(
            16,
            12
        ),

        /* Ambient */
        25.0f,

        /* Maximum hotspot */
        85.0f,

        /* Noise */
        0.6
    );


    // ============================================================
    // CALIBRATION OBJECT
    // ============================================================

    ChessboardCalibration calibration(
        kBoardSize
    );


    // ============================================================
    // CREATE WINDOW
    // ============================================================

    cv::namedWindow(
        kWindowName,
        cv::WINDOW_AUTOSIZE
    );


    cv::setMouseCallback(
        kWindowName,
        onMouse
    );


    // ============================================================
    // INITIAL GUI SETUP
    // ============================================================

    cv::Mat frame;


    // Read one frame to determine dimensions
    if (
        !camera.read(frame) ||
        frame.empty()
    )
    {
        std::cerr
            << "ERROR: Could not read from camera.\n";

        return -1;
    }


    layoutUI(
        frame.cols,
        frame.rows
    );


    // ============================================================
    // MAIN APPLICATION LOOP
    // ============================================================

    while (true)
    {
        // --------------------------------------------------------
        // MENU
        // --------------------------------------------------------

        if (
            g_state.mode ==
            AppMode::MENU
        )
        {
            cv::Mat menu(
                600,
                900,
                CV_8UC3,
                cv::Scalar(
                    25,
                    25,
                    25
                )
            );


            drawMainMenu(
                menu
            );


            cv::imshow(
                kWindowName,
                menu
            );


            int key =
                cv::waitKey(30);


            // ----------------------------------------------------
            // Quit
            // ----------------------------------------------------

            if (
                key == 'q' ||
                key == 'Q'
            )
            {
                break;
            }


            // ----------------------------------------------------
            // Live heatmap
            // ----------------------------------------------------

            if (
                key == '1'
            )
            {
                g_state.mode =
                    AppMode::LIVE_HEATMAP;

                g_state.statusMessage.clear();
            }


            // ----------------------------------------------------
            // Automated calibration
            // ----------------------------------------------------

            else if (
                key == '2'
            )
            {
                g_state.mode =
                    AppMode::AUTOMATED_CALIBRATION;

                g_state.calibrationStatus =
                    CalibrationStatus::SEARCHING;

                g_state.statusMessage =
                    "Searching for chessboard...";
            }


            // ----------------------------------------------------
            // Manual calibration
            // ----------------------------------------------------

            else if (
                key == '3'
            )
            {
                g_state.mode =
                    AppMode::MANUAL_CALIBRATION;

                g_state.calibrationStatus =
                    CalibrationStatus::IDLE;

                g_state.statusMessage =
                    "Press SPACE when ready.";
            }


            continue;
        }


        // ========================================================
        // READ CAMERA FRAME
        // ========================================================

        if (
            !camera.read(frame)
        )
        {
            std::cerr
                << "Camera frame read failed.\n";

            break;
        }


        if (
            frame.empty()
        )
        {
            continue;
        }


        // ========================================================
        // GENERATE FAKE THERMAL
        // ========================================================

        cv::Mat thermalFrame =
            thermalGenerator.getHeatmap(
                frame
            );


        // ========================================================
        // LIVE HEATMAP
        // ========================================================

        if (
            g_state.mode ==
            AppMode::LIVE_HEATMAP
        )
        {
            cv::Mat content =
                runDetection(
                    thermalFrame,
                    frame
                );


            // ----------------------------------------------------
            // Canvas
            // ----------------------------------------------------

            cv::Mat canvas(
                content.rows +
                    kPanelHeight,

                content.cols,

                content.type(),

                cv::Scalar(
                    30,
                    30,
                    30
                )
            );


            content.copyTo(
                canvas(
                    cv::Rect(
                        0,
                        0,
                        content.cols,
                        content.rows
                    )
                )
            );


            // ----------------------------------------------------
            // Buttons
            // ----------------------------------------------------

            drawButton(
                canvas,
                g_state.calibrateBtn,
                false
            );


            drawButton(
                canvas,
                g_state.detectBtn,
                true
            );


            // ----------------------------------------------------
            // Slider
            // ----------------------------------------------------

            drawSlider(
                canvas,
                g_state.thresholdSlider
            );


            // ----------------------------------------------------
            // Menu controls
            // ----------------------------------------------------

            drawLiveModeInfo(
                canvas
            );


            cv::imshow(
                kWindowName,
                canvas
            );


            int key =
                cv::waitKey(1);


            // ----------------------------------------------------
            // Quit
            // ----------------------------------------------------

            if (
                key == 'q' ||
                key == 'Q'
            )
            {
                break;
            }


            // ----------------------------------------------------
            // Back to menu
            // ----------------------------------------------------

            if (
                key == 'm' ||
                key == 'M' ||
                key == 27
            )
            {
                g_state.mode =
                    AppMode::MENU;

                g_state.statusMessage.clear();
            }


            continue;
        }


        // ========================================================
        // AUTOMATED CALIBRATION
        // ========================================================

        if (
            g_state.mode ==
            AppMode::AUTOMATED_CALIBRATION
        )
        {
            cv::Mat calibrationDisplay =
                frame.clone();


            // ----------------------------------------------------
            // Try calibration automatically every frame
            // ----------------------------------------------------

            bool success =
                attemptCalibration(
                    frame,
                    thermalFrame,
                    calibration
                );


            if (success)
            {
                // ------------------------------------------------
                // Show success for a moment
                // ------------------------------------------------

                cv::putText(
                    calibrationDisplay,
                    "CALIBRATION SUCCESSFUL!",
                    cv::Point(
                        20,
                        150
                    ),
                    cv::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    cv::Scalar(
                        0,
                        255,
                        0
                    ),
                    2,
                    cv::LINE_AA
                );


                drawCalibrationInfo(
                    calibrationDisplay,
                    true
                );


                cv::imshow(
                    kWindowName,
                    calibrationDisplay
                );


                int key =
                    cv::waitKey(1500);


                if (
                    key == 'q' ||
                    key == 'Q'
                )
                {
                    break;
                }


                // Return to menu after success
                g_state.mode =
                    AppMode::MENU;

                g_state.statusMessage.clear();

                continue;
            }


            // ----------------------------------------------------
            // No chessboard yet
            // ----------------------------------------------------

            drawCalibrationInfo(
                calibrationDisplay,
                true
            );


            cv::imshow(
                kWindowName,
                calibrationDisplay
            );


            int key =
                cv::waitKey(1);


            // ----------------------------------------------------
            // Quit
            // ----------------------------------------------------

            if (
                key == 'q' ||
                key == 'Q'
            )
            {
                break;
            }


            // ----------------------------------------------------
            // Back
            // ----------------------------------------------------

            if (
                key == 'm' ||
                key == 'M' ||
                key == 27
            )
            {
                g_state.mode =
                    AppMode::MENU;

                g_state.statusMessage.clear();
            }


            continue;
        }


        // ========================================================
        // MANUAL CALIBRATION
        // ========================================================

        if (
            g_state.mode ==
            AppMode::MANUAL_CALIBRATION
        )
        {
            cv::Mat calibrationDisplay =
                frame.clone();


            drawCalibrationInfo(
                calibrationDisplay,
                false
            );


            cv::imshow(
                kWindowName,
                calibrationDisplay
            );


            int key =
                cv::waitKey(1);


            // ----------------------------------------------------
            // Quit
            // ----------------------------------------------------

            if (
                key == 'q' ||
                key == 'Q'
            )
            {
                break;
            }


            // ----------------------------------------------------
            // Back
            // ----------------------------------------------------

            if (
                key == 'm' ||
                key == 'M' ||
                key == 27
            )
            {
                g_state.mode =
                    AppMode::MENU;

                g_state.statusMessage.clear();

                continue;
            }


            // ----------------------------------------------------
            // SPACE -> capture calibration
            // ----------------------------------------------------

            if (
                key == ' '
            )
            {
                g_state.statusMessage =
                    "Capturing calibration frame...";


                cv::imshow(
                    kWindowName,
                    calibrationDisplay
                );


                cv::waitKey(100);


                bool success =
                    attemptCalibration(
                        frame,
                        thermalFrame,
                        calibration
                    );


                if (success)
                {
                    g_state.statusMessage =
                        "Calibration successful!";


                    cv::Mat successFrame =
                        frame.clone();


                    cv::putText(
                        successFrame,
                        "CALIBRATION SUCCESSFUL!",
                        cv::Point(
                            20,
                            150
                        ),
                        cv::FONT_HERSHEY_SIMPLEX,
                        1.0,
                        cv::Scalar(
                            0,
                            255,
                            0
                        ),
                        2,
                        cv::LINE_AA
                    );


                    cv::putText(
                        successFrame,
                        "Saved: " +
                            kHomographyPath,
                        cv::Point(
                            20,
                            195
                        ),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.65,
                        cv::Scalar(
                            255,
                            255,
                            255
                        ),
                        1,
                        cv::LINE_AA
                    );


                    cv::imshow(
                        kWindowName,
                        successFrame
                    );


                    int successKey =
                        cv::waitKey(1500);


                    if (
                        successKey == 'q' ||
                        successKey == 'Q'
                    )
                    {
                        break;
                    }


                    // Return to menu
                    g_state.mode =
                        AppMode::MENU;

                    g_state.statusMessage.clear();
                }
                else
                {
                    g_state.statusMessage =
                        "Calibration failed. "
                        "Make sure the chessboard is visible.";
                }
            }
        }
    }


    // ============================================================
    // CLEANUP
    // ============================================================

    camera.release();

    cv::destroyAllWindows();


    return 0;
}