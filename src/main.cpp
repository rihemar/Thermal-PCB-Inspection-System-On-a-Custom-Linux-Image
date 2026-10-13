// ================================================================
// main.cpp
//
// RGB source (IRGBCamera)        Thermal source (ITCamera / generator)
//   WebcamCamera                   generateFakeHeatmapFromImage (from RGB)
//   IPWebcamCamera                 fakeThermalCamera
//   RecordedRGB                    RecordedThermal
//                                  ThermalCamera (MLX90641)
//          |                               |
//          +---------------+---------------+
//                          |
//                          v
//            PCB detection -> crop RGB to the board
//            Homography    -> re-sample thermal onto the same area
//                          |
//                          v
//                Joint Bilateral Upsample
//                          |
//                          v
//                  Hotspot Detection
//                          |
//                          v
//                   Live Heatmap GUI
//
// Usage:
//   main [<mode>] [options]          (main --help for everything)
//
// Modes:
//   menu          interactive menu (default)
//   live          live heatmap + hotspot detection
//   auto-calib    automated chessboard calibration
//   manual-calib  manual calibration: click matching points in the
//                 RGB and thermal images, then ENTER
//
// Inside every mode:
//   M / ESC -> back (menu, or quit if launched in a mode)
//   F       -> toggle fullscreen
//   Q       -> quit
//
// Structure of this file:
//   AppOptions              command line
//   draw:: / Slider / Button drawing helpers and widgets
//   ThermalSource + factories  RGB and thermal sources
//   DisplayWindow           one window: scaling, fullscreen, mouse
//   PcbRegistration         PCB crop + homography re-sampling of thermal
//   HotspotPipeline         JBU -> detection -> annotated picture
//   ManualCalibration       point pairs -> homography (no GUI)
//   View + MenuView / LiveHeatmapView /
//          AutoCalibrationView / ManualCalibrationView
//   main()                  grab frames, render view, switch views
// ================================================================


// Acquisition
#include "IRGBCamera.hpp"
#include "WebcamCamera.hpp"
#include "IPWebcamCamera.hpp"
#include "RecordedRGB.hpp"

#include "ITCamera.hpp"
#include "ThermalCamera.hpp"
#include "fakeThermalCamera.hpp"
#include "RecordedThermal.hpp"
#include "generateFakeHeatmapFromImage.hpp"

// Processing
#include "HotspotDetector.hpp"
#include "JointBilateralUpsample.hpp"
#include "ChessboardCalibration.hpp"
#include "PCBDetector.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>


// ================================================================
// GLOBAL CONSTANTS
// ================================================================

const std::string kWindowName = "Thermal Inspector";

constexpr float kThresholdMinC  = 30.0f;
constexpr float kThresholdMaxC  = 120.0f;

constexpr int kMargin = 20;

// Give up after this many consecutive failed frame grabs.
constexpr int kMaxGrabFailures = 50;

// How long a "CALIBRATION SUCCESSFUL" banner stays before leaving the mode.
constexpr auto kSuccessDisplayTime = std::chrono::milliseconds(1500);


// ================================================================
// MODES / ACTIONS
// ================================================================

enum class AppMode
{
    MENU,
    LIVE_HEATMAP,
    AUTOMATED_CALIBRATION,
    MANUAL_CALIBRATION
};


// What a view asks the main loop to do.
enum class Action
{
    STAY,
    QUIT,
    BACK,              // leave the mode (menu, or quit if launched in a mode)
    GO_MENU,
    GO_LIVE,
    GO_AUTO_CALIB,
    GO_MANUAL_CALIB
};


// Keys shared by every mode: Q quits, M / ESC go back.
Action commonKeyAction(int key)
{
    if (key == 'q' || key == 'Q')              return Action::QUIT;
    if (key == 'm' || key == 'M' || key == 27) return Action::BACK;

    return Action::STAY;
}


// ================================================================
// COMMAND LINE OPTIONS
// ================================================================

enum class RgbSourceType     { WEBCAM, IPWEBCAM, RECORDED };
enum class ThermalSourceType { FAKE_FROM_IMAGE, FAKE, RECORDED, REAL };


struct AppOptions
{
    AppMode mode         = AppMode::MENU;
    bool    explicitMode = false;   // a non-menu mode was given on the command line

    RgbSourceType     rgb     = RgbSourceType::WEBCAM;
    ThermalSourceType thermal = ThermalSourceType::FAKE_FROM_IMAGE;

    int         device = 0;
    std::string url;                // empty -> IPWebcamCamera default
    int         width  = 1280;
    int         height = 720;
    uint8_t     slaveAddr = 0x33;

    int  displayWidth  = 1024;      // max on-screen window size
    int  displayHeight = 640;
    bool fullscreen    = false;

    float       thresholdC     = 84.0f;
    cv::Size    boardSize      = cv::Size(9, 6);
    std::string homographyPath = "rgb_to_thermal_homography.yml";

    bool crop     = true;   // crop RGB + thermal to the detected PCB
    bool registr  = true;   // apply the saved homography to the thermal frame


    static void printUsage(const char* prog)
    {
        std::cout
            << "Usage: " << prog << " [<mode>] [options]\n"
            << "\n"
            << "Modes (--mode / -m, or first positional argument):\n"
            << "  menu          interactive menu (default)\n"
            << "  live          live heatmap + hotspot detection\n"
            << "  auto-calib    automated chessboard calibration\n"
            << "  manual-calib  manual point-pair calibration\n"
            << "\n"
            << "Sources:\n"
            << "  -r, --rgb <src>        webcam | ipwebcam | recorded      (default: webcam)\n"
            << "  -T, --thermal <src>    fake-image | fake | recorded | real\n"
            << "                                                          (default: fake-image)\n"
            << "  -d, --device <idx>     webcam device index              (default: 0)\n"
            << "  -u, --url <url>        IP Webcam stream URL\n"
            << "  -W, --width <px>       requested RGB capture width       (default: 1280)\n"
            << "  -H, --height <px>      requested RGB capture height      (default: 720)\n"
            << "  -a, --addr <hex>       MLX90641 I2C slave address        (default: 0x33)\n"
            << "\n"
            << "Display:\n"
            << "  -D, --display <WxH>    max on-screen size of the window  (default: 1024x640)\n"
            << "  -f, --fullscreen       start in fullscreen (F toggles at runtime)\n"
            << "\n"
            << "Other options:\n"
            << "  -t, --threshold <C>    initial hotspot threshold         (default: 84)\n"
            << "  -b, --board <CxR>      chessboard inner corners          (default: 9x6)\n"
            << "  -o, --homography <f>   homography file (saved by calibration,\n"
            << "                         loaded by live mode)\n"
            << "                         (default: rgb_to_thermal_homography.yml)\n"
            << "      --no-crop          don't crop to the detected PCB in live mode\n"
            << "      --no-register      don't apply the homography in live mode\n"
            << "  -h, --help             print this help\n"
            << "\n"
            << "Examples:\n"
            << "  " << prog << " live\n"
            << "  " << prog << " live -r ipwebcam -u http://192.168.1.20:8080/video\n"
            << "  " << prog << " live -r webcam -T real -a 0x33\n"
            << "  " << prog << " manual-calib -T fake -D 800x480 -f\n";
    }


    // Returns false when the program should stop (help printed or bad
    // arguments); exitCode is then the value main() should return.
    static bool parse(int argc, char** argv, AppOptions& opts, int& exitCode)
    {
        exitCode = 0;

        auto fail = [&](const std::string& msg) -> bool
        {
            std::cerr << "ERROR: " << msg << "\n";
            printUsage(argv[0]);
            exitCode = 1;
            return false;
        };

        bool modeSet = false;

        for (int i = 1; i < argc; ++i)
        {
            const std::string arg = argv[i];

            auto value = [&](const char*& v) -> bool
            {
                if (i + 1 >= argc) return fail(arg + " needs a value.");
                v = argv[++i];
                return true;
            };

            const char* v = nullptr;

            if (arg == "-h" || arg == "--help")
            {
                printUsage(argv[0]);
                return false;
            }
            else if (arg == "-m" || arg == "--mode")
            {
                if (!value(v)) return false;
                if (!parseMode(v, opts.mode)) return fail("unknown mode '" + std::string(v) + "'.");
                modeSet = true;
            }
            else if (arg == "-r" || arg == "--rgb")
            {
                if (!value(v)) return false;
                if (!parseRgb(v, opts.rgb)) return fail("unknown RGB source '" + std::string(v) + "'.");
            }
            else if (arg == "-T" || arg == "--thermal")
            {
                if (!value(v)) return false;
                if (!parseThermal(v, opts.thermal)) return fail("unknown thermal source '" + std::string(v) + "'.");
            }
            else if (arg == "-d" || arg == "--device")
            {
                if (!value(v)) return false;
                opts.device = std::atoi(v);
            }
            else if (arg == "-u" || arg == "--url")
            {
                if (!value(v)) return false;
                opts.url = v;
            }
            else if (arg == "-W" || arg == "--width")
            {
                if (!value(v)) return false;
                opts.width = std::atoi(v);
            }
            else if (arg == "-H" || arg == "--height")
            {
                if (!value(v)) return false;
                opts.height = std::atoi(v);
            }
            else if (arg == "-a" || arg == "--addr")
            {
                if (!value(v)) return false;
                const long parsed = std::strtol(v, nullptr, 0);   // "0x33" or "51"
                if (parsed < 0 || parsed > 0x7F) return fail("bad I2C address '" + std::string(v) + "'.");
                opts.slaveAddr = static_cast<uint8_t>(parsed);
            }
            else if (arg == "-D" || arg == "--display")
            {
                if (!value(v)) return false;
                cv::Size d;
                if (!parseSize(v, d) || d.width < 320 || d.height < 240)
                    return fail("bad display size '" + std::string(v) + "', expected e.g. 1024x640.");
                opts.displayWidth  = d.width;
                opts.displayHeight = d.height;
            }
            else if (arg == "-f" || arg == "--fullscreen")
            {
                opts.fullscreen = true;
            }
            else if (arg == "-t" || arg == "--threshold")
            {
                if (!value(v)) return false;
                opts.thresholdC = static_cast<float>(std::atof(v));
            }
            else if (arg == "-b" || arg == "--board")
            {
                if (!value(v)) return false;
                if (!parseSize(v, opts.boardSize)) return fail("bad board size '" + std::string(v) + "', expected e.g. 9x6.");
            }
            else if (arg == "-o" || arg == "--homography")
            {
                if (!value(v)) return false;
                opts.homographyPath = v;
            }
            else if (arg == "--no-crop")
            {
                opts.crop = false;
            }
            else if (arg == "--no-register")
            {
                opts.registr = false;
            }
            else if (!arg.empty() && arg[0] != '-' && !modeSet)
            {
                if (!parseMode(arg, opts.mode)) return fail("unknown mode '" + arg + "'.");
                modeSet = true;
            }
            else
            {
                return fail("unknown option '" + arg + "'.");
            }
        }

        opts.explicitMode = (opts.mode != AppMode::MENU);

        return true;
    }

private:
    static std::string toLower(std::string s)
    {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        return s;
    }

    static bool parseMode(const std::string& text, AppMode& out)
    {
        const std::string m = toLower(text);

        if (m == "menu")                                      { out = AppMode::MENU;                  return true; }
        if (m == "live" || m == "heatmap" || m == "1")        { out = AppMode::LIVE_HEATMAP;          return true; }
        if (m == "auto" || m == "auto-calib" || m == "2")     { out = AppMode::AUTOMATED_CALIBRATION; return true; }
        if (m == "manual" || m == "manual-calib" || m == "3") { out = AppMode::MANUAL_CALIBRATION;    return true; }

        return false;
    }

    static bool parseRgb(const std::string& text, RgbSourceType& out)
    {
        const std::string s = toLower(text);

        if (s == "webcam" || s == "usb")    { out = RgbSourceType::WEBCAM;   return true; }
        if (s == "ipwebcam" || s == "ip")   { out = RgbSourceType::IPWEBCAM; return true; }
        if (s == "recorded" || s == "file") { out = RgbSourceType::RECORDED; return true; }

        return false;
    }

    static bool parseThermal(const std::string& text, ThermalSourceType& out)
    {
        const std::string s = toLower(text);

        if (s == "fake-image" || s == "image") { out = ThermalSourceType::FAKE_FROM_IMAGE; return true; }
        if (s == "fake" || s == "random")      { out = ThermalSourceType::FAKE;            return true; }
        if (s == "recorded" || s == "file")    { out = ThermalSourceType::RECORDED;        return true; }
        if (s == "real" || s == "mlx")         { out = ThermalSourceType::REAL;            return true; }

        return false;
    }

    // "WxH", "WXH" or "W,H"
    static bool parseSize(const std::string& text, cv::Size& out)
    {
        const size_t sep = text.find_first_of("xX,");

        if (sep == std::string::npos) return false;

        try
        {
            const int a = std::stoi(text.substr(0, sep));
            const int b = std::stoi(text.substr(sep + 1));

            if (a < 2 || b < 2) return false;

            out = cv::Size(a, b);
            return true;
        }
        catch (...)
        {
            return false;
        }
    }
};


// ================================================================
// DRAWING HELPERS
// ================================================================

namespace draw {

const cv::Scalar kWhite  (255, 255, 255);
const cv::Scalar kGrey   (180, 180, 180);
const cv::Scalar kDim    (140, 140, 140);
const cv::Scalar kYellow (0, 255, 255);
const cv::Scalar kGreen  (0, 255, 0);
const cv::Scalar kPanelBg(30, 30, 30);


void text(cv::Mat& img, const std::string& s, cv::Point org,
          double scale = 0.5, cv::Scalar color = kWhite, int thickness = 1)
{
    cv::putText(img, s, org, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA);
}


void centeredText(cv::Mat& img, const std::string& s, int y,
                  double scale, cv::Scalar color, int thickness = 1)
{
    int baseline = 0;
    const cv::Size size = cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline);

    text(img, s, cv::Point((img.cols - size.width) / 2, y), scale, color, thickness);
}


// Big centered banner (e.g. "CALIBRATION SUCCESSFUL!").
void banner(cv::Mat& img, const std::string& s, cv::Scalar color = kGreen)
{
    const double scale = std::max(0.6, std::min(1.2, img.cols / 900.0));

    centeredText(img, s, img.rows / 2, scale, color, 2);
}


// JET gradient bar with a thin border.
void colorGradientBar(cv::Mat& canvas, const cv::Rect& area)
{
    cv::Mat gradient(1, std::max(2, area.width), CV_8UC1);

    for (int x = 0; x < gradient.cols; ++x)
    {
        gradient.at<uchar>(0, x) = static_cast<uchar>(255.0 * x / (gradient.cols - 1));
    }

    cv::Mat color;
    cv::applyColorMap(gradient, color, cv::COLORMAP_JET);
    cv::resize(color, color, cv::Size(area.width, area.height));

    color.copyTo(canvas(area));

    cv::rectangle(canvas, area, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
}


// Raw temperature map -> BGR picture of the given size. Nearest
// neighbour keeps the sensor pixels visible for hand-picking points.
cv::Mat colorizeThermal(const cv::Mat& thermal, cv::Size size, int interpolation = cv::INTER_NEAREST)
{
    cv::Mat u8;

    if (thermal.type() == CV_32F || thermal.type() == CV_64F)
    {
        cv::normalize(thermal, u8, 0, 255, cv::NORM_MINMAX, CV_8U);
    }
    else if (thermal.channels() == 3)
    {
        cv::cvtColor(thermal, u8, cv::COLOR_BGR2GRAY);
    }
    else
    {
        u8 = thermal;
    }

    cv::Mat color;
    cv::applyColorMap(u8, color, cv::COLORMAP_JET);

    if (color.size() != size)
    {
        cv::resize(color, color, size, 0, 0, interpolation);
    }

    return color;
}

} // namespace draw


// ================================================================
// WIDGETS
// ================================================================

struct Slider
{
    cv::Rect    area;
    float       minVal = 0.0f;
    float       maxVal = 1.0f;
    float       value  = 0.0f;
    std::string label;          // printed as "<label>: <value> C"
    bool        dragging = false;


    // Returns true when the event was consumed.
    bool onMouse(int event, int x, int y)
    {
        if (event == cv::EVENT_LBUTTONDOWN && hit(x, y))
        {
            dragging = true;
            value    = xToValue(x);
            return true;
        }

        if (event == cv::EVENT_MOUSEMOVE && dragging)
        {
            value = xToValue(x);
            return true;
        }

        if (event == cv::EVENT_LBUTTONUP && dragging)
        {
            dragging = false;
            return true;
        }

        return false;
    }


    void draw(cv::Mat& canvas) const
    {
        draw::colorGradientBar(canvas, area);

        const int knobX = valueToX();
        const int cy    = area.y + area.height / 2;

        cv::line(canvas, { knobX, area.y - 8 }, { knobX, area.y + area.height + 8 },
                 draw::kWhite, 2, cv::LINE_AA);

        cv::circle(canvas, { knobX, cy }, 8, cv::Scalar(20, 20, 20), -1, cv::LINE_AA);
        cv::circle(canvas, { knobX, cy }, 8, draw::kWhite, 2, cv::LINE_AA);

        draw::text(canvas, cv::format("%s: %.1f C", label.c_str(), value),
                   { area.x, area.y - 14 }, 0.55);
    }

private:
    float xToValue(int x) const
    {
        float t = (x - area.x) / static_cast<float>(area.width);
        t = std::min(1.0f, std::max(0.0f, t));
        return minVal + t * (maxVal - minVal);
    }

    int valueToX() const
    {
        const float t = (value - minVal) / (maxVal - minVal);
        return area.x + static_cast<int>(t * area.width);
    }

    bool hit(int x, int y) const
    {
        return cv::Rect(area.x, area.y - 10, area.width, area.height + 20).contains(cv::Point(x, y));
    }
};


struct Button
{
    cv::Rect    area;
    std::string label;


    bool contains(int x, int y) const
    {
        return area.contains(cv::Point(x, y));
    }

    void draw(cv::Mat& canvas, bool active) const
    {
        const cv::Scalar fill = active ? cv::Scalar(60, 160, 70) : cv::Scalar(65, 65, 65);

        cv::rectangle(canvas, area, fill, -1, cv::LINE_AA);
        cv::rectangle(canvas, area, draw::kWhite, 1, cv::LINE_AA);

        int baseline = 0;
        const cv::Size size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseline);

        draw::text(canvas, label,
                   { area.x + (area.width - size.width) / 2, area.y + (area.height + size.height) / 2 },
                   0.6);
    }
};


// ================================================================
// THERMAL SOURCE
//
// Unifies the ITCamera implementations (which produce frames on their
// own) with generateFakeHeatmapFromImage (which derives a frame from
// the current RGB image).
// ================================================================

class ThermalSource
{
public:
    virtual ~ThermalSource() = default;

    virtual bool initialize() = 0;

    // rgbFrame is this iteration's RGB frame; real sensors ignore it.
    virtual bool grab(const cv::Mat& rgbFrame, cv::Mat& thermalOut) = 0;

    virtual std::string name() const = 0;
};


class CameraThermalSource : public ThermalSource
{
public:
    CameraThermalSource(std::unique_ptr<ITCamera> cam, std::string name)
        : cam_(std::move(cam)), name_(std::move(name))
    {
    }

    bool initialize() override
    {
        return cam_->initialize();
    }

    bool grab(const cv::Mat& /*rgbFrame*/, cv::Mat& thermalOut) override
    {
        return cam_->captureFrame(thermalOut) && !thermalOut.empty();
    }

    std::string name() const override
    {
        return name_;
    }

private:
    std::unique_ptr<ITCamera> cam_;
    std::string name_;
};


class ImageDerivedThermalSource : public ThermalSource
{
public:
    ImageDerivedThermalSource()
        : generator_(
              /* rgbHFovDeg      */ 55.0,
              /* thermalHFovDeg  */ 55.0,
              /* thermalRes      */ cv::Size(16, 12),
              /* ambientTempC    */ 25.0f,
              /* maxHotspotTempC */ 85.0f,
              /* noiseStdDevC    */ 0.6
          )
    {
    }

    bool initialize() override
    {
        return true;
    }

    bool grab(const cv::Mat& rgbFrame, cv::Mat& thermalOut) override
    {
        if (rgbFrame.empty()) return false;

        thermalOut = generator_.getHeatmap(rgbFrame);

        return !thermalOut.empty();
    }

    std::string name() const override
    {
        return "generateFakeHeatmapFromImage";
    }

private:
    generateFakeHeatmapFromImage generator_;
};


std::unique_ptr<IRGBCamera> makeRgbSource(const AppOptions& opts, std::string& name)
{
    switch (opts.rgb)
    {
        case RgbSourceType::WEBCAM:
            name = "WebcamCamera(device " + std::to_string(opts.device) + ")";
            return std::make_unique<WebcamCamera>(opts.device, opts.width, opts.height);

        case RgbSourceType::IPWEBCAM:
            if (opts.url.empty())
            {
                name = "IPWebcamCamera(default url)";
                return std::make_unique<IPWebcamCamera>();
            }
            name = "IPWebcamCamera(" + opts.url + ")";
            return std::make_unique<IPWebcamCamera>(opts.url, opts.width, opts.height);

        case RgbSourceType::RECORDED:
            name = "RecordedRGB";
            return std::make_unique<RecordedRGB>(opts.width, opts.height);
    }

    return nullptr;
}


std::unique_ptr<ThermalSource> makeThermalSource(const AppOptions& opts)
{
    switch (opts.thermal)
    {
        case ThermalSourceType::FAKE_FROM_IMAGE:
            return std::make_unique<ImageDerivedThermalSource>();

        case ThermalSourceType::FAKE:
            return std::make_unique<CameraThermalSource>(
                std::make_unique<fakeThermalCamera>(), "fakeThermalCamera");

        case ThermalSourceType::RECORDED:
            return std::make_unique<CameraThermalSource>(
                std::make_unique<RecordedThermal>(), "RecordedThermal");

        case ThermalSourceType::REAL:
#ifdef DEVICE_RASPBERRY_PI
            return std::make_unique<CameraThermalSource>(
                std::make_unique<ThermalCamera>(opts.slaveAddr),
                cv::format("ThermalCamera(MLX90641 @ 0x%02X)", opts.slaveAddr));
#else
            std::cerr << "ERROR: thermal source 'real' (MLX90641) is only "
                         "available when built with -DDEVICE=RASPBERRY_PI.\n";
            return nullptr;
#endif
    }

    return nullptr;
}


// ================================================================
// VIEW INTERFACE
//
// One screen of the application. The main loop:
//   1. grabs frames (if needsFrames())
//   2. canvas = render(rgb, thermal)
//   3. shows the canvas
//   4. action = onKey(waitKey(frameDelayMs()))
// Mouse events are forwarded by DisplayWindow in canvas coordinates.
// ================================================================

class View
{
public:
    virtual ~View() = default;

    virtual void onEnter() {}

    virtual bool needsFrames() const { return true; }

    virtual int frameDelayMs() const { return 1; }

    virtual cv::Mat render(const cv::Mat& rgb, const cv::Mat& thermal) = 0;

    virtual void onMouse(int /*event*/, int /*x*/, int /*y*/) {}

    virtual Action onKey(int key) = 0;
};


// ================================================================
// DISPLAY WINDOW
//
//  - scales every canvas down to fit a maximum display size
//  - fullscreen (window must be WINDOW_NORMAL for that to work)
//  - forwards mouse events to the active View in canvas coordinates
// ================================================================

class DisplayWindow
{
public:
    DisplayWindow(const std::string& name, cv::Size maxDisplay, bool fullscreen)
        : name_(name), maxDisplay_(maxDisplay)
    {
        cv::namedWindow(name_, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_NORMAL);
        cv::resizeWindow(name_, maxDisplay_.width, maxDisplay_.height);
        cv::setMouseCallback(name_, &DisplayWindow::mouseThunk, this);

        if (fullscreen) setFullscreen(true);
    }

    ~DisplayWindow()
    {
        cv::setMouseCallback(name_, nullptr, nullptr);
        cv::destroyWindow(name_);
    }

    // The view that receives mouse events (may be nullptr).
    void setView(View* view)
    {
        view_ = view;
    }

    cv::Size maxDisplay() const
    {
        return maxDisplay_;
    }

    // Scales the canvas to fit, shows it, resizes the window if needed.
    void show(const cv::Mat& canvas)
    {
        const double sx = static_cast<double>(maxDisplay_.width)  / canvas.cols;
        const double sy = static_cast<double>(maxDisplay_.height) / canvas.rows;
        const double scale = std::min(1.0, std::min(sx, sy));

        cv::Mat shown;

        if (scale < 0.999)
        {
            cv::resize(canvas, shown, cv::Size(), scale, scale, cv::INTER_AREA);
        }
        else
        {
            shown = canvas;
        }

        scale_ = static_cast<double>(shown.cols) / canvas.cols;

        if (shown.size() != lastShown_)
        {
            lastShown_ = shown.size();

            if (!fullscreen_) cv::resizeWindow(name_, shown.cols, shown.rows);
        }

        cv::imshow(name_, shown);
    }

    void setFullscreen(bool on)
    {
        fullscreen_ = on;

        cv::setWindowProperty(name_, cv::WND_PROP_FULLSCREEN,
                              on ? cv::WINDOW_FULLSCREEN : cv::WINDOW_NORMAL);

        if (!on && lastShown_.width > 0)
        {
            cv::resizeWindow(name_, lastShown_.width, lastShown_.height);
        }
    }

    // Keys that belong to the window itself (F = fullscreen).
    // Returns true when the key was consumed.
    bool handleKey(int key)
    {
        if (key == 'f' || key == 'F')
        {
            setFullscreen(!fullscreen_);
            return true;
        }

        return false;
    }

private:
    static void mouseThunk(int event, int x, int y, int /*flags*/, void* userdata)
    {
        auto* self = static_cast<DisplayWindow*>(userdata);

        if (!self || !self->view_) return;

        // Mouse coordinates arrive in the scaled image; views work in
        // canvas coordinates.
        if (self->scale_ > 0.0 && self->scale_ < 1.0)
        {
            x = static_cast<int>(std::lround(x / self->scale_));
            y = static_cast<int>(std::lround(y / self->scale_));
        }

        self->view_->onMouse(event, x, y);
    }

    std::string name_;
    cv::Size    maxDisplay_;
    double      scale_ = 1.0;
    cv::Size    lastShown_;
    bool        fullscreen_ = false;
    View*       view_ = nullptr;
};


// ================================================================
// PCB REGISTRATION + CROP
//
// Prepares the (rgb, thermal) pair that goes into the fusion:
//
//   1. PCBDetector finds the PCB quad in the RGB frame; its bounding
//      rect is the crop. If nothing is found the last good rect is
//      reused for a while, then the full frame.
//   2. The thermal frame is re-sampled so that it covers exactly the
//      same physical area as the cropped RGB, using the saved RGB ->
//      thermal homography. Without a homography the two frames are
//      assumed to have the same field of view (plain scaling).
//
// The thermal output keeps the sensor's native sampling density, so
// the joint bilateral upsampler still sees "low-res thermal + hi-res
// guide" like before, just aligned and cropped.
// ================================================================

class PcbRegistration
{
public:
    struct Result
    {
        cv::Mat  rgb;              // cropped RGB
        cv::Mat  thermal;          // thermal re-sampled onto the same area (CV_32F)
        cv::Rect rect;             // crop rectangle in the full RGB frame
        bool     pcbFound  = false;
        bool     registered = false;   // homography was applied
    };

    PcbRegistration(bool crop, bool registr)
        : cropEnabled(crop), registerEnabled(registr)
    {
    }

    bool cropEnabled;
    bool registerEnabled;

    bool hasHomography() const { return !H_.empty(); }

    // Reads the homography written by ChessboardCalibration::SaveHomography /
    // the manual calibration. Any 3x3 matrix in the file is accepted,
    // whatever its key is called.
    bool loadHomography(const std::string& path)
    {
        H_.release();

        cv::FileStorage fs(path, cv::FileStorage::READ);

        if (!fs.isOpened()) return false;

        for (const cv::FileNode& node : fs.root())
        {
            if (!node.isMap()) continue;

            cv::Mat m;
            node >> m;

            if (m.rows == 3 && m.cols == 3)
            {
                m.convertTo(H_, CV_64F);
                return true;
            }
        }

        return false;
    }

    Result prepare(const cv::Mat& rgb, const cv::Mat& thermal)
    {
        Result out;

        // ---- 1. crop rectangle ------------------------------------

        out.rect = cv::Rect(0, 0, rgb.cols, rgb.rows);

        if (cropEnabled)
        {
            const cv::Rect found = detectPcbRect(rgb);

            if (found.area() > 0)
            {
                lastRect_    = found;
                lastRectAge_ = 0;
                out.pcbFound = true;
            }
            else if (lastRect_.area() > 0 && lastRectAge_ < kKeepLastRectFrames)
            {
                ++lastRectAge_;
                out.pcbFound = true;
            }
            else
            {
                lastRect_ = cv::Rect();
            }

            if (out.pcbFound) out.rect = lastRect_;
        }

        out.rgb = rgb(out.rect);

        // ---- 2. thermal onto the same area ------------------------

        cv::Mat thermal32f;
        if (thermal.type() != CV_32F) thermal.convertTo(thermal32f, CV_32F);
        else                          thermal32f = thermal;

        // RGB pixel -> thermal pixel. Plain scaling when not registering.
        cv::Mat H;

        if (registerEnabled && hasHomography())
        {
            H = H_;
            out.registered = true;
        }
        else
        {
            H = (cv::Mat_<double>(3, 3) <<
                 static_cast<double>(thermal32f.cols) / rgb.cols, 0, 0,
                 0, static_cast<double>(thermal32f.rows) / rgb.rows, 0,
                 0, 0, 1);
        }

        // Keep the thermal crop at the sensor's own sampling density:
        // thermal pixels per RGB pixel, from the homography's scale.
        const double s = std::sqrt(std::abs(
            (H.at<double>(0, 0) * H.at<double>(1, 1) - H.at<double>(0, 1) * H.at<double>(1, 0))
            / (H.at<double>(2, 2) * H.at<double>(2, 2))));

        const int tw = std::max(2, static_cast<int>(std::lround(out.rect.width  * s)));
        const int th = std::max(2, static_cast<int>(std::lround(out.rect.height * s)));

        // thermal-crop pixel (u, v) -> RGB pixel (rect.x + u/s, rect.y + v/s) -> thermal pixel
        const cv::Mat T = (cv::Mat_<double>(3, 3) <<
                           1.0 / s, 0,       out.rect.x,
                           0,       1.0 / s, out.rect.y,
                           0,       0,       1);

        const cv::Mat M = H * T;   // destination -> source

        cv::warpPerspective(
            thermal32f, out.thermal, M, cv::Size(tw, th),
            cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_REPLICATE);

        return out;
    }

private:
    static constexpr int    kKeepLastRectFrames = 30;
    static constexpr double kMinPcbAreaFraction = 0.05;   // reject tiny quads

    cv::Rect detectPcbRect(const cv::Mat& rgb) const
    {
        const std::vector<cv::Point> quad = detector_.DetectPCB(rgb);

        if (quad.size() != 4) return cv::Rect();

        cv::Rect r = cv::boundingRect(quad) & cv::Rect(0, 0, rgb.cols, rgb.rows);

        if (r.area() < kMinPcbAreaFraction * rgb.cols * rgb.rows) return cv::Rect();

        return r;
    }

    PCBDetector detector_;
    cv::Mat     H_;            // RGB -> thermal, CV_64F
    cv::Rect    lastRect_;
    int         lastRectAge_ = 0;
};


// ================================================================
// HOTSPOT PIPELINE
//
// low-res thermal + RGB guide -> joint bilateral upsampling ->
// threshold detection -> colorized picture with hotspot + RGB edges
// ================================================================

struct HotspotResult
{
    cv::Mat   enhanced;         // upsampled temperature map, CV_32F
    cv::Mat   display;          // colorized BGR picture with annotations
    HotRegion region;
    bool      found = false;
};


class HotspotPipeline
{
public:
    explicit HotspotPipeline(int windowSize = 6, float sigmaSpectral = 0.05f,
                             double sigmaSpatial = 8.0, double edgeAlpha = 0.05)
        : windowSize_(windowSize), sigmaSpectral_(sigmaSpectral),
          sigmaSpatial_(sigmaSpatial), edgeAlpha_(edgeAlpha)
    {
    }

    HotspotResult run(const cv::Mat& thermal, const cv::Mat& rgb, float thresholdC) const
    {
        HotspotResult out;

        cv::Mat thermal32f;

        if (thermal.type() != CV_32F) thermal.convertTo(thermal32f, CV_32F);
        else                          thermal32f = thermal;

        out.enhanced = JointBilateralUpsample::Upsample3D(
            thermal32f, rgb, GuideMode::RAW, windowSize_, sigmaSpectral_, sigmaSpatial_);

        double minV, maxV;
        cv::minMaxLoc(out.enhanced, &minV, &maxV);

        cv::Mat display = JointBilateralUpsample::Colorize(out.enhanced, minV, maxV);

        out.region = HotspotDetector::DetectThreshold(out.enhanced, thresholdC);
        out.found  = out.region.boundingBox.area() > 0;

        if (out.found)
        {
            cv::rectangle(display, out.region.boundingBox, draw::kGreen, 2);
            cv::circle(display, out.region.peakPosition, 4, draw::kWhite, -1);
            draw::text(display, cv::format("%.1f C", out.region.peakTemperature),
                       out.region.peakPosition + cv::Point(8, -8));
        }

        out.display = JointBilateralUpsample::OverlayEdges(display, rgb, edgeAlpha_);

        return out;
    }

private:
    int    windowSize_;
    float  sigmaSpectral_;
    double sigmaSpatial_;
    double edgeAlpha_;
};


// ================================================================
// MANUAL CALIBRATION
//
// The user picks corresponding points by hand: point N in the RGB
// frame matches point N in the thermal frame. Once enough pairs exist
// an RGB -> thermal homography is fitted.
//
// No GUI code here: the view feeds in points (RGB in RGB pixel
// coordinates, thermal in native sensor pixel coordinates) and asks
// for the homography.
// ================================================================

class ManualCalibration
{
public:
    enum class Side { RGB, THERMAL };

    explicit ManualCalibration(std::size_t minPairs = 4)
        : minPairs_(std::max<std::size_t>(4, minPairs))
    {
    }

    void reset()
    {
        rgb_.clear();
        thermal_.clear();
        history_.clear();
    }

    // Adds a point to one side. A side may only run one point ahead of
    // the other, so the two lists always correspond index by index.
    // Returns false when the point was refused.
    bool addPoint(Side side, cv::Point2f p)
    {
        std::vector<cv::Point2f>& mine  = (side == Side::RGB) ? rgb_ : thermal_;
        std::vector<cv::Point2f>& other = (side == Side::RGB) ? thermal_ : rgb_;

        if (mine.size() > other.size()) return false;

        mine.push_back(p);
        history_.push_back(side);

        return true;
    }

    // Removes the most recently added point.
    bool undo()
    {
        if (history_.empty()) return false;

        const Side last = history_.back();
        history_.pop_back();

        if (last == Side::RGB) rgb_.pop_back();
        else                   thermal_.pop_back();

        return true;
    }

    // The side that should receive the next point.
    Side nextSide() const
    {
        return (thermal_.size() < rgb_.size()) ? Side::THERMAL : Side::RGB;
    }

    std::size_t count(Side side) const
    {
        return (side == Side::RGB) ? rgb_.size() : thermal_.size();
    }

    std::size_t pairCount() const
    {
        return std::min(rgb_.size(), thermal_.size());
    }

    std::size_t minPairs() const { return minPairs_; }

    bool ready() const { return pairCount() >= minPairs_; }

    const std::vector<cv::Point2f>& points(Side side) const
    {
        return (side == Side::RGB) ? rgb_ : thermal_;
    }

    // Fits the RGB -> thermal homography from the complete pairs.
    // Returns an empty matrix on failure. meanReprojErrPx (optional)
    // receives the mean distance in thermal pixels between projected
    // RGB points and the picked thermal points.
    cv::Mat computeHomography(double* meanReprojErrPx = nullptr) const
    {
        const std::size_t n = pairCount();

        if (n < 4) return cv::Mat();

        const std::vector<cv::Point2f> src(rgb_.begin(),     rgb_.begin()     + n);
        const std::vector<cv::Point2f> dst(thermal_.begin(), thermal_.begin() + n);

        // Few hand-picked points: plain least squares. RANSAC would
        // happily discard half of 4-6 points.
        cv::Mat H = cv::findHomography(src, dst, 0);

        if (H.empty()) return cv::Mat();

        if (meanReprojErrPx)
        {
            std::vector<cv::Point2f> projected;
            cv::perspectiveTransform(src, projected, H);

            double sum = 0.0;
            for (std::size_t i = 0; i < n; ++i) sum += cv::norm(projected[i] - dst[i]);

            *meanReprojErrPx = sum / static_cast<double>(n);
        }

        return H;
    }

private:
    std::size_t minPairs_;

    std::vector<cv::Point2f> rgb_;
    std::vector<cv::Point2f> thermal_;
    std::vector<Side>        history_;   // order of insertion, for undo()
};


// ================================================================
// VIEW: MENU
// ================================================================

class MenuView : public View
{
public:
    MenuView(cv::Size canvasSize, std::string sourcesLine)
        : canvasSize_(canvasSize), sourcesLine_(std::move(sourcesLine))
    {
    }

    bool needsFrames() const override { return false; }

    int frameDelayMs() const override { return 30; }

    cv::Mat render(const cv::Mat& /*rgb*/, const cv::Mat& /*thermal*/) override
    {
        cv::Mat frame(canvasSize_, CV_8UC3, cv::Scalar(25, 25, 25));

        const int h = frame.rows;
        const int w = frame.cols;

        // Relative to the canvas size so the menu fits any display.
        const double ts = std::max(0.45, std::min(1.0, h / 600.0));

        const int titleY    = static_cast<int>(h * 0.17);
        const int subtitleY = titleY + static_cast<int>(45 * ts);

        draw::centeredText(frame, "THERMAL INSPECTOR", titleY, 1.4 * ts, draw::kWhite, 2);
        draw::centeredText(frame, "Select operating mode", subtitleY, 0.65 * ts, draw::kGrey);

        const int spacing   = static_cast<int>(h * 0.135);
        const int boxHeight = static_cast<int>(spacing * 0.75);
        const int boxWidth  = std::min(500, w - 2 * kMargin);
        const int x         = (w - boxWidth) / 2;
        const int startY    = subtitleY + static_cast<int>(h * 0.07);

        const char* labels[3] = {
            "[1]  LIVE HEATMAP",
            "[2]  AUTOMATED CALIBRATION (chessboard)",
            "[3]  MANUAL CALIBRATION (pick points)"
        };

        for (int i = 0; i < 3; ++i)
        {
            const cv::Rect rect(x, startY + spacing * i, boxWidth, boxHeight);

            cv::rectangle(frame, rect, cv::Scalar(55, 55, 55), -1, cv::LINE_AA);
            cv::rectangle(frame, rect, cv::Scalar(100, 100, 100), 1, cv::LINE_AA);

            draw::centeredText(frame, labels[i],
                               rect.y + rect.height / 2 + static_cast<int>(9 * ts),
                               0.7 * ts, draw::kWhite);
        }

        draw::centeredText(frame, sourcesLine_, h - static_cast<int>(70 * ts), 0.5 * ts, draw::kDim);
        draw::centeredText(frame, "[Q] Quit    [F] Fullscreen", h - static_cast<int>(40 * ts), 0.55 * ts, draw::kGrey);

        return frame;
    }

    Action onKey(int key) override
    {
        switch (key)
        {
            case 'q': case 'Q': return Action::QUIT;
            case '1':           return Action::GO_LIVE;
            case '2':           return Action::GO_AUTO_CALIB;
            case '3':           return Action::GO_MANUAL_CALIB;
            default:            return Action::STAY;
        }
    }

private:
    cv::Size    canvasSize_;
    std::string sourcesLine_;
};


// ================================================================
// VIEW: LIVE HEATMAP
// ================================================================

class LiveHeatmapView : public View
{
public:
    LiveHeatmapView(PcbRegistration& registration, const HotspotPipeline& pipeline,
                    std::string homographyPath, float initialThresholdC)
        : registration_(registration), pipeline_(pipeline), homographyPath_(std::move(homographyPath))
    {
        slider_.minVal = kThresholdMinC;
        slider_.maxVal = kThresholdMaxC;
        slider_.value  = std::min(kThresholdMaxC, std::max(kThresholdMinC, initialThresholdC));
        slider_.label  = "Hotspot Threshold";

        calibrateBtn_.label = "CALIBRATE";
        detectBtn_.label    = "DETECT";
    }

    // Picks up a homography written by a calibration run just before.
    void onEnter() override
    {
        registration_.loadHomography(homographyPath_);
    }

    cv::Mat render(const cv::Mat& rgb, const cv::Mat& thermal) override
    {
        // The panel is laid out for the full frame width so the controls
        // don't jump around when the PCB crop changes size.
        if (!layoutDone_) layout(rgb.cols, rgb.rows);

        const PcbRegistration::Result prep = registration_.prepare(rgb, thermal);

        const HotspotResult result = pipeline_.run(prep.thermal, prep.rgb, slider_.value);

        // Fused picture goes back at the PCB's place inside the full frame,
        // so the operator still sees where the board is.
        cv::Mat content(rgb.rows, rgb.cols, CV_8UC3, cv::Scalar(15, 15, 15));

        result.display.copyTo(content(prep.rect));

        if (prep.rect.size() != rgb.size())
        {
            cv::rectangle(content, prep.rect, draw::kYellow, 2);
        }

        cv::Mat canvas(content.rows + kPanelHeight, content.cols, content.type(), draw::kPanelBg);
        content.copyTo(canvas(cv::Rect(0, 0, content.cols, content.rows)));

        calibrateBtn_.draw(canvas, false);
        detectBtn_.draw(canvas, true);
        slider_.draw(canvas);

        const std::string status = cv::format(
            "PCB: %s   Crop: %s   Registration: %s",
            registration_.cropEnabled ? (prep.pcbFound ? "found" : "not found") : "-",
            registration_.cropEnabled ? "on" : "off",
            !registration_.registerEnabled ? "off"
                : (prep.registered ? "on" : "no homography file"));

        draw::text(canvas, status, { kMargin, 25 }, 0.55, draw::kWhite, 1);

        draw::text(canvas, "[C] Crop on/off    [M / ESC] Back    [F] Fullscreen    [Q] Quit",
                   { kMargin, canvas.rows - 15 });

        return canvas;
    }

    void onMouse(int event, int x, int y) override
    {
        if (!layoutDone_) return;

        if (slider_.onMouse(event, x, y)) return;

        if (event == cv::EVENT_LBUTTONDOWN && calibrateBtn_.contains(x, y))
        {
            pending_ = Action::GO_AUTO_CALIB;
        }
    }

    Action onKey(int key) override
    {
        if (pending_ != Action::STAY)
        {
            const Action a = pending_;
            pending_ = Action::STAY;
            return a;
        }

        if (key == 'c' || key == 'C')
        {
            registration_.cropEnabled = !registration_.cropEnabled;
            return Action::STAY;
        }

        return commonKeyAction(key);
    }

    float thresholdC() const { return slider_.value; }

private:
    static constexpr int kPanelHeight  = 110;
    static constexpr int kButtonWidth  = 150;
    static constexpr int kButtonHeight = 44;

    void layout(int frameWidth, int frameHeight)
    {
        const int panelY  = frameHeight;
        const int buttonY = panelY + (kPanelHeight - kButtonHeight) / 2;

        calibrateBtn_.area = cv::Rect(kMargin, buttonY, kButtonWidth, kButtonHeight);
        detectBtn_.area    = cv::Rect(kMargin * 2 + kButtonWidth, buttonY, kButtonWidth, kButtonHeight);

        const int sliderX = kMargin * 3 + kButtonWidth * 2;

        slider_.area = cv::Rect(sliderX, panelY + kPanelHeight / 2,
                                std::max(100, frameWidth - sliderX - kMargin), 18);

        layoutDone_ = true;
    }

    PcbRegistration&       registration_;
    const HotspotPipeline& pipeline_;
    std::string            homographyPath_;

    Slider slider_;
    Button calibrateBtn_;
    Button detectBtn_;

    bool   layoutDone_ = false;
    Action pending_    = Action::STAY;   // set by mouse clicks
};


// ================================================================
// VIEW: AUTOMATED (CHESSBOARD) CALIBRATION
// ================================================================

class AutoCalibrationView : public View
{
public:
    AutoCalibrationView(ChessboardCalibration& calib, std::string homographyPath)
        : calib_(calib), homographyPath_(std::move(homographyPath))
    {
    }

    void onEnter() override
    {
        succeeded_ = false;
        status_    = "Searching for chessboard...";
    }

    cv::Mat render(const cv::Mat& rgb, const cv::Mat& thermal) override
    {
        cv::Mat canvas = rgb.clone();

        if (!succeeded_ && attempt(rgb, thermal))
        {
            succeeded_ = true;
            successAt_ = std::chrono::steady_clock::now();
        }

        draw::text(canvas, "AUTOMATED CALIBRATION", { kMargin, 40 }, 0.85, draw::kYellow, 2);
        draw::text(canvas, "Place the chessboard in view", { kMargin, 75 }, 0.55);
        draw::text(canvas, "System is continuously searching...", { kMargin, 100 }, 0.55, draw::kGreen);

        if (succeeded_) draw::banner(canvas, "CALIBRATION SUCCESSFUL!");

        draw::text(canvas, status_, { kMargin, canvas.rows - 50 });
        draw::text(canvas, "[M / ESC] Back    [F] Fullscreen    [Q] Quit",
                   { kMargin, canvas.rows - 20 }, 0.5, draw::kGrey);

        return canvas;
    }

    Action onKey(int key) override
    {
        const Action common = commonKeyAction(key);
        if (common != Action::STAY) return common;

        // Leave once the success banner has been shown for a moment.
        if (succeeded_ && std::chrono::steady_clock::now() - successAt_ > kSuccessDisplayTime)
        {
            return Action::BACK;
        }

        return Action::STAY;
    }

private:
    bool attempt(const cv::Mat& rgb, const cv::Mat& thermal)
    {
        // The detector wants an 8-bit single channel thermal picture.
        cv::Mat thermal8u;

        if (thermal.type() == CV_32F || thermal.type() == CV_64F)
        {
            cv::normalize(thermal, thermal8u, 0, 255, cv::NORM_MINMAX, CV_8U);
        }
        else if (thermal.channels() == 3)
        {
            cv::cvtColor(thermal, thermal8u, cv::COLOR_BGR2GRAY);
        }
        else
        {
            thermal8u = thermal;
        }

        auto pair = calib_.DetectPair(rgb, thermal8u);

        if (!pair.success) return false;

        cv::Mat H = ChessboardCalibration::ComputeHomography(pair);

        if (H.empty())
        {
            status_ = "Chessboard seen but homography failed, try again";
            return false;
        }

        const double err = ChessboardCalibration::ComputeMeanReprojectionError(H, pair);

        ChessboardCalibration::SaveHomography(homographyPath_, H);

        status_ = cv::format("Calibration successful - error %.2f px - saved %s",
                             err, homographyPath_.c_str());
        return true;
    }

    ChessboardCalibration& calib_;
    std::string            homographyPath_;
    std::string            status_;

    bool succeeded_ = false;
    std::chrono::steady_clock::time_point successAt_;
};


// ================================================================
// VIEW: MANUAL CALIBRATION
//
//   +----------------------+----------------------+
//   |      RGB frame       |   thermal (zoomed)   |
//   |   click point N      |    click point N     |
//   +----------------------+----------------------+
//   | instructions / status                       |
//   +---------------------------------------------+
//
// Click a point in one picture, then the same physical point in the
// other. Points are numbered; equal numbers are a pair.
//
//   SPACE  freeze / unfreeze the frames while you click
//   U      undo last point
//   R      reset all points
//   ENTER  compute and save the homography (needs >= 4 pairs)
// ================================================================

class ManualCalibrationView : public View
{
public:
    ManualCalibrationView(ManualCalibration& calib, std::string homographyPath)
        : calib_(calib), homographyPath_(std::move(homographyPath))
    {
    }

    void onEnter() override
    {
        calib_.reset();
        frozen_    = false;
        succeeded_ = false;
        status_    = "Click a point in the RGB image, then the same point in the thermal image.";
    }

    cv::Mat render(const cv::Mat& rgb, const cv::Mat& thermal) override
    {
        if (!frozen_)
        {
            rgb_     = rgb;
            thermal_ = thermal;
        }

        // Thermal picture is zoomed (nearest neighbour) to the RGB height
        // so every sensor pixel is a big clickable block.
        thermalZoom_ = static_cast<double>(rgb_.rows) / thermal_.rows;

        const int thermalW = static_cast<int>(thermal_.cols * thermalZoom_);

        rgbPane_     = cv::Rect(0, 0, rgb_.cols, rgb_.rows);
        thermalPane_ = cv::Rect(rgb_.cols + kGap, 0, thermalW, rgb_.rows);

        cv::Mat canvas(rgb_.rows + kPanelHeight, thermalPane_.br().x, CV_8UC3, draw::kPanelBg);

        rgb_.copyTo(canvas(rgbPane_));
        draw::colorizeThermal(thermal_, thermalPane_.size()).copyTo(canvas(thermalPane_));

        // Highlight the pane that expects the next click.
        const cv::Rect& expected = (calib_.nextSide() == Side::RGB) ? rgbPane_ : thermalPane_;
        cv::rectangle(canvas, expected, draw::kYellow, 3);

        drawPoints(canvas, Side::RGB);
        drawPoints(canvas, Side::THERMAL);

        draw::text(canvas, "RGB",     { rgbPane_.x + 10, 25 },     0.7, draw::kWhite, 2);
        draw::text(canvas, "THERMAL", { thermalPane_.x + 10, 25 }, 0.7, draw::kWhite, 2);

        if (frozen_)
        {
            draw::text(canvas, "FROZEN", { rgbPane_.x + rgbPane_.width - 110, 25 }, 0.7, kFrozenColor, 2);
        }

        if (succeeded_) draw::banner(canvas, "CALIBRATION SUCCESSFUL!");

        drawPanel(canvas);

        return canvas;
    }

    void onMouse(int event, int x, int y) override
    {
        if (event != cv::EVENT_LBUTTONDOWN || succeeded_ || rgb_.empty()) return;

        const cv::Point p(x, y);

        if (rgbPane_.contains(p))
        {
            const cv::Point2f pt(static_cast<float>(x - rgbPane_.x), static_cast<float>(y - rgbPane_.y));

            if (calib_.addPoint(Side::RGB, pt))
            {
                status_ = cv::format("RGB point %zu added. Now click the same point in the THERMAL image.",
                                     calib_.count(Side::RGB));
            }
            else
            {
                status_ = "Pick the matching THERMAL point first.";
            }
        }
        else if (thermalPane_.contains(p))
        {
            // canvas -> sensor pixel coordinates (pixel centre at integer)
            const cv::Point2f pt(
                static_cast<float>((x - thermalPane_.x) / thermalZoom_ - 0.5),
                static_cast<float>((y - thermalPane_.y) / thermalZoom_ - 0.5));

            if (calib_.addPoint(Side::THERMAL, pt))
            {
                status_ = cv::format("Thermal point %zu added. %s",
                                     calib_.count(Side::THERMAL),
                                     calib_.ready() ? "Press ENTER to compute, or add more pairs."
                                                    : "Click the next point in the RGB image.");
            }
            else
            {
                status_ = "Pick the matching RGB point first.";
            }
        }
    }

    Action onKey(int key) override
    {
        const Action common = commonKeyAction(key);
        if (common != Action::STAY) return common;

        if (succeeded_)
        {
            return (std::chrono::steady_clock::now() - successAt_ > kSuccessDisplayTime)
                ? Action::BACK : Action::STAY;
        }

        switch (key)
        {
            case ' ':
                frozen_ = !frozen_;
                status_ = frozen_ ? "Frames frozen. Click your points." : "Frames live.";
                break;

            case 'u': case 'U':
                status_ = calib_.undo() ? "Last point removed." : "Nothing to undo.";
                break;

            case 'r': case 'R':
                calib_.reset();
                status_ = "All points cleared.";
                break;

            case 13: case 10:   // ENTER
                finish();
                break;

            default:
                break;
        }

        return Action::STAY;
    }

private:
    using Side = ManualCalibration::Side;

    static constexpr int kGap         = 10;   // between the two pictures
    static constexpr int kPanelHeight = 95;

    const cv::Scalar kRgbColor     = cv::Scalar(0, 255, 0);
    const cv::Scalar kThermalColor = cv::Scalar(255, 255, 255);
    const cv::Scalar kFrozenColor  = cv::Scalar(0, 200, 255);


    void drawPoints(cv::Mat& canvas, Side side) const
    {
        const auto& pts   = calib_.points(side);
        const auto  color = (side == Side::RGB) ? kRgbColor : kThermalColor;

        for (std::size_t i = 0; i < pts.size(); ++i)
        {
            cv::Point p;

            if (side == Side::RGB)
            {
                p = cv::Point(rgbPane_.x + cvRound(pts[i].x), rgbPane_.y + cvRound(pts[i].y));
            }
            else
            {
                // sensor pixel centre -> canvas
                p = cv::Point(thermalPane_.x + cvRound((pts[i].x + 0.5) * thermalZoom_),
                              thermalPane_.y + cvRound((pts[i].y + 0.5) * thermalZoom_));
            }

            cv::circle(canvas, p, 6, color, 2, cv::LINE_AA);
            cv::drawMarker(canvas, p, color, cv::MARKER_CROSS, 16, 1, cv::LINE_AA);
            draw::text(canvas, std::to_string(i + 1), p + cv::Point(9, -9), 0.6, color, 2);
        }
    }

    void drawPanel(cv::Mat& canvas) const
    {
        const int y0 = canvas.rows - kPanelHeight;

        const std::string progress = cv::format(
            "Pairs: %zu / %zu minimum   (RGB %zu, thermal %zu)",
            calib_.pairCount(), calib_.minPairs(), calib_.count(Side::RGB), calib_.count(Side::THERMAL));

        draw::text(canvas, "MANUAL CALIBRATION", { kMargin, y0 + 22 }, 0.7, draw::kYellow, 2);
        draw::text(canvas, progress, { kMargin, y0 + 45 }, 0.5, calib_.ready() ? draw::kGreen : draw::kWhite);
        draw::text(canvas, status_, { kMargin, y0 + 65 }, 0.5, draw::kGrey);
        draw::text(canvas,
                   "[Click] add point   [SPACE] freeze   [U] undo   [R] reset   [ENTER] compute & save   [M/ESC] back   [Q] quit",
                   { kMargin, y0 + 85 }, 0.45, draw::kDim);
    }

    bool finish()
    {
        if (!calib_.ready())
        {
            status_ = cv::format("Need at least %zu pairs (have %zu).", calib_.minPairs(), calib_.pairCount());
            return false;
        }

        double err = 0.0;
        cv::Mat H = calib_.computeHomography(&err);

        if (H.empty())
        {
            status_ = "Homography failed - points may be collinear. Pick points spread over the image.";
            return false;
        }

        // Same file format as the automated mode.
        ChessboardCalibration::SaveHomography(homographyPath_, H);

        status_    = cv::format("Saved %s (mean error %.2f thermal px)", homographyPath_.c_str(), err);
        succeeded_ = true;
        successAt_ = std::chrono::steady_clock::now();

        return true;
    }

    ManualCalibration& calib_;
    std::string        homographyPath_;

    // Frames shown in the last render; when frozen they stop updating.
    cv::Mat rgb_;
    cv::Mat thermal_;
    bool    frozen_ = false;

    // Where the two pictures sit on the canvas (canvas coordinates).
    cv::Rect rgbPane_;
    cv::Rect thermalPane_;
    double   thermalZoom_ = 1.0;   // canvas px per thermal sensor px

    std::string status_;

    bool succeeded_ = false;
    std::chrono::steady_clock::time_point successAt_;
};


// ================================================================
// MAIN
// ================================================================

int main(int argc, char** argv)
{
    // ---- command line ---------------------------------------------

    AppOptions opts;
    int exitCode = 0;

    if (!AppOptions::parse(argc, argv, opts, exitCode)) return exitCode;

    // ---- sources --------------------------------------------------

    std::string rgbName;

    std::unique_ptr<IRGBCamera> rgbCamera = makeRgbSource(opts, rgbName);

    if (!rgbCamera || !rgbCamera->initialize())
    {
        std::cerr << "ERROR: failed to initialize RGB source " << rgbName << "\n";
        return -1;
    }

    std::unique_ptr<ThermalSource> thermal = makeThermalSource(opts);

    if (!thermal || !thermal->initialize())
    {
        std::cerr << "ERROR: failed to initialize thermal source\n";
        return -1;
    }

    const std::string sourcesLine = "RGB: " + rgbName + "   |   Thermal: " + thermal->name();

    std::cout << sourcesLine << "\n";

    // ---- processing -----------------------------------------------

    PcbRegistration       registration(opts.crop, opts.registr);
    HotspotPipeline       pipeline;
    ChessboardCalibration chessboard(opts.boardSize);
    ManualCalibration     manualCalib;

    // ---- window + views -------------------------------------------

    DisplayWindow window(kWindowName, cv::Size(opts.displayWidth, opts.displayHeight), opts.fullscreen);

    MenuView              menuView(window.maxDisplay(), sourcesLine);
    LiveHeatmapView       liveView(registration, pipeline, opts.homographyPath, opts.thresholdC);
    AutoCalibrationView   autoCalibView(chessboard, opts.homographyPath);
    ManualCalibrationView manualCalibView(manualCalib, opts.homographyPath);

    auto viewFor = [&](AppMode mode) -> View&
    {
        switch (mode)
        {
            case AppMode::LIVE_HEATMAP:          return liveView;
            case AppMode::AUTOMATED_CALIBRATION: return autoCalibView;
            case AppMode::MANUAL_CALIBRATION:    return manualCalibView;
            case AppMode::MENU:
            default:                             return menuView;
        }
    };

    View* view = nullptr;

    auto switchTo = [&](AppMode mode)
    {
        view = &viewFor(mode);
        view->onEnter();
        window.setView(view);
    };

    switchTo(opts.mode);

    // ---- main loop ------------------------------------------------

    cv::Mat rgb;
    cv::Mat thermalFrame;

    int grabFailures = 0;

    while (true)
    {
        if (view->needsFrames())
        {
            const bool ok = rgbCamera->captureFrame(rgb) && !rgb.empty() &&
                            thermal->grab(rgb, thermalFrame);

            if (!ok)
            {
                if (++grabFailures >= kMaxGrabFailures)
                {
                    std::cerr << "ERROR: source stopped delivering frames.\n";
                    break;
                }
                continue;
            }

            grabFailures = 0;
        }

        window.show(view->render(rgb, thermalFrame));

        const int key = cv::waitKey(view->frameDelayMs());

        if (window.handleKey(key)) continue;

        const Action action = view->onKey(key);

        if (action == Action::QUIT) break;

        switch (action)
        {
            case Action::BACK:
                if (opts.explicitMode) return 0;   // launched in a mode: nothing to go back to
                switchTo(AppMode::MENU);
                break;

            case Action::GO_MENU:         switchTo(AppMode::MENU);                  break;
            case Action::GO_LIVE:         switchTo(AppMode::LIVE_HEATMAP);          break;
            case Action::GO_AUTO_CALIB:   switchTo(AppMode::AUTOMATED_CALIBRATION); break;
            case Action::GO_MANUAL_CALIB: switchTo(AppMode::MANUAL_CALIBRATION);    break;

            default:
                break;
        }
    }

    return 0;
}