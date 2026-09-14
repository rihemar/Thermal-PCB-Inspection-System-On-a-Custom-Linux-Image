#include "JointBilateralUpsample.hpp"
#include <cmath>
#include <algorithm>

cv::Mat JointBilateralUpsample::CreateGaussianKernel(int windowSize, double sigma) {
    cv::Mat kernel(windowSize, windowSize, CV_32F);
    int half = windowSize / 2;
    double sigma2 = sigma * sigma;
    float sum = 0.f;
    for (int x = -half; x <= half; ++x) {
        for (int y = -half; y <= half; ++y) {
            float v = static_cast<float>(std::exp(-(x * x + y * y) / (2.0 * sigma2)));
            kernel.at<float>(x + half, y + half) = v;
            sum += v;
        }
    }
    kernel /= sum;
    return kernel;
}

cv::Mat JointBilateralUpsample::PrepareGuide(const cv::Mat& rgbFrame, GuideMode mode) {
    cv::Mat gray;
    if (rgbFrame.channels() == 3) {
        cv::cvtColor(rgbFrame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = rgbFrame;
    }

    cv::Mat guide;
    switch (mode) {
        case GuideMode::RAW: {
            gray.convertTo(guide, CV_32F, 1.0 / 255.0);
            break;
        }
        case GuideMode::SOBEL: {
            cv::Mat gx, gy, mag;
            cv::Sobel(gray, gx, CV_32F, 1, 0, 3);
            cv::Sobel(gray, gy, CV_32F, 0, 1, 3);
            cv::magnitude(gx, gy, mag);
            cv::normalize(mag, guide, 0.0, 1.0, cv::NORM_MINMAX, CV_32F);
            break;
        }
        case GuideMode::CANNY: {
            cv::Mat edges;
            cv::Canny(gray, edges, 50, 150);
            cv::Mat inv = 255 - edges;
            cv::Mat dist;
            cv::distanceTransform(inv, dist, cv::DIST_L2, 3);
            cv::normalize(dist, guide, 0.0, 1.0, cv::NORM_MINMAX, CV_32F);
            guide = 1.0f - guide; // edges -> 1 (high weight change), flat -> 0
            break;
        }
    }
    return guide;
}

void JointBilateralUpsample::Filter(const cv::Mat& input, const cv::Mat& guide, cv::Mat& output,
                                     int windowSize, float sigmaSpectral, double sigmaSpatial) {
    CV_Assert(input.size() == guide.size());
    output.create(input.size(), CV_32F);

    cv::Mat kernel = CreateGaussianKernel(windowSize, sigmaSpatial);
    int half = windowSize / 2;
    float twoSigmaSq = 2.f * sigmaSpectral * sigmaSpectral;

    for (int r = 0; r < input.rows; ++r) {
        for (int c = 0; c < input.cols; ++c) {
            float sumW = 0.f, sum = 0.f;
            float centerGuide = guide.at<float>(r, c);

            for (int i = -half; i <= half; ++i) {
                int rr = cv::borderInterpolate(r + i, input.rows, cv::BORDER_REFLECT101);
                for (int j = -half; j <= half; ++j) {
                    int cc = cv::borderInterpolate(c + j, input.cols, cv::BORDER_REFLECT101);

                    float rangeDiff = guide.at<float>(rr, cc) - centerGuide;
                    float w = std::exp(-(rangeDiff * rangeDiff) / twoSigmaSq)
                            * kernel.at<float>(i + half, j + half);

                    sum  += input.at<float>(rr, cc) * w;
                    sumW += w;
                }
            }
            output.at<float>(r, c) = (sumW > 1e-8f) ? (sum / sumW) : input.at<float>(r, c);
        }
    }
}

cv::Mat JointBilateralUpsample::Upsample(const cv::Mat& thermalLowRes, const cv::Mat& rgbGuideAligned,
                                          GuideMode mode,
                                          int windowSize, float sigmaSpectral, double sigmaSpatial) {
    CV_Assert(thermalLowRes.type() == CV_32F);

    cv::Mat guideFull = PrepareGuide(rgbGuideAligned, mode);

    double factor = static_cast<double>(guideFull.rows) / thermalLowRes.rows;
    int steps = std::max(1, static_cast<int>(std::round(std::log2(factor))));

    cv::Mat D = thermalLowRes.clone();
    for (int s = 1; s < steps; ++s) {
        cv::resize(D, D, D.size() * 2, 0, 0, cv::INTER_LINEAR);

        cv::Mat guideDown;
        cv::resize(guideFull, guideDown, D.size(), 0, 0, cv::INTER_AREA);

        cv::Mat filtered;
        Filter(D, guideDown, filtered, windowSize, sigmaSpectral, sigmaSpatial);
        D = filtered;
    }

    // Final step: bring D to the guide's exact resolution and do one more
    // joint bilateral pass against the full-resolution guide.
    cv::resize(D, D, guideFull.size(), 0, 0, cv::INTER_LINEAR);
    cv::Mat result;
    Filter(D, guideFull, result, windowSize, sigmaSpectral, sigmaSpatial);
    return result;
}

cv::Mat JointBilateralUpsample::Colorize(const cv::Mat& tempsFloat, double minC, double maxC, int colormap) {
    cv::Mat norm8;
    double range = (maxC - minC);
    if (range < 1e-6) range = 1e-6;
    tempsFloat.convertTo(norm8, CV_8U, 255.0 / range, -minC * 255.0 / range);
    cv::Mat color;
    cv::applyColorMap(norm8, color, colormap);
    return color;
}

cv::Mat JointBilateralUpsample::OverlayEdges(const cv::Mat& colorizedHeatmap,
                                              const cv::Mat& originalRgb,
                                              double alpha,
                                              int cannyLow,
                                              int cannyHigh,
                                              cv::Scalar edgeColor)
{
    CV_Assert(!colorizedHeatmap.empty());
    CV_Assert(!originalRgb.empty());
    CV_Assert(colorizedHeatmap.type() == CV_8UC3);

    // Match sizes: the heatmap may be upsampled to a different resolution
    // than the raw PCB image it was derived from.
    cv::Mat rgbResized;
    if (originalRgb.size() != colorizedHeatmap.size()) {
        cv::resize(originalRgb, rgbResized, colorizedHeatmap.size(), 0, 0, cv::INTER_LINEAR);
    } else {
        rgbResized = originalRgb;
    }

    cv::Mat gray;
    if (rgbResized.channels() == 3) {
        cv::cvtColor(rgbResized, gray, cv::COLOR_BGR2GRAY);
    } else if (rgbResized.channels() == 4) {
        cv::cvtColor(rgbResized, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = rgbResized;
    }

    // Light blur before Canny to suppress noise-driven false edges,
    // especially useful after upsampling.
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);

    cv::Mat edges;
    cv::Canny(blurred, edges, cannyLow, cannyHigh);

    // Optionally thicken edges slightly so they remain visible once
    // blended at partial alpha over a busy colormap.
    cv::dilate(edges, edges, cv::Mat(), cv::Point(-1, -1), 1);

    cv::Mat result = colorizedHeatmap.clone();

    // Build a solid-color edge layer and blend only where edges fired,
    // leaving non-edge pixels untouched (a true overlay rather than a
    // global alpha blend that would wash out the heatmap colors).
    cv::Mat edgeLayer(colorizedHeatmap.size(), CV_8UC3, edgeColor);
    cv::Mat blendedFull;
    cv::addWeighted(result, 1.0 - alpha, edgeLayer, alpha, 0.0, blendedFull);

    blendedFull.copyTo(result, edges);

    return result; // CV_8UC3 BGR
}