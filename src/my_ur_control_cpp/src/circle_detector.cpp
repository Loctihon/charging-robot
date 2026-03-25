/**
 * charging_port_detector.cpp  (ROS 2)
 *
 * Subscribes:
 *   /base_camera/base_camera_sensor/image_raw  (sensor_msgs/Image, BGR)
 *   /base_camera/depth_heatmap                 (sensor_msgs/Image, BGR heatmap)
 *
 * Intrinsics (hardcoded):
 *   FX = FY = 1108.5,  CX = 640,  CY = 360
 *
 * Publishes:
 *   /charging_port/pose   (geometry_msgs/PointStamped)
 *   /charging_port/debug  (sensor_msgs/Image)
 *
 * Build deps: rclcpp, sensor_msgs, geometry_msgs, cv_bridge,
 *             message_filters, opencv
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <vector>
#include <string>

// ─────────────────────────────────────────────────────────────────────────────
//  Camera intrinsics
// ─────────────────────────────────────────────────────────────────────────────
static constexpr double FX = 1108.5;
static constexpr double FY = 1108.5;
static constexpr double CX = 640.0;
static constexpr double CY = 360.0;

// ─────────────────────────────────────────────────────────────────────────────
//  Colormap LUT
// ─────────────────────────────────────────────────────────────────────────────
struct ColormapLut {
    cv::Mat bgr;        // (256,1,CV_8UC3)
    float   hue[256];   // HSV hue per entry
};

ColormapLut buildLut(int colormapId)
{
    ColormapLut lut;
    cv::Mat ramp(256, 1, CV_8UC1);
    for (int i = 0; i < 256; ++i)
        ramp.at<uchar>(i, 0) = static_cast<uchar>(i);

    cv::applyColorMap(ramp, lut.bgr, colormapId);

    cv::Mat hsv;
    cv::cvtColor(lut.bgr, hsv, cv::COLOR_BGR2HSV);
    for (int i = 0; i < 256; ++i)
        lut.hue[i] = static_cast<float>(hsv.at<cv::Vec3b>(i, 0)[0]);

    return lut;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Auto-detect colormap by matching hue distribution of heatmap
// ─────────────────────────────────────────────────────────────────────────────
ColormapLut autoDetectColormap(const cv::Mat& heatBgr, rclcpp::Logger logger)
{
    static const std::vector<std::pair<int, std::string>> MAPS = {
        {cv::COLORMAP_JET,     "JET"},
        {cv::COLORMAP_TURBO,   "TURBO"},
        {cv::COLORMAP_HOT,     "HOT"},
        {cv::COLORMAP_RAINBOW, "RAINBOW"},
    };

    cv::Mat hsv;
    cv::cvtColor(heatBgr, hsv, cv::COLOR_BGR2HSV);

    // Sample 300 pixels uniformly
    std::vector<float> sampleHues;
    sampleHues.reserve(300);
    cv::RNG rng(0);
    for (int k = 0; k < 300; ++k) {
        int r = rng.uniform(0, hsv.rows);
        int c = rng.uniform(0, hsv.cols);
        sampleHues.push_back(static_cast<float>(hsv.at<cv::Vec3b>(r, c)[0]));
    }

    // Log center pixel for manual check
    cv::Vec3b cp = heatBgr.at<cv::Vec3b>(heatBgr.rows / 2, heatBgr.cols / 2);
    RCLCPP_INFO(logger, "[Colormap] Center pixel BGR=(%d,%d,%d)", cp[0], cp[1], cp[2]);

    int         bestId   = cv::COLORMAP_JET;
    std::string bestName = "JET";
    float       bestErr  = 1e9f;

    for (auto& [mapId, mapName] : MAPS) {
        ColormapLut lut = buildLut(mapId);
        float totalErr = 0.0f;
        for (float sh : sampleHues) {
            float minD = 1e9f;
            for (int i = 0; i < 256; ++i)
                minD = std::min(minD, std::abs(sh - lut.hue[i]));
            totalErr += minD;
        }
        totalErr /= sampleHues.size();
        RCLCPP_INFO(logger, "[Colormap] %s → mean hue err = %.3f", mapName.c_str(), totalErr);

        if (totalErr < bestErr) { bestErr = totalErr; bestId = mapId; bestName = mapName; }
    }

    RCLCPP_INFO(logger, "[Colormap] ✓ Using: %s (err=%.3f)", bestName.c_str(), bestErr);
    return buildLut(bestId);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Heatmap BGR → depth map (float32, metres)
// ─────────────────────────────────────────────────────────────────────────────
cv::Mat heatmapToDepth(const cv::Mat& heatBgr,
                       const ColormapLut& lut,
                       float minDepth, float maxDepth)
{
    cv::Mat hsv;
    cv::cvtColor(heatBgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat depth(hsv.rows, hsv.cols, CV_32FC1);

    for (int r = 0; r < hsv.rows; ++r) {
        const cv::Vec3b* rp = hsv.ptr<cv::Vec3b>(r);
        float*           dp = depth.ptr<float>(r);
        for (int c = 0; c < hsv.cols; ++c) {
            float hue    = static_cast<float>(rp[c][0]);
            int   bestI  = 0;
            float bestD  = std::abs(hue - lut.hue[0]);
            for (int i = 1; i < 256; ++i) {
                float d = std::abs(hue - lut.hue[i]);
                if (d < bestD) { bestD = d; bestI = i; }
            }
            dp[c] = minDepth + (bestI / 255.0f) * (maxDepth - minDepth);
        }
    }
    return depth;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Detect charging port in RGB → returns centroid pixel + contour
// ─────────────────────────────────────────────────────────────────────────────
bool detectChargingPort(const cv::Mat& bgrImg,
                        int& cx_out, int& cy_out,
                        std::vector<cv::Point>& bestContour)
{
    cv::Mat gray;
    cv::cvtColor(bgrImg, gray, cv::COLOR_BGR2GRAY);

    auto clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    clahe->apply(gray, gray);

    cv::Mat thresh;
    cv::adaptiveThreshold(gray, thresh, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY_INV, 31, 8);

    cv::morphologyEx(thresh, thresh, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::morphologyEx(thresh, thresh, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double imgArea = static_cast<double>(bgrImg.rows * bgrImg.cols);
    const double imgCx   = bgrImg.cols / 2.0;
    const double imgCy   = bgrImg.rows / 2.0;

    double bestScore = 0.0;
    int    bestIdx   = -1;

    for (int i = 0; i < (int)contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area < imgArea * 0.0001 || area > imgArea * 0.05) continue;

        cv::Rect bb     = cv::boundingRect(contours[i]);
        double   aspect = (bb.height > 0) ? (double)bb.width / bb.height : 0.0;
        if (aspect < 0.4 || aspect > 2.5) continue;

        std::vector<cv::Point> hull;
        cv::convexHull(contours[i], hull);
        double hullArea = cv::contourArea(hull);
        double solidity = (hullArea > 0) ? area / hullArea : 0.0;
        if (solidity < 0.55) continue;

        double ccx   = bb.x + bb.width  / 2.0;
        double ccy   = bb.y + bb.height / 2.0;
        double dist  = std::hypot(ccx - imgCx, ccy - imgCy);
        double score = solidity * area / (1.0 + dist * 0.01);

        if (score > bestScore) { bestScore = score; bestIdx = i; }
    }

    if (bestIdx < 0) return false;

    cv::Moments M = cv::moments(contours[bestIdx]);
    if (M.m00 == 0.0) return false;

    cx_out      = static_cast<int>(M.m10 / M.m00);
    cy_out      = static_cast<int>(M.m01 / M.m00);
    bestContour = contours[bestIdx];
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Pinhole back-projection
// ─────────────────────────────────────────────────────────────────────────────
inline void pixelTo3D(int u, int v, float depthM,
                      double& X, double& Y, double& Z)
{
    Z = static_cast<double>(depthM);
    X = (u - CX) * Z / FX;
    Y = (v - CY) * Z / FY;
}

// ─────────────────────────────────────────────────────────────────────────────
//  ROS 2 Node
// ─────────────────────────────────────────────────────────────────────────────
using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

class ChargingPortDetector : public rclcpp::Node
{
public:
    ChargingPortDetector()
    : Node("charging_port_detector"), lutReady_(false), frameCount_(0)
    {
        this->declare_parameter("min_depth_m",         0.0);
        this->declare_parameter("max_depth_m",         3.0);
        this->declare_parameter("depth_sample_radius", 5);

        minDepth_ = static_cast<float>(this->get_parameter("min_depth_m").as_double());
        maxDepth_ = static_cast<float>(this->get_parameter("max_depth_m").as_double());
        depthRoi_ = this->get_parameter("depth_sample_radius").as_int();

        RCLCPP_INFO(this->get_logger(),
            "Intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f | depth=[%.2f,%.2f]m",
            FX, FY, CX, CY, minDepth_, maxDepth_);

        // ── Subscribers (ApproximateTime sync) ──
        subRgb_.subscribe(this, "/base_camera/base_camera_sensor/image_raw");
        subDepth_.subscribe(this, "/base_camera/depth_heatmap");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), subRgb_, subDepth_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
        sync_->registerCallback(
            std::bind(&ChargingPortDetector::syncCb, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ── Publishers ──
        pubPose_  = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/charging_port/pose",  1);
        pubDebug_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/charging_port/debug", 1);

        RCLCPP_INFO(this->get_logger(), "Node started.");
    }

private:
    float       minDepth_{0.0f};
    float       maxDepth_{3.0f};
    int         depthRoi_{5};
    ColormapLut lut_;
    bool        lutReady_;
    int         frameCount_;

    message_filters::Subscriber<sensor_msgs::msg::Image> subRgb_, subDepth_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pubPose_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr          pubDebug_;

    // ── Main sync callback ──
    void syncCb(const sensor_msgs::msg::Image::ConstSharedPtr& rgbMsg,
                const sensor_msgs::msg::Image::ConstSharedPtr& depthMsg)
    {
        // Convert ROS → OpenCV
        cv::Mat rgbBgr, heatBgr;
        try {
            rgbBgr  = cv_bridge::toCvShare(rgbMsg,   "bgr8")->image;
            heatBgr = cv_bridge::toCvShare(depthMsg,  "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
            return;
        }

        // Auto-detect colormap on first frame
        if (!lutReady_) {
            lut_      = autoDetectColormap(heatBgr, this->get_logger());
            lutReady_ = true;
        }

        // Align sizes
        if (heatBgr.size() != rgbBgr.size())
            cv::resize(heatBgr, heatBgr, rgbBgr.size(), 0, 0, cv::INTER_LINEAR);

        cv::Mat debugImg = rgbBgr.clone();

        // Detect port in RGB
        int cx_px = 0, cy_px = 0;
        std::vector<cv::Point> contour;
        if (!detectChargingPort(rgbBgr, cx_px, cy_px, contour)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Charging port not detected.");
            publishDebug(debugImg, rgbMsg->header);
            return;
        }

        // Recover depth
        cv::Mat depthMap = heatmapToDepth(heatBgr, lut_, minDepth_, maxDepth_);

        // Debug log every 30 frames
        if (++frameCount_ % 30 == 0) {
            int dr = std::clamp(cy_px, 0, depthMap.rows - 1);
            int dc = std::clamp(cx_px, 0, depthMap.cols - 1);
            float raw = depthMap.at<float>(dr, dc);
            cv::Vec3b hp = heatBgr.at<cv::Vec3b>(dr, dc);
            RCLCPP_INFO(this->get_logger(),
                "[DBG] centroid=(%d,%d) heatBGR=(%d,%d,%d) depth=%.3fm",
                cx_px, cy_px, hp[0], hp[1], hp[2], raw);
        }

        // Median depth in ROI (accept any value > 0)
        int r  = depthRoi_;
        int y0 = std::max(0,             cy_px - r);
        int y1 = std::min(depthMap.rows, cy_px + r + 1);
        int x0 = std::max(0,             cx_px - r);
        int x1 = std::min(depthMap.cols, cx_px + r + 1);
        cv::Mat roi = depthMap(cv::Rect(x0, y0, x1 - x0, y1 - y0));

        std::vector<float> valid;
        valid.reserve(roi.rows * roi.cols);
        for (int rr = 0; rr < roi.rows; ++rr)
            for (int cc = 0; cc < roi.cols; ++cc) {
                float v = roi.at<float>(rr, cc);
                if (v > 0.0f) valid.push_back(v);
            }

        if (valid.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Depth ROI all-zero. "
                "Check min_depth_m/max_depth_m params or colormap choice.");
            publishDebug(debugImg, rgbMsg->header);
            return;
        }

        std::nth_element(valid.begin(),
                         valid.begin() + valid.size() / 2,
                         valid.end());
        float depthVal = valid[valid.size() / 2];

        // Back-project
        double X, Y, Z;
        pixelTo3D(cx_px, cy_px, depthVal, X, Y, Z);

        // Publish pose
        geometry_msgs::msg::PointStamped pt;
        pt.header  = rgbMsg->header;
        pt.point.x = X;
        pt.point.y = Y;
        pt.point.z = Z;
        pubPose_->publish(pt);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Port → px=(%d,%d)  3D=(%.3f, %.3f, %.3f)m  depth=%.3fm",
            cx_px, cy_px, X, Y, Z, depthVal);

        // Draw debug
        cv::drawContours(debugImg,
            std::vector<std::vector<cv::Point>>{contour}, -1,
            cv::Scalar(0, 255, 0), 2);
        cv::circle(debugImg, cv::Point(cx_px, cy_px), 6,
                   cv::Scalar(0, 0, 255), -1);
        char buf[80];
        std::snprintf(buf, sizeof(buf),
                      "(%.2f,%.2f,%.2f)m d=%.2f", X, Y, Z, depthVal);
        cv::putText(debugImg, buf,
                    cv::Point(cx_px + 10, cy_px - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(0, 255, 255), 2);

        publishDebug(debugImg, rgbMsg->header);
    }

    void publishDebug(const cv::Mat& img,
                      const std_msgs::msg::Header& header)
    {
        if (pubDebug_->get_subscription_count() == 0) return;
        cv_bridge::CvImage out;
        out.header   = header;
        out.encoding = "bgr8";
        out.image    = img;
        pubDebug_->publish(*out.toImageMsg());
    }
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChargingPortDetector>());
    rclcpp::shutdown();
    return 0;
}