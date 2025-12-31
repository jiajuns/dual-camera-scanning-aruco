/**
 * @file simple_stereo.cpp
 * @date December 2025
 * @brief 使用双目相机进行 AprilTag(Standard41h12) 标记检测和 3D 定位的 ROS 2 节点。
 *        利用 message_filters 同步左右图像，通过三角测量计算 3D 坐标。
 */

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <array>
#include <algorithm>
#include <unordered_map>
#include <mutex>
#include <future>
#include <iomanip>

// ROS 2 核心头文件
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/asserts.hpp"
#include "rmw/qos_profiles.h"

// 图像传输与转换
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// 消息类型
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// 消息过滤器（用于时间同步）
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

// TF2 坐标变换
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// AprilTag (Standard41h12)
#include <apriltag/apriltag.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/apriltag_pose.h>

//UDP sender
#include "aruco_ros/udp_sender.hpp"

class ArucoStereo : public rclcpp::Node
{
public:
  ArucoStereo()
  : Node("aruco_stereo")
  {
    // 参数
    this->declare_parameter<double>("marker_size", 0.05);   // 用于 RViz marker 尺寸（m）
    this->declare_parameter<int>("marker_id", -1);          // -1 表示不过滤 ID
    this->declare_parameter<std::string>("reference_frame", "");
    this->declare_parameter<std::string>("camera_frame", "");
    this->declare_parameter<bool>("image_is_rectified", true);
    this->declare_parameter<double>("baseline", 0.05);      // 双目基线（m），优先于 CameraInfo P 矩阵推导
    this->declare_parameter<double>("debug_axis_length", 0.05); // debug坐标轴长度（m）
    //改成具体IP和端口
    //////////////////////////*************************** *//////////////////////////////
    this->declare_parameter<std::string>("udp_ip", "192.168.1.100");
    this->declare_parameter<int>("udp_port", 8888);
    this->declare_parameter<double>("udp_send_rate_hz", 15.0);

    // AprilTag 检测参数
    this->declare_parameter<int>("apriltag_threads", 4);
    this->declare_parameter<double>("apriltag_decimate", 1.0);
    this->declare_parameter<double>("apriltag_sigma", 0.0);
    this->declare_parameter<double>("apriltag_decode_sharpening", 0.25);
    this->declare_parameter<bool>("apriltag_refine_edges", true);

    this->get_parameter("marker_size", marker_size_);
    this->get_parameter("marker_id", marker_id_);
    this->get_parameter("reference_frame", reference_frame_);
    this->get_parameter("camera_frame", camera_frame_);
    this->get_parameter("image_is_rectified", image_is_rectified_);
    this->get_parameter("baseline", baseline_);
    this->get_parameter("debug_axis_length", debug_axis_length_);

    std::string udp_ip;
    int udp_port;
    this->get_parameter("udp_ip", udp_ip);
    this->get_parameter("udp_port", udp_port);
    this->get_parameter("udp_send_rate_hz", udp_send_rate_hz_);
    if (udp_send_rate_hz_ > 0.0) {
      udp_send_min_interval_ns_ = static_cast<uint64_t>(1e9 / udp_send_rate_hz_);
    } else {
      udp_send_min_interval_ns_ = 0;
    }
    udp_sender_ = std::make_unique<aruco_ros::UdpSender>(udp_ip, udp_port);

    int tag_threads = 4;
    double tag_decimate = 1.0;
    double tag_sigma = 0.0;
    double tag_decode_sharpening = 0.25;
    bool tag_refine_edges = true;

    this->get_parameter("apriltag_threads", tag_threads);
    this->get_parameter("apriltag_decimate", tag_decimate);
    this->get_parameter("apriltag_sigma", tag_sigma);
    this->get_parameter("apriltag_decode_sharpening", tag_decode_sharpening);
    this->get_parameter("apriltag_refine_edges", tag_refine_edges);

    RCLCPP_INFO(this->get_logger(), "AprilTag(Standard41h12) stereo node starting...");
    RCLCPP_INFO(this->get_logger(), "Marker size: %.3f m", marker_size_);
    RCLCPP_INFO(this->get_logger(), "Marker ID filter: %d", marker_id_);
    RCLCPP_INFO(this->get_logger(), "AprilTag params: threads=%d decimate=%.2f sigma=%.2f sharpen=%.2f refine_edges=%s",
                tag_threads, tag_decimate, tag_sigma, tag_decode_sharpening,
                tag_refine_edges ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Debug: topic=%s axis_length=%.3f m (axes colors: x=red y=green z=blue)",
                (this->get_name() + std::string("/debug")).c_str(), debug_axis_length_);
    RCLCPP_INFO(this->get_logger(), "Debug boxes: %s and %s (red boxes only)",
                (this->get_name() + std::string("/debug_left")).c_str(),
                (this->get_name() + std::string("/debug_right")).c_str());
    RCLCPP_INFO(this->get_logger(), "UDP Sender initialized to %s:%d", udp_ip.c_str(), udp_port);
    if (udp_send_min_interval_ns_ > 0) {
      RCLCPP_INFO(this->get_logger(), "UDP send rate limit: %.2f Hz", udp_send_rate_hz_);
    } else {
      RCLCPP_INFO(this->get_logger(), "UDP send rate limit: disabled");
    }

    // 初始化 AprilTag detector + family(Standard41h12)
    tag_family_ = tagStandard41h12_create();
    
    // Left detector
    tag_detector_left_ = apriltag_detector_create();
    apriltag_detector_add_family(tag_detector_left_, tag_family_);
    tag_detector_left_->nthreads = tag_threads;
    tag_detector_left_->quad_decimate = tag_decimate;
    tag_detector_left_->quad_sigma = tag_sigma;
    tag_detector_left_->decode_sharpening = tag_decode_sharpening;
    tag_detector_left_->refine_edges = tag_refine_edges ? 1 : 0;

    // Right detector
    tag_detector_right_ = apriltag_detector_create();
    apriltag_detector_add_family(tag_detector_right_, tag_family_);
    tag_detector_right_->nthreads = tag_threads;
    tag_detector_right_->quad_decimate = tag_decimate;
    tag_detector_right_->quad_sigma = tag_sigma;
    tag_detector_right_->decode_sharpening = tag_decode_sharpening;
    tag_detector_right_->refine_edges = tag_refine_edges ? 1 : 0;

    // 发布者
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("poses", 10);
    transform_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("transform", 10);
    debug_pub_ = image_transport::create_publisher(this, "~/debug");
    debug_left_pub_ = image_transport::create_publisher(this, "~/debug_left");
    debug_right_pub_ = image_transport::create_publisher(this, "~/debug_right");

    // TF 广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 相机内参单独订阅并缓存（很多相机的 CameraInfo 频率较低/或 transient_local，放进 message_filters 会导致回调触发很稀疏）
    left_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/left/camera_info", rclcpp::SensorDataQoS(),
      std::bind(&ArucoStereo::leftInfoCallback, this, std::placeholders::_1));
    right_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/right/camera_info", rclcpp::SensorDataQoS(),
      std::bind(&ArucoStereo::rightInfoCallback, this, std::placeholders::_1));

    // message_filters 仅同步左右图像
    left_img_sub_.subscribe(this, "/left/image_rect_color", rmw_qos_profile_sensor_data);
    right_img_sub_.subscribe(this, "/right/image_rect_color", rmw_qos_profile_sensor_data);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(20), left_img_sub_, right_img_sub_);

    sync_->registerCallback(
      std::bind(&ArucoStereo::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Node started. Waiting for synchronized topics...");
  }

  ~ArucoStereo() override
  {
    if (tag_detector_left_) {
      apriltag_detector_destroy(tag_detector_left_);
      tag_detector_left_ = nullptr;
    }
    if (tag_detector_right_) {
      apriltag_detector_destroy(tag_detector_right_);
      tag_detector_right_ = nullptr;
    }
    if (tag_family_) {
      tagStandard41h12_destroy(tag_family_);
      tag_family_ = nullptr;
    }
  }

private:
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image,
      sensor_msgs::msg::Image
    > SyncPolicy;

  struct TagDetection2D {
    int id = -1;
    std::array<cv::Point2f, 4> corners{};
    double decision_margin = 0.0;
  };

  struct DetectionResult {
    int id;
    cv::Point3f pos;
    tf2::Quaternion rot;
  };

  // 参数
  double marker_size_;
  int marker_id_;
  std::string reference_frame_;
  std::string camera_frame_;
  bool image_is_rectified_;
  double baseline_;
  double debug_axis_length_;

  // 订阅者
  message_filters::Subscriber<sensor_msgs::msg::Image> left_img_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> right_img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  std::mutex cam_info_mutex_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr last_left_info_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr last_right_info_;

  // 发布者
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub_;
  image_transport::Publisher debug_pub_;
  image_transport::Publisher debug_left_pub_;
  image_transport::Publisher debug_right_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::unique_ptr<aruco_ros::UdpSender> udp_sender_;
  uint32_t packet_seq_{0};
  double udp_send_rate_hz_{15.0};
  uint64_t udp_send_min_interval_ns_{0};
  uint64_t last_udp_send_ts_ns_{0};

  // AprilTag detector
  apriltag_family_t* tag_family_{nullptr};
  apriltag_detector_t* tag_detector_left_{nullptr};
  apriltag_detector_t* tag_detector_right_{nullptr};

  static void drawDetection(cv::Mat& bgr, const TagDetection2D& det, const cv::Scalar& color_bgr)
  {
    std::vector<cv::Point> pts;
    pts.reserve(4);
    for (int i = 0; i < 4; ++i) {
      pts.emplace_back(static_cast<int>(std::lround(det.corners[i].x)),
                       static_cast<int>(std::lround(det.corners[i].y)));
    }
    std::vector<std::vector<cv::Point>> polys{pts};
    cv::polylines(bgr, polys, true, color_bgr, 2);

    // ID 文本放在角点 0 附近
    cv::putText(bgr, std::to_string(det.id),
                pts[0], cv::FONT_HERSHEY_SIMPLEX, 0.7,
                color_bgr, 2);
  }

  static bool projectToPixel(const cv::Point3f& p_cam,
                             double fx, double fy, double cx, double cy,
                             cv::Point2f& uv)
  {
    if (!std::isfinite(p_cam.z) || p_cam.z <= 1e-6) return false;
    const double u = fx * (static_cast<double>(p_cam.x) / static_cast<double>(p_cam.z)) + cx;
    const double v = fy * (static_cast<double>(p_cam.y) / static_cast<double>(p_cam.z)) + cy;
    if (!std::isfinite(u) || !std::isfinite(v)) return false;
    uv = cv::Point2f(static_cast<float>(u), static_cast<float>(v));
    return true;
  }

  static void drawAxesOnImage(cv::Mat& bgr,
                              const cv::Point3f& center_cam,
                              const tf2::Vector3& axis_x_cam,
                              const tf2::Vector3& axis_y_cam,
                              const tf2::Vector3& axis_z_cam,
                              double axis_length_m,
                              double fx, double fy, double cx, double cy)
  {
    const float L = static_cast<float>(std::max(0.0, axis_length_m));
    if (L <= 1e-6f) return;

    cv::Point2f c_uv;
    if (!projectToPixel(center_cam, fx, fy, cx, cy, c_uv)) return;

    const cv::Point3f x_end(center_cam.x + static_cast<float>(axis_x_cam.x()) * L,
                            center_cam.y + static_cast<float>(axis_x_cam.y()) * L,
                            center_cam.z + static_cast<float>(axis_x_cam.z()) * L);
    const cv::Point3f y_end(center_cam.x + static_cast<float>(axis_y_cam.x()) * L,
                            center_cam.y + static_cast<float>(axis_y_cam.y()) * L,
                            center_cam.z + static_cast<float>(axis_y_cam.z()) * L);
    const cv::Point3f z_end(center_cam.x + static_cast<float>(axis_z_cam.x()) * L,
                            center_cam.y + static_cast<float>(axis_z_cam.y()) * L,
                            center_cam.z + static_cast<float>(axis_z_cam.z()) * L);

    cv::Point2f x_uv, y_uv, z_uv;
    const bool ok_x = projectToPixel(x_end, fx, fy, cx, cy, x_uv);
    const bool ok_y = projectToPixel(y_end, fx, fy, cx, cy, y_uv);
    const bool ok_z = projectToPixel(z_end, fx, fy, cx, cy, z_uv);

    const int thickness = 2;
    if (ok_x) {
      cv::line(bgr, c_uv, x_uv, cv::Scalar(0, 0, 255), thickness);
      cv::putText(bgr, "x", x_uv, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    }
    if (ok_y) {
      cv::line(bgr, c_uv, y_uv, cv::Scalar(0, 255, 0), thickness);
      cv::putText(bgr, "y", y_uv, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }
    if (ok_z) {
      cv::line(bgr, c_uv, z_uv, cv::Scalar(255, 0, 0), thickness);
      cv::putText(bgr, "z", z_uv, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
    }
  }

  void leftInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(cam_info_mutex_);
    last_left_info_ = msg;
  }

  void rightInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(cam_info_mutex_);
    last_right_info_ = msg;
  }

  static double nonzeroOr(double v, double fallback)
  {
    return (std::isfinite(v) && std::abs(v) > 1e-12) ? v : fallback;
  }

  std::vector<TagDetection2D> detectStandard41h12(const cv::Mat& gray, apriltag_detector_t* detector)
  {
    std::vector<TagDetection2D> out;
    if (!detector) return out;
    if (gray.empty()) return out;
    if (gray.type() != CV_8UC1) return out;

    // OpenCV cv::Mat -> AprilTag image_u8_t header（不做深拷贝）
    image_u8_t img_header = {
        gray.cols,
        gray.rows,
        static_cast<int32_t>(gray.step),
        gray.data
    };

    zarray_t* detections = apriltag_detector_detect(detector, &img_header);
    const int n = zarray_size(detections);
    out.reserve(std::max(0, n));

    for (int i = 0; i < n; ++i) {
      apriltag_detection_t* det = nullptr;
      zarray_get(detections, i, &det);
      if (!det) continue;

      TagDetection2D d;
      d.id = det->id;
      d.decision_margin = det->decision_margin;
      for (int k = 0; k < 4; ++k) {
        d.corners[k] = cv::Point2f(static_cast<float>(det->p[k][0]),
                                  static_cast<float>(det->p[k][1]));
      }
      out.push_back(d);
    }

    apriltag_detections_destroy(detections);
    return out;
  }

  void syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& left_img_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& right_img_msg)
  {
    sensor_msgs::msg::CameraInfo::ConstSharedPtr left_info_msg;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr right_info_msg;
    {
      std::lock_guard<std::mutex> lk(cam_info_mutex_);
      left_info_msg = last_left_info_;
      right_info_msg = last_right_info_;
    }
    if (!left_info_msg) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "No /left/camera_info received yet, skipping.");
      return;
    }

    // 1) ROS 图像 -> cv::Mat (BGR)
    cv_bridge::CvImagePtr left_cv_ptr, right_cv_ptr;
    try {
      left_cv_ptr = cv_bridge::toCvCopy(left_img_msg, sensor_msgs::image_encodings::BGR8);
      right_cv_ptr = cv_bridge::toCvCopy(right_img_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // 2) 内参 & 基线
    const double fx = nonzeroOr(left_info_msg->p[0], left_info_msg->k[0]);
    const double fy = nonzeroOr(left_info_msg->p[5], left_info_msg->k[4]);
    const double cx = nonzeroOr(left_info_msg->p[2], left_info_msg->k[2]);
    const double cy = nonzeroOr(left_info_msg->p[6], left_info_msg->k[5]);

    // const double tx_right = right_info_msg->p[3];
    // const double baseline = -tx_right / fx;
    const double baseline = 0.05; // 50mm fixed

    if (!std::isfinite(fx) || fx <= 1e-9) {
      RCLCPP_ERROR(this->get_logger(), "Invalid fx from CameraInfo.");
      return;
    }

    if (std::abs(baseline) < 1e-6) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Baseline is too small or zero. Check stereo calibration / CameraInfo P matrix.");
    }
    if (!image_is_rectified_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "image_is_rectified=false but this node uses simple disparity (x_left - x_right); results may be invalid.");
    }

    // 3) AprilTag 检测（灰度）
    cv::Mat gray_left, gray_right;
    cv::cvtColor(left_cv_ptr->image, gray_left, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_cv_ptr->image, gray_right, cv::COLOR_BGR2GRAY);

    // 使用 std::async 并行检测
    auto future_left = std::async(std::launch::async, [this, &gray_left]() {
        return this->detectStandard41h12(gray_left, this->tag_detector_left_);
    });
    auto future_right = std::async(std::launch::async, [this, &gray_right]() {
        return this->detectStandard41h12(gray_right, this->tag_detector_right_);
    });

    auto dets_left = future_left.get();
    auto dets_right = future_right.get();

    const bool publish_debug = debug_pub_.getNumSubscribers() > 0;
    const bool publish_left_boxes = debug_left_pub_.getNumSubscribers() > 0;
    const bool publish_right_boxes = debug_right_pub_.getNumSubscribers() > 0;

    cv::Mat debug_img;
    cv::Mat left_boxes_img;
    cv::Mat right_boxes_img;
    if (publish_debug) debug_img = left_cv_ptr->image.clone();
    if (publish_left_boxes) left_boxes_img = left_cv_ptr->image.clone();
    if (publish_right_boxes) right_boxes_img = right_cv_ptr->image.clone();

    // 新增左右话题：各自相机的检测结果（不要求双目匹配），仅画红框
    if (publish_left_boxes) {
      for (const auto& dl : dets_left) {
        if (marker_id_ != -1 && dl.id != marker_id_) continue;
        drawDetection(left_boxes_img, dl, cv::Scalar(0, 0, 255));
      }
    }
    if (publish_right_boxes) {
      for (const auto& dr : dets_right) {
        if (marker_id_ != -1 && dr.id != marker_id_) continue;
        drawDetection(right_boxes_img, dr, cv::Scalar(0, 0, 255));
      }
    }

    // 4) 按 ID 匹配左右检测
    std::unordered_map<int, const TagDetection2D*> right_by_id;
    right_by_id.reserve(dets_right.size());
    for (const auto& dr : dets_right) {
      right_by_id.emplace(dr.id, &dr);
    }

    // 准备 PoseArray
    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header = left_img_msg->header;
    if (!camera_frame_.empty()) {
      pose_array_msg.header.frame_id = camera_frame_;
    }

    std::vector<DetectionResult> frame_results;

    for (const auto& dl : dets_left) {
      if (marker_id_ != -1 && dl.id != marker_id_) continue;

      const auto it = right_by_id.find(dl.id);
      if (it == right_by_id.end()) continue;

      // 调试画框：仅双目匹配成功时画红框
      if (publish_debug) drawDetection(debug_img, dl, cv::Scalar(0, 0, 255));

      // 双目三角测量 + 姿态
      processStereoAprilTag(dl.id, dl.corners, it->second->corners,
                            fx, fy, cx, cy, baseline, left_img_msg->header, 
                            publish_debug ? &debug_img : nullptr,
                            pose_array_msg, frame_results);
    }

    // 发布 PoseArray
    if (!pose_array_msg.poses.empty()) {
      poses_pub_->publish(pose_array_msg);
    }

    // 只有当识别到两个及以上码时才输出日志
    if (frame_results.size() >= 2) {
      std::stringstream ss;
      ss << "[";
      for (size_t i = 0; i < frame_results.size(); ++i) {
        const auto& res = frame_results[i];
        ss << "{'id': " << res.id 
           << ", 'pos': [" << std::fixed << std::setprecision(2) << res.pos.x << "," << res.pos.y << "," << res.pos.z << "]"
           << "}";
        if (i < frame_results.size() - 1) ss << ", ";
      }
      ss << "]";
      RCLCPP_INFO(this->get_logger(), "Stereo Tags: %s", ss.str().c_str());

      // UDP Send
      uint64_t ts_ns = static_cast<uint64_t>(left_img_msg->header.stamp.sec) * 1000000000ULL + 
                       static_cast<uint64_t>(left_img_msg->header.stamp.nanosec);
      bool should_send = true;
      if (udp_send_min_interval_ns_ > 0 && last_udp_send_ts_ns_ > 0 &&
          ts_ns > last_udp_send_ts_ns_ &&
          (ts_ns - last_udp_send_ts_ns_) < udp_send_min_interval_ns_) {
        should_send = false;
      }
      if (should_send) {
        std::vector<aruco_ros::TagData> tags_data;
        tags_data.reserve(frame_results.size());
        for (const auto& res : frame_results) {
            tags_data.push_back({res.id, res.pos, res.rot});
        }
        last_udp_send_ts_ns_ = ts_ns;
        udp_sender_->sendData(tags_data, packet_seq_++, ts_ns);
      }
    }

    // 发布调试图像：1) 原debug：左图 + 红框 + 坐标轴；2) 新增左右话题：仅红框
    if (publish_debug) {
      cv_bridge::CvImage out;
      out.header = left_img_msg->header;
      out.encoding = sensor_msgs::image_encodings::BGR8;
      out.image = debug_img;
      debug_pub_.publish(out.toImageMsg());
    }
    if (publish_left_boxes) {
      cv_bridge::CvImage out;
      out.header = left_img_msg->header;
      out.encoding = sensor_msgs::image_encodings::BGR8;
      out.image = left_boxes_img;
      debug_left_pub_.publish(out.toImageMsg());
    }
    if (publish_right_boxes) {
      cv_bridge::CvImage out;
      out.header = right_img_msg->header;
      out.encoding = sensor_msgs::image_encodings::BGR8;
      out.image = right_boxes_img;
      debug_right_pub_.publish(out.toImageMsg());
    }
  }

  void processStereoAprilTag(
    int id,
    const std::array<cv::Point2f, 4>& corners_l,
    const std::array<cv::Point2f, 4>& corners_r,
    double fx, double fy, double cx, double cy, double baseline,
    const std_msgs::msg::Header& header,
    cv::Mat* debug_bgr,
    geometry_msgs::msg::PoseArray& pose_array,
    std::vector<DetectionResult>& results)
  {
    // 5) 三角测量：4 角点
    std::vector<cv::Point3f> corners_3d;
    corners_3d.reserve(4);
    cv::Point3f center_3d(0, 0, 0);

    for (int i = 0; i < 4; ++i) {
      const double xl = corners_l[i].x;
      const double yl = corners_l[i].y;
      const double xr = corners_r[i].x;

      const double d = xl - xr; // disparity

      if (d < 1e-5) continue;

      const double Z = (fx * baseline) / d;
      const double X = (xl - cx) * Z / fx;
      const double Y = (yl - cy) * Z / fy;

      corners_3d.emplace_back(static_cast<float>(X),
                              static_cast<float>(Y),
                              static_cast<float>(Z));
      center_3d.x += static_cast<float>(X);
      center_3d.y += static_cast<float>(Y);
      center_3d.z += static_cast<float>(Z);
    }

    if (corners_3d.size() != 4) {
      RCLCPP_WARN(this->get_logger(),
                  "Could not triangulate all corners for tag %d", id);
      return;
    }

    center_3d.x /= 4.0f;
    center_3d.y /= 4.0f;
    center_3d.z /= 4.0f;

    // 6) 姿态（用 3D 角点构建局部坐标轴）
    tf2::Vector3 p0(corners_3d[0].x, corners_3d[0].y, corners_3d[0].z);
    tf2::Vector3 p1(corners_3d[1].x, corners_3d[1].y, corners_3d[1].z);
    tf2::Vector3 p2(corners_3d[2].x, corners_3d[2].y, corners_3d[2].z);
    tf2::Vector3 p3(corners_3d[3].x, corners_3d[3].y, corners_3d[3].z);

    // 1. 修改 v_y 为由上指向下 (Down)
    tf2::Vector3 v_y = (p0 - p3) + (p1 - p2); 
    v_y.normalize();

    // 2. 修改 v_x 为由左指向右 (Right)
    tf2::Vector3 v_x = (p1 - p0) + (p2 - p3);
    v_x.normalize();

    tf2::Vector3 v_z = v_x.cross(v_y);
    v_z.normalize();

    v_y = v_z.cross(v_x);
    v_y.normalize();
    // vx vy vz构建矩阵
    tf2::Matrix3x3 rot_mat(
      v_x.x(), v_y.x(), v_z.x(),
      v_x.y(), v_y.y(), v_z.y(),
      v_x.z(), v_y.z(), v_z.z()
    );

    tf2::Quaternion q;
    rot_mat.getRotation(q);

    if (debug_bgr && !debug_bgr->empty()) {
      drawAxesOnImage(*debug_bgr, cv::Point3f(center_3d.x, center_3d.y, center_3d.z),
                      v_x, v_y, v_z, debug_axis_length_, fx, fy, cx, cy);
    }

    // 添加到 PoseArray
    geometry_msgs::msg::Pose pose;
    pose.position.x = center_3d.x;
    pose.position.y = center_3d.y;
    pose.position.z = center_3d.z;
    pose.orientation = tf2::toMsg(q);
    pose_array.poses.push_back(pose);

    // 记录结果用于日志
    results.push_back({id, center_3d, q});

    // 7) 发布
    publishResults(id, center_3d, q, header);
  }

  void publishResults(int id, const cv::Point3f& position, const tf2::Quaternion& q,
                      const std_msgs::msg::Header& header)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = header;
    if (!camera_frame_.empty()) {
      pose_msg.header.frame_id = camera_frame_;
    }
    pose_msg.pose.position.x = position.x;
    pose_msg.pose.position.y = position.y;
    pose_msg.pose.position.z = position.z;
    pose_msg.pose.orientation = tf2::toMsg(q);
    pose_pub_->publish(pose_msg);

    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header = pose_msg.header;
    transform_msg.child_frame_id = "apriltag_" + std::to_string(id);
    transform_msg.transform.translation.x = position.x;
    transform_msg.transform.translation.y = position.y;
    transform_msg.transform.translation.z = position.z;
    transform_msg.transform.rotation = pose_msg.pose.orientation;

    // 既发 TF，也发 topic（避免 transform_pub_ “未使用”）
    tf_broadcaster_->sendTransform(transform_msg);
    transform_pub_->publish(transform_msg);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoStereo>());
  rclcpp::shutdown();
  return 0;
}
