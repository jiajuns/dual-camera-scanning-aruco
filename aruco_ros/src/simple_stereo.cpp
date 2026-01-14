/**
 * @file simple_stereo.cpp
 * @date January 2026
 * @brief 使用双目相机进行 AprilTag(tag25h9) 标记检测和 3D 定位的 ROS 2 节点。
 * 已修复阈值震荡导致的画面闪烁问题 (Hysteresis & Processing Order Fix).
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
 #include <opencv2/highgui.hpp> // 包含 resize 等
 
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
 
 // AprilTag (tag25h9)
 #include <apriltag/apriltag.h>
 #include <apriltag/tag25h9.h>
 #include <apriltag/apriltag_pose.h>
 
 // UDP sender
 #include "aruco_ros/udp_sender.hpp"
 #include "aruco_ros/serial_sender.hpp"
 
 class ArucoStereo : public rclcpp::Node
 {
 public:
   ArucoStereo()
   : Node("aruco_stereo")
   {
     // 参数
     this->declare_parameter<double>("marker_size", 0.05);   
     this->declare_parameter<int>("marker_id", -1);          
     this->declare_parameter<std::string>("reference_frame", "");
     this->declare_parameter<std::string>("camera_frame", "");
     this->declare_parameter<bool>("image_is_rectified", true);
     this->declare_parameter<double>("baseline", 0.05);      
     this->declare_parameter<double>("debug_axis_length", 0.05); 
     
     // 通信参数
     this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
     this->declare_parameter<int>("baud_rate", 115200);
     this->declare_parameter<double>("udp_send_rate_hz", 0.0);
 
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
 
     std::string serial_port;
     int baud_rate;
     this->get_parameter("serial_port", serial_port);
     this->get_parameter("baud_rate", baud_rate);
     this->get_parameter("udp_send_rate_hz", udp_send_rate_hz_);
     if (udp_send_rate_hz_ > 0.0) {
       udp_send_min_interval_ns_ = static_cast<uint64_t>(1e9 / udp_send_rate_hz_);
     } else {
       udp_send_min_interval_ns_ = 0;
     }
     serial_sender_ = std::make_unique<aruco_ros::SerialSender>(serial_port, baud_rate);
 
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
 
     RCLCPP_INFO(this->get_logger(), "AprilTag(tag25h9) stereo node starting...");
     RCLCPP_INFO(this->get_logger(), "Marker size: %.3f m", marker_size_);
     RCLCPP_INFO(this->get_logger(), "Debug topic: ~/debug");
 
     // 初始化 AprilTag detector + family(tag25h9)
     tag_family_ = tag25h9_create();
     
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
 
     // TF 广播器
     tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
 
     // 相机内参订阅
     left_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
       "/left/camera_info", rclcpp::SensorDataQoS(),
       std::bind(&ArucoStereo::leftInfoCallback, this, std::placeholders::_1));
     right_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
       "/right/camera_info", rclcpp::SensorDataQoS(),
       std::bind(&ArucoStereo::rightInfoCallback, this, std::placeholders::_1));
 
     // message_filters 同步左右图像
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
       tag25h9_destroy(tag_family_);
       tag_family_ = nullptr;
     }
   }
 
 private:
  typedef message_filters::sync_policies::ExactTime<
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
   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
 
   std::unique_ptr<aruco_ros::SerialSender> serial_sender_;
   uint32_t packet_seq_{0};
   double udp_send_rate_hz_{0.0};
   uint64_t udp_send_min_interval_ns_{0};
   uint64_t last_udp_send_ts_ns_{0};
 
   // AprilTag detector
   apriltag_family_t* tag_family_{nullptr};
   apriltag_detector_t* tag_detector_left_{nullptr};
   apriltag_detector_t* tag_detector_right_{nullptr};
 
   // 图像增强状态控制 (解决闪烁的核心变量)
   bool motion_mode_active_{false};
   bool low_light_mode_active_{false};
   
   // 运动模糊对抗缓存
   cv::Mat prev_gray_left_;
   cv::Mat prev_gray_right_;
   bool enable_motion_deblur_{true};
   double motion_threshold_{5.0}; 
 
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
 
   std::vector<TagDetection2D> detectTags(const cv::Mat& gray, apriltag_detector_t* detector)
   {
     std::vector<TagDetection2D> out;
     if (!detector) return out;
     if (gray.empty()) return out;
     
     // AprilTag 要求 uint8
     if (gray.type() != CV_8UC1) return out;
 
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
 
   // 运动模糊检测
   double detectMotionBlur(const cv::Mat& current, const cv::Mat& previous) {
     if (previous.empty() || current.size() != previous.size()) {
         return 0.0;
     }
     
     cv::Mat edges_curr, edges_prev;
     cv::Canny(current, edges_curr, 50, 150);
     cv::Canny(previous, edges_prev, 50, 150);
     
     int prev_edge_pixels = cv::countNonZero(edges_prev);
     if (prev_edge_pixels < 50) return 0.0; 
     
     cv::Mat edge_diff;
     cv::absdiff(edges_curr, edges_prev, edge_diff);
     
     cv::Mat edge_mask;
     cv::bitwise_or(edges_curr, edges_prev, edge_mask);
     
     int edge_pixels = cv::countNonZero(edge_mask);
     if (edge_pixels < 100) return 0.0;
     
     cv::Scalar mean_diff = cv::mean(edge_diff, edge_mask);
     return mean_diff[0];
   }
 
   // 自适应锐化
   void adaptiveSharpen(cv::Mat& gray, double motion_level) {
     // 限制最大锐化强度，防止噪点爆炸
     double sharpen_strength = std::min(5.0, 1.5 + motion_level / 10.0);
     
     cv::Mat blurred;
     cv::GaussianBlur(gray, blurred, cv::Size(0, 0), 0.8);
     cv::addWeighted(gray, 1.0 + sharpen_strength, blurred, -sharpen_strength, 0, gray);
   }
 
   // 边缘增强
   void enhanceEdges(cv::Mat& gray) {
     cv::Mat edges;
     cv::Laplacian(gray, edges, CV_16S, 3);
     cv::convertScaleAbs(edges, edges);
     cv::addWeighted(gray, 1.0, edges, 0.3, 0, gray);
   }
  void syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& left_img_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& right_img_msg)
  {
    // ---------------------------------------------------------
    // 0. 数据获取与线程安全
    // ---------------------------------------------------------
    sensor_msgs::msg::CameraInfo::ConstSharedPtr left_info_msg;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr right_info_msg;
    {
      // 锁定互斥量，安全获取最新的相机内参
      std::lock_guard<std::mutex> lk(cam_info_mutex_);
      left_info_msg = last_left_info_;
      right_info_msg = last_right_info_;
    }
    
    // 如果还没收到内参，跳过处理
    if (!left_info_msg) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "No /left/camera_info received yet, skipping.");
      return;
    }
  
    // ---------------------------------------------------------
    // 1. ROS 图像消息转换为 OpenCV 格式
    // ---------------------------------------------------------
    cv_bridge::CvImagePtr left_cv_ptr, right_cv_ptr;
    try {
      // 强制转换为 BGR8 格式
      left_cv_ptr = cv_bridge::toCvCopy(left_img_msg, sensor_msgs::image_encodings::BGR8);
      right_cv_ptr = cv_bridge::toCvCopy(right_img_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  
    // ---------------------------------------------------------
    // 2. 提取相机参数
    // ---------------------------------------------------------
    // 使用 nonzeroOr 辅助函数优先取 P 矩阵 (Projection)，如果为0则取 K 矩阵 (Intrinsic)
    const double fx = nonzeroOr(left_info_msg->p[0], left_info_msg->k[0]);
    const double fy = nonzeroOr(left_info_msg->p[5], left_info_msg->k[4]);
    const double cx = nonzeroOr(left_info_msg->p[2], left_info_msg->k[2]);
    const double cy = nonzeroOr(left_info_msg->p[6], left_info_msg->k[5]);
    
    // 【注意】这里直接使用了参数中的 baseline_。
    // 确保 launch 文件中配置了正确的基线值 (例如 0.05 或 0.12)
    const double baseline = baseline_; 
  
    if (!std::isfinite(fx) || fx <= 1e-9) {
      RCLCPP_ERROR(this->get_logger(), "Invalid fx from CameraInfo.");
      return;
    }
  
    // ---------------------------------------------------------
    // 3. 图像预处理 (核心修复逻辑)
    // ---------------------------------------------------------
    
    // 3.1 转换为灰度图 (AprilTag 检测的基础)
    cv::Mat gray_left, gray_right;
    cv::cvtColor(left_cv_ptr->image, gray_left, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_cv_ptr->image, gray_right, cv::COLOR_BGR2GRAY);
  
    // ==========================================
    // 【核心修复 A】：分离“计算源”与“处理目标”
    // 问题根源：以前是边修改图像边计算指标，处理后的图像噪点会导致下一帧误判为运动，造成死循环。
    // 解决方案：必须基于【纯净的原始图像】计算亮度和运动指标。
    // ==========================================
    
    // 深拷贝一份原始灰度图，专门用于指标计算，绝对不修改它
    cv::Mat raw_left = gray_left.clone();
    cv::Mat raw_right = gray_right.clone();
  
    // A. 计算亮度 (基于原始灰度)
    cv::Scalar mean_brightness_scalar = cv::mean(raw_left);
    double raw_brightness = mean_brightness_scalar[0];
  
    // B. 计算运动程度 (基于原始灰度)
    double motion_left = 0.0, motion_right = 0.0;
    if (enable_motion_deblur_) {
       // 计算当前原始帧与上一帧原始帧的差异
       motion_left = detectMotionBlur(raw_left, prev_gray_left_);
       motion_right = detectMotionBlur(raw_right, prev_gray_right_);
       
       // 更新上一帧缓存 (必须存原始图，否则会将上一帧的锐化噪点带入计算)
       prev_gray_left_ = raw_left;
       prev_gray_right_ = raw_right;
    }
    double avg_motion = (motion_left + motion_right) / 2.0;
  
    // ==========================================
    // 【核心修复 B】：滞后阈值控制 (Hysteresis / 施密特触发器逻辑)
    // 问题根源：单一阈值会导致数值在临界点（如亮度70）附近时，模式疯狂切换，导致画面闪烁。
    // 解决方案：设置“开启阈值”和“关闭阈值”，形成缓冲区。
    // ==========================================
    
    // --- 1. 运动模式状态机 ---
    // 假设阈值是 5.0。 
    // 进入条件：运动 > 5.0 (剧烈运动)
    // 退出条件：运动 < 3.0 (恢复平稳) -> 3.0到5.0之间保持原状态，不切换
    const double motion_high_thresh = motion_threshold_;       
    const double motion_low_thresh  = std::max(1.0, motion_threshold_ - 2.0); 
  
    if (motion_mode_active_) {
        // 如果已经在运动模式，只有当运动很小（确实停下来了）才退出
        if (avg_motion < motion_low_thresh) motion_mode_active_ = false;
    } else {
        // 如果不在运动模式，只有当运动很大（确实在动）才进入
        if (avg_motion > motion_high_thresh) motion_mode_active_ = true;
    }
  
    // --- 2. 低光照模式状态机 ---
    // 进入条件：亮度 < 70 (太暗了)
    // 退出条件：亮度 > 85 (变亮了) -> 70到85之间保持原状态
    const double brightness_enter = 70.0; 
    const double brightness_exit = 85.0;  
  
    if (low_light_mode_active_) {
        if (raw_brightness > brightness_exit) low_light_mode_active_ = false;
    } else {
        if (raw_brightness < brightness_enter) low_light_mode_active_ = true;
    }
  
    // ==========================================
    // 4. 执行图像增强 (仅修改 gray_left/right)
    // ==========================================
  
    // 4.1 应用运动去模糊 (CLAHE + 锐化)
    // 只有在状态机判定为“运动模式”时才执行
    if (motion_mode_active_) {
       double strength = avg_motion; 
       
       // 自适应锐化：增强边缘
       adaptiveSharpen(gray_left, strength);
       adaptiveSharpen(gray_right, strength);
       // 边缘增强：拉普拉斯算子叠加
       enhanceEdges(gray_left);
       enhanceEdges(gray_right);
  
       // CLAHE (限制对比度自适应直方图均衡)：提升局部对比度
       // 使用较小的 clipLimit (1.5) 以避免过度放大噪点
       cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(1.5, cv::Size(8, 8));
       clahe->apply(gray_left, gray_left);
       clahe->apply(gray_right, gray_right);
       
       // 限流打印日志，避免刷屏
       RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
           "Motion Mode ACTIVE (Motion: %.2f)", avg_motion);
    }
  
    // 4.2 应用低光照增强 (Gamma 变换)
    // 只有在状态机判定为“低光模式”时才执行
    if (low_light_mode_active_) {
       cv::Mat lut(1, 256, CV_8U);
       double gamma = 0.7; // Gamma < 1 提亮暗部
       for (int i = 0; i < 256; i++) {
           lut.at<uchar>(i) = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
       }
       // 查表法快速应用 Gamma
       cv::LUT(gray_left, lut, gray_left);
       cv::LUT(gray_right, lut, gray_right);
       
       RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
           "Low Light Mode ACTIVE (Bright: %.2f)", raw_brightness);
    }
  
    // ==========================================
    // 5. 准备调试图像 (可视化算法效果)
    // ==========================================
    const bool publish_debug = debug_pub_.getNumSubscribers() > 0;
    cv::Mat debug_img;
    
    if (publish_debug) {
        // 关键：这里使用的是【处理后】的 gray_left，转回 BGR 显示
        // 这样你在 RQT 里看到的就是算法“眼中”的图像（比如锐化后的样子）
        cv::cvtColor(gray_left, debug_img, cv::COLOR_GRAY2BGR);
        
        // 在画面左上角打印当前模式，方便调试
        std::string status = "";
        if (motion_mode_active_) status += "[Motion] ";
        if (low_light_mode_active_) status += "[LowLight] ";
        
        if (!status.empty()) {
            cv::putText(debug_img, status, cv::Point(10, 30), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }
    }
  
    // ---------------------------------------------------------
    // 6. AprilTag 并行检测
    // ---------------------------------------------------------
    // 使用 std::async 启动两个异步任务，同时检测左右眼
    auto future_left = std::async(std::launch::async, [this, &gray_left]() {
        return this->detectTags(gray_left, this->tag_detector_left_);
    });
    auto future_right = std::async(std::launch::async, [this, &gray_right]() {
        return this->detectTags(gray_right, this->tag_detector_right_);
    });
  
    // 等待检测完成并获取结果
    auto dets_left = future_left.get();
    auto dets_right = future_right.get();
  
    // ---------------------------------------------------------
    // 7. 双目立体匹配
    // ---------------------------------------------------------
    
    // 建立右眼检测结果的哈希表，方便按 ID 快速查找
    std::unordered_map<int, const TagDetection2D*> right_by_id;
    right_by_id.reserve(dets_right.size());
    for (const auto& dr : dets_right) {
      right_by_id.emplace(dr.id, &dr);
    }
  
    // 准备 PoseArray 消息
    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header = left_img_msg->header;
    if (!camera_frame_.empty()) {
      pose_array_msg.header.frame_id = camera_frame_;
    }
  
    std::vector<DetectionResult> frame_results;
  
    // 遍历左眼检测到的每个 Tag
    for (const auto& dl : dets_left) {
      // 过滤 ID (如果设置了 marker_id 参数)
      if (marker_id_ != -1 && dl.id != marker_id_) continue;
  
      // 在右眼结果中查找相同 ID
      const auto it = right_by_id.find(dl.id);
      
      // 【双目匹配策略】目前代码要求左右眼都必须看到
      // (如果需要优化远距离，可在此处添加 else 分支进行单目 PnP 解算)
      if (it == right_by_id.end()) continue;
  
      // 在调试图上画红框
      if (publish_debug) drawDetection(debug_img, dl, cv::Scalar(0, 0, 255));
  
      // 计算 3D 坐标
      processStereoAprilTag(dl.id, dl.corners, it->second->corners,
                            fx, fy, cx, cy, baseline, left_img_msg->header, 
                            publish_debug ? &debug_img : nullptr,
                            pose_array_msg, frame_results);
    }
  
    // 发布 PoseArray
    if (!pose_array_msg.poses.empty()) {
      poses_pub_->publish(pose_array_msg);
    }
  
    // ---------------------------------------------------------
    // 8. 结果输出与串口/UDP 通信
    // ---------------------------------------------------------
    
    // 日志打印 (仅当检测到2个以上 Tag 时)
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
    }
  
    // 发送数据到下位机 (UDP/Serial)
    uint64_t ts_ns = static_cast<uint64_t>(left_img_msg->header.stamp.sec) * 1000000000ULL +
                      static_cast<uint64_t>(left_img_msg->header.stamp.nanosec);
    bool should_send = true;
    
    // 限频控制 (防止发送太快淹没下位机)
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
      serial_sender_->sendData(tags_data, packet_seq_++, ts_ns);
    }
  
    // 发布最终的调试图像
    if (publish_debug) {
      cv_bridge::CvImage out;
      out.header = left_img_msg->header;
      out.encoding = sensor_msgs::image_encodings::BGR8;
      out.image = debug_img;
      debug_pub_.publish(out.toImageMsg());
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
     // 5) 三角测量
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
 
     // 6) 姿态构建
     tf2::Vector3 p0(corners_3d[0].x, corners_3d[0].y, corners_3d[0].z);
     tf2::Vector3 p1(corners_3d[1].x, corners_3d[1].y, corners_3d[1].z);
     tf2::Vector3 p2(corners_3d[2].x, corners_3d[2].y, corners_3d[2].z);
     tf2::Vector3 p3(corners_3d[3].x, corners_3d[3].y, corners_3d[3].z);
 
     // v_y: Down
     tf2::Vector3 v_y = (p0 - p3) + (p1 - p2); 
     v_y.normalize();
 
     // v_x: Right
     tf2::Vector3 v_x = (p1 - p0) + (p2 - p3);
     v_x.normalize();
 
     tf2::Vector3 v_z = v_x.cross(v_y);
     v_z.normalize();
 
     v_y = v_z.cross(v_x);
     v_y.normalize();
     
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
 
     results.push_back({id, center_3d, q});
 
     // 发布单体 Pose 和 TF
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