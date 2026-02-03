/**
 * @file visualizer_node_v3.cpp
 * @brief 前哨站预测 V3 专用可视化节点
 * 
 * 改进点：
 * 1. 使用message_filters进行时间同步，解决检测框滞后问题
 * 2. 订阅V3预测器的话题
 * 3. 正确投影预测位置到图像
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Message filters for time synchronization
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "armor_detector_ros2/msg/armor_pose_array.hpp"
#include "armor_detector_ros2/msg/armor_b_box_array.hpp"

#include <chrono>
#include <deque>
#include <mutex>
#include <cmath>

// 类型别名
using ArmorBBoxArray = armor_detector_ros2::msg::ArmorBBoxArray;
using ArmorPoseArray = armor_detector_ros2::msg::ArmorPoseArray;

// Sync policy for 3 topics (image, detections, poses)
using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    ArmorBBoxArray,
    ArmorPoseArray>;

/**
 * @brief V3 前哨站可视化节点
 * 
 * 使用 message_filters 时间戳同步，确保图像和检测结果精确匹配
 */
class VisualizerNodeV3 : public rclcpp::Node
{
public:
    VisualizerNodeV3()
    : Node("visualizer_node_v3")
    {
        // 声明参数
        this->declare_parameter("output_path", "/tmp/outpost_v3_result.avi");
        this->declare_parameter("output_fps", 30.0);
        this->declare_parameter("show_predictor_info", true);
        this->declare_parameter("show_aim_point", true);
        this->declare_parameter("show_trajectory", true);
        this->declare_parameter("show_window", false);
        this->declare_parameter("sync_tolerance_ms", 50.0);  // 时间同步容差(毫秒)
        
        // 相机内参配置路径（优先使用YAML文件）
        this->declare_parameter("camera_intrinsics_path", "");
        // 相机内参（fallback默认值）
        this->declare_parameter("camera.fx", 1280.0);
        this->declare_parameter("camera.fy", 1024.0);
        this->declare_parameter("camera.cx", 640.0);
        this->declare_parameter("camera.cy", 512.0);
        
        // 获取参数
        output_video_path_ = this->get_parameter("output_path").as_string();
        fps_ = this->get_parameter("output_fps").as_double();
        show_predictor_info_ = this->get_parameter("show_predictor_info").as_bool();
        show_aim_point_ = this->get_parameter("show_aim_point").as_bool();
        show_trajectory_ = this->get_parameter("show_trajectory").as_bool();
        show_window_ = this->get_parameter("show_window").as_bool();
        sync_tolerance_ms_ = this->get_parameter("sync_tolerance_ms").as_double();
        
        // 加载相机内参
        std::string camera_intrinsics_path = this->get_parameter("camera_intrinsics_path").as_string();
        if (!camera_intrinsics_path.empty() && loadCameraIntrinsicsFromYAML(camera_intrinsics_path)) {
            RCLCPP_INFO(this->get_logger(), "相机内参已从YAML加载: %s", camera_intrinsics_path.c_str());
        } else {
            // fallback: 使用参数
            fx_ = this->get_parameter("camera.fx").as_double();
            fy_ = this->get_parameter("camera.fy").as_double();
            cx_ = this->get_parameter("camera.cx").as_double();
            cy_ = this->get_parameter("camera.cy").as_double();
            RCLCPP_WARN(this->get_logger(), "使用参数中的相机内参: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", 
                       fx_, fy_, cx_, cy_);
        }
        
        // =====================================================================
        // 订阅话题 (message_filters 时间戳同步)
        // =====================================================================
        
        // 使用 message_filters 订阅器进行时间同步
        image_sub_.subscribe(this, "/video/image_raw");
        detection_sub_.subscribe(this, "/detector/armors");
        pose_sub_.subscribe(this, "/solver/armor_poses");
        
        // 创建同步器 (允许 50ms 时间差，队列大小 10)
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), image_sub_, detection_sub_, pose_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_tolerance_ms_ / 1000.0));
        sync_->registerCallback(
            std::bind(&VisualizerNodeV3::syncCallback, this,
                     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        
        // V3 预测器输出
        aim_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/predictor_v3/aim_point", 10,
            std::bind(&VisualizerNodeV3::aimPointCallback, this, std::placeholders::_1));
        
        aim_angles_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/predictor_v3/aim_angles", 10,
            std::bind(&VisualizerNodeV3::aimAnglesCallback, this, std::placeholders::_1));
        
        should_shoot_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/predictor_v3/should_shoot", 10,
            std::bind(&VisualizerNodeV3::shouldShootCallback, this, std::placeholders::_1));
        
        state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/predictor_v3/state_text", 10,
            std::bind(&VisualizerNodeV3::stateCallback, this, std::placeholders::_1));
        
        aim_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/predictor_v3/aim_mode", 10,
            std::bind(&VisualizerNodeV3::aimModeCallback, this, std::placeholders::_1));
        
        // 详细状态 (包含装甲板类型、中心位置等)
        detail_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/predictor_v3/detail", 10,
            std::bind(&VisualizerNodeV3::detailCallback, this, std::placeholders::_1));
        
        // 超时检测
        timeout_timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&VisualizerNodeV3::timeoutCallback, this));
        
        last_image_time_ = std::chrono::steady_clock::now();
        
        RCLCPP_INFO(this->get_logger(), "V3可视化节点已启动");
        RCLCPP_INFO(this->get_logger(), "  输出: %s", output_video_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  时间同步容差: %.1f ms", sync_tolerance_ms_);
    }

    ~VisualizerNodeV3()
    {
        if (video_writer_.isOpened()) {
            video_writer_.release();
            RCLCPP_INFO(this->get_logger(), "视频已保存: %s (%d帧)", 
                       output_video_path_.c_str(), frame_count_);
        }
    }

private:
    // =========================================================================
    // 相机内参加载函数
    // =========================================================================
    
    /**
     * @brief 从YAML文件加载相机内参
     * @param yaml_path YAML配置文件路径
     * @return 加载成功返回true
     */
    bool loadCameraIntrinsicsFromYAML(const std::string& yaml_path)
    {
        try {
            cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
            if (!fs.isOpened()) {
                RCLCPP_WARN(this->get_logger(), "无法打开相机配置文件: %s", yaml_path.c_str());
                return false;
            }
            
            cv::Mat camera_matrix;
            
            // 支持多种YAML格式
            if (fs["camera_matrix"].isMap()) {
                fs["camera_matrix"] >> camera_matrix;
            } else if (fs["K"].isMap() || fs["K"].isSeq()) {
                fs["K"] >> camera_matrix;
            } else if (fs["intrinsic_matrix"].isMap()) {
                fs["intrinsic_matrix"] >> camera_matrix;
            }
            
            fs.release();
            
            if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
                RCLCPP_WARN(this->get_logger(), "相机内参矩阵无效");
                return false;
            }
            
            camera_matrix.convertTo(camera_matrix, CV_64FC1);
            fx_ = camera_matrix.at<double>(0, 0);
            fy_ = camera_matrix.at<double>(1, 1);
            cx_ = camera_matrix.at<double>(0, 2);
            cy_ = camera_matrix.at<double>(1, 2);
            
            RCLCPP_INFO(this->get_logger(), "相机内参: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                       fx_, fy_, cx_, cy_);
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "加载相机配置异常: %s", e.what());
            return false;
        }
    }
    
    // =========================================================================
    // 回调函数 - 时间戳同步
    // =========================================================================
    
    /**
     * @brief 同步回调：图像、检测结果、解算结果三者时间戳匹配时触发
     */
    void syncCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const ArmorBBoxArray::ConstSharedPtr& detection_msg,
        const ArmorPoseArray::ConstSharedPtr& pose_msg)
    {
        last_image_time_ = std::chrono::steady_clock::now();
        
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge错误: %s", e.what());
            return;
        }
        
        // 获取图像时间戳
        double img_timestamp = image_msg->header.stamp.sec + image_msg->header.stamp.nanosec * 1e-9;
        
        // 初始化视频写入器
        if (!video_writer_.isOpened()) {
            initVideoWriter(frame.size());
            // 更新相机参数 (如果使用图像中心作为主点)
            if (cx_ < 1.0) cx_ = frame.cols / 2.0;
            if (cy_ < 1.0) cy_ = frame.rows / 2.0;
        }
        
        // =====================================================================
        // 绘制可视化内容（使用同步的检测和位姿结果）
        // =====================================================================
        
        // 1. 绘制时间同步的检测框
        drawDetections(frame, detection_msg);
        
        // 2. 绘制预测器状态信息
        if (show_predictor_info_) {
            drawPredictorInfo(frame);
        }
        
        // 3. 绘制瞄准点
        if (show_aim_point_) {
            drawAimPoint(frame);
        }
        
        // 4. 绘制轨迹
        if (show_trajectory_) {
            drawTrajectory(frame);
        }
        
        // 5. 绘制射击指示器
        drawShootIndicator(frame);
        
        // 6. 绘制前哨站模型 (中心点和所有装甲板)
        drawOutpostModel(frame);
        
        // 帧信息
        char info[128];
        snprintf(info, sizeof(info), "Frame: %d | t=%.3fs", frame_count_, img_timestamp);
        cv::putText(frame, info, cv::Point(10, frame.rows - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        
        // 写入视频
        video_writer_.write(frame);
        frame_count_++;
        
        if (frame_count_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "已写入 %d 帧", frame_count_);
        }
        
        // 实时显示
        if (show_window_) {
            cv::imshow("Outpost V3 Visualization", frame);
            cv::waitKey(1);
        }
    }
    
    void aimPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        latest_aim_point_ = msg;
        aim_point_received_ = true;
        
        // 记录轨迹
        if (trajectory_points_.size() >= 200) {
            trajectory_points_.pop_front();
        }
        trajectory_points_.push_back(cv::Point3d(msg->point.x, msg->point.y, msg->point.z));
    }
    
    void aimAnglesCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        if (msg->data.size() >= 2) {
            latest_aim_yaw_ = msg->data[0];
            latest_aim_pitch_ = msg->data[1];
            aim_angles_received_ = true;
        }
    }
    
    void shouldShootCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        should_shoot_ = msg->data;
    }
    
    void stateCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        predictor_state_ = msg->data;
    }
    
    void aimModeCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        aim_mode_ = msg->data;
    }
    
    void detailCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        if (msg->data.size() >= 11) {
            // 解析详细状态: [theta, omega, x_c, y_c, z_c, direction, converged, best_type, conf_h, conf_m, conf_l]
            detail_theta_ = msg->data[0];
            detail_omega_ = msg->data[1];
            detail_center_x_ = msg->data[2];
            detail_center_y_ = msg->data[3];
            detail_center_z_ = msg->data[4];
            detail_direction_ = static_cast<int>(msg->data[5]);
            detail_converged_ = (msg->data[6] > 0.5);
            detail_best_type_ = static_cast<int>(msg->data[7]);
            detail_conf_h_ = msg->data[8];
            detail_conf_m_ = msg->data[9];
            detail_conf_l_ = msg->data[10];
            detail_received_ = true;
        }
    }
    
    void timeoutCallback()
    {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_image_time_);
        
        if (duration.count() >= 3 && frame_count_ > 0 && !finished_) {
            finished_ = true;
            if (video_writer_.isOpened()) {
                video_writer_.release();
                RCLCPP_INFO(this->get_logger(), "视频流结束，已保存: %s", output_video_path_.c_str());
            }
            rclcpp::shutdown();
        }
    }
    
    // =========================================================================
    // 绘图函数
    // =========================================================================
    
    /**
     * @brief 绘制检测框 (message_filters 同步版本)
     */
    void drawDetections(cv::Mat& frame, 
                        const ArmorBBoxArray::ConstSharedPtr& detections)
    {
        if (!detections) return;
        for (const auto& armor : detections->armors) {
            // 根据颜色选择绘制颜色
            cv::Scalar color;
            if (armor.color_id == 0) {
                color = cv::Scalar(255, 100, 0);  // 蓝色装甲板
            } else if (armor.color_id == 1) {
                color = cv::Scalar(0, 100, 255);  // 红色装甲板
            } else {
                color = cv::Scalar(0, 255, 255);  // 其他
            }
            
            // 绘制四个角点连线
            std::vector<cv::Point> corners;
            for (int i = 0; i < 4; i++) {
                corners.push_back(cv::Point(
                    static_cast<int>(armor.corners[i].x),
                    static_cast<int>(armor.corners[i].y)
                ));
            }
            
            // 绘制边框
            for (int i = 0; i < 4; i++) {
                cv::line(frame, corners[i], corners[(i + 1) % 4], color, 2);
            }
            
            // 绘制角点
            for (int i = 0; i < 4; i++) {
                cv::circle(frame, corners[i], 4, color, -1);
            }
            
            // 绘制中心点
            cv::Point center(static_cast<int>(armor.center.x),
                            static_cast<int>(armor.center.y));
            cv::circle(frame, center, 5, color, -1);
            
            // 显示置信度和类别
            char label[64];
            snprintf(label, sizeof(label), "%.2f #%d", armor.confidence, armor.tag_id);
            cv::putText(frame, label, cv::Point(center.x - 30, center.y - 15),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }
    }
    
    /**
     * @brief 绘制预测器状态信息
     */
    void drawPredictorInfo(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        
        int y = 30;
        const int line_height = 25;
        
        // 标题
        cv::putText(frame, "=== Outpost Predictor V3 ===", cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        y += line_height + 5;
        
        // 状态
        cv::Scalar state_color = (predictor_state_ == "TRACKING") ? 
                                  cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
        cv::putText(frame, "State: " + predictor_state_, cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, state_color, 1);
        y += line_height;
        
        // 瞄准模式
        std::string mode_display = (aim_mode_ == "DIRECT") ? "DIRECT (current)" : 
                                   (aim_mode_ == "INDIRECT") ? "INDIRECT (next)" : aim_mode_;
        cv::Scalar mode_color = (aim_mode_ == "DIRECT") ? 
                                 cv::Scalar(0, 255, 0) : cv::Scalar(255, 165, 0);
        cv::putText(frame, "Mode: " + mode_display, cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, mode_color, 1);
        y += line_height;
        
        // 瞄准角度
        if (aim_angles_received_) {
            char angles[128];
            double yaw_deg = latest_aim_yaw_ * 180.0 / M_PI;
            double pitch_deg = latest_aim_pitch_ * 180.0 / M_PI;
            snprintf(angles, sizeof(angles), "Aim: yaw=%.2f° pitch=%.2f°", yaw_deg, pitch_deg);
            cv::putText(frame, angles, cv::Point(20, y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
            y += line_height;
        }
        
        // 瞄准点位置
        if (latest_aim_point_) {
            char pos[128];
            snprintf(pos, sizeof(pos), "Target: (%.2f, %.2f, %.2f)m",
                    latest_aim_point_->point.x,
                    latest_aim_point_->point.y,
                    latest_aim_point_->point.z);
            cv::putText(frame, pos, cv::Point(20, y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 180, 180), 1);
            y += line_height;
        }
        
        // 显示装甲板类型和假设置信度
        if (detail_received_) {
            // 装甲板类型
            std::string type_str;
            cv::Scalar type_color;
            if (detail_converged_) {
                switch (detail_best_type_) {
                    case 0: type_str = "HIGH"; type_color = cv::Scalar(0, 200, 255); break;   // 橙色
                    case 1: type_str = "MIDDLE"; type_color = cv::Scalar(0, 255, 0); break;  // 绿色
                    case 2: type_str = "LOW"; type_color = cv::Scalar(255, 100, 100); break; // 蓝色
                    default: type_str = "UNKNOWN"; type_color = cv::Scalar(128, 128, 128); break;
                }
                cv::putText(frame, "Armor: " + type_str, cv::Point(20, y),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, type_color, 2);
            } else {
                cv::putText(frame, "Armor: UNCERTAIN", cv::Point(20, y),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(128, 128, 128), 1);
            }
            y += line_height;
            
            // 假设置信度
            char conf_str[128];
            snprintf(conf_str, sizeof(conf_str), "H:%.1f%% M:%.1f%% L:%.1f%%",
                    detail_conf_h_ * 100, detail_conf_m_ * 100, detail_conf_l_ * 100);
            cv::putText(frame, conf_str, cv::Point(20, y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 180, 180), 1);
            y += line_height;
            
            // 中心位置
            char center_str[128];
            snprintf(center_str, sizeof(center_str), "Center: (%.2f, %.2f, %.2f)m",
                    detail_center_x_, detail_center_y_, detail_center_z_);
            cv::putText(frame, center_str, cv::Point(20, y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 180, 180), 1);
            y += line_height;
            
            // 角速度和相位
            char motion_str[128];
            snprintf(motion_str, sizeof(motion_str), "Omega: %.2f rad/s  Phase: %.1f deg",
                    detail_omega_, detail_theta_ * 180.0 / M_PI);
            cv::putText(frame, motion_str, cv::Point(20, y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 180, 180), 1);
        }
    }
    
    /**
     * @brief 绘制瞄准点 (投影到图像)
     * 
     * 机架坐标系: X右、Y上、Z后（机头前方是-Z）
     * 相机坐标系: X右、Y下、Z前
     * 转换: X_cam = X_body, Y_cam = -Y_body, Z_cam = -Z_body
     */
    void drawAimPoint(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        
        if (!aim_point_received_ || !latest_aim_point_) {
            return;
        }
        
        // 机架坐标系中的3D点 (X右、Y上、Z后)
        double x_body = latest_aim_point_->point.x;
        double y_body = latest_aim_point_->point.y;
        double z_body = latest_aim_point_->point.z;
        
        // 转换到相机坐标系 (X右、Y下、Z前)
        double x_cam = x_body;
        double y_cam = -y_body;
        double z_cam = -z_body;
        
        // 投影到图像 (针孔相机模型)
        if (z_cam > 0.1) {
            int u = static_cast<int>(fx_ * x_cam / z_cam + cx_);
            int v = static_cast<int>(fy_ * y_cam / z_cam + cy_);
            
            if (u >= 0 && u < frame.cols && v >= 0 && v < frame.rows) {
                // 根据是否可射击选择颜色
                cv::Scalar color = should_shoot_ ? 
                                   cv::Scalar(0, 255, 0) :   // 绿色 - 可射击
                                   cv::Scalar(0, 165, 255);  // 橙色 - 等待
                
                // 绘制十字准星
                int size = 25;
                cv::line(frame, cv::Point(u - size, v), cv::Point(u + size, v), color, 2);
                cv::line(frame, cv::Point(u, v - size), cv::Point(u, v + size), color, 2);
                cv::circle(frame, cv::Point(u, v), size, color, 2);
                cv::circle(frame, cv::Point(u, v), 5, color, -1);
                
                // 标注距离（使用相机坐标系的Z即原来机架的-Z）
                char dist_info[32];
                snprintf(dist_info, sizeof(dist_info), "%.2fm", z_cam);
                cv::putText(frame, dist_info, cv::Point(u + size + 5, v + 5),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
            }
        }
    }
    
    /**
     * @brief 绘制历史轨迹
     * 
     * 机架坐标系: X右、Y上、Z后（机头前方是-Z）
     * 相机坐标系: X右、Y下、Z前
     */
    void drawTrajectory(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        
        if (trajectory_points_.size() < 2) {
            return;
        }
        
        std::vector<cv::Point> projected_points;
        
        for (const auto& pt3d : trajectory_points_) {
            // pt3d 存储的是机架坐标系 (x_body, y_body, z_body)
            // 转换到相机坐标系
            double x_cam = pt3d.x;
            double y_cam = -pt3d.y;  // Y翻转
            double z_cam = -pt3d.z;  // Z翻转
            
            if (z_cam > 0.1) {
                int u = static_cast<int>(fx_ * x_cam / z_cam + cx_);
                int v = static_cast<int>(fy_ * y_cam / z_cam + cy_);
                
                if (u >= 0 && u < frame.cols && v >= 0 && v < frame.rows) {
                    projected_points.push_back(cv::Point(u, v));
                }
            }
        }
        
        // 绘制轨迹线
        for (size_t i = 1; i < projected_points.size(); i++) {
            // 颜色渐变 (旧的点更淡)
            double alpha = static_cast<double>(i) / projected_points.size();
            cv::Scalar color(255 * (1 - alpha), 255 * alpha, 0);
            cv::line(frame, projected_points[i - 1], projected_points[i], color, 1);
        }
    }
    
    /**
     * @brief 绘制前哨站模型 (中心点和所有装甲板位置)
     * @note 输入数据是机架坐标系（X右、Y上、Z后），需要先转换到相机坐标系再投影
     */
    void drawOutpostModel(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        
        // 机架坐标系中，前方是-Z，所以检查-Z是否大于0.5
        if (!detail_received_ || (-detail_center_z_) < 0.5) {
            return;
        }
        
        // 前哨站几何参数
        const double radius = 0.2767;  // 装甲板到中心的距离
        const double height_diff = 0.10;  // 高度差
        const double phase_spacing = 2.0 * M_PI / 3.0;  // 120度
        
        // 三个装甲板的高度偏移 [HIGH, MIDDLE, LOW] (机架坐标系Y向上为正)
        const double height_offsets[3] = {height_diff, 0.0, -height_diff};
        // 相位偏移 [HIGH, MIDDLE, LOW]
        const double phase_offsets[3] = {0.0, -phase_spacing, -2.0 * phase_spacing};
        // 颜色 [HIGH=橙色, MIDDLE=绿色, LOW=蓝色]
        const cv::Scalar colors[3] = {
            cv::Scalar(0, 200, 255),   // HIGH - 橙色
            cv::Scalar(0, 255, 0),     // MIDDLE - 绿色
            cv::Scalar(255, 100, 100)  // LOW - 蓝色
        };
        const std::string names[3] = {"H", "M", "L"};
        
        // 绘制旋转中心（机架坐标系）
        double cx = detail_center_x_;
        double cy = detail_center_y_;
        double cz = detail_center_z_;
        
        // 机架坐标系转相机坐标系：X_cam = X_body, Y_cam = -Y_body, Z_cam = -Z_body
        double cx_cam = cx;
        double cy_cam = -cy;
        double cz_cam = -cz;
        
        if (cz_cam > 0.1) {
            int u_center = static_cast<int>(fx_ * cx_cam / cz_cam + cx_);
            int v_center = static_cast<int>(fy_ * cy_cam / cz_cam + cy_);
            
            if (u_center >= 0 && u_center < frame.cols && v_center >= 0 && v_center < frame.rows) {
                // 绘制旋转中心 (黄色十字)
                cv::drawMarker(frame, cv::Point(u_center, v_center), cv::Scalar(0, 255, 255),
                              cv::MARKER_CROSS, 20, 2);
                cv::putText(frame, "C", cv::Point(u_center + 10, v_center - 10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
            }
        }
        
        // 绘制三个装甲板位置
        for (int i = 0; i < 3; i++) {
            // 计算装甲板在机架坐标系中的位置
            // 机架坐标系：X右、Y上、Z后，前方是-Z
            // phase = 0 时装甲板在前方（-Z方向），所以使用 -cos(phase) 和 -sin(phase)
            double phase = detail_theta_ + phase_offsets[i];
            double armor_x = cx + radius * std::sin(phase);   // X = sin(theta) * r (左右)
            double armor_y = cy + height_offsets[i];           // Y = center_y + height_offset (Y向上为正)
            double armor_z = cz - radius * std::cos(phase);   // Z = center_z - cos(theta) * r (前方是-Z)
            
            // 计算装甲板朝向角 (用于判断是否面向相机)
            // 在机架坐标系中，装甲板法向量指向外侧
            // 法向量在XZ平面上的分量: (sin(phase), -cos(phase)) 指向外侧
            // 视线方向（从相机指向装甲板）在机架坐标系中: (armor_x, armor_y, armor_z)
            // 相机前方是-Z，所以视线主要沿-Z方向
            // 面向相机条件：法向量的-Z分量（即cos(phase)）> 0
            bool facing_camera = std::cos(phase) > 0;
            
            // 机架坐标系转相机坐标系
            double armor_x_cam = armor_x;
            double armor_y_cam = -armor_y;
            double armor_z_cam = -armor_z;
            
            if (armor_z_cam > 0.1) {
                int u = static_cast<int>(fx_ * armor_x_cam / armor_z_cam + cx_);
                int v = static_cast<int>(fy_ * armor_y_cam / armor_z_cam + cy_);
                
                if (u >= 0 && u < frame.cols && v >= 0 && v < frame.rows) {
                    // 根据是否是当前瞄准的装甲板和是否面向相机选择样式
                    bool is_current = detail_converged_ && (i == detail_best_type_);
                    cv::Scalar color = colors[i];
                    
                    if (is_current) {
                        // 当前瞄准的装甲板: 实心大圆
                        cv::circle(frame, cv::Point(u, v), 15, color, -1);
                        cv::circle(frame, cv::Point(u, v), 18, cv::Scalar(255, 255, 255), 2);
                    } else if (facing_camera) {
                        // 面向相机但不是当前瞄准: 实心小圆
                        cv::circle(frame, cv::Point(u, v), 10, color, -1);
                    } else {
                        // 背向相机: 空心圆 + 虚线
                        cv::circle(frame, cv::Point(u, v), 10, color, 2);
                    }
                    
                    // 标注装甲板类型
                    cv::putText(frame, names[i], cv::Point(u - 5, v + 5),
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
                    
                    // 绘制从中心到装甲板的连线
                    if (cz_cam > 0.1) {
                        int u_center = static_cast<int>(fx_ * cx_cam / cz_cam + cx_);
                        int v_center = static_cast<int>(fy_ * cy_cam / cz_cam + cy_);
                        
                        if (u_center >= 0 && u_center < frame.cols && 
                            v_center >= 0 && v_center < frame.rows) {
                            // 面向相机用实线，背向用虚线
                            if (facing_camera) {
                                cv::line(frame, cv::Point(u_center, v_center), 
                                        cv::Point(u, v), color, 1);
                            } else {
                                // 虚线效果
                                drawDashedLine(frame, cv::Point(u_center, v_center),
                                              cv::Point(u, v), color, 1, 10);
                            }
                        }
                    }
                }
            }
        }
    }
    
    /**
     * @brief 绘制虚线
     */
    void drawDashedLine(cv::Mat& frame, cv::Point p1, cv::Point p2, 
                        cv::Scalar color, int thickness, int dash_length)
    {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        int num_dashes = static_cast<int>(dist / (2 * dash_length));
        
        for (int i = 0; i < num_dashes; i++) {
            double t1 = static_cast<double>(i * 2) / (num_dashes * 2);
            double t2 = static_cast<double>(i * 2 + 1) / (num_dashes * 2);
            
            cv::Point start(p1.x + dx * t1, p1.y + dy * t1);
            cv::Point end(p1.x + dx * t2, p1.y + dy * t2);
            cv::line(frame, start, end, color, thickness);
        }
    }
    
    /**
     * @brief 绘制射击指示器
     */
    void drawShootIndicator(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        
        // 在右上角绘制射击状态指示器
        int indicator_x = frame.cols - 150;
        int indicator_y = 50;
        
        if (should_shoot_) {
            // 绿色大圆 + "SHOOT"
            cv::circle(frame, cv::Point(indicator_x + 50, indicator_y), 40, 
                      cv::Scalar(0, 255, 0), -1);
            cv::putText(frame, "SHOOT", cv::Point(indicator_x + 20, indicator_y + 8),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
        } else {
            // 灰色空心圆 + "WAIT"
            cv::circle(frame, cv::Point(indicator_x + 50, indicator_y), 40, 
                      cv::Scalar(100, 100, 100), 3);
            cv::putText(frame, "WAIT", cv::Point(indicator_x + 25, indicator_y + 8),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(100, 100, 100), 2);
        }
    }
    
    // =========================================================================
    // 辅助函数
    // =========================================================================
    
    void initVideoWriter(cv::Size frame_size)
    {
        int fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
        std::string out_path = output_video_path_;
        
        // 确保扩展名为 .avi
        if (out_path.size() > 4 && out_path.substr(out_path.size() - 4) == ".mp4") {
            out_path = out_path.substr(0, out_path.size() - 4) + ".avi";
            output_video_path_ = out_path;
        }
        
        video_writer_.open(out_path, fourcc, fps_, frame_size);
        if (!video_writer_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法创建视频: %s", out_path.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "开始写入视频: %s (%.1f FPS)", out_path.c_str(), fps_);
        }
    }
    

    
    // =========================================================================
    // 成员变量
    // =========================================================================
    
    // Message filters 订阅器 (用于时间同步)
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<ArmorBBoxArray> detection_sub_;
    message_filters::Subscriber<ArmorPoseArray> pose_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    // 普通订阅者 (预测器话题，不需要严格同步)
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr aim_point_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr aim_angles_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr should_shoot_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr aim_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr detail_sub_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    
    // 预测器数据
    std::mutex predictor_mutex_;
    geometry_msgs::msg::PointStamped::SharedPtr latest_aim_point_;
    double latest_aim_yaw_ = 0.0;
    double latest_aim_pitch_ = 0.0;
    bool aim_point_received_ = false;
    bool aim_angles_received_ = false;
    bool should_shoot_ = false;
    std::string predictor_state_ = "INITIALIZING";
    std::string aim_mode_ = "NONE";
    
    // 详细状态
    bool detail_received_ = false;
    double detail_theta_ = 0.0;
    double detail_omega_ = 0.0;
    double detail_center_x_ = 0.0;
    double detail_center_y_ = 0.0;
    double detail_center_z_ = 0.0;
    int detail_direction_ = 0;
    bool detail_converged_ = false;
    int detail_best_type_ = 1;  // 默认MIDDLE
    double detail_conf_h_ = 0.333;
    double detail_conf_m_ = 0.333;
    double detail_conf_l_ = 0.333;
    
    // 轨迹
    std::deque<cv::Point3d> trajectory_points_;
    
    // 视频写入
    cv::VideoWriter video_writer_;
    std::string output_video_path_;
    double fps_;
    int frame_count_ = 0;
    bool finished_ = false;
    
    // 参数
    bool show_predictor_info_;
    bool show_aim_point_;
    bool show_trajectory_;
    bool show_window_;
    double sync_tolerance_ms_;
    
    // 相机内参
    double fx_, fy_, cx_, cy_;
    
    std::chrono::steady_clock::time_point last_image_time_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizerNodeV3>());
    rclcpp::shutdown();
    return 0;
}
