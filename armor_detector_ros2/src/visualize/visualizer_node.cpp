/**
 * @file visualizer_node.cpp
 * @brief 检测结果和位姿解算可视化ROS2节点
 * 
 * 功能：
 * - 接收已绘制的图像
 * - 订阅位姿解算结果并在图像上显示
 * - 订阅 EKF 跟踪状态并可视化（可选）
 * - 写入视频文件
 * 
 * 订阅话题：
 * - /detector/image_with_stamp (sensor_msgs/Image): 已绘制检测结果的图像
 * - /solver/armor_poses (ArmorPoseArray): 位姿解算结果
 * - /tracker/state (std_msgs/String): EKF 跟踪状态（可选）
 * 
 * Requirements: 9.1, 9.2
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>
#include <mutex>
#include <cmath>
#include <deque>

#include "armor_detector_ros2/msg/armor_pose.hpp"
#include "armor_detector_ros2/msg/armor_pose_array.hpp"
#include "armor_detector_ros2/msg/outpost_state.hpp"

using namespace std::chrono_literals;

/**
 * @brief 获取装甲板类别名称
 */
inline std::string getTagName(int tag_id) {
    static const std::vector<std::string> tag_names = {
        "Base", "Hero", "Eng", "Inf3", "Inf4", "Inf5", "Sentry", "Outpost"
    };
    if (tag_id >= 0 && tag_id < static_cast<int>(tag_names.size())) {
        return tag_names[tag_id];
    }
    return "Unknown";
}

/**
 * @brief 获取颜色名称
 */
inline std::string getColorName(int color_id) {
    static const std::vector<std::string> color_names = {"B", "R", "N", "P"};
    if (color_id >= 0 && color_id < static_cast<int>(color_names.size())) {
        return color_names[color_id];
    }
    return "?";
}

/**
 * @brief 获取绘制颜色
 */
inline cv::Scalar getDrawColor(int color_id) {
    switch (color_id) {
        case 0: return cv::Scalar(255, 128, 0);    // 蓝色装甲板 -> 橙色文字
        case 1: return cv::Scalar(0, 255, 255);    // 红色装甲板 -> 黄色文字
        default: return cv::Scalar(0, 255, 0);     // 默认绿色
    }
}

/**
 * @brief EKF 跟踪状态数据结构
 */
struct EKFTrackingState {
    std::string state = "IDLE";           // DETECTING, TRACKING, TEMP_LOST, LOST
    double x = 0.0, y = 0.0, z = 0.0;     // 目标位置
    double vx = 0.0, vy = 0.0, vz = 0.0;  // 目标速度
    double yaw = 0.0, yaw_vel = 0.0;      // yaw 角和角速度
    double radius = 0.0;                   // 装甲板到中心距离
    double nis = 0.0;                      // NIS 值
    double nees = 0.0;                     // NEES 值
    bool valid = false;
};

/**
 * @brief 前哨站预测状态数据结构
 */
struct OutpostPredictionState {
    double center_x = 0.0, center_y = 0.0, center_z = 0.0;  // 旋转中心位置
    double vel_x = 0.0, vel_y = 0.0, vel_z = 0.0;           // 中心速度
    double radius = 0.275;                                   // 旋转半径
    double theta = 0.0;                                      // 当前角度
    double omega = 0.0;                                      // 角速度
    int direction = 0;                                       // 旋转方向
    bool valid = false;
};

/**
 * @class VisualizerNode
 * @brief 可视化节点 - 显示位姿信息、EKF 跟踪状态并写入视频
 */
class VisualizerNode : public rclcpp::Node
{
public:
    VisualizerNode() : Node("visualizer_node")
    {
        // 参数
        this->declare_parameter<std::string>("output_video_path", "output.mp4");
        this->declare_parameter<double>("fps", 30.0);
        this->declare_parameter<bool>("show_pose_info", true);
        this->declare_parameter<bool>("show_ekf_info", false);
        this->declare_parameter<bool>("show_trajectory", false);
        this->declare_parameter<bool>("show_outpost_info", false);
        this->declare_parameter<int>("trajectory_length", 50);
        
        output_video_path_ = this->get_parameter("output_video_path").as_string();
        fps_ = this->get_parameter("fps").as_double();
        show_pose_info_ = this->get_parameter("show_pose_info").as_bool();
        show_ekf_info_ = this->get_parameter("show_ekf_info").as_bool();
        show_trajectory_ = this->get_parameter("show_trajectory").as_bool();
        show_outpost_info_ = this->get_parameter("show_outpost_info").as_bool();
        trajectory_length_ = this->get_parameter("trajectory_length").as_int();

        RCLCPP_INFO(this->get_logger(), "可视化配置:");
        RCLCPP_INFO(this->get_logger(), "  输出视频: %s", output_video_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  帧率: %.2f FPS", fps_);
        RCLCPP_INFO(this->get_logger(), "  显示位姿信息: %s", show_pose_info_ ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "  显示 EKF 信息: %s", show_ekf_info_ ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "  显示轨迹: %s", show_trajectory_ ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "  显示前哨站信息: %s", show_outpost_info_ ? "是" : "否");

        // 订阅已绘制的图像
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/detector/image_with_stamp", 
            rclcpp::QoS(100).reliable(),
            std::bind(&VisualizerNode::image_callback, this, std::placeholders::_1)
        );

        // 订阅位姿解算结果
        pose_sub_ = this->create_subscription<armor_detector_ros2::msg::ArmorPoseArray>(
            "/solver/armor_poses",
            rclcpp::QoS(100).reliable(),
            std::bind(&VisualizerNode::pose_callback, this, std::placeholders::_1)
        );

        // 订阅 EKF 跟踪状态（可选）
        if (show_ekf_info_) {
            tracker_state_sub_ = this->create_subscription<std_msgs::msg::String>(
                "/tracker/state",
                rclcpp::QoS(10).reliable(),
                std::bind(&VisualizerNode::tracker_state_callback, this, std::placeholders::_1)
            );
            
            tracker_data_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/tracker/data",
                rclcpp::QoS(10).reliable(),
                std::bind(&VisualizerNode::tracker_data_callback, this, std::placeholders::_1)
            );
            
            RCLCPP_INFO(this->get_logger(), "已订阅 EKF 跟踪话题");
        }

        // 订阅前哨站预测状态（可选）
        if (show_outpost_info_) {
            outpost_sub_ = this->create_subscription<armor_detector_ros2::msg::OutpostState>(
                "/outpost/prediction",
                rclcpp::QoS(10).reliable(),
                std::bind(&VisualizerNode::outpost_callback, this, std::placeholders::_1)
            );
            
            RCLCPP_INFO(this->get_logger(), "已订阅前哨站预测话题");
        }

        // 超时检测定时器
        timeout_timer_ = this->create_wall_timer(1s, [this]() {
            if (frame_count_ < 10) return;
            
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                now - last_image_time_).count();
            
            if (elapsed > 2 && !finished_) {
                RCLCPP_INFO(this->get_logger(), "检测到视频结束（2秒无新帧）");
                finish_and_exit();
            }
        });

        last_image_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "可视化节点已启动，等待图像和位姿数据...");
    }

    ~VisualizerNode()
    {
        finish_and_exit();
    }

private:
    void finish_and_exit()
    {
        if (!finished_) {
            finished_ = true;
            if (video_writer_.isOpened()) {
                video_writer_.release();
                RCLCPP_INFO(this->get_logger(), 
                    "视频写入完成，共 %d 帧，保存到: %s", 
                    frame_count_, output_video_path_.c_str());
            }
            rclcpp::shutdown();
        }
    }

    /**
     * @brief 位姿数据回调
     */
    void pose_callback(const armor_detector_ros2::msg::ArmorPoseArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_poses_ = msg;
        pose_received_ = true;
        
        // 调试：输出检测到的装甲板位姿，并计算其在图像中的预期位置
        pose_callback_count_++;
        if (pose_callback_count_ % 30 == 1 && !msg->poses.empty()) {
            for (const auto& pose : msg->poses) {
                if (pose.valid && pose.tag_id == 7) {  // 只输出前哨站装甲板
                    // 根据 yaw, pitch, distance 计算 3D 位置
                    double x = pose.distance * std::cos(pose.pitch) * std::sin(-pose.yaw);
                    double y = pose.distance * std::cos(pose.pitch) * std::cos(pose.yaw);
                    double z = pose.distance * std::sin(pose.pitch);
                    
                    // 计算图像坐标
                    double fx = 1280.0, fy = 1280.0;
                    double cx = 640.0, cy = 512.0;
                    double u = cx + fx * (x / y);
                    double v = cy - fy * (z / y);
                    
                    RCLCPP_INFO(this->get_logger(), 
                        "[检测调试] yaw=%.2f pitch=%.2f dist=%.2f -> 3D=(%.2f,%.2f,%.2f) -> 图像=(%.1f,%.1f)",
                        pose.yaw, pose.pitch, pose.distance, x, y, z, u, v);
                }
            }
        }
    }

    /**
     * @brief EKF 跟踪状态回调
     */
    void tracker_state_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);
        ekf_state_.state = msg->data;
        ekf_state_.valid = true;
    }

    /**
     * @brief EKF 跟踪数据回调
     * 数据格式: [x, y, z, vx, vy, vz, yaw, yaw_vel, radius, nis, nees]
     */
    void tracker_data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);
        if (msg->data.size() >= 11) {
            ekf_state_.x = msg->data[0];
            ekf_state_.y = msg->data[1];
            ekf_state_.z = msg->data[2];
            ekf_state_.vx = msg->data[3];
            ekf_state_.vy = msg->data[4];
            ekf_state_.vz = msg->data[5];
            ekf_state_.yaw = msg->data[6];
            ekf_state_.yaw_vel = msg->data[7];
            ekf_state_.radius = msg->data[8];
            ekf_state_.nis = msg->data[9];
            ekf_state_.nees = msg->data[10];
            ekf_state_.valid = true;
            
            // 记录轨迹点
            if (show_trajectory_) {
                cv::Point2d pt(ekf_state_.x * 100 + 640, ekf_state_.z * 100 + 360);  // 简单投影
                trajectory_points_.push_back(pt);
                if (trajectory_points_.size() > static_cast<size_t>(trajectory_length_)) {
                    trajectory_points_.pop_front();
                }
            }
        }
    }

    /**
     * @brief 前哨站预测数据回调
     */
    void outpost_callback(const armor_detector_ros2::msg::OutpostState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(outpost_mutex_);
        outpost_state_.center_x = msg->center.x;
        outpost_state_.center_y = msg->center.y;
        outpost_state_.center_z = msg->center.z;
        outpost_state_.vel_x = msg->velocity.x;
        outpost_state_.vel_y = msg->velocity.y;
        outpost_state_.vel_z = msg->velocity.z;
        outpost_state_.radius = msg->radius;
        outpost_state_.theta = msg->theta;
        outpost_state_.omega = msg->omega;
        outpost_state_.direction = msg->direction;
        outpost_state_.valid = msg->valid;
        
        // 记录前哨站预测轨迹点（基于预测的装甲板位置）
        if (show_trajectory_ && outpost_state_.valid) {
            // 计算预测的装甲板位置（3D 世界坐标）
            // 坐标系：X 向右，Y 向前（深度），Z 向上
            double armor_x = outpost_state_.center_x + outpost_state_.radius * std::sin(outpost_state_.theta);
            double armor_y = outpost_state_.center_y - outpost_state_.radius * std::cos(outpost_state_.theta);
            double armor_z = outpost_state_.center_z;
            
            // 调试输出
            outpost_callback_count_++;
            if (outpost_callback_count_ % 30 == 1) {
                RCLCPP_INFO(this->get_logger(), 
                    "[轨迹调试] center=(%.2f,%.2f,%.2f) theta=%.2f omega=%.2f dir=%d armor_3d=(%.2f,%.2f,%.2f)",
                    outpost_state_.center_x, outpost_state_.center_y, outpost_state_.center_z,
                    outpost_state_.theta, outpost_state_.omega, outpost_state_.direction,
                    armor_x, armor_y, armor_z);
            }
            
            // 使用针孔相机模型投影到图像坐标
            // 假设相机内参：fx=fy=1280, cx=640, cy=512 (1280x1024 图像)
            if (armor_y > 0.1) {  // 避免除以零，且目标必须在相机前方
                double fx = 1280.0, fy = 1280.0;
                double cx = 640.0, cy = 512.0;  // 1280x1024 图像的中心
                
                double u = cx + fx * (armor_x / armor_y);
                double v = cy - fy * (armor_z / armor_y);  // Z 向上，图像 v 向下
                
                if (outpost_callback_count_ % 30 == 1) {
                    RCLCPP_INFO(this->get_logger(), 
                        "[轨迹调试] 投影: u=%.1f, v=%.1f (图像尺寸 1280x1024)",
                        u, v);
                }
                
                cv::Point2d pt(u, v);
                outpost_trajectory_.push_back(pt);
                if (outpost_trajectory_.size() > static_cast<size_t>(trajectory_length_)) {
                    outpost_trajectory_.pop_front();
                }
            }
        }
    }

    /**
     * @brief 在图像上绘制位姿信息
     */
    void drawPoseInfo(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        
        if (!pose_received_ || !latest_poses_) {
            return;
        }

        int y_offset = 60;  // 起始 Y 位置（避开检测器的统计信息）
        int line_height = 22;
        int panel_width = 280;
        int panel_height = 30 + latest_poses_->poses.size() * line_height * 2 + 10;
        
        // 绘制半透明背景面板
        cv::Mat overlay = frame.clone();
        cv::rectangle(overlay, 
                     cv::Point(frame.cols - panel_width - 10, y_offset - 25),
                     cv::Point(frame.cols - 10, y_offset + panel_height),
                     cv::Scalar(0, 0, 0), cv::FILLED);
        cv::addWeighted(overlay, 0.6, frame, 0.4, 0, frame);

        // 绘制标题
        cv::putText(frame, "Pose Solver Results", 
                   cv::Point(frame.cols - panel_width, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        
        // 绘制解算耗时
        char time_info[64];
        snprintf(time_info, sizeof(time_info), "Solve Time: %.2f ms", 
                latest_poses_->solve_time_ms);
        cv::putText(frame, time_info,
                   cv::Point(frame.cols - panel_width, y_offset + line_height),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);

        y_offset += line_height * 2;

        // 绘制每个装甲板的位姿信息
        for (size_t i = 0; i < latest_poses_->poses.size(); i++)
        {
            const auto& pose = latest_poses_->poses[i];
            
            if (!pose.valid) {
                continue;
            }

            cv::Scalar color = getDrawColor(pose.color_id);
            
            // 第一行：目标信息
            char info1[128];
            snprintf(info1, sizeof(info1), "[%zu] %s-%s %s", 
                    i + 1,
                    getColorName(pose.color_id).c_str(),
                    getTagName(pose.tag_id).c_str(),
                    pose.armor_type == 1 ? "(L)" : "(S)");
            cv::putText(frame, info1,
                       cv::Point(frame.cols - panel_width, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            
            // 第二行：位姿数据
            char info2[128];
            double yaw_deg = pose.yaw * 180.0 / M_PI;
            double pitch_deg = pose.pitch * 180.0 / M_PI;
            snprintf(info2, sizeof(info2), "  D:%.2fm Y:%.1f P:%.1f", 
                    pose.distance, yaw_deg, pitch_deg);
            cv::putText(frame, info2,
                       cv::Point(frame.cols - panel_width, y_offset + line_height),
                       cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 180, 180), 1);
            y_offset += line_height * 2;
            
            // 位移向量（机架坐标系）
            char pos_body[128];
            snprintf(pos_body, sizeof(pos_body), "  Pos: [%.3f,%.3f,%.3f]", 
                    pose.position.x, pose.position.y, pose.position.z);
            cv::putText(frame, pos_body,
                       cv::Point(frame.cols - panel_width, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(100, 200, 255), 1);
            y_offset += line_height;
            
            // 位移向量（相机坐标系）
            char pos_cam[128];
            snprintf(pos_cam, sizeof(pos_cam), "  PosCam:[%.3f,%.3f,%.3f]", 
                    pose.position_cam.x, pose.position_cam.y, pose.position_cam.z);
            cv::putText(frame, pos_cam,
                       cv::Point(frame.cols - panel_width, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(150, 150, 150), 1);
            y_offset += line_height;
            
        }

        // 如果没有有效的位姿
        if (latest_poses_->poses.empty()) {
            cv::putText(frame, "No targets",
                       cv::Point(frame.cols - panel_width, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(128, 128, 128), 1);
        }
    }

    /**
     * @brief 在图像上绘制 EKF 跟踪信息
     */
    void drawEKFInfo(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);
        
        if (!ekf_state_.valid) {
            return;
        }

        int y_offset = 60;
        int line_height = 20;
        int panel_width = 220;
        int panel_height = 200;
        
        // 绘制半透明背景面板（左侧）
        cv::Mat overlay = frame.clone();
        cv::rectangle(overlay, 
                     cv::Point(10, y_offset - 25),
                     cv::Point(10 + panel_width, y_offset + panel_height),
                     cv::Scalar(0, 0, 0), cv::FILLED);
        cv::addWeighted(overlay, 0.6, frame, 0.4, 0, frame);

        // 绘制标题
        cv::putText(frame, "EKF Tracker", 
                   cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        
        y_offset += line_height + 5;

        // 状态颜色
        cv::Scalar state_color;
        if (ekf_state_.state == "TRACKING") {
            state_color = cv::Scalar(0, 255, 0);  // 绿色
        } else if (ekf_state_.state == "DETECTING") {
            state_color = cv::Scalar(0, 255, 255);  // 黄色
        } else if (ekf_state_.state == "TEMP_LOST") {
            state_color = cv::Scalar(0, 165, 255);  // 橙色
        } else {
            state_color = cv::Scalar(0, 0, 255);  // 红色
        }

        // 绘制状态
        char info[128];
        snprintf(info, sizeof(info), "State: %s", ekf_state_.state.c_str());
        cv::putText(frame, info, cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, state_color, 1);
        y_offset += line_height;

        // 绘制位置
        snprintf(info, sizeof(info), "Pos: (%.2f, %.2f, %.2f)", 
                ekf_state_.x, ekf_state_.y, ekf_state_.z);
        cv::putText(frame, info, cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);
        y_offset += line_height;

        // 绘制速度
        snprintf(info, sizeof(info), "Vel: (%.2f, %.2f, %.2f)", 
                ekf_state_.vx, ekf_state_.vy, ekf_state_.vz);
        cv::putText(frame, info, cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);
        y_offset += line_height;

        // 绘制 yaw 信息
        double yaw_deg = ekf_state_.yaw * 180.0 / M_PI;
        double yaw_vel_deg = ekf_state_.yaw_vel * 180.0 / M_PI;
        snprintf(info, sizeof(info), "Yaw: %.1f deg (%.1f/s)", yaw_deg, yaw_vel_deg);
        cv::putText(frame, info, cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);
        y_offset += line_height;

        // 绘制半径
        snprintf(info, sizeof(info), "Radius: %.3f m", ekf_state_.radius);
        cv::putText(frame, info, cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);
        y_offset += line_height + 5;

        // 绘制 NIS/NEES（卡方检验）
        cv::Scalar nis_color = (ekf_state_.nis < 0.711) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        snprintf(info, sizeof(info), "NIS: %.3f", ekf_state_.nis);
        cv::putText(frame, info, cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, nis_color, 1);
        y_offset += line_height;

        cv::Scalar nees_color = (ekf_state_.nees < 0.711) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        snprintf(info, sizeof(info), "NEES: %.3f", ekf_state_.nees);
        cv::putText(frame, info, cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, nees_color, 1);
    }

    /**
     * @brief 绘制目标轨迹
     */
    void drawTrajectory(cv::Mat& frame)
    {
        if (!show_trajectory_ || trajectory_points_.size() < 2) {
            return;
        }

        // 绘制轨迹线
        for (size_t i = 1; i < trajectory_points_.size(); ++i) {
            // 渐变颜色（旧的点更暗）
            double alpha = static_cast<double>(i) / trajectory_points_.size();
            cv::Scalar color(0, static_cast<int>(255 * alpha), static_cast<int>(255 * (1 - alpha)));
            
            cv::line(frame, 
                    cv::Point(static_cast<int>(trajectory_points_[i-1].x), 
                             static_cast<int>(trajectory_points_[i-1].y)),
                    cv::Point(static_cast<int>(trajectory_points_[i].x), 
                             static_cast<int>(trajectory_points_[i].y)),
                    color, 2);
        }

        // 绘制当前位置点
        if (!trajectory_points_.empty()) {
            cv::circle(frame, 
                      cv::Point(static_cast<int>(trajectory_points_.back().x),
                               static_cast<int>(trajectory_points_.back().y)),
                      5, cv::Scalar(0, 255, 0), -1);
        }
    }

    /**
     * @brief 在图像上绘制前哨站预测信息
     */
    void drawOutpostInfo(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(outpost_mutex_);
        
        int y_offset = 60;
        int line_height = 20;
        int panel_width = 240;
        int panel_height = 220;
        
        // 绘制半透明背景面板（左侧）
        cv::Mat overlay = frame.clone();
        cv::rectangle(overlay, 
                     cv::Point(10, y_offset - 25),
                     cv::Point(10 + panel_width, y_offset + panel_height),
                     cv::Scalar(0, 0, 0), cv::FILLED);
        cv::addWeighted(overlay, 0.6, frame, 0.4, 0, frame);

        // 绘制标题
        cv::putText(frame, "Outpost Predictor", 
                   cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 165, 0), 2);
        
        y_offset += line_height + 5;

        // 状态颜色
        cv::Scalar state_color;
        std::string state_str;
        if (outpost_state_.valid) {
            state_color = cv::Scalar(0, 255, 0);  // 绿色
            state_str = "TRACKING";
        } else {
            state_color = cv::Scalar(0, 0, 255);  // 红色
            state_str = "WAITING";
        }

        // 绘制状态
        char info[128];
        snprintf(info, sizeof(info), "State: %s", state_str.c_str());
        cv::putText(frame, info, cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, state_color, 1);
        y_offset += line_height;

        if (!outpost_state_.valid) {
            cv::putText(frame, "Waiting for outpost...", cv::Point(20, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(128, 128, 128), 1);
            return;
        }

        // 绘制旋转中心位置
        snprintf(info, sizeof(info), "Center: (%.2f, %.2f, %.2f)", 
                outpost_state_.center_x, outpost_state_.center_y, outpost_state_.center_z);
        cv::putText(frame, info, cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);
        y_offset += line_height;

        // 绘制中心速度
        snprintf(info, sizeof(info), "Vel: (%.2f, %.2f, %.2f)", 
                outpost_state_.vel_x, outpost_state_.vel_y, outpost_state_.vel_z);
        cv::putText(frame, info, cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);
        y_offset += line_height;

        // 绘制旋转半径
        snprintf(info, sizeof(info), "Radius: %.3f m", outpost_state_.radius);
        cv::putText(frame, info, cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);
        y_offset += line_height;

        // 绘制当前角度
        double theta_deg = outpost_state_.theta * 180.0 / M_PI;
        snprintf(info, sizeof(info), "Theta: %.1f deg", theta_deg);
        cv::putText(frame, info, cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);
        y_offset += line_height;

        // 绘制角速度
        double omega_deg = outpost_state_.omega * 180.0 / M_PI;
        snprintf(info, sizeof(info), "Omega: %.1f deg/s", omega_deg);
        cv::putText(frame, info, cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);
        y_offset += line_height;

        // 绘制旋转方向
        std::string dir_str;
        cv::Scalar dir_color;
        if (outpost_state_.direction == 1) {
            dir_str = "CCW (Counter-clockwise)";
            dir_color = cv::Scalar(0, 255, 255);  // 黄色
        } else if (outpost_state_.direction == -1) {
            dir_str = "CW (Clockwise)";
            dir_color = cv::Scalar(255, 0, 255);  // 紫色
        } else {
            dir_str = "Unknown";
            dir_color = cv::Scalar(128, 128, 128);
        }
        snprintf(info, sizeof(info), "Direction: %s", dir_str.c_str());
        cv::putText(frame, info, cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, dir_color, 1);
        y_offset += line_height + 10;

        // 绘制预测的装甲板位置指示
        if (outpost_state_.valid) {
            // 计算预测的装甲板位置
            double armor_x = outpost_state_.center_x + outpost_state_.radius * std::sin(outpost_state_.theta);
            double armor_y = outpost_state_.center_y - outpost_state_.radius * std::cos(outpost_state_.theta);
            
            snprintf(info, sizeof(info), "Pred Armor: (%.2f, %.2f)", armor_x, armor_y);
            cv::putText(frame, info, cv::Point(20, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 0), 1);
        }
    }

    /**
     * @brief 绘制前哨站预测轨迹
     */
    void drawOutpostTrajectory(cv::Mat& frame)
    {
        if (!show_trajectory_ || outpost_trajectory_.size() < 2) {
            return;
        }

        // 绘制轨迹线（橙色渐变）
        for (size_t i = 1; i < outpost_trajectory_.size(); ++i) {
            double alpha = static_cast<double>(i) / outpost_trajectory_.size();
            cv::Scalar color(0, static_cast<int>(165 * alpha), static_cast<int>(255 * alpha));
            
            cv::line(frame, 
                    cv::Point(static_cast<int>(outpost_trajectory_[i-1].x), 
                             static_cast<int>(outpost_trajectory_[i-1].y)),
                    cv::Point(static_cast<int>(outpost_trajectory_[i].x), 
                             static_cast<int>(outpost_trajectory_[i].y)),
                    color, 2);
        }

        // 绘制当前预测位置点
        if (!outpost_trajectory_.empty()) {
            cv::circle(frame, 
                      cv::Point(static_cast<int>(outpost_trajectory_.back().x),
                               static_cast<int>(outpost_trajectory_.back().y)),
                      6, cv::Scalar(0, 165, 255), -1);
            cv::circle(frame, 
                      cv::Point(static_cast<int>(outpost_trajectory_.back().x),
                               static_cast<int>(outpost_trajectory_.back().y)),
                      8, cv::Scalar(255, 255, 255), 2);
        }
    }

    /**
     * @brief 图像回调
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        last_image_time_ = std::chrono::steady_clock::now();
        
        // 转换图像
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge错误: %s", e.what());
            return;
        }

        // 初始化视频写入器
        if (!video_writer_.isOpened()) {
            int fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
            std::string out_path = output_video_path_;
            if (out_path.size() > 4 && out_path.substr(out_path.size()-4) == ".mp4") {
                out_path = out_path.substr(0, out_path.size()-4) + ".avi";
                output_video_path_ = out_path;
            }
            
            video_writer_.open(out_path, fourcc, fps_, frame.size());
            if (!video_writer_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "无法创建视频: %s", out_path.c_str());
                return;
            }
            RCLCPP_INFO(this->get_logger(), "开始写入视频: %s (%dx%d)", 
                       out_path.c_str(), frame.cols, frame.rows);
        }

        // 绘制位姿信息
        if (show_pose_info_) {
            drawPoseInfo(frame);
        }

        // 绘制 EKF 跟踪信息
        if (show_ekf_info_) {
            drawEKFInfo(frame);
            drawTrajectory(frame);
        }

        // 绘制前哨站预测信息
        if (show_outpost_info_) {
            drawOutpostInfo(frame);
            drawOutpostTrajectory(frame);
        }

        // 添加帧号信息
        char info[64];
        snprintf(info, sizeof(info), "Frame: %d", frame_count_);
        cv::putText(frame, info, cv::Point(10, frame.rows - 20), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

        // 写入视频
        video_writer_.write(frame);
        frame_count_++;

        if (frame_count_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "已写入 %d 帧", frame_count_);
        }
    }

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<armor_detector_ros2::msg::ArmorPoseArray>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tracker_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tracker_data_sub_;
    rclcpp::Subscription<armor_detector_ros2::msg::OutpostState>::SharedPtr outpost_sub_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;

    // 位姿数据
    std::mutex pose_mutex_;
    armor_detector_ros2::msg::ArmorPoseArray::SharedPtr latest_poses_;
    bool pose_received_ = false;

    // EKF 跟踪数据
    std::mutex ekf_mutex_;
    EKFTrackingState ekf_state_;
    std::deque<cv::Point2d> trajectory_points_;

    // 前哨站预测数据
    std::mutex outpost_mutex_;
    OutpostPredictionState outpost_state_;
    std::deque<cv::Point2d> outpost_trajectory_;
    int outpost_callback_count_ = 0;
    int pose_callback_count_ = 0;

    // 视频写入
    cv::VideoWriter video_writer_;
    std::string output_video_path_;
    double fps_;
    bool show_pose_info_;
    bool show_ekf_info_;
    bool show_trajectory_;
    bool show_outpost_info_;
    int trajectory_length_;
    int frame_count_ = 0;
    bool finished_ = false;
    
    std::chrono::steady_clock::time_point last_image_time_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
