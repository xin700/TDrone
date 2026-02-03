/**
 * @file visualizer_node_v2.cpp
 * @brief 前哨站预测 V2 专用可视化节点
 * 
 * 功能：
 * 1. 显示多假设收敛状态（未收敛时显示三个假设的置信度）
 * 2. 显示当前时刻三个装甲板的位置（用不同颜色标识）
 * 3. 显示预测的弹道轨迹和击中点
 * 4. 显示可打窗口状态
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "armor_detector_ros2/msg/outpost_state_v2.hpp"
#include "armor_detector_ros2/msg/armor_pose_array.hpp"
#include "armor_detector_ros2/msg/armor_b_box_array.hpp"

#include <chrono>
#include <deque>
#include <mutex>
#include <cmath>
#include <cstdlib>
#include <cstring>

/**
 * @brief V2 前哨站可视化节点
 */
class VisualizerNodeV2 : public rclcpp::Node
{
public:
    VisualizerNodeV2()
    : Node("visualizer_node_v2")
    {
        // 声明参数
        this->declare_parameter("output_path", "/tmp/outpost_v2_result.avi");
        this->declare_parameter("output_fps", 30.0);
        this->declare_parameter("show_convergence_info", true);
        this->declare_parameter("show_armor_positions", true);
        this->declare_parameter("show_trajectory_prediction", true);
        this->declare_parameter("show_shootable_window", true);
        this->declare_parameter("trajectory_length", 100);
        this->declare_parameter("show_window", false);
        
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
        show_convergence_info_ = this->get_parameter("show_convergence_info").as_bool();
        show_armor_positions_ = this->get_parameter("show_armor_positions").as_bool();
        show_trajectory_prediction_ = this->get_parameter("show_trajectory_prediction").as_bool();
        show_shootable_window_ = this->get_parameter("show_shootable_window").as_bool();
        trajectory_length_ = this->get_parameter("trajectory_length").as_int();
        show_window_ = this->get_parameter("show_window").as_bool();
        
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
        
        // 测试是否能使用GUI（避免在无显示环境中卡住）
        if (show_window_) {
            const char* display = std::getenv("DISPLAY");
            if (!display || strlen(display) == 0) {
                RCLCPP_WARN(this->get_logger(), "未检测到DISPLAY环境变量，禁用实时显示窗口");
                show_window_ = false;
                window_disabled_ = true;
            } else {
                RCLCPP_INFO(this->get_logger(), "实时显示窗口已启用 (DISPLAY=%s)", display);
            }
        }
        
        // 订阅图像
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/video/image_raw", 10,
            std::bind(&VisualizerNodeV2::image_callback, this, std::placeholders::_1));
        
        // 订阅 V2 前哨站状态
        outpost_sub_ = this->create_subscription<armor_detector_ros2::msg::OutpostStateV2>(
            "/outpost_state_v2", 10,
            std::bind(&VisualizerNodeV2::outpost_callback, this, std::placeholders::_1));
        
        // 订阅装甲板位姿（用于显示原始检测）
        pose_sub_ = this->create_subscription<armor_detector_ros2::msg::ArmorPoseArray>(
            "/solver/armor_poses", 10,
            std::bind(&VisualizerNodeV2::pose_callback, this, std::placeholders::_1));
        
        // 订阅装甲板检测结果（用于绘制角点）
        detection_sub_ = this->create_subscription<armor_detector_ros2::msg::ArmorBBoxArray>(
            "/detector/armors", 10,
            std::bind(&VisualizerNodeV2::detection_callback, this, std::placeholders::_1));
        
        // 超时检测
        timeout_timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&VisualizerNodeV2::timeout_callback, this));
        
        last_image_time_ = std::chrono::steady_clock::now();
        
        RCLCPP_INFO(this->get_logger(), "V2可视化节点已启动");
        RCLCPP_INFO(this->get_logger(), "  输出: %s", output_video_path_.c_str());
    }

    ~VisualizerNodeV2()
    {
        if (show_window_ && !window_disabled_) {
            try {
                cv::destroyAllWindows();
            } catch (...) {
                // 忽略销毁窗口时的异常
            }
        }
        
        if (video_writer_.isOpened()) {
            video_writer_.release();
            RCLCPP_INFO(this->get_logger(), "视频已保存: %s (%d帧)", 
                       output_video_path_.c_str(), frame_count_);
        } else {
            RCLCPP_WARN(this->get_logger(), "视频写入器从未打开！帧数: %d", frame_count_);
            RCLCPP_WARN(this->get_logger(), "预期输出路径: %s", output_video_path_.c_str());
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
    // 回调函数
    // =========================================================================
    
    void outpost_callback(const armor_detector_ros2::msg::OutpostStateV2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_state_ = msg;
        if (!state_received_) {
            RCLCPP_INFO(this->get_logger(), "首次收到OutpostStateV2消息");
        }
        state_received_ = true;
        
        // 记录轨迹
        if (msg->valid && trajectory_points_.size() < static_cast<size_t>(trajectory_length_)) {
            // 使用最佳假设对应的装甲板位置
            int target_id = msg->predicted_target_id >= 0 ? msg->predicted_target_id : 1;
            trajectory_points_.push_back(cv::Point3d(
                msg->armor_positions[target_id].x,
                msg->armor_positions[target_id].y,
                msg->armor_positions[target_id].z
            ));
        }
        if (trajectory_points_.size() > static_cast<size_t>(trajectory_length_)) {
            trajectory_points_.pop_front();
        }
    }
    
    void pose_callback(const armor_detector_ros2::msg::ArmorPoseArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_poses_ = msg;
        pose_received_ = true;
    }
    
    void detection_callback(const armor_detector_ros2::msg::ArmorBBoxArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(detection_mutex_);
        latest_detections_ = msg;
        detection_received_ = true;
    }
    
    void timeout_callback()
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
    
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        last_image_time_ = std::chrono::steady_clock::now();
        
        if (frame_count_ == 0) {
            RCLCPP_INFO(this->get_logger(), "首次收到图像，尺寸: %dx%d", msg->width, msg->height);
        }
        
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
                RCLCPP_ERROR(this->get_logger(), "无法创建视频: %s (fps=%.1f)", out_path.c_str(), fps_);
                return;
            }
            frame_size_ = frame.size();
            RCLCPP_INFO(this->get_logger(), "开始写入视频: %s (%.1f FPS)", out_path.c_str(), fps_);
        }
        
        // 绘制可视化内容
        if (show_convergence_info_) {
            drawConvergenceInfo(frame);
        }
        if (show_armor_positions_) {
            drawArmorPositions(frame);
        }
        if (show_trajectory_prediction_) {
            drawTrajectoryPrediction(frame);
        }
        if (show_shootable_window_) {
            drawShootableWindow(frame);
        }
        
        // 绘制原始检测框
        drawDetections(frame);
        
        // 绘制检测到的装甲板角点
        drawDetectionCorners(frame);
        
        // 绘制预测击中位置
        drawPredictedArmorPosition(frame);
        
        // 帧号
        char info[64];
        snprintf(info, sizeof(info), "Frame: %d", frame_count_);
        cv::putText(frame, info, cv::Point(10, frame.rows - 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        
        video_writer_.write(frame);
        frame_count_++;
        
        if (frame_count_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "已写入 %d 帧", frame_count_);
        }
        
        // 实时显示窗口（已禁用，避免WSL环境卡死）
        // if (show_window_) {
        //     cv::imshow("Outpost V2 Visualization", frame);
        //     cv::waitKey(1);
        // }
    }
    
    // =========================================================================
    // 绘图函数
    // =========================================================================
    
    /**
     * @brief 绘制收敛状态信息
     * 
     * 未收敛时：显示 "未收敛 (H:33% M:45% L:22%)"
     * 已收敛时：显示 "已收敛: 中位装甲板"
     */
    void drawConvergenceInfo(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        char info[256];
        int y_offset = 30;
        const int line_height = 25;
        
        // 标题
        cv::putText(frame, "=== Outpost V2 (Multi-Hypothesis) ===", cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2);
        y_offset += line_height + 5;
        
        if (!state_received_ || !latest_state_) {
            cv::putText(frame, "Waiting for data...", cv::Point(20, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(128, 128, 128), 1);
            return;
        }
        
        if (!latest_state_->valid) {
            cv::putText(frame, "State: INVALID / LOST", cv::Point(20, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
            return;
        }
        
        // 收敛状态
        if (latest_state_->converged) {
            // 已收敛
            const char* armor_names[] = {"HIGH", "MIDDLE", "LOW"};
            int best_id = latest_state_->best_hypothesis_id;
            if (best_id < 0 || best_id > 2) best_id = 1;
            
            snprintf(info, sizeof(info), "CONVERGED: %s armor", armor_names[best_id]);
            cv::putText(frame, info, cv::Point(20, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 2);
            y_offset += line_height;
            
            // 最终置信度
            snprintf(info, sizeof(info), "Confidence: %.1f%%", 
                    latest_state_->hypothesis_confidences[best_id] * 100.0);
            cv::putText(frame, info, cv::Point(20, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);
            y_offset += line_height;
            
        } else {
            // 未收敛 - 显示三个假设的置信度
            cv::putText(frame, "CONVERGING...", cv::Point(20, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 165, 255), 2);
            y_offset += line_height;
            
            // 绘制三个假设的置信度条
            const char* labels[] = {"H", "M", "L"};
            cv::Scalar colors[] = {
                cv::Scalar(0, 0, 255),     // 红 - HIGH
                cv::Scalar(0, 255, 0),     // 绿 - MIDDLE  
                cv::Scalar(255, 0, 0)      // 蓝 - LOW
            };
            
            for (int i = 0; i < 3; i++) {
                double conf = latest_state_->hypothesis_confidences[i];
                int bar_width = static_cast<int>(conf * 150);
                
                // 标签
                snprintf(info, sizeof(info), "%s: %.0f%%", labels[i], conf * 100);
                cv::putText(frame, info, cv::Point(20, y_offset),
                           cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
                
                // 进度条背景
                cv::rectangle(frame, cv::Point(70, y_offset - 12),
                             cv::Point(70 + 150, y_offset + 2),
                             cv::Scalar(50, 50, 50), -1);
                
                // 进度条
                cv::rectangle(frame, cv::Point(70, y_offset - 12),
                             cv::Point(70 + bar_width, y_offset + 2),
                             colors[i], -1);
                
                y_offset += 20;
            }
        }
        
        // 显示基本状态
        y_offset += 10;
        double theta_deg = latest_state_->theta * 180.0 / M_PI;
        double omega_deg = latest_state_->omega * 180.0 / M_PI;
        
        snprintf(info, sizeof(info), "Theta: %.1f deg | Omega: %.1f deg/s", theta_deg, omega_deg);
        cv::putText(frame, info, cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);
        y_offset += line_height;
        
        // 旋转方向
        std::string dir_str = (latest_state_->direction == 1) ? "CCW" : 
                              (latest_state_->direction == -1) ? "CW" : "?";
        snprintf(info, sizeof(info), "Direction: %s | Radius: %.3fm", 
                dir_str.c_str(), latest_state_->radius);
        cv::putText(frame, info, cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);
    }
    
    /**
     * @brief 绘制三个装甲板的当前位置
     * 
     * 使用不同颜色：
     * - 红色：高位装甲板
     * - 绿色：中位装甲板
     * - 蓝色：低位装甲板
     * 
     * 可打的装甲板用实心圆，不可打的用空心圆
     */
    void drawArmorPositions(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        if (!state_received_ || !latest_state_ || !latest_state_->valid) {
            return;
        }
        
        cv::Scalar colors[] = {
            cv::Scalar(0, 0, 255),     // 红 - HIGH
            cv::Scalar(0, 255, 0),     // 绿 - MIDDLE
            cv::Scalar(255, 0, 0)      // 蓝 - LOW
        };
        
        const char* labels[] = {"H", "M", "L"};
        
        // 在右侧绘制三装甲板状态图例
        int legend_x = frame.cols - 180;
        int legend_y = 30;
        
        cv::putText(frame, "=== Armor Status ===", cv::Point(legend_x, legend_y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1);
        legend_y += 25;
        
        for (int i = 0; i < 3; i++) {
            auto& pos = latest_state_->armor_positions[i];
            bool shootable = latest_state_->armor_shootable[i];
            double orient_deg = latest_state_->armor_orientations[i] * 180.0 / M_PI;
            
            // 图例
            char info[128];
            snprintf(info, sizeof(info), "%s: %.1f deg %s", 
                    labels[i], orient_deg, shootable ? "[OK]" : "");
            
            // 可打的用实心圆
            if (shootable) {
                cv::circle(frame, cv::Point(legend_x + 10, legend_y - 5), 8, colors[i], -1);
            } else {
                cv::circle(frame, cv::Point(legend_x + 10, legend_y - 5), 8, colors[i], 2);
            }
            
            cv::putText(frame, info, cv::Point(legend_x + 25, legend_y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, 
                       shootable ? cv::Scalar(0, 255, 0) : cv::Scalar(150, 150, 150), 1);
            
            legend_y += 20;
            
            // 3D投影到2D（简化：假设在图像中心区域显示俯视图）
            // 这里我们在图像右下角绘制一个俯视图
        }
        
        // 绘制俯视图（在右下角）
        drawTopView(frame);
    }
    
    /**
     * @brief 绘制俯视图显示三个装甲板的相对位置
     * 注意：调用此函数前必须已持有state_mutex_锁
     */
    void drawTopView(cv::Mat& frame)
    {
        // 不再获取锁，因为调用者(drawArmorPositions)已持有锁
        // std::lock_guard<std::mutex> lock(state_mutex_);
        
        if (!latest_state_ || !latest_state_->valid) {
            return;
        }
        
        // 俯视图区域
        int view_size = 180;
        int margin = 20;
        int view_x = frame.cols - view_size - margin;
        int view_y = frame.rows - view_size - margin - 50;
        
        // 背景
        cv::rectangle(frame, cv::Point(view_x, view_y),
                     cv::Point(view_x + view_size, view_y + view_size),
                     cv::Scalar(30, 30, 30), -1);
        cv::rectangle(frame, cv::Point(view_x, view_y),
                     cv::Point(view_x + view_size, view_y + view_size),
                     cv::Scalar(100, 100, 100), 1);
        
        // 标题
        cv::putText(frame, "Top View", cv::Point(view_x + 50, view_y + 15),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
        
        // 中心点
        int cx = view_x + view_size / 2;
        int cy = view_y + view_size / 2;
        
        // 绘制中心
        cv::circle(frame, cv::Point(cx, cy), 3, cv::Scalar(255, 255, 255), -1);
        
        // 缩放因子（1米 = 多少像素）
        double scale = 100.0;  // 假设radius约0.275m，显示效果约27像素
        
        // 中心位置
        double center_x = latest_state_->center.x;
        double center_y = latest_state_->center.y;
        
        cv::Scalar colors[] = {
            cv::Scalar(0, 0, 255),     // 红 - HIGH
            cv::Scalar(0, 255, 0),     // 绿 - MIDDLE
            cv::Scalar(255, 0, 0)      // 蓝 - LOW
        };
        
        // 绘制三个装甲板（俯视图只看x-y平面）
        for (int i = 0; i < 3; i++) {
            auto& pos = latest_state_->armor_positions[i];
            
            // 相对于中心的偏移
            double dx = pos.x - center_x;
            double dy = pos.y - center_y;
            
            // 转换到视图坐标（y轴取反，因为图像y向下）
            int px = cx + static_cast<int>(dx * scale);
            int py = cy - static_cast<int>(dy * scale);  // 注意取反
            
            // 限制在视图范围内
            px = std::max(view_x + 10, std::min(view_x + view_size - 10, px));
            py = std::max(view_y + 20, std::min(view_y + view_size - 10, py));
            
            bool shootable = latest_state_->armor_shootable[i];
            
            if (shootable) {
                cv::circle(frame, cv::Point(px, py), 8, colors[i], -1);
            } else {
                cv::circle(frame, cv::Point(px, py), 8, colors[i], 2);
            }
            
            // 标签
            const char* labels[] = {"H", "M", "L"};
            cv::putText(frame, labels[i], cv::Point(px - 4, py + 4),
                       cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 1);
        }
        
        // 绘制旋转方向指示（固定位置的圆弧箭头）
        if (latest_state_->direction != 0) {
            int arrow_radius = 35;
            
            // 绘制圆弧（顺时针或逆时针）
            double arc_start = -30.0;  // 固定起始角度
            double arc_end = 30.0;     // 固定结束角度
            
            if (latest_state_->direction < 0) {
                // 逆时针：从30度到-30度
                std::swap(arc_start, arc_end);
            }
            
            // 使用ellipse绘制圆弧
            cv::ellipse(frame, cv::Point(cx, cy), cv::Size(arrow_radius, arrow_radius),
                       0, arc_start, arc_end, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            
            // 在圆弧末端绘制箭头
            double arrow_tip_angle = (arc_end) * M_PI / 180.0;
            cv::Point arrow_tip(cx + static_cast<int>(arrow_radius * std::cos(arrow_tip_angle)),
                               cy + static_cast<int>(arrow_radius * std::sin(arrow_tip_angle)));
            
            // 箭头的两条边
            double arrow_size = 8;
            double arrow_angle1 = arrow_tip_angle + (latest_state_->direction > 0 ? 2.5 : -2.5);
            double arrow_angle2 = arrow_tip_angle + (latest_state_->direction > 0 ? 2.8 : -2.8);
            
            cv::Point arrow_p1(arrow_tip.x - static_cast<int>(arrow_size * std::cos(arrow_angle1)),
                              arrow_tip.y - static_cast<int>(arrow_size * std::sin(arrow_angle1)));
            cv::Point arrow_p2(arrow_tip.x - static_cast<int>(arrow_size * std::cos(arrow_angle2)),
                              arrow_tip.y - static_cast<int>(arrow_size * std::sin(arrow_angle2)));
            
            cv::line(frame, arrow_tip, arrow_p1, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            cv::line(frame, arrow_tip, arrow_p2, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
        }
    }
    
    /**
     * @brief 绘制弹道预测轨迹
     * 
     * 显示预测的击中点和预计的装甲板位置
     */
    void drawTrajectoryPrediction(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        if (!state_received_ || !latest_state_ || !latest_state_->valid) {
            return;
        }
        
        // 在底部显示弹道信息
        int y_offset = frame.rows - 120;
        char info[256];
        
        cv::putText(frame, "=== Trajectory Prediction ===", cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1);
        y_offset += 20;
        
        // 子弹飞行时间
        snprintf(info, sizeof(info), "Flight time: %.3f s", latest_state_->bullet_flight_time);
        cv::putText(frame, info, cv::Point(20, y_offset),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
        y_offset += 18;
        
        // 预测目标
        if (latest_state_->predicted_target_id >= 0) {
            const char* armor_names[] = {"HIGH", "MIDDLE", "LOW"};
            int target_id = latest_state_->predicted_target_id;
            
            snprintf(info, sizeof(info), "Target: %s armor", armor_names[target_id]);
            cv::putText(frame, info, cv::Point(20, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
            y_offset += 18;
            
            // 预测击中点
            auto& hit = latest_state_->predicted_hit_point;
            snprintf(info, sizeof(info), "Hit point: (%.2f, %.2f, %.2f)", hit.x, hit.y, hit.z);
            cv::putText(frame, info, cv::Point(20, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
        } else {
            snprintf(info, sizeof(info), "No shootable target");
            cv::putText(frame, info, cv::Point(20, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
            y_offset += 18;
            
            // 下一个可打窗口
            if (latest_state_->next_window_time > 0) {
                snprintf(info, sizeof(info), "Next window in: %.2f s", latest_state_->next_window_time);
                cv::putText(frame, info, cv::Point(20, y_offset),
                           cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 165, 0), 1);
            }
        }
    }
    
    /**
     * @brief 绘制可打窗口指示器
     */
    void drawShootableWindow(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        if (!state_received_ || !latest_state_ || !latest_state_->valid) {
            return;
        }
        
        // 检查是否有可打目标
        bool any_shootable = false;
        for (int i = 0; i < 3; i++) {
            if (latest_state_->armor_shootable[i]) {
                any_shootable = true;
                break;
            }
        }
        
        // 大号指示器在画面中央顶部
        if (any_shootable) {
            // FIRE 指示
            cv::rectangle(frame, cv::Point(frame.cols/2 - 60, 5),
                         cv::Point(frame.cols/2 + 60, 35),
                         cv::Scalar(0, 255, 0), -1);
            cv::putText(frame, "FIRE", cv::Point(frame.cols/2 - 35, 28),
                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
        } else {
            // WAIT 指示
            cv::rectangle(frame, cv::Point(frame.cols/2 - 60, 5),
                         cv::Point(frame.cols/2 + 60, 35),
                         cv::Scalar(0, 165, 255), -1);
            cv::putText(frame, "WAIT", cv::Point(frame.cols/2 - 35, 28),
                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
        }
    }
    
    /**
     * @brief 绘制检测到的装甲板角点和边框
     */
    void drawDetectionCorners(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(detection_mutex_);
        
        if (!detection_received_ || !latest_detections_) {
            return;
        }
        
        for (const auto& armor : latest_detections_->armors) {
            // 选择颜色（根据装甲板颜色）
            cv::Scalar color;
            if (armor.color_id == 0) {  // 蓝色
                color = cv::Scalar(255, 0, 0);
            } else if (armor.color_id == 1) {  // 红色
                color = cv::Scalar(0, 0, 255);
            } else {  // 其他
                color = cv::Scalar(0, 255, 255);
            }
            
            // 绘制四个角点
            for (int i = 0; i < 4; i++) {
                cv::Point pt(static_cast<int>(armor.corners[i].x),
                            static_cast<int>(armor.corners[i].y));
                cv::circle(frame, pt, 4, color, -1);
                
                // 绘制编号
                char num[8];
                snprintf(num, sizeof(num), "%d", i);
                cv::putText(frame, num, cv::Point(pt.x + 5, pt.y - 5),
                           cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
            }
            
            // 绘制连线（形成边框）
            for (int i = 0; i < 4; i++) {
                cv::Point pt1(static_cast<int>(armor.corners[i].x),
                             static_cast<int>(armor.corners[i].y));
                cv::Point pt2(static_cast<int>(armor.corners[(i + 1) % 4].x),
                             static_cast<int>(armor.corners[(i + 1) % 4].y));
                cv::line(frame, pt1, pt2, color, 2);
            }
            
            // 显示置信度
            cv::Point center(static_cast<int>(armor.center.x),
                            static_cast<int>(armor.center.y));
            char conf[32];
            snprintf(conf, sizeof(conf), "%.2f", armor.confidence);
            cv::putText(frame, conf, cv::Point(center.x - 20, center.y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        }
    }
    
    /**
     * @brief 绘制预测的装甲板位置（在图像上投影显示）
     */
    void drawPredictedArmorPosition(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        if (!state_received_ || !latest_state_ || !latest_state_->valid) {
            return;
        }
        
        // 只绘制预测击中的目标装甲板
        if (latest_state_->predicted_target_id < 0) {
            return;
        }
        
        int target_id = latest_state_->predicted_target_id;
        
        // 获取预测击中点和装甲板位置
        auto& hit_point = latest_state_->predicted_hit_point;
        auto& armor_pos = latest_state_->armor_positions[target_id];
        double armor_orient = latest_state_->armor_orientations[target_id];
        
        // 使用相机内参进行投影
        // x 投影到图像 x，y 是深度，z 投影到图像 y
        if (hit_point.y > 0.1) {  // 避免除以零
            // 使用正确的相机投影模型: u = fx * x / z + cx, v = fy * y / z + cy
            // 注意：hit_point 坐标系为 X右、Y前、Z上
            // 相机坐标系为 X右、Y下、Z前
            int proj_x = static_cast<int>(fx_ * hit_point.x / hit_point.y + cx_);
            int proj_y = static_cast<int>(fy_ * (-hit_point.z) / hit_point.y + cy_);
            
            // 限制在图像范围内
            if (proj_x >= 0 && proj_x < frame.cols && proj_y >= 0 && proj_y < frame.rows) {
                // 绘制十字准星
                cv::Scalar target_color(0, 255, 0);  // 绿色
                int cross_size = 20;
                cv::line(frame, cv::Point(proj_x - cross_size, proj_y),
                        cv::Point(proj_x + cross_size, proj_y), target_color, 2);
                cv::line(frame, cv::Point(proj_x, proj_y - cross_size),
                        cv::Point(proj_x, proj_y + cross_size), target_color, 2);
                cv::circle(frame, cv::Point(proj_x, proj_y), cross_size, target_color, 2);
                
                // 标注信息
                char info[64];
                const char* armor_names[] = {"HIGH", "MID", "LOW"};
                snprintf(info, sizeof(info), "HIT: %s", armor_names[target_id]);
                cv::putText(frame, info, cv::Point(proj_x + 25, proj_y - 10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, target_color, 2);
                
                // 显示飞行时间
                snprintf(info, sizeof(info), "T: %.3fs", latest_state_->bullet_flight_time);
                cv::putText(frame, info, cv::Point(proj_x + 25, proj_y + 15),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, target_color, 1);
            }
        }
    }
    
    /**
     * @brief 绘制原始检测框
     */
    void drawDetections(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        
        if (!pose_received_ || !latest_poses_) {
            return;
        }
        
        for (const auto& pose : latest_poses_->poses) {
            // 绘制检测中心点
            // 注意：这里假设pose里有像素坐标，实际需要根据msg定义调整
            // 暂时用3D位置简单显示
            char info[64];
            snprintf(info, sizeof(info), "Det: (%.2f, %.2f, %.2f)", 
                    pose.position.x, pose.position.y, pose.position.z);
            cv::putText(frame, info, cv::Point(20, frame.rows - 40),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
        }
    }
    
    // =========================================================================
    // 成员变量
    // =========================================================================
    
    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<armor_detector_ros2::msg::OutpostStateV2>::SharedPtr outpost_sub_;
    rclcpp::Subscription<armor_detector_ros2::msg::ArmorPoseArray>::SharedPtr pose_sub_;
    rclcpp::Subscription<armor_detector_ros2::msg::ArmorBBoxArray>::SharedPtr detection_sub_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    
    // 状态数据
    std::mutex state_mutex_;
    armor_detector_ros2::msg::OutpostStateV2::SharedPtr latest_state_;
    bool state_received_ = false;
    
    // 位姿数据
    std::mutex pose_mutex_;
    armor_detector_ros2::msg::ArmorPoseArray::SharedPtr latest_poses_;
    bool pose_received_ = false;
    
    // 检测数据
    std::mutex detection_mutex_;
    armor_detector_ros2::msg::ArmorBBoxArray::SharedPtr latest_detections_;
    bool detection_received_ = false;
    
    // 轨迹
    std::deque<cv::Point3d> trajectory_points_;
    
    // 视频写入
    cv::VideoWriter video_writer_;
    std::string output_video_path_;
    double fps_;
    cv::Size frame_size_;
    int frame_count_ = 0;
    bool finished_ = false;
    
    // 参数
    bool show_convergence_info_;
    bool show_armor_positions_;
    bool show_trajectory_prediction_;
    bool show_shootable_window_;
    bool show_window_;
    bool window_disabled_ = false;  // GUI显示是否被禁用
    int trajectory_length_;
    
    // 相机内参
    double fx_ = 1280.0;
    double fy_ = 1024.0;
    double cx_ = 640.0;
    double cy_ = 512.0;
    
    std::chrono::steady_clock::time_point last_image_time_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizerNodeV2>());
    rclcpp::shutdown();
    return 0;
}
