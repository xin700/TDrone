/**
 * @file solver_visualizer_node.cpp
 * @brief Solver 节点专用调试可视化界面 - 重构版本
 * 
 * 功能：
 * 1. 在图像上绘制检测框和角点
 * 2. 显示 PnP 解算结果（yaw, pitch, distance, position）
 * 3. 在图像上绘制 3D 坐标轴验证位姿解算（直接使用相机坐标系数据重投影）
 * 4. 显示解算耗时统计
 * 
 * 订阅话题：
 * - /video/image_raw (sensor_msgs/Image): 原始图像
 * - /detector/armors (ArmorBBoxArray): 检测结果
 * - /solver/armor_poses (ArmorPoseArray): 解算结果
 * - /imu/quaternion (QuaternionStamped): IMU 数据（可选）
 * 
 * 坐标系说明：
 * =============
 * - 相机坐标系: X右, Y下, Z前 (OpenCV标准)
 * - 云台坐标系: X右, Y前, Z上
 * - 机架坐标系: X右, Y前, Z上
 * 
 * 重投影验证原理：
 * ================
 * 使用接收到的 position_cam（相机坐标系位置）直接投影，
 * 或者将机架坐标系数据通过IMU逆变换回相机坐标系后投影。
 * 这样可以验证整个解算链条的正确性。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "armor_detector_ros2/msg/armor_b_box.hpp"
#include "armor_detector_ros2/msg/armor_b_box_array.hpp"
#include "armor_detector_ros2/msg/armor_pose.hpp"
#include "armor_detector_ros2/msg/armor_pose_array.hpp"

#include <chrono>
#include <deque>
#include <mutex>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>

using ArmorBBoxArray = armor_detector_ros2::msg::ArmorBBoxArray;
using ArmorPoseArray = armor_detector_ros2::msg::ArmorPoseArray;

using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    ArmorBBoxArray,
    ArmorPoseArray>;

/**
 * @brief Solver 调试可视化节点
 */
class SolverVisualizerNode : public rclcpp::Node
{
public:
    SolverVisualizerNode()
    : Node("solver_visualizer_node")
    {
        // ==================== 参数声明 ====================
        this->declare_parameter("output_path", "/home/user/droneAim/TDrone/output/solver_debug.avi");
        this->declare_parameter("output_fps", 30.0);
        this->declare_parameter("show_window", true);
        this->declare_parameter("show_axis", true);
        this->declare_parameter("show_reproj", true);
        this->declare_parameter("axis_length", 0.1);
        
        // 相机内参
        this->declare_parameter("fx", 3083.8105952278529);
        this->declare_parameter("fy", 3087.5127345486885);
        this->declare_parameter("cx", 632.75929313977156);
        this->declare_parameter("cy", 506.7684422240867);
        // 畸变系数 (k1, k2, p1, p2, k3) - 必须与pose_solver使用的一致！
        this->declare_parameter("k1", -0.029972610213195684);
        this->declare_parameter("k2", 0.63682485437133696);
        this->declare_parameter("p1", 0.0);
        this->declare_parameter("p2", 0.0);
        this->declare_parameter("k3", 0.0);
        // 装甲板尺寸（用于重投影）
        this->declare_parameter("small_armor_width", 0.135);
        this->declare_parameter("small_armor_height", 0.055);
        this->declare_parameter("large_armor_width", 0.225);
        this->declare_parameter("large_armor_height", 0.055);
        
        // 获取参数
        output_video_path_ = this->get_parameter("output_path").as_string();
        fps_ = this->get_parameter("output_fps").as_double();
        show_window_ = this->get_parameter("show_window").as_bool();
        show_axis_ = this->get_parameter("show_axis").as_bool();
        show_reproj_ = this->get_parameter("show_reproj").as_bool();
        axis_length_ = this->get_parameter("axis_length").as_double();
        
        fx_ = this->get_parameter("fx").as_double();
        fy_ = this->get_parameter("fy").as_double();
        cx_ = this->get_parameter("cx").as_double();
        cy_ = this->get_parameter("cy").as_double();
        
        double k1 = this->get_parameter("k1").as_double();
        double k2 = this->get_parameter("k2").as_double();
        double p1 = this->get_parameter("p1").as_double();
        double p2 = this->get_parameter("p2").as_double();
        double k3 = this->get_parameter("k3").as_double();
        
        small_armor_width_ = this->get_parameter("small_armor_width").as_double();
        small_armor_height_ = this->get_parameter("small_armor_height").as_double();
        large_armor_width_ = this->get_parameter("large_armor_width").as_double();
        large_armor_height_ = this->get_parameter("large_armor_height").as_double();
        
        // 初始化相机矩阵
        camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            fx_, 0.0, cx_,
            0.0, fy_, cy_,
            0.0, 0.0, 1.0);
        // 初始化畸变系数 - 必须与pose_solver使用的一致！
        dist_coeffs_ = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
        
        RCLCPP_INFO(this->get_logger(), "Distortion coefficients: k1=%.4f, k2=%.4f, p1=%.4f, p2=%.4f, k3=%.4f",
                    k1, k2, p1, p2, k3);
        
        // 初始化装甲板3D点
        initArmor3DPoints();
        
        // 检测DISPLAY环境变量
        if (show_window_) {
            const char* display = std::getenv("DISPLAY");
            if (!display || strlen(display) == 0) {
                RCLCPP_WARN(this->get_logger(), "No DISPLAY, disabling window");
                show_window_ = false;
            }
        }
        
        // ==================== 创建订阅者 ====================
        image_sub_.subscribe(this, "/video/image_raw");
        detection_sub_.subscribe(this, "/detector/armors");
        pose_sub_.subscribe(this, "/solver/armor_poses");
        
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), image_sub_, detection_sub_, pose_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
        sync_->registerCallback(
            std::bind(&SolverVisualizerNode::syncCallback, this,
                     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        
        imu_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
            "/imu/quaternion", 10,
            std::bind(&SolverVisualizerNode::imuCallback, this, std::placeholders::_1));
        
        timeout_timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&SolverVisualizerNode::timeoutCallback, this));
        
        last_image_time_ = std::chrono::steady_clock::now();
        
        RCLCPP_INFO(this->get_logger(), "============================================");
        RCLCPP_INFO(this->get_logger(), "  Solver Visualizer Node (Refactored)");
        RCLCPP_INFO(this->get_logger(), "  Output: %s", output_video_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Camera: fx=%.4f fy=%.4f cx=%.4f cy=%.4f", fx_, fy_, cx_, cy_);
        RCLCPP_INFO(this->get_logger(), "  Distortion: k1=%.6f k2=%.6f p1=%.6f p2=%.6f k3=%.6f",
                    dist_coeffs_.at<double>(0), dist_coeffs_.at<double>(1), 
                    dist_coeffs_.at<double>(2), dist_coeffs_.at<double>(3), dist_coeffs_.at<double>(4));
        RCLCPP_INFO(this->get_logger(), "  Armor Small: %.4f x %.4f m", small_armor_width_, small_armor_height_);
        RCLCPP_INFO(this->get_logger(), "  Armor Large: %.4f x %.4f m", large_armor_width_, large_armor_height_);
        RCLCPP_INFO(this->get_logger(), "============================================");
        
        // 打开日志文件用于对比
        log_file_.open("/home/user/droneAim/TDrone/output/solver_visualizer_log.csv");
        log_file_ << "frame,rvec0,rvec1,rvec2,tvec0,tvec1,tvec2,corners" << std::endl;
        RCLCPP_INFO(this->get_logger(), "Log file: /home/user/droneAim/TDrone/output/solver_visualizer_log.csv");
        
        // 打开机架坐标系位置日志文件
        body_pos_log_file_.open("/home/user/droneAim/TDrone/output/body_position_log.csv");
        body_pos_log_file_ << "frame,x,y,z,distance,yaw_deg,pitch_deg,armor_type" << std::endl;
        RCLCPP_INFO(this->get_logger(), "Body position log: /home/user/droneAim/TDrone/output/body_position_log.csv");
    }

    ~SolverVisualizerNode()
    {
        if (show_window_) {
            try { cv::destroyAllWindows(); } catch (...) {}
        }
        if (video_writer_.isOpened()) {
            video_writer_.release();
            RCLCPP_INFO(this->get_logger(), "Video saved: %s (%d frames)", 
                       output_video_path_.c_str(), frame_count_);
        }
        if (log_file_.is_open()) {
            log_file_.close();
            RCLCPP_INFO(this->get_logger(), "Log file closed");
        }
        if (body_pos_log_file_.is_open()) {
            body_pos_log_file_.close();
            RCLCPP_INFO(this->get_logger(), "Body position log file closed");
        }
    }

private:
    void initArmor3DPoints()
    {
        // 装甲板坐标系: 原点在中心, X左, Y上, Z朝外(朝向相机)
        // 角点顺序: 左上(0), 左下(1), 右下(2), 右上(3)
        float sw = small_armor_width_ / 2.0f;
        float sh = small_armor_height_ / 2.0f;
        points_small_3d_ = {
            cv::Point3f( sw,  sh, 0.0f),
            cv::Point3f( sw, -sh, 0.0f),
            cv::Point3f(-sw, -sh, 0.0f),
            cv::Point3f(-sw,  sh, 0.0f)
        };
        
        float lw = large_armor_width_ / 2.0f;
        float lh = large_armor_height_ / 2.0f;
        points_large_3d_ = {
            cv::Point3f( lw,  lh, 0.0f),
            cv::Point3f( lw, -lh, 0.0f),
            cv::Point3f(-lw, -lh, 0.0f),
            cv::Point3f(-lw,  lh, 0.0f)
        };
    }

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
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
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
            if (video_writer_.isOpened()) {
                RCLCPP_INFO(this->get_logger(), "Video writer opened: %s", out_path.c_str());
            }
        }
        
        // 存储当前检测结果
        current_detections_.clear();
        for (const auto& armor : detection_msg->armors) {
            current_detections_.push_back(armor);
        }
        
        // 绘制可视化
        drawVisualization(frame, detection_msg, pose_msg);
        
        // 写入视频
        if (video_writer_.isOpened()) {
            video_writer_.write(frame);
        }
        
        // 显示窗口
        if (show_window_) {
            cv::imshow("Solver Debug", frame);
            int key = cv::waitKey(1);
            if (key == 'q' || key == 27) {
                rclcpp::shutdown();
            } else if (key == 'a') {
                show_axis_ = !show_axis_;
            } else if (key == 'r') {
                show_reproj_ = !show_reproj_;
            }
        }
        
        frame_count_++;
        
        // 记录rvec/tvec到日志文件（与verify_pnp对比用）
        if (log_file_.is_open() && pose_msg && detection_msg) {
            for (size_t i = 0; i < pose_msg->poses.size() && i < detection_msg->armors.size(); i++) {
                const auto& pose = pose_msg->poses[i];
                const auto& det = detection_msg->armors[i];
                if (pose.valid) {
                    log_file_ << frame_count_ << ","
                             << std::fixed << std::setprecision(6)
                             << pose.rvec[0] << ","
                             << pose.rvec[1] << ","
                             << pose.rvec[2] << ","
                             << pose.position_cam.x << ","
                             << pose.position_cam.y << ","
                             << pose.position_cam.z << ",\"";
                    // 记录角点
                    for (int j = 0; j < 4; j++) {
                        log_file_ << "(" << det.corners[j].x << "," 
                                 << det.corners[j].y << ")";
                        if (j < 3) log_file_ << ";";
                    }
                    log_file_ << "\"" << std::endl;
                }
            }
        }
        
        // 记录最左侧装甲板的机架坐标系位置
        if (body_pos_log_file_.is_open() && pose_msg && detection_msg && !pose_msg->poses.empty()) {
            // 找到有效的最左侧装甲板（根据检测框中心X坐标判断）
            int leftmost_idx = -1;
            double min_center_x = std::numeric_limits<double>::max();
            
            for (size_t i = 0; i < pose_msg->poses.size() && i < detection_msg->armors.size(); i++) {
                if (pose_msg->poses[i].valid) {
                    const auto& det = detection_msg->armors[i];
                    // 计算检测框中心X坐标
                    double center_x = (det.corners[0].x + det.corners[1].x + 
                                       det.corners[2].x + det.corners[3].x) / 4.0;
                    if (center_x < min_center_x) {
                        min_center_x = center_x;
                        leftmost_idx = static_cast<int>(i);
                    }
                }
            }
            
            // 记录最左侧装甲板的机架坐标系位置
            if (leftmost_idx >= 0) {
                const auto& pose = pose_msg->poses[leftmost_idx];
                body_pos_log_file_ << frame_count_ << ","
                                   << std::fixed << std::setprecision(6)
                                   << pose.position.x << ","
                                   << pose.position.y << ","
                                   << pose.position.z << ","
                                   << pose.distance << ","
                                   << (pose.yaw * 180.0 / M_PI) << ","
                                   << (pose.pitch * 180.0 / M_PI) << ","
                                   << static_cast<int>(pose.armor_type)
                                   << std::endl;
            }
        }
        
        if (pose_msg->poses.size() > 0) {
            solve_time_history_.push_back(pose_msg->solve_time_ms);
            if (solve_time_history_.size() > 100) {
                solve_time_history_.pop_front();
            }
        }
    }
    
    void imuCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        latest_imu_ = msg;
        imu_received_ = true;
    }
    
    void timeoutCallback()
    {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_image_time_);
        
        if (duration.count() >= 3 && frame_count_ > 0 && !finished_) {
            finished_ = true;
            if (video_writer_.isOpened()) {
                video_writer_.release();
                RCLCPP_INFO(this->get_logger(), "Stream ended, video saved: %s", output_video_path_.c_str());
            }
            rclcpp::shutdown();
        }
    }
    
    // =========================================================================
    // 可视化绘制
    // =========================================================================
    
    void drawVisualization(
        cv::Mat& frame, 
        const ArmorBBoxArray::ConstSharedPtr& detections,
        const ArmorPoseArray::ConstSharedPtr& poses)
    {
        // 绘制检测结果
        drawDetections(frame, detections);
        
        // 绘制解算结果（包括重投影验证）
        drawPoses(frame, poses);
        
        // 绘制信息面板
        drawInfoPanel(frame, detections, poses);
        
        // 绘制帮助信息
        drawHelp(frame);
    }
    
    void drawDetections(cv::Mat& frame, const ArmorBBoxArray::ConstSharedPtr& detections)
    {
        if (!detections || detections->armors.empty()) return;
        
        for (size_t i = 0; i < detections->armors.size(); i++) {
            const auto& armor = detections->armors[i];
            
            cv::Scalar color = (armor.color_id == 0) ? cv::Scalar(255, 100, 100) :
                               (armor.color_id == 1) ? cv::Scalar(100, 100, 255) :
                               cv::Scalar(200, 200, 200);
            
            // 绘制角点和边
            std::vector<cv::Point> corners;
            for (int j = 0; j < 4; j++) {
                cv::Point pt(static_cast<int>(armor.corners[j].x), 
                            static_cast<int>(armor.corners[j].y));
                corners.push_back(pt);
                cv::circle(frame, pt, 4, cv::Scalar(0, 255, 0), -1);
                
                // 标注角点序号（LT=0, LB=1, RB=2, RT=3）
                const char* labels[] = {"LT", "LB", "RB", "RT"};
                cv::putText(frame, labels[j], pt + cv::Point(5, -5),
                           cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 0), 1);
            }
            
            for (int j = 0; j < 4; j++) {
                cv::line(frame, corners[j], corners[(j+1)%4], color, 1);
            }
            
            // 标注ID和置信度
            char label[64];
            snprintf(label, sizeof(label), "#%d T:%d C:%.2f", 
                    (int)i, armor.tag_id, armor.confidence);
            cv::putText(frame, label, 
                       cv::Point(armor.rect_x, armor.rect_y - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1);
        }
    }
    
    void drawPoses(cv::Mat& frame, const ArmorPoseArray::ConstSharedPtr& poses)
    {
        if (!poses || poses->poses.empty()) return;
        
        for (size_t i = 0; i < poses->poses.size(); i++) {
            const auto& pose = poses->poses[i];
            
            if (!pose.valid) continue;
            
            // 使用相机坐标系位置（直接来自PnP解算）
            double x_cam = pose.position_cam.x;
            double y_cam = pose.position_cam.y;
            double z_cam = pose.position_cam.z;
            
            if (z_cam <= 0.01) continue;
            
            // 中心点投影
            int u = static_cast<int>(fx_ * x_cam / z_cam + cx_);
            int v = static_cast<int>(fy_ * y_cam / z_cam + cy_);
            
            // 绘制投影中心点（黄色十字）
            cv::circle(frame, cv::Point(u, v), 8, cv::Scalar(0, 255, 255), 2);
            cv::drawMarker(frame, cv::Point(u, v), cv::Scalar(0, 255, 255), 
                          cv::MARKER_CROSS, 16, 2);
            
            // 绘制3D坐标轴
            if (show_axis_) {
                drawCoordinateAxis(frame, pose, u, v);
            }
            
            // 绘制重投影验证
            if (show_reproj_ && i < current_detections_.size()) {
                drawReprojection(frame, pose, current_detections_[i]);
            }
            
            // 绘制位姿信息文字
            drawPoseInfo(frame, pose, u, v);
        }
    }
    
    /**
     * @brief 绘制3D坐标轴
     * 
     * 使用消息中的rvec/tvec直接投影坐标轴
     */
    void drawCoordinateAxis(cv::Mat& frame, 
                            const armor_detector_ros2::msg::ArmorPose& pose,
                            int cx, int cy)
    {
        double z_cam = pose.position_cam.z;
        if (z_cam <= 0.01) return;
        
        // 从消息中获取rvec和tvec
        cv::Mat rvec = (cv::Mat_<double>(3, 1) << 
            pose.rvec[0], pose.rvec[1], pose.rvec[2]);
        cv::Mat tvec = (cv::Mat_<double>(3, 1) << 
            pose.position_cam.x, pose.position_cam.y, pose.position_cam.z);
        
        // 调试：打印rvec值
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "rvec=[%.4f, %.4f, %.4f], tvec=[%.4f, %.4f, %.4f]",
            pose.rvec[0], pose.rvec[1], pose.rvec[2],
            pose.position_cam.x, pose.position_cam.y, pose.position_cam.z);
        
        // 调试：检查rvec是否为零（未初始化）
        if (std::abs(pose.rvec[0]) < 1e-10 && 
            std::abs(pose.rvec[1]) < 1e-10 && 
            std::abs(pose.rvec[2]) < 1e-10) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "WARNING: rvec is all zeros! rvec not being transmitted properly.");
        }
        
        // 坐标轴长度（米）
        float axis_len = static_cast<float>(axis_length_);
        
        // 定义坐标轴端点（装甲板坐标系）
        std::vector<cv::Point3f> axis_points = {
            cv::Point3f(0, 0, 0),           // 原点
            cv::Point3f(axis_len, 0, 0),    // X轴端点（左）
            cv::Point3f(0, axis_len, 0),    // Y轴端点（上）
            cv::Point3f(0, 0, axis_len)     // Z轴端点（外）
        };
        
        // 投影到图像
        std::vector<cv::Point2f> axis_2d;
        cv::projectPoints(axis_points, rvec, tvec, camera_matrix_, dist_coeffs_, axis_2d);
        
        // 绘制坐标轴
        cv::Point origin = axis_2d[0];
        cv::arrowedLine(frame, origin, axis_2d[1], cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.3);  // X: 红
        cv::arrowedLine(frame, origin, axis_2d[2], cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.3);  // Y: 绿
        cv::arrowedLine(frame, origin, axis_2d[3], cv::Scalar(255, 0, 0), 2, cv::LINE_AA, 0, 0.3);  // Z: 蓝
        
        // 标注轴名称
        cv::putText(frame, "X", axis_2d[1] + cv::Point2f(5, 5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
        cv::putText(frame, "Y", axis_2d[2] + cv::Point2f(5, 5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
        cv::putText(frame, "Z", axis_2d[3] + cv::Point2f(5, 5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);
    }
    
    /**
     * @brief 绘制重投影验证
     * 
     * 使用消息中的rvec/tvec重投影装甲板角点
     */
    void drawReprojection(cv::Mat& frame, 
                          const armor_detector_ros2::msg::ArmorPose& pose,
                          const armor_detector_ros2::msg::ArmorBBox& detection)
    {
        // 从消息中获取rvec和tvec
        cv::Mat rvec = (cv::Mat_<double>(3, 1) << 
            pose.rvec[0], pose.rvec[1], pose.rvec[2]);
        cv::Mat tvec = (cv::Mat_<double>(3, 1) << 
            pose.position_cam.x, pose.position_cam.y, pose.position_cam.z);
        
        // 选择3D点
        bool is_large = (pose.armor_type == 1);
        const auto& points_3d = is_large ? points_large_3d_ : points_small_3d_;
        
        // 重投影
        std::vector<cv::Point2f> reproj_pts;
        cv::projectPoints(points_3d, rvec, tvec, camera_matrix_, dist_coeffs_, reproj_pts);
        
        // 绘制重投影角点（黄色圆圈）
        for (size_t j = 0; j < reproj_pts.size(); j++) {
            cv::circle(frame, reproj_pts[j], 6, cv::Scalar(0, 255, 255), 2);
        }
        
        // 连接重投影角点
        for (int j = 0; j < 4; j++) {
            cv::line(frame, reproj_pts[j], reproj_pts[(j+1)%4], 
                    cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        }
    }
    
    void drawPoseInfo(cv::Mat& frame, 
                      const armor_detector_ros2::msg::ArmorPose& pose,
                      int u, int v)
    {
        int text_x = u + 100;
        int text_y = v - 100;
        
        text_x = std::max(10, std::min(text_x, frame.cols - 200));
        text_y = std::max(100, std::min(text_y, frame.rows - 120));
        
        cv::Scalar textColor(0, 255, 255);
        double scale = 0.4;
        int thickness = 1;
        
        // 背景框
        cv::rectangle(frame, 
                     cv::Point(text_x - 5, text_y - 15),
                     cv::Point(text_x + 180, text_y + 100),
                     cv::Scalar(30, 30, 30), -1);
        cv::rectangle(frame, 
                     cv::Point(text_x - 5, text_y - 15),
                     cv::Point(text_x + 180, text_y + 100),
                     cv::Scalar(100, 100, 100), 1);
        
        char buf[128];
        int line_h = 14;
        int y_offset = 0;
        
        // 机架坐标系信息
        cv::putText(frame, "[Body Frame] XYZ=R/F/U", cv::Point(text_x, text_y + y_offset), 
                   cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(200, 200, 0), thickness);
        y_offset += line_h;
        
        snprintf(buf, sizeof(buf), "Yaw:  %+6.2f deg", pose.yaw * 180.0 / M_PI);
        cv::putText(frame, buf, cv::Point(text_x, text_y + y_offset), 
                   cv::FONT_HERSHEY_SIMPLEX, scale, textColor, thickness);
        y_offset += line_h;
        
        snprintf(buf, sizeof(buf), "Pitch:%+6.2f deg", pose.pitch * 180.0 / M_PI);
        cv::putText(frame, buf, cv::Point(text_x, text_y + y_offset), 
                   cv::FONT_HERSHEY_SIMPLEX, scale, textColor, thickness);
        y_offset += line_h;
        
        snprintf(buf, sizeof(buf), "Dist: %6.3f m", pose.distance);
        cv::putText(frame, buf, cv::Point(text_x, text_y + y_offset), 
                   cv::FONT_HERSHEY_SIMPLEX, scale, textColor, thickness);
        y_offset += line_h;
        
        snprintf(buf, sizeof(buf), "Pos:[%.2f,%.2f,%.2f]", 
                pose.position.x, pose.position.y, pose.position.z);
        cv::putText(frame, buf, cv::Point(text_x, text_y + y_offset), 
                   cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(150, 200, 150), thickness);
        y_offset += line_h + 4;
        
        // 相机坐标系信息
        cv::putText(frame, "[Camera Frame] XYZ=R/D/F", cv::Point(text_x, text_y + y_offset), 
                   cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(150, 150, 200), thickness);
        y_offset += line_h;
        
        snprintf(buf, sizeof(buf), "Pos:[%.2f,%.2f,%.2f]", 
                pose.position_cam.x, pose.position_cam.y, pose.position_cam.z);
        cv::putText(frame, buf, cv::Point(text_x, text_y + y_offset), 
                   cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(150, 150, 200), thickness);
    }
    
    void drawInfoPanel(
        cv::Mat& frame,
        const ArmorBBoxArray::ConstSharedPtr& detections,
        const ArmorPoseArray::ConstSharedPtr& poses)
    {
        int panel_x = 10;
        int panel_y = 10;
        int panel_w = 260;
        int panel_h = 150;
        
        cv::Mat roi = frame(cv::Rect(panel_x, panel_y, panel_w, panel_h));
        cv::Mat overlay(roi.size(), roi.type(), cv::Scalar(20, 20, 20));
        cv::addWeighted(overlay, 0.7, roi, 0.3, 0, roi);
        
        cv::rectangle(frame, cv::Rect(panel_x, panel_y, panel_w, panel_h),
                     cv::Scalar(100, 100, 100), 1);
        
        int y = panel_y + 18;
        int line_height = 20;
        cv::Scalar titleColor(0, 255, 255);
        cv::Scalar infoColor(200, 200, 200);
        
        cv::putText(frame, "=== Solver Visualizer ===", 
                   cv::Point(panel_x + 10, y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, titleColor, 1);
        y += line_height + 3;
        
        char buf[128];
        snprintf(buf, sizeof(buf), "Frame: %d", frame_count_);
        cv::putText(frame, buf, cv::Point(panel_x + 10, y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, infoColor, 1);
        y += line_height;
        
        int det_count = detections ? detections->armors.size() : 0;
        int pose_count = poses ? poses->poses.size() : 0;
        snprintf(buf, sizeof(buf), "Detections: %d  Poses: %d", det_count, pose_count);
        cv::putText(frame, buf, cv::Point(panel_x + 10, y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, infoColor, 1);
        y += line_height;
        
        if (!solve_time_history_.empty()) {
            double avg_time = 0;
            for (double t : solve_time_history_) avg_time += t;
            avg_time /= solve_time_history_.size();
            
            snprintf(buf, sizeof(buf), "Solve Time: %.2f ms", avg_time);
            cv::putText(frame, buf, cv::Point(panel_x + 10, y), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.45, 
                       avg_time < 5.0 ? cv::Scalar(100, 255, 100) : cv::Scalar(100, 100, 255), 1);
        }
        y += line_height;
        
        // IMU状态
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            if (imu_received_ && latest_imu_) {
                double yaw, pitch;
                getIMUAnglesLocked(yaw, pitch);
                snprintf(buf, sizeof(buf), "IMU: Y:%.1f P:%.1f", yaw, pitch);
                cv::putText(frame, buf, cv::Point(panel_x + 10, y), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(100, 255, 100), 1);
            } else {
                cv::putText(frame, "IMU: Not received", cv::Point(panel_x + 10, y), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(100, 100, 255), 1);
            }
        }
    }
    
    void drawHelp(cv::Mat& frame)
    {
        int y = frame.rows - 25;
        cv::Scalar helpColor(150, 150, 150);
        cv::putText(frame, "[A] Axis  [R] Reproj  [Q/ESC] Quit", 
                   cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.4, helpColor, 1);
    }
    
    void getIMUAngles(double& yaw_deg, double& pitch_deg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        getIMUAnglesLocked(yaw_deg, pitch_deg);
    }
    
    void getIMUAnglesLocked(double& yaw_deg, double& pitch_deg)
    {
        yaw_deg = 0.0;
        pitch_deg = 0.0;
        
        if (!imu_received_ || !latest_imu_) return;
        
        double w = latest_imu_->quaternion.w;
        double x = latest_imu_->quaternion.x;
        double y = latest_imu_->quaternion.y;
        double z = latest_imu_->quaternion.z;
        
        // Pitch (Y-axis)
        double sinp = 2.0 * (w * y - z * x);
        pitch_deg = (std::abs(sinp) >= 1.0) ? 
                    std::copysign(90.0, sinp) : 
                    std::asin(sinp) * 180.0 / M_PI;
        
        // Yaw (Z-axis)
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        yaw_deg = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
    }

    // =========================================================================
    // 成员变量
    // =========================================================================
    
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<ArmorBBoxArray> detection_sub_;
    message_filters::Subscriber<ArmorPoseArray> pose_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    
    geometry_msgs::msg::QuaternionStamped::SharedPtr latest_imu_;
    std::mutex imu_mutex_;
    bool imu_received_ = false;
    
    bool finished_ = false;
    
    cv::VideoWriter video_writer_;
    std::string output_video_path_;
    double fps_;
    int frame_count_ = 0;
    
    bool show_window_;
    bool show_axis_;
    bool show_reproj_;
    double axis_length_;
    
    double fx_, fy_, cx_, cy_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    
    double small_armor_width_, small_armor_height_;
    double large_armor_width_, large_armor_height_;
    std::vector<cv::Point3f> points_small_3d_;
    std::vector<cv::Point3f> points_large_3d_;
    
    std::vector<armor_detector_ros2::msg::ArmorBBox> current_detections_;
    std::deque<double> solve_time_history_;
    std::chrono::steady_clock::time_point last_image_time_;
    
    // 日志文件用于对比
    std::ofstream log_file_;
    
    // 机架坐标系位置日志文件
    std::ofstream body_pos_log_file_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SolverVisualizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
