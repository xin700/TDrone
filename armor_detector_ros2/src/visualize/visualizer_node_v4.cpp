/**
 * @file visualizer_node_v4.cpp
 * @brief 前哨站预测 V4 专用可视化节点
 * 
 * 功能：
 * 1. 检测框与重投影显示
 * 2. 预瞄点从机架坐标系转换到相机坐标系并重投影（使用消息中发布的转换矩阵）
 * 3. 开火条件显示（满足绘制十字，否则绘制点）
 * 4. EKF收敛状态
 * 5. 预瞄点高度状态（HIGH/MIDDLE/LOW）
 * 6. 观测装甲板高度状态
 * 7. 实时显示窗口 + 视频输出
 * 
 * 坐标系转换链：
 *   相机坐标系 --[T_gimbal_camera]--> 云台坐标系 --[R_body_gimbal]--> 机架坐标系
 *   逆变换：机架坐标系 --[R_body_gimbal^T]--> 云台坐标系 --[T_gimbal_camera^-1]--> 相机坐标系
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

// Message filters for time synchronization
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "armor_detector_ros2/msg/armor_pose_array.hpp"
#include "armor_detector_ros2/msg/armor_b_box_array.hpp"
#include "armor_detector_ros2/msg/outpost_aim_result.hpp"

#include <chrono>
#include <deque>
#include <mutex>
#include <cmath>

// 类型别名
using ArmorBBoxArray = armor_detector_ros2::msg::ArmorBBoxArray;
using ArmorPoseArray = armor_detector_ros2::msg::ArmorPoseArray;
using OutpostAimResult = armor_detector_ros2::msg::OutpostAimResult;

// Sync policy for 3 topics (image, detections, poses)
using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    ArmorBBoxArray,
    ArmorPoseArray>;

/**
 * @brief V4 前哨站可视化节点
 * 
 * 使用 message_filters 时间戳同步，确保图像和检测结果精确匹配
 * 订阅V4预测器的 OutpostAimResult 消息，使用其中的转换矩阵进行坐标变换
 */
class VisualizerNodeV4 : public rclcpp::Node
{
public:
    VisualizerNodeV4()
    : Node("visualizer_node_v4")
    {
        // 声明参数
        this->declare_parameter("output_path", "/tmp/outpost_v4_result.avi");
        this->declare_parameter("output_fps", 30.0);
        this->declare_parameter("save_video", true);
        this->declare_parameter("show_predictor_info", true);
        this->declare_parameter("show_aim_point", true);
        this->declare_parameter("show_trajectory", true);
        this->declare_parameter("show_window", false);
        this->declare_parameter("sync_tolerance_ms", 50.0);
        this->declare_parameter("show_imu_data", true);
        
        // 相机内参配置路径（优先使用YAML文件）
        this->declare_parameter("camera_intrinsics_path", "");
        // 相机内参（fallback默认值，仅在无法从YAML加载时使用）
        this->declare_parameter("camera.fx", 1280.0);
        this->declare_parameter("camera.fy", 1024.0);
        this->declare_parameter("camera.cx", 640.0);
        this->declare_parameter("camera.cy", 512.0);
        
        // 获取参数
        output_video_path_ = this->get_parameter("output_path").as_string();
        fps_ = this->get_parameter("output_fps").as_double();
        save_video_ = this->get_parameter("save_video").as_bool();
        show_predictor_info_ = this->get_parameter("show_predictor_info").as_bool();
        show_aim_point_ = this->get_parameter("show_aim_point").as_bool();
        show_trajectory_ = this->get_parameter("show_trajectory").as_bool();
        show_window_ = this->get_parameter("show_window").as_bool();
        sync_tolerance_ms_ = this->get_parameter("sync_tolerance_ms").as_double();
        show_imu_data_ = this->get_parameter("show_imu_data").as_bool();
        
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
        
        // 初始化相机矩阵
        camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            fx_, 0.0, cx_,
            0.0, fy_, cy_,
            0.0, 0.0, 1.0);
        
        // =====================================================================
        // 订阅话题 (message_filters 时间戳同步)
        // =====================================================================
        
        image_sub_.subscribe(this, "/video/image_raw");
        detection_sub_.subscribe(this, "/detector/armors");
        pose_sub_.subscribe(this, "/solver/armor_poses");
        
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), image_sub_, detection_sub_, pose_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_tolerance_ms_ / 1000.0));
        sync_->registerCallback(
            std::bind(&VisualizerNodeV4::syncCallback, this,
                     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        
        // V4 预测器输出 (OutpostAimResult 消息)
        aim_result_sub_ = this->create_subscription<OutpostAimResult>(
            "/predictor_v4/aim_result", 10,
            std::bind(&VisualizerNodeV4::aimResultCallback, this, std::placeholders::_1));
        
        // IMU 数据订阅
        imu_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
            "/imu/quaternion", 10,
            std::bind(&VisualizerNodeV4::imuCallback, this, std::placeholders::_1));
        
        // 发布已绘制的可视化图像（供Web前端等使用）
        visualized_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/visualizer_v4/image_visualized", 10);
        
        // 超时检测
        timeout_timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&VisualizerNodeV4::timeoutCallback, this));
        
        last_image_time_ = std::chrono::steady_clock::now();
        
        RCLCPP_INFO(this->get_logger(), "V4可视化节点已启动 (使用消息中的转换矩阵)");
        if (save_video_) {
            RCLCPP_INFO(this->get_logger(), "  输出: %s", output_video_path_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "  输出: 不保存视频");
        }
        RCLCPP_INFO(this->get_logger(), "  时间同步容差: %.1f ms", sync_tolerance_ms_);
        RCLCPP_INFO(this->get_logger(), "  显示窗口: %s", show_window_ ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "  显示IMU数据: %s", show_imu_data_ ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "  发布可视化图像: /visualizer_v4/image_visualized");
    }

    ~VisualizerNodeV4()
    {
        if (show_window_) {
            cv::destroyAllWindows();
        }
        if (video_writer_.isOpened()) {
            video_writer_.release();
            if (save_video_) {
                RCLCPP_INFO(this->get_logger(), "视频已保存: %s (%d帧)", 
                           output_video_path_.c_str(), frame_count_);
            }
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
    // 坐标系转换函数
    // =========================================================================
    
    /**
     * @brief 从消息中解析转换矩阵
     * 
     * 消息中包含：
     * - t_gimbal_camera[9]: 相机坐标系 → 云台坐标系 的转换矩阵
     * - r_body_gimbal[9]: 云台坐标系 → 机架坐标系 的旋转矩阵
     */
    void parseTransformMatrices(const OutpostAimResult::SharedPtr& msg)
    {
        // 解析 T_gimbal_camera (3x3，按行存储)
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                T_gimbal_camera_(i, j) = msg->t_gimbal_camera[i * 3 + j];
            }
        }
        
        // 解析 R_body_gimbal (3x3，按行存储)
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R_body_gimbal_(i, j) = msg->r_body_gimbal[i * 3 + j];
            }
        }
        
        transform_valid_ = true;
    }
    
    /**
     * @brief 机架坐标系 → 相机坐标系 的逆变换
     * 
     * 正变换链（pose_solver.cpp）:
     *   p_gimbal = camera_offset_gimbal + T_gimbal_camera * p_cam
     *   p_body = R_body_gimbal * p_gimbal
     * 
     * 逆变换链:
     *   p_gimbal = R_body_gimbal^T * p_body
     *   p_cam = T_gimbal_camera^T * (p_gimbal - camera_offset_gimbal)
     */
    Eigen::Vector3d bodyToCamera(const Eigen::Vector3d& p_body) const
    {
        // 相机相对云台旋转中心的偏移（云台坐标系）
        // 必须与 pose_solver.cpp 中的 camera_offset_gimbal 保持一致！
        const Eigen::Vector3d camera_offset_gimbal(0.0, 0.00, -0.10);
        
        if (!transform_valid_) {
            // 使用默认变换（硬编码）作为备用
            // 机架坐标系: X右、Y前、Z上
            // 相机坐标系: X右、Y下、Z前
            // 默认情况下 R_body_gimbal = I（yaw=pitch=0）
            Eigen::Vector3d p_gimbal = p_body;
            Eigen::Vector3d p_gimbal_no_offset = p_gimbal - camera_offset_gimbal;
            // T_gimbal_camera^T: Gy->Cz, Gz->-Cy
            return Eigen::Vector3d(p_gimbal_no_offset.x(), -p_gimbal_no_offset.z(), p_gimbal_no_offset.y());
        }
        
        // 逆变换: 
        // Step 1: p_gimbal = R_body_gimbal^T * p_body
        Eigen::Vector3d p_gimbal = R_body_gimbal_.transpose() * p_body;
        
        // Step 2: 减去相机偏移
        Eigen::Vector3d p_gimbal_no_offset = p_gimbal - camera_offset_gimbal;
        
        // Step 3: p_cam = T_gimbal_camera^T * p_gimbal_no_offset
        Eigen::Vector3d p_cam = T_gimbal_camera_.transpose() * p_gimbal_no_offset;
        return p_cam;
    }
    
    /**
     * @brief 投影3D点到图像（相机坐标系）
     */
    cv::Point2i projectPoint(const Eigen::Vector3d& pt_cam, bool& valid) const
    {
        valid = false;
        if (pt_cam.z() < 0.1) {
            return cv::Point2i(-1, -1);
        }
        
        int u = static_cast<int>(fx_ * pt_cam.x() / pt_cam.z() + cx_);
        int v = static_cast<int>(fy_ * pt_cam.y() / pt_cam.z() + cy_);
        valid = true;
        return cv::Point2i(u, v);
    }
    
    // =========================================================================
    // 回调函数
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
            if (cx_ < 1.0) cx_ = frame.cols / 2.0;
            if (cy_ < 1.0) cy_ = frame.rows / 2.0;
        }
        
        // =====================================================================
        // 绘制可视化内容
        // =====================================================================
        
        // 1. 绘制时间同步的检测框和世界坐标
        drawDetections(frame, detection_msg, pose_msg);
        
        // 2. 绘制预测器状态信息
        if (show_predictor_info_) {
            drawPredictorInfo(frame);
        }
        
        // 2.5. 绘制 IMU 数据（右上角）
        if (show_imu_data_) {
            drawImuData(frame);
        }
        
        // 3. 绘制瞄准点（使用消息中的转换矩阵从机架坐标系转换到相机坐标系）
        if (show_aim_point_) {
            drawAimPoint(frame);
        }
        
        // 4. 绘制轨迹
        if (show_trajectory_) {
            drawTrajectory(frame);
        }
        
        // 5. 绘制开火指示器
        drawFireIndicator(frame);
        
        // 6. 绘制前哨站模型
        drawOutpostModel(frame);
        
        // 帧信息
        char info[128];
        snprintf(info, sizeof(info), "Frame: %d | t=%.3fs", frame_count_, img_timestamp);
        cv::putText(frame, info, cv::Point(10, frame.rows - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        
        // 写入视频（仅当 save_video 为 true 时）
        if (save_video_ && video_writer_.isOpened()) {
            video_writer_.write(frame);
        }
        frame_count_++;
        
        if (save_video_ && frame_count_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "已写入 %d 帧", frame_count_);
        }
        
        // 发布已绘制的可视化图像（供Web前端等使用）
        publishVisualizedImage(frame, image_msg->header);
        
        // 实时显示
        if (show_window_) {
            cv::imshow("Outpost V4 Visualization", frame);
            cv::waitKey(1);
        }
    }
    
    /**
     * @brief V4预测器结果回调
     */
    void aimResultCallback(const OutpostAimResult::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        latest_aim_result_ = msg;
        aim_result_received_ = true;
        
        // 解析转换矩阵
        parseTransformMatrices(msg);
        
        // 记录轨迹（使用转换矩阵转换到相机坐标系后的位置）
        if (trajectory_points_.size() >= 200) {
            trajectory_points_.pop_front();
        }
        
        Eigen::Vector3d p_body(msg->aim_position.x, msg->aim_position.y, msg->aim_position.z);
        Eigen::Vector3d p_cam = bodyToCamera(p_body);
        trajectory_points_.push_back(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
    }
    
    /**
     * @brief IMU 数据回调
     */
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
                RCLCPP_INFO(this->get_logger(), "视频流结束，已保存: %s", output_video_path_.c_str());
            }
            rclcpp::shutdown();
        }
    }
    
    // =========================================================================
    // 绘图函数
    // =========================================================================
    
    /**
     * @brief 绘制检测框和世界坐标
     */
    void drawDetections(cv::Mat& frame, 
                        const ArmorBBoxArray::ConstSharedPtr& detections,
                        const ArmorPoseArray::ConstSharedPtr& poses)
    {
        if (!detections) return;
        
        for (size_t i = 0; i < detections->armors.size(); i++) {
            const auto& armor = detections->armors[i];
            
            cv::Scalar color;
            if (armor.color_id == 0) {
                color = cv::Scalar(255, 100, 0);  // 蓝色装甲板
            } else if (armor.color_id == 1) {
                color = cv::Scalar(0, 100, 255);  // 红色装甲板
            } else {
                color = cv::Scalar(0, 255, 255);
            }
            
            std::vector<cv::Point> corners;
            for (int j = 0; j < 4; j++) {
                corners.push_back(cv::Point(
                    static_cast<int>(armor.corners[j].x),
                    static_cast<int>(armor.corners[j].y)
                ));
            }
            
            for (int j = 0; j < 4; j++) {
                cv::line(frame, corners[j], corners[(j + 1) % 4], color, 2);
            }
            
            for (int j = 0; j < 4; j++) {
                cv::circle(frame, corners[j], 4, color, -1);
            }
            
            cv::Point center(static_cast<int>(armor.center.x),
                            static_cast<int>(armor.center.y));
            cv::circle(frame, center, 5, color, -1);
            
            // 绘制原有的 tag_id 和 confidence
            char label[64];
            snprintf(label, sizeof(label), "%.2f #%d", armor.confidence, armor.tag_id);
            cv::putText(frame, label, cv::Point(center.x - 30, center.y - 15),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
            
            // 绘制世界坐标（如果对应的 pose 存在）
            if (poses && i < poses->poses.size()) {
                const auto& pose = poses->poses[i];
                char coord_label[128];
                snprintf(coord_label, sizeof(coord_label), 
                        "W: (%.2f, %.2f, %.2f)", 
                        pose.position.x, pose.position.y, pose.position.z);
                
                // 黄色显示世界坐标，显示在 tag_id 下方
                cv::putText(frame, coord_label, cv::Point(center.x - 80, center.y + 10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1);
            }
        }
    }
    
    /**
     * @brief 获取高度状态名称
     */
    std::string getHeightStateName(int32_t state) const
    {
        switch (state) {
            case 0: return "LOW";
            case 1: return "MIDDLE";
            case 2: return "HIGH";
            default: return "UNKNOWN";
        }
    }
    
    /**
     * @brief 获取高度状态颜色
     */
    cv::Scalar getHeightStateColor(int32_t state) const
    {
        switch (state) {
            case 0: return cv::Scalar(255, 100, 100);   // LOW - 蓝色
            case 1: return cv::Scalar(0, 255, 0);        // MIDDLE - 绿色
            case 2: return cv::Scalar(0, 200, 255);      // HIGH - 橙色
            default: return cv::Scalar(128, 128, 128);
        }
    }
    
    /**
     * @brief 获取初始化阶段名称（根据标志位推断）
     */
    std::string getInitPhaseStr(const OutpostAimResult::SharedPtr& msg) const
    {
        if (!msg->ekf_initialized) {
            if (!msg->height_initialized) {
                return "DIRECTION/HEIGHT_INIT";
            }
            return "THETA_INIT";
        }
        return "EKF_RUNNING";
    }
    
    /**
     * @brief 获取旋转方向名称
     */
    std::string getRotationDirName(int32_t dir) const
    {
        switch (dir) {
            case -1: return "CW (顺时针)";
            case 0: return "STATIC (静止)";
            case 1: return "CCW (逆时针)";
            default: return "UNKNOWN";
        }
    }
    
    /**
     * @brief 获取相位状态名称
     */
    std::string getPhaseStateName(int32_t phase) const
    {
        switch (phase) {
            case 0: return "RISING";
            case 1: return "PLATEAU";
            case 2: return "FALLING";
            default: return "UNKNOWN";
        }
    }
    
    /**
     * @brief 绘制预测器状态信息
     */
    void drawPredictorInfo(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        
        int y = 30;
        const int line_height = 22;
        
        // 标题
        cv::putText(frame, "=== Outpost Predictor V4 ===", cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        y += line_height + 5;
        
        if (!aim_result_received_ || !latest_aim_result_) {
            cv::putText(frame, "Waiting for predictor data...", cv::Point(20, y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 100), 1);
            return;
        }
        
        const auto& msg = latest_aim_result_;
        
        // 初始化阶段（根据标志位推断）
        std::string phase_str = getInitPhaseStr(msg);
        cv::Scalar phase_color = msg->ekf_initialized ? 
                                  cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
        cv::putText(frame, "Phase: " + phase_str, cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, phase_color, 1);
        y += line_height;
        
        // EKF收敛状态
        std::string ekf_status = msg->ekf_converged ? "CONVERGED" : "CONVERGING";
        cv::Scalar ekf_color = msg->ekf_converged ? 
                               cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
        cv::putText(frame, "EKF: " + ekf_status, cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, ekf_color, 1);
        y += line_height;
        
        // 旋转方向
        std::string dir_str = getRotationDirName(msg->rotation_direction);
        cv::putText(frame, "Dir: " + dir_str, cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        y += line_height;
        
        // 相位状态
        std::string phase_state_str = getPhaseStateName(msg->phase_state);
        cv::putText(frame, "Phase State: " + phase_state_str, cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        y += line_height;
        
        // 当前观测高度状态
        std::string obs_height = getHeightStateName(msg->current_height_state);
        cv::Scalar obs_color = getHeightStateColor(msg->current_height_state);
        cv::putText(frame, "Obs Height: " + obs_height, cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, obs_color, 1);
        y += line_height;
        
        // EKF内部高度状态 (height_k)
        std::string ekf_height = getHeightStateName(msg->ekf_height_state);
        cv::Scalar ekf_height_color = getHeightStateColor(msg->ekf_height_state);
        cv::putText(frame, "EKF Height: " + ekf_height, cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, ekf_height_color, 1);
        y += line_height;
        
        // 预瞄装甲板高度状态
        std::string aim_height = getHeightStateName(msg->aim_height_state);
        cv::Scalar aim_color = getHeightStateColor(msg->aim_height_state);
        cv::putText(frame, "Aim Height: " + aim_height, cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, aim_color, 2);
        y += line_height;
        
        // EKF状态 [θ, ω, x_c, y_c, z_c]
        char state_str[128];
        snprintf(state_str, sizeof(state_str), 
                "θ=%.1f° ω=%.2f rad/s",
                msg->ang, msg->omega);
        cv::putText(frame, state_str, cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 180, 180), 1);
        y += line_height;
        
        snprintf(state_str, sizeof(state_str), 
                "Center: (%.2f, %.2f, %.2f)m",
                msg->center_position.x, msg->center_position.y, msg->center_position.z);
        cv::putText(frame, state_str, cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 180, 180), 1);
        y += line_height;
        
        // 瞄准点位置（机架坐标系）
        char pos_str[128];
        snprintf(pos_str, sizeof(pos_str), "Aim: (%.2f, %.2f, %.2f)m",
                msg->aim_position.x, msg->aim_position.y, msg->aim_position.z);
        cv::putText(frame, pos_str, cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 180, 180), 1);
        y += line_height;
        
        // 观测θ和长宽比
        snprintf(pos_str, sizeof(pos_str), "ObsTheta: %.1f° AR: %.2f",
                msg->observed_theta, msg->aspect_ratio);
        cv::putText(frame, pos_str, cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 180, 180), 1);
        y += line_height;
        
        // 距离和子弹飞行时间
        snprintf(pos_str, sizeof(pos_str), "Dist: %.2fm ToF: %.3fs",
                msg->distance, msg->bullet_flight_time);
        cv::putText(frame, pos_str, cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 180, 180), 1);
        y += line_height;
        
        // 开火条件
        std::string fire_str = msg->fire_condition_1 ? "YES" : "NO";
        cv::Scalar fire_color = msg->fire_condition_1 ? 
                                cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::putText(frame, "Fire Condition: " + fire_str, cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, fire_color, 2);
    }
    
    /**
     * @brief 绘制 IMU 数据
     * 
     * 显示实时的四元数姿态信息以及转换后的欧拉角
     */
    void drawImuData(cv::Mat& frame)
    {
        if (!show_imu_data_) {
            return;
        }
        
        std::lock_guard<std::mutex> lock(imu_mutex_);
        
        // 计算显示位置（画面右上角）
        int x = frame.cols - 320;
        int y = 30;
        const int line_height = 22;
        
        // 绘制半透明背景
        cv::Rect bg_rect(x - 10, y - 25, 310, 160);
        cv::Mat roi = frame(bg_rect);
        cv::Mat color(roi.size(), CV_8UC3, cv::Scalar(0, 0, 0));
        cv::addWeighted(color, 0.4, roi, 0.6, 0, roi);
        
        // 标题
        cv::putText(frame, "=== IMU Data ===", cv::Point(x, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        y += line_height + 5;
        
        if (!imu_received_ || !latest_imu_) {
            cv::putText(frame, "Waiting for IMU...", cv::Point(x, y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 100), 1);
            return;
        }
        
        const auto& q = latest_imu_->quaternion;
        
        // 显示四元数
        char quat_str[128];
        snprintf(quat_str, sizeof(quat_str), "Quat:");
        cv::putText(frame, quat_str, cv::Point(x, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);
        y += line_height;
        
        snprintf(quat_str, sizeof(quat_str), "  w=%.4f", q.w);
        cv::putText(frame, quat_str, cv::Point(x, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 180, 180), 1);
        y += line_height;
        
        snprintf(quat_str, sizeof(quat_str), "  x=%.4f", q.x);
        cv::putText(frame, quat_str, cv::Point(x, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 180, 180), 1);
        y += line_height;
        
        snprintf(quat_str, sizeof(quat_str), "  y=%.4f", q.y);
        cv::putText(frame, quat_str, cv::Point(x, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 180, 180), 1);
        y += line_height;
        
        snprintf(quat_str, sizeof(quat_str), "  z=%.4f", q.z);
        cv::putText(frame, quat_str, cv::Point(x, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 180, 180), 1);
        y += line_height;
        
        // 转换为欧拉角（Roll, Pitch, Yaw）
        // 使用 ZYX 顺序
        double roll, pitch, yaw;
        quaternionToEuler(q.w, q.x, q.y, q.z, roll, pitch, yaw);
        
        char euler_str[128];
        snprintf(euler_str, sizeof(euler_str), "Euler (deg):");
        cv::putText(frame, euler_str, cv::Point(x, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);
        y += line_height;
        
        snprintf(euler_str, sizeof(euler_str), "  Roll =%.2f", roll);
        cv::putText(frame, euler_str, cv::Point(x, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(100, 180, 255), 1);
        y += line_height;
        
        snprintf(euler_str, sizeof(euler_str), "  Pitch=%.2f", pitch);
        cv::putText(frame, euler_str, cv::Point(x, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(100, 255, 100), 1);
        y += line_height;
        
        snprintf(euler_str, sizeof(euler_str), "  Yaw  =%.2f", yaw);
        cv::putText(frame, euler_str, cv::Point(x, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 100, 100), 1);
    }
    
    /**
     * @brief 四元数转欧拉角 (Roll, Pitch, Yaw，单位：度)
     * 
     * 使用 ZYX 顺序 (Yaw-Pitch-Roll)
     */
    void quaternionToEuler(double w, double x, double y, double z, 
                          double& roll, double& pitch, double& yaw)
    {
        // Roll (x-axis rotation)
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;
        
        // Pitch (y-axis rotation)
        double sinp = 2.0 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp) * 180.0 / M_PI; // use 90 degrees if out of range
        else
            pitch = std::asin(sinp) * 180.0 / M_PI;
        
        // Yaw (z-axis rotation)
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
    }
    
    /**
     * @brief 绘制瞄准点 (使用消息中的转换矩阵投影到图像)
     */
    void drawAimPoint(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        
        if (!aim_result_received_ || !latest_aim_result_) {
            return;
        }
        
        const auto& msg = latest_aim_result_;
        
        // 机架坐标系中的3D点
        Eigen::Vector3d p_body(msg->aim_position.x, msg->aim_position.y, msg->aim_position.z);
        
        // 使用消息中的转换矩阵转换到相机坐标系
        Eigen::Vector3d p_cam = bodyToCamera(p_body);
        
        // 投影到图像
        bool valid;
        cv::Point2i uv = projectPoint(p_cam, valid);
        
        if (valid && uv.x >= 0 && uv.x < frame.cols && uv.y >= 0 && uv.y < frame.rows) {
            // 根据开火条件选择样式
            if (msg->fire_condition_1) {
                // 绿色十字准星 + 圆圈
                cv::Scalar color(0, 255, 0);
                int size = 30;
                cv::line(frame, cv::Point(uv.x - size, uv.y), cv::Point(uv.x + size, uv.y), color, 2);
                cv::line(frame, cv::Point(uv.x, uv.y - size), cv::Point(uv.x, uv.y + size), color, 2);
                cv::circle(frame, uv, size, color, 2);
                cv::circle(frame, uv, 8, color, -1);
            } else {
                // 橙色点
                cv::Scalar color(0, 165, 255);
                cv::circle(frame, uv, 12, color, -1);
                cv::circle(frame, uv, 15, color, 2);
            }
            
            // 标注距离
            char dist_info[32];
            snprintf(dist_info, sizeof(dist_info), "%.2fm", p_cam.z());
            cv::putText(frame, dist_info, cv::Point(uv.x + 20, uv.y + 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
            
            // 标注瞄准高度
            std::string height_str = getHeightStateName(msg->aim_height_state);
            cv::Scalar height_color = getHeightStateColor(msg->aim_height_state);
            cv::putText(frame, height_str, cv::Point(uv.x + 20, uv.y + 25),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, height_color, 2);
        }
    }
    
    /**
     * @brief 绘制历史轨迹
     */
    void drawTrajectory(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        
        if (trajectory_points_.size() < 2) {
            return;
        }
        
        std::vector<cv::Point> projected_points;
        
        for (const auto& pt3d : trajectory_points_) {
            // pt3d 已经是相机坐标系（在回调中转换过了）
            Eigen::Vector3d pt_cam(pt3d.x, pt3d.y, pt3d.z);
            
            bool valid;
            cv::Point2i uv = projectPoint(pt_cam, valid);
            
            if (valid && uv.x >= 0 && uv.x < frame.cols && uv.y >= 0 && uv.y < frame.rows) {
                projected_points.push_back(uv);
            }
        }
        
        // 绘制轨迹线
        for (size_t i = 1; i < projected_points.size(); i++) {
            double alpha = static_cast<double>(i) / projected_points.size();
            cv::Scalar color(255 * (1 - alpha), 255 * alpha, 0);
            cv::line(frame, projected_points[i - 1], projected_points[i], color, 1);
        }
    }
    
    /**
     * @brief 绘制开火指示器
     */
    void drawFireIndicator(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        
        // 右上角指示器
        int indicator_x = frame.cols - 150;
        int indicator_y = 50;
        
        if (aim_result_received_ && latest_aim_result_) {
            if (latest_aim_result_->fire_condition_1) {
                // 绿色大圆 + "FIRE"
                cv::circle(frame, cv::Point(indicator_x + 50, indicator_y), 40, 
                          cv::Scalar(0, 255, 0), -1);
                cv::putText(frame, "OK", cv::Point(indicator_x + 22, indicator_y + 8),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
            } else {
                // 灰色空心圆 + "WAIT"
                cv::circle(frame, cv::Point(indicator_x + 50, indicator_y), 40, 
                          cv::Scalar(100, 100, 100), 3);
                cv::putText(frame, "WAIT", cv::Point(indicator_x + 23, indicator_y + 8),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(100, 100, 100), 2);
            }
        } else {
            // 红色空心圆 + "N/A"
            cv::circle(frame, cv::Point(indicator_x + 50, indicator_y), 40, 
                      cv::Scalar(0, 0, 200), 3);
            cv::putText(frame, "N/A", cv::Point(indicator_x + 30, indicator_y + 8),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 200), 2);
        }
    }
    
    /**
     * @brief 绘制前哨站模型 (中心点和所有装甲板位置)
     */
    void drawOutpostModel(cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        
        if (!aim_result_received_ || !latest_aim_result_) {
            return;
        }
        
        const auto& msg = latest_aim_result_;
        
        // 获取EKF状态
        double theta = msg->ang * M_PI / 180.0;  // 转为弧度
        double cx_body = msg->center_position.x;
        double cy_body = msg->center_position.y;
        double cz_body = msg->center_position.z;
        double omega = msg->omega;
        if ( omega > 0) {
            theta += 120 * M_PI / 180.0; 
        }
        // 前哨站几何参数
        const double radius = 0.2767;
        const double height_diff = 0.10;
        const double phase_spacing = 2.0 * M_PI / 3.0;
        
        // 三个装甲板的高度偏移（机架坐标系Z向上）: [LOW, MIDDLE, HIGH]
        const double height_offsets[3] = {-height_diff, height_diff, 0.0};
        // 相位偏移 [LOW, MIDDLE, HIGH]
        // 根据predictor的computeHeightFromAng：ang=0~120是LOW，120~240是MIDDLE，240~360是HIGH
        // 即：LOW在ang位置，MIDDLE在ang-120位置，HIGH在ang-240位置
        // 所以偏移量：LOW=0, MIDDLE=-120度, HIGH=-240度 (等价于0, 240, 120度)
        const double phase_offsets[3] = {0.0, -phase_spacing, -2.0 * phase_spacing};
        // 颜色
        const cv::Scalar colors[3] = {
            cv::Scalar(255, 100, 100),   // LOW - 蓝色
            cv::Scalar(0, 255, 0),        // MIDDLE - 绿色
            cv::Scalar(0, 200, 255)       // HIGH - 橙色
        };
        const std::string names[3] = {"L", "M", "H"};
        
        // 绘制旋转中心
        Eigen::Vector3d center_body(cx_body, cy_body, cz_body);
        Eigen::Vector3d center_cam = bodyToCamera(center_body);
        bool valid;
        cv::Point2i center_uv = projectPoint(center_cam, valid);
        
        if (valid && center_uv.x >= 0 && center_uv.x < frame.cols && 
            center_uv.y >= 0 && center_uv.y < frame.rows) {
            cv::drawMarker(frame, center_uv, cv::Scalar(0, 255, 255),
                          cv::MARKER_CROSS, 20, 2);
            cv::putText(frame, "C", cv::Point(center_uv.x + 10, center_uv.y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
        }
        
        // 计算中心相对于原点的方位角（与predictor一致）
        double azimuth = std::atan2(cx_body, cy_body);
        
        // 绘制三个装甲板位置
        for (int i = 0; i < 3; i++) {
            // 每个装甲板的世界角度
            // theta是全局角度（弧度），当theta=60度(π/3)时LOW装甲板正对
            // 所以需要减去π/3来修正：M_PI - π/3 = 2π/3
            double world_ang = azimuth + theta + phase_offsets[i] + 2.0 * M_PI / 3.0;
            
            double armor_x = cx_body + radius * std::sin(world_ang);
            double armor_y = cy_body + radius * std::cos(world_ang);
            double armor_z = cz_body + height_offsets[i];
            
            // 判断是否面向相机
            bool facing_camera = std::cos(world_ang) < 0;
            
            // 使用消息中的转换矩阵转换到相机坐标系
            Eigen::Vector3d armor_body(armor_x, armor_y, armor_z);
            Eigen::Vector3d armor_cam = bodyToCamera(armor_body);
            cv::Point2i armor_uv = projectPoint(armor_cam, valid);
            
            if (valid && armor_uv.x >= 0 && armor_uv.x < frame.cols && 
                armor_uv.y >= 0 && armor_uv.y < frame.rows) {
                
                // 判断是否是当前瞄准的装甲板
                bool is_aim_target = (i == msg->aim_height_state);
                
                cv::Scalar color = colors[i];
                
                // 根据到相机的距离计算圆点大小
                // 距离越近圆点越大，距离越远圆点越小
                double dist_to_cam = armor_cam.norm();
                // 基于距离的大小映射：距离2m时半径15，距离5m时半径6
                int base_radius = static_cast<int>(std::clamp(30.0 / dist_to_cam, 6.0, 18.0));
                
                if (facing_camera) {
                    // 面向相机: 实心圆，大小根据距离
                    cv::circle(frame, armor_uv, base_radius, color, -1);
                } else {
                    // 背向相机: 空心圆，较小
                    cv::circle(frame, armor_uv, base_radius - 2, color, 2);
                }
                
                // 被瞄准的装甲板用白圈标记（不放大）
                if (is_aim_target) {
                    cv::circle(frame, armor_uv, base_radius + 4, cv::Scalar(255, 255, 255), 2);
                }
                
                // 标注装甲板类型
                cv::putText(frame, names[i], cv::Point(armor_uv.x - 5, armor_uv.y + 5),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
                
                // 绘制从中心到装甲板的连线
                if (center_uv.x >= 0 && center_uv.x < frame.cols && 
                    center_uv.y >= 0 && center_uv.y < frame.rows) {
                    if (facing_camera) {
                        cv::line(frame, center_uv, armor_uv, color, 1);
                    } else {
                        drawDashedLine(frame, center_uv, armor_uv, color, 1, 10);
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
        
        if (num_dashes < 1) num_dashes = 1;
        
        for (int i = 0; i < num_dashes; i++) {
            double t1 = static_cast<double>(i * 2) / (num_dashes * 2);
            double t2 = static_cast<double>(i * 2 + 1) / (num_dashes * 2);
            
            cv::Point start(p1.x + dx * t1, p1.y + dy * t1);
            cv::Point end(p1.x + dx * t2, p1.y + dy * t2);
            cv::line(frame, start, end, color, thickness);
        }
    }
    
    // =========================================================================
    // 辅助函数
    // =========================================================================
    
    /**
     * @brief 发布已绘制的可视化图像
     * 
     * 将完成所有标注绘制后的图像发布到话题，供Web前端等其他节点直接使用
     */
    void publishVisualizedImage(const cv::Mat& frame, const std_msgs::msg::Header& header)
    {
        try {
            auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            visualized_image_pub_->publish(*msg);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "发布可视化图像失败: %s", e.what());
        }
    }
    
    void initVideoWriter(cv::Size frame_size)
    {
        if (!save_video_) {
            // RCLCPP_INFO(this->get_logger(), "不保存视频，仅进行可视化处理");
            return;
        }
        
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
    
    // Message filters 订阅器
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<ArmorBBoxArray> detection_sub_;
    message_filters::Subscriber<ArmorPoseArray> pose_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    // V4预测器订阅
    rclcpp::Subscription<OutpostAimResult>::SharedPtr aim_result_sub_;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    
    // 发布已绘制的可视化图像
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualized_image_pub_;
    
    // 预测器数据
    std::mutex predictor_mutex_;
    OutpostAimResult::SharedPtr latest_aim_result_;
    bool aim_result_received_ = false;
    
    // IMU 数据
    std::mutex imu_mutex_;
    geometry_msgs::msg::QuaternionStamped::SharedPtr latest_imu_;
    bool imu_received_ = false;
    
    // 坐标转换矩阵（从消息中解析）
    Eigen::Matrix3d T_gimbal_camera_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_body_gimbal_ = Eigen::Matrix3d::Identity();
    bool transform_valid_ = false;
    
    // 轨迹（存储相机坐标系中的位置）
    std::deque<cv::Point3d> trajectory_points_;
    
    // 视频写入
    cv::VideoWriter video_writer_;
    std::string output_video_path_;
    double fps_;
    bool save_video_;
    int frame_count_ = 0;
    bool finished_ = false;
    
    // 参数
    bool show_predictor_info_;
    bool show_aim_point_;
    bool show_trajectory_;
    bool show_window_;
    double sync_tolerance_ms_;
    bool show_imu_data_;
    
    // 相机内参
    double fx_, fy_, cx_, cy_;
    cv::Mat camera_matrix_;
    
    std::chrono::steady_clock::time_point last_image_time_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizerNodeV4>());
    rclcpp::shutdown();
    return 0;
}
