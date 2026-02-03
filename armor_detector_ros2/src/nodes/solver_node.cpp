/**
 * @file solver_node.cpp
 * @brief 装甲板位姿解算 ROS2 节点 - 重构版本
 * 
 * 功能：订阅检测结果和 IMU 数据，进行 PnP 位姿解算，发布位姿结果
 * 
 * 订阅话题：
 * - /detector/armors (ArmorBBoxArray): 检测结果
 * - /imu/quaternion (geometry_msgs/QuaternionStamped): IMU 姿态
 * 
 * 发布话题：
 * - /solver/armor_poses (ArmorPoseArray): 位姿解算结果
 * 
 * 坐标系说明：
 * - 相机坐标系: X右, Y下, Z前 (OpenCV标准)
 * - 云台坐标系: X右, Y前, Z上
 * - 机架坐标系: X右, Y前, Z上 (与云台yaw=pitch=0时重合)
 * 
 * IMU定义：
 * - yaw: 向右为正
 * - pitch: 向上为正
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>
#include <memory>
#include <mutex>
#include <cmath>
#include <deque>

#include "core/pose_solver.hpp"
#include "armor_detector_ros2/msg/armor_b_box.hpp"
#include "armor_detector_ros2/msg/armor_b_box_array.hpp"
#include "armor_detector_ros2/msg/armor_pose.hpp"
#include "armor_detector_ros2/msg/armor_pose_array.hpp"
#include "armor_detector_ros2/msg/point2f.hpp"

using namespace std::chrono_literals;

// IMU 缓存最大时长（秒）
constexpr double IMU_BUFFER_DURATION = 1.0;

/**
 * @brief 从四元数计算欧拉角
 * 
 * 注意：四元数到欧拉角的转换与坐标系定义密切相关
 * 这里假设标准的ZYX欧拉角顺序（航空顺序）
 */
inline void quaternionToEuler(double w, double x, double y, double z,
                               double& yaw, double& pitch, double& roll)
{
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(90.0, sinp);
    else
        pitch = std::asin(sinp) * 180.0 / M_PI;

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
}

/**
 * @class SolverNode
 * @brief 装甲板位姿解算节点类
 */
class SolverNode : public rclcpp::Node
{
public:
    SolverNode() : Node("solver_node")
    {
        // ==================== 参数声明 ====================
        this->declare_parameter<std::string>("camera_intrinsics_path", "");
        this->declare_parameter<double>("fx", 3083.8105952278529);
        this->declare_parameter<double>("fy", 3087.5127345486885);
        this->declare_parameter<double>("cx", 632.75929313977156);
        this->declare_parameter<double>("cy", 506.7684422240867);
        this->declare_parameter<double>("k1", 0.0);
        this->declare_parameter<double>("k2", 0.0);
        this->declare_parameter<double>("p1", 0.0);
        this->declare_parameter<double>("p2", 0.0);
        this->declare_parameter<double>("k3", 0.0);
        // 获取参数
        std::string camera_intrinsics_path = this->get_parameter("camera_intrinsics_path").as_string();

        // ==================== 初始化位姿解算器 ====================
        solver_ = std::make_unique<SOLVER::PoseSolver>();
        
        // 尝试从 YAML 文件加载相机内参
        if (!camera_intrinsics_path.empty())
        {
            if (solver_->loadCameraIntrinsicsFromYAML(camera_intrinsics_path))
            {
                RCLCPP_INFO(this->get_logger(), "Camera intrinsics loaded from: %s", 
                           camera_intrinsics_path.c_str());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to load YAML, using parameters");
                loadIntrinsicsFromParams();
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Using parameter-based camera intrinsics");
            loadIntrinsicsFromParams();
        }

        // 验证相机内参
        if (!solver_->isCameraIntrinsicsSet())
        {
            RCLCPP_ERROR(this->get_logger(), "Camera intrinsics not set!");
        }
        else
        {
            const auto& intrinsics = solver_->getCameraIntrinsics();
            RCLCPP_INFO(this->get_logger(), "Camera intrinsics: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f",
                       intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy);
        }

        // ==================== 创建发布者 ====================
        pose_pub_ = this->create_publisher<armor_detector_ros2::msg::ArmorPoseArray>(
            "/solver/armor_poses",
            rclcpp::QoS(10).reliable()
        );

        // ==================== 创建订阅者 ====================
        // 使用 SensorDataQoS 以兼容上游丢帧的场景
        armor_sub_ = this->create_subscription<armor_detector_ros2::msg::ArmorBBoxArray>(
            "/detector/armors",
            rclcpp::SensorDataQoS(),
            std::bind(&SolverNode::armorCallback, this, std::placeholders::_1)
        );

        imu_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
            "/imu/quaternion",
            rclcpp::SensorDataQoS(),
            std::bind(&SolverNode::imuCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "======================================");
        RCLCPP_INFO(this->get_logger(), "  Solver Node Started (Refactored)");
        RCLCPP_INFO(this->get_logger(), "  Coordinate System: XYZ = Right/Front/Up");
        RCLCPP_INFO(this->get_logger(), "======================================");
    }

private:
    void loadIntrinsicsFromParams()
    {
        double fx = this->get_parameter("fx").as_double();
        double fy = this->get_parameter("fy").as_double();
        double cx = this->get_parameter("cx").as_double();
        double cy = this->get_parameter("cy").as_double();
        double k1 = this->get_parameter("k1").as_double();
        double k2 = this->get_parameter("k2").as_double();
        double p1 = this->get_parameter("p1").as_double();
        double p2 = this->get_parameter("p2").as_double();
        double k3 = this->get_parameter("k3").as_double();

        solver_->setCameraMatrix(fx, fy, cx, cy, k1, k2, p1, p2, k3);
    }

    void imuCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        
        // 添加到缓存
        imu_buffer_.push_back(msg);
        
        // 移除过期的 IMU 数据（保留最近 IMU_BUFFER_DURATION 秒）
        double current_time = rclcpp::Time(msg->header.stamp).seconds();
        while (!imu_buffer_.empty()) {
            double oldest_time = rclcpp::Time(imu_buffer_.front()->header.stamp).seconds();
            if (current_time - oldest_time > IMU_BUFFER_DURATION) {
                imu_buffer_.pop_front();
            } else {
                break;
            }
        }
        
        imu_received_ = true;
    }
    
    /**
     * @brief 根据时间戳查找最近的 IMU 数据
     * @param target_stamp 目标时间戳
     * @return 最近的 IMU 数据，如果缓存为空则返回 nullptr
     */
    geometry_msgs::msg::QuaternionStamped::SharedPtr findClosestImu(const builtin_interfaces::msg::Time& target_stamp)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        
        if (imu_buffer_.empty()) {
            return nullptr;
        }
        
        double target_time = rclcpp::Time(target_stamp).seconds();
        
        geometry_msgs::msg::QuaternionStamped::SharedPtr closest = nullptr;
        double min_diff = std::numeric_limits<double>::max();
        
        for (const auto& imu : imu_buffer_) {
            double imu_time = rclcpp::Time(imu->header.stamp).seconds();
            double diff = std::abs(imu_time - target_time);
            if (diff < min_diff) {
                min_diff = diff;
                closest = imu;
            }
        }
        
        // 如果时间差超过 100ms，打印警告
        if (min_diff > 0.1) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "IMU-Image time diff: %.1f ms (may be out of sync)", min_diff * 1000);
        }
        
        return closest;
    }

    void armorCallback(const armor_detector_ros2::msg::ArmorBBoxArray::SharedPtr msg)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        // 根据图像时间戳查找最近的 IMU 数据
        double imu_yaw = 0.0;
        double imu_pitch = 0.0;
        
        auto closest_imu = findClosestImu(msg->header.stamp);
        if (closest_imu) {
            double roll;
            quaternionToEuler(
                closest_imu->quaternion.w,
                closest_imu->quaternion.x,
                closest_imu->quaternion.y,
                closest_imu->quaternion.z,
                imu_yaw, imu_pitch, roll
            );
            
            // 调试：打印所有三个欧拉角，确认哪个对应云台的抬头低头
            // RCLCPP_INFO(this->get_logger(),
            //     "Euler angles - yaw: %.2f, pitch: %.2f, roll: %.2f",
            //     imu_yaw, imu_pitch, roll);
            
            // ============================================================
            // 重要：四元数转欧拉角使用ZYX航空顺序：
            //   - yaw: 绕Z轴（水平旋转）    -> 对应云台yaw ✓
            //   - pitch: 绕Y轴（前后轴）    -> 可能不是云台pitch！
            //   - roll: 绕X轴（左右轴）     -> 可能才是云台pitch！
            // 
            // 如果云台抬头低头时 roll 在变而 pitch 基本不变，
            // 那么应该使用 roll 作为云台的 pitch：
            // ============================================================
            
            // 取消下面这行的注释来交换 pitch 和 roll
            // imu_pitch = roll;
            
            // 注意：根据实际IMU安装方向，可能需要调整符号
            // imu_yaw = -imu_yaw;  // 如果需要取反
        }

        // 创建输出消息
        armor_detector_ros2::msg::ArmorPoseArray pose_array_msg;
        pose_array_msg.header = msg->header;

        // 对每个检测到的装甲板进行位姿解算
        for (const auto& armor : msg->armors)
        {
            // 提取角点
            std::vector<cv::Point2f> corners;
            for (int i = 0; i < 4; i++)
            {
                corners.emplace_back(armor.corners[i].x, armor.corners[i].y);
            }

            // 确定装甲板类型
            // tag_id: 1=英雄(大), 6=哨兵(大), 7=前哨站(大)
            SOLVER::ArmorType armor_type = SOLVER::ArmorType::SMALL;
            if (armor.tag_id == 1 || armor.tag_id == 6 || armor.tag_id == 7)
            {
                armor_type = SOLVER::ArmorType::LARGE;
            }

            // 执行 PnP 解算
            SOLVER::ArmorPose pose = solver_->solve(corners, armor_type, imu_yaw, imu_pitch);

            // 转换为 ROS 消息
            armor_detector_ros2::msg::ArmorPose pose_msg;
            pose_msg.header = msg->header;
            pose_msg.yaw = pose.yaw;
            pose_msg.pitch = pose.pitch;
            pose_msg.distance = pose.distance;
            pose_msg.armor_yaw = pose.armor_yaw;
            pose_msg.armor_pitch = pose.armor_pitch;
            pose_msg.color_id = armor.color_id;
            pose_msg.tag_id = armor.tag_id;
            pose_msg.armor_type = static_cast<int32_t>(armor_type);
            pose_msg.valid = pose.valid;

            // 机架坐标系位置
            pose_msg.position.x = pose.position(0);
            pose_msg.position.y = pose.position(1);
            pose_msg.position.z = pose.position(2);

            // 相机坐标系位置
            pose_msg.position_cam.x = pose.position_cam(0);
            pose_msg.position_cam.y = pose.position_cam(1);
            pose_msg.position_cam.z = pose.position_cam(2);

            // PnP解算的旋转向量（用于重投影）
            if (!pose.rvec.empty()) {
                pose_msg.rvec[0] = pose.rvec.at<double>(0);
                pose_msg.rvec[1] = pose.rvec.at<double>(1);
                pose_msg.rvec[2] = pose.rvec.at<double>(2);
                
                // 调试：打印rvec值
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "solver_node sending rvec=[%.4f, %.4f, %.4f]",
                    pose_msg.rvec[0], pose_msg.rvec[1], pose_msg.rvec[2]);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "pose.rvec is empty!");
            }

            // 坐标系转换矩阵（按行存储为9元素数组）
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    pose_msg.t_gimbal_camera[r * 3 + c] = pose.T_gimbal_camera(r, c);
                    pose_msg.r_body_gimbal[r * 3 + c] = pose.R_body_gimbal(r, c);
                }
            }

            // 角点3D位置（机架坐标系）
            for (int i = 0; i < 4; i++)
            {
                pose_msg.corners_3d[i].x = pose.corners_3d[i](0);
                pose_msg.corners_3d[i].y = pose.corners_3d[i](1);
                pose_msg.corners_3d[i].z = pose.corners_3d[i](2);
            }

            // ==================== 计算像素空间信息 ====================
            // 角点顺序: 左上(0), 左下(1), 右下(2), 右上(3)
            
            // 计算像素中心（四角点平均值）
            pose_msg.center_pixel_x = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0;
            pose_msg.center_pixel_y = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0;
            
            // 计算宽高比
            // 上边宽度: |右上.x - 左上.x|
            double top_width = std::abs(corners[3].x - corners[0].x);
            // 下边宽度: |右下.x - 左下.x|
            double bottom_width = std::abs(corners[2].x - corners[1].x);
            // 平均宽度
            double avg_width = (top_width + bottom_width) / 2.0;
            
            // 左边高度: |左下.y - 左上.y|
            double left_height = std::abs(corners[1].y - corners[0].y);
            // 右边高度: |右下.y - 右上.y|
            double right_height = std::abs(corners[2].y - corners[3].y);
            // 平均高度
            double avg_height = (left_height + right_height) / 2.0;
            
            // 宽高比 (避免除以零)
            pose_msg.aspect_ratio = (avg_height > 1e-6) ? (avg_width / avg_height) : -1.0;

            pose_array_msg.poses.push_back(pose_msg);
        }

        // 计算解算耗时
        auto end_time = std::chrono::high_resolution_clock::now();
        pose_array_msg.solve_time_ms = std::chrono::duration<float, std::milli>(
            end_time - start_time
        ).count();

        // 添加当前帧对应的IMU数据
        pose_array_msg.imu_yaw = imu_yaw;
        pose_array_msg.imu_pitch = imu_pitch;

        // 发布结果
        pose_pub_->publish(pose_array_msg);

        // 更新统计
        frame_count_++;
        total_solve_time_ += pose_array_msg.solve_time_ms;

        // 定期输出统计信息
        if (frame_count_ % 100 == 0)
        {
            float avg_time = total_solve_time_ / frame_count_;
            RCLCPP_INFO(this->get_logger(),
                "Stats - Frames: %d, Avg solve time: %.2f ms, Current poses: %zu",
                frame_count_, avg_time, pose_array_msg.poses.size());
        }
    }

    // ==================== 成员变量 ====================
    rclcpp::Publisher<armor_detector_ros2::msg::ArmorPoseArray>::SharedPtr pose_pub_;
    rclcpp::Subscription<armor_detector_ros2::msg::ArmorBBoxArray>::SharedPtr armor_sub_;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr imu_sub_;

    std::unique_ptr<SOLVER::PoseSolver> solver_;

    std::mutex imu_mutex_;
    std::deque<geometry_msgs::msg::QuaternionStamped::SharedPtr> imu_buffer_;  // IMU 数据缓存
    bool imu_received_ = false;

    int frame_count_ = 0;
    float total_solve_time_ = 0.0f;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SolverNode>());
    rclcpp::shutdown();
    return 0;
}
