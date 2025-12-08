/**
 * @file solver_node.cpp
 * @brief 装甲板位姿解算 ROS2 节点
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
 * 参数：
 * - camera_intrinsics_path: 相机内参配置文件路径
 * 
 * Requirements: 2.1, 2.2
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>
#include <memory>
#include <mutex>
#include <cmath>

#include "core/pose_solver.hpp"
#include "armor_detector_ros2/msg/armor_b_box.hpp"
#include "armor_detector_ros2/msg/armor_b_box_array.hpp"
#include "armor_detector_ros2/msg/armor_pose.hpp"
#include "armor_detector_ros2/msg/armor_pose_array.hpp"
#include "armor_detector_ros2/msg/point2f.hpp"

using namespace std::chrono_literals;

/**
 * @brief 从四元数计算欧拉角
 * @param q 四元数 (w, x, y, z)
 * @param yaw 输出 yaw 角（度）
 * @param pitch 输出 pitch 角（度）
 * @param roll 输出 roll 角（度）
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
 * 
 * 核心功能：
 * 1. 订阅检测结果和 IMU 数据
 * 2. 使用 PoseSolver 进行 PnP 解算
 * 3. 发布位姿结果
 */
class SolverNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     */
    SolverNode() : Node("solver_node")
    {
        // ==================== 参数声明 ====================
        this->declare_parameter<std::string>("camera_intrinsics_path", "");
        this->declare_parameter<double>("fx", 1280.0);
        this->declare_parameter<double>("fy", 1280.0);
        this->declare_parameter<double>("cx", 640.0);
        this->declare_parameter<double>("cy", 360.0);
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
                RCLCPP_INFO(this->get_logger(), "从 YAML 文件加载相机内参: %s", 
                           camera_intrinsics_path.c_str());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "无法从 YAML 文件加载相机内参，使用参数配置");
                loadIntrinsicsFromParams();
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "使用参数配置相机内参");
            loadIntrinsicsFromParams();
        }

        // 验证相机内参
        if (!solver_->isCameraIntrinsicsSet())
        {
            RCLCPP_ERROR(this->get_logger(), "相机内参未正确设置！");
        }
        else
        {
            const auto& intrinsics = solver_->getCameraIntrinsics();
            RCLCPP_INFO(this->get_logger(), "相机内参: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f",
                       intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy);
        }

        // ==================== 创建发布者 ====================
        pose_pub_ = this->create_publisher<armor_detector_ros2::msg::ArmorPoseArray>(
            "/solver/armor_poses",
            rclcpp::QoS(10).reliable()
        );

        // ==================== 创建订阅者 ====================
        armor_sub_ = this->create_subscription<armor_detector_ros2::msg::ArmorBBoxArray>(
            "/detector/armors",
            rclcpp::QoS(10).reliable(),
            std::bind(&SolverNode::armorCallback, this, std::placeholders::_1)
        );

        imu_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
            "/imu/quaternion",
            rclcpp::QoS(10).reliable(),
            std::bind(&SolverNode::imuCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "位姿解算节点已启动");
    }

private:
    /**
     * @brief 从参数加载相机内参
     */
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

    /**
     * @brief IMU 数据回调函数
     * @param msg IMU 四元数消息
     */
    void imuCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        latest_imu_ = msg;
        imu_received_ = true;
    }

    /**
     * @brief 装甲板检测结果回调函数
     * @param msg 检测结果消息
     */
    void armorCallback(const armor_detector_ros2::msg::ArmorBBoxArray::SharedPtr msg)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        // 获取最新的 IMU 数据
        double imu_yaw = 0.0;
        double imu_pitch = 0.0;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            if (imu_received_ && latest_imu_)
            {
                double roll;
                quaternionToEuler(
                    latest_imu_->quaternion.w,
                    latest_imu_->quaternion.x,
                    latest_imu_->quaternion.y,
                    latest_imu_->quaternion.z,
                    imu_yaw, imu_pitch, roll
                );
            }
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
            // 根据 tag_id 判断：英雄(1)和哨兵(6)使用大装甲板
            SOLVER::ArmorType armor_type = SOLVER::ArmorType::SMALL;
            if (armor.tag_id == 1 || armor.tag_id == 6)
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
            pose_msg.armor_yaw = pose.theta_world;
            pose_msg.color_id = armor.color_id;
            pose_msg.tag_id = armor.tag_id;
            pose_msg.armor_type = static_cast<int32_t>(armor_type);
            pose_msg.valid = pose.valid;

            // 设置位置
            pose_msg.position.x = pose.position(0);
            pose_msg.position.y = pose.position(1);
            pose_msg.position.z = pose.position(2);

            pose_array_msg.poses.push_back(pose_msg);
        }

        // 计算解算耗时
        auto end_time = std::chrono::high_resolution_clock::now();
        pose_array_msg.solve_time_ms = std::chrono::duration<float, std::milli>(
            end_time - start_time
        ).count();

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
                "解算统计 - 帧数: %d, 平均耗时: %.2f ms, 当前解算数: %zu",
                frame_count_, avg_time, pose_array_msg.poses.size());
        }
    }

    // ==================== 成员变量 ====================
    // 发布者
    rclcpp::Publisher<armor_detector_ros2::msg::ArmorPoseArray>::SharedPtr pose_pub_;

    // 订阅者
    rclcpp::Subscription<armor_detector_ros2::msg::ArmorBBoxArray>::SharedPtr armor_sub_;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr imu_sub_;

    // 位姿解算器
    std::unique_ptr<SOLVER::PoseSolver> solver_;

    // IMU 数据
    std::mutex imu_mutex_;
    geometry_msgs::msg::QuaternionStamped::SharedPtr latest_imu_;
    bool imu_received_ = false;

    // 统计
    int frame_count_ = 0;
    float total_solve_time_ = 0.0f;
};

/**
 * @brief 主函数
 */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SolverNode>());
    rclcpp::shutdown();
    return 0;
}
