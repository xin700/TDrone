/**
 * @file mock_imu_node.cpp
 * @brief 模拟 IMU ROS2 节点
 * 
 * 功能：发布模拟的 IMU 四元数数据到 /imu/quaternion 话题
 * 用于开发和测试时模拟真实 IMU 输入
 * 
 * 发布话题：
 * - /imu/quaternion (geometry_msgs/QuaternionStamped): IMU 姿态四元数
 * 
 * 参数：
 * - publish_rate: 发布频率（默认200 Hz）
 * - mode: 模式 (static/rotating/oscillating)
 *   - static: 静态四元数（单位四元数）
 *   - rotating: 绕 Z 轴匀速旋转
 *   - oscillating: 绕 Y 轴振荡（模拟云台运动）
 * - rotation_speed: 旋转速度（rad/s，默认0.5）
 * - oscillation_amplitude: 振荡幅度（rad，默认0.3）
 * - oscillation_frequency: 振荡频率（Hz，默认1.0）
 * 
 * Requirements: 9.3
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <chrono>
#include <string>
#include <cmath>

using namespace std::chrono_literals;

/**
 * @class MockIMUNode
 * @brief 模拟 IMU 节点类
 * 
 * 发布模拟的 IMU 四元数数据，支持多种模式
 */
class MockIMUNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     */
    MockIMUNode() : Node("mock_imu_node")
    {
        // ==================== 参数声明 ====================
        this->declare_parameter<double>("publish_rate", 200.0);
        this->declare_parameter<std::string>("mode", "static");
        this->declare_parameter<double>("rotation_speed", 0.5);
        this->declare_parameter<double>("oscillation_amplitude", 0.3);
        this->declare_parameter<double>("oscillation_frequency", 1.0);

        // 获取参数
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        mode_ = this->get_parameter("mode").as_string();
        rotation_speed_ = this->get_parameter("rotation_speed").as_double();
        oscillation_amplitude_ = this->get_parameter("oscillation_amplitude").as_double();
        oscillation_frequency_ = this->get_parameter("oscillation_frequency").as_double();

        RCLCPP_INFO(this->get_logger(), "MockIMU 配置:");
        RCLCPP_INFO(this->get_logger(), "  发布频率: %.1f Hz", publish_rate_);
        RCLCPP_INFO(this->get_logger(), "  模式: %s", mode_.c_str());
        if (mode_ == "rotating") {
            RCLCPP_INFO(this->get_logger(), "  旋转速度: %.2f rad/s", rotation_speed_);
        } else if (mode_ == "oscillating") {
            RCLCPP_INFO(this->get_logger(), "  振荡幅度: %.2f rad", oscillation_amplitude_);
            RCLCPP_INFO(this->get_logger(), "  振荡频率: %.2f Hz", oscillation_frequency_);
        }

        // ==================== 创建发布者 ====================
        quaternion_pub_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
            "/imu/quaternion", 
            rclcpp::QoS(10).reliable()
        );

        // ==================== 创建定时器 ====================
        auto period = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&MockIMUNode::timer_callback, this)
        );

        start_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "MockIMU 节点已启动，发布到 /imu/quaternion");
    }

    /**
     * @brief 获取已发布消息数
     */
    int get_publish_count() const { return publish_count_; }

    /**
     * @brief 获取发布频率
     */
    double get_publish_rate() const { return publish_rate_; }

private:
    /**
     * @brief 定时器回调函数
     */
    void timer_callback()
    {
        auto msg = geometry_msgs::msg::QuaternionStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "imu_frame";

        // 计算经过的时间
        double elapsed = (this->get_clock()->now() - start_time_).seconds();

        // 根据模式生成四元数
        if (mode_ == "static") {
            // 静态模式：单位四元数
            msg.quaternion.w = 1.0;
            msg.quaternion.x = 0.0;
            msg.quaternion.y = 0.0;
            msg.quaternion.z = 0.0;
        } else if (mode_ == "rotating") {
            // 旋转模式：绕 Z 轴匀速旋转
            double angle = rotation_speed_ * elapsed;
            msg.quaternion.w = std::cos(angle / 2.0);
            msg.quaternion.x = 0.0;
            msg.quaternion.y = 0.0;
            msg.quaternion.z = std::sin(angle / 2.0);
        } else if (mode_ == "oscillating") {
            // 振荡模式：绕 Y 轴振荡（模拟云台 pitch 运动）
            double angle = oscillation_amplitude_ * std::sin(2.0 * M_PI * oscillation_frequency_ * elapsed);
            msg.quaternion.w = std::cos(angle / 2.0);
            msg.quaternion.x = 0.0;
            msg.quaternion.y = std::sin(angle / 2.0);
            msg.quaternion.z = 0.0;
        } else {
            // 默认：单位四元数
            msg.quaternion.w = 1.0;
            msg.quaternion.x = 0.0;
            msg.quaternion.y = 0.0;
            msg.quaternion.z = 0.0;
        }

        // 发布
        quaternion_pub_->publish(msg);

        // 更新计数
        publish_count_++;
        if (publish_count_ % 1000 == 0) {
            RCLCPP_INFO(this->get_logger(), "MockIMU 已发布 %d 条消息", publish_count_);
        }
    }

    // ==================== 成员变量 ====================
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr quaternion_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double publish_rate_;
    std::string mode_;
    double rotation_speed_;
    double oscillation_amplitude_;
    double oscillation_frequency_;

    rclcpp::Time start_time_;
    int publish_count_ = 0;
};

/**
 * @brief 主函数
 */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockIMUNode>());
    rclcpp::shutdown();
    return 0;
}
