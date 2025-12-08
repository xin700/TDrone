/**
 * @file mock_cboard_node.cpp
 * @brief 模拟 CBoard（下位机）ROS2 节点
 * 
 * 功能：
 * - 发布模拟弹速到 /cboard/bullet_speed
 * - 接收控制指令从 /aimer/command
 * - 记录接收到的控制指令用于调试
 * 
 * 发布话题：
 * - /cboard/bullet_speed (std_msgs/Float64): 弹速
 * - /cboard/mode (std_msgs/Int32): 当前模式
 * 
 * 订阅话题：
 * - /aimer/command (armor_detector_ros2/Command): 控制指令
 * 
 * 参数：
 * - publish_rate: 发布频率（默认100 Hz）
 * - bullet_speed: 模拟弹速（默认25.0 m/s）
 * - bullet_speed_noise: 弹速噪声标准差（默认0.5 m/s）
 * - mode: 当前模式（默认0）
 * 
 * Requirements: 9.3
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/header.hpp>
#include <chrono>
#include <string>
#include <random>
#include <mutex>

using namespace std::chrono_literals;

/**
 * @brief 简化的 Command 结构（避免依赖自定义消息）
 * 
 * 在实际使用中，应该使用 armor_detector_ros2/Command 消息
 * 这里为了测试方便，使用简化的结构
 */
struct CommandData {
    bool control = false;
    bool fire = false;
    double yaw = 0.0;
    double pitch = 0.0;
    double yaw_velocity = 0.0;
    double pitch_velocity = 0.0;
    rclcpp::Time timestamp;
};

/**
 * @class MockCBoardNode
 * @brief 模拟 CBoard 节点类
 * 
 * 模拟下位机通信，发布弹速和模式，接收控制指令
 */
class MockCBoardNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     */
    MockCBoardNode() : Node("mock_cboard_node"), gen_(rd_())
    {
        // ==================== 参数声明 ====================
        this->declare_parameter<double>("publish_rate", 100.0);
        this->declare_parameter<double>("bullet_speed", 25.0);
        this->declare_parameter<double>("bullet_speed_noise", 0.5);
        this->declare_parameter<int>("mode", 0);

        // 获取参数
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        bullet_speed_ = this->get_parameter("bullet_speed").as_double();
        bullet_speed_noise_ = this->get_parameter("bullet_speed_noise").as_double();
        mode_ = this->get_parameter("mode").as_int();

        // 初始化噪声分布
        noise_dist_ = std::normal_distribution<double>(0.0, bullet_speed_noise_);

        RCLCPP_INFO(this->get_logger(), "MockCBoard 配置:");
        RCLCPP_INFO(this->get_logger(), "  发布频率: %.1f Hz", publish_rate_);
        RCLCPP_INFO(this->get_logger(), "  弹速: %.2f m/s (噪声: %.2f)", bullet_speed_, bullet_speed_noise_);
        RCLCPP_INFO(this->get_logger(), "  模式: %d", mode_);

        // ==================== 创建发布者 ====================
        bullet_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/cboard/bullet_speed", 
            rclcpp::QoS(10).reliable()
        );

        mode_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/cboard/mode", 
            rclcpp::QoS(10).reliable()
        );

        // ==================== 创建订阅者 ====================
        // 使用通用消息类型订阅控制指令
        // 在实际使用中应该使用 armor_detector_ros2::msg::Command
        command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/aimer/command_yaw", 10,
            std::bind(&MockCBoardNode::command_callback, this, std::placeholders::_1));

        // ==================== 创建定时器 ====================
        auto period = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&MockCBoardNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "MockCBoard 节点已启动");
        RCLCPP_INFO(this->get_logger(), "  发布: /cboard/bullet_speed, /cboard/mode");
        RCLCPP_INFO(this->get_logger(), "  订阅: /aimer/command_yaw");
    }

    /**
     * @brief 获取已发布消息数
     */
    int get_publish_count() const { return publish_count_; }

    /**
     * @brief 获取已接收命令数
     */
    int get_command_count() const { return command_count_; }

    /**
     * @brief 获取最后接收的命令
     */
    CommandData get_last_command() const {
        std::lock_guard<std::mutex> lock(command_mutex_);
        return last_command_;
    }

    /**
     * @brief 获取弹速
     */
    double get_bullet_speed() const { return bullet_speed_; }

    /**
     * @brief 设置弹速
     */
    void set_bullet_speed(double speed) { bullet_speed_ = speed; }

    /**
     * @brief 获取模式
     */
    int get_mode() const { return mode_; }

    /**
     * @brief 设置模式
     */
    void set_mode(int mode) { mode_ = mode; }

private:
    /**
     * @brief 定时器回调函数 - 发布弹速和模式
     */
    void timer_callback()
    {
        // 发布弹速（带噪声）
        auto bullet_speed_msg = std_msgs::msg::Float64();
        bullet_speed_msg.data = bullet_speed_ + noise_dist_(gen_);
        bullet_speed_pub_->publish(bullet_speed_msg);

        // 发布模式
        auto mode_msg = std_msgs::msg::Int32();
        mode_msg.data = mode_;
        mode_pub_->publish(mode_msg);

        // 更新计数
        publish_count_++;
        if (publish_count_ % 500 == 0) {
            RCLCPP_INFO(this->get_logger(), "MockCBoard 已发布 %d 条消息，接收 %d 条命令", 
                        publish_count_, command_count_.load());
        }
    }

    /**
     * @brief 命令回调函数
     */
    void command_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        last_command_.yaw = msg->data;
        last_command_.timestamp = this->get_clock()->now();
        command_count_++;

        if (command_count_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "MockCBoard 接收命令 #%d: yaw=%.4f", 
                        command_count_.load(), msg->data);
        }
    }

    // ==================== 成员变量 ====================
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bullet_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double publish_rate_;
    double bullet_speed_;
    double bullet_speed_noise_;
    int mode_;

    // 随机数生成器
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<double> noise_dist_;

    // 命令记录
    mutable std::mutex command_mutex_;
    CommandData last_command_;
    std::atomic<int> command_count_{0};
    int publish_count_ = 0;
};

/**
 * @brief 主函数
 */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockCBoardNode>());
    rclcpp::shutdown();
    return 0;
}
