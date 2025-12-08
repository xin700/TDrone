/**
 * @file outpost_predictor_node.cpp
 * @brief 前哨站预测 ROS2 节点
 * 
 * 功能：订阅装甲板位姿解算结果，使用 EKF 估计前哨站旋转状态，发布预测结果
 * 
 * 订阅话题：
 * - /solver/armor_poses (ArmorPoseArray): 位姿解算结果
 * 
 * 发布话题：
 * - /outpost/prediction (OutpostState): 前哨站状态预测
 * 
 * 参数：
 * - outpost_tag_id: 前哨站装甲板的 tag_id (默认: 7)
 * - chi_square_threshold: 卡方检验阈值 (默认: 11.07)
 * - initial_direction: 初始旋转方向 (默认: 0)
 * 
 * Requirements: 7.1, 7.2, 7.3
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <chrono>
#include <memory>

#include "core/outpost_estimator.hpp"
#include "armor_detector_ros2/msg/armor_pose.hpp"
#include "armor_detector_ros2/msg/armor_pose_array.hpp"
#include "armor_detector_ros2/msg/outpost_state.hpp"

using namespace std::chrono_literals;

/**
 * @class OutpostPredictorNode
 * @brief 前哨站预测节点类
 * 
 * 核心功能：
 * 1. 订阅装甲板位姿解算结果
 * 2. 过滤出前哨站装甲板
 * 3. 使用 OutpostEstimator 进行状态估计
 * 4. 发布前哨站状态预测
 */
class OutpostPredictorNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     */
    OutpostPredictorNode() : Node("outpost_predictor_node")
    {
        // ==================== 参数声明 ====================
        this->declare_parameter<int>("outpost_tag_id", 7);
        this->declare_parameter<double>("chi_square_threshold", 11.07);
        this->declare_parameter<int>("initial_direction", 0);
        this->declare_parameter<double>("prediction_dt", 0.01);
        this->declare_parameter<int>("min_observations_for_valid", 3);  // 降低阈值，因为前哨站旋转时检测不连续
        this->declare_parameter<double>("observation_timeout", 3.0);  // 超时时间（秒），前哨站旋转一圈约2秒
        this->declare_parameter<bool>("accept_any_tag", false);  // 是否接受任何 tag_id
        this->declare_parameter<double>("prediction_ahead_time", 0.1);  // 预测超前时间（秒），用于补偿延迟

        // 获取参数
        outpost_tag_id_ = this->get_parameter("outpost_tag_id").as_int();
        accept_any_tag_ = this->get_parameter("accept_any_tag").as_bool();
        double chi_square_threshold = this->get_parameter("chi_square_threshold").as_double();
        initial_direction_ = this->get_parameter("initial_direction").as_int();
        prediction_dt_ = this->get_parameter("prediction_dt").as_double();
        min_observations_for_valid_ = this->get_parameter("min_observations_for_valid").as_int();
        observation_timeout_ = this->get_parameter("observation_timeout").as_double();
        prediction_ahead_time_ = this->get_parameter("prediction_ahead_time").as_double();

        // ==================== 初始化前哨站估计器 ====================
        estimator_ = std::make_unique<armor_detector::OutpostEstimator>();
        estimator_->setChiSquareThreshold(chi_square_threshold);
        estimator_->setDeltaTime(prediction_dt_);

        RCLCPP_INFO(this->get_logger(), 
            "前哨站预测器参数: tag_id=%d, chi_square_threshold=%.2f, initial_direction=%d, accept_any_tag=%s",
            outpost_tag_id_, chi_square_threshold, initial_direction_, accept_any_tag_ ? "true" : "false");

        // ==================== 创建发布者 ====================
        prediction_pub_ = this->create_publisher<armor_detector_ros2::msg::OutpostState>(
            "/outpost/prediction",
            rclcpp::QoS(10).reliable()
        );

        // ==================== 创建订阅者 ====================
        pose_sub_ = this->create_subscription<armor_detector_ros2::msg::ArmorPoseArray>(
            "/solver/armor_poses",
            rclcpp::QoS(10).reliable(),
            std::bind(&OutpostPredictorNode::poseCallback, this, std::placeholders::_1)
        );

        // ==================== 创建定时器用于周期性预测 ====================
        prediction_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(prediction_dt_ * 1000)),
            std::bind(&OutpostPredictorNode::predictionTimerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "前哨站预测节点已启动");
    }

private:
    /**
     * @brief 装甲板位姿回调函数
     * @param msg 位姿解算结果消息
     */
    void poseCallback(const armor_detector_ros2::msg::ArmorPoseArray::SharedPtr msg)
    {
        total_pose_callbacks_++;
        
        // 调试：每30帧输出一次状态
        if (total_pose_callbacks_ % 30 == 1)
        {
            if (msg->poses.empty())
            {
                RCLCPP_INFO(this->get_logger(), 
                    "[帧 %d] 收到空的位姿数组，没有检测到装甲板",
                    total_pose_callbacks_);
            }
            else
            {
                std::string armor_info = "";
                for (const auto& pose : msg->poses)
                {
                    char buf[64];
                    snprintf(buf, sizeof(buf), "(tag=%d,valid=%d) ", pose.tag_id, pose.valid ? 1 : 0);
                    armor_info += buf;
                }
                RCLCPP_INFO(this->get_logger(), 
                    "[帧 %d] 收到 %zu 个装甲板: %s| accept_any_tag=%s, outpost_tag_id=%d",
                    total_pose_callbacks_, msg->poses.size(), armor_info.c_str(),
                    accept_any_tag_ ? "true" : "false", outpost_tag_id_);
            }
        }
        
        // 查找前哨站装甲板
        const armor_detector_ros2::msg::ArmorPose* outpost_armor = nullptr;
        
        for (const auto& pose : msg->poses)
        {
            // 如果 accept_any_tag_ 为 true，接受任何有效的装甲板
            // 否则只接受 tag_id == outpost_tag_id_ 的装甲板
            if (pose.valid && (accept_any_tag_ || pose.tag_id == outpost_tag_id_))
            {
                outpost_armor = &pose;
                break;
            }
        }

        // 如果没有检测到前哨站装甲板
        if (!outpost_armor)
        {
            // 如果估计器已初始化，执行纯预测并发布预测状态
            if (estimator_->isInitialized())
            {
                estimator_->predict();
                // 发布预测状态（即使没有新观测，也使用 EKF 预测值）
                publishPrediction(msg->header);
            }
            else
            {
                // 估计器未初始化，发布等待状态
                publishWaitingState(msg->header);
            }
            return;
        }
        
        // 找到前哨站装甲板
        if (total_pose_callbacks_ % 30 == 1)
        {
            RCLCPP_INFO(this->get_logger(), 
                "检测到前哨站装甲板! yaw=%.2f, pitch=%.2f, dist=%.2f",
                outpost_armor->yaw, outpost_armor->pitch, outpost_armor->distance);
        }

        // 更新时间戳
        auto current_time = this->now();
        if (last_observation_time_.nanoseconds() > 0)
        {
            double dt = (current_time - last_observation_time_).seconds();
            if (dt > 0.0 && dt < 1.0)  // 合理的时间间隔
            {
                estimator_->setDeltaTime(dt);
            }
        }
        last_observation_time_ = current_time;

        // 构造测量值
        armor_detector::ArmorMeasurement measurement;
        measurement.yaw = outpost_armor->yaw;
        measurement.pitch = outpost_armor->pitch;
        measurement.distance = outpost_armor->distance;
        measurement.theta_world = outpost_armor->armor_yaw;
        measurement.timestamp = current_time.seconds();

        // 初始化或更新估计器
        if (!estimator_->isInitialized())
        {
            estimator_->initialize(measurement, initial_direction_);
            observation_count_ = 1;
            RCLCPP_INFO(this->get_logger(), "前哨站估计器已初始化");
        }
        else
        {
            bool accepted = estimator_->update(measurement);
            if (accepted)
            {
                observation_count_++;
                
                // 每 30 次接受的观测输出一次
                if (observation_count_ % 30 == 1 || observation_count_ <= 5)
                {
                    RCLCPP_INFO(this->get_logger(), 
                        "观测被接受, observation_count=%d, min_required=%d, valid=%s",
                        observation_count_, min_observations_for_valid_,
                        (observation_count_ >= min_observations_for_valid_) ? "true" : "false");
                }
                
                // 检测方向变化
                if (estimator_->detectDirectionChange())
                {
                    RCLCPP_INFO(this->get_logger(), 
                        "检测到前哨站旋转方向变化: %d", estimator_->getDirection());
                }
            }
            else
            {
                rejected_count_++;
                if (rejected_count_ % 30 == 1)
                {
                    RCLCPP_WARN(this->get_logger(), 
                        "卡方检验拒绝观测 (累计: %d), chi2=%.2f",
                        rejected_count_, estimator_->getLastChiSquareValue());
                }
            }
        }

        // 发布预测结果
        publishPrediction(msg->header);
    }

    /**
     * @brief 定时器回调函数，用于周期性预测
     */
    void predictionTimerCallback()
    {
        if (!estimator_->isInitialized())
        {
            return;
        }

        // 检查是否长时间没有观测
        auto current_time = this->now();
        if (last_observation_time_.nanoseconds() > 0)
        {
            double time_since_observation = (current_time - last_observation_time_).seconds();
            
            // 如果超过超时时间没有观测，重置估计器
            if (time_since_observation > observation_timeout_)
            {
                RCLCPP_WARN(this->get_logger(), 
                    "前哨站观测超时 (%.2f s > %.2f s)，重置估计器", 
                    time_since_observation, observation_timeout_);
                estimator_->reset();
                observation_count_ = 0;
                return;
            }
        }
    }

    /**
     * @brief 发布前哨站状态预测
     * @param header 消息头
     */
    void publishPrediction(const std_msgs::msg::Header& header)
    {
        armor_detector_ros2::msg::OutpostState state_msg;
        state_msg.header = header;

        // 获取估计器状态（使用预测状态，而不是滤波后的状态）
        auto info = estimator_->getInformation(true);  // true = 使用预测状态

        // 应用超前预测时间：theta_predicted = theta + omega * prediction_ahead_time
        // 这样可以补偿系统延迟（检测延迟 + 弹道飞行时间）
        double theta_ahead = info.outpost_theta + info.outpost_omega * prediction_ahead_time_;
        
        // 角度归一化到 [-pi, pi]
        while (theta_ahead > M_PI) theta_ahead -= 2 * M_PI;
        while (theta_ahead < -M_PI) theta_ahead += 2 * M_PI;

        // 填充消息
        state_msg.center.x = info.center_position.x();
        state_msg.center.y = info.center_position.y();
        state_msg.center.z = info.center_position.z();

        state_msg.velocity.x = info.center_velocity.x();
        state_msg.velocity.y = info.center_velocity.y();
        state_msg.velocity.z = info.center_velocity.z();

        state_msg.radius = info.outpost_radius;
        state_msg.theta = theta_ahead;  // 使用超前预测的角度
        state_msg.omega = info.outpost_omega;
        state_msg.direction = info.direction;

        // 只有在足够多的观测后才认为有效
        state_msg.valid = info.is_valid && 
                          (observation_count_ >= min_observations_for_valid_);

        // 发布
        prediction_pub_->publish(state_msg);

        // 定期输出统计信息
        if (observation_count_ % 100 == 0 && observation_count_ > 0)
        {
            RCLCPP_INFO(this->get_logger(),
                "前哨站统计 - 观测数: %d, 拒绝数: %d, omega: %.3f rad/s, direction: %d, ahead_time: %.3f s",
                observation_count_, rejected_count_, info.outpost_omega, info.direction, prediction_ahead_time_);
        }
    }

    // ==================== 成员变量 ====================
    // 发布者
    rclcpp::Publisher<armor_detector_ros2::msg::OutpostState>::SharedPtr prediction_pub_;

    // 订阅者
    rclcpp::Subscription<armor_detector_ros2::msg::ArmorPoseArray>::SharedPtr pose_sub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr prediction_timer_;

    // 前哨站估计器
    std::unique_ptr<armor_detector::OutpostEstimator> estimator_;

    // 参数
    int outpost_tag_id_;
    int initial_direction_;
    double prediction_dt_;
    int min_observations_for_valid_;
    bool accept_any_tag_;
    double observation_timeout_;
    double prediction_ahead_time_;

    // 状态
    rclcpp::Time last_observation_time_{0, 0, RCL_ROS_TIME};
    int observation_count_ = 0;
    int rejected_count_ = 0;
    int total_pose_callbacks_ = 0;
    
    /**
     * @brief 发布等待状态（未检测到前哨站时）
     */
    void publishWaitingState(const std_msgs::msg::Header& header)
    {
        armor_detector_ros2::msg::OutpostState state_msg;
        state_msg.header = header;
        state_msg.valid = false;
        state_msg.direction = 0;
        state_msg.radius = 0.275;  // 默认前哨站半径
        state_msg.theta = 0.0;
        state_msg.omega = 0.0;
        prediction_pub_->publish(state_msg);
    }
};

/**
 * @brief 主函数
 */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OutpostPredictorNode>());
    rclcpp::shutdown();
    return 0;
}
