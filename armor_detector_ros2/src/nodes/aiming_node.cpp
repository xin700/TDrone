/**
 * @file aiming_node.cpp
 * @brief 瞄准控制节点
 * 
 * 功能：
 *   - 订阅预测器发布的预瞄结果 (/predictor_v4/aim_result)
 *   - 实现三种瞄准状态：初始化、追踪、切换
 *   - 处理世界坐标系到云台坐标系的转换
 *   - 计算云台目标yaw/pitch并进行滤波
 *   - 发布云台控制指令 (/aiming/gimbal_command)
 *   - 支持pipeline_control服务控制
 * 
 * 坐标系说明：
 *   - 输入：预瞄点在机架坐标系（世界坐标系）下的位置
 *   - 输出：云台需要的绝对yaw/pitch角度
 *   - 云台坐标系：原点在云台旋转中心
 * 
 * 状态机：
 *   - INITIALIZING: EKF未收敛，等待初始化
 *   - TRACKING: 追踪预瞄点
 *   - SWITCHING: 检测到装甲板切换，平滑过渡到下一块装甲板
 */

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <deque>
#include <cmath>
#include <Eigen/Dense>

#include "armor_detector_ros2/msg/outpost_aim_result.hpp"
#include "armor_detector_ros2/msg/gimbal_command.hpp"

namespace armor_detector
{

// 瞄准状态枚举
enum class AimingState : int
{
    INITIALIZING = 0,  // 初始化中（EKF未收敛）
    TRACKING = 1,      // 追踪状态
    SWITCHING = 2      // 切换状态
};

class AimingNode : public rclcpp::Node
{
public:
    AimingNode() : Node("aiming_node")
    {
        RCLCPP_INFO(this->get_logger(), "初始化瞄准控制节点...");
        
        // 声明参数
        declareParameters();
        
        // 加载参数
        loadParameters();
        
        // 创建订阅者
        aim_result_sub_ = this->create_subscription<armor_detector_ros2::msg::OutpostAimResult>(
            "/predictor_v4/aim_result", 10,
            std::bind(&AimingNode::aimResultCallback, this, std::placeholders::_1));
        
        // 创建发布者
        gimbal_cmd_pub_ = this->create_publisher<armor_detector_ros2::msg::GimbalCommand>(
            "/aiming/gimbal_command", 10);
        
        // 创建pipeline_control服务
        pipeline_control_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "/aiming/pipeline_control",
            std::bind(&AimingNode::pipelineControlCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        // 参数变化回调
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&AimingNode::parametersCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "瞄准控制节点初始化完成");
        RCLCPP_INFO(this->get_logger(), "======================================");
        RCLCPP_INFO(this->get_logger(), "  订阅: /predictor_v4/aim_result");
        RCLCPP_INFO(this->get_logger(), "  发布: /aiming/gimbal_command");
        RCLCPP_INFO(this->get_logger(), "  服务: /aiming/pipeline_control");
        RCLCPP_INFO(this->get_logger(), "======================================");
    }

private:
    void declareParameters()
    {
        // 滤波器参数
        this->declare_parameter("filter.window_size", 5);
        
        // 切换检测参数
        this->declare_parameter("switch.angle_jump_threshold", 60.0);  // 突变检测阈值（度）
        this->declare_parameter("switch.yaw_match_threshold", 3.0);    // yaw匹配阈值（度）
        this->declare_parameter("switch.moving_delay", 0.2);           // 切换运动时间（秒）
        
        // 重力补偿参数
        this->declare_parameter("gravity.g", 9.8);                     // 重力加速度
        this->declare_parameter("gravity.bullet_speed", 28.0);         // 子弹速度
        
        // 相机偏移（相机相对云台旋转中心的偏移，单位：米）
        this->declare_parameter("camera_offset.x", 0.0);
        this->declare_parameter("camera_offset.y", 0.0);
        this->declare_parameter("camera_offset.z", 0.0);
    }
    
    void loadParameters()
    {
        filter_window_size_ = this->get_parameter("filter.window_size").as_int();
        angle_jump_threshold_ = this->get_parameter("switch.angle_jump_threshold").as_double();
        yaw_match_threshold_ = this->get_parameter("switch.yaw_match_threshold").as_double();
        moving_delay_ = this->get_parameter("switch.moving_delay").as_double();
        gravity_g_ = this->get_parameter("gravity.g").as_double();
        bullet_speed_ = this->get_parameter("gravity.bullet_speed").as_double();
        
        camera_offset_ = Eigen::Vector3d(
            this->get_parameter("camera_offset.x").as_double(),
            this->get_parameter("camera_offset.y").as_double(),
            this->get_parameter("camera_offset.z").as_double()
        );
        
        RCLCPP_INFO(this->get_logger(), 
            "参数加载: filter_window=%d, jump_threshold=%.1f°, match_threshold=%.1f°",
            filter_window_size_, angle_jump_threshold_, yaw_match_threshold_);
    }
    
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        for (const auto& param : params) {
            RCLCPP_INFO(this->get_logger(), "参数更新: %s", param.get_name().c_str());
        }
        
        loadParameters();
        return result;
    }
    
    void pipelineControlCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        pipeline_enabled_ = request->data;
        
        if (!pipeline_enabled_) {
            // 停止时重置状态
            resetState();
        }
        
        response->success = true;
        response->message = pipeline_enabled_ ? "Aiming pipeline enabled" : "Aiming pipeline disabled";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }
    
    void resetState()
    {
        current_state_ = AimingState::INITIALIZING;
        yaw_filter_window_.clear();
        pitch_filter_window_.clear();
        last_further_ang_ = 0.0;
        last_further_ang_valid_ = false;
        switch_start_time_ = 0.0;
        switch_target_yaw_ = 0.0;
        switch_target_pitch_ = 0.0;
        switch_start_yaw_ = 0.0;
        switch_start_pitch_ = 0.0;
    }
    
    void aimResultCallback(const armor_detector_ros2::msg::OutpostAimResult::SharedPtr msg)
    {
        if (!pipeline_enabled_) {
            return;
        }
        
        // 获取当前时间戳
        double current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        
        // 检查EKF是否已收敛
        if (!msg->ekf_converged) {
            current_state_ = AimingState::INITIALIZING;
            publishCommand(msg->header, msg->imu_yaw, msg->imu_pitch, 0.0, 0.0, 0.0, false);
            return;
        }
        
        // 检查消息是否有效
        if (!msg->valid) {
            // 无效消息时，保持当前IMU位置（不移动云台）
            publishCommand(msg->header, msg->imu_yaw, msg->imu_pitch, 0.0, 0.0, 0.0, false);
            return;
        }
        
        // 获取坐标转换矩阵
        Eigen::Matrix3d T_gimbal_camera;
        Eigen::Matrix3d R_body_gimbal;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                T_gimbal_camera(i, j) = msg->t_gimbal_camera[i * 3 + j];
                R_body_gimbal(i, j) = msg->r_body_gimbal[i * 3 + j];
            }
        }
        
        // 获取预瞄点位置（机架坐标系）
        Eigen::Vector3d aim_position_body(
            msg->aim_position.x,
            msg->aim_position.y,
            msg->aim_position.z
        );
        
        // 获取更远预瞄点位置（机架坐标系）
        Eigen::Vector3d further_aim_position_body(
            msg->further_aim_position.x,
            msg->further_aim_position.y,
            msg->further_aim_position.z
        );
        
        // ========== 坐标转换：机架坐标系 -> 云台坐标系 ==========
        // R_gimbal_body = R_body_gimbal^T（正交矩阵的逆）
        Eigen::Matrix3d R_gimbal_body = R_body_gimbal.transpose();
        
        // 先转换到云台坐标系
        Eigen::Vector3d aim_position_gimbal = R_gimbal_body * aim_position_body;
        Eigen::Vector3d further_aim_position_gimbal = R_gimbal_body * further_aim_position_body;
        
        // 考虑相机偏移：发射口在云台旋转中心，但预瞄点是相对于相机的
        // 需要从云台坐标系下减去相机偏移（近似处理）
        // 由于发射口在云台旋转中心，计算从旋转中心到目标的角度
        // 实际上这里不需要额外处理相机偏移，因为PnP解算出来的位置已经是相对于相机的
        // 而我们需要的是从发射口（云台中心）到目标的角度
        // 简化处理：假设发射口和云台旋转中心重合，直接使用云台坐标系下的位置
        
        // ========== 计算目标yaw和pitch（相对量）==========
        // 云台坐标系：X右, Y前, Z上
        // yaw = atan2(-x, y)，左正右负，需要取反
        // pitch = atan2(z, sqrt(x^2+y^2))，上正下负
        
        double dist_aim = aim_position_gimbal.norm();
        double dist_xy_aim = std::sqrt(aim_position_gimbal.x() * aim_position_gimbal.x() + 
                                       aim_position_gimbal.y() * aim_position_gimbal.y());
        
        double delta_yaw = -std::atan2(-aim_position_gimbal.x(), aim_position_gimbal.y()) * 180.0 / M_PI;
        double delta_pitch = std::atan2(aim_position_gimbal.z(), dist_xy_aim) * 180.0 / M_PI;
        
        // 计算更远预瞄点的目标角度
        double dist_xy_further = std::sqrt(further_aim_position_gimbal.x() * further_aim_position_gimbal.x() + 
                                          further_aim_position_gimbal.y() * further_aim_position_gimbal.y());
        
        double delta_yaw_further = -std::atan2(-further_aim_position_gimbal.x(), further_aim_position_gimbal.y()) * 180.0 / M_PI;
        double delta_pitch_further = std::atan2(further_aim_position_gimbal.z(), dist_xy_further) * 180.0 / M_PI;
        
        // ========== 计算绝对目标角度 ==========
        double target_yaw = msg->imu_yaw + delta_yaw;
        double target_pitch = msg->imu_pitch + delta_pitch;
        
        double target_yaw_further = msg->imu_yaw + delta_yaw_further;
        double target_pitch_further = msg->imu_pitch + delta_pitch_further;
        
        // ========== 重力补偿 ==========
        // Δpitch = (g * d^2) / (2 * v^2 * cos^2(pitch))
        // 简化近似：Δpitch ≈ g * d / v^2（小角度近似）
        double gravity_compensation = 0.0;
        if (bullet_speed_ > 0.0 && dist_aim > 0.0) {
            double t_flight = dist_aim / bullet_speed_;
            double drop = 0.5 * gravity_g_ * t_flight * t_flight;
            // 将下落量转换为角度
            gravity_compensation = std::atan2(drop, dist_aim) * 180.0 / M_PI;
        }
        target_pitch += gravity_compensation;
        target_pitch_further += gravity_compensation;
        
        // ========== 状态机处理 ==========
        double output_yaw = target_yaw;
        double output_pitch = target_pitch;
        bool fire_allowed = msg->fire_condition_1;
        
        // 检测further_aim_ang突变（装甲板切换检测）
        if (last_further_ang_valid_) {
            double ang_diff = std::abs(msg->further_aim_ang - last_further_ang_);
            // 处理跨周期的情况
            if (ang_diff > 60.0) {
                ang_diff = 120.0 - ang_diff;
            }
            
            if (ang_diff > angle_jump_threshold_ && current_state_ == AimingState::TRACKING) {
                // 检测到突变，切换到SWITCHING状态
                current_state_ = AimingState::SWITCHING;
                switch_start_time_ = current_time;
                switch_start_yaw_ = msg->imu_yaw;
                switch_start_pitch_ = msg->imu_pitch;
                switch_target_yaw_ = target_yaw_further;
                switch_target_pitch_ = target_pitch_further;
                
                RCLCPP_INFO(this->get_logger(), 
                    "检测到装甲板切换! ang_diff=%.1f°, 进入SWITCHING状态", ang_diff);
            }
        }
        last_further_ang_ = msg->further_aim_ang;
        last_further_ang_valid_ = true;
        
        // 状态处理
        switch (current_state_) {
            case AimingState::INITIALIZING:
            {
                // 初始化状态：等待aim_position的yaw与当前imu_yaw接近
                double yaw_diff = std::abs(target_yaw - msg->imu_yaw);
                if (yaw_diff < yaw_match_threshold_) {
                    current_state_ = AimingState::TRACKING;
                    RCLCPP_INFO(this->get_logger(), "进入TRACKING状态");
                }
                // 初始化状态下保持当前位置
                output_yaw = msg->imu_yaw;
                output_pitch = msg->imu_pitch;
                fire_allowed = false;
                break;
            }
            
            case AimingState::TRACKING:
            {
                // 追踪状态：跟随滤波后的预瞄yaw和pitch
                yaw_filter_window_.push_back(target_yaw);
                pitch_filter_window_.push_back(target_pitch);
                
                while (yaw_filter_window_.size() > static_cast<size_t>(filter_window_size_)) {
                    yaw_filter_window_.pop_front();
                    pitch_filter_window_.pop_front();
                }
                
                // 计算滑动平均
                output_yaw = 0.0;
                output_pitch = 0.0;
                for (size_t i = 0; i < yaw_filter_window_.size(); i++) {
                    output_yaw += yaw_filter_window_[i];
                    output_pitch += pitch_filter_window_[i];
                }
                output_yaw /= static_cast<double>(yaw_filter_window_.size());
                output_pitch /= static_cast<double>(pitch_filter_window_.size());
                break;
            }
            
            case AimingState::SWITCHING:
            {
                // 切换状态：插值运动到further_aim位置
                double elapsed = current_time - switch_start_time_;
                double progress = std::min(elapsed / moving_delay_, 1.0);
                
                // 线性插值
                output_yaw = switch_start_yaw_ + (switch_target_yaw_ - switch_start_yaw_) * progress;
                output_pitch = switch_start_pitch_ + (switch_target_pitch_ - switch_start_pitch_) * progress;
                
                // 检查是否可以切换回TRACKING状态
                double yaw_diff = std::abs(target_yaw - msg->imu_yaw);
                if (yaw_diff < yaw_match_threshold_) {
                    current_state_ = AimingState::TRACKING;
                    yaw_filter_window_.clear();
                    pitch_filter_window_.clear();
                    RCLCPP_INFO(this->get_logger(), "切换完成，返回TRACKING状态");
                } else if (progress >= 1.0) {
                    // 已经到达目标位置，等待aim_position接近
                    output_yaw = switch_target_yaw_;
                    output_pitch = switch_target_pitch_;
                }
                
                fire_allowed = false;  // 切换过程中不开火
                break;
            }
        }
        
        // 发布指令
        publishCommand(msg->header, output_yaw, output_pitch, delta_yaw, gravity_compensation, dist_aim, fire_allowed);
    }
    
    void publishCommand(const std_msgs::msg::Header& header, 
                        double yaw, double pitch, 
                        double delta_yaw, double gravity_compensation, double distance,
                        bool fire_allowed)
    {
        auto cmd_msg = armor_detector_ros2::msg::GimbalCommand();
        cmd_msg.header = header;
        cmd_msg.yaw = yaw;
        cmd_msg.pitch = pitch;
        cmd_msg.aiming_state = static_cast<int32_t>(current_state_);
        cmd_msg.fire_allowed = fire_allowed;
        cmd_msg.delta_yaw = delta_yaw;
        cmd_msg.delta_pitch = pitch - cmd_msg.pitch;  // 近似
        cmd_msg.distance = distance;
        cmd_msg.gravity_compensation = gravity_compensation;
        
        gimbal_cmd_pub_->publish(cmd_msg);
    }
    
    // 成员变量
    rclcpp::Subscription<armor_detector_ros2::msg::OutpostAimResult>::SharedPtr aim_result_sub_;
    rclcpp::Publisher<armor_detector_ros2::msg::GimbalCommand>::SharedPtr gimbal_cmd_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pipeline_control_srv_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // 参数
    int filter_window_size_{5};
    double angle_jump_threshold_{60.0};
    double yaw_match_threshold_{3.0};
    double moving_delay_{0.2};
    double gravity_g_{9.8};
    double bullet_speed_{28.0};
    Eigen::Vector3d camera_offset_{0.0, 0.0, 0.0};
    
    // 状态
    bool pipeline_enabled_{true};
    AimingState current_state_{AimingState::INITIALIZING};
    
    // 滤波窗口
    std::deque<double> yaw_filter_window_;
    std::deque<double> pitch_filter_window_;
    
    // 切换检测
    double last_further_ang_{0.0};
    bool last_further_ang_valid_{false};
    
    // 切换状态参数
    double switch_start_time_{0.0};
    double switch_target_yaw_{0.0};
    double switch_target_pitch_{0.0};
    double switch_start_yaw_{0.0};
    double switch_start_pitch_{0.0};
};

}  // namespace armor_detector

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<armor_detector::AimingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
