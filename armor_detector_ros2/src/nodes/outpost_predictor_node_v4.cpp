/**
 * @file outpost_predictor_node_v4.cpp
 * @brief 前哨站预测器 V4 ROS2 节点
 * 
 * 功能:
 *   - 订阅装甲板位姿信息 (/solver/armor_poses)
 *   - 使用 V4 预测器进行状态估计和预瞄计算
 *   - 发布预瞄结果 (/predictor_v4/aim_result)
 *   - 支持参数动态更新
 * 
 * 坐标系说明:
 *   - 输入/输出均使用机架坐标系
 *   - X右, Y前, Z上
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "core/outpost_predictor_v4.hpp"
#include "armor_detector_ros2/msg/armor_pose_array.hpp"
#include "armor_detector_ros2/msg/outpost_aim_result.hpp"

namespace armor_detector
{

class OutpostPredictorNodeV4 : public rclcpp::Node
{
public:
    OutpostPredictorNodeV4() : Node("outpost_predictor_v4")
    {
        RCLCPP_INFO(this->get_logger(), "初始化前哨站预测器V4节点...");
        
        // 声明参数
        declareParameters();
        
        // 创建预测器并加载配置
        predictor_ = std::make_unique<OutpostPredictorV4>();
        loadConfig();
        
        // 订阅装甲板位姿
        armor_sub_ = this->create_subscription<armor_detector_ros2::msg::ArmorPoseArray>(
            "/solver/armor_poses", 10,
            std::bind(&OutpostPredictorNodeV4::armorCallback, this, std::placeholders::_1));
        
        // 发布预瞄结果
        aim_result_pub_ = this->create_publisher<armor_detector_ros2::msg::OutpostAimResult>(
            "/predictor_v4/aim_result", 10);
        
        // 发布瞄准点（用于可视化）
        aim_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/predictor_v4/aim_point", 10);
        
        // 发布状态向量 [ang, omega, x_c, y_c, z_c]
        state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/predictor_v4/state", 10);
        
        // 发布文本状态 (用于可视化)
        state_text_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/predictor_v4/state_text", 10);
        
        // 发布初始化阶段
        init_phase_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/predictor_v4/init_phase", 10);
        
        // 发布开火条件
        fire_condition_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/predictor_v4/fire_condition", 10);
        
        // 发布可视化标记
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/predictor_v4/markers", 10);
        
        // 定时预测（无观测时保持预测）
        predict_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 100Hz
            std::bind(&OutpostPredictorNodeV4::predictTimerCallback, this));
        
        // 参数变化回调
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&OutpostPredictorNodeV4::parametersCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "前哨站预测器V4节点初始化完成");
        RCLCPP_INFO(this->get_logger(), "======================================");
        RCLCPP_INFO(this->get_logger(), "  坐标系: 机架坐标系 (X右/Y前/Z上)");
        RCLCPP_INFO(this->get_logger(), "  订阅: /solver/armor_poses");
        RCLCPP_INFO(this->get_logger(), "  发布: /predictor_v4/aim_result");
        RCLCPP_INFO(this->get_logger(), "======================================");
    }

private:
    void declareParameters()
    {
        // 几何参数
        this->declare_parameter("geometry.radius", 0.2767);
        this->declare_parameter("geometry.height_diff", 0.10);
        this->declare_parameter("geometry.tilt_angle_deg", 75.0);
        this->declare_parameter("geometry.max_aspect_ratio", 2.58);
        // 装甲板排列：false=高-中-低(默认)，true=低-中-高(反向)
        this->declare_parameter("geometry.armor_arrangement_reversed", false);
        
        // 运动参数
        this->declare_parameter("motion.standard_omega", 2.5133);
        
        // 初始化参数
        this->declare_parameter("init.direction_frames", 10);
        this->declare_parameter("init.cx_change_threshold", 30.0);
        this->declare_parameter("init.z_jump_threshold", 0.12);
        
        // 相位观测参数
        this->declare_parameter("phase.plateau_threshold", 2.2);
        this->declare_parameter("phase.plateau_exit_ratio", 0.80);
        this->declare_parameter("phase.plateau_confirm_ratio", 2.4);
        
        // EKF协方差初始化
        this->declare_parameter("ekf.sigma_theta_sq", 0.1);
        this->declare_parameter("ekf.sigma_omega_sq", 1.0);
        this->declare_parameter("ekf.sigma_x_sq", 50.0);
        this->declare_parameter("ekf.sigma_y_sq", 50.0);
        this->declare_parameter("ekf.sigma_z_sq", 50.0);
        
        // 过程噪声
        this->declare_parameter("ekf.q_theta", 1e-4);
        this->declare_parameter("ekf.q_omega", 1e-2);
        this->declare_parameter("ekf.q_x", 1.0);
        this->declare_parameter("ekf.q_y", 1.0);
        this->declare_parameter("ekf.q_z", 0.1);
        
        // 观测噪声
        this->declare_parameter("ekf.r_theta", 0.01);
        this->declare_parameter("ekf.r_x", 0.01);
        this->declare_parameter("ekf.r_y", 0.01);
        this->declare_parameter("ekf.r_z", 0.01);
        
        // 预瞄参数
        this->declare_parameter("aim.t_delay", 0.05);
        this->declare_parameter("aim.filter_delay", 0.1);
        this->declare_parameter("aim.moving_delay", 0.2);
        this->declare_parameter("aim.v_bullet", 28.0);
        this->declare_parameter("aim.angle_threshold_deg", 55.0);
        
        // 收敛参数
        this->declare_parameter("converge.ekf_frames", 50);
        this->declare_parameter("converge.height_verify_frames", 10.0);
        this->declare_parameter("converge.height_recovery_frames", 20);
        this->declare_parameter("converge.cov_omega_threshold", 0.1);
        this->declare_parameter("converge.cov_position_threshold", 0.05);
    }
    
    void loadConfig()
    {
        OutpostConfigV4 config;
        
        // 几何参数
        config.radius = this->get_parameter("geometry.radius").as_double();
        config.height_diff = this->get_parameter("geometry.height_diff").as_double();
        config.tilt_angle_deg = this->get_parameter("geometry.tilt_angle_deg").as_double();
        config.max_aspect_ratio = this->get_parameter("geometry.max_aspect_ratio").as_double();
        config.armor_arrangement_reversed = this->get_parameter("geometry.armor_arrangement_reversed").as_bool();
        
        // 运动参数
        config.standard_omega = this->get_parameter("motion.standard_omega").as_double();
        
        // 初始化参数
        config.direction_init_frames = this->get_parameter("init.direction_frames").as_int();
        config.cx_change_threshold = this->get_parameter("init.cx_change_threshold").as_double();
        config.z_jump_threshold = this->get_parameter("init.z_jump_threshold").as_double();
        
        // 相位观测参数
        config.plateau_threshold = this->get_parameter("phase.plateau_threshold").as_double();
        config.plateau_exit_ratio = this->get_parameter("phase.plateau_exit_ratio").as_double();
        config.plateau_confirm_ratio = this->get_parameter("phase.plateau_confirm_ratio").as_double();
        
        // EKF协方差初始化
        config.sigma_theta_sq = this->get_parameter("ekf.sigma_theta_sq").as_double();
        config.sigma_omega_sq = this->get_parameter("ekf.sigma_omega_sq").as_double();
        config.sigma_x_sq = this->get_parameter("ekf.sigma_x_sq").as_double();
        config.sigma_y_sq = this->get_parameter("ekf.sigma_y_sq").as_double();
        config.sigma_z_sq = this->get_parameter("ekf.sigma_z_sq").as_double();
        
        // 过程噪声
        config.q_theta = this->get_parameter("ekf.q_theta").as_double();
        config.q_omega = this->get_parameter("ekf.q_omega").as_double();
        config.q_x = this->get_parameter("ekf.q_x").as_double();
        config.q_y = this->get_parameter("ekf.q_y").as_double();
        config.q_z = this->get_parameter("ekf.q_z").as_double();
        
        // 观测噪声
        config.r_theta = this->get_parameter("ekf.r_theta").as_double();
        config.r_x = this->get_parameter("ekf.r_x").as_double();
        config.r_y = this->get_parameter("ekf.r_y").as_double();
        config.r_z = this->get_parameter("ekf.r_z").as_double();
        
        // 预瞄参数
        config.t_delay = this->get_parameter("aim.t_delay").as_double();
        config.filter_delay = this->get_parameter("aim.filter_delay").as_double();
        config.moving_delay = this->get_parameter("aim.moving_delay").as_double();
        config.v_bullet = this->get_parameter("aim.v_bullet").as_double();
        config.aim_angle_threshold_deg = this->get_parameter("aim.angle_threshold_deg").as_double();
        
        // 收敛参数
        config.ekf_converge_frames = this->get_parameter("converge.ekf_frames").as_int();
        config.height_verify_frames = this->get_parameter("converge.height_verify_frames").as_double();
        config.height_recovery_frames = this->get_parameter("converge.height_recovery_frames").as_int();
        config.cov_omega_threshold = this->get_parameter("converge.cov_omega_threshold").as_double();
        config.cov_position_threshold = this->get_parameter("converge.cov_position_threshold").as_double();
        
        predictor_->setConfig(config);
        
        RCLCPP_INFO(this->get_logger(), 
            "V4配置已加载: radius=%.4f, height_diff=%.3f, omega=%.3f, reversed=%s",
            config.radius, config.height_diff, config.standard_omega,
            config.armor_arrangement_reversed ? "true(低-中-高)" : "false(高-中-低)");
    }
    
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        for (const auto& param : params) {
            RCLCPP_INFO(this->get_logger(), "参数更新: %s", param.get_name().c_str());
        }
        
        loadConfig();
        return result;
    }
    
    void armorCallback(const armor_detector_ros2::msg::ArmorPoseArray::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            lost_count_++;
            
            // 即使没有检测到装甲板，也要更新预测器（使用无效观测）
            // 这样EKF可以继续预测
            ObservationV4 obs;
            obs.valid = false;
            obs.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            predictor_->update(obs);
            
            // 如果EKF已初始化，仍然发布预测结果
            if (predictor_->isEKFInitialized()) {
                publishResults(msg->header);
            }
            return;
        }
        
        lost_count_ = 0;
        
        // 获取第一个装甲板（前哨站通常只检测到一个）
        const auto& armor = msg->poses[0];
        
        // 构建观测
        ObservationV4 obs;
        obs.position = Eigen::Vector3d(
            armor.position.x,
            armor.position.y,
            armor.position.z
        );
        obs.aspect_ratio = armor.aspect_ratio;
        obs.center_pixel_x = armor.center_pixel_x;
        
        // 提取转换矩阵
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                obs.t_gimbal_camera(i, j) = armor.t_gimbal_camera[i * 3 + j];
                obs.r_body_gimbal(i, j) = armor.r_body_gimbal[i * 3 + j];
            }
        }
        
        // 时间戳
        obs.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        obs.valid = armor.valid;
        
        // IMU数据（从ArmorPoseArray消息中获取）
        obs.imu_yaw = msg->imu_yaw;
        obs.imu_pitch = msg->imu_pitch;
        
        // 更新预测器
        predictor_->update(obs);
        
        // 发布结果
        publishResults(msg->header);
    }
    
    void predictTimerCallback()
    {
        if (!predictor_->isEKFInitialized()) {
            return;
        }
        
        // 周期性发布（即使无新观测）
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "body_frame";
        publishResults(header);
    }
    
    void publishResults(const std_msgs::msg::Header& header)
    {
        // 计算预瞄结果
        AimResultV4 aim = predictor_->computeAim();
        
        // 发布 OutpostAimResult 消息
        auto aim_result_msg = armor_detector_ros2::msg::OutpostAimResult();
        aim_result_msg.header = header;
        
        // 预瞄位置
        aim_result_msg.aim_position.x = aim.aim_position.x();
        aim_result_msg.aim_position.y = aim.aim_position.y();
        aim_result_msg.aim_position.z = aim.aim_position.z();
        aim_result_msg.aim_height_state = static_cast<int32_t>(aim.aim_height_state);
        aim_result_msg.aim_ang_local = aim.aim_ang_local;
        
        // 更远预瞄位置（用于切换预判）
        aim_result_msg.further_aim_position.x = aim.further_aim_position.x();
        aim_result_msg.further_aim_position.y = aim.further_aim_position.y();
        aim_result_msg.further_aim_position.z = aim.further_aim_position.z();
        aim_result_msg.further_aim_ang = aim.further_aim_ang;
        aim_result_msg.further_aim_height_state = static_cast<int32_t>(aim.further_aim_height_state);
        
        // IMU数据（转发）
        aim_result_msg.imu_yaw = aim.imu_yaw;
        aim_result_msg.imu_pitch = aim.imu_pitch;
        
        // 开火条件
        aim_result_msg.fire_condition_1 = aim.fire_condition_1;
        
        // 坐标转换矩阵
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                aim_result_msg.t_gimbal_camera[i * 3 + j] = aim.t_gimbal_camera(i, j);
                aim_result_msg.r_body_gimbal[i * 3 + j] = aim.r_body_gimbal(i, j);
            }
        }
        
        // EKF状态
        aim_result_msg.ekf_initialized = predictor_->isEKFInitialized();
        aim_result_msg.ekf_converged = predictor_->isEKFConverged();
        
        auto ekf_state = predictor_->getEKFState();
        aim_result_msg.ang = ekf_state.ang;
        aim_result_msg.omega = ekf_state.omega;
        aim_result_msg.center_position.x = ekf_state.x_c;
        aim_result_msg.center_position.y = ekf_state.y_c;
        aim_result_msg.center_position.z = ekf_state.z_c;
        // 观测高度状态（高度状态机的当前状态）
        aim_result_msg.current_height_state = static_cast<int32_t>(predictor_->getCurrentHeightState());
        // EKF内部的高度状态 height_k
        aim_result_msg.ekf_height_state = static_cast<int32_t>(ekf_state.height_k);
        
        // 初始化状态
        aim_result_msg.rotation_direction = static_cast<int32_t>(predictor_->getRotationDirection());
        // 高度初始化完成的条件：已进入 HEIGHT_INIT 之后的阶段
        aim_result_msg.height_initialized = predictor_->isHeightInitialized();
        aim_result_msg.theta_initialized = (predictor_->getInitPhase() >= InitPhase::EKF_RUNNING);
        aim_result_msg.phase_state = static_cast<int32_t>(predictor_->getPhaseState());
        
        // 弹道信息
        aim_result_msg.bullet_flight_time = aim.bullet_flight_time;
        aim_result_msg.distance = aim.distance;
        
        // 调试信息
        aim_result_msg.observed_theta = predictor_->getCurrentTheta();
        aim_result_msg.aspect_ratio = 0.0;  // 从观测历史获取
        auto residual = predictor_->getLastResidual();
        for (int i = 0; i < 4; i++) {
            aim_result_msg.measurement_residual[i] = residual(i);
        }
        aim_result_msg.valid = aim.valid;
        
        aim_result_pub_->publish(aim_result_msg);
        
        // 发布瞄准点（用于可视化）
        if (aim.valid) {
            auto aim_point_msg = geometry_msgs::msg::PointStamped();
            aim_point_msg.header = header;
            aim_point_msg.point.x = aim.aim_position.x();
            aim_point_msg.point.y = aim.aim_position.y();
            aim_point_msg.point.z = aim.aim_position.z();
            aim_point_pub_->publish(aim_point_msg);
        }
        
        // 发布状态向量
        auto state = predictor_->getStateVector();
        auto state_msg = std_msgs::msg::Float64MultiArray();
        state_msg.data.resize(state.size());
        for (int i = 0; i < state.size(); i++) {
            state_msg.data[i] = state(i);
        }
        state_pub_->publish(state_msg);
        
        // 发布文本状态
        auto state_text_msg = std_msgs::msg::String();
        switch (predictor_->getInitPhase()) {
            case InitPhase::NOT_STARTED:
                state_text_msg.data = "NOT_STARTED";
                break;
            case InitPhase::DIRECTION_INIT:
                state_text_msg.data = "DIRECTION_INIT";
                break;
            case InitPhase::HEIGHT_INIT:
                state_text_msg.data = "HEIGHT_INIT";
                break;
            case InitPhase::THETA_INIT:
                state_text_msg.data = "THETA_INIT";
                break;
            case InitPhase::EKF_RUNNING:
                state_text_msg.data = predictor_->isEKFConverged() ? "CONVERGED" : "EKF_RUNNING";
                break;
        }
        state_text_pub_->publish(state_text_msg);
        
        // 发布初始化阶段
        auto init_phase_msg = std_msgs::msg::Int32();
        init_phase_msg.data = static_cast<int32_t>(predictor_->getInitPhase());
        init_phase_pub_->publish(init_phase_msg);
        
        // 发布开火条件
        auto fire_msg = std_msgs::msg::Bool();
        fire_msg.data = aim.fire_condition_1 && predictor_->isEKFConverged();
        fire_condition_pub_->publish(fire_msg);
        
        // 发布可视化标记
        publishMarkers(header, aim);
    }
    
    void publishMarkers(const std_msgs::msg::Header& header, const AimResultV4& aim)
    {
        visualization_msgs::msg::MarkerArray markers;
        
        if (!predictor_->isEKFInitialized()) {
            marker_pub_->publish(markers);
            return;
        }
        
        auto ekf_state = predictor_->getEKFState();
        auto config = predictor_->getConfig();
        
        // 1. 旋转中心标记
        {
            visualization_msgs::msg::Marker center_marker;
            center_marker.header = header;
            center_marker.ns = "outpost_center";
            center_marker.id = 0;
            center_marker.type = visualization_msgs::msg::Marker::SPHERE;
            center_marker.action = visualization_msgs::msg::Marker::ADD;
            center_marker.pose.position.x = ekf_state.x_c;
            center_marker.pose.position.y = ekf_state.y_c;
            center_marker.pose.position.z = ekf_state.z_c;
            center_marker.scale.x = 0.05;
            center_marker.scale.y = 0.05;
            center_marker.scale.z = 0.05;
            center_marker.color.r = 1.0;
            center_marker.color.g = 1.0;
            center_marker.color.b = 0.0;
            center_marker.color.a = 1.0;
            center_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
            markers.markers.push_back(center_marker);
        }
        
        // 2. 三个装甲板位置标记
        double ang = ekf_state.ang;
        for (int i = 0; i < 3; i++) {
            double armor_ang = ang + i * 120.0;
            double ang_local = std::fmod(armor_ang, 120.0);
            if (ang_local < 0) ang_local += 120.0;
            double ang_rad = (ang_local - 60.0) * M_PI / 180.0;
            
            double x = ekf_state.x_c - config.radius * std::sin(ang_rad);
            double y = ekf_state.y_c + config.radius * std::cos(ang_rad);
            
            double z_offset = 0.0;
            HeightState height = static_cast<HeightState>((static_cast<int>(ekf_state.height_k) + i) % 3);
            if (height == HeightState::LOW) z_offset = -config.height_diff;
            else if (height == HeightState::HIGH) z_offset = config.height_diff;
            double z = ekf_state.z_c + z_offset;
            
            visualization_msgs::msg::Marker armor_marker;
            armor_marker.header = header;
            armor_marker.ns = "outpost_armors";
            armor_marker.id = i;
            armor_marker.type = visualization_msgs::msg::Marker::CUBE;
            armor_marker.action = visualization_msgs::msg::Marker::ADD;
            armor_marker.pose.position.x = x;
            armor_marker.pose.position.y = y;
            armor_marker.pose.position.z = z;
            armor_marker.scale.x = 0.135;
            armor_marker.scale.y = 0.055;
            armor_marker.scale.z = 0.01;
            
            // 颜色区分
            if (i == 0) {
                armor_marker.color.r = 0.0;
                armor_marker.color.g = 1.0;
                armor_marker.color.b = 0.0;
            } else {
                armor_marker.color.r = 1.0;
                armor_marker.color.g = 0.0;
                armor_marker.color.b = 0.0;
            }
            armor_marker.color.a = 0.8;
            armor_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
            
            markers.markers.push_back(armor_marker);
        }
        
        // 3. 瞄准点标记
        if (aim.valid) {
            visualization_msgs::msg::Marker aim_marker;
            aim_marker.header = header;
            aim_marker.ns = "aim_point";
            aim_marker.id = 0;
            aim_marker.type = visualization_msgs::msg::Marker::SPHERE;
            aim_marker.action = visualization_msgs::msg::Marker::ADD;
            aim_marker.pose.position.x = aim.aim_position.x();
            aim_marker.pose.position.y = aim.aim_position.y();
            aim_marker.pose.position.z = aim.aim_position.z();
            aim_marker.scale.x = 0.03;
            aim_marker.scale.y = 0.03;
            aim_marker.scale.z = 0.03;
            
            if (aim.fire_condition_1) {
                aim_marker.color.r = 0.0;
                aim_marker.color.g = 1.0;
                aim_marker.color.b = 0.0;
            } else {
                aim_marker.color.r = 1.0;
                aim_marker.color.g = 0.5;
                aim_marker.color.b = 0.0;
            }
            aim_marker.color.a = 1.0;
            aim_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
            
            markers.markers.push_back(aim_marker);
        }
        
        // 4. 瞄准线
        if (aim.valid) {
            visualization_msgs::msg::Marker line_marker;
            line_marker.header = header;
            line_marker.ns = "aim_line";
            line_marker.id = 0;
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            
            geometry_msgs::msg::Point p1, p2;
            p1.x = 0; p1.y = 0; p1.z = 0;
            p2.x = aim.aim_position.x();
            p2.y = aim.aim_position.y();
            p2.z = aim.aim_position.z();
            line_marker.points.push_back(p1);
            line_marker.points.push_back(p2);
            
            line_marker.scale.x = 0.005;
            line_marker.color.r = 0.0;
            line_marker.color.g = 1.0;
            line_marker.color.b = 1.0;
            line_marker.color.a = 0.5;
            line_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
            
            markers.markers.push_back(line_marker);
        }
        
        marker_pub_->publish(markers);
    }
    
    // 成员变量
    std::unique_ptr<OutpostPredictorV4> predictor_;
    
    // 订阅者
    rclcpp::Subscription<armor_detector_ros2::msg::ArmorPoseArray>::SharedPtr armor_sub_;
    
    // 发布者
    rclcpp::Publisher<armor_detector_ros2::msg::OutpostAimResult>::SharedPtr aim_result_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr aim_point_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_text_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr init_phase_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fire_condition_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr predict_timer_;
    
    // 参数回调
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // 丢失计数
    int lost_count_{0};
};

}  // namespace armor_detector

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<armor_detector::OutpostPredictorNodeV4>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
