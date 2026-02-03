/**
 * @file outpost_predictor_node_v2.cpp
 * @brief 前哨站预测器 V2 ROS2 节点
 * 
 * 订阅装甲板位姿，发布预测瞄准结果
 * V2版本支持多假设追踪，发布详细的收敛状态
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "core/outpost_predictor_v2.hpp"
#include "armor_detector_ros2/msg/outpost_state_v2.hpp"
#include "armor_detector_ros2/msg/armor_pose_array.hpp"

namespace armor_detector
{

class OutpostPredictorNodeV2 : public rclcpp::Node
{
public:
  OutpostPredictorNodeV2() : Node("outpost_predictor_v2")
  {
    RCLCPP_INFO(this->get_logger(), "初始化前哨站预测器V2节点...");
    
    // 声明参数
    declareParameters();
    
    // 创建预测器
    predictor_ = std::make_unique<OutpostPredictorV2>();
    loadConfig();
    
    // 订阅者
    armor_sub_ = this->create_subscription<armor_detector_ros2::msg::ArmorPoseArray>(
      "/solver/armor_poses", 10,
      std::bind(&OutpostPredictorNodeV2::armorCallback, this, std::placeholders::_1));
    
    // 发布者
    aim_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/predictor_v2/aim_point", 10);
    
    aim_angles_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/predictor_v2/aim_angles", 10);
    
    should_shoot_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/predictor_v2/should_shoot", 10);
    
    state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/predictor_v2/state", 10);
    
    // V2状态发布者 (用于可视化)
    state_v2_pub_ = this->create_publisher<armor_detector_ros2::msg::OutpostStateV2>(
      "/outpost_state_v2", 10);
    
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/predictor_v2/markers", 10);
    
    // 定时器用于预测 (即使没有观测也要更新预测)
    predict_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),  // 100Hz
      std::bind(&OutpostPredictorNodeV2::predictTimerCallback, this));
    
    // 参数变化回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&OutpostPredictorNodeV2::parametersCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "前哨站预测器V2节点初始化完成");
  }

private:
  void declareParameters()
  {
    // 几何参数
    this->declare_parameter("geometry.radius", 0.275);
    this->declare_parameter("geometry.height_diff", 0.0);  // 如果三个装甲板等高则为0
    this->declare_parameter("geometry.tilt_angle", 1.48);  // ~85度
    
    // 运动参数
    this->declare_parameter("known_omega", 0.8 * M_PI);  // 标准前哨站0.4转/秒
    this->declare_parameter("use_fixed_omega", true);
    this->declare_parameter("omega_threshold", 0.1);
    
    // 噪声参数
    this->declare_parameter("noise.process_pos", 0.001);
    this->declare_parameter("noise.process_theta", 0.01);
    this->declare_parameter("noise.process_omega", 0.001);
    this->declare_parameter("noise.measurement_pos", 0.02);
    this->declare_parameter("noise.measurement_angle", 0.05);
    
    // 子弹参数
    this->declare_parameter("bullet_speed", 28.0);
    this->declare_parameter("system_delay", 0.05);  // 50ms系统延迟
    
    // 瞄准参数
    this->declare_parameter("max_orientation_angle", 0.65);  // ~37度
    this->declare_parameter("chi_square_threshold", 15.0);
    this->declare_parameter("direction_detection_samples", 10);
    
    // 重置参数
    this->declare_parameter("max_lost_count", 30);  // 增加到30帧（约1秒@30fps）
    this->declare_parameter("reset_on_lost", false);  // 默认不重置，继续预测
  }
  
  void loadConfig()
  {
    OutpostPredictorConfig config;
    
    // 几何参数
    config.geometry.radius = this->get_parameter("geometry.radius").as_double();
    config.geometry.height_diff = this->get_parameter("geometry.height_diff").as_double();
    config.geometry.tilt_angle = this->get_parameter("geometry.tilt_angle").as_double();
    
    // 运动参数
    config.known_omega = this->get_parameter("known_omega").as_double();
    config.use_fixed_omega = this->get_parameter("use_fixed_omega").as_bool();
    config.omega_threshold = this->get_parameter("omega_threshold").as_double();
    
    // 噪声参数
    config.noise.process_pos = this->get_parameter("noise.process_pos").as_double();
    config.noise.process_theta = this->get_parameter("noise.process_theta").as_double();
    config.noise.process_omega = this->get_parameter("noise.process_omega").as_double();
    config.noise.measurement_pos = this->get_parameter("noise.measurement_pos").as_double();
    config.noise.measurement_angle = this->get_parameter("noise.measurement_angle").as_double();
    
    // 子弹参数
    config.bullet_speed = this->get_parameter("bullet_speed").as_double();
    config.system_delay = this->get_parameter("system_delay").as_double();
    
    // 瞄准参数
    config.max_orientation_angle = this->get_parameter("max_orientation_angle").as_double();
    config.chi_square_threshold = this->get_parameter("chi_square_threshold").as_double();
    config.direction_detection_samples = 
      this->get_parameter("direction_detection_samples").as_int();
    
    max_lost_count_ = this->get_parameter("max_lost_count").as_int();
    reset_on_lost_ = this->get_parameter("reset_on_lost").as_bool();
    
    predictor_->setConfig(config);
    
    RCLCPP_INFO(this->get_logger(), 
      "配置已加载: radius=%.3f, omega=%.3f, bullet_speed=%.1f",
      config.geometry.radius, config.known_omega, config.bullet_speed);
  }
  
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter>& params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto& param : params) {
      RCLCPP_INFO(this->get_logger(), "参数更新: %s", param.get_name().c_str());
    }
    
    // 重新加载配置
    loadConfig();
    
    return result;
  }
  
  void armorCallback(const armor_detector_ros2::msg::ArmorPoseArray::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      return;
    }
    
    lost_count_ = 0;
    
    // 取第一个装甲板（前哨站通常只有一个观测）
    const auto& armor_pose = msg->poses[0];
    
    // 从ROS消息构建观测
    ArmorObservation obs;
    obs.position = Eigen::Vector3d(
      armor_pose.position.x,
      armor_pose.position.y,
      armor_pose.position.z);
    
    // 直接使用 armor_yaw 作为朝向
    obs.orientation = armor_pose.armor_yaw;
    
    // 时间戳
    obs.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    
    // 装甲板ID未知，让多假设跟踪自己判断
    obs.armor_id = -1;
    
    obs.valid = true;
    
    // 更新预测器
    bool accepted = predictor_->update(obs);
    
    if (!accepted) {
      RCLCPP_DEBUG(this->get_logger(), "观测被拒绝 (卡方检验未通过)");
      rejected_count_++;
      
      // 如果连续拒绝次数过多，可能跟丢了，需要重置
      if (rejected_count_ > 20) {
        RCLCPP_WARN(this->get_logger(), "连续%d次观测被拒绝，重置预测器", rejected_count_);
        predictor_->reset();
        rejected_count_ = 0;
        return;
      }
    } else {
      rejected_count_ = 0;  // 重置拒绝计数器
      lost_count_ = 0;  // 重置丢失计数器
      static int update_count = 0;
      if (++update_count % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "已处理 %d 次观测", update_count);
      }
    }
    
    // 发布结果
    publishResults(msg->header.stamp);
    
    // 发布V2状态用于可视化
    publishStateV2(msg->header.stamp);
  }
  
  void predictTimerCallback()
  {
    lost_count_++;
    
    // 智能重置策略：
    // 1. 短时间丢失(装甲板切换)：不重置，继续预测
    // 2. 已收敛且长时间丢失：不重置，继续预测（可能是遮挡）
    // 3. 未收敛且长时间丢失：重置（说明初始化错误）
    // 4. 连续卡方检验失败：在armorCallback中已处理
    
    if (lost_count_ > max_lost_count_) {
      bool is_converged = predictor_->isConverged();
      
      // 只有在启用reset_on_lost或未收敛时才考虑重置
      if (predictor_->isInitialized()) {
        if (!is_converged) {
          // 未收敛状态下长时间丢失，应该重置
          RCLCPP_WARN(this->get_logger(), 
                     "未收敛状态下目标丢失超过%d帧，重置预测器", max_lost_count_);
          predictor_->reset();
          rejected_count_ = 0;
          lost_count_ = 0;
          return;
        } else if (reset_on_lost_) {
          // 已收敛但用户强制要求重置
          RCLCPP_WARN(this->get_logger(), 
                     "目标丢失超过%d帧，强制重置预测器", max_lost_count_);
          predictor_->reset();
          rejected_count_ = 0;
          lost_count_ = 0;
          return;
        } else {
          // 已收敛且不强制重置，继续预测
          RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "已收敛状态，丢失%d帧，继续预测", lost_count_);
        }
      }
    }
    
    if (!predictor_->isInitialized()) {
      return;
    }
    
    // 执行预测步骤
    predictor_->predict();
    
    // 发布V2状态用于可视化
    publishStateV2(this->now());
    
    // 发布可视化
    publishMarkers(this->now());
  }
  
  void publishResults(const rclcpp::Time& stamp)
  {
    if (!predictor_->isInitialized()) {
      return;
    }
    
    // 计算瞄准结果
    AimResult aim = predictor_->computeAim();
    
    // 发布瞄准点
    geometry_msgs::msg::PointStamped aim_point_msg;
    aim_point_msg.header.stamp = stamp;
    aim_point_msg.header.frame_id = "camera_link";
    aim_point_msg.point.x = aim.aim_point.x();
    aim_point_msg.point.y = aim.aim_point.y();
    aim_point_msg.point.z = aim.aim_point.z();
    aim_point_pub_->publish(aim_point_msg);
    
    // 发布瞄准角度 [yaw, pitch, distance, confidence]
    std_msgs::msg::Float64MultiArray angles_msg;
    angles_msg.data = {
      aim.aim_yaw, aim.aim_pitch, aim.distance, aim.confidence};
    aim_angles_pub_->publish(angles_msg);
    
    // 发布是否应该射击
    std_msgs::msg::Bool should_shoot_msg;
    should_shoot_msg.data = aim.should_shoot;
    should_shoot_pub_->publish(should_shoot_msg);
    
    // 发布状态向量 [x_c, y_c, z_c, theta, omega]
    auto state = predictor_->getStateVector();
    std_msgs::msg::Float64MultiArray state_msg;
    state_msg.data.resize(state.size());
    for (int i = 0; i < state.size(); ++i) {
      state_msg.data[i] = state(i);
    }
    state_pub_->publish(state_msg);
    
    // 打印调试信息
    RCLCPP_DEBUG(this->get_logger(),
      "Aim: yaw=%.2f° pitch=%.2f° dist=%.2f shoot=%s conf=%.2f",
      aim.aim_yaw * 180.0 / M_PI,
      aim.aim_pitch * 180.0 / M_PI,
      aim.distance,
      aim.should_shoot ? "YES" : "NO",
      aim.confidence);
  }
  
  /**
   * @brief 发布V2状态消息用于可视化
   */
  void publishStateV2(const rclcpp::Time& stamp)
  {
    armor_detector_ros2::msg::OutpostStateV2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "camera_link";
    
    // 检查是否初始化
    if (!predictor_->isInitialized()) {
      msg.valid = false;
      state_v2_pub_->publish(msg);
      static int log_count = 0;
      if (log_count++ % 100 == 0) {
        RCLCPP_WARN(this->get_logger(), "预测器未初始化，等待观测数据...");
      }
      return;
    }
    
    msg.valid = true;
    
    static int pub_count = 0;
    if (pub_count++ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), "已发布 %d 条V2状态消息", pub_count);
    }
    
    // 获取多假设状态
    auto hyp_state = predictor_->getMultiHypothesisState();
    
    msg.converged = hyp_state.converged;
    msg.best_hypothesis_id = hyp_state.best_hypothesis_id;
    
    for (int i = 0; i < 3; i++) {
      msg.hypothesis_confidences[i] = hyp_state.confidences[i];
      msg.hypothesis_update_counts[i] = hyp_state.update_counts[i];
    }
    
    // 获取当前状态
    auto state = predictor_->getState();
    msg.center.x = state.center.x();
    msg.center.y = state.center.y();
    msg.center.z = state.center.z();
    msg.theta = state.theta;
    msg.omega = state.omega;
    msg.direction = state.direction;
    msg.radius = predictor_->getConfig().geometry.radius;
    msg.height_diff = predictor_->getConfig().geometry.height_diff;
    
    // 获取当前三个装甲板位置
    auto predictions = predictor_->predictArmors(0.0);
    for (int i = 0; i < 3; i++) {
      msg.armor_positions[i].x = predictions[i].position.x();
      msg.armor_positions[i].y = predictions[i].position.y();
      msg.armor_positions[i].z = predictions[i].position.z();
      msg.armor_shootable[i] = predictions[i].shootable;
      msg.armor_orientations[i] = predictions[i].orientation;
    }
    
    // 计算弹道预测
    auto aim = predictor_->computeAim();
    msg.predicted_hit_point.x = aim.aim_point.x();
    msg.predicted_hit_point.y = aim.aim_point.y();
    msg.predicted_hit_point.z = aim.aim_point.z();
    msg.bullet_flight_time = aim.bullet_flight_time;
    msg.predicted_target_id = aim.target_armor_id;
    
    // 预测击中时刻的装甲板位置
    if (aim.bullet_flight_time > 0) {
      double total_delay = predictor_->getConfig().system_delay + aim.bullet_flight_time;
      auto future_predictions = predictor_->predictArmors(total_delay);
      for (int i = 0; i < 3; i++) {
        msg.predicted_armor_positions[i].x = future_predictions[i].position.x();
        msg.predicted_armor_positions[i].y = future_predictions[i].position.y();
        msg.predicted_armor_positions[i].z = future_predictions[i].position.z();
      }
    }
    
    // 计算下一个可打窗口时间
    msg.next_window_time = aim.should_shoot ? -1.0 : predictor_->computeNextWindowTime();
    
    // 上次观测时间差
    msg.time_since_last_observation = predictor_->getTimeSinceLastObservation();
    
    state_v2_pub_->publish(msg);
  }
  
  void publishMarkers(const rclcpp::Time& stamp)
  {
    if (!predictor_->isInitialized()) {
      return;
    }
    
    visualization_msgs::msg::MarkerArray markers;
    
    // 获取状态
    auto state = predictor_->getState();
    auto predictions = predictor_->predictArmors(0.0);  // 当前时刻
    
    // 中心点标记
    visualization_msgs::msg::Marker center_marker;
    center_marker.header.stamp = stamp;
    center_marker.header.frame_id = "camera_link";
    center_marker.ns = "outpost_center";
    center_marker.id = 0;
    center_marker.type = visualization_msgs::msg::Marker::SPHERE;
    center_marker.action = visualization_msgs::msg::Marker::ADD;
    center_marker.pose.position.x = state.center.x();
    center_marker.pose.position.y = state.center.y();
    center_marker.pose.position.z = state.center.z();
    center_marker.scale.x = 0.1;
    center_marker.scale.y = 0.1;
    center_marker.scale.z = 0.1;
    center_marker.color.r = 1.0;
    center_marker.color.g = 1.0;
    center_marker.color.b = 0.0;
    center_marker.color.a = 0.8;
    markers.markers.push_back(center_marker);
    
    // 装甲板标记
    for (int i = 0; i < 3; ++i) {
      visualization_msgs::msg::Marker armor_marker;
      armor_marker.header.stamp = stamp;
      armor_marker.header.frame_id = "camera_link";
      armor_marker.ns = "outpost_armors";
      armor_marker.id = i + 1;
      armor_marker.type = visualization_msgs::msg::Marker::CUBE;
      armor_marker.action = visualization_msgs::msg::Marker::ADD;
      armor_marker.pose.position.x = predictions[i].position.x();
      armor_marker.pose.position.y = predictions[i].position.y();
      armor_marker.pose.position.z = predictions[i].position.z();
      
      // 设置朝向
      tf2::Quaternion q;
      q.setRPY(0, 0, predictions[i].orientation);
      armor_marker.pose.orientation.x = q.x();
      armor_marker.pose.orientation.y = q.y();
      armor_marker.pose.orientation.z = q.z();
      armor_marker.pose.orientation.w = q.w();
      
      armor_marker.scale.x = 0.135;  // 装甲板宽度
      armor_marker.scale.y = 0.02;   // 厚度
      armor_marker.scale.z = 0.055;  // 高度
      
      // 可打的装甲板显示绿色,否则红色
      if (predictions[i].shootable) {
        armor_marker.color.r = 0.0;
        armor_marker.color.g = 1.0;
        armor_marker.color.b = 0.0;
      } else {
        armor_marker.color.r = 1.0;
        armor_marker.color.g = 0.0;
        armor_marker.color.b = 0.0;
      }
      armor_marker.color.a = 0.8;
      
      markers.markers.push_back(armor_marker);
    }
    
    // 预测轨迹 (未来1秒的装甲板轨迹)
    double omega = std::abs(state.omega);
    if (omega > 0.1) {
      for (int i = 0; i < 3; ++i) {
        visualization_msgs::msg::Marker trajectory_marker;
        trajectory_marker.header.stamp = stamp;
        trajectory_marker.header.frame_id = "camera_link";
        trajectory_marker.ns = "outpost_trajectory";
        trajectory_marker.id = i + 10;
        trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
        trajectory_marker.scale.x = 0.01;  // 线宽
        
        // 生成轨迹点
        for (double t = 0; t <= 1.0; t += 0.02) {
          auto future_predictions = predictor_->predictArmors(t);
          geometry_msgs::msg::Point p;
          p.x = future_predictions[i].position.x();
          p.y = future_predictions[i].position.y();
          p.z = future_predictions[i].position.z();
          trajectory_marker.points.push_back(p);
        }
        
        trajectory_marker.color.r = 0.5;
        trajectory_marker.color.g = 0.5;
        trajectory_marker.color.b = 1.0;
        trajectory_marker.color.a = 0.5;
        
        markers.markers.push_back(trajectory_marker);
      }
    }
    
    // 瞄准点标记
    auto aim = predictor_->computeAim();
    if (aim.target_armor_id >= 0) {
      visualization_msgs::msg::Marker aim_marker;
      aim_marker.header.stamp = stamp;
      aim_marker.header.frame_id = "camera_link";
      aim_marker.ns = "outpost_aim";
      aim_marker.id = 0;
      aim_marker.type = visualization_msgs::msg::Marker::SPHERE;
      aim_marker.action = visualization_msgs::msg::Marker::ADD;
      aim_marker.pose.position.x = aim.aim_point.x();
      aim_marker.pose.position.y = aim.aim_point.y();
      aim_marker.pose.position.z = aim.aim_point.z();
      aim_marker.scale.x = 0.05;
      aim_marker.scale.y = 0.05;
      aim_marker.scale.z = 0.05;
      aim_marker.color.r = aim.should_shoot ? 0.0 : 1.0;
      aim_marker.color.g = aim.should_shoot ? 1.0 : 0.0;
      aim_marker.color.b = 0.0;
      aim_marker.color.a = 1.0;
      markers.markers.push_back(aim_marker);
    }
    
    marker_pub_->publish(markers);
  }
  
  // 成员变量
  std::unique_ptr<OutpostPredictorV2> predictor_;
  
  // ROS2接口
  rclcpp::Subscription<armor_detector_ros2::msg::ArmorPoseArray>::SharedPtr armor_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr aim_point_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr aim_angles_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr should_shoot_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::Publisher<armor_detector_ros2::msg::OutpostStateV2>::SharedPtr state_v2_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr predict_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  
  // 状态变量
  int lost_count_ = 0;
  int rejected_count_ = 0;  // 观测被拒绝的连续次数（用于检测跟丢）
  int max_lost_count_ = 30;
  bool reset_on_lost_ = false;
};

}  // namespace armor_detector


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<armor_detector::OutpostPredictorNodeV2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
