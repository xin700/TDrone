/**
 * @file outpost_predictor_node_v3.cpp
 * @brief 前哨站预测器 V3 ROS2 节点
 * 
 * 功能:
 *   - 订阅装甲板位姿信息
 *   - 使用 V3 预测器进行状态估计
 *   - 发布瞄准结果和状态信息
 *   - 支持参数动态更新
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "core/outpost_predictor_v3.hpp"
#include "armor_detector_ros2/msg/armor_pose_array.hpp"

namespace armor_detector
{

class OutpostPredictorNodeV3 : public rclcpp::Node
{
public:
  OutpostPredictorNodeV3() : Node("outpost_predictor_v3")
  {
    RCLCPP_INFO(this->get_logger(), "初始化前哨站预测器V3节点...");
    
    // 声明参数
    declareParameters();
    
    // 创建预测器并加载配置
    predictor_ = std::make_unique<OutpostPredictorV3>();
    loadConfig();
    
    // 订阅装甲板位姿
    armor_sub_ = this->create_subscription<armor_detector_ros2::msg::ArmorPoseArray>(
      "/solver/armor_poses", 10,
      std::bind(&OutpostPredictorNodeV3::armorCallback, this, std::placeholders::_1));
    
    // 发布瞄准点
    aim_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/predictor_v3/aim_point", 10);
    
    // 发布瞄准角度 [yaw, pitch]
    aim_angles_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/predictor_v3/aim_angles", 10);
    
    // 发布射击建议
    should_shoot_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/predictor_v3/should_shoot", 10);
    
    // 发布状态向量 [theta, omega, x_c, y_c, z_c]
    state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/predictor_v3/state", 10);
    
    // 发布文本状态 (用于可视化)
    state_text_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/predictor_v3/state_text", 10);
    
    // 发布瞄准模式
    aim_mode_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/predictor_v3/aim_mode", 10);
    
    // 发布详细状态 (用于可视化)
    detail_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/predictor_v3/detail", 10);
    
    // 发布可视化标记
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/predictor_v3/markers", 10);
    
    // 定时预测 (即使无观测也更新)
    predict_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),  // 100Hz
      std::bind(&OutpostPredictorNodeV3::predictTimerCallback, this));
    
    // 参数变化回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&OutpostPredictorNodeV3::parametersCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "前哨站预测器V3节点初始化完成");
  }

private:
  void declareParameters()
  {
    // 几何参数
    this->declare_parameter("geometry.radius", 0.2767);
    this->declare_parameter("geometry.height_diff", 0.10);
    this->declare_parameter("geometry.tilt_angle", 1.309);
    
    // 运动参数
    this->declare_parameter("use_fixed_omega", true);
    this->declare_parameter("known_omega", 2.5133);
    this->declare_parameter("omega_threshold", 0.3);
    
    // 过程噪声
    this->declare_parameter("noise.q_theta", 0.01);
    this->declare_parameter("noise.q_omega", 0.001);
    this->declare_parameter("noise.q_position", 0.001);
    
    // 观测噪声
    this->declare_parameter("noise.r_yaw", 0.01);
    this->declare_parameter("noise.r_pitch", 0.01);
    this->declare_parameter("noise.r_distance", 0.01);
    this->declare_parameter("noise.r_orientation", 0.05);
    
    // 子弹参数
    this->declare_parameter("bullet_speed", 28.0);
    this->declare_parameter("system_delay", 0.05);
    
    // 瞄准参数
    this->declare_parameter("max_shootable_angle", 1.047);
    
    // 多假设参数
    this->declare_parameter("hypothesis_converge_threshold", 0.8);
    this->declare_parameter("min_updates_to_converge", 15);
    this->declare_parameter("chi_square_threshold", 9.49);
    this->declare_parameter("switch_angle_threshold", 1.047);
    
    // 重置参数
    this->declare_parameter("max_lost_frames", 30);
  }
  
  void loadConfig()
  {
    OutpostConfigV3 config;
    
    // 几何参数
    config.geometry.radius = this->get_parameter("geometry.radius").as_double();
    config.geometry.height_diff = this->get_parameter("geometry.height_diff").as_double();
    config.geometry.tilt_angle = this->get_parameter("geometry.tilt_angle").as_double();
    
    // 运动参数
    config.use_fixed_omega = this->get_parameter("use_fixed_omega").as_bool();
    config.known_omega = this->get_parameter("known_omega").as_double();
    config.omega_threshold = this->get_parameter("omega_threshold").as_double();
    
    // 过程噪声
    config.noise.q_theta = this->get_parameter("noise.q_theta").as_double();
    config.noise.q_omega = this->get_parameter("noise.q_omega").as_double();
    config.noise.q_position = this->get_parameter("noise.q_position").as_double();
    
    // 观测噪声
    config.noise.r_yaw = this->get_parameter("noise.r_yaw").as_double();
    config.noise.r_pitch = this->get_parameter("noise.r_pitch").as_double();
    config.noise.r_distance = this->get_parameter("noise.r_distance").as_double();
    config.noise.r_orientation = this->get_parameter("noise.r_orientation").as_double();
    
    // 子弹参数
    config.bullet_speed = this->get_parameter("bullet_speed").as_double();
    config.system_delay = this->get_parameter("system_delay").as_double();
    
    // 瞄准参数
    config.max_shootable_angle = this->get_parameter("max_shootable_angle").as_double();
    
    // 多假设参数
    config.hypothesis_converge_threshold = 
      this->get_parameter("hypothesis_converge_threshold").as_double();
    config.min_updates_to_converge = 
      this->get_parameter("min_updates_to_converge").as_int();
    config.chi_square_threshold = this->get_parameter("chi_square_threshold").as_double();
    config.switch_angle_threshold = this->get_parameter("switch_angle_threshold").as_double();
    
    // 重置参数
    config.max_lost_frames = this->get_parameter("max_lost_frames").as_int();
    
    predictor_->setConfig(config);
    
    RCLCPP_INFO(this->get_logger(), 
      "V3配置已加载: radius=%.4f, height_diff=%.3f, omega=%.3f",
      config.geometry.radius, config.geometry.height_diff, config.known_omega);
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
      if (lost_count_ > predictor_->getConfig().max_lost_frames) {
        // 长时间丢失，可以选择重置
        // predictor_->reset();
      }
      return;
    }
    
    lost_count_ = 0;
    
    // 获取第一个装甲板
    const auto& armor = msg->poses[0];
    
    // ========================================================================
    // 坐标系转换：机架坐标系 → 预测器内部坐标系（相机坐标系）
    // 机架坐标系: X右、Y上、Z后（机头前方是-Z）
    // 预测器坐标系: X右、Y下、Z前（相机坐标系）
    // 转换: X_cam = X_body, Y_cam = -Y_body, Z_cam = -Z_body
    // ========================================================================
    double x_cam = armor.position.x;       // X保持
    double y_cam = -armor.position.y;      // Y翻转
    double z_cam = -armor.position.z;      // Z翻转
    
    // 构建观测
    ArmorObservationV3 obs;
    obs.position = Eigen::Vector3d(x_cam, y_cam, z_cam);
    
    // 计算极坐标（预测器内部坐标系: X右、Y下、Z前）
    double dist = obs.position.norm();
    obs.distance = dist;
    
    if (dist > 0.01) {
      // 相机坐标系下的角度计算
      // yaw = atan2(x, z)，相对于Z轴（前方）的水平偏角
      obs.yaw = std::atan2(x_cam, z_cam);
      // pitch = asin(-y/dist)，Y向下为正，所以向上看时pitch为负
      obs.pitch = std::asin(-y_cam / dist);
    }
    
    // 装甲板朝向（需要适配）
    // 在机架坐标系中，armor_yaw是相对于-Z方向的朝向角
    // 在相机坐标系中，朝向角是相对于Z方向的
    // 转换: orientation_cam = armor_yaw + π 或 orientation_cam = -armor_yaw
    // 简化：装甲板法向量在水平面上的角度，需要翻转Z轴的影响
    obs.orientation = armor.armor_yaw + M_PI;  // 翻转180度
    if (obs.orientation > M_PI) obs.orientation -= 2 * M_PI;
    
    // 时间戳
    obs.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    
    obs.valid = true;
    
    // 更新预测器
    predictor_->update(obs);
    
    // 发布结果
    publishResults(msg->header);
  }
  
  void predictTimerCallback()
  {
    if (!predictor_->isInitialized()) {
      return;
    }
    
    // 周期性预测 (补偿无观测时的状态漂移)
    predictor_->predict(0.01);  // 10ms
    
    // 发布状态 (即使无观测也发布预测状态)
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera_link";
    publishResults(header);
  }
  
  void publishResults(const std_msgs::msg::Header& header)
  {
    // 计算瞄准结果（预测器内部使用相机坐标系）
    AimResultV3 aim = predictor_->computeAim();
    
    // ========================================================================
    // 坐标系转换：预测器内部坐标系（相机坐标系） → 机架坐标系
    // 预测器坐标系: X右、Y下、Z前
    // 机架坐标系: X右、Y上、Z后
    // 转换: X_body = X_cam, Y_body = -Y_cam, Z_body = -Z_cam
    // ========================================================================
    double aim_x_body = aim.aim_point.x();
    double aim_y_body = -aim.aim_point.y();
    double aim_z_body = -aim.aim_point.z();
    
    // 发布瞄准点（机架坐标系）
    auto aim_point_msg = geometry_msgs::msg::PointStamped();
    aim_point_msg.header = header;
    aim_point_msg.point.x = aim_x_body;
    aim_point_msg.point.y = aim_y_body;
    aim_point_msg.point.z = aim_z_body;
    aim_point_pub_->publish(aim_point_msg);
    
    // 重新计算瞄准角度（机架坐标系）
    // 机架坐标系: 前方是-Z，Y上为正
    // yaw = atan2(x, -z)，正值表示目标在右边
    // pitch = atan2(y, sqrt(x²+z²))，正值表示目标在上方
    double aim_yaw_body = std::atan2(aim_x_body, -aim_z_body);
    double xz_dist = std::sqrt(aim_x_body * aim_x_body + aim_z_body * aim_z_body);
    double aim_pitch_body = std::atan2(aim_y_body, xz_dist);
    
    // 发布瞄准角度
    auto angles_msg = std_msgs::msg::Float64MultiArray();
    angles_msg.data = {aim_yaw_body, aim_pitch_body, aim.distance, aim.confidence};
    aim_angles_pub_->publish(angles_msg);
    
    // 发布射击建议
    auto shoot_msg = std_msgs::msg::Bool();
    shoot_msg.data = aim.should_shoot;
    should_shoot_pub_->publish(shoot_msg);
    
    // 发布状态向量（预测器内部状态，不转换）
    auto state = predictor_->getStateVector();
    auto state_msg = std_msgs::msg::Float64MultiArray();
    state_msg.data.resize(state.size());
    for (int i = 0; i < state.size(); ++i) {
      state_msg.data[i] = state(i);
    }
    state_pub_->publish(state_msg);
    
    // 发布文本状态 (用于可视化)
    auto state_text_msg = std_msgs::msg::String();
    if (!predictor_->isInitialized()) {
      state_text_msg.data = "INITIALIZING";
    } else {
      auto outpost_state = predictor_->getState();
      if (outpost_state.converged) {
        state_text_msg.data = "TRACKING";
      } else {
        state_text_msg.data = "CONVERGING";
      }
    }
    state_text_pub_->publish(state_text_msg);
    
    // 发布瞄准模式
    auto mode_msg = std_msgs::msg::String();
    mode_msg.data = aim.aim_mode;
    aim_mode_pub_->publish(mode_msg);
    
    // 发布详细状态（转换中心位置到机架坐标系）
    auto outpost_state = predictor_->getState();
    // 中心位置从相机坐标系转换到机架坐标系
    double center_x_body = outpost_state.center.x();
    double center_y_body = -outpost_state.center.y();
    double center_z_body = -outpost_state.center.z();
    
    auto detail_msg = std_msgs::msg::Float64MultiArray();
    detail_msg.data = {
      outpost_state.theta,
      outpost_state.omega,
      center_x_body,
      center_y_body,
      center_z_body,
      static_cast<double>(outpost_state.direction),
      static_cast<double>(outpost_state.converged),
      static_cast<double>(static_cast<int>(outpost_state.best_hypothesis_type)),
      outpost_state.hypothesis_confidences[0],
      outpost_state.hypothesis_confidences[1],
      outpost_state.hypothesis_confidences[2]
    };
    detail_pub_->publish(detail_msg);
    
    // 发布可视化标记
    publishMarkers(header, aim);
  }
  
  void publishMarkers(const std_msgs::msg::Header& header, const AimResultV3& aim)
  {
    visualization_msgs::msg::MarkerArray markers;
    
    // 获取预测的装甲板位置（预测器内部坐标系）
    auto armors = predictor_->predictArmors(0.0);
    auto state = predictor_->getState();
    
    // ========================================================================
    // 坐标转换：预测器内部坐标系 → 机架坐标系
    // X_body = X_cam, Y_body = -Y_cam, Z_body = -Z_cam
    // ========================================================================
    
    // 1. 中心点标记（转换到机架坐标系）
    {
      visualization_msgs::msg::Marker center_marker;
      center_marker.header = header;
      center_marker.ns = "outpost_center";
      center_marker.id = 0;
      center_marker.type = visualization_msgs::msg::Marker::SPHERE;
      center_marker.action = visualization_msgs::msg::Marker::ADD;
      center_marker.pose.position.x = state.center.x();
      center_marker.pose.position.y = -state.center.y();  // Y翻转
      center_marker.pose.position.z = -state.center.z();  // Z翻转
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
    
    // 2. 装甲板位置标记（转换到机架坐标系）
    for (int i = 0; i < 3; ++i) {
      visualization_msgs::msg::Marker armor_marker;
      armor_marker.header = header;
      armor_marker.ns = "outpost_armors";
      armor_marker.id = i;
      armor_marker.type = visualization_msgs::msg::Marker::CUBE;
      armor_marker.action = visualization_msgs::msg::Marker::ADD;
      armor_marker.pose.position.x = armors[i].position.x();
      armor_marker.pose.position.y = -armors[i].position.y();  // Y翻转
      armor_marker.pose.position.z = -armors[i].position.z();  // Z翻转
      
      // 设置朝向（需要适配机架坐标系）
      // 相机坐标系中的orientation转换到机架坐标系
      double orientation_body = armors[i].orientation + M_PI;
      if (orientation_body > M_PI) orientation_body -= 2 * M_PI;
      
      tf2::Quaternion q;
      q.setRPY(0, 0, orientation_body);
      armor_marker.pose.orientation.x = q.x();
      armor_marker.pose.orientation.y = q.y();
      armor_marker.pose.orientation.z = q.z();
      armor_marker.pose.orientation.w = q.w();
      
      armor_marker.scale.x = 0.135;  // 装甲板宽度
      armor_marker.scale.y = 0.055;  // 装甲板高度
      armor_marker.scale.z = 0.01;   // 厚度
      
      // 颜色: 可打=绿色, 不可打=红色
      if (armors[i].shootable) {
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
    
    // 3. 瞄准点标记（转换到机架坐标系）
    {
      visualization_msgs::msg::Marker aim_marker;
      aim_marker.header = header;
      aim_marker.ns = "aim_point";
      aim_marker.id = 0;
      aim_marker.type = visualization_msgs::msg::Marker::SPHERE;
      aim_marker.action = visualization_msgs::msg::Marker::ADD;
      aim_marker.pose.position.x = aim.aim_point.x();
      aim_marker.pose.position.y = -aim.aim_point.y();   // Y翻转
      aim_marker.pose.position.z = -aim.aim_point.z();   // Z翻转
      aim_marker.scale.x = 0.03;
      aim_marker.scale.y = 0.03;
      aim_marker.scale.z = 0.03;
      
      if (aim.should_shoot) {
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
    
    // 4. 瞄准线（转换到机架坐标系）
    {
      visualization_msgs::msg::Marker line_marker;
      line_marker.header = header;
      line_marker.ns = "aim_line";
      line_marker.id = 0;
      line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_marker.action = visualization_msgs::msg::Marker::ADD;
      
      geometry_msgs::msg::Point p1, p2;
      p1.x = 0; p1.y = 0; p1.z = 0;  // 相机位置
      p2.x = aim.aim_point.x();
      p2.y = -aim.aim_point.y();   // Y翻转
      p2.z = -aim.aim_point.z();   // Z翻转
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
  std::unique_ptr<OutpostPredictorV3> predictor_;
  
  // 订阅者
  rclcpp::Subscription<armor_detector_ros2::msg::ArmorPoseArray>::SharedPtr armor_sub_;
  
  // 发布者
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr aim_point_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr aim_angles_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr should_shoot_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_text_pub_;  // 文本状态
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr aim_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr detail_pub_;
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
  auto node = std::make_shared<armor_detector::OutpostPredictorNodeV3>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
