/**
 * @file outpost_estimator.cpp
 * @brief Outpost state estimator implementation
 * 
 * Reference: OrangeAim-Drone OutpostPredictor implementation
 * Requirements: 7.1, 7.2, 7.3, 7.4
 */

#include "core/outpost_estimator.hpp"

#include <cmath>

namespace armor_detector
{

OutpostEstimator::OutpostEstimator()
: ekf_(nullptr),
  dt_(0.01),
  direction_(0),
  prev_direction_(0),
  chi_square_threshold_(11.07),
  chi_square_value_(0.0),
  initialized_(false),
  use_fixed_omega_(true),
  direction_samples_(0),
  theta_change_sum_(0.0),
  last_theta_(0.0)
{
  // Initialize matrices
  F_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  Q_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
  R_ = Eigen::MatrixXd::Identity(MEAS_DIM, MEAS_DIM);
  H_ = Eigen::MatrixXd::Zero(MEAS_DIM, STATE_DIM);
  x_predicted_ = Eigen::VectorXd::Zero(STATE_DIM);
}

void OutpostEstimator::initialize(const ArmorMeasurement & measurement, int direction)
{
  // Convert measurement to Cartesian coordinates
  double distance = measurement.distance;
  double yaw = measurement.yaw;
  double pitch = measurement.pitch;

  // Armor position in camera/gimbal frame
  // Note: yaw is negated because positive yaw is to the left
  double x = distance * std::cos(pitch) * std::sin(-yaw);
  double y = distance * std::cos(pitch) * std::cos(yaw);
  double z = distance * std::sin(pitch);

  // Compute center position from armor position
  // 装甲板位置 = 中心位置 + 半径 * (sin(theta), -cos(theta), 0)
  // 所以中心位置 = 装甲板位置 - 半径 * (sin(theta), -cos(theta), 0)
  //             = (x - R*sin(theta), y + R*cos(theta), z)
  double x_c = x - OUTPOST_RADIUS * std::sin(measurement.theta_world);
  double y_c = y + OUTPOST_RADIUS * std::cos(measurement.theta_world);
  double z_c = z;
  


  // Initialize state vector
  // [x_c, v_x, y_c, v_y, z_c, v_z, theta, omega]
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(STATE_DIM);
  
  // 使用固定的 omega 值（前哨站角速度是已知常量）
  // direction: -1 = 顺时针, 1 = 逆时针, 0 = 未知
  // 如果方向未知，先设置为顺时针，后续通过观测自动检测
  int init_direction = (direction != 0) ? direction : -1;
  double initial_omega = init_direction * OUTPOST_OMEGA;
  use_fixed_omega_ = true;  // 默认使用固定 omega
  direction_ = init_direction;  // 设置初始方向
  
  x0 << x_c, 0.0, y_c, 0.0, z_c, 0.0, 
        measurement.theta_world, 
        initial_omega;

  // Initialize covariance
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 0.1;

  // Create EKF with angle wrapping for theta (index 6)
  auto angle_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    Eigen::VectorXd result = a + b;
    // Wrap theta (index 6) to [-pi, pi]
    while (result(6) > M_PI) result(6) -= 2 * M_PI;
    while (result(6) < -M_PI) result(6) += 2 * M_PI;
    return result;
  };

  ekf_ = std::make_unique<ExtendedKalmanFilter>(x0, P0, angle_add);

  // Store direction and measurement
  // 注意：direction_ 已经在上面设置为 init_direction，不要覆盖！
  // direction_ = direction;  // BUG: 这行会把 direction_ 覆盖为 0
  prev_direction_ = init_direction;
  last_measurement_ = measurement;
  x_predicted_ = x0;
  initialized_ = true;
}

void OutpostEstimator::normalizeAngle(double & angle)
{
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
}

void OutpostEstimator::setupTransitionMatrix()
{
  // State transition matrix for constant velocity model
  // x_c(k+1) = x_c(k) + v_x(k) * dt
  // v_x(k+1) = v_x(k)
  // ... same for y, z
  // theta(k+1) = theta(k) + omega(k) * dt
  // omega(k+1) = omega(k)
  
  F_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  F_(0, 1) = dt_;  // x_c += v_x * dt
  F_(2, 3) = dt_;  // y_c += v_y * dt
  F_(4, 5) = dt_;  // z_c += v_z * dt
  F_(6, 7) = dt_;  // theta += omega * dt
}

void OutpostEstimator::setupProcessNoise()
{
  // Process noise using acceleration model
  // For each position-velocity pair: Q = [dt^4/4, dt^3/2; dt^3/2, dt^2] * sigma^2
  
  Eigen::Matrix2d process_block;
  double dt2 = dt_ * dt_;
  double dt3 = dt2 * dt_;
  double dt4 = dt3 * dt_;
  
  // Process noise variances
  // 前哨站中心位置应该是静止的，所以 process noise 应该很小
  // 这样可以防止中心位置漂移
  constexpr double PROCESS_NOISE_XY = 0.1;  // 减小，防止中心漂移
  constexpr double PROCESS_NOISE_Z = 0.1;   // 减小，防止中心漂移
  // 使用固定 omega 时，theta 的 process noise 可以较小
  constexpr double PROCESS_NOISE_THETA = 0.1;

  Q_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);

  // X position-velocity block
  process_block << dt4 / 4, dt3 / 2,
                   dt3 / 2, dt2;
  Q_.block<2, 2>(0, 0) = process_block * PROCESS_NOISE_XY;

  // Y position-velocity block
  Q_.block<2, 2>(2, 2) = process_block * PROCESS_NOISE_XY;

  // Z position-velocity block
  Q_.block<2, 2>(4, 4) = process_block * PROCESS_NOISE_Z;

  // Theta-omega block
  Q_.block<2, 2>(6, 6) = process_block * PROCESS_NOISE_THETA;
}

void OutpostEstimator::setupMeasurementNoise(double distance)
{
  // Measurement noise covariance
  // 增大噪声以提高稳定性和平滑度
  constexpr double YAW_NOISE = 0.1;
  constexpr double PITCH_NOISE = 0.1;
  constexpr double DISTANCE_NOISE = 0.1;
  constexpr double THETA_NOISE = 1.0;  // theta 噪声较大，因为装甲板朝向检测不太准

  R_ = Eigen::MatrixXd::Zero(MEAS_DIM, MEAS_DIM);
  R_(0, 0) = YAW_NOISE;
  R_(1, 1) = PITCH_NOISE;
  R_(2, 2) = DISTANCE_NOISE;
  R_(3, 3) = THETA_NOISE;
}

void OutpostEstimator::computeArmorPosition(
  double x_c, double y_c, double z_c, double theta,
  double & x_a, double & y_a, double & z_a) const
{
  // Armor position is at radius distance from center
  // in the direction perpendicular to the armor face
  x_a = x_c + OUTPOST_RADIUS * std::sin(theta);
  y_a = y_c - OUTPOST_RADIUS * std::cos(theta);
  z_a = z_c;
}

Eigen::VectorXd OutpostEstimator::observationFunction(const Eigen::VectorXd & state) const
{
  // Extract state components
  double x_c = state(0);
  double y_c = state(2);
  double z_c = state(4);
  double theta = state(6);

  // Compute armor position
  double x_a, y_a, z_a;
  computeArmorPosition(x_c, y_c, z_c, theta, x_a, y_a, z_a);

  // Compute expected measurement
  Eigen::VectorXd z(MEAS_DIM);
  
  // yaw = -atan2(x_a, y_a)
  z(0) = -std::atan2(x_a, y_a);
  
  // pitch = atan2(z_a, sqrt(x_a^2 + y_a^2))
  double xy_dist = std::sqrt(x_a * x_a + y_a * y_a);
  z(1) = std::atan2(z_a, xy_dist);
  
  // distance = sqrt(x_a^2 + y_a^2 + z_a^2)
  z(2) = std::sqrt(x_a * x_a + y_a * y_a + z_a * z_a);
  
  // theta_a = theta
  z(3) = theta;

  return z;
}

void OutpostEstimator::computeObservationJacobian()
{
  // Compute Jacobian numerically using finite differences
  // This is simpler and more robust than analytical derivation
  
  const double eps = 1e-6;
  Eigen::VectorXd state = ekf_->x;
  Eigen::VectorXd h0 = observationFunction(state);

  H_ = Eigen::MatrixXd::Zero(MEAS_DIM, STATE_DIM);

  for (int i = 0; i < STATE_DIM; ++i) {
    Eigen::VectorXd state_plus = state;
    state_plus(i) += eps;
    Eigen::VectorXd h_plus = observationFunction(state_plus);
    
    H_.col(i) = (h_plus - h0) / eps;
  }
}

Eigen::VectorXd OutpostEstimator::measurementToVector(const ArmorMeasurement & measurement) const
{
  Eigen::VectorXd z(MEAS_DIM);
  z << measurement.yaw, measurement.pitch, measurement.distance, measurement.theta_world;
  return z;
}

bool OutpostEstimator::chiSquareTest(
  const Eigen::VectorXd & innovation,
  const Eigen::MatrixXd & innovation_cov)
{
  // Chi-square test: chi2 = innovation^T * S^(-1) * innovation
  // Using Cholesky decomposition for numerical stability
  Eigen::LLT<Eigen::MatrixXd> llt(innovation_cov);
  if (llt.info() != Eigen::Success) {
    // Matrix not positive definite, use direct inverse
    chi_square_value_ = innovation.transpose() * innovation_cov.inverse() * innovation;
  } else {
    Eigen::VectorXd weighted = llt.solve(innovation);
    chi_square_value_ = innovation.dot(weighted);
  }

  return chi_square_value_ <= chi_square_threshold_;
}

void OutpostEstimator::predict()
{
  if (!initialized_ || !ekf_) {
    return;
  }

  setupTransitionMatrix();
  setupProcessNoise();

  // Nonlinear state transition (with angle wrapping)
  auto f = [this](const Eigen::VectorXd & state) {
    Eigen::VectorXd new_state = F_ * state;
    // Angle wrapping is handled by the custom addition function
    return new_state;
  };

  x_predicted_ = ekf_->predict(F_, Q_, f);
}

bool OutpostEstimator::update(const ArmorMeasurement & measurement)
{
  if (!initialized_ || !ekf_) {
    return false;
  }

  // 通过观测 theta 变化来检测旋转方向
  // 只在前几个样本检测，之后保持方向不变
  if (direction_samples_ < DIRECTION_DETECTION_SAMPLES) {
    double theta_diff = measurement.theta_world - last_theta_;
    normalizeAngle(theta_diff);
    theta_change_sum_ += theta_diff;
    direction_samples_++;
    last_theta_ = measurement.theta_world;
    
    // 收集足够样本后确定方向
    if (direction_samples_ >= DIRECTION_DETECTION_SAMPLES) {
      // 使用更大的阈值来确保方向检测准确
      if (theta_change_sum_ > 0.2) {
        direction_ = 1;   // 逆时针 (theta 增加)
      } else if (theta_change_sum_ < -0.2) {
        direction_ = -1;  // 顺时针 (theta 减少)
      }
      // 否则保持初始方向
      
      // 设置 omega
      ekf_->x(7) = direction_ * OUTPOST_OMEGA;
      x_predicted_(7) = direction_ * OUTPOST_OMEGA;
    }
  }

  // Setup matrices
  setupTransitionMatrix();
  setupProcessNoise();
  setupMeasurementNoise(measurement.distance);

  // Predict step
  auto f = [this](const Eigen::VectorXd & state) {
    return F_ * state;
  };
  x_predicted_ = ekf_->predict(F_, Q_, f);

  // 如果使用固定 omega，在预测后强制设置 omega 为固定值
  if (use_fixed_omega_ && direction_ != 0) {
    ekf_->x(7) = direction_ * OUTPOST_OMEGA;
    x_predicted_(7) = direction_ * OUTPOST_OMEGA;
  }

  // Compute observation Jacobian at predicted state
  computeObservationJacobian();

  // Get measurement vector
  Eigen::VectorXd z = measurementToVector(measurement);

  // Compute expected measurement
  Eigen::VectorXd z_pred = observationFunction(ekf_->x);

  // Compute innovation (with angle wrapping for yaw and theta)
  Eigen::VectorXd innovation = z - z_pred;
  normalizeAngle(innovation(0));  // yaw
  normalizeAngle(innovation(3));  // theta

  // Compute innovation covariance: S = H * P * H^T + R
  Eigen::MatrixXd S = H_ * ekf_->P * H_.transpose() + R_;

  // Chi-square test
  bool measurement_valid = chiSquareTest(innovation, S);

  // Special case: if velocity is zero (just initialized), accept measurement
  if (std::abs(ekf_->x(1)) < 1e-6 && std::abs(ekf_->x(3)) < 1e-6) {
    measurement_valid = true;
  }

  if (measurement_valid) {
    // Custom measurement subtraction with angle wrapping
    auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
      Eigen::VectorXd result = a - b;
      // Wrap angles (yaw at index 0, theta at index 3)
      while (result(0) > M_PI) result(0) -= 2 * M_PI;
      while (result(0) < -M_PI) result(0) += 2 * M_PI;
      while (result(3) > M_PI) result(3) -= 2 * M_PI;
      while (result(3) < -M_PI) result(3) += 2 * M_PI;
      return result;
    };

    // Update with nonlinear observation
    ekf_->update(z, H_, R_, 
      [this](const Eigen::VectorXd & state) { return observationFunction(state); },
      z_subtract);

    // 如果使用固定 omega，更新后也强制设置
    if (use_fixed_omega_ && direction_ != 0) {
      ekf_->x(7) = direction_ * OUTPOST_OMEGA;
    }
    
    // 更新 x_predicted_ 以反映最新状态
    x_predicted_ = ekf_->x;
    
    // 使用指数移动平均校正中心位置，防止漂移同时保持平滑
    // alpha 越小越平滑，但响应越慢
    constexpr double ALPHA = 0.05;  // 平滑因子，减小以减少抖动
    
    double distance = measurement.distance;
    double yaw = measurement.yaw;
    double pitch = measurement.pitch;
    double x = distance * std::cos(pitch) * std::sin(-yaw);
    double y = distance * std::cos(pitch) * std::cos(yaw);
    double z_pos = distance * std::sin(pitch);
    
    // 使用 EKF 估计的 theta 来计算中心
    double theta = ekf_->x(6);
    double x_c = x - OUTPOST_RADIUS * std::sin(theta);
    double y_c = y + OUTPOST_RADIUS * std::cos(theta);
    
    // 指数移动平均更新中心位置
    ekf_->x(0) = (1.0 - ALPHA) * ekf_->x(0) + ALPHA * x_c;
    ekf_->x(2) = (1.0 - ALPHA) * ekf_->x(2) + ALPHA * y_c;
    ekf_->x(4) = (1.0 - ALPHA) * ekf_->x(4) + ALPHA * z_pos;
  }

  // Store measurement for direction detection
  last_measurement_ = measurement;

  // 确保 x_predicted_ 的 omega 总是固定值
  if (use_fixed_omega_ && direction_ != 0) {
    x_predicted_(7) = direction_ * OUTPOST_OMEGA;
  }

  return measurement_valid;
}

bool OutpostEstimator::detectDirectionChange()
{
  bool changed = (direction_ != 0 && prev_direction_ != 0 && direction_ != prev_direction_);
  prev_direction_ = direction_;
  return changed;
}

OutpostInformation OutpostEstimator::getInformation(bool use_predicted) const
{
  OutpostInformation info;

  if (!initialized_ || !ekf_) {
    info.is_valid = false;
    return info;
  }

  Eigen::VectorXd state = use_predicted ? x_predicted_ : ekf_->x;

  info.center_position = Eigen::Vector3d(state(0), state(2), state(4));
  info.center_velocity = Eigen::Vector3d(state(1), state(3), state(5));
  info.outpost_radius = OUTPOST_RADIUS;
  info.outpost_theta = state(6);
  // 如果使用固定 omega，直接返回固定值
  info.outpost_omega = (use_fixed_omega_ && direction_ != 0) ? 
                       direction_ * OUTPOST_OMEGA : state(7);
  info.direction = direction_;
  info.is_valid = true;

  return info;
}

void OutpostEstimator::reset()
{
  ekf_.reset();
  initialized_ = false;
  direction_ = 0;
  prev_direction_ = 0;
  chi_square_value_ = 0.0;
  x_predicted_ = Eigen::VectorXd::Zero(STATE_DIM);
  direction_samples_ = 0;
  theta_change_sum_ = 0.0;
  last_theta_ = 0.0;
}

Eigen::VectorXd OutpostEstimator::getState() const
{
  if (!initialized_ || !ekf_) {
    return Eigen::VectorXd::Zero(STATE_DIM);
  }
  return ekf_->x;
}

Eigen::MatrixXd OutpostEstimator::getCovariance() const
{
  if (!initialized_ || !ekf_) {
    return Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  }
  return ekf_->P;
}

}  // namespace armor_detector
