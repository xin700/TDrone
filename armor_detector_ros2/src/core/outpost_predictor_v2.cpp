/**
 * @file outpost_predictor_v2.cpp
 * @brief 前哨站预测器 V2 实现
 * 
 * 基于多假设跟踪的前哨站状态估计和预测
 * 
 * 核心机制:
 * - 同时维护3个EKF假设（假设观测的是高/中/低装甲板）
 * - 通过预测误差自动收敛到正确假设
 * - 解决了"无人机高度可变，无法用绝对高度判断"的问题
 */

#include "core/outpost_predictor_v2.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>

namespace armor_detector
{

// ============================================================================
// 构造函数
// ============================================================================

OutpostPredictorV2::OutpostPredictorV2()
{
  // 使用默认配置
  setConfig(OutpostPredictorConfig{});
}

OutpostPredictorV2::OutpostPredictorV2(const OutpostPredictorConfig& config)
{
  setConfig(config);
}

void OutpostPredictorV2::setConfig(const OutpostPredictorConfig& config)
{
  config_ = config;
  
  // 初始化矩阵大小
  F_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  Q_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
  R_ = Eigen::MatrixXd::Zero(MEAS_DIM, MEAS_DIM);
  H_ = Eigen::MatrixXd::Zero(MEAS_DIM, STATE_DIM);
  
  setupProcessNoise();
  setupMeasurementNoise();
}

// ============================================================================
// 初始化 - 使用多假设跟踪
// ============================================================================

void OutpostPredictorV2::initialize(const ArmorObservation& obs, int initial_direction)
{
  if (!obs.valid) {
    return;
  }
  
  // 使用多假设跟踪初始化
  initializeHypotheses(obs, initial_direction);
  
  // 初始化方向检测变量
  last_theta_ = obs.orientation;
  theta_change_sum_ = 0.0;
  direction_samples_ = 0;
  last_obs_timestamp_ = obs.timestamp;
  
  initialized_ = true;
  converged_ = false;
  best_hypothesis_id_ = -1;
  
  std::cout << "[OutpostPredictorV2] 初始化完成，启用多假设跟踪模式" << std::endl;
  std::cout << "  观测位置: [" << obs.position.transpose() << "]" << std::endl;
  std::cout << "  观测朝向: " << obs.orientation * 180.0 / M_PI << "°" << std::endl;
}

void OutpostPredictorV2::initializeHypotheses(const ArmorObservation& obs, int initial_direction)
{
  // 创建3个假设：分别假设观测到的是 高/中/低 装甲板
  for (int i = 0; i < 3; ++i) {
    ArmorHeight height = static_cast<ArmorHeight>(i);
    hypotheses_[i].assumed_height = height;
    hypotheses_[i].ekf = createEKFForHypothesis(obs, height, initial_direction);
    hypotheses_[i].confidence = 1.0 / 3.0;  // 初始均等置信度
    hypotheses_[i].num_updates = 1;
    hypotheses_[i].last_prediction_error = 0.0;
    
    std::cout << "  假设" << i << ": 假定观测的是" 
              << (i == 0 ? "高位" : (i == 1 ? "中位" : "低位")) 
              << "装甲板, z_center=" 
              << hypotheses_[i].ekf->x(2) << std::endl;
  }
}

std::unique_ptr<ExtendedKalmanFilter> OutpostPredictorV2::createEKFForHypothesis(
  const ArmorObservation& obs, 
  ArmorHeight assumed_height,
  int initial_direction)
{
  int armor_id = static_cast<int>(assumed_height);
  
  // 从装甲板位置反推中心位置
  // 关键: 不同假设会计算出不同的 z_center
  // 假设 HIGH (0):  z_c = z_a - h  (装甲板在上面，中心在下面)
  // 假设 MIDDLE (1): z_c = z_a     (装甲板在中间)
  // 假设 LOW (2):   z_c = z_a + h  (装甲板在下面，中心在上面)
  
  // 从观测的装甲板反推中心角度
  // 装甲板逆时针排列：id=0在theta，id=1在theta+120°，id=2在theta+240°
  // 如果观测到的装甲板角度是theta_armor，且装甲板ID是armor_id
  // 则中心角度 = theta_armor - armor_id * ARMOR_ANGLE_INTERVAL
  // （因为正向计算是 theta_armor = theta_center + armor_id * ARMOR_ANGLE_INTERVAL）
  double theta_armor = obs.orientation;
  double theta_center = normalizeAngle(theta_armor - armor_id * ARMOR_ANGLE_INTERVAL);
  
  double R = config_.geometry.radius;
  auto height_offsets = config_.geometry.getHeightOffsets();
  
  double x_c = obs.position.x() - R * std::sin(theta_armor);
  double y_c = obs.position.y() + R * std::cos(theta_armor);
  double z_c = obs.position.z() - height_offsets[armor_id];  // 关键差异在这里
  
  // 初始化状态向量
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(STATE_DIM);
  x0(0) = x_c;
  x0(1) = y_c;
  x0(2) = z_c;
  x0(3) = theta_center;
  
  // 设置初始角速度
  if (initial_direction != 0) {
    x0(4) = initial_direction * config_.known_omega;
  } else {
    x0(4) = 0.0;
  }
  
  // 初始化协方差矩阵
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  P0(0, 0) = 0.1;   // x_c
  P0(1, 1) = 0.1;   // y_c
  P0(2, 2) = 0.05;  // z_c (关键状态，给较小的初始不确定性)
  P0(3, 3) = 0.5;   // theta
  P0(4, 4) = config_.use_fixed_omega ? 0.01 : 1.0;
  
  // 创建EKF，使用自定义加法处理角度周期性
  auto angle_add = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    Eigen::VectorXd result = a + b;
    while (result(3) > M_PI) result(3) -= 2.0 * M_PI;
    while (result(3) < -M_PI) result(3) += 2.0 * M_PI;
    return result;
  };
  
  return std::make_unique<ExtendedKalmanFilter>(x0, P0, angle_add);
}

// ============================================================================
// 预测和更新 - 支持多假设跟踪
// ============================================================================

void OutpostPredictorV2::predict()
{
  if (!initialized_) {
    return;
  }
  
  setupTransitionMatrix();
  
  // 状态预测函数
  auto f = [this](const Eigen::VectorXd& state) {
    Eigen::VectorXd new_state = F_ * state;
    new_state(3) = normalizeAngle(new_state(3));
    return new_state;
  };
  
  if (converged_ && ekf_) {
    // 已收敛，只更新主EKF
    ekf_->predict(F_, Q_, f);
    if (config_.use_fixed_omega && direction_ != 0) {
      ekf_->x(4) = direction_ * config_.known_omega;
    }
  } else {
    // 未收敛，更新所有假设
    for (auto& hyp : hypotheses_) {
      if (hyp.ekf) {
        hyp.ekf->predict(F_, Q_, f);
        if (config_.use_fixed_omega && direction_ != 0) {
          hyp.ekf->x(4) = direction_ * config_.known_omega;
        }
      }
    }
  }
}

bool OutpostPredictorV2::update(const ArmorObservation& obs)
{
  if (!obs.valid) {
    return false;
  }
  
  if (!initialized_) {
    initialize(obs, 0);
    return true;
  }
  
  // 更新时间步长
  if (obs.timestamp > last_obs_timestamp_) {
    dt_ = obs.timestamp - last_obs_timestamp_;
    dt_ = std::clamp(dt_, 0.001, 0.1);
  }
  last_obs_timestamp_ = obs.timestamp;
  
  // 方向检测
  if (direction_samples_ < config_.direction_detection_samples) {
    double theta_diff = angleDiff(obs.orientation, last_theta_);
    theta_change_sum_ += theta_diff;
    direction_samples_++;
    last_theta_ = obs.orientation;
    
    if (direction_samples_ >= config_.direction_detection_samples) {
      if (theta_change_sum_ > 0.3) {
        direction_ = 1;
      } else if (theta_change_sum_ < -0.3) {
        direction_ = -1;
      } else {
        direction_ = 0;
      }
      
      // 更新所有假设的omega
      for (auto& hyp : hypotheses_) {
        if (hyp.ekf && config_.use_fixed_omega && direction_ != 0) {
          hyp.ekf->x(4) = direction_ * config_.known_omega;
        }
      }
      if (ekf_ && config_.use_fixed_omega && direction_ != 0) {
        ekf_->x(4) = direction_ * config_.known_omega;
      }
    }
  } else {
    last_theta_ = obs.orientation;
  }
  
  setupTransitionMatrix();
  setupProcessNoise();
  setupMeasurementNoise();
  
  if (converged_ && ekf_) {
    // 已收敛，只更新主EKF
    return updateSingleEKF(obs, ekf_.get(), static_cast<ArmorHeight>(best_hypothesis_id_));
  } else {
    // 未收敛，更新所有假设
    bool result = updateAllHypotheses(obs);
    checkConvergence();
    return result;
  }
}

bool OutpostPredictorV2::updateSingleEKF(
  const ArmorObservation& obs, 
  ExtendedKalmanFilter* ekf, 
  ArmorHeight assumed_height)
{
  if (!ekf) return false;
  
  int armor_id = static_cast<int>(assumed_height);
  
  // 预测步骤
  auto f = [this](const Eigen::VectorXd& state) {
    Eigen::VectorXd new_state = F_ * state;
    new_state(3) = normalizeAngle(new_state(3));
    return new_state;
  };
  ekf->predict(F_, Q_, f);
  
  // 如果使用固定角速度
  if (config_.use_fixed_omega && direction_ != 0) {
    ekf->x(4) = direction_ * config_.known_omega;
  }
  
  // 计算观测雅可比矩阵 (使用假设的装甲板ID)
  computeObservationJacobian(armor_id);
  
  // 构建观测向量
  Eigen::VectorXd z(MEAS_DIM);
  z << obs.position.x(), obs.position.y(), obs.position.z(), obs.orientation;
  
  // 预测观测
  Eigen::VectorXd z_pred = observationFunction(ekf->x, armor_id);
  
  // 计算新息 (考虑角度周期性)
  Eigen::VectorXd innovation = z - z_pred;
  innovation(3) = normalizeAngle(innovation(3));
  
  // 计算新息协方差
  Eigen::MatrixXd S = H_ * ekf->P * H_.transpose() + R_;
  
  // 卡方检验
  double chi_square = innovation.transpose() * S.inverse() * innovation;
  last_chi_square_ = chi_square;
  
  bool accept_measurement = (chi_square <= config_.chi_square_threshold);
  
  // 初始化阶段或低速时，放宽条件
  if (ekf->P(0, 0) > 0.05 || std::abs(ekf->x(4)) < config_.omega_threshold) {
    accept_measurement = true;
  }
  
  if (accept_measurement) {
    // 自定义观测减法 (处理角度)
    auto z_subtract = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
      Eigen::VectorXd result = a - b;
      result(3) = normalizeAngle(result(3));
      return result;
    };
    
    // 更新步骤
    ekf->update(
      z, H_, R_,
      [this, armor_id](const Eigen::VectorXd& state) {
        return observationFunction(state, armor_id);
      },
      z_subtract
    );
    
    // 强制固定omega
    if (config_.use_fixed_omega && direction_ != 0) {
      ekf->x(4) = direction_ * config_.known_omega;
    }
  }
  
  // 计算预测误差（用于多假设置信度更新）
  double prediction_error = innovation.head<3>().norm();  // 只用位置误差
  
  return accept_measurement;
}

// ============================================================================
// 多假设跟踪核心方法
// ============================================================================

bool OutpostPredictorV2::updateAllHypotheses(const ArmorObservation& obs)
{
  bool any_accepted = false;
  
  for (int i = 0; i < 3; ++i) {
    auto& hyp = hypotheses_[i];
    if (!hyp.ekf) continue;
    
    // 记录更新前的预测位置
    int armor_id = static_cast<int>(hyp.assumed_height);
    Eigen::Vector3d predicted_pos = computeArmorPosition(hyp.ekf->x, armor_id);
    
    // 更新EKF
    bool accepted = updateSingleEKF(obs, hyp.ekf.get(), hyp.assumed_height);
    hyp.num_updates++;
    
    // 计算预测误差
    Eigen::Vector3d obs_pos = obs.position;
    hyp.last_prediction_error = (predicted_pos - obs_pos).norm();
    
    if (accepted) {
      any_accepted = true;
    }
  }
  
  // 更新置信度
  updateHypothesisConfidences();
  
  return any_accepted;
}

void OutpostPredictorV2::updateHypothesisConfidences()
{
  // 基于预测误差计算似然
  std::array<double, 3> likelihoods;
  double sigma = 0.1;  // 位置误差的标准差估计
  
  for (int i = 0; i < 3; ++i) {
    double error = hypotheses_[i].last_prediction_error;
    // 高斯似然: exp(-error^2 / (2*sigma^2))
    likelihoods[i] = std::exp(-error * error / (2.0 * sigma * sigma));
  }
  
  // 贝叶斯更新: new_conf = likelihood * prior / normalization
  double total = 0.0;
  for (int i = 0; i < 3; ++i) {
    hypotheses_[i].confidence *= likelihoods[i];
    total += hypotheses_[i].confidence;
  }
  
  // 归一化
  if (total > 1e-10) {
    for (auto& hyp : hypotheses_) {
      hyp.confidence /= total;
    }
  } else {
    // 如果所有假设都不太可能，重置为均等
    for (auto& hyp : hypotheses_) {
      hyp.confidence = 1.0 / 3.0;
    }
  }
  
  // 打印调试信息
  std::cout << "[MultiHypothesis] 置信度: ";
  for (int i = 0; i < 3; ++i) {
    std::cout << (i == 0 ? "H=" : (i == 1 ? " M=" : " L=")) 
              << std::fixed << std::setprecision(3) << hypotheses_[i].confidence
              << "(err=" << std::setprecision(4) << hypotheses_[i].last_prediction_error << ")";
  }
  std::cout << std::endl;
}

void OutpostPredictorV2::checkConvergence()
{
  // 找到置信度最高的假设
  int best_id = 0;
  double best_conf = hypotheses_[0].confidence;
  
  for (int i = 1; i < 3; ++i) {
    if (hypotheses_[i].confidence > best_conf) {
      best_conf = hypotheses_[i].confidence;
      best_id = i;
    }
  }
  
  // 检查是否满足收敛条件
  bool enough_updates = hypotheses_[best_id].num_updates >= config_.min_updates_to_converge;
  bool high_confidence = best_conf >= config_.hypothesis_converge_threshold;
  
  if (enough_updates && high_confidence) {
    // 收敛！切换到单EKF模式
    converged_ = true;
    best_hypothesis_id_ = best_id;
    ekf_ = std::move(hypotheses_[best_id].ekf);
    
    std::cout << "[MultiHypothesis] ✓ 已收敛！确定观测的是" 
              << (best_id == 0 ? "高位" : (best_id == 1 ? "中位" : "低位")) 
              << "装甲板 (置信度=" << best_conf << ")" << std::endl;
    
    // 清理其他假设
    for (auto& hyp : hypotheses_) {
      hyp.ekf.reset();
    }
  }
}

ExtendedKalmanFilter* OutpostPredictorV2::getBestEKF() const
{
  if (converged_ && ekf_) {
    return ekf_.get();
  }
  
  // 返回置信度最高的假设的EKF
  int best_id = getBestHypothesisId();
  if (best_id >= 0 && hypotheses_[best_id].ekf) {
    return hypotheses_[best_id].ekf.get();
  }
  
  return nullptr;
}

int OutpostPredictorV2::getBestHypothesisId() const
{
  if (converged_) {
    return best_hypothesis_id_;
  }
  
  int best_id = 0;
  double best_conf = hypotheses_[0].confidence;
  
  for (int i = 1; i < 3; ++i) {
    if (hypotheses_[i].confidence > best_conf) {
      best_conf = hypotheses_[i].confidence;
      best_id = i;
    }
  }
  
  return best_id;
}

// ============================================================================
// 状态获取 - 支持多假设模式
// ============================================================================

OutpostState OutpostPredictorV2::getState() const
{
  OutpostState state;
  
  if (!initialized_) {
    state.valid = false;
    return state;
  }
  
  // 获取当前最佳EKF
  ExtendedKalmanFilter* best_ekf = getBestEKF();
  if (!best_ekf) {
    state.valid = false;
    return state;
  }
  
  state.center = Eigen::Vector3d(best_ekf->x(0), best_ekf->x(1), best_ekf->x(2));
  state.theta = best_ekf->x(3);
  state.omega = (config_.use_fixed_omega && direction_ != 0) ?
                direction_ * config_.known_omega : best_ekf->x(4);
  state.direction = direction_;
  state.valid = true;
  state.covariance_diag = best_ekf->P.diagonal();
  
  return state;
}

Eigen::VectorXd OutpostPredictorV2::getStateVector() const
{
  ExtendedKalmanFilter* best_ekf = getBestEKF();
  if (!initialized_ || !best_ekf) {
    return Eigen::VectorXd::Zero(STATE_DIM);
  }
  return best_ekf->x;
}

Eigen::MatrixXd OutpostPredictorV2::getCovarianceMatrix() const
{
  ExtendedKalmanFilter* best_ekf = getBestEKF();
  if (!initialized_ || !best_ekf) {
    return Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  }
  return best_ekf->P;
}

MultiHypothesisState OutpostPredictorV2::getMultiHypothesisState() const
{
  MultiHypothesisState mhs;
  mhs.converged = converged_;
  mhs.best_hypothesis_id = getBestHypothesisId();
  
  for (int i = 0; i < 3; ++i) {
    mhs.confidences[i] = hypotheses_[i].confidence;
  }
  
  mhs.determined_height = static_cast<ArmorHeight>(mhs.best_hypothesis_id);
  return mhs;
}

bool OutpostPredictorV2::isConverged() const
{
  return converged_;
}

ArmorHeight OutpostPredictorV2::getDeterminedHeight() const
{
  return static_cast<ArmorHeight>(getBestHypothesisId());
}

// ============================================================================
// 预测功能
// ============================================================================

Eigen::VectorXd OutpostPredictorV2::predictState(
  const Eigen::VectorXd& current_state, double dt) const
{
  Eigen::VectorXd future_state = current_state;
  
  // θ += ω * dt
  future_state(3) = normalizeAngle(current_state(3) + current_state(4) * dt);
  
  // 中心位置不变
  return future_state;
}

std::array<ArmorPrediction, 3> OutpostPredictorV2::predictArmors(double dt) const
{
  std::array<ArmorPrediction, 3> predictions;
  
  ExtendedKalmanFilter* best_ekf = getBestEKF();
  if (!initialized_ || !best_ekf) {
    return predictions;
  }
  
  // 调试：打印当前状态
  static int pred_count = 0;
  if (++pred_count % 30 == 0) {  // 每30次打印一次
    double theta_deg = best_ekf->x(3) * 180.0 / M_PI;
    double omega_deg = best_ekf->x(4) * 180.0 / M_PI;
    // std::cout << "[State] theta=" << theta_deg << "° omega=" << omega_deg 
    //           << "°/s converged=" << (converged_ ? "YES" : "NO");
    // std::cout << std::endl;
    if (!converged_) {
        std::cout << " conf[H/M/L]=" << hypotheses_[0].confidence 
                << "/" << hypotheses_[1].confidence 
                << "/" << hypotheses_[2].confidence;
        std::cout << std::endl;
    }
    
  }
  
  // 预测未来状态
  Eigen::VectorXd future_state = predictState(best_ekf->x, dt);
  
  for (int i = 0; i < 3; ++i) {
    predictions[i].armor_id = i;
    predictions[i].position = computeArmorPosition(future_state, i);
    predictions[i].orientation = computeArmorOrientation(future_state, i);
    
    // 计算距离
    predictions[i].distance = predictions[i].position.norm();
    
    // 计算相对于正面的角度偏差
    // 当装甲板正对相机时,朝向角应该接近π (或-π)
    double facing_camera_angle = M_PI;
    predictions[i].orientation_error = std::abs(
      angleDiff(predictions[i].orientation, facing_camera_angle));
    
    // 判断是否可打
    predictions[i].shootable = 
      (predictions[i].orientation_error < config_.max_orientation_angle);
  }
  
  return predictions;
}

std::array<ShootableWindow, 3> OutpostPredictorV2::computeShootableWindows() const
{
  std::array<ShootableWindow, 3> windows;
  
  ExtendedKalmanFilter* best_ekf = getBestEKF();
  if (!initialized_ || !best_ekf) {
    return windows;
  }
  
  double omega = (config_.use_fixed_omega && direction_ != 0) ?
                 direction_ * config_.known_omega : best_ekf->x(4);
  
  if (std::abs(omega) < config_.omega_threshold) {
    // 静止状态: 检查当前是否可打
    auto current_predictions = predictArmors(0.0);
    for (int i = 0; i < 3; ++i) {
      windows[i].armor_id = i;
      if (current_predictions[i].shootable) {
        windows[i].enter_time = 0.0;
        windows[i].exit_time = 1e6;  // 无限长
        windows[i].valid = true;
      } else {
        windows[i].valid = false;
      }
    }
    return windows;
  }
  
  // 旋转状态: 计算进入和离开时间
  double T = 2.0 * M_PI / std::abs(omega);  // 旋转周期
  double max_angle = config_.max_orientation_angle;
  
  for (int i = 0; i < 3; ++i) {
    windows[i].armor_id = i;
    
    // 当前装甲板的朝向角
    double theta_armor = computeArmorOrientation(best_ekf->x, i);
    
    // 计算当前角度与"正对相机"的差距
    // 正对相机时 orientation = π
    double current_error = angleDiff(theta_armor, M_PI);
    
    // 计算进入可打区域的时间
    // 需要转过多少角度才能进入: |current_error| - max_angle
    // 需要转过多少角度才能离开: |current_error| + max_angle
    
    if (std::abs(current_error) <= max_angle) {
      // 当前已经在可打区域内
      windows[i].enter_time = 0.0;
      double angle_to_exit = max_angle - current_error * (omega > 0 ? 1 : -1);
      windows[i].exit_time = std::abs(angle_to_exit / omega);
    } else {
      // 当前不在可打区域
      double angle_to_enter = std::abs(current_error) - max_angle;
      windows[i].enter_time = angle_to_enter / std::abs(omega);
      windows[i].exit_time = windows[i].enter_time + 2.0 * max_angle / std::abs(omega);
    }
    
    // 如果进入时间超过一个周期,则在本周期内不可打
    if (windows[i].enter_time > T) {
      windows[i].enter_time -= T;
      windows[i].exit_time -= T;
    }
    
    windows[i].valid = true;
  }
  
  return windows;
}

// ============================================================================
// 瞄准计算 (核心)
// ============================================================================

AimResult OutpostPredictorV2::computeAim(double additional_delay) const
{
  AimResult result;
  
  if (!initialized_ || !ekf_) {
    result.should_shoot = false;
    result.confidence = 0.0;
    return result;
  }
  
  ExtendedKalmanFilter* best_ekf = getBestEKF();
  if (!best_ekf) {
    result.should_shoot = false;
    result.confidence = 0.0;
    return result;
  }
  
  // 计算总延迟 = 系统延迟 + 额外延迟 + 子弹飞行时间估计
  // 子弹飞行时间需要迭代计算,这里先用当前距离估计
  Eigen::Vector3d center(best_ekf->x(0), best_ekf->x(1), best_ekf->x(2));
  double approx_distance = center.norm();
  double bullet_flight_time = approx_distance / config_.bullet_speed;
  double total_delay = config_.system_delay + additional_delay + bullet_flight_time;
  
  // 预测击中时刻的装甲板位置
  auto predictions = predictArmors(total_delay);
  
  // 选择最佳目标
  int target_id = selectTarget(predictions, total_delay);
  
  if (target_id < 0) {
    result.should_shoot = false;
    result.confidence = 0.0;
    result.target_armor_id = -1;
    return result;
  }
  
  // 迭代修正子弹飞行时间
  double new_distance = predictions[target_id].distance;
  bullet_flight_time = new_distance / config_.bullet_speed;
  total_delay = config_.system_delay + additional_delay + bullet_flight_time;
  
  // 用修正后的延迟重新预测
  predictions = predictArmors(total_delay);
  
  // 填充结果
  result.target_armor_id = target_id;
  result.aim_point = predictions[target_id].position;
  result.distance = predictions[target_id].distance;
  result.hit_time = total_delay;
  result.bullet_flight_time = bullet_flight_time;
  
  // 计算yaw和pitch角
  // yaw = atan2(x, y)  (相机坐标系: y朝前, x朝右)
  // pitch = atan2(-z, sqrt(x^2 + y^2))  (z朝上, 所以向下看是负pitch)
  result.aim_yaw = std::atan2(result.aim_point.x(), result.aim_point.y());
  double xy_dist = std::sqrt(
    result.aim_point.x() * result.aim_point.x() + 
    result.aim_point.y() * result.aim_point.y());
  result.aim_pitch = std::atan2(-result.aim_point.z(), xy_dist);
  
  // 判断是否可打
  result.should_shoot = predictions[target_id].shootable;
  
  // 计算置信度 (基于角度误差和协方差)
  double angle_confidence = 1.0 - predictions[target_id].orientation_error / 
                           (config_.max_orientation_angle + 0.1);
  double cov_confidence = 1.0 / (1.0 + best_ekf->P.diagonal().norm());
  
  // 如果还没收敛，降低置信度
  if (!converged_) {
    double hyp_conf = hypotheses_[getBestHypothesisId()].confidence;
    result.confidence = std::clamp(angle_confidence * cov_confidence * hyp_conf, 0.0, 1.0);
  } else {
    result.confidence = std::clamp(angle_confidence * cov_confidence, 0.0, 1.0);
  }
  
  return result;
}

int OutpostPredictorV2::selectTarget(
  const std::array<ArmorPrediction, 3>& predictions, double hit_delay) const
{
  int best_target = -1;
  double best_score = -1e6;
  
  for (int i = 0; i < 3; ++i) {
    if (!predictions[i].shootable) {
      continue;
    }
    
    // 评分: 越正对相机越好,距离越近越好
    double angle_score = 1.0 - predictions[i].orientation_error / M_PI;
    double distance_score = 1.0 / (1.0 + predictions[i].distance);
    
    double score = 0.7 * angle_score + 0.3 * distance_score;
    
    if (score > best_score) {
      best_score = score;
      best_target = i;
    }
  }
  
  return best_target;
}

// ============================================================================
// 辅助函数
// ============================================================================

int OutpostPredictorV2::identifyArmorByHeight(double z_armor, double z_center) const
{
  double h = config_.geometry.height_diff;
  double diff = z_armor - z_center;
  
  if (diff > h * 0.5) {
    return 0;  // 最高的装甲板
  } else if (diff < -h * 0.5) {
    return 2;  // 最低的装甲板
  } else {
    return 1;  // 中间的装甲板
  }
}

void OutpostPredictorV2::setupTransitionMatrix()
{
  F_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  // θ(k+1) = θ(k) + ω(k) * dt
  F_(3, 4) = dt_;
}

void OutpostPredictorV2::setupProcessNoise()
{
  Q_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
  Q_(0, 0) = config_.noise.process_pos * dt_ * dt_;  // x_c
  Q_(1, 1) = config_.noise.process_pos * dt_ * dt_;  // y_c
  Q_(2, 2) = config_.noise.process_pos * dt_ * dt_;  // z_c
  Q_(3, 3) = config_.noise.process_theta * dt_ * dt_;  // theta
  Q_(4, 4) = config_.noise.process_omega * dt_ * dt_;  // omega
}

void OutpostPredictorV2::setupMeasurementNoise()
{
  R_ = Eigen::MatrixXd::Zero(MEAS_DIM, MEAS_DIM);
  R_(0, 0) = config_.noise.measurement_pos;  // x_a
  R_(1, 1) = config_.noise.measurement_pos;  // y_a
  R_(2, 2) = config_.noise.measurement_pos;  // z_a
  R_(3, 3) = config_.noise.measurement_angle;  // θ_armor
}

void OutpostPredictorV2::computeObservationJacobian(int armor_id)
{
  // 观测方程:
  // x_a = x_c + R * sin(θ + k*2π/3)
  // y_a = y_c - R * cos(θ + k*2π/3)
  // z_a = z_c + h_k
  // θ_armor = θ + k*2π/3  (数学逆时针排列)
  //
  // 雅可比矩阵 H = ∂h/∂X:
  // ∂x_a/∂x_c = 1, ∂x_a/∂θ = R*cos(θ + k*2π/3)
  // ∂y_a/∂y_c = 1, ∂y_a/∂θ = R*sin(θ + k*2π/3)
  // ∂z_a/∂z_c = 1
  // ∂θ_armor/∂θ = 1
  
  // 注意: 这里使用的theta需要从外部传入或从最佳EKF获取
  ExtendedKalmanFilter* best_ekf = getBestEKF();
  double theta = best_ekf ? best_ekf->x(3) : 0.0;
  double theta_armor = theta + armor_id * ARMOR_ANGLE_INTERVAL;  // 数学逆时针递增
  double R = config_.geometry.radius;
  
  H_ = Eigen::MatrixXd::Zero(MEAS_DIM, STATE_DIM);
  
  // ∂h/∂x_c
  H_(0, 0) = 1.0;  // ∂x_a/∂x_c
  
  // ∂h/∂y_c
  H_(1, 1) = 1.0;  // ∂y_a/∂y_c
  
  // ∂h/∂z_c
  H_(2, 2) = 1.0;  // ∂z_a/∂z_c
  
  // ∂h/∂θ
  H_(0, 3) = R * std::cos(theta_armor);   // ∂x_a/∂θ
  H_(1, 3) = R * std::sin(theta_armor);   // ∂y_a/∂θ
  H_(3, 3) = 1.0;                          // ∂θ_armor/∂θ
}

Eigen::VectorXd OutpostPredictorV2::observationFunction(
  const Eigen::VectorXd& state, int armor_id) const
{
  double x_c = state(0);
  double y_c = state(1);
  double z_c = state(2);
  double theta = state(3);
  
  double R = config_.geometry.radius;
  auto height_offsets = config_.geometry.getHeightOffsets();
  
  // 装甲板按逆时针排列匹配前哨站CCW旋转: HIGH(0°) → MIDDLE(+120°) → LOW(+240°)
  // 使用加法实现逆时针递增
  double theta_armor = theta + armor_id * ARMOR_ANGLE_INTERVAL;
  
  Eigen::VectorXd z(MEAS_DIM);
  z(0) = x_c + R * std::sin(theta_armor);
  z(1) = y_c - R * std::cos(theta_armor);
  z(2) = z_c + height_offsets[armor_id];
  z(3) = normalizeAngle(theta_armor);
  
  return z;
}

Eigen::Vector3d OutpostPredictorV2::computeArmorPosition(
  const Eigen::VectorXd& state, int armor_id) const
{
  double x_c = state(0);
  double y_c = state(1);
  double z_c = state(2);
  double theta = state(3);
  
  double R = config_.geometry.radius;
  auto height_offsets = config_.geometry.getHeightOffsets();
  
  // 装甲板按逆时针排列 High→Middle→Low（使用加法）
  double theta_armor = theta + armor_id * ARMOR_ANGLE_INTERVAL;
  
  return Eigen::Vector3d(
    x_c + R * std::sin(theta_armor),
    y_c - R * std::cos(theta_armor),
    z_c + height_offsets[armor_id]
  );
}

double OutpostPredictorV2::computeArmorOrientation(
  const Eigen::VectorXd& state, int armor_id) const
{
  double theta = state(3);
  // 装甲板按逆时针排列
  return normalizeAngle(theta + armor_id * ARMOR_ANGLE_INTERVAL);
}

double OutpostPredictorV2::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double OutpostPredictorV2::angleDiff(double a, double b)
{
  return normalizeAngle(a - b);
}

void OutpostPredictorV2::reset()
{
  ekf_.reset();
  for (auto& hyp : hypotheses_) {
    hyp.ekf.reset();
    hyp.confidence = 1.0 / 3.0;
    hyp.num_updates = 0;
    hyp.last_prediction_error = 0.0;
  }
  converged_ = false;
  best_hypothesis_id_ = -1;
  initialized_ = false;
  direction_ = 0;
  last_theta_ = 0.0;
  theta_change_sum_ = 0.0;
  direction_samples_ = 0;
  last_chi_square_ = 0.0;
  last_obs_timestamp_ = 0.0;
  
  std::cout << "[OutpostPredictorV2] 已重置" << std::endl;
}

double OutpostPredictorV2::computeNextWindowTime() const
{
  if (!initialized_) {
    return -1.0;
  }
  
  // 获取当前状态
  auto* best_ekf = getBestEKF();
  if (!best_ekf) {
    return -1.0;
  }
  
  double omega = best_ekf->x(4);
  if (std::abs(omega) < 0.01) {
    return -1.0;  // 不旋转时无法计算窗口
  }
  
  double theta = best_ekf->x(3);
  double max_angle = config_.max_orientation_angle;
  
  // 检查每个装甲板的下一个可打窗口
  double min_wait = 1e9;
  
  for (int i = 0; i < 3; ++i) {
    double armor_theta = theta + i * ARMOR_ANGLE_INTERVAL;
    armor_theta = normalizeAngle(armor_theta);
    
    // 计算该装甲板到达正对相机（theta=0）的时间
    double angle_to_front = normalizeAngle(-armor_theta);
    
    // 如果omega为正，角度增加；如果为负，角度减少
    double time_to_front;
    if (omega > 0) {
      if (angle_to_front < 0) {
        angle_to_front += 2.0 * M_PI;
      }
      time_to_front = angle_to_front / omega;
    } else {
      if (angle_to_front > 0) {
        angle_to_front -= 2.0 * M_PI;
      }
      time_to_front = angle_to_front / omega;  // omega为负，结果为正
    }
    
    // 考虑可打窗口的提前量
    double window_half_width = max_angle / std::abs(omega);
    double time_to_window = time_to_front - window_half_width;
    
    if (time_to_window < 0) {
      // 当前可能已经在窗口内，检查是否快要离开
      double time_to_exit = time_to_front + window_half_width;
      if (time_to_exit > 0 && std::abs(armor_theta) <= max_angle) {
        return -1.0;  // 当前已可打
      }
      // 需要等到下一圈
      time_to_window += 2.0 * M_PI / std::abs(omega);
    }
    
    if (time_to_window < min_wait) {
      min_wait = time_to_window;
    }
  }
  
  return min_wait < 1e8 ? min_wait : -1.0;
}

double OutpostPredictorV2::getTimeSinceLastObservation() const
{
  if (last_obs_timestamp_ <= 0) {
    return -1.0;
  }
  
  // 获取当前时间（简单实现，使用系统时间）
  auto now = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()).count() / 1000.0;
  
  // 注意：这里假设 last_obs_timestamp_ 是相对同一时间基准的
  // 实际应用中可能需要使用ROS时间
  return 0.0;  // 简化实现，由节点层面处理
}

}  // namespace armor_detector
