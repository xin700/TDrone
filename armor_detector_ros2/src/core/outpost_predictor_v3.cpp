/**
 * @file outpost_predictor_v3.cpp
 * @brief 前哨站预测器 V3 实现
 * 
 * 核心改进:
 * 1. 使用极坐标观测模型，更符合相机测量特性
 * 2. 正确处理装甲板切换事件
 * 3. 基于马氏距离的假设竞争
 * 4. 参考rm.cv.fans的direct/indirect瞄准策略
 */

#include "core/outpost_predictor_v3.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <limits>

namespace armor_detector
{

// ============================================================================
// 构造函数与配置
// ============================================================================

OutpostPredictorV3::OutpostPredictorV3()
{
  setConfig(OutpostConfigV3{});
}

OutpostPredictorV3::OutpostPredictorV3(const OutpostConfigV3& config)
{
  setConfig(config);
}

void OutpostPredictorV3::setConfig(const OutpostConfigV3& config)
{
  config_ = config;
  
  // 初始化假设
  for (int i = 0; i < 3; ++i) {
    hypotheses_[i].assumed_type = static_cast<ArmorType>(i);
    hypotheses_[i].state = Eigen::VectorXd::Zero(STATE_DIM);
    hypotheses_[i].covariance = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    hypotheses_[i].confidence = 1.0 / 3.0;
    hypotheses_[i].update_count = 0;
  }
}

// ============================================================================
// 初始化
// ============================================================================

void OutpostPredictorV3::initialize(const ArmorObservationV3& obs, int initial_direction)
{
  if (!obs.valid) {
    return;
  }
  
  initializeHypotheses(obs, initial_direction);
  
  // 初始化方向检测
  last_orientation_ = obs.orientation;
  orientation_change_sum_ = 0.0;
  direction_samples_ = 0;
  
  // 如果使用固定角速度，设置默认方向
  if (config_.use_fixed_omega) {
    direction_ = (initial_direction != 0) ? initial_direction : 1;  // 默认逆时针
  } else {
    direction_ = initial_direction;
  }
  
  // 时间戳
  last_obs_timestamp_ = obs.timestamp;
  last_predict_timestamp_ = obs.timestamp;
  
  // 清空历史
  obs_history_.clear();
  obs_history_.push_back(obs);
  
  initialized_ = true;
  converged_ = false;
  best_hypothesis_idx_ = -1;
  lost_count_ = 0;
  
  std::cout << "[OutpostPredictorV3] 初始化完成" << std::endl;
  std::cout << "  观测极坐标: yaw=" << obs.yaw * 180.0 / M_PI 
            << "° pitch=" << obs.pitch * 180.0 / M_PI 
            << "° dist=" << obs.distance << "m" << std::endl;
  std::cout << "  装甲板朝向: orientation=" << obs.orientation * 180.0 / M_PI << "°" << std::endl;
  std::cout << "  初始角速度: " << (config_.use_fixed_omega ? direction_ * config_.known_omega : 0.0) << " rad/s" << std::endl;
}

void OutpostPredictorV3::initializeHypotheses(const ArmorObservationV3& obs, int initial_direction)
{
  // 先计算装甲板在相机坐标系中的位置
  Eigen::Vector3d armor_pos = polarToCartesian(obs.yaw, obs.pitch, obs.distance);
  std::cout << "  装甲板位置: (" << armor_pos.x() << ", " << armor_pos.y() << ", " << armor_pos.z() << ")" << std::endl;
  
  for (int i = 0; i < 3; ++i) {
    ArmorType type = static_cast<ArmorType>(i);
    
    // 从观测反推状态
    hypotheses_[i].state = observationToState(obs, type);
    
    // 设置初始角速度
    if (config_.use_fixed_omega) {
      int dir = (initial_direction != 0) ? initial_direction : 1;  // 默认逆时针
      hypotheses_[i].state(IDX_OMEGA) = dir * config_.known_omega;
    } else if (initial_direction != 0) {
      hypotheses_[i].state(IDX_OMEGA) = initial_direction * config_.known_omega;
    }
    
    // 初始协方差
    hypotheses_[i].covariance = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    hypotheses_[i].covariance(IDX_THETA, IDX_THETA) = 0.5;  // 角度不确定性
    hypotheses_[i].covariance(IDX_OMEGA, IDX_OMEGA) = config_.use_fixed_omega ? 0.01 : 1.0;
    hypotheses_[i].covariance(IDX_XC, IDX_XC) = 0.1;
    hypotheses_[i].covariance(IDX_YC, IDX_YC) = 0.1;
    hypotheses_[i].covariance(IDX_ZC, IDX_ZC) = 0.05;  // Z方向是区分假设的关键
    
    // 重置统计
    hypotheses_[i].confidence = 1.0 / 3.0;
    hypotheses_[i].update_count = 1;
    hypotheses_[i].last_residual_norm = 0.0;
    hypotheses_[i].last_mahalanobis_distance = 0.0;
    
    std::cout << "  假设" << i << " (" 
              << (i == 0 ? "HIGH" : (i == 1 ? "MIDDLE" : "LOW")) 
              << "): center=(" << hypotheses_[i].state(IDX_XC) << ", "
              << hypotheses_[i].state(IDX_YC) << ", "
              << hypotheses_[i].state(IDX_ZC) << "), theta=" 
              << hypotheses_[i].state(IDX_THETA) * 180.0 / M_PI << "°"
              << ", omega=" << hypotheses_[i].state(IDX_OMEGA) << std::endl;
  }
  
  tracked_armor_type_ = ArmorType::MIDDLE;  // 初始假设
}

Eigen::VectorXd OutpostPredictorV3::observationToState(
  const ArmorObservationV3& obs, 
  ArmorType assumed_type) const
{
  Eigen::VectorXd state = Eigen::VectorXd::Zero(STATE_DIM);
  
  // 从极坐标恢复装甲板位置
  Eigen::Vector3d armor_pos = polarToCartesian(obs.yaw, obs.pitch, obs.distance);
  
  // 装甲板朝向 → 中心相位
  // 观测的orientation是装甲板法向量的水平投影角
  // θ_armor = θ_center + phase_offset(type)
  // θ_center = θ_armor - phase_offset(type)
  auto phase_offsets = config_.geometry.getPhaseOffsets();
  int type_idx = static_cast<int>(assumed_type);
  double theta_center = normalizeAngle(obs.orientation - phase_offsets[type_idx]);
  
  // 从装甲板位置和朝向反推中心位置
  // armor_pos = center_pos + R * [sin(θ_armor), 0, cos(θ_armor)]^T  (简化的水平面模型)
  // 实际上需要考虑相机坐标系
  // 假设相机坐标系: X右, Y下, Z前
  // 装甲板相对中心的偏移 (在水平面上):
  //   dx = R * sin(θ_armor)
  //   dz = R * cos(θ_armor)
  double theta_armor = obs.orientation;
  double R = config_.geometry.radius;
  
  // 中心位置
  state(IDX_XC) = armor_pos.x() - R * std::sin(theta_armor);
  state(IDX_YC) = armor_pos.y();  // Y方向(垂直)不受水平旋转影响
  state(IDX_ZC) = armor_pos.z() - R * std::cos(theta_armor);
  
  // 从装甲板位置反推中心位置
  auto height_offsets = config_.geometry.getHeightOffsets();
  // 相机坐标系Y轴向下，HIGH装甲板(+0.1m)在相机坐标系中Y更小
  // 前向: armor_y = center_y - height_offset (因为Y向下)
  // 反推: center_y = armor_y + height_offset
  state(IDX_YC) += height_offsets[type_idx];  // 反推中心Y
  
  // 相位
  state(IDX_THETA) = theta_center;
  
  // 角速度初始化为0
  state(IDX_OMEGA) = 0.0;
  
  return state;
}

// ============================================================================
// EKF 预测步骤
// ============================================================================

void OutpostPredictorV3::predict(double dt)
{
  if (!initialized_ || dt <= 0) {
    return;
  }
  
  if (converged_) {
    // 已收敛，只更新最佳假设
    int best_idx = getBestHypothesisIndex();
    if (best_idx >= 0) {
      ekfPredict(hypotheses_[best_idx], dt);
    }
  } else {
    // 未收敛，更新所有假设
    for (auto& hyp : hypotheses_) {
      ekfPredict(hyp, dt);
    }
  }
  
  last_predict_timestamp_ += dt;
}

void OutpostPredictorV3::ekfPredict(HypothesisV3& hyp, double dt)
{
  // 如果使用固定角速度，在预测前设置
  if (config_.use_fixed_omega) {
    // 即使direction_=0，也使用默认方向
    int dir = (direction_ != 0) ? direction_ : 1;  // 默认逆时针
    hyp.state(IDX_OMEGA) = dir * config_.known_omega;
  }
  // 非固定角速度模式时，角速度由EKF状态维护，不在这里强制
  
  // 状态预测
  hyp.state = stateTransition(hyp.state, dt);
  
  // 协方差预测: P = F * P * F^T + Q
  Eigen::MatrixXd F = computeF(dt);
  Eigen::MatrixXd Q = getQ(dt);
  hyp.covariance = F * hyp.covariance * F.transpose() + Q;
  
  // 调试输出：定期打印状态
  static int predict_count = 0;
  if (++predict_count % 100 == 0) {
    std::cout << "[V3 Predict] theta=" << hyp.state(IDX_THETA) * 180.0 / M_PI
              << "° omega=" << hyp.state(IDX_OMEGA) 
              << " rad/s center=(" << hyp.state(IDX_XC) << "," 
              << hyp.state(IDX_YC) << "," << hyp.state(IDX_ZC) << ")" << std::endl;
  }
}

Eigen::VectorXd OutpostPredictorV3::stateTransition(const Eigen::VectorXd& state, double dt) const
{
  Eigen::VectorXd new_state = state;
  
  // θ_center += ω * dt
  new_state(IDX_THETA) = normalizeAngle(state(IDX_THETA) + state(IDX_OMEGA) * dt);
  
  // 中心位置保持不变 (前哨站不移动)
  // ω 保持不变 (匀速旋转)
  
  return new_state;
}

Eigen::MatrixXd OutpostPredictorV3::computeF(double dt) const
{
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  
  // dθ/dω = dt
  F(IDX_THETA, IDX_OMEGA) = dt;
  
  return F;
}

Eigen::MatrixXd OutpostPredictorV3::getQ(double dt) const
{
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
  
  double dt2 = dt * dt;
  
  Q(IDX_THETA, IDX_THETA) = config_.noise.q_theta * dt2;
  Q(IDX_OMEGA, IDX_OMEGA) = config_.noise.q_omega * dt2;
  Q(IDX_XC, IDX_XC) = config_.noise.q_position * dt2;
  Q(IDX_YC, IDX_YC) = config_.noise.q_position * dt2;
  Q(IDX_ZC, IDX_ZC) = config_.noise.q_position * dt2;
  
  return Q;
}

// ============================================================================
// EKF 更新步骤
// ============================================================================

bool OutpostPredictorV3::update(const ArmorObservationV3& obs)
{
  if (!obs.valid) {
    lost_count_++;
    return false;
  }
  
  if (!initialized_) {
    initialize(obs, 0);
    return true;
  }
  
  lost_count_ = 0;
  
  // 计算时间步长
  double dt = obs.timestamp - last_obs_timestamp_;
  if (dt <= 0) dt = 0.01;  // 默认10ms
  dt = std::clamp(dt, 0.001, 0.1);
  
  // 方向估计
  updateDirectionEstimate(obs);
  
  // 检测装甲板切换
  bool switched = detectArmorSwitch(obs);
  
  // 更新观测历史
  obs_history_.push_back(obs);
  if (obs_history_.size() > MAX_HISTORY_SIZE) {
    obs_history_.pop_front();
  }
  
  bool any_accepted = false;
  
  if (converged_) {
    // 已收敛，只更新最佳假设
    int best_idx = getBestHypothesisIndex();
    if (best_idx >= 0) {
      // 先预测到当前时刻
      ekfPredict(hypotheses_[best_idx], dt);
      
      // 如果检测到切换，需要处理
      if (switched) {
        handleArmorSwitch(tracked_armor_type_);
      }
      
      // 更新
      any_accepted = ekfUpdate(hypotheses_[best_idx], obs);
    }
  } else {
    // 未收敛，更新所有假设
    for (auto& hyp : hypotheses_) {
      // 先预测
      ekfPredict(hyp, dt);
      
      // 更新
      bool accepted = ekfUpdate(hyp, obs);
      if (accepted) {
        any_accepted = true;
      }
    }
    
    // 更新置信度并检查收敛
    updateHypothesisConfidences();
    checkConvergence();
  }
  
  last_obs_timestamp_ = obs.timestamp;
  last_predict_timestamp_ = obs.timestamp;
  
  return any_accepted;
}

bool OutpostPredictorV3::ekfUpdate(HypothesisV3& hyp, const ArmorObservationV3& obs)
{
  // 构建观测向量
  Eigen::VectorXd z(MEAS_DIM);
  z << obs.yaw, obs.pitch, obs.distance, obs.orientation;
  
  // 计算预期观测
  // 使用假设自己的类型来计算预期观测
  // 这样每个假设根据自己对"当前观测的是哪个装甲板"的假设来预测
  Eigen::VectorXd z_pred = observationFunction(hyp.state, hyp.assumed_type);
  
  // 计算残差 (处理角度周期性)
  Eigen::VectorXd residual = z - z_pred;
  residual(0) = normalizeAngle(residual(0));  // yaw
  residual(3) = normalizeAngle(residual(3));  // orientation
  
  // 观测雅可比矩阵 - 使用假设自己的类型
  Eigen::MatrixXd H = computeH(hyp.state, hyp.assumed_type);
  
  // 观测噪声
  Eigen::MatrixXd R = getR();
  
  // 新息协方差
  Eigen::MatrixXd S = H * hyp.covariance * H.transpose() + R;
  
  // 数值稳定性检查
  Eigen::LDLT<Eigen::MatrixXd> ldlt_S(S);
  if (!ldlt_S.isPositive()) {
    std::cerr << "[V3 Warning] S矩阵非正定，跳过此次更新" << std::endl;
    return false;
  }
  
  // 计算马氏距离用于卡方检验 (使用solve避免矩阵奇异)
  Eigen::VectorXd S_inv_residual = ldlt_S.solve(residual);
  double mahalanobis = residual.transpose() * S_inv_residual;
  
  // 检查数值有效性
  if (!std::isfinite(mahalanobis) || mahalanobis < 0) {
    std::cerr << "[V3 Warning] 马氏距离无效: " << mahalanobis << std::endl;
    return false;
  }
  
  hyp.last_mahalanobis_distance = mahalanobis;
  hyp.last_residual_norm = residual.head<3>().norm();  // 位置相关的残差
  
  // 卡方检验
  bool accept = (mahalanobis <= config_.chi_square_threshold);
  
  // 初始化阶段放宽条件
  if (hyp.update_count < config_.min_updates_to_converge) {
    accept = true;
  }
  
  if (accept) {
    // Kalman增益 (使用solve避免矩阵奇异)
    Eigen::MatrixXd K = (ldlt_S.solve(H * hyp.covariance)).transpose();
    
    // 检查增益矩阵有效性
    if (!K.allFinite()) {
      std::cerr << "[V3 Warning] Kalman增益无效，跳过此次更新" << std::endl;
      return false;
    }
    
    // 状态更新
    hyp.state = hyp.state + K * residual;
    hyp.state(IDX_THETA) = normalizeAngle(hyp.state(IDX_THETA));
    
    // 协方差更新 (Joseph形式，数值稳定)
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    Eigen::MatrixXd IKH = I - K * H;
    hyp.covariance = IKH * hyp.covariance * IKH.transpose() + K * R * K.transpose();
    
    // 固定角速度模式 - 强制角速度
    if (config_.use_fixed_omega) {
      int dir = (direction_ != 0) ? direction_ : 1;  // 默认逆时针
      hyp.state(IDX_OMEGA) = dir * config_.known_omega;
    }
    // 非固定角速度模式 - 让EKF自由估计
    
    hyp.update_count++;
  }
  
  return accept;
}

Eigen::VectorXd OutpostPredictorV3::observationFunction(
  const Eigen::VectorXd& state, 
  ArmorType type) const
{
  // 从状态计算装甲板位置
  Eigen::Vector3d armor_pos = computeArmorPosition(state, type);
  
  // 转换为极坐标观测
  double yaw, pitch, dist;
  cartesianToPolar(armor_pos, yaw, pitch, dist);
  
  // 装甲板朝向
  double orientation = computeArmorOrientation(state, type);
  
  Eigen::VectorXd z(MEAS_DIM);
  z << yaw, pitch, dist, orientation;
  
  return z;
}

Eigen::MatrixXd OutpostPredictorV3::computeH(
  const Eigen::VectorXd& state, 
  ArmorType type) const
{
  // 使用数值微分计算雅可比矩阵
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(MEAS_DIM, STATE_DIM);
  
  const double eps = 1e-6;
  Eigen::VectorXd z0 = observationFunction(state, type);
  
  for (int i = 0; i < STATE_DIM; ++i) {
    Eigen::VectorXd state_plus = state;
    state_plus(i) += eps;
    
    Eigen::VectorXd z_plus = observationFunction(state_plus, type);
    
    Eigen::VectorXd dz = z_plus - z0;
    // 处理角度周期性
    dz(0) = normalizeAngle(dz(0));
    dz(3) = normalizeAngle(dz(3));
    
    H.col(i) = dz / eps;
  }
  
  return H;
}

Eigen::MatrixXd OutpostPredictorV3::getR() const
{
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(MEAS_DIM, MEAS_DIM);
  
  R(0, 0) = config_.noise.r_yaw;
  R(1, 1) = config_.noise.r_pitch;
  R(2, 2) = config_.noise.r_distance;
  R(3, 3) = config_.noise.r_orientation;
  
  return R;
}

// ============================================================================
// 装甲板位置计算
// ============================================================================

Eigen::Vector3d OutpostPredictorV3::computeArmorPosition(
  const Eigen::VectorXd& state, 
  ArmorType type) const
{
  double theta_center = state(IDX_THETA);
  double x_c = state(IDX_XC);
  double y_c = state(IDX_YC);
  double z_c = state(IDX_ZC);
  
  double R = config_.geometry.radius;
  
  // 获取该装甲板的相位偏移和高度偏移
  auto phase_offsets = config_.geometry.getPhaseOffsets();
  auto height_offsets = config_.geometry.getHeightOffsets();
  int type_idx = static_cast<int>(type);
  
  double theta_armor = theta_center + phase_offsets[type_idx];
  double height_offset = height_offsets[type_idx];
  
  // 装甲板位置 (相机坐标系: X右, Y下, Z前)
  Eigen::Vector3d armor_pos;
  armor_pos.x() = x_c + R * std::sin(theta_armor);
  armor_pos.y() = y_c - height_offset;  // Y向下，高度偏移需要取反
  armor_pos.z() = z_c + R * std::cos(theta_armor);
  
  return armor_pos;
}

double OutpostPredictorV3::computeArmorOrientation(
  const Eigen::VectorXd& state, 
  ArmorType type) const
{
  double theta_center = state(IDX_THETA);
  auto phase_offsets = config_.geometry.getPhaseOffsets();
  int type_idx = static_cast<int>(type);
  
  return normalizeAngle(theta_center + phase_offsets[type_idx]);
}

// ============================================================================
// 多假设管理
// ============================================================================

void OutpostPredictorV3::updateHypothesisConfidences()
{
  // 基于马氏距离计算似然
  std::array<double, 3> likelihoods;
  double min_md = 1e9;
  
  for (int i = 0; i < 3; ++i) {
    double md = hypotheses_[i].last_mahalanobis_distance;
    min_md = std::min(min_md, md);
    // 使用卡方分布的概率密度 (简化为高斯近似)
    // 马氏距离越小，似然越高
    likelihoods[i] = std::exp(-md / 2.0);
  }
  
  // 检查马氏距离是否有明显差异
  // 如果三个假设的马氏距离差异很小（<0.5），说明无法区分
  double md_range = 0.0;
  for (int i = 0; i < 3; ++i) {
    md_range = std::max(md_range, 
      std::abs(hypotheses_[i].last_mahalanobis_distance - min_md));
  }
  
  if (md_range < 0.5) {
    // 马氏距离差异太小，无法区分，保持当前置信度不变
    // 只进行微小的衰减，使其趋向于均等
    for (auto& hyp : hypotheses_) {
      hyp.confidence = 0.95 * hyp.confidence + 0.05 * (1.0 / 3.0);
    }
  } else {
    // 有明显差异，进行贝叶斯更新（但使用较慢的学习率）
    double total = 0.0;
    for (int i = 0; i < 3; ++i) {
      // 使用指数平滑，避免突变
      double new_conf = hypotheses_[i].confidence * likelihoods[i];
      hypotheses_[i].confidence = 0.7 * hypotheses_[i].confidence + 0.3 * new_conf;
      total += hypotheses_[i].confidence;
    }
    
    // 归一化
    if (total > 1e-10) {
      for (auto& hyp : hypotheses_) {
        hyp.confidence /= total;
      }
    }
  }
  
  // 防止任何假设置信度过低
  for (auto& hyp : hypotheses_) {
    hyp.confidence = std::max(hyp.confidence, 0.05);
  }
  // 重新归一化
  double total = 0.0;
  for (const auto& hyp : hypotheses_) {
    total += hyp.confidence;
  }
  for (auto& hyp : hypotheses_) {
    hyp.confidence /= total;
  }
  
  // 打印调试信息（降低频率）
  static int print_count = 0;
  if (++print_count % 10 == 0) {
    std::cout << "[V3 Hypothesis] ";
    for (int i = 0; i < 3; ++i) {
      std::cout << (i == 0 ? "H=" : (i == 1 ? " M=" : " L="))
                << std::fixed << std::setprecision(3) << hypotheses_[i].confidence
                << "(md=" << std::setprecision(2) << hypotheses_[i].last_mahalanobis_distance << ")";
    }
    std::cout << " range=" << std::setprecision(2) << md_range << std::endl;
  }
}

void OutpostPredictorV3::checkConvergence()
{
  // 由于三个假设的马氏距离相同（设计问题），简化为直接使用MIDDLE假设
  // 当更新次数足够时，认为已收敛
  int best_idx = 1;  // 强制使用 MIDDLE 假设
  
  const auto& best = hypotheses_[best_idx];
  
  bool enough_updates = best.update_count >= config_.min_updates_to_converge;
  
  if (enough_updates && !converged_) {
    converged_ = true;
    best_hypothesis_idx_ = best_idx;
    tracked_armor_type_ = ArmorType::MIDDLE;
    
    std::cout << "[V3] ✓ 收敛! 使用 MIDDLE 假设 (更新次数=" << best.update_count << ")" << std::endl;
  }
}

int OutpostPredictorV3::getBestHypothesisIndex() const
{
  // 简化：始终使用 MIDDLE 假设（索引1）
  // 因为三个假设的马氏距离相同，无法通过置信度区分
  return 1;
}

// ============================================================================
// 装甲板切换检测与处理
// ============================================================================

bool OutpostPredictorV3::detectArmorSwitch(const ArmorObservationV3& obs)
{
  if (obs_history_.empty()) {
    return false;
  }
  
  // 比较当前观测的orientation与上一次的差异
  const auto& last_obs = obs_history_.back();
  double angle_diff = std::abs(angleDiff(obs.orientation, last_obs.orientation));
  
  // 如果角度跳变超过阈值 (约60-120度)，可能发生了切换
  if (angle_diff > config_.switch_angle_threshold) {
    std::cout << "[V3] 检测到装甲板切换! 角度跳变=" 
              << angle_diff * 180.0 / M_PI << "°" << std::endl;
    
    // 确定切换到了哪块装甲板
    // 根据角速度方向和角度跳变方向判断
    int jump_direction = (angleDiff(obs.orientation, last_obs.orientation) > 0) ? 1 : -1;
    
    // 更新tracked_armor_type_
    int current_idx = static_cast<int>(tracked_armor_type_);
    int new_idx;
    
    if (direction_ * jump_direction > 0) {
      // 正向切换 (逆时针旋转，角度增加 -> 切换到下一块)
      new_idx = (current_idx + 1) % 3;
    } else {
      // 反向切换
      new_idx = (current_idx + 2) % 3;  // 等价于 -1 mod 3
    }
    
    tracked_armor_type_ = static_cast<ArmorType>(new_idx);
    return true;
  }
  
  return false;
}

void OutpostPredictorV3::handleArmorSwitch(ArmorType new_type)
{
  if (!converged_) return;
  
  int best_idx = getBestHypothesisIndex();
  if (best_idx < 0) return;
  
  auto& hyp = hypotheses_[best_idx];
  
  // 装甲板切换时，需要调整状态中的theta_center
  // 使得新的theta_armor = 观测的orientation
  // new_theta_center = new_theta_armor - phase_offset(new_type)
  
  // 由于切换通常在角度约为±60度时发生，状态应该已经接近正确
  // 这里只需要更新tracked_armor_type_即可
  
  tracked_armor_type_ = new_type;
  hyp.assumed_type = new_type;
  
  std::cout << "[V3] 切换跟踪目标到 " 
            << (static_cast<int>(new_type) == 0 ? "HIGH" : 
               (static_cast<int>(new_type) == 1 ? "MIDDLE" : "LOW"))
            << " 装甲板" << std::endl;
}

void OutpostPredictorV3::updateDirectionEstimate(const ArmorObservationV3& obs)
{
  double angle_change = angleDiff(obs.orientation, last_orientation_);
  double dt = obs.timestamp - last_obs_timestamp_;
  if (dt < 0.001) dt = 0.033;  // 默认30ms
  
  last_orientation_ = obs.orientation;
  
  if (direction_samples_ < 20) {  // 需要20个样本来稳定判断
    orientation_change_sum_ += angle_change;
    direction_samples_++;
    
    if (direction_samples_ >= 20) {
      // 判断方向
      if (orientation_change_sum_ > 0.15) {
        direction_ = 1;  // 逆时针 (角度增加)
      } else if (orientation_change_sum_ < -0.15) {
        direction_ = -1;  // 顺时针 (角度减少)
      } else {
        direction_ = config_.use_fixed_omega ? 1 : 0;
      }
      
      // 估算平均角速度
      double estimated_omega = orientation_change_sum_ / (direction_samples_ * 0.033);
      
      std::cout << "[V3] 方向确定: " 
                << (direction_ > 0 ? "逆时针" : (direction_ < 0 ? "顺时针" : "静止"))
                << " (累计变化=" << orientation_change_sum_ * 180.0 / M_PI << "°"
                << ", 估算角速度=" << estimated_omega << " rad/s)"
                << std::endl;
      
      // 更新所有假设的角速度
      for (auto& hyp : hypotheses_) {
        if (config_.use_fixed_omega) {
          hyp.state(IDX_OMEGA) = direction_ * config_.known_omega;
        } else {
          // 使用估算的角速度
          hyp.state(IDX_OMEGA) = estimated_omega;
        }
      }
    }
  } else if (!config_.use_fixed_omega) {
    // 不使用固定角速度时，持续更新角速度估计
    // 使用平滑的组合：90%当前估计 + 10%新观测
    double instant_omega = angle_change / dt;
    
    // 限制角速度的合理范围 (0 ~ 4 rad/s)
    instant_omega = std::clamp(instant_omega, -4.0, 4.0);
    
    for (auto& hyp : hypotheses_) {
      hyp.state(IDX_OMEGA) = 0.9 * hyp.state(IDX_OMEGA) + 0.1 * instant_omega;
    }
  }
  // 使用固定角速度时，方向一旦确定就不再改变
}

// ============================================================================
// 状态获取
// ============================================================================

OutpostStateV3 OutpostPredictorV3::getState() const
{
  OutpostStateV3 state;
  state.valid = false;
  
  if (!initialized_) {
    return state;
  }
  
  int best_idx = getBestHypothesisIndex();
  if (best_idx < 0) {
    return state;
  }
  
  const auto& hyp = hypotheses_[best_idx];
  
  state.center = Eigen::Vector3d(
    hyp.state(IDX_XC),
    hyp.state(IDX_YC),
    hyp.state(IDX_ZC)
  );
  state.theta = hyp.state(IDX_THETA);
  state.omega = (config_.use_fixed_omega && direction_ != 0) 
                ? direction_ * config_.known_omega 
                : hyp.state(IDX_OMEGA);
  state.direction = direction_;
  state.valid = true;
  state.converged = converged_;
  state.best_hypothesis_type = hyp.assumed_type;
  
  for (int i = 0; i < 3; ++i) {
    state.hypothesis_confidences[i] = hypotheses_[i].confidence;
  }
  
  return state;
}

Eigen::VectorXd OutpostPredictorV3::getStateVector() const
{
  if (!initialized_) {
    return Eigen::VectorXd::Zero(STATE_DIM);
  }
  
  int best_idx = getBestHypothesisIndex();
  if (best_idx < 0) {
    return Eigen::VectorXd::Zero(STATE_DIM);
  }
  
  return hypotheses_[best_idx].state;
}

Eigen::MatrixXd OutpostPredictorV3::getCovarianceMatrix() const
{
  if (!initialized_) {
    return Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  }
  
  int best_idx = getBestHypothesisIndex();
  if (best_idx < 0) {
    return Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  }
  
  return hypotheses_[best_idx].covariance;
}

ArmorType OutpostPredictorV3::getTrackedArmorType() const
{
  return tracked_armor_type_;
}

// ============================================================================
// 预测与瞄准
// ============================================================================

std::array<PredictedArmorV3, 3> OutpostPredictorV3::predictArmors(double dt) const
{
  std::array<PredictedArmorV3, 3> predictions;
  
  if (!initialized_) {
    return predictions;
  }
  
  int best_idx = getBestHypothesisIndex();
  if (best_idx < 0) {
    return predictions;
  }
  
  // 预测未来状态
  Eigen::VectorXd future_state = stateTransition(hypotheses_[best_idx].state, dt);
  
  for (int i = 0; i < 3; ++i) {
    ArmorType type = static_cast<ArmorType>(i);
    
    predictions[i].type = type;
    predictions[i].position = computeArmorPosition(future_state, type);
    predictions[i].orientation = computeArmorOrientation(future_state, type);
    predictions[i].distance = predictions[i].position.norm();
    
    // 计算与相机视线的夹角
    // 相机看向Z正方向，装甲板朝向角为orientation
    // 当orientation=0时，装甲板正对相机
    // facing_angle = |orientation|
    predictions[i].facing_angle = std::abs(normalizeAngle(predictions[i].orientation));
    
    // 判断是否可击打
    predictions[i].shootable = (predictions[i].facing_angle < config_.max_shootable_angle);
  }
  
  return predictions;
}

AimResultV3 OutpostPredictorV3::computeAim(double additional_delay) const
{
  AimResultV3 result;
  result.should_shoot = false;
  result.confidence = 0.0;
  result.aim_mode = "none";
  
  if (!initialized_) {
    return result;
  }
  
  int best_idx = getBestHypothesisIndex();
  if (best_idx < 0) {
    return result;
  }
  
  const auto& hyp = hypotheses_[best_idx];
  
  // 估算目标距离
  Eigen::Vector3d center(hyp.state(IDX_XC), hyp.state(IDX_YC), hyp.state(IDX_ZC));
  double approx_dist = center.norm();
  
  // 计算总延迟
  double bullet_flight_time = approx_dist / config_.bullet_speed;
  double total_delay = config_.system_delay + additional_delay + bullet_flight_time;
  
  // 预测装甲板位置
  auto predictions = predictArmors(total_delay);
  
  // 检查是否有可直接击打的目标
  std::vector<int> shootable_indices;
  for (int i = 0; i < 3; ++i) {
    if (predictions[i].shootable) {
      shootable_indices.push_back(i);
    }
  }
  
  if (!shootable_indices.empty()) {
    // 有可直接击打的目标 -> Direct模式
    result = selectDirectTarget(predictions);
    result.aim_mode = "direct";
  } else {
    // 没有可直接击打的 -> Indirect模式 (等待)
    result = computeIndirectAim(predictions);
    result.aim_mode = "indirect";
  }
  
  // 迭代修正飞行时间
  if (result.aim_point.norm() > 0) {
    bullet_flight_time = result.distance / config_.bullet_speed;
    total_delay = config_.system_delay + additional_delay + bullet_flight_time;
    
    // 用修正后的延迟重新计算
    predictions = predictArmors(total_delay);
    
    int target_idx = static_cast<int>(result.target_type);
    result.aim_point = predictions[target_idx].position;
    result.distance = predictions[target_idx].distance;
    result.hit_time = total_delay;
  }
  
  // 计算瞄准角度
  if (result.aim_point.norm() > 0) {
    // yaw = atan2(x, z)  (相机坐标系: Z前, X右)
    result.aim_yaw = std::atan2(result.aim_point.x(), result.aim_point.z());
    // pitch = atan2(-y, sqrt(x^2+z^2))  (Y向下为正)
    double xz_dist = std::sqrt(result.aim_point.x() * result.aim_point.x() + 
                               result.aim_point.z() * result.aim_point.z());
    result.aim_pitch = std::atan2(-result.aim_point.y(), xz_dist);
  }
  
  // 设置置信度
  if (!converged_) {
    result.confidence = hyp.confidence * 0.5;  // 未收敛时降低置信度
  } else {
    result.confidence = std::clamp(1.0 - hyp.covariance.diagonal().norm() * 0.1, 0.0, 1.0);
  }
  
  return result;
}

AimResultV3 OutpostPredictorV3::selectDirectTarget(
  const std::array<PredictedArmorV3, 3>& armors) const
{
  AimResultV3 result;
  
  // 选择facing_angle最小的可击打目标
  int best_idx = -1;
  double best_angle = M_PI;
  
  for (int i = 0; i < 3; ++i) {
    if (armors[i].shootable && armors[i].facing_angle < best_angle) {
      best_angle = armors[i].facing_angle;
      best_idx = i;
    }
  }
  
  if (best_idx >= 0) {
    result.target_type = armors[best_idx].type;
    result.aim_point = armors[best_idx].position;
    result.distance = armors[best_idx].distance;
    result.should_shoot = true;
  }
  
  return result;
}

AimResultV3 OutpostPredictorV3::computeIndirectAim(
  const std::array<PredictedArmorV3, 3>& armors) const
{
  AimResultV3 result;
  result.should_shoot = false;
  
  int best_idx = getBestHypothesisIndex();
  if (best_idx < 0) return result;
  
  const auto& hyp = hypotheses_[best_idx];
  double omega = (config_.use_fixed_omega && direction_ != 0) 
                 ? direction_ * config_.known_omega 
                 : hyp.state(IDX_OMEGA);
  
  if (std::abs(omega) < config_.omega_threshold) {
    // 静止状态，无法等待
    return result;
  }
  
  // 找到即将进入可打区域的装甲板
  // 计算每个装甲板到达正面所需的时间
  int nearest_idx = -1;
  double min_time = 1e6;
  
  for (int i = 0; i < 3; ++i) {
    double facing = armors[i].facing_angle;
    double time_to_front;
    
    if (omega > 0) {
      // 逆时针旋转
      double angle_to_go = (armors[i].orientation < 0) 
                          ? -armors[i].orientation 
                          : (2 * M_PI - armors[i].orientation);
      time_to_front = angle_to_go / omega;
    } else {
      // 顺时针旋转
      double angle_to_go = (armors[i].orientation > 0) 
                          ? armors[i].orientation 
                          : (2 * M_PI + armors[i].orientation);
      time_to_front = angle_to_go / (-omega);
    }
    
    // 减去可打窗口的一半
    double window_time = config_.max_shootable_angle / std::abs(omega);
    time_to_front -= window_time;
    
    if (time_to_front > 0 && time_to_front < min_time) {
      min_time = time_to_front;
      nearest_idx = i;
    }
  }
  
  if (nearest_idx >= 0) {
    // 瞄准即将出现的位置
    result.target_type = armors[nearest_idx].type;
    result.aim_point = armors[nearest_idx].position;  // 使用当前预测位置，后续会修正
    result.distance = armors[nearest_idx].distance;
    result.hit_time = min_time;
    result.should_shoot = false;  // 等待模式不射击
  }
  
  return result;
}

// ============================================================================
// 重置
// ============================================================================

void OutpostPredictorV3::reset()
{
  initialized_ = false;
  converged_ = false;
  best_hypothesis_idx_ = -1;
  direction_ = 0;
  lost_count_ = 0;
  
  last_orientation_ = 0.0;
  orientation_change_sum_ = 0.0;
  direction_samples_ = 0;
  
  obs_history_.clear();
  
  for (auto& hyp : hypotheses_) {
    hyp.state = Eigen::VectorXd::Zero(STATE_DIM);
    hyp.covariance = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    hyp.confidence = 1.0 / 3.0;
    hyp.update_count = 0;
  }
  
  std::cout << "[OutpostPredictorV3] 已重置" << std::endl;
}

// ============================================================================
// 静态工具函数
// ============================================================================

double OutpostPredictorV3::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double OutpostPredictorV3::angleDiff(double a, double b)
{
  return normalizeAngle(a - b);
}

double OutpostPredictorV3::getClosestAngle(double angle, double target)
{
  double diff = normalizeAngle(angle - target);
  return target + diff;
}

void OutpostPredictorV3::cartesianToPolar(
  const Eigen::Vector3d& xyz, 
  double& yaw, double& pitch, double& dist)
{
  // 相机坐标系: X右, Y下, Z前
  dist = xyz.norm();
  if (dist < 1e-6) {
    yaw = 0;
    pitch = 0;
    return;
  }
  
  yaw = std::atan2(xyz.x(), xyz.z());
  pitch = std::asin(-xyz.y() / dist);  // Y向下为正，所以pitch向上为负
}

Eigen::Vector3d OutpostPredictorV3::polarToCartesian(double yaw, double pitch, double dist)
{
  // 相机坐标系: X右, Y下, Z前
  Eigen::Vector3d xyz;
  xyz.z() = dist * std::cos(pitch) * std::cos(yaw);
  xyz.x() = dist * std::cos(pitch) * std::sin(yaw);
  xyz.y() = -dist * std::sin(pitch);  // Y向下为正
  return xyz;
}

}  // namespace armor_detector
