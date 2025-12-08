/**
 * @file extended_kalman_filter.cpp
 * @brief Extended Kalman Filter implementation
 * 
 * Reference: OrangeAim-Drone EKF and sp_vision_25 EKF implementations
 * Requirements: 7.1 - 前哨站预测模块 EKF 估计
 */

#include "core/extended_kalman_filter.hpp"

#include <numeric>

namespace armor_detector
{

ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0,
  const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I_(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add_(x_add)
{
  // Initialize diagnostic data
  diagnostics_["residual_0"] = 0.0;
  diagnostics_["residual_1"] = 0.0;
  diagnostics_["residual_2"] = 0.0;
  diagnostics_["residual_3"] = 0.0;
  diagnostics_["nis"] = 0.0;
  diagnostics_["nees"] = 0.0;
  diagnostics_["nis_fail"] = 0.0;
  diagnostics_["nees_fail"] = 0.0;
  diagnostics_["recent_nis_failure_rate"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F,
  const Eigen::MatrixXd & Q)
{
  // Linear prediction: x = F * x
  return predict(F, Q, [&](const Eigen::VectorXd & state) { return F * state; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F,
  const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  // Predict state covariance: P = F * P * F^T + Q
  P = F * P * F.transpose() + Q;
  
  // Predict state: x = f(x)
  x = f(x);
  
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z,
  const Eigen::MatrixXd & H,
  const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  // Linear observation: h(x) = H * x
  return update(z, H, R, [&](const Eigen::VectorXd & state) { return H * state; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z,
  const Eigen::MatrixXd & H,
  const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  // Store prior state for NEES calculation
  Eigen::VectorXd x_prior = x;

  // Innovation covariance: S = H * P * H^T + R
  Eigen::MatrixXd S = H * P * H.transpose() + R;

  // Kalman gain: K = P * H^T * S^(-1)
  Eigen::MatrixXd K = P * H.transpose() * S.inverse();

  // Innovation (measurement residual): y = z - h(x)
  Eigen::VectorXd innovation = z_subtract(z, h(x));

  // Update state: x = x + K * y (using custom addition)
  x = x_add_(x, K * innovation);

  // Update covariance using Joseph form for numerical stability
  // P = (I - K * H) * P * (I - K * H)^T + K * R * K^T
  Eigen::MatrixXd I_KH = I_ - K * H;
  P = I_KH * P * I_KH.transpose() + K * R * K.transpose();

  // Update chi-square statistics
  updateChiSquareStats(innovation, S, x_prior);

  return x;
}

void ExtendedKalmanFilter::reset(const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0)
{
  x = x0;
  P = P0;
  I_ = Eigen::MatrixXd::Identity(x0.rows(), x0.rows());
  
  // Reset statistics
  last_nis_ = 0.0;
  last_nees_ = 0.0;
  nis_fail_count_ = 0;
  nees_fail_count_ = 0;
  total_update_count_ = 0;
  recent_nis_failures_.clear();
  
  // Reset diagnostics
  for (auto & kv : diagnostics_) {
    kv.second = 0.0;
  }
}

double ExtendedKalmanFilter::getRecentNISFailureRate() const
{
  if (recent_nis_failures_.empty()) {
    return 0.0;
  }
  int failures = std::accumulate(recent_nis_failures_.begin(), recent_nis_failures_.end(), 0);
  return static_cast<double>(failures) / recent_nis_failures_.size();
}

void ExtendedKalmanFilter::updateChiSquareStats(
  const Eigen::VectorXd & residual,
  const Eigen::MatrixXd & S,
  const Eigen::VectorXd & x_prior,
  double nis_threshold,
  double nees_threshold)
{
  // Compute NIS (Normalized Innovation Squared)
  // NIS = y^T * S^(-1) * y
  last_nis_ = residual.transpose() * S.inverse() * residual;

  // Compute NEES (Normalized Estimation Error Squared)
  // NEES = (x - x_prior)^T * P^(-1) * (x - x_prior)
  Eigen::VectorXd state_diff = x - x_prior;
  last_nees_ = state_diff.transpose() * P.inverse() * state_diff;

  // Update failure counts
  total_update_count_++;
  
  diagnostics_["nis_fail"] = 0.0;
  diagnostics_["nees_fail"] = 0.0;
  
  if (last_nis_ > nis_threshold) {
    nis_fail_count_++;
    diagnostics_["nis_fail"] = 1.0;
  }
  
  if (last_nees_ > nees_threshold) {
    nees_fail_count_++;
    diagnostics_["nees_fail"] = 1.0;
  }

  // Track recent NIS failures
  recent_nis_failures_.push_back(last_nis_ > nis_threshold ? 1 : 0);
  if (recent_nis_failures_.size() > nis_window_size_) {
    recent_nis_failures_.pop_front();
  }

  // Update diagnostics
  diagnostics_["nis"] = last_nis_;
  diagnostics_["nees"] = last_nees_;
  diagnostics_["recent_nis_failure_rate"] = getRecentNISFailureRate();
  
  // Store residuals (up to 4 dimensions)
  for (int i = 0; i < std::min(4, static_cast<int>(residual.size())); ++i) {
    diagnostics_["residual_" + std::to_string(i)] = residual[i];
  }
}

}  // namespace armor_detector
