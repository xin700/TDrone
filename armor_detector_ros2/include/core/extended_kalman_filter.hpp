/**
 * @file extended_kalman_filter.hpp
 * @brief Extended Kalman Filter implementation for state estimation
 * 
 * This EKF implementation supports:
 * - Linear and nonlinear state transition models
 * - Linear and nonlinear observation models
 * - Custom state addition and measurement subtraction functions
 * - Chi-square test for outlier rejection (NIS/NEES)
 * 
 * Reference: OrangeAim-Drone EKF and sp_vision_25 EKF implementations
 * Requirements: 7.1 - 前哨站预测模块 EKF 估计
 */

#ifndef ARMOR_DETECTOR_ROS2__CORE__EXTENDED_KALMAN_FILTER_HPP_
#define ARMOR_DETECTOR_ROS2__CORE__EXTENDED_KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <map>
#include <string>

namespace armor_detector
{

/**
 * @brief Extended Kalman Filter class for state estimation
 * 
 * Implements the standard EKF predict-update cycle with support for:
 * - Nonlinear state transition via custom function f(x)
 * - Nonlinear observation via custom function h(x)
 * - Chi-square test (NIS/NEES) for outlier detection
 */
class ExtendedKalmanFilter
{
public:
  /// State vector
  Eigen::VectorXd x;
  
  /// State covariance matrix
  Eigen::MatrixXd P;

  /**
   * @brief Default constructor
   */
  ExtendedKalmanFilter() = default;

  /**
   * @brief Construct EKF with initial state and covariance
   * @param x0 Initial state vector
   * @param P0 Initial state covariance matrix
   * @param x_add Custom state addition function (default: vector addition)
   */
  ExtendedKalmanFilter(
    const Eigen::VectorXd & x0,
    const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; });

  /**
   * @brief Predict step with linear state transition
   * @param F State transition matrix
   * @param Q Process noise covariance matrix
   * @return Predicted state vector
   */
  Eigen::VectorXd predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q);

  /**
   * @brief Predict step with nonlinear state transition
   * @param F Jacobian of state transition function
   * @param Q Process noise covariance matrix
   * @param f Nonlinear state transition function
   * @return Predicted state vector
   */
  Eigen::VectorXd predict(
    const Eigen::MatrixXd & F,
    const Eigen::MatrixXd & Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f);

  /**
   * @brief Update step with linear observation model
   * @param z Measurement vector
   * @param H Observation matrix
   * @param R Measurement noise covariance matrix
   * @param z_subtract Custom measurement subtraction function (default: vector subtraction)
   * @return Updated state vector
   */
  Eigen::VectorXd update(
    const Eigen::VectorXd & z,
    const Eigen::MatrixXd & H,
    const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

  /**
   * @brief Update step with nonlinear observation model
   * @param z Measurement vector
   * @param H Jacobian of observation function
   * @param R Measurement noise covariance matrix
   * @param h Nonlinear observation function
   * @param z_subtract Custom measurement subtraction function (default: vector subtraction)
   * @return Updated state vector
   */
  Eigen::VectorXd update(
    const Eigen::VectorXd & z,
    const Eigen::MatrixXd & H,
    const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

  /**
   * @brief Reset the filter with new initial state and covariance
   * @param x0 New initial state vector
   * @param P0 New initial state covariance matrix
   */
  void reset(const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0);

  /**
   * @brief Get the last computed NIS (Normalized Innovation Squared) value
   * @return Last NIS value
   */
  double getLastNIS() const { return last_nis_; }

  /**
   * @brief Get the last computed NEES (Normalized Estimation Error Squared) value
   * @return Last NEES value
   */
  double getLastNEES() const { return last_nees_; }

  /**
   * @brief Check if the last measurement passed the chi-square test
   * @param threshold Chi-square threshold (default: 0.711 for 95% confidence, df=4)
   * @return true if NIS is below threshold
   */
  bool passedChiSquareTest(double threshold = 0.711) const { return last_nis_ <= threshold; }

  /**
   * @brief Get the recent NIS failure rate
   * @return Failure rate in [0, 1]
   */
  double getRecentNISFailureRate() const;

  /**
   * @brief Get diagnostic data map
   * @return Map of diagnostic data
   */
  const std::map<std::string, double> & getDiagnostics() const { return diagnostics_; }

  /**
   * @brief Set the window size for recent NIS failure tracking
   * @param size Window size
   */
  void setNISWindowSize(size_t size) { nis_window_size_ = size; }

private:
  /// Identity matrix (cached for efficiency)
  Eigen::MatrixXd I_;
  
  /// Custom state addition function
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add_;

  /// Chi-square test statistics
  double last_nis_ = 0.0;
  double last_nees_ = 0.0;
  int nis_fail_count_ = 0;
  int nees_fail_count_ = 0;
  int total_update_count_ = 0;

  /// Recent NIS failures tracking
  std::deque<int> recent_nis_failures_;
  size_t nis_window_size_ = 100;

  /// Diagnostic data
  std::map<std::string, double> diagnostics_;

  /// Update chi-square statistics
  void updateChiSquareStats(
    const Eigen::VectorXd & residual,
    const Eigen::MatrixXd & S,
    const Eigen::VectorXd & x_prior,
    double nis_threshold = 0.711,
    double nees_threshold = 0.711);
};

}  // namespace armor_detector

#endif  // ARMOR_DETECTOR_ROS2__CORE__EXTENDED_KALMAN_FILTER_HPP_
