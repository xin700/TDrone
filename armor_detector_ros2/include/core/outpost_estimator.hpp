/**
 * @file outpost_estimator.hpp
 * @brief Outpost state estimator using Extended Kalman Filter
 * 
 * This module estimates the state of an outpost (rotating turret) including:
 * - Center position and velocity
 * - Rotation angle (theta) and angular velocity (omega)
 * - Rotation direction detection
 * - Chi-square test for outlier rejection
 * 
 * Reference: OrangeAim-Drone OutpostPredictor implementation
 * Requirements: 7.1, 7.2, 7.3, 7.4
 */

#ifndef ARMOR_DETECTOR_ROS2__CORE__OUTPOST_ESTIMATOR_HPP_
#define ARMOR_DETECTOR_ROS2__CORE__OUTPOST_ESTIMATOR_HPP_

#include <Eigen/Dense>
#include <memory>

#include "core/extended_kalman_filter.hpp"

namespace armor_detector
{

/// Outpost physical constants
constexpr double OUTPOST_RADIUS = 0.275;  // Outpost radius in meters
constexpr double OUTPOST_OMEGA = 0.4 * 2 * M_PI;  // 0.4转/秒 = 0.8π rad/s ≈ 2.51 rad/s

/**
 * @brief Outpost state information structure
 */
struct OutpostInformation
{
  /// Center position in world frame (x, y, z)
  Eigen::Vector3d center_position{0.0, 0.0, 0.0};
  
  /// Center velocity in world frame (vx, vy, vz)
  Eigen::Vector3d center_velocity{0.0, 0.0, 0.0};
  
  /// Outpost radius (meters)
  double outpost_radius{OUTPOST_RADIUS};
  
  /// Current rotation angle (radians)
  double outpost_theta{0.0};
  
  /// Angular velocity (rad/s)
  double outpost_omega{0.0};
  
  /// Rotation direction: -1 = clockwise, 1 = counterclockwise, 0 = unknown
  int direction{0};
  
  /// Whether the estimation is valid
  bool is_valid{false};
};

/**
 * @brief Armor pose measurement for outpost estimation
 */
struct ArmorMeasurement
{
  /// Yaw angle relative to gimbal (radians)
  double yaw{0.0};
  
  /// Pitch angle relative to gimbal (radians)
  double pitch{0.0};
  
  /// Distance to armor (meters)
  double distance{0.0};
  
  /// Armor orientation in world frame (radians)
  double theta_world{0.0};
  
  /// Timestamp (seconds)
  double timestamp{0.0};
};

/**
 * @brief Outpost state estimator using EKF
 * 
 * State vector: [x_c, v_x, y_c, v_y, z_c, v_z, theta, omega]
 * - x_c, y_c, z_c: Center position
 * - v_x, v_y, v_z: Center velocity
 * - theta: Rotation angle
 * - omega: Angular velocity
 * 
 * Measurement vector: [yaw_a, pitch_a, dis_a, theta_a]
 * - yaw_a, pitch_a: Armor angles relative to gimbal
 * - dis_a: Distance to armor
 * - theta_a: Armor orientation in world frame
 */
class OutpostEstimator
{
public:
  /// State dimension
  static constexpr int STATE_DIM = 8;
  
  /// Measurement dimension
  static constexpr int MEAS_DIM = 4;

  /**
   * @brief Default constructor
   */
  OutpostEstimator();

  /**
   * @brief Initialize the estimator with first measurement
   * @param measurement First armor measurement
   * @param direction Initial rotation direction (-1, 0, or 1)
   */
  void initialize(const ArmorMeasurement & measurement, int direction = 0);

  /**
   * @brief Set the time step for prediction
   * @param dt Time step in seconds
   */
  void setDeltaTime(double dt) { dt_ = dt; }

  /**
   * @brief Process a new measurement
   * @param measurement New armor measurement
   * @return true if measurement was accepted, false if rejected by chi-square test
   */
  bool update(const ArmorMeasurement & measurement);

  /**
   * @brief Predict state without measurement (time update only)
   */
  void predict();

  /**
   * @brief Get current outpost state information
   * @param use_predicted If true, return predicted state; otherwise return filtered state
   * @return OutpostInformation structure
   */
  OutpostInformation getInformation(bool use_predicted = false) const;

  /**
   * @brief Reset the estimator
   */
  void reset();

  /**
   * @brief Check if the estimator is initialized
   * @return true if initialized
   */
  bool isInitialized() const { return initialized_; }

  /**
   * @brief Get the current rotation direction
   * @return -1 (clockwise), 1 (counterclockwise), or 0 (unknown)
   */
  int getDirection() const { return direction_; }

  /**
   * @brief Detect rotation direction change
   * @return true if direction changed since last check
   */
  bool detectDirectionChange();

  /**
   * @brief Get the last chi-square test value
   * @return Chi-square value
   */
  double getLastChiSquareValue() const { return chi_square_value_; }

  /**
   * @brief Check if last measurement passed chi-square test
   * @return true if passed
   */
  bool passedChiSquareTest() const { return chi_square_value_ <= chi_square_threshold_; }

  /**
   * @brief Set chi-square threshold for outlier rejection
   * @param threshold Chi-square threshold (default: 11.07 for 95% confidence, df=4)
   */
  void setChiSquareThreshold(double threshold) { chi_square_threshold_ = threshold; }

  /**
   * @brief Get the current state vector
   * @return State vector [x_c, v_x, y_c, v_y, z_c, v_z, theta, omega]
   */
  Eigen::VectorXd getState() const;

  /**
   * @brief Get the predicted state vector (before measurement update)
   * @return Predicted state vector
   */
  Eigen::VectorXd getPredictedState() const { return x_predicted_; }

  /**
   * @brief Get the state covariance matrix
   * @return State covariance matrix
   */
  Eigen::MatrixXd getCovariance() const;

private:
  /// Extended Kalman Filter
  std::unique_ptr<ExtendedKalmanFilter> ekf_;

  /// Time step (seconds)
  double dt_{0.01};

  /// Rotation direction: -1 = clockwise, 1 = counterclockwise, 0 = unknown
  int direction_{0};

  /// Previous direction for change detection
  int prev_direction_{0};

  /// Chi-square threshold for outlier rejection (df=4, 95% confidence)
  double chi_square_threshold_{11.07};

  /// Last chi-square test value
  double chi_square_value_{0.0};

  /// Whether the estimator is initialized
  bool initialized_{false};

  /// Last armor measurement for direction detection
  ArmorMeasurement last_measurement_;

  /// Predicted state (before measurement update)
  Eigen::VectorXd x_predicted_;

  /// Whether to use fixed omega (known outpost angular velocity)
  bool use_fixed_omega_{true};

  /// Direction detection: number of samples collected
  int direction_samples_{0};

  /// Direction detection: sum of theta changes
  double theta_change_sum_{0.0};

  /// Direction detection: last theta value
  double last_theta_{0.0};

  /// Number of samples needed for direction detection
  static constexpr int DIRECTION_DETECTION_SAMPLES = 5;

  /// State transition matrix
  Eigen::MatrixXd F_;

  /// Process noise covariance
  Eigen::MatrixXd Q_;

  /// Measurement noise covariance
  Eigen::MatrixXd R_;

  /// Observation Jacobian matrix
  Eigen::MatrixXd H_;

  /**
   * @brief Normalize angle to [-pi, pi]
   * @param angle Angle to normalize
   */
  static void normalizeAngle(double & angle);

  /**
   * @brief Set up the state transition matrix
   */
  void setupTransitionMatrix();

  /**
   * @brief Set up the process noise covariance matrix
   */
  void setupProcessNoise();

  /**
   * @brief Set up the measurement noise covariance matrix
   * @param distance Distance to armor (affects noise)
   */
  void setupMeasurementNoise(double distance);

  /**
   * @brief Compute observation Jacobian matrix
   */
  void computeObservationJacobian();

  /**
   * @brief Nonlinear observation function: state -> measurement
   * @param state State vector
   * @return Expected measurement vector
   */
  Eigen::VectorXd observationFunction(const Eigen::VectorXd & state) const;

  /**
   * @brief Perform chi-square test on innovation
   * @param innovation Measurement innovation (residual)
   * @param innovation_cov Innovation covariance matrix
   * @return true if measurement passes test
   */
  bool chiSquareTest(
    const Eigen::VectorXd & innovation,
    const Eigen::MatrixXd & innovation_cov);

  /**
   * @brief Convert armor measurement to state-space measurement
   * @param measurement Armor measurement
   * @return Measurement vector [yaw, pitch, distance, theta]
   */
  Eigen::VectorXd measurementToVector(const ArmorMeasurement & measurement) const;

  /**
   * @brief Compute armor position from center state
   * @param x_c Center x position
   * @param y_c Center y position
   * @param z_c Center z position
   * @param theta Rotation angle
   * @param x_a Output armor x position
   * @param y_a Output armor y position
   * @param z_a Output armor z position
   */
  void computeArmorPosition(
    double x_c, double y_c, double z_c, double theta,
    double & x_a, double & y_a, double & z_a) const;
};

}  // namespace armor_detector

#endif  // ARMOR_DETECTOR_ROS2__CORE__OUTPOST_ESTIMATOR_HPP_
