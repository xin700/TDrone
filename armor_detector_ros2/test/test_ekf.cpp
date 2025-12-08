/**
 * @file test_ekf.cpp
 * @brief Unit tests for ExtendedKalmanFilter class
 * 
 * Tests cover:
 * - Predict step (linear and nonlinear)
 * - Update step (linear and nonlinear)
 * - Chi-square test (NIS/NEES)
 * - State reset
 * 
 * Requirements: 7.1 - 前哨站预测模块 EKF 估计
 * 
 * Property tests validate:
 * - Property 7: EKF 更新协方差减小 (covariance trace should not increase after update)
 * - Property 10: EKF 预测模型一致性 (state evolves according to motion model)
 * - Property 24: 卡方检验异常值拒绝 (chi-square test rejects outliers)
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "core/extended_kalman_filter.hpp"

namespace armor_detector
{
namespace test
{

class EKFTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize a simple 2D position-velocity EKF
    // State: [x, vx, y, vy]
    state_dim_ = 4;
    meas_dim_ = 2;
    
    // Initial state: position (0, 0), velocity (1, 1)
    x0_ = Eigen::VectorXd::Zero(state_dim_);
    x0_ << 0.0, 1.0, 0.0, 1.0;
    
    // Initial covariance
    P0_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 1.0;
    
    // State transition matrix (constant velocity model, dt = 0.1)
    dt_ = 0.1;
    F_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    F_(0, 1) = dt_;  // x += vx * dt
    F_(2, 3) = dt_;  // y += vy * dt
    
    // Process noise
    Q_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 0.01;
    
    // Observation matrix (observe position only)
    H_ = Eigen::MatrixXd::Zero(meas_dim_, state_dim_);
    H_(0, 0) = 1.0;  // observe x
    H_(1, 2) = 1.0;  // observe y
    
    // Measurement noise
    R_ = Eigen::MatrixXd::Identity(meas_dim_, meas_dim_) * 0.1;
  }

  int state_dim_;
  int meas_dim_;
  double dt_;
  Eigen::VectorXd x0_;
  Eigen::MatrixXd P0_;
  Eigen::MatrixXd F_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;
};

// Test: EKF initialization
TEST_F(EKFTest, Initialization)
{
  ExtendedKalmanFilter ekf(x0_, P0_);
  
  EXPECT_EQ(ekf.x.size(), state_dim_);
  EXPECT_EQ(ekf.P.rows(), state_dim_);
  EXPECT_EQ(ekf.P.cols(), state_dim_);
  
  // Check initial state
  EXPECT_DOUBLE_EQ(ekf.x(0), 0.0);
  EXPECT_DOUBLE_EQ(ekf.x(1), 1.0);
  EXPECT_DOUBLE_EQ(ekf.x(2), 0.0);
  EXPECT_DOUBLE_EQ(ekf.x(3), 1.0);
}

// Test: Linear predict step
TEST_F(EKFTest, LinearPredict)
{
  ExtendedKalmanFilter ekf(x0_, P0_);
  
  // Predict one step
  Eigen::VectorXd x_pred = ekf.predict(F_, Q_);
  
  // Expected: x = 0 + 1*0.1 = 0.1, y = 0 + 1*0.1 = 0.1
  EXPECT_NEAR(x_pred(0), 0.1, 1e-10);
  EXPECT_NEAR(x_pred(1), 1.0, 1e-10);  // velocity unchanged
  EXPECT_NEAR(x_pred(2), 0.1, 1e-10);
  EXPECT_NEAR(x_pred(3), 1.0, 1e-10);  // velocity unchanged
  
  // Covariance should increase after predict (due to process noise)
  double trace_before = P0_.trace();
  double trace_after = ekf.P.trace();
  EXPECT_GT(trace_after, trace_before);
}

// Test: Nonlinear predict step
TEST_F(EKFTest, NonlinearPredict)
{
  ExtendedKalmanFilter ekf(x0_, P0_);
  
  // Nonlinear state transition (same as linear for this test)
  auto f = [this](const Eigen::VectorXd & state) {
    Eigen::VectorXd new_state = state;
    new_state(0) += state(1) * dt_;
    new_state(2) += state(3) * dt_;
    return new_state;
  };
  
  Eigen::VectorXd x_pred = ekf.predict(F_, Q_, f);
  
  // Should give same result as linear predict
  EXPECT_NEAR(x_pred(0), 0.1, 1e-10);
  EXPECT_NEAR(x_pred(2), 0.1, 1e-10);
}

// Test: Linear update step
TEST_F(EKFTest, LinearUpdate)
{
  ExtendedKalmanFilter ekf(x0_, P0_);
  
  // Predict first
  ekf.predict(F_, Q_);
  double trace_before_update = ekf.P.trace();
  
  // Measurement: observe position (0.12, 0.08)
  Eigen::VectorXd z(meas_dim_);
  z << 0.12, 0.08;
  
  Eigen::VectorXd x_upd = ekf.update(z, H_, R_);
  
  // State should be updated towards measurement
  // The exact value depends on Kalman gain
  EXPECT_GT(x_upd(0), 0.1);  // Should move towards 0.12
  EXPECT_LT(x_upd(2), 0.1);  // Should move towards 0.08
  
  // Property 7: Covariance trace should not increase after update
  // (information gain from measurement)
  double trace_after_update = ekf.P.trace();
  EXPECT_LE(trace_after_update, trace_before_update);
}

// Test: Nonlinear update step
TEST_F(EKFTest, NonlinearUpdate)
{
  ExtendedKalmanFilter ekf(x0_, P0_);
  ekf.predict(F_, Q_);
  
  // Nonlinear observation function (same as linear for this test)
  auto h = [this](const Eigen::VectorXd & state) {
    Eigen::VectorXd z(meas_dim_);
    z << state(0), state(2);
    return z;
  };
  
  Eigen::VectorXd z(meas_dim_);
  z << 0.12, 0.08;
  
  Eigen::VectorXd x_upd = ekf.update(z, H_, R_, h);
  
  // Should give similar result as linear update
  EXPECT_GT(x_upd(0), 0.1);
  EXPECT_LT(x_upd(2), 0.1);
}

/**
 * Feature: ros2-vision-migration, Property 7: EKF 更新协方差减小
 * Validates: Requirements 3.1
 * 
 * For any EKF state and valid observation, executing update step
 * should result in covariance trace not greater than before update.
 */
TEST_F(EKFTest, Property7_CovarianceTraceDecreasesAfterUpdate)
{
  // Test with multiple random measurements
  for (int trial = 0; trial < 10; ++trial) {
    ExtendedKalmanFilter ekf(x0_, P0_);
    ekf.predict(F_, Q_);
    
    double trace_before = ekf.P.trace();
    
    // Generate random measurement near predicted position
    Eigen::VectorXd z(meas_dim_);
    z << ekf.x(0) + (rand() % 100 - 50) * 0.001,
         ekf.x(2) + (rand() % 100 - 50) * 0.001;
    
    ekf.update(z, H_, R_);
    
    double trace_after = ekf.P.trace();
    
    // Property 7: Covariance trace should not increase
    EXPECT_LE(trace_after, trace_before + 1e-10)
      << "Trial " << trial << ": Covariance trace increased from "
      << trace_before << " to " << trace_after;
  }
}

/**
 * Feature: ros2-vision-migration, Property 10: EKF 预测模型一致性
 * Validates: Requirements 3.5
 * 
 * For any EKF state, in the absence of observation, executing predict
 * should evolve state according to motion model (position += velocity * dt).
 */
TEST_F(EKFTest, Property10_PredictFollowsMotionModel)
{
  // Test with different initial velocities
  for (int trial = 0; trial < 10; ++trial) {
    Eigen::VectorXd x_init = Eigen::VectorXd::Zero(state_dim_);
    double vx = (rand() % 100 - 50) * 0.1;  // Random velocity
    double vy = (rand() % 100 - 50) * 0.1;
    x_init << 0.0, vx, 0.0, vy;
    
    ExtendedKalmanFilter ekf(x_init, P0_);
    
    // Predict multiple steps
    for (int step = 0; step < 5; ++step) {
      double x_before = ekf.x(0);
      double y_before = ekf.x(2);
      double vx_current = ekf.x(1);
      double vy_current = ekf.x(3);
      
      ekf.predict(F_, Q_);
      
      // Property 10: Position should evolve as position += velocity * dt
      EXPECT_NEAR(ekf.x(0), x_before + vx_current * dt_, 1e-10)
        << "Trial " << trial << ", Step " << step << ": X position mismatch";
      EXPECT_NEAR(ekf.x(2), y_before + vy_current * dt_, 1e-10)
        << "Trial " << trial << ", Step " << step << ": Y position mismatch";
      
      // Velocity should remain unchanged in constant velocity model
      EXPECT_NEAR(ekf.x(1), vx, 1e-10);
      EXPECT_NEAR(ekf.x(3), vy, 1e-10);
    }
  }
}

// Test: Chi-square test (NIS)
TEST_F(EKFTest, ChiSquareTest_NIS)
{
  ExtendedKalmanFilter ekf(x0_, P0_);
  ekf.predict(F_, Q_);
  
  // Normal measurement (should pass chi-square test)
  Eigen::VectorXd z_normal(meas_dim_);
  z_normal << ekf.x(0) + 0.01, ekf.x(2) + 0.01;
  
  ekf.update(z_normal, H_, R_);
  
  // NIS should be computed
  EXPECT_GE(ekf.getLastNIS(), 0.0);
  
  // For a normal measurement, NIS should be relatively small
  // (exact threshold depends on chi-square distribution)
}

/**
 * Feature: ros2-vision-migration, Property 24: 卡方检验异常值拒绝
 * Validates: Requirements 7.4
 * 
 * For any observation, when its Mahalanobis distance exceeds chi-square threshold,
 * the observation should be flagged as an outlier.
 */
TEST_F(EKFTest, Property24_ChiSquareRejectsOutliers)
{
  ExtendedKalmanFilter ekf(x0_, P0_);
  ekf.predict(F_, Q_);
  
  // Outlier measurement (far from predicted position)
  Eigen::VectorXd z_outlier(meas_dim_);
  z_outlier << ekf.x(0) + 10.0, ekf.x(2) + 10.0;  // Very far from prediction
  
  ekf.update(z_outlier, H_, R_);
  
  // NIS should be large for outlier
  double nis = ekf.getLastNIS();
  EXPECT_GT(nis, 0.711)  // Chi-square threshold for 95% confidence
    << "Outlier should have NIS > threshold, got NIS = " << nis;
  
  // passedChiSquareTest should return false
  EXPECT_FALSE(ekf.passedChiSquareTest())
    << "Outlier should fail chi-square test";
  
  // Diagnostics should indicate failure
  const auto & diag = ekf.getDiagnostics();
  EXPECT_EQ(diag.at("nis_fail"), 1.0);
}

// Test: Normal measurement passes chi-square test
TEST_F(EKFTest, ChiSquarePassesForNormalMeasurement)
{
  ExtendedKalmanFilter ekf(x0_, P0_);
  ekf.predict(F_, Q_);
  
  // Very close measurement (should definitely pass)
  Eigen::VectorXd z_close(meas_dim_);
  z_close << ekf.x(0), ekf.x(2);  // Exactly at predicted position
  
  ekf.update(z_close, H_, R_);
  
  // NIS should be very small
  double nis = ekf.getLastNIS();
  EXPECT_LT(nis, 0.711)
    << "Close measurement should have small NIS, got NIS = " << nis;
  
  EXPECT_TRUE(ekf.passedChiSquareTest());
}

// Test: Reset functionality
TEST_F(EKFTest, Reset)
{
  ExtendedKalmanFilter ekf(x0_, P0_);
  
  // Run some predictions and updates
  ekf.predict(F_, Q_);
  Eigen::VectorXd z(meas_dim_);
  z << 0.5, 0.5;
  ekf.update(z, H_, R_);
  
  // State should have changed
  EXPECT_NE(ekf.x(0), x0_(0));
  
  // Reset
  Eigen::VectorXd new_x0 = Eigen::VectorXd::Zero(state_dim_);
  new_x0 << 1.0, 0.0, 1.0, 0.0;
  Eigen::MatrixXd new_P0 = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 2.0;
  
  ekf.reset(new_x0, new_P0);
  
  // Check reset state
  EXPECT_DOUBLE_EQ(ekf.x(0), 1.0);
  EXPECT_DOUBLE_EQ(ekf.x(1), 0.0);
  EXPECT_DOUBLE_EQ(ekf.P(0, 0), 2.0);
  
  // Statistics should be reset
  EXPECT_DOUBLE_EQ(ekf.getLastNIS(), 0.0);
  EXPECT_DOUBLE_EQ(ekf.getLastNEES(), 0.0);
}

// Test: Recent NIS failure rate tracking
TEST_F(EKFTest, RecentNISFailureRate)
{
  ExtendedKalmanFilter ekf(x0_, P0_);
  ekf.setNISWindowSize(10);
  
  // Run 10 updates with alternating normal and outlier measurements
  for (int i = 0; i < 10; ++i) {
    ekf.predict(F_, Q_);
    
    Eigen::VectorXd z(meas_dim_);
    if (i % 2 == 0) {
      // Normal measurement
      z << ekf.x(0), ekf.x(2);
    } else {
      // Outlier measurement
      z << ekf.x(0) + 100.0, ekf.x(2) + 100.0;
    }
    
    ekf.update(z, H_, R_);
  }
  
  // Failure rate should be approximately 0.5 (5 outliers out of 10)
  double failure_rate = ekf.getRecentNISFailureRate();
  EXPECT_NEAR(failure_rate, 0.5, 0.1);
}

// Test: Custom state addition function (for angle wrapping)
TEST_F(EKFTest, CustomStateAddition)
{
  // State with angle: [x, vx, theta]
  int state_dim = 3;
  int meas_dim = 1;
  
  Eigen::VectorXd x0(state_dim);
  x0 << 0.0, 1.0, M_PI - 0.1;  // theta near pi
  
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(state_dim, state_dim);
  
  // Custom addition that wraps angle to [-pi, pi]
  auto angle_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    Eigen::VectorXd result = a + b;
    // Wrap angle (index 2) to [-pi, pi]
    while (result(2) > M_PI) result(2) -= 2 * M_PI;
    while (result(2) < -M_PI) result(2) += 2 * M_PI;
    return result;
  };
  
  ExtendedKalmanFilter ekf(x0, P0, angle_add);
  
  // State transition that increases angle
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(state_dim, state_dim);
  F(0, 1) = 0.1;
  
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(state_dim, state_dim) * 0.01;
  
  // Observation of angle
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(meas_dim, state_dim);
  H(0, 2) = 1.0;
  
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(meas_dim, meas_dim) * 0.1;
  
  ekf.predict(F, Q);
  
  // Measurement that would push angle past pi
  Eigen::VectorXd z(meas_dim);
  z << -M_PI + 0.2;  // Measurement on the other side of the wrap
  
  // Custom subtraction for angle
  auto angle_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    Eigen::VectorXd result = a - b;
    while (result(0) > M_PI) result(0) -= 2 * M_PI;
    while (result(0) < -M_PI) result(0) += 2 * M_PI;
    return result;
  };
  
  ekf.update(z, H, R, angle_subtract);
  
  // Angle should be wrapped to [-pi, pi]
  EXPECT_GE(ekf.x(2), -M_PI);
  EXPECT_LE(ekf.x(2), M_PI);
}

// Test: Multiple predict-update cycles
TEST_F(EKFTest, MultipleCycles)
{
  ExtendedKalmanFilter ekf(x0_, P0_);
  
  // Simulate tracking a target moving at constant velocity
  double true_x = 0.0;
  double true_y = 0.0;
  double true_vx = 1.0;
  double true_vy = 0.5;
  
  for (int i = 0; i < 20; ++i) {
    // True state evolution
    true_x += true_vx * dt_;
    true_y += true_vy * dt_;
    
    // Predict
    ekf.predict(F_, Q_);
    
    // Noisy measurement
    Eigen::VectorXd z(meas_dim_);
    z << true_x + (rand() % 100 - 50) * 0.001,
         true_y + (rand() % 100 - 50) * 0.001;
    
    // Update
    ekf.update(z, H_, R_);
    
    // State should track true position reasonably well
    if (i > 5) {  // Allow some convergence time
      EXPECT_NEAR(ekf.x(0), true_x, 0.5)
        << "Step " << i << ": X position tracking error too large";
      EXPECT_NEAR(ekf.x(2), true_y, 0.5)
        << "Step " << i << ": Y position tracking error too large";
    }
  }
  
  // Final velocity estimate should be close to true velocity
  EXPECT_NEAR(ekf.x(1), true_vx, 0.3);
  EXPECT_NEAR(ekf.x(3), true_vy, 0.3);
}

// Test: Diagnostics data
TEST_F(EKFTest, DiagnosticsData)
{
  ExtendedKalmanFilter ekf(x0_, P0_);
  ekf.predict(F_, Q_);
  
  Eigen::VectorXd z(meas_dim_);
  z << 0.15, 0.05;
  
  ekf.update(z, H_, R_);
  
  const auto & diag = ekf.getDiagnostics();
  
  // Check that diagnostics are populated
  EXPECT_TRUE(diag.count("nis") > 0);
  EXPECT_TRUE(diag.count("nees") > 0);
  EXPECT_TRUE(diag.count("residual_0") > 0);
  EXPECT_TRUE(diag.count("residual_1") > 0);
  
  // NIS and NEES should be non-negative
  EXPECT_GE(diag.at("nis"), 0.0);
  EXPECT_GE(diag.at("nees"), 0.0);
}

}  // namespace test
}  // namespace armor_detector

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
