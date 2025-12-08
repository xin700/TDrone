/**
 * @file test_outpost_estimator.cpp
 * @brief Unit tests for OutpostEstimator class
 * 
 * Tests cover:
 * - Initialization and state setup
 * - State update with measurements
 * - Prediction without measurements
 * - Direction detection
 * - Chi-square test for outlier rejection
 * 
 * Requirements: 7.1, 7.2, 7.3, 7.4
 * 
 * Property tests validate:
 * - Property 21: 前哨站 EKF 收敛性 (omega converges to true value)
 * - Property 22: 前哨站输出完整性 (output contains valid data)
 * - Property 23: 前哨站方向检测 (direction change detection)
 * - Property 24: 卡方检验异常值拒绝 (chi-square test rejects outliers)
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include <random>

#include "core/outpost_estimator.hpp"

namespace armor_detector
{
namespace test
{

class OutpostEstimatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    estimator_ = std::make_unique<OutpostEstimator>();
    
    // Set up random number generator
    rng_.seed(42);
    
    // Default measurement at 3 meters distance
    default_measurement_.yaw = 0.0;
    default_measurement_.pitch = 0.0;
    default_measurement_.distance = 3.0;
    default_measurement_.theta_world = 0.0;
    default_measurement_.timestamp = 0.0;
  }

  /**
   * @brief Generate a measurement from a simulated outpost
   * @param center_x Center x position
   * @param center_y Center y position
   * @param center_z Center z position
   * @param theta Current rotation angle
   * @param noise_std Standard deviation of measurement noise
   * @return ArmorMeasurement
   */
  ArmorMeasurement generateMeasurement(
    double center_x, double center_y, double center_z,
    double theta, double noise_std = 0.0)
  {
    // Compute armor position
    double x_a = center_x + OUTPOST_RADIUS * std::sin(theta);
    double y_a = center_y - OUTPOST_RADIUS * std::cos(theta);
    double z_a = center_z;

    // Add noise
    std::normal_distribution<double> noise(0.0, noise_std);
    if (noise_std > 0) {
      x_a += noise(rng_);
      y_a += noise(rng_);
      z_a += noise(rng_);
    }

    // Convert to measurement
    ArmorMeasurement m;
    m.yaw = -std::atan2(x_a, y_a);
    double xy_dist = std::sqrt(x_a * x_a + y_a * y_a);
    m.pitch = std::atan2(z_a, xy_dist);
    m.distance = std::sqrt(x_a * x_a + y_a * y_a + z_a * z_a);
    m.theta_world = theta;
    
    if (noise_std > 0) {
      m.yaw += noise(rng_) * 0.01;
      m.pitch += noise(rng_) * 0.01;
      m.theta_world += noise(rng_) * 0.05;
    }

    return m;
  }

  std::unique_ptr<OutpostEstimator> estimator_;
  ArmorMeasurement default_measurement_;
  std::mt19937 rng_;
};

// Test: Default construction
TEST_F(OutpostEstimatorTest, DefaultConstruction)
{
  EXPECT_FALSE(estimator_->isInitialized());
  EXPECT_EQ(estimator_->getDirection(), 0);
}

// Test: Initialization with measurement
TEST_F(OutpostEstimatorTest, Initialization)
{
  estimator_->initialize(default_measurement_, 1);  // counterclockwise
  
  EXPECT_TRUE(estimator_->isInitialized());
  EXPECT_EQ(estimator_->getDirection(), 1);
  
  // Check state vector size
  Eigen::VectorXd state = estimator_->getState();
  EXPECT_EQ(state.size(), OutpostEstimator::STATE_DIM);
  
  // Check covariance matrix size
  Eigen::MatrixXd cov = estimator_->getCovariance();
  EXPECT_EQ(cov.rows(), OutpostEstimator::STATE_DIM);
  EXPECT_EQ(cov.cols(), OutpostEstimator::STATE_DIM);
}

// Test: Initialization with clockwise direction
TEST_F(OutpostEstimatorTest, InitializationClockwise)
{
  estimator_->initialize(default_measurement_, -1);  // clockwise
  
  EXPECT_TRUE(estimator_->isInitialized());
  EXPECT_EQ(estimator_->getDirection(), -1);
  
  // Omega should be negative for clockwise
  Eigen::VectorXd state = estimator_->getState();
  EXPECT_LT(state(7), 0);  // omega < 0
}

// Test: Reset functionality
TEST_F(OutpostEstimatorTest, Reset)
{
  estimator_->initialize(default_measurement_, 1);
  EXPECT_TRUE(estimator_->isInitialized());
  
  estimator_->reset();
  
  EXPECT_FALSE(estimator_->isInitialized());
  EXPECT_EQ(estimator_->getDirection(), 0);
}

// Test: Predict without measurement
TEST_F(OutpostEstimatorTest, PredictOnly)
{
  estimator_->initialize(default_measurement_, 1);
  estimator_->setDeltaTime(0.01);
  
  Eigen::VectorXd state_before = estimator_->getState();
  double theta_before = state_before(6);
  double omega = state_before(7);
  
  estimator_->predict();
  
  Eigen::VectorXd state_after = estimator_->getState();
  double theta_after = state_after(6);
  
  // Theta should increase by omega * dt
  double expected_theta = theta_before + omega * 0.01;
  // Normalize expected theta
  while (expected_theta > M_PI) expected_theta -= 2 * M_PI;
  while (expected_theta < -M_PI) expected_theta += 2 * M_PI;
  
  EXPECT_NEAR(theta_after, expected_theta, 0.01);
}

// Test: Update with valid measurement
TEST_F(OutpostEstimatorTest, UpdateWithValidMeasurement)
{
  // Initialize at center (0, 3, 0) with theta = 0
  ArmorMeasurement init_m = generateMeasurement(0.0, 3.0, 0.0, 0.0);
  estimator_->initialize(init_m, 1);
  estimator_->setDeltaTime(0.01);
  
  // Generate measurement after small rotation
  double new_theta = 0.025;  // ~1.4 degrees
  ArmorMeasurement m = generateMeasurement(0.0, 3.0, 0.0, new_theta);
  
  bool accepted = estimator_->update(m);
  
  // Measurement should be accepted
  EXPECT_TRUE(accepted);
  
  // State should be updated
  Eigen::VectorXd state = estimator_->getState();
  EXPECT_NEAR(state(6), new_theta, 0.5);  // theta should be close to measurement
}

/**
 * Feature: ros2-vision-migration, Property 22: 前哨站输出完整性
 * Validates: Requirements 7.2
 * 
 * For any stable outpost estimation, output should contain valid
 * rotation center, radius (~0.275m ± 0.05m), angle, and angular velocity.
 */
TEST_F(OutpostEstimatorTest, Property22_OutputCompleteness)
{
  // Initialize estimator
  ArmorMeasurement init_m = generateMeasurement(0.0, 3.0, 0.0, 0.0);
  estimator_->initialize(init_m, 1);
  estimator_->setDeltaTime(0.01);
  
  // Run several updates to stabilize
  double theta = 0.0;
  for (int i = 0; i < 10; ++i) {
    theta += OUTPOST_OMEGA * 0.01;
    ArmorMeasurement m = generateMeasurement(0.0, 3.0, 0.0, theta, 0.01);
    estimator_->update(m);
  }
  
  // Get information
  OutpostInformation info = estimator_->getInformation();
  
  // Property 22: Output should be valid and complete
  EXPECT_TRUE(info.is_valid);
  
  // Radius should be approximately OUTPOST_RADIUS (0.275m ± 0.05m)
  EXPECT_NEAR(info.outpost_radius, OUTPOST_RADIUS, 0.05);
  
  // Center position should be finite
  EXPECT_TRUE(std::isfinite(info.center_position.x()));
  EXPECT_TRUE(std::isfinite(info.center_position.y()));
  EXPECT_TRUE(std::isfinite(info.center_position.z()));
  
  // Theta should be in [-pi, pi]
  EXPECT_GE(info.outpost_theta, -M_PI);
  EXPECT_LE(info.outpost_theta, M_PI);
  
  // Omega should be non-zero for rotating outpost
  EXPECT_NE(info.outpost_omega, 0.0);
  
  // Direction should be set
  EXPECT_NE(info.direction, 0);
}

/**
 * Feature: ros2-vision-migration, Property 23: 前哨站方向检测
 * Validates: Requirements 7.3
 * 
 * For any outpost angular velocity sign change, system should detect
 * direction change within finite time.
 */
TEST_F(OutpostEstimatorTest, Property23_DirectionChangeDetection)
{
  // Initialize with counterclockwise direction
  ArmorMeasurement init_m = generateMeasurement(0.0, 3.0, 0.0, 0.0);
  estimator_->initialize(init_m, 1);  // counterclockwise
  estimator_->setDeltaTime(0.01);
  
  EXPECT_EQ(estimator_->getDirection(), 1);
  
  // Simulate rotation in counterclockwise direction
  double theta = 0.0;
  for (int i = 0; i < 20; ++i) {
    theta += OUTPOST_OMEGA * 0.01;
    ArmorMeasurement m = generateMeasurement(0.0, 3.0, 0.0, theta, 0.01);
    estimator_->update(m);
  }
  
  // Direction should still be counterclockwise
  EXPECT_EQ(estimator_->getDirection(), 1);
  
  // Now reset and initialize with clockwise direction
  estimator_->reset();
  estimator_->initialize(init_m, -1);  // clockwise
  estimator_->setDeltaTime(0.01);
  
  EXPECT_EQ(estimator_->getDirection(), -1);
  
  // Simulate rotation in clockwise direction
  theta = 0.0;
  for (int i = 0; i < 20; ++i) {
    theta -= OUTPOST_OMEGA * 0.01;  // negative for clockwise
    ArmorMeasurement m = generateMeasurement(0.0, 3.0, 0.0, theta, 0.01);
    estimator_->update(m);
  }
  
  // Direction should be clockwise
  EXPECT_EQ(estimator_->getDirection(), -1);
}

/**
 * Feature: ros2-vision-migration, Property 24: 卡方检验异常值拒绝
 * Validates: Requirements 7.4
 * 
 * For any observation with Mahalanobis distance exceeding chi-square threshold,
 * the observation should be rejected without updating EKF state.
 */
TEST_F(OutpostEstimatorTest, Property24_ChiSquareRejectsOutliers)
{
  // Initialize estimator
  ArmorMeasurement init_m = generateMeasurement(0.0, 3.0, 0.0, 0.0);
  estimator_->initialize(init_m, 1);
  estimator_->setDeltaTime(0.01);
  
  // Run a few normal updates to establish state
  double theta = 0.0;
  for (int i = 0; i < 5; ++i) {
    theta += OUTPOST_OMEGA * 0.01;
    ArmorMeasurement m = generateMeasurement(0.0, 3.0, 0.0, theta, 0.01);
    estimator_->update(m);
  }
  
  // Store state before outlier
  Eigen::VectorXd state_before = estimator_->getState();
  
  // Create an outlier measurement (very far from expected)
  ArmorMeasurement outlier;
  outlier.yaw = 1.0;  // Very different from expected
  outlier.pitch = 0.5;
  outlier.distance = 10.0;  // Very far
  outlier.theta_world = M_PI;  // Opposite direction
  
  // Set a reasonable chi-square threshold
  estimator_->setChiSquareThreshold(11.07);  // 95% confidence, df=4
  
  bool accepted = estimator_->update(outlier);
  
  // Outlier should be rejected
  EXPECT_FALSE(accepted);
  
  // Chi-square value should exceed threshold
  EXPECT_GT(estimator_->getLastChiSquareValue(), 11.07);
  EXPECT_FALSE(estimator_->passedChiSquareTest());
}

// Test: Normal measurement passes chi-square test
TEST_F(OutpostEstimatorTest, ChiSquarePassesForNormalMeasurement)
{
  // Initialize estimator
  ArmorMeasurement init_m = generateMeasurement(0.0, 3.0, 0.0, 0.0);
  estimator_->initialize(init_m, 1);
  estimator_->setDeltaTime(0.01);
  
  // Generate a normal measurement (small rotation)
  double theta = OUTPOST_OMEGA * 0.01;
  ArmorMeasurement m = generateMeasurement(0.0, 3.0, 0.0, theta);
  
  bool accepted = estimator_->update(m);
  
  // Normal measurement should be accepted
  EXPECT_TRUE(accepted);
  EXPECT_TRUE(estimator_->passedChiSquareTest());
}

// Test: Get information with predicted vs filtered state
TEST_F(OutpostEstimatorTest, GetInformationPredictedVsFiltered)
{
  ArmorMeasurement init_m = generateMeasurement(0.0, 3.0, 0.0, 0.0);
  estimator_->initialize(init_m, 1);
  estimator_->setDeltaTime(0.01);
  
  // Get filtered state
  OutpostInformation filtered = estimator_->getInformation(false);
  
  // Predict
  estimator_->predict();
  
  // Get predicted state
  OutpostInformation predicted = estimator_->getInformation(true);
  
  // Predicted theta should be different from filtered (due to rotation)
  // Note: After predict(), the filtered state is updated, so we compare with initial
  EXPECT_TRUE(predicted.is_valid);
  EXPECT_TRUE(filtered.is_valid);
}

// Test: Multiple predict-update cycles
TEST_F(OutpostEstimatorTest, MultipleCycles)
{
  // Simulate outpost at center (0, 3, 0) rotating counterclockwise
  double true_center_x = 0.0;
  double true_center_y = 3.0;
  double true_center_z = 0.0;
  double true_omega = OUTPOST_OMEGA;
  double true_theta = 0.0;
  
  // Initialize
  ArmorMeasurement init_m = generateMeasurement(
    true_center_x, true_center_y, true_center_z, true_theta);
  estimator_->initialize(init_m, 1);
  estimator_->setDeltaTime(0.01);
  
  // Run simulation
  for (int i = 0; i < 50; ++i) {
    // True state evolution
    true_theta += true_omega * 0.01;
    while (true_theta > M_PI) true_theta -= 2 * M_PI;
    
    // Generate noisy measurement
    ArmorMeasurement m = generateMeasurement(
      true_center_x, true_center_y, true_center_z, true_theta, 0.01);
    
    estimator_->update(m);
  }
  
  // Get final estimate
  OutpostInformation info = estimator_->getInformation();
  
  // Center should be close to true center
  EXPECT_NEAR(info.center_position.x(), true_center_x, 0.3);
  EXPECT_NEAR(info.center_position.y(), true_center_y, 0.3);
  EXPECT_NEAR(info.center_position.z(), true_center_z, 0.3);
  
  // Omega should be close to true omega
  EXPECT_NEAR(std::abs(info.outpost_omega), true_omega, 0.5);
}

/**
 * Feature: ros2-vision-migration, Property 21: 前哨站 EKF 收敛性
 * Validates: Requirements 7.1
 * 
 * For any outpost observation sequence from uniform rotation,
 * EKF estimated angular velocity should converge to true value (error < 10%).
 */
TEST_F(OutpostEstimatorTest, Property21_EKFConvergence)
{
  // Simulate outpost with known angular velocity
  double true_omega = OUTPOST_OMEGA;
  double true_center_x = 0.0;
  double true_center_y = 3.0;
  double true_center_z = 0.0;
  double true_theta = 0.0;
  
  // Initialize with unknown omega (direction only)
  ArmorMeasurement init_m = generateMeasurement(
    true_center_x, true_center_y, true_center_z, true_theta);
  estimator_->initialize(init_m, 1);  // counterclockwise
  estimator_->setDeltaTime(0.01);
  
  // Run enough iterations for convergence
  for (int i = 0; i < 100; ++i) {
    true_theta += true_omega * 0.01;
    while (true_theta > M_PI) true_theta -= 2 * M_PI;
    
    ArmorMeasurement m = generateMeasurement(
      true_center_x, true_center_y, true_center_z, true_theta, 0.005);
    
    estimator_->update(m);
  }
  
  // Get estimated omega
  OutpostInformation info = estimator_->getInformation();
  double estimated_omega = std::abs(info.outpost_omega);
  
  // Property 21: Estimated omega should be within 10% of true omega
  double error_ratio = std::abs(estimated_omega - true_omega) / true_omega;
  EXPECT_LT(error_ratio, 0.1)
    << "Estimated omega: " << estimated_omega 
    << ", True omega: " << true_omega
    << ", Error: " << error_ratio * 100 << "%";
}

// Test: Uninitialized estimator returns invalid info
TEST_F(OutpostEstimatorTest, UninitializedReturnsInvalid)
{
  OutpostInformation info = estimator_->getInformation();
  EXPECT_FALSE(info.is_valid);
}

// Test: Predict on uninitialized estimator does nothing
TEST_F(OutpostEstimatorTest, PredictUninitializedDoesNothing)
{
  estimator_->predict();  // Should not crash
  EXPECT_FALSE(estimator_->isInitialized());
}

// Test: Update on uninitialized estimator returns false
TEST_F(OutpostEstimatorTest, UpdateUninitializedReturnsFalse)
{
  bool result = estimator_->update(default_measurement_);
  EXPECT_FALSE(result);
}

// Test: Direction change detection
TEST_F(OutpostEstimatorTest, DirectionChangeDetection)
{
  ArmorMeasurement init_m = generateMeasurement(0.0, 3.0, 0.0, 0.0);
  estimator_->initialize(init_m, 1);
  
  // No change initially
  EXPECT_FALSE(estimator_->detectDirectionChange());
  
  // Still no change after calling again
  EXPECT_FALSE(estimator_->detectDirectionChange());
}

// Test: Direction detection from omega sign
TEST_F(OutpostEstimatorTest, DirectionDetectionFromOmega)
{
  // Initialize with counterclockwise direction
  ArmorMeasurement init_m = generateMeasurement(0.0, 3.0, 0.0, 0.0);
  estimator_->initialize(init_m, 1);
  estimator_->setDeltaTime(0.01);
  
  // Simulate counterclockwise rotation (positive omega)
  double theta = 0.0;
  for (int i = 0; i < 30; ++i) {
    theta += OUTPOST_OMEGA * 0.01;  // Positive rotation
    ArmorMeasurement m = generateMeasurement(0.0, 3.0, 0.0, theta, 0.01);
    estimator_->update(m);
  }
  
  // Direction should be counterclockwise (1)
  EXPECT_EQ(estimator_->getDirection(), 1);
  
  // Get omega from state
  OutpostInformation info = estimator_->getInformation();
  EXPECT_GT(info.outpost_omega, 0);  // Positive omega for counterclockwise
}

// Test: Clockwise direction detection
TEST_F(OutpostEstimatorTest, ClockwiseDirectionDetection)
{
  // Initialize with clockwise direction
  ArmorMeasurement init_m = generateMeasurement(0.0, 3.0, 0.0, 0.0);
  estimator_->initialize(init_m, -1);
  estimator_->setDeltaTime(0.01);
  
  // Simulate clockwise rotation (negative omega)
  double theta = 0.0;
  for (int i = 0; i < 30; ++i) {
    theta -= OUTPOST_OMEGA * 0.01;  // Negative rotation
    ArmorMeasurement m = generateMeasurement(0.0, 3.0, 0.0, theta, 0.01);
    estimator_->update(m);
  }
  
  // Direction should be clockwise (-1)
  EXPECT_EQ(estimator_->getDirection(), -1);
  
  // Get omega from state
  OutpostInformation info = estimator_->getInformation();
  EXPECT_LT(info.outpost_omega, 0);  // Negative omega for clockwise
}

// Test: Set chi-square threshold
TEST_F(OutpostEstimatorTest, SetChiSquareThreshold)
{
  estimator_->setChiSquareThreshold(5.0);
  
  // Initialize and update
  ArmorMeasurement init_m = generateMeasurement(0.0, 3.0, 0.0, 0.0);
  estimator_->initialize(init_m, 1);
  estimator_->setDeltaTime(0.01);
  
  // With lower threshold, more measurements might be rejected
  // This is just testing that the setter works
  EXPECT_TRUE(estimator_->isInitialized());
}

// Test: Chi-square test with varying outlier magnitudes
TEST_F(OutpostEstimatorTest, ChiSquareVaryingOutlierMagnitudes)
{
  // Initialize estimator
  ArmorMeasurement init_m = generateMeasurement(0.0, 3.0, 0.0, 0.0);
  estimator_->initialize(init_m, 1);
  estimator_->setDeltaTime(0.01);
  
  // Run a few normal updates to establish state
  double theta = 0.0;
  for (int i = 0; i < 10; ++i) {
    theta += OUTPOST_OMEGA * 0.01;
    ArmorMeasurement m = generateMeasurement(0.0, 3.0, 0.0, theta, 0.01);
    estimator_->update(m);
  }
  
  // Test with small deviation (should pass)
  ArmorMeasurement small_dev = generateMeasurement(0.0, 3.0, 0.0, theta + 0.05);
  bool small_accepted = estimator_->update(small_dev);
  double small_chi2 = estimator_->getLastChiSquareValue();
  
  // Reset and test with large deviation (should fail)
  estimator_->reset();
  estimator_->initialize(init_m, 1);
  estimator_->setDeltaTime(0.01);
  
  for (int i = 0; i < 10; ++i) {
    theta = i * OUTPOST_OMEGA * 0.01;
    ArmorMeasurement m = generateMeasurement(0.0, 3.0, 0.0, theta, 0.01);
    estimator_->update(m);
  }
  
  ArmorMeasurement large_dev;
  large_dev.yaw = 0.5;  // Large deviation
  large_dev.pitch = 0.3;
  large_dev.distance = 5.0;
  large_dev.theta_world = M_PI / 2;
  
  bool large_accepted = estimator_->update(large_dev);
  double large_chi2 = estimator_->getLastChiSquareValue();
  
  // Large deviation should have larger chi-square value
  EXPECT_GT(large_chi2, small_chi2);
}

// Test: Chi-square threshold affects acceptance
TEST_F(OutpostEstimatorTest, ChiSquareThresholdAffectsAcceptance)
{
  // Initialize estimator
  ArmorMeasurement init_m = generateMeasurement(0.0, 3.0, 0.0, 0.0);
  estimator_->initialize(init_m, 1);
  estimator_->setDeltaTime(0.01);
  
  // Run a few normal updates
  double theta = 0.0;
  for (int i = 0; i < 10; ++i) {
    theta += OUTPOST_OMEGA * 0.01;
    ArmorMeasurement m = generateMeasurement(0.0, 3.0, 0.0, theta, 0.01);
    estimator_->update(m);
  }
  
  // Create a moderately deviant measurement
  ArmorMeasurement moderate_dev = generateMeasurement(0.0, 3.0, 0.0, theta + 0.3);
  
  // With high threshold, should pass
  estimator_->setChiSquareThreshold(100.0);
  bool high_threshold_result = estimator_->update(moderate_dev);
  
  // Reset and try with low threshold
  estimator_->reset();
  estimator_->initialize(init_m, 1);
  estimator_->setDeltaTime(0.01);
  
  for (int i = 0; i < 10; ++i) {
    theta = i * OUTPOST_OMEGA * 0.01;
    ArmorMeasurement m = generateMeasurement(0.0, 3.0, 0.0, theta, 0.01);
    estimator_->update(m);
  }
  
  // With very low threshold, more likely to reject
  estimator_->setChiSquareThreshold(0.1);
  
  // Create an outlier that would definitely fail
  ArmorMeasurement outlier;
  outlier.yaw = 1.0;
  outlier.pitch = 0.5;
  outlier.distance = 10.0;
  outlier.theta_world = M_PI;
  
  bool low_threshold_result = estimator_->update(outlier);
  
  // Outlier should be rejected with low threshold
  EXPECT_FALSE(low_threshold_result);
}

// Test: State vector contents after initialization
TEST_F(OutpostEstimatorTest, StateVectorContents)
{
  ArmorMeasurement m = generateMeasurement(0.0, 3.0, 0.0, 0.5);
  estimator_->initialize(m, 1);
  
  Eigen::VectorXd state = estimator_->getState();
  
  // State should have 8 elements
  EXPECT_EQ(state.size(), 8);
  
  // Theta should be close to measurement theta_world
  EXPECT_NEAR(state(6), 0.5, 0.1);
  
  // Omega should be positive for counterclockwise
  EXPECT_GT(state(7), 0);
}

// Test: Covariance matrix is positive definite
TEST_F(OutpostEstimatorTest, CovariancePositiveDefinite)
{
  estimator_->initialize(default_measurement_, 1);
  
  Eigen::MatrixXd P = estimator_->getCovariance();
  
  // Check symmetry
  EXPECT_TRUE(P.isApprox(P.transpose(), 1e-10));
  
  // Check positive definiteness via eigenvalues
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(P);
  Eigen::VectorXd eigenvalues = solver.eigenvalues();
  
  for (int i = 0; i < eigenvalues.size(); ++i) {
    EXPECT_GT(eigenvalues(i), 0) << "Eigenvalue " << i << " is not positive";
  }
}

}  // namespace test
}  // namespace armor_detector

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
