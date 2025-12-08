/**
 * @file test_mock_imu.cpp
 * @brief MockIMUNode 单元测试
 * 
 * 测试内容：
 * 1. 四元数格式正确性（单位四元数）
 * 2. 发布频率
 * 
 * Requirements: 9.3
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

/**
 * @brief Test fixture for MockIMU tests
 */
class MockIMUTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    void TearDown() override {
        // Don't shutdown rclcpp here as other tests may need it
    }

    /**
     * @brief Calculate quaternion norm
     */
    double quaternion_norm(const geometry_msgs::msg::Quaternion& q) {
        return std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    }
};

/**
 * @brief Test quaternion format correctness
 * 
 * Verifies that received quaternions are unit quaternions:
 * - Norm should be approximately 1.0
 * - Components should be valid (not NaN or Inf)
 */
TEST_F(MockIMUTest, QuaternionFormatCorrectness) {
    auto node = rclcpp::Node::make_shared("test_quaternion_format_node");
    
    std::atomic<bool> message_received{false};
    geometry_msgs::msg::QuaternionStamped::SharedPtr received_msg;
    
    // Create subscriber to /imu/quaternion
    auto subscription = node->create_subscription<geometry_msgs::msg::QuaternionStamped>(
        "/imu/quaternion", 10,
        [&message_received, &received_msg](const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
            received_msg = msg;
            message_received = true;
        });
    
    // Wait for a message
    auto start = std::chrono::steady_clock::now();
    auto timeout = 5s;
    
    while (!message_received && 
           (std::chrono::steady_clock::now() - start) < timeout) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(10ms);
    }
    
    if (!message_received) {
        GTEST_SKIP() << "No message received from /imu/quaternion. "
                     << "Make sure mock_imu_node is running.";
    }
    
    // Verify quaternion format
    ASSERT_NE(received_msg, nullptr) << "Received message is null";
    
    const auto& q = received_msg->quaternion;
    
    // Check for NaN or Inf
    EXPECT_FALSE(std::isnan(q.w)) << "Quaternion w is NaN";
    EXPECT_FALSE(std::isnan(q.x)) << "Quaternion x is NaN";
    EXPECT_FALSE(std::isnan(q.y)) << "Quaternion y is NaN";
    EXPECT_FALSE(std::isnan(q.z)) << "Quaternion z is NaN";
    
    EXPECT_FALSE(std::isinf(q.w)) << "Quaternion w is Inf";
    EXPECT_FALSE(std::isinf(q.x)) << "Quaternion x is Inf";
    EXPECT_FALSE(std::isinf(q.y)) << "Quaternion y is Inf";
    EXPECT_FALSE(std::isinf(q.z)) << "Quaternion z is Inf";
    
    // Check unit quaternion (norm should be ~1.0)
    double norm = quaternion_norm(q);
    EXPECT_NEAR(norm, 1.0, 0.001) << "Quaternion should be unit quaternion (norm = 1.0)";
    
    // Check header
    EXPECT_FALSE(received_msg->header.frame_id.empty()) 
        << "Frame ID should not be empty";
    
    // Verify timestamp is reasonable (not zero)
    EXPECT_GT(received_msg->header.stamp.sec + received_msg->header.stamp.nanosec, 0u)
        << "Timestamp should be non-zero";
    
    std::cout << "Received quaternion: w=" << q.w << ", x=" << q.x 
              << ", y=" << q.y << ", z=" << q.z
              << ", norm=" << norm
              << ", frame_id: " << received_msg->header.frame_id << std::endl;
}

/**
 * @brief Test that multiple quaternions are all unit quaternions
 * 
 * Verifies consistency across multiple messages
 */
TEST_F(MockIMUTest, MultipleQuaternionsAreUnit) {
    auto node = rclcpp::Node::make_shared("test_multiple_quaternions_node");
    
    std::vector<geometry_msgs::msg::QuaternionStamped> received_msgs;
    std::atomic<int> message_count{0};
    const int target_messages = 20;
    
    // Create subscriber to /imu/quaternion
    auto subscription = node->create_subscription<geometry_msgs::msg::QuaternionStamped>(
        "/imu/quaternion", 10,
        [&received_msgs, &message_count](const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
            received_msgs.push_back(*msg);
            message_count++;
        });
    
    // Wait for enough messages
    auto start = std::chrono::steady_clock::now();
    auto timeout = 5s;
    
    while (message_count < target_messages && 
           (std::chrono::steady_clock::now() - start) < timeout) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(5ms);
    }
    
    if (message_count < target_messages) {
        GTEST_SKIP() << "Not enough messages received from /imu/quaternion. "
                     << "Received: " << message_count << ", expected: " << target_messages
                     << ". Make sure mock_imu_node is running.";
    }
    
    // Verify all quaternions are unit quaternions
    for (size_t i = 0; i < received_msgs.size(); ++i) {
        const auto& q = received_msgs[i].quaternion;
        double norm = quaternion_norm(q);
        EXPECT_NEAR(norm, 1.0, 0.001) 
            << "Quaternion " << i << " should be unit quaternion, got norm=" << norm;
    }
    
    std::cout << "Verified " << received_msgs.size() << " quaternions are all unit quaternions" << std::endl;
}

/**
 * @brief Test IMU publishing frequency
 * 
 * Verifies that quaternions are published at approximately the configured rate
 */
TEST_F(MockIMUTest, QuaternionPublishingFrequency) {
    auto node = rclcpp::Node::make_shared("test_imu_frequency_node");
    
    std::vector<rclcpp::Time> timestamps;
    std::atomic<int> message_count{0};
    const int target_messages = 50;
    
    // Create subscriber to /imu/quaternion
    auto subscription = node->create_subscription<geometry_msgs::msg::QuaternionStamped>(
        "/imu/quaternion", 50,
        [&timestamps, &message_count, &node](const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
            timestamps.push_back(node->get_clock()->now());
            message_count++;
        });
    
    // Wait for enough messages
    auto start = std::chrono::steady_clock::now();
    auto timeout = 5s;
    
    while (message_count < target_messages && 
           (std::chrono::steady_clock::now() - start) < timeout) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(2ms);
    }
    
    if (message_count < target_messages) {
        GTEST_SKIP() << "Not enough messages received from /imu/quaternion. "
                     << "Received: " << message_count << ", expected: " << target_messages
                     << ". Make sure mock_imu_node is running.";
    }
    
    // Calculate average frequency
    if (timestamps.size() >= 2) {
        double total_duration = 0.0;
        for (size_t i = 1; i < timestamps.size(); ++i) {
            double dt = (timestamps[i] - timestamps[i-1]).seconds();
            total_duration += dt;
        }
        double avg_period = total_duration / (timestamps.size() - 1);
        double avg_frequency = 1.0 / avg_period;
        
        std::cout << "Average IMU publishing frequency: " << avg_frequency << " Hz" << std::endl;
        
        // Expect frequency to be within reasonable range (50-500 Hz for IMU)
        EXPECT_GT(avg_frequency, 30.0) << "IMU publishing frequency too low";
        EXPECT_LT(avg_frequency, 500.0) << "IMU publishing frequency too high";
    }
}

/**
 * @brief Test quaternion timestamp monotonicity
 * 
 * Verifies that timestamps are monotonically increasing
 */
TEST_F(MockIMUTest, TimestampMonotonicity) {
    auto node = rclcpp::Node::make_shared("test_timestamp_monotonicity_node");
    
    std::vector<rclcpp::Time> timestamps;
    std::atomic<int> message_count{0};
    const int target_messages = 20;
    
    // Create subscriber to /imu/quaternion
    auto subscription = node->create_subscription<geometry_msgs::msg::QuaternionStamped>(
        "/imu/quaternion", 10,
        [&timestamps, &message_count](const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
            timestamps.push_back(rclcpp::Time(msg->header.stamp));
            message_count++;
        });
    
    // Wait for enough messages
    auto start = std::chrono::steady_clock::now();
    auto timeout = 5s;
    
    while (message_count < target_messages && 
           (std::chrono::steady_clock::now() - start) < timeout) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(5ms);
    }
    
    if (message_count < target_messages) {
        GTEST_SKIP() << "Not enough messages received from /imu/quaternion. "
                     << "Make sure mock_imu_node is running.";
    }
    
    // Verify timestamps are monotonically increasing
    for (size_t i = 1; i < timestamps.size(); ++i) {
        EXPECT_GE(timestamps[i].nanoseconds(), timestamps[i-1].nanoseconds())
            << "Timestamp at index " << i << " is not monotonically increasing";
    }
    
    std::cout << "Verified " << timestamps.size() << " timestamps are monotonically increasing" << std::endl;
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
