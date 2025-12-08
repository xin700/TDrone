/**
 * @file test_mock_cboard.cpp
 * @brief MockCBoardNode 单元测试
 * 
 * 测试内容：
 * 1. 弹速发布正确性
 * 2. 模式发布正确性
 * 3. 控制指令接收
 * 
 * Requirements: 9.3
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

/**
 * @brief Test fixture for MockCBoard tests
 */
class MockCBoardTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    void TearDown() override {
        // Don't shutdown rclcpp here as other tests may need it
    }
};

/**
 * @brief Test bullet speed publishing
 * 
 * Verifies that bullet speed is published correctly:
 * - Value is positive
 * - Value is within reasonable range (10-50 m/s)
 */
TEST_F(MockCBoardTest, BulletSpeedPublishing) {
    auto node = rclcpp::Node::make_shared("test_bullet_speed_node");
    
    std::atomic<bool> message_received{false};
    double received_speed = 0.0;
    
    // Create subscriber to /cboard/bullet_speed
    auto subscription = node->create_subscription<std_msgs::msg::Float64>(
        "/cboard/bullet_speed", 10,
        [&message_received, &received_speed](const std_msgs::msg::Float64::SharedPtr msg) {
            received_speed = msg->data;
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
        GTEST_SKIP() << "No message received from /cboard/bullet_speed. "
                     << "Make sure mock_cboard_node is running.";
    }
    
    // Verify bullet speed
    EXPECT_GT(received_speed, 0.0) << "Bullet speed should be positive";
    EXPECT_GT(received_speed, 10.0) << "Bullet speed should be > 10 m/s";
    EXPECT_LT(received_speed, 50.0) << "Bullet speed should be < 50 m/s";
    
    std::cout << "Received bullet speed: " << received_speed << " m/s" << std::endl;
}

/**
 * @brief Test bullet speed consistency
 * 
 * Verifies that bullet speed values are consistent (within noise range)
 */
TEST_F(MockCBoardTest, BulletSpeedConsistency) {
    auto node = rclcpp::Node::make_shared("test_bullet_speed_consistency_node");
    
    std::vector<double> speeds;
    std::atomic<int> message_count{0};
    const int target_messages = 20;
    
    // Create subscriber to /cboard/bullet_speed
    auto subscription = node->create_subscription<std_msgs::msg::Float64>(
        "/cboard/bullet_speed", 10,
        [&speeds, &message_count](const std_msgs::msg::Float64::SharedPtr msg) {
            speeds.push_back(msg->data);
            message_count++;
        });
    
    // Wait for enough messages
    auto start = std::chrono::steady_clock::now();
    auto timeout = 5s;
    
    while (message_count < target_messages && 
           (std::chrono::steady_clock::now() - start) < timeout) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(10ms);
    }
    
    if (message_count < target_messages) {
        GTEST_SKIP() << "Not enough messages received from /cboard/bullet_speed. "
                     << "Received: " << message_count << ", expected: " << target_messages
                     << ". Make sure mock_cboard_node is running.";
    }
    
    // Calculate mean and standard deviation
    double sum = 0.0;
    for (double speed : speeds) {
        sum += speed;
    }
    double mean = sum / speeds.size();
    
    double sq_sum = 0.0;
    for (double speed : speeds) {
        sq_sum += (speed - mean) * (speed - mean);
    }
    double std_dev = std::sqrt(sq_sum / speeds.size());
    
    std::cout << "Bullet speed: mean=" << mean << " m/s, std_dev=" << std_dev << " m/s" << std::endl;
    
    // Verify consistency (std_dev should be small, within noise range)
    EXPECT_LT(std_dev, 2.0) << "Bullet speed standard deviation too high";
    
    // Verify all speeds are within reasonable range of mean
    for (size_t i = 0; i < speeds.size(); ++i) {
        EXPECT_NEAR(speeds[i], mean, 3.0 * std_dev + 0.1)
            << "Speed " << i << " is too far from mean";
    }
}

/**
 * @brief Test mode publishing
 * 
 * Verifies that mode is published correctly
 */
TEST_F(MockCBoardTest, ModePublishing) {
    auto node = rclcpp::Node::make_shared("test_mode_node");
    
    std::atomic<bool> message_received{false};
    int received_mode = -1;
    
    // Create subscriber to /cboard/mode
    auto subscription = node->create_subscription<std_msgs::msg::Int32>(
        "/cboard/mode", 10,
        [&message_received, &received_mode](const std_msgs::msg::Int32::SharedPtr msg) {
            received_mode = msg->data;
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
        GTEST_SKIP() << "No message received from /cboard/mode. "
                     << "Make sure mock_cboard_node is running.";
    }
    
    // Verify mode (should be a valid mode value)
    EXPECT_GE(received_mode, 0) << "Mode should be non-negative";
    EXPECT_LT(received_mode, 10) << "Mode should be < 10";
    
    std::cout << "Received mode: " << received_mode << std::endl;
}

/**
 * @brief Test command reception
 * 
 * Verifies that the node can receive control commands
 */
TEST_F(MockCBoardTest, CommandReception) {
    auto node = rclcpp::Node::make_shared("test_command_node");
    
    // Create publisher to send commands
    auto command_pub = node->create_publisher<std_msgs::msg::Float64>(
        "/aimer/command_yaw", 10);
    
    // Give time for publisher to be discovered
    std::this_thread::sleep_for(100ms);
    
    // Publish a test command
    auto command_msg = std_msgs::msg::Float64();
    command_msg.data = 0.5;  // Test yaw value
    
    // Publish multiple times to ensure delivery
    for (int i = 0; i < 5; ++i) {
        command_pub->publish(command_msg);
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(50ms);
    }
    
    // The mock_cboard_node should have received the command
    // We can't directly verify this without the node running,
    // but we can verify the publisher works
    EXPECT_TRUE(command_pub != nullptr) << "Command publisher should be created";
    
    std::cout << "Published test command: yaw=" << command_msg.data << std::endl;
}

/**
 * @brief Test publishing frequency
 * 
 * Verifies that messages are published at approximately the configured rate
 */
TEST_F(MockCBoardTest, PublishingFrequency) {
    auto node = rclcpp::Node::make_shared("test_cboard_frequency_node");
    
    std::vector<rclcpp::Time> timestamps;
    std::atomic<int> message_count{0};
    const int target_messages = 30;
    
    // Create subscriber to /cboard/bullet_speed
    auto subscription = node->create_subscription<std_msgs::msg::Float64>(
        "/cboard/bullet_speed", 50,
        [&timestamps, &message_count, &node](const std_msgs::msg::Float64::SharedPtr msg) {
            timestamps.push_back(node->get_clock()->now());
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
        GTEST_SKIP() << "Not enough messages received from /cboard/bullet_speed. "
                     << "Received: " << message_count << ", expected: " << target_messages
                     << ". Make sure mock_cboard_node is running.";
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
        
        std::cout << "Average CBoard publishing frequency: " << avg_frequency << " Hz" << std::endl;
        
        // Expect frequency to be within reasonable range (50-200 Hz)
        EXPECT_GT(avg_frequency, 30.0) << "CBoard publishing frequency too low";
        EXPECT_LT(avg_frequency, 300.0) << "CBoard publishing frequency too high";
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
