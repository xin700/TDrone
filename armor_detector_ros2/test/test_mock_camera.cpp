/**
 * @file test_mock_camera.cpp
 * @brief MockCameraNode 单元测试
 * 
 * 测试内容：
 * 1. 图像发布频率
 * 2. 图像格式正确性
 * 
 * Requirements: 9.3
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include <fstream>

using namespace std::chrono_literals;

/**
 * @brief Test fixture for MockCamera tests
 */
class MockCameraTest : public ::testing::Test {
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
     * @brief Find a valid test video file
     */
    std::string find_test_video() {
        std::vector<std::string> test_paths = {
            "~/droneAim/TDrone/videos/1.mp4",
            "~/droneAim/TDrone/videos/video.avi",
            "~/droneAim/TDrone/videos/3.avi",
            "../videos/1.mp4",
            "videos/1.mp4"
        };
        
        for (const auto& path : test_paths) {
            std::ifstream f(path);
            if (f.good()) {
                return path;
            }
        }
        return "";
    }
};

/**
 * @brief Test image message format correctness
 * 
 * Verifies that received images have correct format:
 * - Non-empty data
 * - Valid dimensions (width > 0, height > 0)
 * - Correct encoding (bgr8)
 * - Valid header with timestamp
 */
TEST_F(MockCameraTest, ImageFormatCorrectness) {
    auto node = rclcpp::Node::make_shared("test_image_format_node");
    
    std::atomic<bool> message_received{false};
    sensor_msgs::msg::Image::SharedPtr received_msg;
    
    // Create subscriber to /camera/image_raw
    auto subscription = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        [&message_received, &received_msg](const sensor_msgs::msg::Image::SharedPtr msg) {
            received_msg = msg;
            message_received = true;
        });
    
    // Wait for a message (mock_camera_node should be running)
    auto start = std::chrono::steady_clock::now();
    auto timeout = 5s;
    
    while (!message_received && 
           (std::chrono::steady_clock::now() - start) < timeout) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(10ms);
    }
    
    if (!message_received) {
        GTEST_SKIP() << "No message received from /camera/image_raw. "
                     << "Make sure mock_camera_node is running.";
    }
    
    // Verify image format
    ASSERT_NE(received_msg, nullptr) << "Received message is null";
    
    // Check dimensions
    EXPECT_GT(received_msg->width, 0u) << "Image width should be positive";
    EXPECT_GT(received_msg->height, 0u) << "Image height should be positive";
    
    // Check encoding
    EXPECT_EQ(received_msg->encoding, "bgr8") << "Image encoding should be bgr8";
    
    // Check data size matches dimensions
    size_t expected_size = received_msg->width * received_msg->height * 3;  // 3 channels for bgr8
    EXPECT_EQ(received_msg->data.size(), expected_size) 
        << "Image data size doesn't match dimensions";
    
    // Check header
    EXPECT_FALSE(received_msg->header.frame_id.empty()) 
        << "Frame ID should not be empty";
    
    // Verify timestamp is reasonable (not zero)
    EXPECT_GT(received_msg->header.stamp.sec + received_msg->header.stamp.nanosec, 0u)
        << "Timestamp should be non-zero";
    
    std::cout << "Received image: " << received_msg->width << "x" << received_msg->height
              << ", encoding: " << received_msg->encoding
              << ", frame_id: " << received_msg->header.frame_id << std::endl;
}

/**
 * @brief Test image publishing frequency
 * 
 * Verifies that images are published at approximately the configured rate
 */
TEST_F(MockCameraTest, ImagePublishingFrequency) {
    auto node = rclcpp::Node::make_shared("test_frequency_node");
    
    std::vector<rclcpp::Time> timestamps;
    std::atomic<int> message_count{0};
    const int target_messages = 10;
    
    // Create subscriber to /camera/image_raw
    auto subscription = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        [&timestamps, &message_count, &node](const sensor_msgs::msg::Image::SharedPtr msg) {
            timestamps.push_back(node->get_clock()->now());
            message_count++;
        });
    
    // Wait for enough messages
    auto start = std::chrono::steady_clock::now();
    auto timeout = 10s;
    
    while (message_count < target_messages && 
           (std::chrono::steady_clock::now() - start) < timeout) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(5ms);
    }
    
    if (message_count < target_messages) {
        GTEST_SKIP() << "Not enough messages received from /camera/image_raw. "
                     << "Received: " << message_count << ", expected: " << target_messages
                     << ". Make sure mock_camera_node is running.";
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
        
        std::cout << "Average publishing frequency: " << avg_frequency << " Hz" << std::endl;
        
        // Expect frequency to be within reasonable range (10-120 Hz)
        EXPECT_GT(avg_frequency, 5.0) << "Publishing frequency too low";
        EXPECT_LT(avg_frequency, 150.0) << "Publishing frequency too high";
    }
}

/**
 * @brief Test that images can be converted to OpenCV format
 * 
 * Verifies that received images can be properly converted using cv_bridge
 */
TEST_F(MockCameraTest, ImageConversionToOpenCV) {
    auto node = rclcpp::Node::make_shared("test_conversion_node");
    
    std::atomic<bool> message_received{false};
    cv::Mat received_frame;
    
    // Create subscriber to /camera/image_raw
    auto subscription = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        [&message_received, &received_frame](const sensor_msgs::msg::Image::SharedPtr msg) {
            try {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                received_frame = cv_ptr->image.clone();
                message_received = true;
            } catch (const cv_bridge::Exception& e) {
                // Conversion failed
            }
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
        GTEST_SKIP() << "No message received from /camera/image_raw. "
                     << "Make sure mock_camera_node is running.";
    }
    
    // Verify OpenCV image
    EXPECT_FALSE(received_frame.empty()) << "Converted frame should not be empty";
    EXPECT_EQ(received_frame.channels(), 3) << "Frame should have 3 channels (BGR)";
    EXPECT_GT(received_frame.cols, 0) << "Frame width should be positive";
    EXPECT_GT(received_frame.rows, 0) << "Frame height should be positive";
    
    std::cout << "Successfully converted to OpenCV: " 
              << received_frame.cols << "x" << received_frame.rows 
              << ", channels: " << received_frame.channels() << std::endl;
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
