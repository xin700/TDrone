/**
 * @file test_environment.cpp
 * @brief Environment verification tests for ROS2 Vision Migration
 * 
 * Tests:
 * 1. ROS2 node communication
 * 2. OpenVINO inference capability
 * 3. Video file reading with OpenCV
 * 
 * Requirements: 8.1
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <atomic>
#include <fstream>

using namespace std::chrono_literals;

/**
 * @brief Test fixture for environment verification
 */
class EnvironmentTest : public ::testing::Test {
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
 * @brief Test ROS2 node creation and basic functionality
 */
TEST_F(EnvironmentTest, ROS2NodeCreation) {
    auto node = rclcpp::Node::make_shared("test_node");
    ASSERT_NE(node, nullptr) << "Failed to create ROS2 node";
    EXPECT_STREQ(node->get_name(), "test_node");
}


/**
 * @brief Test ROS2 publisher-subscriber communication
 */
TEST_F(EnvironmentTest, ROS2PubSubCommunication) {
    auto node = rclcpp::Node::make_shared("test_pubsub_node");
    
    std::atomic<bool> message_received{false};
    std::string received_data;
    
    // Create subscriber
    auto subscription = node->create_subscription<std_msgs::msg::String>(
        "test_topic", 10,
        [&message_received, &received_data](const std_msgs::msg::String::SharedPtr msg) {
            received_data = msg->data;
            message_received = true;
        });
    
    // Create publisher
    auto publisher = node->create_publisher<std_msgs::msg::String>("test_topic", 10);
    
    // Publish a test message
    auto message = std_msgs::msg::String();
    message.data = "Hello ROS2";
    
    // Spin for a short time to allow message delivery
    auto start = std::chrono::steady_clock::now();
    auto timeout = 2s;
    
    while (!message_received && 
           (std::chrono::steady_clock::now() - start) < timeout) {
        publisher->publish(message);
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(10ms);
    }
    
    ASSERT_TRUE(message_received) << "Message was not received within timeout";
    EXPECT_EQ(received_data, "Hello ROS2");
}

/**
 * @brief Test OpenVINO runtime initialization
 */
TEST_F(EnvironmentTest, OpenVINOInitialization) {
    ov::Core core;
    
    // Get available devices
    auto devices = core.get_available_devices();
    ASSERT_FALSE(devices.empty()) << "No OpenVINO devices available";
    
    // Check that CPU device is available (always should be)
    bool cpu_found = false;
    for (const auto& device : devices) {
        if (device.find("CPU") != std::string::npos) {
            cpu_found = true;
            break;
        }
    }
    EXPECT_TRUE(cpu_found) << "CPU device not found in OpenVINO";
    
    // Print available devices for debugging
    std::cout << "Available OpenVINO devices: ";
    for (const auto& device : devices) {
        std::cout << device << " ";
    }
    std::cout << std::endl;
}

/**
 * @brief Test OpenVINO model compilation capability using existing model file
 */
TEST_F(EnvironmentTest, OpenVINOModelCompilation) {
    ov::Core core;
    
    // Try to load an existing model file from the models directory
    std::vector<std::string> model_paths = {
        "/ros2_ws/src/armor_detector_ros2/models/BRpoints_nano.xml",
        "src/armor_detector_ros2/models/BRpoints_nano.xml",
        "../models/BRpoints_nano.xml"
    };
    
    std::string found_model;
    for (const auto& path : model_paths) {
        std::ifstream f(path);
        if (f.good()) {
            found_model = path;
            break;
        }
    }
    
    if (found_model.empty()) {
        // Skip if no model file found, but still verify core works
        std::cout << "No model file found, skipping model compilation test" << std::endl;
        GTEST_SKIP() << "No model file found for compilation test";
    }
    
    // Try to read and compile the model on CPU
    ASSERT_NO_THROW({
        auto model = core.read_model(found_model);
        auto compiled_model = core.compile_model(model, "CPU");
        EXPECT_TRUE(compiled_model);
        std::cout << "Successfully compiled model: " << found_model << std::endl;
    }) << "Failed to compile model on CPU";
}

/**
 * @brief Test OpenCV video file reading
 */
TEST_F(EnvironmentTest, OpenCVVideoReading) {
    // Test with a video file if available
    std::vector<std::string> test_paths = {
        "/ros2_ws/videos/1.mp4",
        "/ros2_ws/videos/video.avi",
        "../videos/1.mp4",
        "videos/1.mp4"
    };
    
    cv::VideoCapture cap;
    std::string found_path;
    
    for (const auto& path : test_paths) {
        cap.open(path);
        if (cap.isOpened()) {
            found_path = path;
            break;
        }
    }
    
    if (!cap.isOpened()) {
        GTEST_SKIP() << "No test video file found. Skipping video read test.";
    }
    
    // Read a frame
    cv::Mat frame;
    bool success = cap.read(frame);
    
    ASSERT_TRUE(success) << "Failed to read frame from video: " << found_path;
    EXPECT_FALSE(frame.empty()) << "Read frame is empty";
    EXPECT_GT(frame.cols, 0) << "Frame width should be positive";
    EXPECT_GT(frame.rows, 0) << "Frame height should be positive";
    
    std::cout << "Successfully read video: " << found_path 
              << " (" << frame.cols << "x" << frame.rows << ")" << std::endl;
    
    cap.release();
}

/**
 * @brief Test OpenCV image creation and basic operations
 */
TEST_F(EnvironmentTest, OpenCVBasicOperations) {
    // Create a test image
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 255));  // Red image
    
    EXPECT_EQ(image.cols, 640);
    EXPECT_EQ(image.rows, 480);
    EXPECT_EQ(image.channels(), 3);
    
    // Test color conversion
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    EXPECT_EQ(gray.channels(), 1);
    
    // Test resize
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(320, 240));
    EXPECT_EQ(resized.cols, 320);
    EXPECT_EQ(resized.rows, 240);
}

/**
 * @brief Test OpenCV video writing capability (no GUI required)
 */
TEST_F(EnvironmentTest, OpenCVVideoWriting) {
    // Output path for test video
    std::vector<std::string> output_paths = {
        "/ros2_ws/output/test_output.avi",
        "./output/test_output.avi",
        "/tmp/test_output.avi"
    };
    
    std::string output_path;
    for (const auto& path : output_paths) {
        // Try to create parent directory
        size_t last_slash = path.find_last_of('/');
        if (last_slash != std::string::npos) {
            std::string dir = path.substr(0, last_slash);
            // Simple directory existence check
            cv::Mat test_img(10, 10, CV_8UC3);
            if (cv::imwrite(dir + "/test_write_check.png", test_img)) {
                std::remove((dir + "/test_write_check.png").c_str());
                output_path = path;
                break;
            }
        }
    }
    
    if (output_path.empty()) {
        output_path = "/tmp/test_output.avi";
    }
    
    // Create video writer
    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    cv::VideoWriter writer(output_path, fourcc, 30.0, cv::Size(640, 480));
    
    ASSERT_TRUE(writer.isOpened()) << "Failed to open video writer at: " << output_path;
    
    // Write some test frames
    for (int i = 0; i < 30; ++i) {
        cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(0, 0, 255));
        
        // Draw frame number
        cv::putText(frame, "Frame " + std::to_string(i), 
                    cv::Point(50, 240), cv::FONT_HERSHEY_SIMPLEX, 
                    2.0, cv::Scalar(255, 255, 255), 3);
        
        // Draw test info
        cv::putText(frame, "Environment Test", 
                    cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 
                    1.0, cv::Scalar(0, 255, 0), 2);
        
        writer.write(frame);
    }
    
    writer.release();
    
    // Verify the video was written by reading it back
    cv::VideoCapture cap(output_path);
    ASSERT_TRUE(cap.isOpened()) << "Failed to read back written video";
    
    int frame_count = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    EXPECT_GE(frame_count, 30) << "Written video has fewer frames than expected";
    
    cap.release();
    
    std::cout << "Successfully wrote test video to: " << output_path 
              << " (" << frame_count << " frames)" << std::endl;
    
    // Clean up test file
    std::remove(output_path.c_str());
}

/**
 * @brief Test Eigen library availability
 */
TEST_F(EnvironmentTest, EigenAvailability) {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation(1.0, 2.0, 3.0);
    
    // Test basic matrix operations
    Eigen::Vector3d result = rotation * translation;
    
    EXPECT_DOUBLE_EQ(result(0), 1.0);
    EXPECT_DOUBLE_EQ(result(1), 2.0);
    EXPECT_DOUBLE_EQ(result(2), 3.0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
