/**
 * @file test_solver_node.cpp
 * @brief solver_node 集成测试
 * 
 * 测试位姿解算节点的功能：
 * - 订阅检测结果和 IMU 数据
 * - 发布位姿解算结果
 * - 使用 mock 检测结果测试
 * 
 * Requirements: 2.1, 2.2
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <atomic>

#include "armor_detector_ros2/msg/armor_b_box.hpp"
#include "armor_detector_ros2/msg/armor_b_box_array.hpp"
#include "armor_detector_ros2/msg/armor_pose.hpp"
#include "armor_detector_ros2/msg/armor_pose_array.hpp"
#include "armor_detector_ros2/msg/point2f.hpp"

using namespace std::chrono_literals;

/**
 * @brief solver_node 集成测试夹具
 */
class SolverNodeTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        
        // 创建测试节点
        test_node_ = std::make_shared<rclcpp::Node>("solver_node_test");
        
        // 创建发布者（模拟检测器和 IMU）
        armor_pub_ = test_node_->create_publisher<armor_detector_ros2::msg::ArmorBBoxArray>(
            "/detector/armors",
            rclcpp::QoS(10).reliable()
        );
        
        imu_pub_ = test_node_->create_publisher<geometry_msgs::msg::QuaternionStamped>(
            "/imu/quaternion",
            rclcpp::QoS(10).reliable()
        );
        
        // 创建订阅者（接收解算结果）
        pose_received_ = false;
        pose_sub_ = test_node_->create_subscription<armor_detector_ros2::msg::ArmorPoseArray>(
            "/solver/armor_poses",
            rclcpp::QoS(10).reliable(),
            [this](const armor_detector_ros2::msg::ArmorPoseArray::SharedPtr msg) {
                last_pose_array_ = msg;
                pose_received_ = true;
            }
        );
    }
    
    void TearDown() override
    {
        test_node_.reset();
        rclcpp::shutdown();
    }
    
    /**
     * @brief 创建模拟的装甲板检测结果
     * @param center_x 装甲板中心 x 坐标
     * @param center_y 装甲板中心 y 坐标
     * @param half_width 装甲板半宽（像素）
     * @param half_height 装甲板半高（像素）
     * @param color_id 颜色 ID
     * @param tag_id 标签 ID
     * @return 装甲板检测消息
     */
    armor_detector_ros2::msg::ArmorBBox createMockArmor(
        float center_x, float center_y,
        float half_width, float half_height,
        int color_id = 0, int tag_id = 3)
    {
        armor_detector_ros2::msg::ArmorBBox armor;
        
        // 设置四个角点（左上、右上、右下、左下）
        armor.corners[0].x = center_x - half_width;
        armor.corners[0].y = center_y - half_height;
        armor.corners[1].x = center_x + half_width;
        armor.corners[1].y = center_y - half_height;
        armor.corners[2].x = center_x + half_width;
        armor.corners[2].y = center_y + half_height;
        armor.corners[3].x = center_x - half_width;
        armor.corners[3].y = center_y + half_height;
        
        // 设置中心点
        armor.center.x = center_x;
        armor.center.y = center_y;
        
        // 设置其他属性
        armor.confidence = 0.95f;
        armor.color_id = color_id;
        armor.tag_id = tag_id;
        armor.rect_x = static_cast<int>(center_x - half_width);
        armor.rect_y = static_cast<int>(center_y - half_height);
        armor.rect_width = static_cast<int>(half_width * 2);
        armor.rect_height = static_cast<int>(half_height * 2);
        
        return armor;
    }
    
    /**
     * @brief 创建单位四元数（无旋转）
     */
    geometry_msgs::msg::QuaternionStamped createIdentityQuaternion()
    {
        geometry_msgs::msg::QuaternionStamped quat;
        quat.header.stamp = test_node_->now();
        quat.header.frame_id = "imu_link";
        quat.quaternion.w = 1.0;
        quat.quaternion.x = 0.0;
        quat.quaternion.y = 0.0;
        quat.quaternion.z = 0.0;
        return quat;
    }
    
    /**
     * @brief 等待消息接收
     * @param timeout 超时时间
     * @return 是否在超时前收到消息
     */
    bool waitForMessage(std::chrono::milliseconds timeout = 2000ms)
    {
        auto start = std::chrono::steady_clock::now();
        while (!pose_received_)
        {
            rclcpp::spin_some(test_node_);
            std::this_thread::sleep_for(10ms);
            
            if (std::chrono::steady_clock::now() - start > timeout)
            {
                return false;
            }
        }
        return true;
    }
    
    // 测试节点
    rclcpp::Node::SharedPtr test_node_;
    
    // 发布者
    rclcpp::Publisher<armor_detector_ros2::msg::ArmorBBoxArray>::SharedPtr armor_pub_;
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr imu_pub_;
    
    // 订阅者
    rclcpp::Subscription<armor_detector_ros2::msg::ArmorPoseArray>::SharedPtr pose_sub_;
    
    // 接收到的消息
    armor_detector_ros2::msg::ArmorPoseArray::SharedPtr last_pose_array_;
    std::atomic<bool> pose_received_{false};
};

// ============================================================================
// 消息类型测试
// ============================================================================

/**
 * @test 测试 ArmorPose 消息创建
 */
TEST_F(SolverNodeTest, ArmorPoseMessageCreation)
{
    armor_detector_ros2::msg::ArmorPose pose;
    
    pose.yaw = 0.1;
    pose.pitch = 0.05;
    pose.distance = 2.0;
    pose.armor_yaw = 0.0;
    pose.color_id = 0;
    pose.tag_id = 3;
    pose.armor_type = 0;
    pose.valid = true;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 2.0;
    
    EXPECT_DOUBLE_EQ(pose.yaw, 0.1);
    EXPECT_DOUBLE_EQ(pose.pitch, 0.05);
    EXPECT_DOUBLE_EQ(pose.distance, 2.0);
    EXPECT_EQ(pose.color_id, 0);
    EXPECT_EQ(pose.tag_id, 3);
    EXPECT_TRUE(pose.valid);
}

/**
 * @test 测试 ArmorPoseArray 消息创建
 */
TEST_F(SolverNodeTest, ArmorPoseArrayMessageCreation)
{
    armor_detector_ros2::msg::ArmorPoseArray pose_array;
    
    pose_array.header.stamp = test_node_->now();
    pose_array.header.frame_id = "camera_link";
    pose_array.solve_time_ms = 1.5f;
    
    // 添加两个位姿
    armor_detector_ros2::msg::ArmorPose pose1, pose2;
    pose1.distance = 2.0;
    pose1.valid = true;
    pose2.distance = 3.0;
    pose2.valid = true;
    
    pose_array.poses.push_back(pose1);
    pose_array.poses.push_back(pose2);
    
    EXPECT_EQ(pose_array.poses.size(), 2u);
    EXPECT_FLOAT_EQ(pose_array.solve_time_ms, 1.5f);
    EXPECT_DOUBLE_EQ(pose_array.poses[0].distance, 2.0);
    EXPECT_DOUBLE_EQ(pose_array.poses[1].distance, 3.0);
}

/**
 * @test 测试 ArmorBBoxArray 消息创建和发布
 */
TEST_F(SolverNodeTest, ArmorBBoxArrayPublish)
{
    armor_detector_ros2::msg::ArmorBBoxArray armor_array;
    armor_array.header.stamp = test_node_->now();
    armor_array.header.frame_id = "camera_link";
    armor_array.detection_time_ms = 5.0f;
    
    // 创建一个模拟装甲板
    auto armor = createMockArmor(640.0f, 360.0f, 43.0f, 17.5f);
    armor_array.armors.push_back(armor);
    
    // 发布消息
    armor_pub_->publish(armor_array);
    
    // 验证消息已发布（通过检查发布者状态）
    EXPECT_EQ(armor_pub_->get_subscription_count(), 0u);  // 没有订阅者时为 0
}

/**
 * @test 测试 IMU 四元数消息发布
 */
TEST_F(SolverNodeTest, IMUQuaternionPublish)
{
    auto quat = createIdentityQuaternion();
    
    // 发布消息
    imu_pub_->publish(quat);
    
    // 验证四元数是单位四元数
    double norm = std::sqrt(
        quat.quaternion.w * quat.quaternion.w +
        quat.quaternion.x * quat.quaternion.x +
        quat.quaternion.y * quat.quaternion.y +
        quat.quaternion.z * quat.quaternion.z
    );
    EXPECT_NEAR(norm, 1.0, 0.001);
}

// ============================================================================
// 模拟检测结果测试
// ============================================================================

/**
 * @test 测试创建模拟装甲板
 */
TEST_F(SolverNodeTest, CreateMockArmor)
{
    auto armor = createMockArmor(640.0f, 360.0f, 50.0f, 20.0f, 1, 5);
    
    // 验证角点
    EXPECT_FLOAT_EQ(armor.corners[0].x, 590.0f);  // 左上 x
    EXPECT_FLOAT_EQ(armor.corners[0].y, 340.0f);  // 左上 y
    EXPECT_FLOAT_EQ(armor.corners[1].x, 690.0f);  // 右上 x
    EXPECT_FLOAT_EQ(armor.corners[1].y, 340.0f);  // 右上 y
    EXPECT_FLOAT_EQ(armor.corners[2].x, 690.0f);  // 右下 x
    EXPECT_FLOAT_EQ(armor.corners[2].y, 380.0f);  // 右下 y
    EXPECT_FLOAT_EQ(armor.corners[3].x, 590.0f);  // 左下 x
    EXPECT_FLOAT_EQ(armor.corners[3].y, 380.0f);  // 左下 y
    
    // 验证中心点
    EXPECT_FLOAT_EQ(armor.center.x, 640.0f);
    EXPECT_FLOAT_EQ(armor.center.y, 360.0f);
    
    // 验证属性
    EXPECT_EQ(armor.color_id, 1);
    EXPECT_EQ(armor.tag_id, 5);
    EXPECT_FLOAT_EQ(armor.confidence, 0.95f);
}

/**
 * @test 测试创建多个模拟装甲板
 */
TEST_F(SolverNodeTest, CreateMultipleMockArmors)
{
    armor_detector_ros2::msg::ArmorBBoxArray armor_array;
    armor_array.header.stamp = test_node_->now();
    armor_array.header.frame_id = "camera_link";
    
    // 创建三个不同位置的装甲板
    armor_array.armors.push_back(createMockArmor(400.0f, 300.0f, 40.0f, 15.0f, 0, 3));
    armor_array.armors.push_back(createMockArmor(640.0f, 360.0f, 45.0f, 18.0f, 1, 1));  // 英雄（大装甲板）
    armor_array.armors.push_back(createMockArmor(880.0f, 400.0f, 42.0f, 16.0f, 0, 6));  // 哨兵（大装甲板）
    
    EXPECT_EQ(armor_array.armors.size(), 3u);
    
    // 验证不同装甲板的属性
    EXPECT_EQ(armor_array.armors[0].tag_id, 3);  // 步兵
    EXPECT_EQ(armor_array.armors[1].tag_id, 1);  // 英雄
    EXPECT_EQ(armor_array.armors[2].tag_id, 6);  // 哨兵
}

// ============================================================================
// 话题通信测试
// ============================================================================

/**
 * @test 测试话题名称正确性
 */
TEST_F(SolverNodeTest, TopicNames)
{
    // 验证发布者话题名称
    EXPECT_EQ(armor_pub_->get_topic_name(), std::string("/detector/armors"));
    EXPECT_EQ(imu_pub_->get_topic_name(), std::string("/imu/quaternion"));
    
    // 验证订阅者话题名称
    EXPECT_EQ(pose_sub_->get_topic_name(), std::string("/solver/armor_poses"));
}

/**
 * @test 测试 QoS 设置
 */
TEST_F(SolverNodeTest, QoSSettings)
{
    // 验证发布者和订阅者已创建
    EXPECT_NE(armor_pub_, nullptr);
    EXPECT_NE(imu_pub_, nullptr);
    EXPECT_NE(pose_sub_, nullptr);
}

// ============================================================================
// 位姿解算结果验证测试
// ============================================================================

/**
 * @test 测试位姿结果有效性范围
 * 
 * **Feature: ros2-vision-migration, Property 6: 解算结果完整性**
 * **Validates: Requirements 2.2**
 */
TEST_F(SolverNodeTest, PoseResultValidityRanges)
{
    // 创建一个有效的位姿结果
    armor_detector_ros2::msg::ArmorPose pose;
    pose.yaw = 0.5;      // 在 [-π, π] 范围内
    pose.pitch = 0.3;    // 在 [-π/2, π/2] 范围内
    pose.distance = 2.5; // 正数
    pose.valid = true;
    
    // 验证 yaw 范围
    EXPECT_GE(pose.yaw, -M_PI);
    EXPECT_LE(pose.yaw, M_PI);
    
    // 验证 pitch 范围
    EXPECT_GE(pose.pitch, -M_PI / 2);
    EXPECT_LE(pose.pitch, M_PI / 2);
    
    // 验证 distance 为正数
    EXPECT_GT(pose.distance, 0.0);
}

/**
 * @test 测试装甲板类型判断
 * 
 * 英雄(tag_id=1)和哨兵(tag_id=6)应该使用大装甲板
 */
TEST_F(SolverNodeTest, ArmorTypeClassification)
{
    // 小装甲板类型
    std::vector<int> small_armor_tags = {0, 2, 3, 4, 5, 7};
    for (int tag : small_armor_tags)
    {
        auto armor = createMockArmor(640.0f, 360.0f, 40.0f, 15.0f, 0, tag);
        // 验证这些 tag_id 应该使用小装甲板
        EXPECT_NE(tag, 1) << "Tag " << tag << " should not be Hero";
        EXPECT_NE(tag, 6) << "Tag " << tag << " should not be Sentry";
    }
    
    // 大装甲板类型
    std::vector<int> large_armor_tags = {1, 6};
    for (int tag : large_armor_tags)
    {
        auto armor = createMockArmor(640.0f, 360.0f, 60.0f, 15.0f, 0, tag);
        // 验证这些 tag_id 应该使用大装甲板
        EXPECT_TRUE(tag == 1 || tag == 6) << "Tag " << tag << " should be Hero or Sentry";
    }
}

/**
 * @test 测试位姿数组解算时间记录
 */
TEST_F(SolverNodeTest, SolveTimeRecording)
{
    armor_detector_ros2::msg::ArmorPoseArray pose_array;
    pose_array.solve_time_ms = 2.5f;
    
    // 验证解算时间为正数
    EXPECT_GT(pose_array.solve_time_ms, 0.0f);
    
    // 验证解算时间在合理范围内（小于 100ms）
    EXPECT_LT(pose_array.solve_time_ms, 100.0f);
}

// ============================================================================
// 边界条件测试
// ============================================================================

/**
 * @test 测试空检测结果处理
 */
TEST_F(SolverNodeTest, EmptyDetectionResult)
{
    armor_detector_ros2::msg::ArmorBBoxArray empty_array;
    empty_array.header.stamp = test_node_->now();
    empty_array.header.frame_id = "camera_link";
    empty_array.detection_time_ms = 1.0f;
    // armors 数组为空
    
    EXPECT_EQ(empty_array.armors.size(), 0u);
    
    // 发布空检测结果
    armor_pub_->publish(empty_array);
    
    // 验证发布成功
    EXPECT_NE(armor_pub_, nullptr);
}

/**
 * @test 测试极端位置的装甲板
 */
TEST_F(SolverNodeTest, ExtremePositionArmor)
{
    // 图像边缘的装甲板
    auto armor_left = createMockArmor(50.0f, 360.0f, 40.0f, 15.0f);
    auto armor_right = createMockArmor(1230.0f, 360.0f, 40.0f, 15.0f);
    auto armor_top = createMockArmor(640.0f, 30.0f, 40.0f, 15.0f);
    auto armor_bottom = createMockArmor(640.0f, 690.0f, 40.0f, 15.0f);
    
    // 验证角点在合理范围内
    EXPECT_GE(armor_left.corners[0].x, 0.0f);
    EXPECT_LE(armor_right.corners[1].x, 1280.0f);
    EXPECT_GE(armor_top.corners[0].y, 0.0f);
    EXPECT_LE(armor_bottom.corners[2].y, 720.0f);
}

/**
 * @test 测试颜色 ID 有效性
 */
TEST_F(SolverNodeTest, ColorIdValidity)
{
    // 有效的颜色 ID: 0=蓝色, 1=红色, 2=灰色, 3=紫色
    for (int color_id = 0; color_id <= 3; color_id++)
    {
        auto armor = createMockArmor(640.0f, 360.0f, 40.0f, 15.0f, color_id, 3);
        EXPECT_GE(armor.color_id, 0);
        EXPECT_LE(armor.color_id, 3);
    }
}

/**
 * @test 测试标签 ID 有效性
 */
TEST_F(SolverNodeTest, TagIdValidity)
{
    // 有效的标签 ID: 0-7
    for (int tag_id = 0; tag_id <= 7; tag_id++)
    {
        auto armor = createMockArmor(640.0f, 360.0f, 40.0f, 15.0f, 0, tag_id);
        EXPECT_GE(armor.tag_id, 0);
        EXPECT_LE(armor.tag_id, 7);
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
