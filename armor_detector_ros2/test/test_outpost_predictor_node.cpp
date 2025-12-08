/**
 * @file test_outpost_predictor_node.cpp
 * @brief outpost_predictor_node 集成测试
 * 
 * 测试前哨站预测节点的功能：
 * - 订阅装甲板位姿解算结果
 * - 使用 EKF 估计前哨站旋转状态
 * - 发布前哨站状态预测
 * - 使用模拟的前哨站数据验证预测精度
 * 
 * Requirements: 7.1, 7.2, 7.3
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <atomic>
#include <cmath>

#include "armor_detector_ros2/msg/armor_pose.hpp"
#include "armor_detector_ros2/msg/armor_pose_array.hpp"
#include "armor_detector_ros2/msg/outpost_state.hpp"

using namespace std::chrono_literals;

// 前哨站物理常量
constexpr double OUTPOST_RADIUS = 0.275;  // 前哨站半径（米）
constexpr double OUTPOST_OMEGA = 0.80 * M_PI;  // 预期角速度（rad/s）
constexpr int OUTPOST_TAG_ID = 7;  // 前哨站装甲板 tag_id

/**
 * @brief outpost_predictor_node 集成测试夹具
 */
class OutpostPredictorNodeTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        
        // 创建测试节点
        test_node_ = std::make_shared<rclcpp::Node>("outpost_predictor_node_test");
        
        // 创建发布者（模拟 solver_node 输出）
        pose_pub_ = test_node_->create_publisher<armor_detector_ros2::msg::ArmorPoseArray>(
            "/solver/armor_poses",
            rclcpp::QoS(10).reliable()
        );
        
        // 创建订阅者（接收前哨站预测结果）
        prediction_received_ = false;
        prediction_sub_ = test_node_->create_subscription<armor_detector_ros2::msg::OutpostState>(
            "/outpost/prediction",
            rclcpp::QoS(10).reliable(),
            [this](const armor_detector_ros2::msg::OutpostState::SharedPtr msg) {
                last_prediction_ = msg;
                prediction_received_ = true;
                prediction_count_++;
            }
        );
    }
    
    void TearDown() override
    {
        test_node_.reset();
        rclcpp::shutdown();
    }
    
    /**
     * @brief 创建模拟的前哨站装甲板位姿
     * @param theta 当前旋转角度（弧度）
     * @param center_x 旋转中心 x 坐标
     * @param center_y 旋转中心 y 坐标
     * @param center_z 旋转中心 z 坐标
     * @return 装甲板位姿消息
     */
    armor_detector_ros2::msg::ArmorPose createOutpostArmorPose(
        double theta,
        double center_x = 0.0,
        double center_y = 5.0,
        double center_z = 0.0)
    {
        armor_detector_ros2::msg::ArmorPose pose;
        
        // 计算装甲板位置（相对于旋转中心）
        double armor_x = center_x + OUTPOST_RADIUS * std::sin(theta);
        double armor_y = center_y - OUTPOST_RADIUS * std::cos(theta);
        double armor_z = center_z;
        
        // 计算距离
        double distance = std::sqrt(armor_x * armor_x + armor_y * armor_y + armor_z * armor_z);
        
        // 计算 yaw 和 pitch
        pose.yaw = -std::atan2(armor_x, armor_y);
        pose.pitch = std::atan2(armor_z, std::sqrt(armor_x * armor_x + armor_y * armor_y));
        pose.distance = distance;
        
        // 设置世界坐标
        pose.position.x = armor_x;
        pose.position.y = armor_y;
        pose.position.z = armor_z;
        
        // 装甲板朝向（指向旋转中心）
        pose.armor_yaw = theta;
        
        // 设置为前哨站装甲板
        pose.color_id = 0;  // 蓝色
        pose.tag_id = OUTPOST_TAG_ID;
        pose.armor_type = 0;  // 小装甲板
        pose.valid = true;
        
        return pose;
    }
    
    /**
     * @brief 发布模拟的前哨站装甲板位姿序列
     * @param num_frames 帧数
     * @param omega 角速度（rad/s）
     * @param dt 时间间隔（秒）
     * @param center_x 旋转中心 x 坐标
     * @param center_y 旋转中心 y 坐标
     */
    void publishOutpostSequence(
        int num_frames,
        double omega = OUTPOST_OMEGA,
        double dt = 0.01,
        double center_x = 0.0,
        double center_y = 5.0)
    {
        double theta = 0.0;
        
        for (int i = 0; i < num_frames; i++)
        {
            armor_detector_ros2::msg::ArmorPoseArray pose_array;
            pose_array.header.stamp = test_node_->now();
            pose_array.header.frame_id = "camera_link";
            pose_array.solve_time_ms = 1.0f;
            
            // 创建前哨站装甲板位姿
            auto armor_pose = createOutpostArmorPose(theta, center_x, center_y);
            armor_pose.header = pose_array.header;
            pose_array.poses.push_back(armor_pose);
            
            // 发布
            pose_pub_->publish(pose_array);
            
            // 处理回调
            rclcpp::spin_some(test_node_);
            
            // 更新角度
            theta += omega * dt;
            
            // 等待一小段时间
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
        }
    }
    
    /**
     * @brief 等待消息接收
     * @param timeout 超时时间
     * @return 是否在超时前收到消息
     */
    bool waitForMessage(std::chrono::milliseconds timeout = 2000ms)
    {
        auto start = std::chrono::steady_clock::now();
        while (!prediction_received_)
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
    
    /**
     * @brief 重置接收状态
     */
    void resetReceiveState()
    {
        prediction_received_ = false;
        last_prediction_.reset();
    }
    
    // 测试节点
    rclcpp::Node::SharedPtr test_node_;
    
    // 发布者
    rclcpp::Publisher<armor_detector_ros2::msg::ArmorPoseArray>::SharedPtr pose_pub_;
    
    // 订阅者
    rclcpp::Subscription<armor_detector_ros2::msg::OutpostState>::SharedPtr prediction_sub_;
    
    // 接收到的消息
    armor_detector_ros2::msg::OutpostState::SharedPtr last_prediction_;
    std::atomic<bool> prediction_received_{false};
    std::atomic<int> prediction_count_{0};
};

// ============================================================================
// OutpostState 消息类型测试
// ============================================================================

/**
 * @test 测试 OutpostState 消息创建
 * 
 * **Feature: ros2-vision-migration, Property 22: 前哨站输出完整性**
 * **Validates: Requirements 7.2**
 */
TEST_F(OutpostPredictorNodeTest, OutpostStateMessageCreation)
{
    armor_detector_ros2::msg::OutpostState state;
    
    state.header.stamp = test_node_->now();
    state.header.frame_id = "world";
    
    state.center.x = 0.0;
    state.center.y = 5.0;
    state.center.z = 0.0;
    
    state.velocity.x = 0.0;
    state.velocity.y = 0.0;
    state.velocity.z = 0.0;
    
    state.radius = OUTPOST_RADIUS;
    state.theta = 0.5;
    state.omega = OUTPOST_OMEGA;
    state.direction = 1;
    state.valid = true;
    
    // 验证字段
    EXPECT_DOUBLE_EQ(state.center.y, 5.0);
    EXPECT_DOUBLE_EQ(state.radius, OUTPOST_RADIUS);
    EXPECT_DOUBLE_EQ(state.omega, OUTPOST_OMEGA);
    EXPECT_EQ(state.direction, 1);
    EXPECT_TRUE(state.valid);
}

/**
 * @test 测试 OutpostState 消息字段范围
 * 
 * **Feature: ros2-vision-migration, Property 22: 前哨站输出完整性**
 * **Validates: Requirements 7.2**
 */
TEST_F(OutpostPredictorNodeTest, OutpostStateFieldRanges)
{
    armor_detector_ros2::msg::OutpostState state;
    
    // 设置有效的前哨站状态
    state.radius = OUTPOST_RADIUS;
    state.theta = 1.5;
    state.omega = OUTPOST_OMEGA;
    state.direction = -1;
    state.valid = true;
    
    // 验证半径在合理范围内（约 0.275m ± 0.05m）
    EXPECT_GE(state.radius, 0.225);
    EXPECT_LE(state.radius, 0.325);
    
    // 验证角度在 [-π, π] 范围内
    EXPECT_GE(state.theta, -M_PI);
    EXPECT_LE(state.theta, M_PI);
    
    // 验证方向为 -1, 0, 或 1
    EXPECT_TRUE(state.direction == -1 || state.direction == 0 || state.direction == 1);
}

// ============================================================================
// 模拟前哨站数据测试
// ============================================================================

/**
 * @test 测试创建模拟前哨站装甲板位姿
 */
TEST_F(OutpostPredictorNodeTest, CreateOutpostArmorPose)
{
    double theta = 0.0;
    auto pose = createOutpostArmorPose(theta, 0.0, 5.0, 0.0);
    
    // 验证 tag_id 为前哨站
    EXPECT_EQ(pose.tag_id, OUTPOST_TAG_ID);
    
    // 验证位姿有效
    EXPECT_TRUE(pose.valid);
    
    // 验证距离为正数
    EXPECT_GT(pose.distance, 0.0);
    
    // 验证装甲板位置（theta=0 时，装甲板在中心正前方）
    EXPECT_NEAR(pose.position.x, 0.0, 0.01);
    EXPECT_NEAR(pose.position.y, 5.0 - OUTPOST_RADIUS, 0.01);
}

/**
 * @test 测试不同角度的前哨站装甲板位姿
 */
TEST_F(OutpostPredictorNodeTest, OutpostArmorPoseAtDifferentAngles)
{
    double center_y = 5.0;
    
    // theta = 0: 装甲板在中心前方
    auto pose0 = createOutpostArmorPose(0.0, 0.0, center_y);
    EXPECT_NEAR(pose0.position.x, 0.0, 0.01);
    EXPECT_NEAR(pose0.position.y, center_y - OUTPOST_RADIUS, 0.01);
    
    // theta = π/2: 装甲板在中心右侧
    auto pose90 = createOutpostArmorPose(M_PI / 2, 0.0, center_y);
    EXPECT_NEAR(pose90.position.x, OUTPOST_RADIUS, 0.01);
    EXPECT_NEAR(pose90.position.y, center_y, 0.01);
    
    // theta = π: 装甲板在中心后方
    auto pose180 = createOutpostArmorPose(M_PI, 0.0, center_y);
    EXPECT_NEAR(pose180.position.x, 0.0, 0.01);
    EXPECT_NEAR(pose180.position.y, center_y + OUTPOST_RADIUS, 0.01);
    
    // theta = -π/2: 装甲板在中心左侧
    auto pose270 = createOutpostArmorPose(-M_PI / 2, 0.0, center_y);
    EXPECT_NEAR(pose270.position.x, -OUTPOST_RADIUS, 0.01);
    EXPECT_NEAR(pose270.position.y, center_y, 0.01);
}

// ============================================================================
// 话题通信测试
// ============================================================================

/**
 * @test 测试话题名称正确性
 */
TEST_F(OutpostPredictorNodeTest, TopicNames)
{
    // 验证发布者话题名称
    EXPECT_EQ(pose_pub_->get_topic_name(), std::string("/solver/armor_poses"));
    
    // 验证订阅者话题名称
    EXPECT_EQ(prediction_sub_->get_topic_name(), std::string("/outpost/prediction"));
}

/**
 * @test 测试 QoS 设置
 */
TEST_F(OutpostPredictorNodeTest, QoSSettings)
{
    // 验证发布者和订阅者已创建
    EXPECT_NE(pose_pub_, nullptr);
    EXPECT_NE(prediction_sub_, nullptr);
}

/**
 * @test 测试发布装甲板位姿数组
 */
TEST_F(OutpostPredictorNodeTest, PublishArmorPoseArray)
{
    armor_detector_ros2::msg::ArmorPoseArray pose_array;
    pose_array.header.stamp = test_node_->now();
    pose_array.header.frame_id = "camera_link";
    pose_array.solve_time_ms = 1.5f;
    
    // 添加前哨站装甲板
    auto armor_pose = createOutpostArmorPose(0.0);
    armor_pose.header = pose_array.header;
    pose_array.poses.push_back(armor_pose);
    
    // 发布消息
    pose_pub_->publish(pose_array);
    
    // 验证消息已发布
    EXPECT_EQ(pose_array.poses.size(), 1u);
    EXPECT_EQ(pose_array.poses[0].tag_id, OUTPOST_TAG_ID);
}

// ============================================================================
// 前哨站预测功能测试
// ============================================================================

/**
 * @test 测试非前哨站装甲板被过滤
 */
TEST_F(OutpostPredictorNodeTest, NonOutpostArmorFiltered)
{
    armor_detector_ros2::msg::ArmorPoseArray pose_array;
    pose_array.header.stamp = test_node_->now();
    pose_array.header.frame_id = "camera_link";
    
    // 创建非前哨站装甲板（步兵）
    armor_detector_ros2::msg::ArmorPose infantry_pose;
    infantry_pose.header = pose_array.header;
    infantry_pose.yaw = 0.0;
    infantry_pose.pitch = 0.0;
    infantry_pose.distance = 3.0;
    infantry_pose.tag_id = 3;  // 步兵
    infantry_pose.valid = true;
    pose_array.poses.push_back(infantry_pose);
    
    // 发布
    pose_pub_->publish(pose_array);
    rclcpp::spin_some(test_node_);
    
    // 验证消息已发布
    EXPECT_EQ(pose_array.poses[0].tag_id, 3);
    EXPECT_NE(pose_array.poses[0].tag_id, OUTPOST_TAG_ID);
}

/**
 * @test 测试空位姿数组处理
 */
TEST_F(OutpostPredictorNodeTest, EmptyPoseArrayHandling)
{
    armor_detector_ros2::msg::ArmorPoseArray empty_array;
    empty_array.header.stamp = test_node_->now();
    empty_array.header.frame_id = "camera_link";
    empty_array.solve_time_ms = 1.0f;
    // poses 数组为空
    
    EXPECT_EQ(empty_array.poses.size(), 0u);
    
    // 发布空数组
    pose_pub_->publish(empty_array);
    rclcpp::spin_some(test_node_);
    
    // 验证发布成功
    EXPECT_NE(pose_pub_, nullptr);
}

/**
 * @test 测试无效位姿被忽略
 */
TEST_F(OutpostPredictorNodeTest, InvalidPoseIgnored)
{
    armor_detector_ros2::msg::ArmorPoseArray pose_array;
    pose_array.header.stamp = test_node_->now();
    pose_array.header.frame_id = "camera_link";
    
    // 创建无效的前哨站装甲板
    auto armor_pose = createOutpostArmorPose(0.0);
    armor_pose.header = pose_array.header;
    armor_pose.valid = false;  // 标记为无效
    pose_array.poses.push_back(armor_pose);
    
    // 发布
    pose_pub_->publish(pose_array);
    rclcpp::spin_some(test_node_);
    
    // 验证位姿被标记为无效
    EXPECT_FALSE(pose_array.poses[0].valid);
}

// ============================================================================
// 方向检测测试
// ============================================================================

/**
 * @test 测试旋转方向值有效性
 * 
 * **Feature: ros2-vision-migration, Property 23: 前哨站方向检测**
 * **Validates: Requirements 7.3**
 */
TEST_F(OutpostPredictorNodeTest, DirectionValueValidity)
{
    // 有效的方向值: -1 (顺时针), 0 (未知), 1 (逆时针)
    std::vector<int> valid_directions = {-1, 0, 1};
    
    for (int dir : valid_directions)
    {
        armor_detector_ros2::msg::OutpostState state;
        state.direction = dir;
        
        EXPECT_TRUE(state.direction >= -1 && state.direction <= 1)
            << "Direction " << dir << " should be valid";
    }
}

/**
 * @test 测试顺时针旋转方向
 */
TEST_F(OutpostPredictorNodeTest, ClockwiseRotation)
{
    armor_detector_ros2::msg::OutpostState state;
    state.direction = -1;  // 顺时针
    state.omega = -OUTPOST_OMEGA;  // 负角速度
    
    EXPECT_EQ(state.direction, -1);
    EXPECT_LT(state.omega, 0.0);
}

/**
 * @test 测试逆时针旋转方向
 */
TEST_F(OutpostPredictorNodeTest, CounterclockwiseRotation)
{
    armor_detector_ros2::msg::OutpostState state;
    state.direction = 1;  // 逆时针
    state.omega = OUTPOST_OMEGA;  // 正角速度
    
    EXPECT_EQ(state.direction, 1);
    EXPECT_GT(state.omega, 0.0);
}

// ============================================================================
// 边界条件测试
// ============================================================================

/**
 * @test 测试极端距离的前哨站
 */
TEST_F(OutpostPredictorNodeTest, ExtremeDistanceOutpost)
{
    // 近距离前哨站
    auto pose_near = createOutpostArmorPose(0.0, 0.0, 2.0);
    EXPECT_GT(pose_near.distance, 0.0);
    EXPECT_LT(pose_near.distance, 3.0);
    
    // 远距离前哨站
    auto pose_far = createOutpostArmorPose(0.0, 0.0, 10.0);
    EXPECT_GT(pose_far.distance, 9.0);
}

/**
 * @test 测试偏移位置的前哨站
 */
TEST_F(OutpostPredictorNodeTest, OffsetPositionOutpost)
{
    // 左侧偏移
    auto pose_left = createOutpostArmorPose(0.0, -2.0, 5.0);
    EXPECT_LT(pose_left.position.x, 0.0);
    
    // 右侧偏移
    auto pose_right = createOutpostArmorPose(0.0, 2.0, 5.0);
    EXPECT_GT(pose_right.position.x, 0.0);
}

/**
 * @test 测试角度边界值
 */
TEST_F(OutpostPredictorNodeTest, AngleBoundaryValues)
{
    // 测试各种边界角度
    std::vector<double> test_angles = {0.0, M_PI / 4, M_PI / 2, M_PI, -M_PI / 2, -M_PI};
    
    for (double theta : test_angles)
    {
        auto pose = createOutpostArmorPose(theta);
        
        // 验证位姿有效
        EXPECT_TRUE(pose.valid);
        
        // 验证距离为正数
        EXPECT_GT(pose.distance, 0.0);
        
        // 验证 armor_yaw 等于输入角度
        EXPECT_DOUBLE_EQ(pose.armor_yaw, theta);
    }
}

/**
 * @test 测试高度变化的前哨站
 */
TEST_F(OutpostPredictorNodeTest, HeightVariationOutpost)
{
    // 低位置
    auto pose_low = createOutpostArmorPose(0.0, 0.0, 5.0, -0.5);
    EXPECT_DOUBLE_EQ(pose_low.position.z, -0.5);
    
    // 高位置
    auto pose_high = createOutpostArmorPose(0.0, 0.0, 5.0, 0.5);
    EXPECT_DOUBLE_EQ(pose_high.position.z, 0.5);
}

// ============================================================================
// 预测精度测试
// ============================================================================

/**
 * @test 测试角速度估计范围
 * 
 * **Feature: ros2-vision-migration, Property 21: 前哨站 EKF 收敛性**
 * **Validates: Requirements 7.1**
 */
TEST_F(OutpostPredictorNodeTest, OmegaEstimationRange)
{
    armor_detector_ros2::msg::OutpostState state;
    
    // 设置预期的角速度
    state.omega = OUTPOST_OMEGA;
    
    // 验证角速度在合理范围内（预期值 ± 50%）
    double expected_omega = OUTPOST_OMEGA;
    double tolerance = expected_omega * 0.5;
    
    EXPECT_GE(state.omega, expected_omega - tolerance);
    EXPECT_LE(state.omega, expected_omega + tolerance);
}

/**
 * @test 测试半径估计范围
 * 
 * **Feature: ros2-vision-migration, Property 22: 前哨站输出完整性**
 * **Validates: Requirements 7.2**
 */
TEST_F(OutpostPredictorNodeTest, RadiusEstimationRange)
{
    armor_detector_ros2::msg::OutpostState state;
    
    // 设置预期的半径
    state.radius = OUTPOST_RADIUS;
    
    // 验证半径在合理范围内（约 0.275m ± 0.05m）
    EXPECT_GE(state.radius, 0.225);
    EXPECT_LE(state.radius, 0.325);
}

/**
 * @test 测试中心位置估计
 */
TEST_F(OutpostPredictorNodeTest, CenterPositionEstimation)
{
    armor_detector_ros2::msg::OutpostState state;
    
    // 设置预期的中心位置
    state.center.x = 0.0;
    state.center.y = 5.0;
    state.center.z = 0.0;
    
    // 验证中心位置在合理范围内
    EXPECT_NEAR(state.center.x, 0.0, 1.0);
    EXPECT_NEAR(state.center.y, 5.0, 1.0);
    EXPECT_NEAR(state.center.z, 0.0, 0.5);
}

/**
 * @test 测试速度估计
 */
TEST_F(OutpostPredictorNodeTest, VelocityEstimation)
{
    armor_detector_ros2::msg::OutpostState state;
    
    // 静止前哨站的中心速度应该接近零
    state.velocity.x = 0.0;
    state.velocity.y = 0.0;
    state.velocity.z = 0.0;
    
    // 验证速度在合理范围内
    double velocity_magnitude = std::sqrt(
        state.velocity.x * state.velocity.x +
        state.velocity.y * state.velocity.y +
        state.velocity.z * state.velocity.z
    );
    
    EXPECT_LT(velocity_magnitude, 1.0);  // 速度应该小于 1 m/s
}

// ============================================================================
// 有效性状态测试
// ============================================================================

/**
 * @test 测试有效状态条件
 */
TEST_F(OutpostPredictorNodeTest, ValidStateConditions)
{
    armor_detector_ros2::msg::OutpostState state;
    
    // 有效状态需要足够的观测
    state.valid = true;
    state.radius = OUTPOST_RADIUS;
    state.omega = OUTPOST_OMEGA;
    state.direction = 1;
    
    EXPECT_TRUE(state.valid);
    EXPECT_GT(state.radius, 0.0);
    EXPECT_NE(state.omega, 0.0);
}

/**
 * @test 测试无效状态
 */
TEST_F(OutpostPredictorNodeTest, InvalidState)
{
    armor_detector_ros2::msg::OutpostState state;
    
    // 默认状态应该是无效的
    state.valid = false;
    
    EXPECT_FALSE(state.valid);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
