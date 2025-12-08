/**
 * @file test_pose_solver.cpp
 * @brief PoseSolver 单元测试
 * 
 * 测试 PnP 位姿解算器的功能：
 * - 相机内参设置
 * - YAML 配置文件加载
 * - 已知位姿的装甲板解算
 * 
 * Requirements: 2.1, 2.3
 */

#include <gtest/gtest.h>
#include "core/pose_solver.hpp"
#include <opencv2/calib3d.hpp>
#include <cmath>

using namespace SOLVER;

/**
 * @brief PoseSolver 测试夹具
 */
class PoseSolverTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // 设置典型的相机内参 (1280x720 分辨率)
        test_intrinsics_.fx = 1280.0;
        test_intrinsics_.fy = 1280.0;
        test_intrinsics_.cx = 640.0;
        test_intrinsics_.cy = 360.0;
        test_intrinsics_.k1 = 0.0;
        test_intrinsics_.k2 = 0.0;
        test_intrinsics_.p1 = 0.0;
        test_intrinsics_.p2 = 0.0;
        test_intrinsics_.k3 = 0.0;
    }
    
    CameraIntrinsics test_intrinsics_;
    PoseSolver solver_;
};

// ============================================================================
// 相机内参设置测试
// ============================================================================

/**
 * @test 测试默认构造函数
 */
TEST_F(PoseSolverTest, DefaultConstructor)
{
    PoseSolver solver;
    EXPECT_FALSE(solver.isCameraIntrinsicsSet());
}

/**
 * @test 测试通过结构体设置相机内参
 */
TEST_F(PoseSolverTest, SetCameraIntrinsicsStruct)
{
    solver_.setCameraIntrinsics(test_intrinsics_);
    
    EXPECT_TRUE(solver_.isCameraIntrinsicsSet());
    
    const auto& intrinsics = solver_.getCameraIntrinsics();
    EXPECT_DOUBLE_EQ(intrinsics.fx, 1280.0);
    EXPECT_DOUBLE_EQ(intrinsics.fy, 1280.0);
    EXPECT_DOUBLE_EQ(intrinsics.cx, 640.0);
    EXPECT_DOUBLE_EQ(intrinsics.cy, 360.0);
}

/**
 * @test 测试通过参数设置相机内参
 */
TEST_F(PoseSolverTest, SetCameraMatrix)
{
    solver_.setCameraMatrix(1000.0, 1000.0, 320.0, 240.0, 0.1, 0.01, 0.001, 0.001, 0.0001);
    
    EXPECT_TRUE(solver_.isCameraIntrinsicsSet());
    
    const auto& intrinsics = solver_.getCameraIntrinsics();
    EXPECT_DOUBLE_EQ(intrinsics.fx, 1000.0);
    EXPECT_DOUBLE_EQ(intrinsics.fy, 1000.0);
    EXPECT_DOUBLE_EQ(intrinsics.cx, 320.0);
    EXPECT_DOUBLE_EQ(intrinsics.cy, 240.0);
    EXPECT_DOUBLE_EQ(intrinsics.k1, 0.1);
    EXPECT_DOUBLE_EQ(intrinsics.k2, 0.01);
    EXPECT_DOUBLE_EQ(intrinsics.p1, 0.001);
    EXPECT_DOUBLE_EQ(intrinsics.p2, 0.001);
    EXPECT_DOUBLE_EQ(intrinsics.k3, 0.0001);
}

/**
 * @test 测试内参有效性检查
 */
TEST_F(PoseSolverTest, IntrinsicsValidityCheck)
{
    CameraIntrinsics invalid_intrinsics;
    invalid_intrinsics.fx = 0.0;  // 无效的焦距
    invalid_intrinsics.fy = 1000.0;
    invalid_intrinsics.cx = 320.0;
    invalid_intrinsics.cy = 240.0;
    
    EXPECT_FALSE(invalid_intrinsics.isValid());
    
    CameraIntrinsics valid_intrinsics;
    valid_intrinsics.fx = 1000.0;
    valid_intrinsics.fy = 1000.0;
    valid_intrinsics.cx = 320.0;
    valid_intrinsics.cy = 240.0;
    
    EXPECT_TRUE(valid_intrinsics.isValid());
}

// ============================================================================
// 3D点初始化测试
// ============================================================================

/**
 * @test 测试小装甲板3D点初始化
 */
TEST_F(PoseSolverTest, SmallArmorPointsInitialization)
{
    const auto& points = solver_.getSmallArmorPoints();
    
    ASSERT_EQ(points.size(), 4u);
    
    // 验证点的顺序和尺寸
    // 左上
    EXPECT_FLOAT_EQ(points[0].x, -0.0675f);
    EXPECT_FLOAT_EQ(points[0].y, -0.0275f);
    EXPECT_FLOAT_EQ(points[0].z, 0.0f);
    
    // 右上
    EXPECT_FLOAT_EQ(points[1].x, 0.0675f);
    EXPECT_FLOAT_EQ(points[1].y, -0.0275f);
    EXPECT_FLOAT_EQ(points[1].z, 0.0f);
    
    // 右下
    EXPECT_FLOAT_EQ(points[2].x, 0.0675f);
    EXPECT_FLOAT_EQ(points[2].y, 0.0275f);
    EXPECT_FLOAT_EQ(points[2].z, 0.0f);
    
    // 左下
    EXPECT_FLOAT_EQ(points[3].x, -0.0675f);
    EXPECT_FLOAT_EQ(points[3].y, 0.0275f);
    EXPECT_FLOAT_EQ(points[3].z, 0.0f);
}

/**
 * @test 测试大装甲板3D点初始化
 */
TEST_F(PoseSolverTest, LargeArmorPointsInitialization)
{
    const auto& points = solver_.getLargeArmorPoints();
    
    ASSERT_EQ(points.size(), 4u);
    
    // 验证点的顺序和尺寸
    // 左上
    EXPECT_FLOAT_EQ(points[0].x, -0.1125f);
    EXPECT_FLOAT_EQ(points[0].y, -0.0275f);
    EXPECT_FLOAT_EQ(points[0].z, 0.0f);
    
    // 右上
    EXPECT_FLOAT_EQ(points[1].x, 0.1125f);
    EXPECT_FLOAT_EQ(points[1].y, -0.0275f);
    EXPECT_FLOAT_EQ(points[1].z, 0.0f);
}

// ============================================================================
// PnP解算测试
// ============================================================================

/**
 * @test 测试未设置内参时的解算
 */
TEST_F(PoseSolverTest, SolveWithoutIntrinsics)
{
    std::vector<cv::Point2f> corners = {
        cv::Point2f(600, 340),
        cv::Point2f(680, 340),
        cv::Point2f(680, 380),
        cv::Point2f(600, 380)
    };
    
    ArmorPose result = solver_.solve(corners, ArmorType::SMALL);
    
    EXPECT_FALSE(result.valid);
}

/**
 * @test 测试无效角点数量
 */
TEST_F(PoseSolverTest, SolveWithInvalidCornerCount)
{
    solver_.setCameraIntrinsics(test_intrinsics_);
    
    std::vector<cv::Point2f> corners = {
        cv::Point2f(600, 340),
        cv::Point2f(680, 340),
        cv::Point2f(680, 380)
        // 缺少第4个角点
    };
    
    ArmorPose result = solver_.solve(corners, ArmorType::SMALL);
    
    EXPECT_FALSE(result.valid);
}

/**
 * @test 测试正面装甲板解算（装甲板在相机正前方）
 * 
 * 这个测试验证当装甲板位于相机正前方时，解算结果的合理性。
 */
TEST_F(PoseSolverTest, SolveFrontalArmor)
{
    solver_.setCameraIntrinsics(test_intrinsics_);
    
    // 模拟一个位于相机正前方约2米处的小装甲板
    // 装甲板尺寸: 0.135m x 0.055m (小装甲板)
    // 在图像中心附近，大约占据80x30像素
    double distance = 2.0;  // 2米
    double armor_width = 0.135;  // 小装甲板宽度
    double armor_height = 0.055; // 小装甲板高度
    
    // 计算投影尺寸
    double proj_half_width = (armor_width / 2.0) * test_intrinsics_.fx / distance;
    double proj_half_height = (armor_height / 2.0) * test_intrinsics_.fy / distance;
    
    std::vector<cv::Point2f> corners = {
        cv::Point2f(test_intrinsics_.cx - proj_half_width, test_intrinsics_.cy - proj_half_height),  // 左上
        cv::Point2f(test_intrinsics_.cx + proj_half_width, test_intrinsics_.cy - proj_half_height),  // 右上
        cv::Point2f(test_intrinsics_.cx + proj_half_width, test_intrinsics_.cy + proj_half_height),  // 右下
        cv::Point2f(test_intrinsics_.cx - proj_half_width, test_intrinsics_.cy + proj_half_height)   // 左下
    };
    
    ArmorPose result = solver_.solve(corners, ArmorType::SMALL);
    
    EXPECT_TRUE(result.valid);
    
    // 验证距离（允许10%误差）
    EXPECT_NEAR(result.distance, distance, distance * 0.1);
    
    // 验证yaw和pitch接近0（装甲板在正前方）
    EXPECT_NEAR(result.yaw, 0.0, 0.1);  // 允许0.1弧度误差
    EXPECT_NEAR(result.pitch, 0.0, 0.1);
    
    // 验证位置
    EXPECT_NEAR(result.position(0), 0.0, 0.1);  // x接近0
    EXPECT_NEAR(result.position(1), 0.0, 0.1);  // y接近0
    EXPECT_NEAR(result.position(2), distance, distance * 0.1);  // z接近distance
}

/**
 * @test 测试偏移装甲板解算（装甲板在相机右侧）
 */
TEST_F(PoseSolverTest, SolveOffsetArmor)
{
    solver_.setCameraIntrinsics(test_intrinsics_);
    
    // 模拟一个位于相机右侧的装甲板
    // 装甲板在图像右半部分
    double distance = 2.0;
    double armor_width = 0.135;
    double armor_height = 0.055;
    
    double proj_half_width = (armor_width / 2.0) * test_intrinsics_.fx / distance;
    double proj_half_height = (armor_height / 2.0) * test_intrinsics_.fy / distance;
    
    // 装甲板中心偏右200像素
    double offset_x = 200.0;
    
    std::vector<cv::Point2f> corners = {
        cv::Point2f(test_intrinsics_.cx + offset_x - proj_half_width, test_intrinsics_.cy - proj_half_height),
        cv::Point2f(test_intrinsics_.cx + offset_x + proj_half_width, test_intrinsics_.cy - proj_half_height),
        cv::Point2f(test_intrinsics_.cx + offset_x + proj_half_width, test_intrinsics_.cy + proj_half_height),
        cv::Point2f(test_intrinsics_.cx + offset_x - proj_half_width, test_intrinsics_.cy + proj_half_height)
    };
    
    ArmorPose result = solver_.solve(corners, ArmorType::SMALL);
    
    EXPECT_TRUE(result.valid);
    
    // 验证yaw为正值（装甲板在右侧，相机坐标系x轴向右）
    // 注意：yaw的计算方式是 imu_yaw - atan2(x, z)，所以当x>0时yaw为负
    // 但这里imu_yaw=0，所以yaw = -atan2(x, z)
    EXPECT_LT(result.yaw, 0.0);  // 装甲板在右侧，yaw应该为负
    
    // 验证x坐标为正（装甲板在右侧）
    EXPECT_GT(result.position(0), 0.0);
}

/**
 * @test 测试大装甲板解算
 */
TEST_F(PoseSolverTest, SolveLargeArmor)
{
    solver_.setCameraIntrinsics(test_intrinsics_);
    
    // 使用2米距离，确保投影尺寸足够大
    double distance = 2.0;
    double armor_width = 0.225;  // 大装甲板宽度
    double armor_height = 0.055;
    
    double proj_half_width = (armor_width / 2.0) * test_intrinsics_.fx / distance;
    double proj_half_height = (armor_height / 2.0) * test_intrinsics_.fy / distance;
    
    std::vector<cv::Point2f> corners = {
        cv::Point2f(test_intrinsics_.cx - proj_half_width, test_intrinsics_.cy - proj_half_height),
        cv::Point2f(test_intrinsics_.cx + proj_half_width, test_intrinsics_.cy - proj_half_height),
        cv::Point2f(test_intrinsics_.cx + proj_half_width, test_intrinsics_.cy + proj_half_height),
        cv::Point2f(test_intrinsics_.cx - proj_half_width, test_intrinsics_.cy + proj_half_height)
    };
    
    ArmorPose result = solver_.solve(corners, ArmorType::LARGE);
    
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.armor_type, ArmorType::LARGE);
    
    // 验证距离（允许10%误差）
    EXPECT_NEAR(result.distance, distance, distance * 0.1);
}

/**
 * @test 测试使用数组形式的角点
 */
TEST_F(PoseSolverTest, SolveWithArrayCorners)
{
    solver_.setCameraIntrinsics(test_intrinsics_);
    
    cv::Point2f corners[4] = {
        cv::Point2f(596.5, 342.0),
        cv::Point2f(683.5, 342.0),
        cv::Point2f(683.5, 378.0),
        cv::Point2f(596.5, 378.0)
    };
    
    ArmorPose result = solver_.solve(corners, ArmorType::SMALL);
    
    EXPECT_TRUE(result.valid);
    EXPECT_GT(result.distance, 0.0);
}

/**
 * @test 测试带IMU角度的解算
 */
TEST_F(PoseSolverTest, SolveWithIMUAngles)
{
    solver_.setCameraIntrinsics(test_intrinsics_);
    
    double distance = 2.0;
    double armor_width = 0.135;
    double armor_height = 0.055;
    
    double proj_half_width = (armor_width / 2.0) * test_intrinsics_.fx / distance;
    double proj_half_height = (armor_height / 2.0) * test_intrinsics_.fy / distance;
    
    std::vector<cv::Point2f> corners = {
        cv::Point2f(test_intrinsics_.cx - proj_half_width, test_intrinsics_.cy - proj_half_height),
        cv::Point2f(test_intrinsics_.cx + proj_half_width, test_intrinsics_.cy - proj_half_height),
        cv::Point2f(test_intrinsics_.cx + proj_half_width, test_intrinsics_.cy + proj_half_height),
        cv::Point2f(test_intrinsics_.cx - proj_half_width, test_intrinsics_.cy + proj_half_height)
    };
    
    // 测试带IMU角度的解算
    double imu_yaw = 10.0;    // 10度
    double imu_pitch = 5.0;   // 5度
    
    ArmorPose result = solver_.solve(corners, ArmorType::SMALL, imu_yaw, imu_pitch);
    
    EXPECT_TRUE(result.valid);
    
    // 验证yaw和pitch包含了IMU角度的偏移
    double imu_yaw_rad = imu_yaw * M_PI / 180.0;
    double imu_pitch_rad = imu_pitch * M_PI / 180.0;
    
    // yaw应该接近imu_yaw（因为装甲板在正前方）
    EXPECT_NEAR(result.yaw, imu_yaw_rad, 0.1);
    // pitch应该接近imu_pitch
    EXPECT_NEAR(result.pitch, imu_pitch_rad, 0.1);
}

/**
 * @test 测试theta_world范围
 */
TEST_F(PoseSolverTest, ThetaWorldRange)
{
    solver_.setCameraIntrinsics(test_intrinsics_);
    
    double distance = 2.0;
    double armor_width = 0.135;
    double armor_height = 0.055;
    
    double proj_half_width = (armor_width / 2.0) * test_intrinsics_.fx / distance;
    double proj_half_height = (armor_height / 2.0) * test_intrinsics_.fy / distance;
    
    std::vector<cv::Point2f> corners = {
        cv::Point2f(test_intrinsics_.cx - proj_half_width, test_intrinsics_.cy - proj_half_height),
        cv::Point2f(test_intrinsics_.cx + proj_half_width, test_intrinsics_.cy - proj_half_height),
        cv::Point2f(test_intrinsics_.cx + proj_half_width, test_intrinsics_.cy + proj_half_height),
        cv::Point2f(test_intrinsics_.cx - proj_half_width, test_intrinsics_.cy + proj_half_height)
    };
    
    ArmorPose result = solver_.solve(corners, ArmorType::SMALL);
    
    EXPECT_TRUE(result.valid);
    
    // 验证theta_world在[-π, π]范围内
    EXPECT_GE(result.theta_world, -M_PI);
    EXPECT_LE(result.theta_world, M_PI);
}

// ============================================================================
// 解算结果完整性测试 (Property 6)
// ============================================================================

// ============================================================================
// YAML配置文件加载测试 (Requirements 2.3)
// ============================================================================

/**
 * @test 测试从YAML文件加载相机内参
 */
TEST_F(PoseSolverTest, LoadCameraIntrinsicsFromYAML)
{
    // 使用测试数据目录中的配置文件
    std::string yaml_path = std::string(TEST_DATA_DIR) + "/test_camera_intrinsics.yaml";
    
    bool result = solver_.loadCameraIntrinsicsFromYAML(yaml_path);
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(solver_.isCameraIntrinsicsSet());
    
    const auto& intrinsics = solver_.getCameraIntrinsics();
    EXPECT_DOUBLE_EQ(intrinsics.fx, 1500.0);
    EXPECT_DOUBLE_EQ(intrinsics.fy, 1500.0);
    EXPECT_DOUBLE_EQ(intrinsics.cx, 640.0);
    EXPECT_DOUBLE_EQ(intrinsics.cy, 360.0);
    EXPECT_DOUBLE_EQ(intrinsics.k1, 0.1);
    EXPECT_DOUBLE_EQ(intrinsics.k2, 0.01);
    EXPECT_DOUBLE_EQ(intrinsics.p1, 0.001);
    EXPECT_DOUBLE_EQ(intrinsics.p2, 0.001);
    EXPECT_DOUBLE_EQ(intrinsics.k3, 0.0001);
}

/**
 * @test 测试从YAML文件加载相机内参（替代格式）
 */
TEST_F(PoseSolverTest, LoadCameraIntrinsicsFromYAMLAltFormat)
{
    std::string yaml_path = std::string(TEST_DATA_DIR) + "/test_camera_intrinsics_alt.yaml";
    
    bool result = solver_.loadCameraIntrinsicsFromYAML(yaml_path);
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(solver_.isCameraIntrinsicsSet());
    
    const auto& intrinsics = solver_.getCameraIntrinsics();
    EXPECT_DOUBLE_EQ(intrinsics.fx, 1200.0);
    EXPECT_DOUBLE_EQ(intrinsics.fy, 1200.0);
    EXPECT_DOUBLE_EQ(intrinsics.cx, 320.0);
    EXPECT_DOUBLE_EQ(intrinsics.cy, 240.0);
}

/**
 * @test 测试加载不存在的YAML文件
 */
TEST_F(PoseSolverTest, LoadCameraIntrinsicsFromNonexistentFile)
{
    bool result = solver_.loadCameraIntrinsicsFromYAML("/nonexistent/path/config.yaml");
    
    EXPECT_FALSE(result);
    EXPECT_FALSE(solver_.isCameraIntrinsicsSet());
}

/**
 * @test 测试加载无效的YAML文件
 */
TEST_F(PoseSolverTest, LoadCameraIntrinsicsFromInvalidFile)
{
    std::string yaml_path = std::string(TEST_DATA_DIR) + "/test_camera_intrinsics_invalid.yaml";
    
    bool result = solver_.loadCameraIntrinsicsFromYAML(yaml_path);
    
    EXPECT_FALSE(result);
}

/**
 * @test 测试加载YAML后进行解算
 */
TEST_F(PoseSolverTest, SolveAfterLoadingYAML)
{
    std::string yaml_path = std::string(TEST_DATA_DIR) + "/test_camera_intrinsics.yaml";
    
    bool load_result = solver_.loadCameraIntrinsicsFromYAML(yaml_path);
    ASSERT_TRUE(load_result);
    
    // 使用加载的内参进行解算
    const auto& intrinsics = solver_.getCameraIntrinsics();
    double distance = 2.0;
    double armor_width = 0.135;
    double armor_height = 0.055;
    
    double proj_half_width = (armor_width / 2.0) * intrinsics.fx / distance;
    double proj_half_height = (armor_height / 2.0) * intrinsics.fy / distance;
    
    std::vector<cv::Point2f> corners = {
        cv::Point2f(intrinsics.cx - proj_half_width, intrinsics.cy - proj_half_height),
        cv::Point2f(intrinsics.cx + proj_half_width, intrinsics.cy - proj_half_height),
        cv::Point2f(intrinsics.cx + proj_half_width, intrinsics.cy + proj_half_height),
        cv::Point2f(intrinsics.cx - proj_half_width, intrinsics.cy + proj_half_height)
    };
    
    ArmorPose result = solver_.solve(corners, ArmorType::SMALL);
    
    EXPECT_TRUE(result.valid);
    EXPECT_NEAR(result.distance, distance, distance * 0.1);
}

// ============================================================================
// 解算结果完整性测试 (Property 6)
// ============================================================================

/**
 * @test 验证解算结果的完整性
 * 
 * **Feature: ros2-vision-migration, Property 6: 解算结果完整性**
 * **Validates: Requirements 2.2**
 * 
 * 对于任何解算完成的装甲板，输出应该包含有效的:
 * - yaw: [-π, π]
 * - pitch: [-π/2, π/2]
 * - distance: 正数
 * - 世界坐标
 */
TEST_F(PoseSolverTest, SolveResultCompleteness)
{
    solver_.setCameraIntrinsics(test_intrinsics_);
    
    // 测试多个不同位置的装甲板
    std::vector<std::pair<double, double>> offsets = {
        {0, 0},      // 中心
        {100, 0},    // 右侧
        {-100, 0},   // 左侧
        {0, 50},     // 下方
        {0, -50},    // 上方
        {100, 50},   // 右下
        {-100, -50}  // 左上
    };
    
    double distance = 2.0;
    double armor_width = 0.135;
    double armor_height = 0.055;
    
    double proj_half_width = (armor_width / 2.0) * test_intrinsics_.fx / distance;
    double proj_half_height = (armor_height / 2.0) * test_intrinsics_.fy / distance;
    
    for (const auto& offset : offsets)
    {
        double ox = offset.first;
        double oy = offset.second;
        
        std::vector<cv::Point2f> corners = {
            cv::Point2f(test_intrinsics_.cx + ox - proj_half_width, test_intrinsics_.cy + oy - proj_half_height),
            cv::Point2f(test_intrinsics_.cx + ox + proj_half_width, test_intrinsics_.cy + oy - proj_half_height),
            cv::Point2f(test_intrinsics_.cx + ox + proj_half_width, test_intrinsics_.cy + oy + proj_half_height),
            cv::Point2f(test_intrinsics_.cx + ox - proj_half_width, test_intrinsics_.cy + oy + proj_half_height)
        };
        
        ArmorPose result = solver_.solve(corners, ArmorType::SMALL);
        
        EXPECT_TRUE(result.valid) << "Failed for offset (" << ox << ", " << oy << ")";
        
        // 验证yaw范围 [-π, π]
        EXPECT_GE(result.yaw, -M_PI) << "yaw out of range for offset (" << ox << ", " << oy << ")";
        EXPECT_LE(result.yaw, M_PI) << "yaw out of range for offset (" << ox << ", " << oy << ")";
        
        // 验证pitch范围 [-π/2, π/2]
        EXPECT_GE(result.pitch, -M_PI / 2) << "pitch out of range for offset (" << ox << ", " << oy << ")";
        EXPECT_LE(result.pitch, M_PI / 2) << "pitch out of range for offset (" << ox << ", " << oy << ")";
        
        // 验证distance为正数
        EXPECT_GT(result.distance, 0.0) << "distance not positive for offset (" << ox << ", " << oy << ")";
        
        // 验证位置向量不是NaN
        EXPECT_FALSE(std::isnan(result.position(0))) << "position.x is NaN for offset (" << ox << ", " << oy << ")";
        EXPECT_FALSE(std::isnan(result.position(1))) << "position.y is NaN for offset (" << ox << ", " << oy << ")";
        EXPECT_FALSE(std::isnan(result.position(2))) << "position.z is NaN for offset (" << ox << ", " << oy << ")";
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
