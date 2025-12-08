/**
 * @file pose_solver.hpp
 * @brief PnP位姿解算器
 * 
 * 使用OpenCV的solvePnP算法计算装甲板在相机坐标系中的位姿。
 * 参考 OrangeAim-Drone 的 PoseSolver 实现。
 * 
 * Requirements: 2.1, 2.3
 */

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>

namespace SOLVER
{

/**
 * @brief 装甲板类型枚举
 */
enum class ArmorType
{
    SMALL = 0,  ///< 小装甲板
    LARGE = 1   ///< 大装甲板
};

/**
 * @brief 装甲板位姿结果
 */
struct ArmorPose
{
    double yaw = 0.0;           ///< 相对云台的yaw角 (rad)
    double pitch = 0.0;         ///< 相对云台的pitch角 (rad)
    double distance = 0.0;      ///< 距离 (m)
    double theta_world = 0.0;   ///< 装甲板朝向角 (rad)
    
    Eigen::Vector3d position;   ///< 相机坐标系下的位置 (m)
    Eigen::Matrix3d rotation;   ///< 相机坐标系下的旋转矩阵
    
    int color_id = 0;           ///< 颜色ID
    int tag_id = 0;             ///< 标签ID
    ArmorType armor_type = ArmorType::SMALL;  ///< 装甲板类型
    
    bool valid = false;         ///< 解算是否有效
    
    ArmorPose() : position(Eigen::Vector3d::Zero()), rotation(Eigen::Matrix3d::Identity()) {}
};

/**
 * @brief 相机内参结构体
 */
struct CameraIntrinsics
{
    double fx = 0.0;    ///< 焦距x
    double fy = 0.0;    ///< 焦距y
    double cx = 0.0;    ///< 主点x (u0)
    double cy = 0.0;    ///< 主点y (v0)
    double k1 = 0.0;    ///< 径向畸变系数1
    double k2 = 0.0;    ///< 径向畸变系数2
    double p1 = 0.0;    ///< 切向畸变系数1
    double p2 = 0.0;    ///< 切向畸变系数2
    double k3 = 0.0;    ///< 径向畸变系数3
    
    /**
     * @brief 检查内参是否有效
     */
    bool isValid() const
    {
        return fx > 0 && fy > 0 && cx > 0 && cy > 0;
    }
};

/**
 * @class PoseSolver
 * @brief PnP位姿解算器
 * 
 * 使用OpenCV的solvePnP算法，根据装甲板的2D角点坐标和已知的3D尺寸，
 * 计算装甲板在相机坐标系中的位姿。
 */
class PoseSolver
{
public:
    /**
     * @brief 默认构造函数
     */
    PoseSolver();
    
    /**
     * @brief 析构函数
     */
    ~PoseSolver() = default;
    
    /**
     * @brief 设置相机内参
     * @param intrinsics 相机内参结构体
     */
    void setCameraIntrinsics(const CameraIntrinsics& intrinsics);
    
    /**
     * @brief 设置相机内参（兼容旧接口）
     * @param fx 焦距x
     * @param fy 焦距y
     * @param cx 主点x
     * @param cy 主点y
     * @param k1 径向畸变系数1
     * @param k2 径向畸变系数2
     * @param p1 切向畸变系数1
     * @param p2 切向畸变系数2
     * @param k3 径向畸变系数3
     */
    void setCameraMatrix(double fx, double fy, double cx, double cy,
                         double k1 = 0.0, double k2 = 0.0, 
                         double p1 = 0.0, double p2 = 0.0, double k3 = 0.0);
    
    /**
     * @brief 从YAML文件加载相机内参
     * @param yaml_path YAML文件路径
     * @return 是否加载成功
     */
    bool loadCameraIntrinsicsFromYAML(const std::string& yaml_path);
    
    /**
     * @brief 解算单个装甲板的位姿
     * @param corners 装甲板四个角点坐标 (左上, 右上, 右下, 左下)
     * @param armor_type 装甲板类型
     * @param imu_yaw IMU的yaw角 (度)
     * @param imu_pitch IMU的pitch角 (度)
     * @return 装甲板位姿
     */
    ArmorPose solve(const std::vector<cv::Point2f>& corners,
                    ArmorType armor_type,
                    double imu_yaw = 0.0,
                    double imu_pitch = 0.0);
    
    /**
     * @brief 解算单个装甲板的位姿（使用数组形式的角点）
     * @param corners 装甲板四个角点坐标数组
     * @param armor_type 装甲板类型
     * @param imu_yaw IMU的yaw角 (度)
     * @param imu_pitch IMU的pitch角 (度)
     * @return 装甲板位姿
     */
    ArmorPose solve(const cv::Point2f corners[4],
                    ArmorType armor_type,
                    double imu_yaw = 0.0,
                    double imu_pitch = 0.0);
    
    /**
     * @brief 获取当前相机内参
     * @return 相机内参结构体
     */
    const CameraIntrinsics& getCameraIntrinsics() const { return intrinsics_; }
    
    /**
     * @brief 检查相机内参是否已设置
     * @return 是否已设置有效的相机内参
     */
    bool isCameraIntrinsicsSet() const { return intrinsics_.isValid(); }
    
    /**
     * @brief 获取小装甲板的3D点
     */
    const std::vector<cv::Point3f>& getSmallArmorPoints() const { return points_small_3d_; }
    
    /**
     * @brief 获取大装甲板的3D点
     */
    const std::vector<cv::Point3f>& getLargeArmorPoints() const { return points_large_3d_; }

private:
    /**
     * @brief 将2D点转换为角度
     * @param p 2D点坐标
     * @param delta_yaw 输出的yaw角偏移
     * @param delta_pitch 输出的pitch角偏移
     */
    void point2Angle(const cv::Point2f& p, double& delta_yaw, double& delta_pitch);
    
    /**
     * @brief 更新OpenCV格式的相机矩阵
     */
    void updateCVMatrices();

private:
    // 相机内参
    CameraIntrinsics intrinsics_;
    cv::Mat intrinsic_matrix_;      ///< OpenCV格式的内参矩阵
    cv::Mat distortion_coeffs_;     ///< OpenCV格式的畸变系数
    
    // 装甲板物理尺寸 (单位: 米)
    // 参考 OrangeAim-Drone 的参数
    static constexpr float kSmallArmorHalfWidth = 0.0675f;   ///< 小装甲板半宽
    static constexpr float kSmallArmorHalfHeight = 0.0275f;  ///< 小装甲板半高
    static constexpr float kLargeArmorHalfWidth = 0.1125f;   ///< 大装甲板半宽
    static constexpr float kLargeArmorHalfHeight = 0.0275f;  ///< 大装甲板半高
    
    // 装甲板3D坐标点 (装甲板坐标系，中心为原点)
    std::vector<cv::Point3f> points_small_3d_;  ///< 小装甲板3D点
    std::vector<cv::Point3f> points_large_3d_;  ///< 大装甲板3D点
};

} // namespace SOLVER
