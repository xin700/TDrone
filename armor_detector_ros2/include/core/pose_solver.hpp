/**
 * @file pose_solver.hpp
 * @brief PnP位姿解算器 - 重构版本
 * 
 * 坐标系定义（非常重要！）：
 * ==========================
 * 
 * 1. 相机坐标系 (Camera Frame) - OpenCV标准:
 *    - X: 右
 *    - Y: 下
 *    - Z: 前（光轴方向）
 *    - 原点: 相机光心
 * 
 * 2. 云台坐标系 (Gimbal Frame):
 *    - X: 右
 *    - Y: 前
 *    - Z: 上
 *    - 原点: 云台旋转中心（约等于相机位置）
 *    - 当yaw=pitch=0时，与机架坐标系重合
 * 
 * 3. 机架坐标系 (Body Frame):
 *    - X: 右
 *    - Y: 前（机头方向）
 *    - Z: 上
 *    - 原点: 机架中心
 *    - 右手系: X×Y=Z
 * 
 * 云台IMU输入定义：
 * =================
 * - yaw: 向左为正（绕Z轴，从上往下看逆时针为正）—— 符合右手系
 * - pitch: 向上为正（绕X轴，抬头为正）
 * - 云台 yaw=0, pitch=0 时，相机光轴指向机架前方（Y正方向）
 * 
 * 坐标系转换链：
 * ==============
 * 1. PnP解算 → 相机坐标系位置 (position_cam)
 * 2. 相机→云台: T_gimbal_camera
 *    - Gimbal_X =  Camera_X  (右→右)
 *    - Gimbal_Y =  Camera_Z  (前→前)
 *    - Gimbal_Z = -Camera_Y  (下取反→上)
 * 3. 云台→机架: R_body_gimbal = (R_gimbal_body)^(-1)
 *    - R_gimbal_body = R_yaw * R_pitch (先yaw后pitch，本体坐标系旋转)
 *    - 所以 R_body_gimbal = R_pitch^(-1) * R_yaw^(-1)
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
    // ==================== 相机坐标系（PnP直接输出）====================
    Eigen::Vector3d position_cam;   ///< 相机坐标系下的位置 (m): X右, Y下, Z前
    Eigen::Matrix3d rotation_cam;   ///< 相机坐标系下的旋转矩阵 (装甲板→相机)
    cv::Mat rvec;                   ///< 旋转向量（用于重投影）
    cv::Mat tvec;                   ///< 位移向量（用于重投影）
    
    // ==================== 坐标系转换矩阵（用于逆变换）====================
    Eigen::Matrix3d T_gimbal_camera;  ///< 相机→云台转换矩阵（正交，逆=转置）
    Eigen::Matrix3d R_body_gimbal;    ///< 云台→机架旋转矩阵（正交，逆=转置）
    
    // ==================== 云台坐标系 ====================
    Eigen::Vector3d position_gimbal;  ///< 云台坐标系下的位置 (m): X右, Y前, Z上
    
    // ==================== 机架坐标系（最终输出）====================
    Eigen::Vector3d position;       ///< 机架坐标系下的位置 (m): X右, Y前, Z上
    Eigen::Matrix3d rotation;       ///< 机架坐标系下的旋转矩阵 (装甲板→机架)
    
    // 装甲板方向向量（机架坐标系）
    Eigen::Vector3d normal;         ///< 装甲板法向量（从装甲板指向相机）
    Eigen::Vector3d armor_x;        ///< 装甲板X轴（从相机看向左）
    Eigen::Vector3d armor_y;        ///< 装甲板Y轴（向上）
    
    // 4个角点在机架坐标系下的3D位置
    Eigen::Vector3d corners_3d[4];  ///< 角点3D坐标（左上、左下、右下、右上）
    
    // ==================== 角度信息（弧度）====================
    double yaw = 0.0;               ///< 目标相对机架的yaw角，正值=目标在右边
    double pitch = 0.0;             ///< 目标相对机架的pitch角，正值=目标在上方
    double distance = 0.0;          ///< 距离 (m)
    
    // 装甲板自身姿态（法向量的朝向）
    double armor_yaw = 0.0;         ///< 装甲板法向量的yaw角
    double armor_pitch = 0.0;       ///< 装甲板法向量的pitch角
    double theta_world = 0.0;       ///< 兼容旧接口，等于armor_yaw
    
    // ==================== 元信息 ====================
    int color_id = 0;               ///< 颜色ID
    int tag_id = 0;                 ///< 标签ID
    ArmorType armor_type = ArmorType::SMALL;  ///< 装甲板类型
    
    bool valid = false;             ///< 解算是否有效
    
    ArmorPose() 
        : position_cam(Eigen::Vector3d::Zero())
        , rotation_cam(Eigen::Matrix3d::Identity())
        , T_gimbal_camera(Eigen::Matrix3d::Identity())
        , R_body_gimbal(Eigen::Matrix3d::Identity())
        , position_gimbal(Eigen::Vector3d::Zero())
        , position(Eigen::Vector3d::Zero())
        , rotation(Eigen::Matrix3d::Identity())
        , normal(Eigen::Vector3d::UnitY())
        , armor_x(Eigen::Vector3d::UnitX())
        , armor_y(Eigen::Vector3d::UnitZ())
    {
        for (int i = 0; i < 4; i++) {
            corners_3d[i] = Eigen::Vector3d::Zero();
        }
    }
};

/**
 * @brief 相机内参结构体
 */
struct CameraIntrinsics
{
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double k1 = 0.0;
    double k2 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    double k3 = 0.0;
    
    bool isValid() const { return fx > 0 && fy > 0 && cx > 0 && cy > 0; }
};

/**
 * @class PoseSolver
 * @brief PnP位姿解算器
 */
class PoseSolver
{
public:
    PoseSolver();
    ~PoseSolver() = default;
    
    // 设置相机内参
    void setCameraIntrinsics(const CameraIntrinsics& intrinsics);
    void setCameraMatrix(double fx, double fy, double cx, double cy,
                         double k1 = 0.0, double k2 = 0.0, 
                         double p1 = 0.0, double p2 = 0.0, double k3 = 0.0);
    bool loadCameraIntrinsicsFromYAML(const std::string& yaml_path);
    
    /**
     * @brief 解算单个装甲板的位姿
     * @param corners 装甲板四个角点坐标（左上、左下、右下、右上）
     * @param armor_type 装甲板类型
     * @param imu_yaw IMU的yaw角（度），向右为正
     * @param imu_pitch IMU的pitch角（度），向上为正
     * @return 装甲板位姿
     */
    ArmorPose solve(const std::vector<cv::Point2f>& corners,
                    ArmorType armor_type,
                    double imu_yaw = 0.0,
                    double imu_pitch = 0.0);
    
    ArmorPose solve(const cv::Point2f corners[4],
                    ArmorType armor_type,
                    double imu_yaw = 0.0,
                    double imu_pitch = 0.0);
    
    // Getters
    const CameraIntrinsics& getCameraIntrinsics() const { return intrinsics_; }
    bool isCameraIntrinsicsSet() const { return intrinsics_.isValid(); }
    const std::vector<cv::Point3f>& getSmallArmorPoints() const { return points_small_3d_; }
    const std::vector<cv::Point3f>& getLargeArmorPoints() const { return points_large_3d_; }
    const cv::Mat& getCameraMatrix() const { return intrinsic_matrix_; }
    const cv::Mat& getDistCoeffs() const { return distortion_coeffs_; }

private:
    void updateCVMatrices();

private:
    CameraIntrinsics intrinsics_;
    cv::Mat intrinsic_matrix_;
    cv::Mat distortion_coeffs_;
    
    // 装甲板尺寸（单位：米）
    static constexpr float kSmallArmorHalfWidth = 0.0675f;   // 135mm/2
    static constexpr float kSmallArmorHalfHeight = 0.0275f;  // 55mm/2
    static constexpr float kLargeArmorHalfWidth = 0.1125f;   // 225mm/2
    static constexpr float kLargeArmorHalfHeight = 0.0275f;  // 55mm/2
    
    // 装甲板3D坐标点（装甲板坐标系：原点在中心，X左Y上Z朝外/朝向相机）
    // 角点顺序必须与检测器输出一致: 左上(0), 左下(1), 右下(2), 右上(3)
    std::vector<cv::Point3f> points_small_3d_;
    std::vector<cv::Point3f> points_large_3d_;
};

} // namespace SOLVER
