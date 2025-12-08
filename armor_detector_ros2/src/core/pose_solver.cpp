/**
 * @file pose_solver.cpp
 * @brief PnP位姿解算器实现
 * 
 * 参考 OrangeAim-Drone 的 PoseSolver 实现。
 * 
 * Requirements: 2.1, 2.3
 */

#include "core/pose_solver.hpp"
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <iostream>
#include <cmath>

namespace SOLVER
{

PoseSolver::PoseSolver()
{
    // 初始化小装甲板3D坐标点
    // 顺序: 左上, 右上, 右下, 左下 (与检测器输出一致)
    points_small_3d_ = {
        cv::Point3f(-kSmallArmorHalfWidth, -kSmallArmorHalfHeight, 0.0f),  // 左上
        cv::Point3f(kSmallArmorHalfWidth, -kSmallArmorHalfHeight, 0.0f),   // 右上
        cv::Point3f(kSmallArmorHalfWidth, kSmallArmorHalfHeight, 0.0f),    // 右下
        cv::Point3f(-kSmallArmorHalfWidth, kSmallArmorHalfHeight, 0.0f)    // 左下
    };
    
    // 初始化大装甲板3D坐标点
    points_large_3d_ = {
        cv::Point3f(-kLargeArmorHalfWidth, -kLargeArmorHalfHeight, 0.0f),  // 左上
        cv::Point3f(kLargeArmorHalfWidth, -kLargeArmorHalfHeight, 0.0f),   // 右上
        cv::Point3f(kLargeArmorHalfWidth, kLargeArmorHalfHeight, 0.0f),    // 右下
        cv::Point3f(-kLargeArmorHalfWidth, kLargeArmorHalfHeight, 0.0f)    // 左下
    };
    
    // 初始化OpenCV矩阵
    intrinsic_matrix_ = cv::Mat::zeros(3, 3, CV_64FC1);
    distortion_coeffs_ = cv::Mat::zeros(5, 1, CV_64FC1);
}

void PoseSolver::setCameraIntrinsics(const CameraIntrinsics& intrinsics)
{
    intrinsics_ = intrinsics;
    updateCVMatrices();
}

void PoseSolver::setCameraMatrix(double fx, double fy, double cx, double cy,
                                  double k1, double k2, double p1, double p2, double k3)
{
    intrinsics_.fx = fx;
    intrinsics_.fy = fy;
    intrinsics_.cx = cx;
    intrinsics_.cy = cy;
    intrinsics_.k1 = k1;
    intrinsics_.k2 = k2;
    intrinsics_.p1 = p1;
    intrinsics_.p2 = p2;
    intrinsics_.k3 = k3;
    
    updateCVMatrices();
}

void PoseSolver::updateCVMatrices()
{
    // 设置内参矩阵
    intrinsic_matrix_ = cv::Mat::zeros(3, 3, CV_64FC1);
    intrinsic_matrix_.at<double>(0, 0) = intrinsics_.fx;
    intrinsic_matrix_.at<double>(0, 2) = intrinsics_.cx;
    intrinsic_matrix_.at<double>(1, 1) = intrinsics_.fy;
    intrinsic_matrix_.at<double>(1, 2) = intrinsics_.cy;
    intrinsic_matrix_.at<double>(2, 2) = 1.0;
    
    // 设置畸变系数
    distortion_coeffs_ = cv::Mat::zeros(5, 1, CV_64FC1);
    distortion_coeffs_.at<double>(0, 0) = intrinsics_.k1;
    distortion_coeffs_.at<double>(1, 0) = intrinsics_.k2;
    distortion_coeffs_.at<double>(2, 0) = intrinsics_.p1;
    distortion_coeffs_.at<double>(3, 0) = intrinsics_.p2;
    distortion_coeffs_.at<double>(4, 0) = intrinsics_.k3;
}

bool PoseSolver::loadCameraIntrinsicsFromYAML(const std::string& yaml_path)
{
    try
    {
        cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            std::cerr << "Failed to open camera config file: " << yaml_path << std::endl;
            return false;
        }
        
        // 尝试读取相机内参
        cv::Mat camera_matrix, dist_coeffs;
        
        // 支持多种常见的YAML格式
        if (fs["camera_matrix"].isMap())
        {
            fs["camera_matrix"] >> camera_matrix;
        }
        else if (fs["K"].isMap() || fs["K"].isSeq())
        {
            fs["K"] >> camera_matrix;
        }
        else if (fs["intrinsic_matrix"].isMap())
        {
            fs["intrinsic_matrix"] >> camera_matrix;
        }
        
        if (fs["distortion_coefficients"].isMap() || fs["distortion_coefficients"].isSeq())
        {
            fs["distortion_coefficients"] >> dist_coeffs;
        }
        else if (fs["D"].isMap() || fs["D"].isSeq())
        {
            fs["D"] >> dist_coeffs;
        }
        else if (fs["dist_coeffs"].isMap() || fs["dist_coeffs"].isSeq())
        {
            fs["dist_coeffs"] >> dist_coeffs;
        }
        
        fs.release();
        
        // 验证并设置内参
        if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3)
        {
            std::cerr << "Invalid camera matrix in config file" << std::endl;
            return false;
        }
        
        // 转换为double类型
        camera_matrix.convertTo(camera_matrix, CV_64FC1);
        
        intrinsics_.fx = camera_matrix.at<double>(0, 0);
        intrinsics_.fy = camera_matrix.at<double>(1, 1);
        intrinsics_.cx = camera_matrix.at<double>(0, 2);
        intrinsics_.cy = camera_matrix.at<double>(1, 2);
        
        // 读取畸变系数（可选）
        if (!dist_coeffs.empty())
        {
            dist_coeffs.convertTo(dist_coeffs, CV_64FC1);
            if (dist_coeffs.total() >= 4)
            {
                intrinsics_.k1 = dist_coeffs.at<double>(0);
                intrinsics_.k2 = dist_coeffs.at<double>(1);
                intrinsics_.p1 = dist_coeffs.at<double>(2);
                intrinsics_.p2 = dist_coeffs.at<double>(3);
                if (dist_coeffs.total() >= 5)
                {
                    intrinsics_.k3 = dist_coeffs.at<double>(4);
                }
            }
        }
        
        updateCVMatrices();
        return true;
    }
    catch (const cv::Exception& e)
    {
        std::cerr << "OpenCV exception while loading camera config: " << e.what() << std::endl;
        return false;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception while loading camera config: " << e.what() << std::endl;
        return false;
    }
}

ArmorPose PoseSolver::solve(const std::vector<cv::Point2f>& corners,
                             ArmorType armor_type,
                             double imu_yaw,
                             double imu_pitch)
{
    ArmorPose result;
    result.armor_type = armor_type;
    
    // 检查输入有效性
    if (corners.size() != 4)
    {
        std::cerr << "Invalid number of corners: " << corners.size() << ", expected 4" << std::endl;
        return result;
    }
    
    // 检查相机内参是否已设置
    if (!isCameraIntrinsicsSet())
    {
        std::cerr << "Camera intrinsics not set" << std::endl;
        return result;
    }
    
    // 选择对应的3D点
    const std::vector<cv::Point3f>& points_3d = 
        (armor_type == ArmorType::LARGE) ? points_large_3d_ : points_small_3d_;
    
    // PnP解算
    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(points_3d, corners, 
                                 intrinsic_matrix_, distortion_coeffs_,
                                 rvec, tvec, false, cv::SOLVEPNP_IPPE);
    
    if (!success)
    {
        std::cerr << "solvePnP failed" << std::endl;
        return result;
    }
    
    // 转换旋转向量为旋转矩阵
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    
    // 转换为Eigen格式
    Eigen::Vector3d e_tvec;
    Eigen::Matrix3d e_rotation;
    cv::cv2eigen(tvec, e_tvec);
    cv::cv2eigen(rotation_matrix, e_rotation);
    
    // 设置位置和旋转
    result.position = e_tvec;
    result.rotation = e_rotation;
    
    // 计算距离
    result.distance = e_tvec.norm();
    
    // 计算装甲板朝向角 theta_world
    // 参考 OrangeAim-Drone: theta_world = -atan2(R[2,0], R[0,0])
    result.theta_world = -std::atan2(rotation_matrix.at<double>(2, 0), 
                                      rotation_matrix.at<double>(0, 0));
    
    // 将theta_world限制在[-π, π]范围内
    while (result.theta_world > M_PI) result.theta_world -= 2 * M_PI;
    while (result.theta_world < -M_PI) result.theta_world += 2 * M_PI;
    
    // 计算相对云台的pitch和yaw角
    // 参考 OrangeAim-Drone 的计算方式
    // pitch = imu_pitch + atan2(-y, sqrt(z^2 + x^2))
    // yaw = imu_yaw - atan2(x, z)
    double imu_pitch_rad = imu_pitch * M_PI / 180.0;
    double imu_yaw_rad = imu_yaw * M_PI / 180.0;
    
    result.pitch = imu_pitch_rad + std::atan2(-e_tvec(1), 
                                               std::sqrt(e_tvec(2) * e_tvec(2) + e_tvec(0) * e_tvec(0)));
    result.yaw = imu_yaw_rad - std::atan2(e_tvec(0), e_tvec(2));
    
    result.valid = true;
    return result;
}

ArmorPose PoseSolver::solve(const cv::Point2f corners[4],
                             ArmorType armor_type,
                             double imu_yaw,
                             double imu_pitch)
{
    std::vector<cv::Point2f> corners_vec(corners, corners + 4);
    return solve(corners_vec, armor_type, imu_yaw, imu_pitch);
}

void PoseSolver::point2Angle(const cv::Point2f& p, double& delta_yaw, double& delta_pitch)
{
    double u = (p.x - intrinsics_.cx) / intrinsics_.fx;
    double v = (p.y - intrinsics_.cy) / intrinsics_.fy;
    
    delta_yaw = std::atan(u);
    delta_pitch = std::atan(v);
}

} // namespace SOLVER
