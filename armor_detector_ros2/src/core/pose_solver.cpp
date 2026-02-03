/**
 * @file pose_solver.cpp
 * @brief PnP位姿解算器实现 - 重构版本
 * 
 * 参考verify_pnp.cpp中经过验证的PnP解算逻辑
 * 
 * 坐标系转换链：
 * ==============
 * 装甲板 → (PnP) → 相机坐标系 → (T_gimbal_camera) → 云台坐标系 → (云台旋转补偿) → 机架坐标系
 * 
 * 关键转换矩阵：
 * 1. T_gimbal_camera: 相机→云台
 *    相机: X右, Y下, Z前
 *    云台: X右, Y前, Z上
 *    变换: Gx = Cx, Gy = Cz, Gz = -Cy
 * 
 * 2. R_body_gimbal: 云台→机架
 *    云台按先yaw后pitch的顺序从机架旋转而来
 *    逆变换为: R_body_gimbal = R_yaw^T * R_pitch^T
 */

#include "core/pose_solver.hpp"
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <cmath>

namespace SOLVER
{

PoseSolver::PoseSolver()
{
    // 初始化装甲板3D坐标点
    // 装甲板坐标系: 原点在中心, X指向左(从相机看), Y指向上, Z指向相机(法向量)
    // 角点顺序: 左上(0), 左下(1), 右下(2), 右上(3)
    // 从相机视角看：
    //   左上: X+, Y+  (左上方)
    //   左下: X+, Y-  (左下方)
    //   右下: X-, Y-  (右下方)
    //   右上: X-, Y+  (右上方)
    
    float sw = kSmallArmorHalfWidth;
    float sh = kSmallArmorHalfHeight;
    points_small_3d_ = {
        cv::Point3f( sw,  sh, 0.0f),  // 左上 (0)
        cv::Point3f( sw, -sh, 0.0f),  // 左下 (1)
        cv::Point3f(-sw, -sh, 0.0f),  // 右下 (2)
        cv::Point3f(-sw,  sh, 0.0f)   // 右上 (3)
    };
    
    float lw = kLargeArmorHalfWidth;
    float lh = kLargeArmorHalfHeight;
    points_large_3d_ = {
        cv::Point3f( lw,  lh, 0.0f),  // 左上 (0)
        cv::Point3f( lw, -lh, 0.0f),  // 左下 (1)
        cv::Point3f(-lw, -lh, 0.0f),  // 右下 (2)
        cv::Point3f(-lw,  lh, 0.0f)   // 右上 (3)
    };
    
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
    intrinsic_matrix_ = cv::Mat::zeros(3, 3, CV_64FC1);
    intrinsic_matrix_.at<double>(0, 0) = intrinsics_.fx;
    intrinsic_matrix_.at<double>(0, 2) = intrinsics_.cx;
    intrinsic_matrix_.at<double>(1, 1) = intrinsics_.fy;
    intrinsic_matrix_.at<double>(1, 2) = intrinsics_.cy;
    intrinsic_matrix_.at<double>(2, 2) = 1.0;
    
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
            std::cerr << "[PoseSolver] Failed to open camera config: " << yaml_path << std::endl;
            return false;
        }
        
        cv::Mat camera_matrix, dist_coeffs;
        
        // 支持多种YAML格式
        if (fs["camera_matrix"].isMap()) {
            fs["camera_matrix"] >> camera_matrix;
        } else if (fs["K"].isMap() || fs["K"].isSeq()) {
            fs["K"] >> camera_matrix;
        } else if (fs["intrinsic_matrix"].isMap()) {
            fs["intrinsic_matrix"] >> camera_matrix;
        }
        
        if (fs["distortion_coefficients"].isMap() || fs["distortion_coefficients"].isSeq()) {
            fs["distortion_coefficients"] >> dist_coeffs;
        } else if (fs["D"].isMap() || fs["D"].isSeq()) {
            fs["D"] >> dist_coeffs;
        } else if (fs["dist_coeffs"].isMap() || fs["dist_coeffs"].isSeq()) {
            fs["dist_coeffs"] >> dist_coeffs;
        }
        
        fs.release();
        
        if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3)
        {
            std::cerr << "[PoseSolver] Invalid camera matrix" << std::endl;
            return false;
        }
        
        camera_matrix.convertTo(camera_matrix, CV_64FC1);
        intrinsics_.fx = camera_matrix.at<double>(0, 0);
        intrinsics_.fy = camera_matrix.at<double>(1, 1);
        intrinsics_.cx = camera_matrix.at<double>(0, 2);
        intrinsics_.cy = camera_matrix.at<double>(1, 2);
        
        if (!dist_coeffs.empty())
        {
            dist_coeffs.convertTo(dist_coeffs, CV_64FC1);
            if (dist_coeffs.total() >= 4)
            {
                intrinsics_.k1 = dist_coeffs.at<double>(0);
                intrinsics_.k2 = dist_coeffs.at<double>(1);
                intrinsics_.p1 = dist_coeffs.at<double>(2);
                intrinsics_.p2 = dist_coeffs.at<double>(3);
                if (dist_coeffs.total() >= 5) {
                    intrinsics_.k3 = dist_coeffs.at<double>(4);
                }
            }
        }
        
        updateCVMatrices();
        std::cout << "[PoseSolver] Camera intrinsics loaded: fx=" << intrinsics_.fx 
                  << " fy=" << intrinsics_.fy << " cx=" << intrinsics_.cx 
                  << " cy=" << intrinsics_.cy << std::endl;
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[PoseSolver] Exception loading camera config: " << e.what() << std::endl;
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
    
    // 检查输入
    if (corners.size() != 4)
    {
        std::cerr << "[PoseSolver] Invalid corner count: " << corners.size() << std::endl;
        return result;
    }
    
    if (!isCameraIntrinsicsSet())
    {
        std::cerr << "[PoseSolver] Camera intrinsics not set" << std::endl;
        return result;
    }
    
    // 选择3D点
    const std::vector<cv::Point3f>& points_3d = 
        (armor_type == ArmorType::LARGE) ? points_large_3d_ : points_small_3d_;
    
    // ==================== Step 1: PnP解算 ====================
    cv::Mat rvec, tvec;
    // 使用ITERATIVE方法，比IPPE更稳定，不会产生两个歧义解
    bool success = cv::solvePnP(points_3d, corners, 
                                 intrinsic_matrix_, distortion_coeffs_,
                                 rvec, tvec, false, cv::SOLVEPNP_IPPE);
    
    if (!success)
    {
        std::cerr << "[PoseSolver] solvePnP failed" << std::endl;
        return result;
    }
    
    // 转换旋转向量为旋转矩阵
    cv::Mat R_cam_armor;  // 装甲板→相机的旋转矩阵
    cv::Rodrigues(rvec, R_cam_armor);
    
    // ==================== Step 2: 修复Z轴方向歧义 ====================
    // 装甲板的Z轴（法向量）应该指向相机（即在相机坐标系中Z分量为负）
    // 旋转矩阵的第三列是装甲板Z轴在相机坐标系中的方向
    cv::Vec3d z_axis(R_cam_armor.at<double>(0, 2),
                     R_cam_armor.at<double>(1, 2),
                     R_cam_armor.at<double>(2, 2));
    
    // 方法1：检查Z轴的Z分量（应该为负，指向相机）
    // 方法2：检查Z轴与tvec的点积（应该为负）
    // 这里使用更可靠的方法：确保Z轴指向相机方向（Z分量为负）
    cv::Vec3d tvec_dir(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    double dot = z_axis.dot(tvec_dir);
    
    // 如果Z轴与到装甲板的向量同向（点积>0），说明法向量背离相机，需要翻转
    if (dot > 0)
    {
        // 翻转：Z轴取反，同时X轴取反以保持右手系
        R_cam_armor.col(0) *= -1;
        R_cam_armor.col(2) *= -1;
        cv::Rodrigues(R_cam_armor, rvec);
    }
    
    // 保存相机坐标系结果（用于重投影验证）
    result.rvec = rvec.clone();
    result.tvec = tvec.clone();
    
    // 提取相机坐标系位置
    result.position_cam = Eigen::Vector3d(
        tvec.at<double>(0),
        tvec.at<double>(1),
        tvec.at<double>(2)
    );
    
    cv::cv2eigen(R_cam_armor, result.rotation_cam);
    
    // ==================== Step 3: 相机坐标系 → 云台坐标系 ====================
    // 相机: X右, Y下, Z前
    // 云台: X右, Y前, Z上
    // 转换: Gx = Cx, Gy = Cz, Gz = -Cy
    Eigen::Matrix3d T_gimbal_camera;
    T_gimbal_camera <<  1,  0,  0,   // Gx = Cx
                        0,  0,  1,   // Gy = Cz
                        0, -1,  0;   // Gz = -Cy
    
    // 保存转换矩阵（用于visualizer逆变换）
    result.T_gimbal_camera = T_gimbal_camera;
    
    // ========== 相机相对云台旋转中心的偏移（云台坐标系）==========
    // 云台坐标系：X右, Y前, Z上
    // 相机偏移参数（可调）：
    const double cam_offset_x = 0.0;    // 右偏移（正=相机在旋转中心右边）
    const double cam_offset_y = 0.00;    // 前偏移（正=相机在旋转中心前面）
    const double cam_offset_z = -0.10;  // 上偏移（负=相机在旋转中心下面，当前-10cm）
    Eigen::Vector3d camera_offset_gimbal(cam_offset_x, cam_offset_y, cam_offset_z);
    
    // 目标在云台坐标系中的位置 = 相机偏移 + 相机坐标系位置旋转到云台坐标系
    result.position_gimbal = camera_offset_gimbal + T_gimbal_camera * result.position_cam;
    
    // ==================== Step 4: 云台坐标系 → 机架坐标系 ====================
    //
    // 坐标系定义：
    //   Cam:    X右, Y下, Z前
    //   Gimbal: X右, Y前, Z上
    //   Body:   yaw=pitch=0 时的 Gimbal
    //
    // IMU角度定义：
    //   yaw: 向左为正（绕Z轴，从上往下看逆时针）
    //   pitch: 向上为正（绕X轴）
    //
    // 测试结果分析：
    //   - yaw补偿效果好（x变化减少78%）
    //   - pitch补偿方向反了（z变化反而增大）
    //   - 需要对pitch取反
    //
    // 修正后公式：R_body_gimbal = R_z(yaw) * R_x(pitch)
    //   注意：这里用 R_x(+pitch) 而不是 R_x(-pitch)
    //
    // 显式矩阵：
    //   [cy,  -sy*cp,  sy*sp]
    //   [sy,   cy*cp, -cy*sp]
    //   [0,       sp,     cp]
    
    double yaw_rad = imu_yaw * M_PI / 180.0;
    double pitch_rad = imu_pitch * M_PI / 180.0;
    
    double cy = std::cos(yaw_rad);
    double sy = std::sin(yaw_rad);
    double cp = std::cos(pitch_rad);
    double sp = std::sin(pitch_rad);
    
    Eigen::Matrix3d R_body_gimbal;
    R_body_gimbal <<  cy, -sy*cp,  sy*sp,
                      sy,  cy*cp, -cy*sp,
                       0,     sp,     cp;
    
    // 保存转换矩阵（用于visualizer逆变换）
    result.R_body_gimbal = R_body_gimbal;
    
    // 机架坐标系位置
    result.position = R_body_gimbal * result.position_gimbal;
    
    // ==================== Step 5: 计算装甲板朝向向量（机架坐标系）====================
    // 装甲板坐标系: X左, Y上, Z朝外(朝向相机)
    // 首先转换到云台坐标系，再转到机架坐标系
    
    // 装甲板→云台的旋转矩阵
    Eigen::Matrix3d R_gimbal_armor = T_gimbal_camera * result.rotation_cam;
    
    // 装甲板→机架的旋转矩阵
    result.rotation = R_body_gimbal * R_gimbal_armor;
    
    // 提取方向向量
    // rotation的列向量是装甲板坐标系各轴在机架坐标系中的方向
   
    // ==================== Step 6: 计算角点3D位置（机架坐标系）====================
    // 相机偏移（与Step 3保持一致！）
    Eigen::Vector3d camera_offset_gimbal_for_corners(cam_offset_x, cam_offset_y, cam_offset_z);
    
    for (int i = 0; i < 4; i++)
    {
        Eigen::Vector3d pt_armor(points_3d[i].x, points_3d[i].y, points_3d[i].z);
        // 装甲板→相机→云台→机架
        Eigen::Vector3d pt_cam = result.rotation_cam * pt_armor + result.position_cam;
        Eigen::Vector3d pt_gimbal = camera_offset_gimbal_for_corners + T_gimbal_camera * pt_cam;
        result.corners_3d[i] = R_body_gimbal * pt_gimbal;
    }
    
    // ==================== Step 7: 计算角度信息 ====================
    // 机架坐标系: X右, Y前, Z上
    double x = result.position.x();
    double y = result.position.y();
    double z = result.position.z();
    
    result.distance = result.position.norm();
    
    // yaw: 目标相对前方(Y轴)的水平偏角
    // 从上往下看，目标在右边时yaw为正
    // yaw = atan2(x, y)
    result.yaw = std::atan2(x, y);
    
    // pitch: 目标相对水平面的仰角
    // 目标在上方时pitch为正
    // pitch = atan2(z, sqrt(x^2 + y^2))
    result.pitch = std::atan2(z, std::sqrt(x*x + y*y));
    
    // 装甲板朝向角（法向量的朝向）
    // armor_yaw: 法向量在XY平面的投影与Y轴的夹角
    result.armor_yaw = std::atan2(result.normal.x(), result.normal.y());
    
    // armor_pitch: 法向量与XY平面的夹角
    result.armor_pitch = std::asin(result.normal.z() / result.normal.norm());
    
    // 兼容旧接口
    result.theta_world = result.armor_yaw;
    
    result.valid = true;
    return result;
}

ArmorPose PoseSolver::solve(const cv::Point2f corners[4],
                             ArmorType armor_type,
                             double imu_yaw,
                             double imu_pitch)
{
    std::cout<<"PoseSolver:"<<imu_pitch<<";"<<imu_yaw<<std::endl;
    std::vector<cv::Point2f> corners_vec(corners, corners + 4);
    return solve(corners_vec, armor_type, imu_yaw, imu_pitch);
}

} // namespace SOLVER
