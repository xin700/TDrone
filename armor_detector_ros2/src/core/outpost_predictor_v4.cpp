/**
 * @file outpost_predictor_v4.cpp
 * @brief 前哨站预测器 V4 实现
 */

#include "core/outpost_predictor_v4.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace armor_detector
{

// ============================================================================
// 构造函数和配置
// ============================================================================

OutpostPredictorV4::OutpostPredictorV4()
{
    x_ = Eigen::VectorXd::Zero(STATE_DIM);
    P_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    
    // 打开 CSV 日志文件（清空模式）
    csv_log_file_.open("/ros2_ws/outpost_predictor_v4_log.csv", std::ios::out | std::ios::trunc);
    if (csv_log_file_.is_open()) {
        // 写入表头
        csv_log_file_ << "timestamp,x,y,z_raw,z_filtered,proj_t,phase_state,height_state" << std::endl;
        csv_log_file_.flush();
        std::cout << "[OutpostPredictorV4] CSV log file opened: /ros2_ws/outpost_predictor_v4_log.csv" << std::endl;
    } else {
        std::cerr << "[OutpostPredictorV4] ERROR: Failed to open CSV log file!" << std::endl;
    }
}

OutpostPredictorV4::OutpostPredictorV4(const OutpostConfigV4& config)
    : OutpostPredictorV4()
{
    setConfig(config);
}

void OutpostPredictorV4::setConfig(const OutpostConfigV4& config)
{
    config_ = config;
}

void OutpostPredictorV4::reset()
{
    init_phase_ = InitPhase::NOT_STARTED;
    init_frame_count_ = 0;
    
    rotation_direction_ = RotationDirection::STATIC;
    cx_history_.clear();
    
    // 静止确认重置
    static_confirmed_ = false;
    static_start_time_ = 0.0;
    last_static_check_x_ = 0.0;
    static_check_frames_ = 0;
    
    // X 范围重置
    x_range_initialized_ = false;
    x_init_values_.clear();
    x_min_ = 0.0;
    x_max_ = 0.0;
    last_x_ = 0.0;
    last_x_valid_ = false;
    
    // X 剧变检测重置
    pending_x_jump_ = false;
    pending_x_stable_frames_ = 0;
    pre_jump_filtered_z_ = 0.0;
    
    // Z 滤波重置
    z_filter_window_.clear();
    current_filtered_z_ = 0.0;
    
    current_height_state_ = HeightState::MIDDLE;
    predicted_next_height_ = HeightState::MIDDLE;
    height_initialized_ = false;
    height_verify_count_ = 0;
    height_lost_count_ = 0;
    height_sync_with_ekf_ = false;
    
    // 高度标定数据重置
    height_calibration_[0] = 0.0;
    height_calibration_[1] = 0.0;
    height_calibration_[2] = 0.0;
    height_calibrated_ = false;
    height_init_jump_count_ = 0;
    current_height_group_.clear();
    last_valid_z_ = 0.0;
    
    // 突变检测重置
    pending_jump_z_diff_ = 0.0;
    pending_jump_stable_frames_ = 0;
    pending_jump_active_ = false;
    pending_jump_last_z_ = 0.0;

    phase_state_ = PhaseState::RISING;
    first_z_jump_detected_ = false;
    max_aspect_ratio_recorded_ = 0.0;
    max_aspect_ratio_position_x_ = 0.0;
    max_aspect_ratio_position_y_ = 0.0;
    plateau_calib_ = PlateauCalibration();
    plateau_entered_ = false;
    plateau_aspect_ratio_peaked_ = false;
    
    // FALLING状态超时检测重置
    min_ar_in_falling_ = 1.0;
    falling_frames_ = 0;
    falling_start_time_ = 0.0;
    
    current_theta_ = 0.0;
    last_theta_ = 0.0;
    theta_initialized_ = false;
    
    x_ = Eigen::VectorXd::Zero(STATE_DIM);
    P_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    ekf_initialized_ = false;
    ekf_converged_ = false;
    ekf_update_count_ = 0;
    height_k_ = HeightState::MIDDLE;
    last_residual_ = Eigen::Vector4d::Zero();
    last_ekf_ang_ = 0.0;
    
    obs_history_.clear();
    last_timestamp_ = 0.0;
    
    last_t_gimbal_camera_ = Eigen::Matrix3d::Identity();
    last_r_body_gimbal_ = Eigen::Matrix3d::Identity();
}

// ============================================================================
// 主更新接口
// ============================================================================

bool OutpostPredictorV4::update(const ObservationV4& obs)
{
    // 计算dt（无论观测是否有效都需要）
    double dt = 0.0;
    if (last_timestamp_ > 0.0 && obs.timestamp > 0.0) {
        dt = obs.timestamp - last_timestamp_;
        if (dt < 0.0 || dt > 1.0) {
            dt = 0.01;  // 默认10ms
        }
    }
    if (obs.timestamp > 0.0) {
        last_timestamp_ = obs.timestamp;
    }
    
    // 记录日志到 CSV 文件
    if (csv_log_file_.is_open()) {
        double x_value = obs.valid ? obs.position.x() : -1.0;
        double y_value = obs.valid ? obs.position.y() : -1.0;
        double z_value = obs.valid ? obs.position.z() : -1.0;
        int phase_code = 0;  // 0=RISING, 1=PLATEAU, 2=FALLING
        if (phase_state_ == PhaseState::PLATEAU) phase_code = 1;
        else if (phase_state_ == PhaseState::FALLING) phase_code = 2;
        
        int height_code = static_cast<int>(current_height_state_);  // 0=LOW, 1=MIDDLE, 2=HIGH
        
        csv_log_file_ << obs.timestamp << "," 
                      << x_value << ","
                      << y_value << ","
                      << z_value << "," 
                      << current_filtered_z_ << ","
                      << last_x_ << ","
                      << phase_code << "," 
                      << height_code << std::endl;
    }
    
    if (!obs.valid) {
        // 无效观测，记录丢失
        height_lost_count_++;
        
        // 如果EKF已初始化，仍然执行预测步骤（纯预测，无更新）
        // ekfPredict 内部会调用 updateEKFHeightState() 处理高度状态
        if (init_phase_ == InitPhase::EKF_RUNNING && ekf_initialized_ && dt > 0.0) {
            ekfPredict(dt);
        }
        
        return false;  // 返回false表示本帧无有效观测，但EKF已做预测
    }
    
    // 保存观测历史
    obs_history_.push_back(obs);
    if (obs_history_.size() > MAX_HISTORY_SIZE) {
        obs_history_.pop_front();
    }
    
    // 保存最近的转换矩阵
    last_t_gimbal_camera_ = obs.t_gimbal_camera;
    last_r_body_gimbal_ = obs.r_body_gimbal;
    
    // 保存最近的IMU数据
    last_imu_yaw_ = obs.imu_yaw;
    last_imu_pitch_ = obs.imu_pitch;
    
    // 根据初始化阶段执行不同逻辑
    switch (init_phase_) {
        case InitPhase::NOT_STARTED:
            init_phase_ = InitPhase::DIRECTION_INIT;
            init_frame_count_ = 0;
            // fall through
            
        case InitPhase::DIRECTION_INIT:
            updateDirectionInit(obs);
            break;
            
        case InitPhase::XY_LINE_INIT:
            // X 范围初始化阶段
            updateXRangeInit(obs);
            break;
            
        case InitPhase::HEIGHT_INIT:
            updateHeightInit(obs);
            // 同时更新相位状态机
            updatePhaseState(obs);
            break;
            
        case InitPhase::THETA_INIT:
            // 在 THETA_INIT 阶段，只调用 updateThetaInit
            // 不要调用 updateHeightStateMachine，因为两者都会调用 detectConfirmedJump，
            // 导致跳变检测结果被重复消费
            updateThetaInit(obs);
            // updateHeightStateMachine(obs);  // 移除：避免重复消费跳变检测
            // updatePhaseState(obs);  // 已在 updateThetaInit 中调用
            break;
            
        case InitPhase::EKF_RUNNING:
            // EKF运行
            if (dt > 0.0) {
                ekfPredict(dt);
            }
            
            // 更新相位状态机和计算theta
            updatePhaseState(obs);
            
            // 计算当前theta
            double theta;
            bool should_update_ekf = true;
            
            if (obs.aspect_ratio > 0) {
                if (phase_state_ == PhaseState::PLATEAU && plateau_calib_.calibrated) {
                    theta = computePlateauTheta(obs.center_pixel_x);
                } else {
                    theta = computeTheta(obs.aspect_ratio, phase_state_);
                }
            } else {
                // 长宽比为-1，不更新EKF，让其自己预测滑过去
                theta = last_theta_;  // 保持上一帧的值用于记录
                should_update_ekf = false;
            }
            current_theta_ = theta;
            last_theta_ = theta;
            
            // EKF更新（仅当有有效的长宽比观测时）
            if (should_update_ekf) {
                ekfUpdate(obs, theta);
            }
            
            // 更新高度状态机
            updateHeightStateMachine(obs);
            
            // 检查收敛：结合帧数和协方差收敛性
            ekf_update_count_++;
            if (!ekf_converged_ && ekf_update_count_ >= config_.ekf_converge_frames) {
                // 检查协方差是否收敛（omega 和位置的不确定性足够小）
                bool omega_converged = P_(IDX_OMEGA, IDX_OMEGA) < config_.sigma_omega_sq * config_.cov_omega_threshold;
                bool position_converged = (P_(IDX_XC, IDX_XC) < config_.sigma_x_sq * config_.cov_position_threshold) &&
                                         (P_(IDX_YC, IDX_YC) < config_.sigma_y_sq * config_.cov_position_threshold);
                
                if (omega_converged && position_converged) {
                    ekf_converged_ = true;
                    std::cout << "[V4] EKF converged at frame " << ekf_update_count_ 
                              << ", P_omega=" << P_(IDX_OMEGA, IDX_OMEGA)
                              << " (threshold=" << config_.sigma_omega_sq * config_.cov_omega_threshold << ")"
                              << ", P_x=" << P_(IDX_XC, IDX_XC)
                              << " (threshold=" << config_.sigma_x_sq * config_.cov_position_threshold << ")"
                              << ", P_y=" << P_(IDX_YC, IDX_YC)
                              << std::endl;
                } else if (ekf_update_count_ % 30 == 0) {
                    // 每30帧输出一次收敛进度
                    std::cout << "[V4] EKF converging... frame=" << ekf_update_count_
                              << ", omega: " << (omega_converged ? "OK" : "WAIT")
                              << ", position: " << (position_converged ? "OK" : "WAIT")
                              << std::endl;
                }
            }
            break;
    }
    
    // ========== 静止确认检测 ==========
    // 如果初始方向判断为静止（STATIC），持续检测是否真的完全静止
    if (rotation_direction_ == RotationDirection::STATIC && !static_confirmed_ && static_start_time_ > 0.0) {
        // 检测 x 坐标是否有显著变化
        double x_diff = std::abs(obs.position.x() - last_static_check_x_);
        
        if (x_diff > STATIC_MOVE_THRESHOLD) {
            // 检测到显著运动，取消静止判定，可能需要重新判断方向
            std::cout << "[V4] Static check: motion detected (x_diff=" << x_diff 
                      << "), resetting static timer" << std::endl;
            static_start_time_ = obs.timestamp;
            static_check_frames_ = 0;
        } else {
            // 持续静止，检查是否超过阈值
            static_check_frames_++;
            double elapsed = obs.timestamp - static_start_time_;
            
            if (elapsed >= STATIC_CONFIRM_SEC) {
                static_confirmed_ = true;
                std::cout << "[V4] STATIC CONFIRMED! elapsed=" << elapsed 
                          << "s, frames=" << static_check_frames_ << std::endl;
            }
        }
        
        last_static_check_x_ = obs.position.x();
    }
    
    return true;
}

// ============================================================================
// 旋转方向初始化
// ============================================================================

void OutpostPredictorV4::updateDirectionInit(const ObservationV4& obs)
{
    std::cout << "directTest:"<< config_.armor_arrangement_reversed << std::endl;
    // 追踪cx变化
    cx_history_.push_back(obs.center_pixel_x);
    if (cx_history_.size() > static_cast<size_t>(config_.direction_init_frames)) {
        cx_history_.pop_front();
    }
    
    init_frame_count_++;
    
    if (init_frame_count_ >= config_.direction_init_frames) {
        // 分析cx的变化趋势
        double total_change = 0.0;
        double max_jump = 0.0;
        bool has_right_jump = false;
        bool has_left_jump = false;
        double avg_change = 0.0;
        int above_zero_count = 0;
        int below_zero_count = 0;
        if (cx_history_.size() >= 2) {
            for (size_t i = 1; i < cx_history_.size(); i++) {
                double diff = cx_history_[i] - cx_history_[i-1];
                std::cout<<diff<<std::endl;
                total_change += diff;
                if (diff > 0) above_zero_count++;
                else if (diff < 0) below_zero_count++;
                double curAvg = total_change / i;
                // 检测大幅度突变
                if (std::abs(diff) > curAvg * 10.0) {
                    if (diff > 0) has_right_jump = true;
                    else has_left_jump = true;
                    max_jump = std::max(max_jump, std::abs(diff));
                }
            }
            
            // 判断旋转方向
            // 顺时针：cx稳定向左移动（avg < 0），可能伴随向右突变（装甲板切换）
            // 逆时针：cx稳定向右移动（avg > 0），可能伴随向左突变（装甲板切换）
            avg_change = total_change / (cx_history_.size() - 1);
            
            if (std::abs(avg_change) < config_.cx_change_threshold / config_.direction_init_frames) {
                // 变化太小，认为静止
                rotation_direction_ = RotationDirection::STATIC;
            } else if (above_zero_count < below_zero_count) {
                // 平均向左移动 -> 顺时针
                // 注意：即使有向右突变（has_right_jump），只要平均趋势向左，仍判为顺时针
                rotation_direction_ = RotationDirection::CLOCKWISE;
            } else if (above_zero_count > below_zero_count) {
                // 平均向右移动 -> 逆时针
                // 注意：即使有向左突变（has_left_jump），只要平均趋势向右，仍判为逆时针
                rotation_direction_ = RotationDirection::COUNTER_CLOCKWISE;
            } else {
                // avg_change == 0（几乎不可能），但突变信息可用
                if (has_right_jump && !has_left_jump) {
                    rotation_direction_ = RotationDirection::CLOCKWISE;
                } else if (has_left_jump && !has_right_jump) {
                    rotation_direction_ = RotationDirection::COUNTER_CLOCKWISE;
                } else {
                    rotation_direction_ = RotationDirection::STATIC;
                }
            }
        }
        
        // 进入 XY 线性方程初始化阶段
        init_phase_ = InitPhase::XY_LINE_INIT;
        init_frame_count_ = 0;
        x_init_values_.clear();
        
        // 如果判定为静止，开始静止确认计时
        if (rotation_direction_ == RotationDirection::STATIC) {
            static_start_time_ = obs.timestamp;
            last_static_check_x_ = obs.position.x();
            static_check_frames_ = 0;
            std::cout << "[V4] Static detected, starting confirmation timer" << std::endl;
        }
        
        std::cout << "[V4] Direction initialized: " 
                  << static_cast<int>(rotation_direction_) 
                  << " (avg_change=" << avg_change
                  << ", has_right_jump=" << has_right_jump
                  << ", has_left_jump=" << has_left_jump
                  << ", max_jump=" << max_jump << ")" << std::endl;
    }
}

// ============================================================================
// X 范围初始化（用于检测装甲板切换）
// ============================================================================

void OutpostPredictorV4::updateXRangeInit(const ObservationV4& obs)
{
    // 收集 X 值
    x_init_values_.push_back(obs.position.x());
    
    // 同时更新 Z 滤波
    updateFilteredZ(obs.position.z());
    
    init_frame_count_++;
    
    // 达到初始化帧数后，计算 X 范围
    if (init_frame_count_ >= config_.xy_line_init_frames) {
        // 计算 x_min 和 x_max
        x_min_ = std::numeric_limits<double>::max();
        x_max_ = std::numeric_limits<double>::lowest();
        
        for (double x : x_init_values_) {
            x_min_ = std::min(x_min_, x);
            x_max_ = std::max(x_max_, x);
        }
        
        double x_range = x_max_ - x_min_;
        
        // 检查 X 范围是否足够（至少 0.1m）
        if (x_range >= 0.3) {
            x_range_initialized_ = true;
            last_x_ = x_init_values_.back();
            last_x_valid_ = true;
            
            std::cout << "[V4] X range initialized: x_min=" << x_min_ 
                      << ", x_max=" << x_max_
                      << ", x_range=" << x_range << std::endl;
            
            // 进入高度状态机初始化
            init_phase_ = InitPhase::HEIGHT_INIT;
            init_frame_count_ = 0;
        } else {
            // X 范围太小，重新收集
            std::cout << "[V4] X range too small (" << x_range << "m), retrying..." << std::endl;
            x_init_values_.clear();
            init_frame_count_ = 0;
        }
    }
}

// ============================================================================
// X 范围更新和装甲板切换检测（使用 X 坐标变化）
// ============================================================================

void OutpostPredictorV4::updateXRange(const ObservationV4& obs)
{
    if (!x_range_initialized_) {
        return;
    }
    
    double x = obs.position.x();
    
    // 动态扩展 X 范围
    if (x < x_min_) {
        x_min_ = x;
    }
    if (x > x_max_) {
        x_max_ = x;
    }
}

void OutpostPredictorV4::updateFilteredZ(double z)
{
    z_filter_window_.push_back(z);
    if (z_filter_window_.size() > static_cast<size_t>(config_.z_filter_window)) {
        z_filter_window_.pop_front();
    }
    
    // 计算滑动平均
    double sum = 0.0;
    for (double val : z_filter_window_) {
        sum += val;
    }
    current_filtered_z_ = sum / static_cast<double>(z_filter_window_.size());
}

double OutpostPredictorV4::detectArmorSwitchByX(const ObservationV4& obs)
{
    double current_x = obs.position.x();
    
    if (!x_range_initialized_ || !last_x_valid_) {
        // 还没初始化，更新 last_x 并返回
        last_x_ = current_x;
        last_x_valid_ = true;
        return 0.0;
    }
    
    double x_range = x_max_ - x_min_;
    
    if (x_range < 0.05) {
        last_x_ = current_x;
        return 0.0;
    }
    
    // 计算 X 坐标跳变
    double x_jump = std::abs(current_x - last_x_);
    double jump_ratio = x_jump / x_range;
    
    // 如果有待确认的 X 剧变
    if (pending_x_jump_) {
        // 检查是否跳回去了（使用较小的阈值）
        if (jump_ratio > config_.xy_jump_ratio * 0.8) {
            // 跳回去了，取消
            pending_x_jump_ = false;
            pending_x_stable_frames_ = 0;
            last_x_ = current_x;
            return 0.0;
        }
        
        // 稳定帧计数
        pending_x_stable_frames_++;
        
        // 达到确认帧数
        if (pending_x_stable_frames_ >= config_.xy_jump_confirm_frames) {
            // 确认装甲板切换！
            // 计算 z 跳变方向（使用滤波后的值）
            double z_diff = current_filtered_z_ - pre_jump_filtered_z_;
            
            pending_x_jump_ = false;
            pending_x_stable_frames_ = 0;
            last_x_ = current_x;
            
            std::cout << "[V4] X jump confirmed! z_diff=" << z_diff 
                      << ", pre_z=" << pre_jump_filtered_z_
                      << ", cur_z=" << current_filtered_z_ 
                      << ", x_jump=" << x_jump
                      << ", x_range=" << x_range << std::endl;
            
            return z_diff;
        }
        
        last_x_ = current_x;
        return 0.0;
    }
    
    // 检测新的 X 剧变
    if (jump_ratio >= config_.xy_jump_ratio) {
        pending_x_jump_ = true;
        pending_x_stable_frames_ = 0;
        pre_jump_filtered_z_ = current_filtered_z_;  // 记录跳变前的滤波 z
        
        std::cout << "[V4] X jump detected! ratio=" << jump_ratio 
                  << ", x_jump=" << x_jump 
                  << ", x_range=" << x_range 
                  << ", last_x=" << last_x_
                  << ", current_x=" << current_x << std::endl;
    }
    
    last_x_ = current_x;
    return 0.0;
}

// ============================================================================
// 高度状态机初始化（新机制：完整观测一圈，记录三组高度）
// ============================================================================

void OutpostPredictorV4::updateHeightInit(const ObservationV4& obs)
{
    // 更新 Z 滤波
    updateFilteredZ(obs.position.z());
    
    // 更新 X 范围
    updateXRange(obs);
    
    // 初始化 last_valid_z_
    if (last_valid_z_ == 0.0) {
        last_valid_z_ = current_filtered_z_;
        current_height_group_.push_back(obs.position.z());
        return;
    }
    
    // 使用 X 坐标检测装甲板切换
    double confirmed_jump = detectArmorSwitchByX(obs);
    
    // confirmed_jump != 0 表示检测到装甲板切换（通过 X 坐标剧变确认）
    // confirmed_jump 的值是滤波后的 z 差值，用于判断跳变方向
    if (std::abs(confirmed_jump) > 0.001) {  // 有跳变（非零即可）
        // 发生确认的高度突变
        height_init_jump_count_++;
        
        // 保存当前高度组的平均值（在突变前的数据）
        if (!current_height_group_.empty()) {
            double avg_z = 0.0;
            for (double z : current_height_group_) {
                avg_z += z;
            }
            avg_z /= current_height_group_.size();
            
            // 根据突变次数保存到对应位置（临时存储，最后排序）
            if (height_init_jump_count_ <= 3) {
                height_calibration_[height_init_jump_count_ - 1] = avg_z;
            }
            
            std::cout << "[V4] Height init: jump #" << height_init_jump_count_ 
                      << ", group avg_z=" << avg_z 
                      << ", jump=" << confirmed_jump << std::endl;
        }
        
        // 清空当前组，开始记录新组
        current_height_group_.clear();
        
        // 完成三次突变后，进行高度标定
        if (height_init_jump_count_ >= 3) {
            // 记录第三组之后的当前帧
            current_height_group_.push_back(obs.position.z());
            
            // 排序三组高度，确定 LOW/MIDDLE/HIGH
            std::array<std::pair<double, int>, 3> sorted_heights;
            for (int i = 0; i < 3; i++) {
                sorted_heights[i] = {height_calibration_[i], i};
            }
            std::sort(sorted_heights.begin(), sorted_heights.end());
            
            // 重新映射：最小值为LOW，中间为MIDDLE，最大为HIGH
            double temp[3];
            temp[0] = sorted_heights[0].first;  // LOW
            temp[1] = sorted_heights[1].first;  // MIDDLE
            temp[2] = sorted_heights[2].first;  // HIGH
            
            height_calibration_[0] = temp[0];
            height_calibration_[1] = temp[1];
            height_calibration_[2] = temp[2];
            
            height_calibrated_ = true;
            height_initialized_ = true;
            
            // 根据当前z确定初始高度状态
            current_height_state_ = determineHeightFromZ(obs.position.z());
            predicted_next_height_ = nextHeightState(current_height_state_,
                rotation_direction_ == RotationDirection::CLOCKWISE);
            
            std::cout << "[V4] Height calibration complete: LOW=" << height_calibration_[0]
                      << ", MID=" << height_calibration_[1]
                      << ", HIGH=" << height_calibration_[2]
                      << ", current=" << static_cast<int>(current_height_state_) << std::endl;
            
            // 进入θ初始化阶段
            init_phase_ = InitPhase::THETA_INIT;
            phase_state_ = PhaseState::RISING;
            init_frame_count_ = 0;
            min_ar_in_falling_ = 1.0;
            falling_frames_ = 0;
            falling_start_time_ = 0.0;
            
            // 重置跳变检测状态，准备检测下一次跳变
            pending_jump_active_ = false;
            pending_jump_z_diff_ = 0.0;
            pending_jump_stable_frames_ = 0;
            pending_jump_last_z_ = 0.0;
            // last_valid_z_ 保持不变，作为下一次跳变检测的基准
        }
    } else {
        // 无突变，累积当前高度
        current_height_group_.push_back(obs.position.z());
    }
    // 注意：last_valid_z_ 由 detectConfirmedJump() 在确认跳动后自动更新
}

// ============================================================================
// θ观测初始化（第一圈）
// ============================================================================

void OutpostPredictorV4::updateThetaInit(const ObservationV4& obs)
{
    // 更新相位状态机
    updatePhaseState(obs);
    
    // 更新 Z 滤波和 X 范围
    updateFilteredZ(obs.position.z());
    updateXRange(obs);
    
    // 使用 X 坐标检测装甲板切换
    double confirmed_jump = detectArmorSwitchByX(obs);
    
    // 调试输出
    static int frame_count = 0;
    if (++frame_count % 30 == 0) {
        std::cout << "[V4] Theta init: frame=" << frame_count 
                  << ", z=" << obs.position.z()
                  << ", filtered_z=" << current_filtered_z_
                  << ", pending_x=" << pending_x_jump_
                  << ", stable_frames=" << pending_x_stable_frames_
                  << std::endl;
    }
    
    // EKF初始化条件：
    // - mode_a (正向顺时针 或 逆向逆时针)：z往下跳
    // - mode_b (正向逆时针 或 逆向顺时针)：z往上跳
    // 现在通过 XY 投影检测装甲板切换，confirmed_jump 是滤波后的 z 差值
    bool is_forward_cw = (rotation_direction_ == RotationDirection::CLOCKWISE && !config_.armor_arrangement_reversed);
    bool is_reversed_ccw = (rotation_direction_ == RotationDirection::COUNTER_CLOCKWISE && config_.armor_arrangement_reversed);
    bool mode_a = is_forward_cw || is_reversed_ccw;
    
    // 只要检测到装甲板切换（confirmed_jump != 0）且方向正确，就初始化 EKF
    bool should_init_ekf = false;
    if (std::abs(confirmed_jump) > 0.001) {  // 有切换
        if (mode_a) {
            // mode_a: z往下跳（confirmed_jump < 0）
            should_init_ekf = (confirmed_jump < 0);
        } else {
            // mode_b: z往上跳（confirmed_jump > 0）
            should_init_ekf = (confirmed_jump > 0);
        }
    }
    
    if (should_init_ekf) {
        // 根据模式确定初始高度
        if (mode_a) {
            // 正向顺时针 或 逆向逆时针：往下跳到LOW
            current_height_state_ = HeightState::LOW;
        } else {
            // 正向逆时针 或 逆向顺时针：往上跳到HIGH
            current_height_state_ = HeightState::HIGH;
        }
        
        std::cout << "[V4] Theta init: valid large z_jump detected (" << confirmed_jump 
                  << "m, mode_" << (mode_a ? "a" : "b") 
                  << "), height=" << static_cast<int>(current_height_state_) << std::endl;
        
        predicted_next_height_ = nextHeightState(current_height_state_,
            rotation_direction_ == RotationDirection::CLOCKWISE);
        
        // 强制相位状态为上升段
        phase_state_ = PhaseState::RISING;
        plateau_entered_ = false;
        plateau_aspect_ratio_peaked_ = false;
        min_ar_in_falling_ = 1.0;
        falling_frames_ = 0;
        falling_start_time_ = 0.0;
        
        // 初始化EKF
        initializeEKF(obs);
        init_phase_ = InitPhase::EKF_RUNNING;
        theta_initialized_ = true;
        std::cout << "[V4] Theta initialized, starting EKF" << std::endl;
    } else if (std::abs(confirmed_jump) > 0.001) {
        // 检测到跳变但方向不对（不满足EKF初始化条件），只更新高度状态
        current_height_state_ = determineHeightFromZ(current_filtered_z_);
        predicted_next_height_ = nextHeightState(current_height_state_,
            rotation_direction_ == RotationDirection::CLOCKWISE);
        std::cout << "[V4] Theta init: jump detected (" << confirmed_jump 
                  << "m) but direction wrong for EKF init, height=" << static_cast<int>(current_height_state_) << std::endl;
    }
    
    // 更新长宽比最大值记录（用于EKF初始化）
    if (obs.aspect_ratio > max_aspect_ratio_recorded_) {
        max_aspect_ratio_recorded_ = obs.aspect_ratio;
        max_aspect_ratio_position_x_ = obs.position.x();
        max_aspect_ratio_position_y_ = obs.position.y();
    }
}

// ============================================================================
// EKF初始化
// ============================================================================

void OutpostPredictorV4::initializeEKF(const ObservationV4& obs)
{
    // 初始化状态向量
    x_ = Eigen::VectorXd::Zero(STATE_DIM);
    
    // 计算当前观测的 theta（用于初始化 ang）
    // EKF 初始化发生在装甲板切换时，此时 phase_state_ = RISING
    double init_theta = 0.0;
    if (obs.aspect_ratio > 0) {
        init_theta = computeTheta(obs.aspect_ratio, PhaseState::RISING);
    }
    
    // ang 初始化：
    // ang_local = fmod(ang, 120) - 60 应该等于 init_theta
    // 即 fmod(ang, 120) = init_theta + 60
    // 所以 ang = init_theta + 60（在第一个 sector [0, 120) 内）
    // 
    // 对于顺时针和逆时针：
    // - 顺时针：init_theta ≈ -60（装甲板刚从右侧进入），ang ≈ 0
    // - 逆时针：init_theta ≈ +60（装甲板刚从左侧进入），ang ≈ 120
    //   但 fmod(120, 120) = 0，所以需要用 ang = 119.99... 或调整初始 sector
    double init_ang = init_theta + 60.0;
    if (init_ang >= 120.0) {
        init_ang -= 0.01;  // 避免刚好在边界
    }
    if (init_ang < 0.0) {
        init_ang += 120.0;
    }
    
    x_(IDX_ANG) = init_ang;
    
    // omega 初始化
    if (rotation_direction_ == RotationDirection::CLOCKWISE) {
        x_(IDX_OMEGA) = config_.standard_omega;  // 正值
    } else if (rotation_direction_ == RotationDirection::COUNTER_CLOCKWISE) {
        x_(IDX_OMEGA) = -config_.standard_omega;  // 负值
    } else {
        x_(IDX_OMEGA) = 0.0;
    }
    
    // 旋转中心位置：使用长宽比最大时记录的装甲板位置反推
    // theta=0时（正对），装甲板在中心朝向机架的方向偏移R
    // 装甲板位置 = 中心 + R * 单位向量（从中心指向机架）
    // 设中心为(x_c, y_c)，机架在原点，则从中心指向机架的单位向量为 (-x_c, -y_c)/dist
    // 装甲板位置: (x_armor, y_armor) = (x_c, y_c) + R * (-x_c, -y_c)/dist
    //           x_armor = x_c * (1 - R/dist)
    //           y_armor = y_c * (1 - R/dist)
    // 反推: x_c = x_armor / (1 - R/dist)，但dist依赖于x_c...
    // 
    // 近似方法：假设dist_armor ≈ dist_center（前哨站距离远时成立）
    // dist_armor ≈ sqrt(x_armor^2 + y_armor^2)
    // x_c ≈ x_armor / (1 - R/dist_armor) = x_armor * dist_armor / (dist_armor - R)
    // y_c ≈ y_armor / (1 - R/dist_armor) = y_armor * dist_armor / (dist_armor - R)
    double dist_armor = std::sqrt(max_aspect_ratio_position_x_ * max_aspect_ratio_position_x_ 
                                + max_aspect_ratio_position_y_ * max_aspect_ratio_position_y_);
    double scale_factor = dist_armor / (dist_armor - config_.radius);
    if (scale_factor < 1.0 || scale_factor > 2.0) {
        scale_factor = 1.0;  // 保护：距离太近时用原始值
    }
    x_(IDX_XC) = max_aspect_ratio_position_x_ * scale_factor;
    x_(IDX_YC) = max_aspect_ratio_position_y_ * scale_factor;
    
    // z_c: 根据当前高度状态调整
    if (current_height_state_ == HeightState::LOW) {
        x_(IDX_ZC) = obs.position.z() + config_.height_diff;  // 低+10cm=中
    } else if (current_height_state_ == HeightState::HIGH) {
        x_(IDX_ZC) = obs.position.z() - config_.height_diff;  // 高-10cm=中
    } else {
        x_(IDX_ZC) = obs.position.z();
    }
    
    std::cout << "[V4] z_c init: obs.z=" << obs.position.z() 
              << ", height_state=" << static_cast<int>(current_height_state_)
              << ", height_diff=" << config_.height_diff
              << ", z_c=" << x_(IDX_ZC) << std::endl;
    
    // 初始化协方差矩阵P
    P_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    P_(IDX_ANG, IDX_ANG) = config_.sigma_theta_sq;
    P_(IDX_OMEGA, IDX_OMEGA) = config_.sigma_omega_sq;
    P_(IDX_XC, IDX_XC) = config_.sigma_x_sq;
    P_(IDX_YC, IDX_YC) = config_.sigma_y_sq;
    P_(IDX_ZC, IDX_ZC) = config_.sigma_z_sq;
    
    // 高度状态
    height_k_ = current_height_state_;
    
    ekf_initialized_ = true;
    ekf_converged_ = false;
    ekf_update_count_ = 0;
    
    std::cout << "[V4] EKF initialized: ang=" << x_(IDX_ANG) 
              << " (init_theta=" << init_theta << ")"
              << ", omega=" << x_(IDX_OMEGA)
              << ", center=(" << x_(IDX_XC) << ", " << x_(IDX_YC) << ", " << x_(IDX_ZC) << ")"
              << ", direction=" << static_cast<int>(rotation_direction_)
              << std::endl;
}

// ============================================================================
// θ计算方法
// ============================================================================

double OutpostPredictorV4::computeThetaAbs(double aspect_ratio) const
{
    // 定义有效长宽比范围
    constexpr double MIN_ASPECT_RATIO = 1.26;  // 对应约60度
    constexpr double MAX_ASPECT_RATIO = 2.58;  // 对应0度（正对）
    
    if (aspect_ratio < 0) {
        return 60.0;  // 无效值返回最大角度
    }
    
    // 约束aspect_ratio到有效区间
    double clamped_ar = std::clamp(aspect_ratio, MIN_ASPECT_RATIO, MAX_ASPECT_RATIO);
    
    double ratio = clamped_ar / config_.max_aspect_ratio;
    double theta_rad = std::acos(ratio);
    return theta_rad * 180.0 / M_PI;  // 转为度
}

double OutpostPredictorV4::computeTheta(double aspect_ratio, PhaseState phase) const
{
    double theta_abs = computeThetaAbs(aspect_ratio);
    
    // 根据旋转方向和相位确定符号
    // 
    // EKF模型约定：
    // - 顺时针：omega > 0，ang 递增，装甲板从右向左移动
    // - 逆时针：omega < 0，ang 递减，装甲板从左向右移动
    //
    // theta 的物理含义：相对于"正面朝向相机"的相位角
    // - theta < 0：装甲板在正面的左侧（还没转到正面）
    // - theta > 0：装甲板在正面的右侧（已经转过正面）
    //
    // 顺时针转动：
    //   上升段（装甲板逐渐转向正对）：装甲板从右侧往中间移，theta 从 -60 → 0
    //   下降段（装甲板逐渐转离正对）：装甲板从中间往左侧移，theta 从 0 → +60
    //
    // 逆时针转动：
    //   上升段（装甲板逐渐转向正对）：装甲板从左侧往中间移，theta 从 +60 → 0
    //   下降段（装甲板逐渐转离正对）：装甲板从中间往右侧移，theta 从 0 → -60
    //
    // 总结：
    // - 顺时针：上升段为负，下降段为正
    // - 逆时针：上升段为正，下降段为负（注意：上升段时长宽比在增加，但theta在减小！）
    
    bool is_clockwise = (rotation_direction_ == RotationDirection::CLOCKWISE);
    
    if (phase == PhaseState::RISING) {
        // 顺时针上升段：theta = -theta_abs（从-60往0变化）
        // 逆时针上升段：theta = +theta_abs（从+60往0变化）
        return is_clockwise ? -theta_abs : theta_abs;
    } else if (phase == PhaseState::FALLING) {
        // 顺时针下降段：theta = +theta_abs（从0往+60变化）
        // 逆时针下降段：theta = -theta_abs（从0往-60变化）
        return is_clockwise ? theta_abs : -theta_abs;
    } else {
        // 平台区：返回0（或使用插值）
        return 0.0;
    }
}

double OutpostPredictorV4::computePlateauTheta(double cx) const
{
    if (!plateau_calib_.calibrated) {
        return 0.0;
    }
    
    // 计算平台区对应的相位角范围
    double theta_half = std::acos(config_.plateau_threshold / config_.max_aspect_ratio) * 180.0 / M_PI;
    double sin_theta_half = std::sin(theta_half * M_PI / 180.0);
    
    // cx 与 sin(theta) 是线性关系（基于投影几何）
    // 
    // 顺时针：装甲板从右往左移动
    //   当 theta = -theta_half 时, cx = cx_enter (右侧)
    //   当 theta = +theta_half 时, cx = cx_exit (左侧)
    //   cx_enter > cx_exit
    //
    // 逆时针：装甲板从左往右移动
    //   当 theta = +theta_half 时, cx = cx_enter (左侧)
    //   当 theta = -theta_half 时, cx = cx_exit (右侧)
    //   cx_enter < cx_exit
    // 
    // 线性插值 sin(theta)，然后用 asin 得到 theta
    double ratio = (cx - plateau_calib_.cx_enter) / (plateau_calib_.cx_exit - plateau_calib_.cx_enter);
    ratio = std::clamp(ratio, 0.0, 1.0);
    
    double theta;
    if (rotation_direction_ == RotationDirection::CLOCKWISE) {
        // 顺时针：ratio 从 0→1 对应 theta 从 -theta_half → +theta_half
        double sin_theta = sin_theta_half * (2.0 * ratio - 1.0);
        sin_theta = std::clamp(sin_theta, -1.0, 1.0);
        theta = std::asin(sin_theta) * 180.0 / M_PI;
    } else {
        // 逆时针：ratio 从 0→1 对应 theta 从 +theta_half → -theta_half
        double sin_theta = sin_theta_half * (1.0 - 2.0 * ratio);
        sin_theta = std::clamp(sin_theta, -1.0, 1.0);
        theta = std::asin(sin_theta) * 180.0 / M_PI;
    }
    
    return theta;
}

// ============================================================================
// 相位状态机
// ============================================================================

void OutpostPredictorV4::updatePhaseState(const ObservationV4& obs)
{
    if (obs.aspect_ratio < 0) {
        return;  // 无效长宽比
    }
    
    double ar = obs.aspect_ratio;
    
    switch (phase_state_) {
        case PhaseState::RISING:
            // 上升段：检测是否进入平台区
            if (ar >= config_.plateau_threshold) {
                phase_state_ = PhaseState::PLATEAU;
                plateau_entered_ = true;
                plateau_aspect_ratio_peaked_ = false;
                plateau_calib_.cx_enter = obs.center_pixel_x;
                // std::cout << "[V4] Entered plateau at cx=" << obs.center_pixel_x << std::endl;
            }
            break;
            
        case PhaseState::PLATEAU:
            // 平台区
            if (ar >= config_.plateau_confirm_ratio) {
                plateau_aspect_ratio_peaked_ = true;
            }
            
            // 检测是否退出平台区
            if (ar < config_.plateau_threshold) {
                if (plateau_aspect_ratio_peaked_) {
                    // 正常退出
                    phase_state_ = PhaseState::FALLING;
                    falling_start_time_ = obs.timestamp;  // 记录进入FALLING的时间
                    plateau_calib_.cx_exit = obs.center_pixel_x;
                    plateau_calib_.calibrated = true;
                    // std::cout << "[V4] Exited plateau at cx=" << obs.center_pixel_x << std::endl;
                }
                // 如果没有peaked过，可能是误入，不算退出
            }
            
            // 强制退出条件
            if (ar < config_.plateau_threshold * config_.plateau_exit_ratio) {
                phase_state_ = PhaseState::FALLING;
                falling_start_time_ = obs.timestamp;  // 记录进入FALLING的时间
                if (!plateau_calib_.calibrated) {
                    plateau_calib_.cx_exit = obs.center_pixel_x;
                }
                std::cout << "[V4] Force exited plateau" << std::endl;
            }
            break;
            
        case PhaseState::FALLING:
            // 下降段：等待z突变（装甲板切换）重置为上升段
            // 超时自保机制：FALLING状态不可能持续超过0.3秒
            if (falling_start_time_ > 0.0 && (obs.timestamp - falling_start_time_) > FALLING_TIMEOUT_SEC) {
                std::cout << "[V4] FALLING timeout! Duration=" << (obs.timestamp - falling_start_time_)
                          << "s > " << FALLING_TIMEOUT_SEC << "s, forcing to RISING" << std::endl;
                phase_state_ = PhaseState::RISING;
                falling_start_time_ = 0.0;
                min_ar_in_falling_ = 1.0;
                falling_frames_ = 0;
            }
            break;
    }
}

// ============================================================================
// 高度状态机（新机制）
// ============================================================================

void OutpostPredictorV4::updateHeightStateMachine(const ObservationV4& obs)
{
    if (!height_initialized_) {
        return;
    }
    
    // 更新 Z 滤波和 X 范围
    updateFilteredZ(obs.position.z());
    updateXRange(obs);
    
    // 使用 X 坐标检测装甲板切换
    double confirmed_jump = detectArmorSwitchByX(obs);
    
    // 规则五：如果观测连续缺失超过10帧，同步EKF高度状态
    if (height_lost_count_ > 10) {
        if (!height_sync_with_ekf_) {
            height_sync_with_ekf_ = true;
            std::cout << "[V4] Height syncing with EKF (lost > 10 frames)" << std::endl;
        }
        current_height_state_ = height_k_;
        return;
    }
    
    // 恢复观测后恢复原机制
    if (height_sync_with_ekf_ && obs.valid) {
        height_sync_with_ekf_ = false;
        height_lost_count_ = 0;
        std::cout << "[V4] Height observation restored" << std::endl;
    }
    
    // 检测到确认的跳动（通过 XY 投影检测，confirmed_jump 是 z 差值）
    if (std::abs(confirmed_jump) > 0.001) {  // 有切换
        HeightState old_state = current_height_state_;
        HeightState next_state = nextHeightState(old_state, rotation_direction_ == RotationDirection::CLOCKWISE);
        HeightState z_based_state = determineHeightFromZ(current_filtered_z_);
        
        // 判断旋转模式
        bool is_forward_cw = (rotation_direction_ == RotationDirection::CLOCKWISE && !config_.armor_arrangement_reversed);
        bool is_reversed_ccw = (rotation_direction_ == RotationDirection::COUNTER_CLOCKWISE && config_.armor_arrangement_reversed);
        bool mode_a = is_forward_cw || is_reversed_ccw;  // 正向顺时针 或 逆向逆时针
        
        if (mode_a) {
            // 规则一：正向排布顺时针 或 逆向排布逆时针
            if (confirmed_jump > 0) {
                // 高度往上突变
                if (next_state == z_based_state) {
                    current_height_state_ = next_state;
                } else {
                    current_height_state_ = next_state;  // 以状态机为准
                    std::cout << "[V4] Height mismatch (up): SM=" << static_cast<int>(next_state)
                              << ", Z-based=" << static_cast<int>(z_based_state) 
                              << ", using SM" << std::endl;
                }
            } else {
                // 高度往下突变：强制设为LOW
                current_height_state_ = HeightState::LOW;
                std::cout << "[V4] Height down jump in mode_a: force LOW" << std::endl;
            }
        } else {
            // 规则二：逆向排布顺时针 或 正向排布逆时针
            if (confirmed_jump < 0) {
                // 高度往下突变
                if (next_state == z_based_state) {
                    current_height_state_ = next_state;
                } else {
                    current_height_state_ = next_state;  // 以状态机为准
                    std::cout << "[V4] Height mismatch (down): SM=" << static_cast<int>(next_state)
                              << ", Z-based=" << static_cast<int>(z_based_state) 
                              << ", using SM" << std::endl;
                }
            } else {
                // 高度往上突变：强制设为HIGH
                current_height_state_ = HeightState::HIGH;
                std::cout << "[V4] Height up jump in mode_b: force HIGH" << std::endl;
            }
        }
        
        predicted_next_height_ = nextHeightState(current_height_state_,
            rotation_direction_ == RotationDirection::CLOCKWISE);
        
        // 重置相位状态
        phase_state_ = PhaseState::RISING;
        plateau_entered_ = false;
        plateau_aspect_ratio_peaked_ = false;
        min_ar_in_falling_ = 1.0;
        falling_frames_ = 0;
        falling_start_time_ = 0.0;
        
        height_lost_count_ = 0;
        height_verify_count_++;
        
        std::cout << "[V4] Height state: " << static_cast<int>(old_state) 
                  << " -> " << static_cast<int>(current_height_state_)
                  << " (jump=" << confirmed_jump << ")" << std::endl;
    }
    
    // 规则四(2)：安全区域内，用观测高度覆盖EKF高度
    // 不依赖 phase_state_，直接检查 ang_local 是否在安全区域内
    if (ekf_initialized_) {
        // 计算局部角
        double ang_local = std::fmod(x_(IDX_ANG), 120.0);
        if (ang_local < 0) ang_local += 120.0;
        ang_local -= 60.0;  // 转到 [-60, 60] 范围
        
        // 安全区域：局部角在 (-45, 45) 内
        // 此时装甲板接近正对，观测高度可信度高
        if (std::abs(ang_local) < 45.0) {
            if (height_k_ != current_height_state_) {
                std::cout << "[V4] Safe zone correction: EKF height " << static_cast<int>(height_k_)
                          << " -> " << static_cast<int>(current_height_state_)
                          << " (ang_local=" << ang_local << ", phase=" << static_cast<int>(phase_state_) << ")" << std::endl;
                height_k_ = current_height_state_;
            }
        }
    }
}

// ============================================================================
// 新增辅助函数
// ============================================================================

HeightState OutpostPredictorV4::determineHeightFromZ(double z) const
{
    if (!height_calibrated_) {
        return HeightState::MIDDLE;  // 未标定时返回默认值
    }
    
    // 计算与三个高度档位的距离
    double dist_low = std::abs(z - height_calibration_[0]);
    double dist_mid = std::abs(z - height_calibration_[1]);
    double dist_high = std::abs(z - height_calibration_[2]);
    
    if (dist_low <= dist_mid && dist_low <= dist_high) {
        return HeightState::LOW;
    } else if (dist_mid <= dist_low && dist_mid <= dist_high) {
        return HeightState::MIDDLE;
    } else {
        return HeightState::HIGH;
    }
}

double OutpostPredictorV4::detectConfirmedJump(double current_z)
{
    // 如果还没有上一帧的z值，初始化
    if (last_valid_z_ == 0.0) {
        last_valid_z_ = current_z;
        return 0.0;
    }
    
    // 计算与上一帧的差值（相邻帧比较）
    double z_diff_from_prev = current_z - last_valid_z_;
    if (z_diff_from_prev >= config_.single_z_jump) {
        std::cout << "[V4] z jump candidate: " << z_diff_from_prev << "m" << std::endl;
    }
    // 如果有待确认的跳动
    if (pending_jump_active_) {
        // 检查是否跳回去了（与前一帧比较，方向相反且幅度较大）
        if (z_diff_from_prev * pending_jump_z_diff_ < 0 && 
            std::abs(z_diff_from_prev) > config_.single_z_jump * 0.5) {
            // 方向相反且幅度较大，取消跳动
            pending_jump_active_ = false;
            pending_jump_z_diff_ = 0.0;
            pending_jump_stable_frames_ = 0;
            last_valid_z_ = current_z;  // 更新为当前帧
            return 0.0;
        }
        
        // 检查是否稳定（与前一帧的差值小于阈值的一半）
        if (std::abs(z_diff_from_prev) < config_.single_z_jump * 0.5) {
            pending_jump_stable_frames_++;
        }
        // 注意：即使不稳定也继续等待，只有反向跳回才取消
        
        // 达到确认帧数
        if (pending_jump_stable_frames_ >= JUMP_CONFIRM_FRAMES) {
            double confirmed = pending_jump_z_diff_;
            pending_jump_active_ = false;
            pending_jump_z_diff_ = 0.0;
            pending_jump_stable_frames_ = 0;
            last_valid_z_ = current_z;  // 更新为当前帧
            return confirmed;
        }
        
        last_valid_z_ = current_z;  // 每帧都更新！
        return 0.0;
    }
    
    // 没有待确认跳动，检测新跳变（与前一帧比较）
    if (std::abs(z_diff_from_prev) > config_.single_z_jump) {
        pending_jump_active_ = true;
        pending_jump_z_diff_ = z_diff_from_prev;
        pending_jump_stable_frames_ = 0;  // 跳变帧本身不算稳定帧，从下一帧开始计数
    }
    
    last_valid_z_ = current_z;  // 每帧都更新！
    return 0.0;
}

void OutpostPredictorV4::updateEKFHeightState()
{
    if (!ekf_initialized_) {
        return;
    }
    
    double current_ang = x_(IDX_ANG);
    
    // 检测是否跨越120度的整数倍临界点
    // 计算上一次和这一次所在的 sector
    auto getSector = [](double ang) -> int {
        double normalized = std::fmod(ang, 360.0);
        if (normalized < 0) normalized += 360.0;
        return static_cast<int>(normalized / 120.0);
    };
    
    int last_sector = getSector(last_ekf_ang_);
    int current_sector = getSector(current_ang);
    
    // 如果 sector 发生变化，说明跨越了临界点
    if (last_sector != current_sector) {
        // 根据 omega 方向决定是正向还是反向跨越
        if (x_(IDX_OMEGA) > 0) {
            // 正向（顺时针），ang 递增
            height_k_ = nextHeightState(height_k_, true);
        } else if (x_(IDX_OMEGA) < 0) {
            // 反向（逆时针），ang 递减
            height_k_ = nextHeightState(height_k_, false);
        }
        
        std::cout << "[V4] EKF height crossed boundary: sector " << last_sector 
                  << " -> " << current_sector 
                  << ", height_k=" << static_cast<int>(height_k_) << std::endl;
    }
    
    last_ekf_ang_ = current_ang;
}

// computeHeightFromAng 现在仅用于调试和备用，不再用于主要逻辑
HeightState OutpostPredictorV4::computeHeightFromAng(double ang) const
{
    // 简化版本：仅用于调试
    double local_ang = normalizeAngle360(ang);
    int sector = static_cast<int>(local_ang / 120.0) % 3;
    
    if (!config_.armor_arrangement_reversed) {
        if (rotation_direction_ == RotationDirection::CLOCKWISE) {
            return static_cast<HeightState>(sector);
        } else {
            int adjusted = (sector + 2) % 3;
            return static_cast<HeightState>(adjusted);
        }
    } else {
        if (rotation_direction_ == RotationDirection::CLOCKWISE) {
            return static_cast<HeightState>((2 - sector + 3) % 3);
        } else {
            return static_cast<HeightState>((1 - sector + 3) % 3);
        }
    }
}

HeightState OutpostPredictorV4::nextHeightState(HeightState current, bool positive_direction) const
{
    int val = static_cast<int>(current);
    if ((positive_direction && !config_.armor_arrangement_reversed) || (config_.armor_arrangement_reversed && !positive_direction)) {
        // 顺时针：Low->Middle->High->Low
        return static_cast<HeightState>((val + 1) % 3);
    } else {
        // 逆时针：Low->High->Middle->Low
        return static_cast<HeightState>((val + 2) % 3);
    }
}

HeightState OutpostPredictorV4::prevHeightState(HeightState current, bool positive_direction) const
{
    return nextHeightState(current, !positive_direction);
}

// ============================================================================
// EKF预测
// ============================================================================

void OutpostPredictorV4::ekfPredict(double dt)
{
    if (!ekf_initialized_) {
        return;
    }
    
    // 保存旧的 ang 用于跨临界检测
    double old_ang = x_(IDX_ANG);
    
    // 状态外推
    double ang_pred = x_(IDX_ANG) + x_(IDX_OMEGA) * dt * 180.0 / M_PI;  // omega是rad/s，ang是度
    double omega_pred = x_(IDX_OMEGA);
    double xc_pred = x_(IDX_XC);
    double yc_pred = x_(IDX_YC);
    double zc_pred = x_(IDX_ZC);
    
    // 更新状态
    x_(IDX_ANG) = ang_pred;
    x_(IDX_OMEGA) = omega_pred;
    x_(IDX_XC) = xc_pred;
    x_(IDX_YC) = yc_pred;
    x_(IDX_ZC) = zc_pred;
    
    // 规则四(1)：EKF高度状态基于ang跨临界检测
    last_ekf_ang_ = old_ang;
    updateEKFHeightState();
    
    // 协方差外推 P = F*P*F' + Q
    Eigen::MatrixXd F = computeF(dt);
    Eigen::MatrixXd Q = getQ();
    P_ = F * P_ * F.transpose() + Q;
}

Eigen::MatrixXd OutpostPredictorV4::computeF(double dt) const
{
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    F(IDX_ANG, IDX_OMEGA) = dt * 180.0 / M_PI;  // d(ang)/d(omega) = dt，单位转换
    return F;
}

Eigen::MatrixXd OutpostPredictorV4::getQ() const
{
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    Q(IDX_ANG, IDX_ANG) = config_.q_theta;
    Q(IDX_OMEGA, IDX_OMEGA) = config_.q_omega;
    Q(IDX_XC, IDX_XC) = config_.q_x;
    Q(IDX_YC, IDX_YC) = config_.q_y;
    Q(IDX_ZC, IDX_ZC) = config_.q_z;
    return Q;
}

// ============================================================================
// EKF更新
// ============================================================================

void OutpostPredictorV4::ekfUpdate(const ObservationV4& obs, double theta)
{
    if (!ekf_initialized_) {
        return;
    }
    
    // 计算预测的局部相位角
    // ang_local 表示相对于 sector 中心的偏移，范围 [-60, 60]
    double ang_pred = x_(IDX_ANG);
    double ang_local = std::fmod(ang_pred, 120.0);
    if (ang_local < 0) ang_local += 120.0;
    ang_local -= 60.0;  // 转到 [-60, 60] 范围
    
    // 判断使用哪个装甲板的预测
    HeightState height_for_pred = height_k_;
    double y_theta = theta - ang_local;
    
    // 角度残差归一化
    if (y_theta > 60.0) y_theta -= 120.0;
    if (y_theta < -60.0) y_theta += 120.0;
    
    // 如果残差太大，可能是装甲板不匹配
    if (std::abs(y_theta) > 60.0) {
        // 尝试相邻装甲板
        double alt_ang_local;
        if (x_(IDX_OMEGA) > 0 ) {
            alt_ang_local = std::fmod(ang_pred + 120.0, 120.0) - 60.0;
            height_for_pred = prevHeightState(height_k_, true);
        } else {
            alt_ang_local = std::fmod(ang_pred - 120.0, 120.0) - 60.0;
            height_for_pred = nextHeightState(height_k_, true);
        }
        
        y_theta = theta - alt_ang_local;
        if (y_theta > 60.0) y_theta -= 120.0;
        if (y_theta < -60.0) y_theta += 120.0;
        
        ang_local = alt_ang_local;
    }
    
    // 计算预测的装甲板位置
    Eigen::Vector3d pred_pos = computePredictedArmorPosition(ang_local + 60.0, height_for_pred);
    
    // 计算位置残差
    double y_x = obs.position.x() - pred_pos.x();
    double y_y = obs.position.y() - pred_pos.y();
    double y_z = obs.position.z() - pred_pos.z();
    
    // 残差向量
    Eigen::Vector4d y;
    y << y_theta, y_x, y_y, y_z;
    last_residual_ = y;
    
    // 观测雅可比矩阵H
    Eigen::MatrixXd H = computeH(ang_local + 60.0);
    
    // 观测噪声R
    Eigen::MatrixXd R = getR();
    
    // 卡尔曼增益 K = P*H' * (H*P*H' + R)^-1
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    
    // 状态更新
    Eigen::VectorXd dx = K * y;
    x_ += dx;
    
    // 协方差更新
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    P_ = (I - K * H) * P_;
    
    // 注意：height_k_ 由 updateHeightStateMachine() 和 updateEKFHeightState() 负责更新
    // 这里不再做 ang 跨越边界的检测
}

Eigen::Vector3d OutpostPredictorV4::computePredictedArmorPosition(double ang_local, HeightState height) const
{
    // ang_local: [0, 120] 范围，60度为正对（相对于机架-中心连线）
    // theta是相对于机架到前哨站连线的相位角
    // theta=0时装甲板朝向机架（在中心的朝向机架方向偏移R）
    
    // 计算从原点（机架）看向中心的方位角
    double azimuth = std::atan2(x_(IDX_XC), x_(IDX_YC));  // 注意：atan2(x,y)因为y是前方
    
    // 局部相位角（相对于机架-中心连线）
    double theta_rad = (ang_local - 60.0) * M_PI / 180.0;  // [-60, 60] deg -> rad
    
    // 装甲板在世界坐标系中相对于中心的角度
    // theta=0时，装甲板在中心朝向机架的方向
    // 朝向机架的方向 = azimuth + pi（从中心看向机架）
    // 实际角度 = (azimuth + pi) + theta_rad，但我们用减法表示偏移
    double world_ang = azimuth + M_PI + theta_rad;
    
    // 装甲板位置 = 中心 + R * 方向向量
    double x_pred = x_(IDX_XC) + config_.radius * std::sin(world_ang);
    double y_pred = x_(IDX_YC) + config_.radius * std::cos(world_ang);
    
    double z_offset = 0.0;
    if (height == HeightState::LOW) {
        z_offset = -config_.height_diff;
    } else if (height == HeightState::HIGH) {
        z_offset = config_.height_diff;
    }
    double z_pred = x_(IDX_ZC) + z_offset;
    
    return Eigen::Vector3d(x_pred, y_pred, z_pred);
}

Eigen::MatrixXd OutpostPredictorV4::computeH(double ang_local) const
{
    // 观测模型: [theta, x, y, z]
    // theta = ang_local - 60 (局部相位，相对于机架-中心连线)
    // 
    // 位置计算（考虑方位角修正）:
    // azimuth = atan2(x_c, y_c)
    // world_ang = azimuth + pi + theta_rad
    // x = x_c + R*sin(world_ang)
    // y = y_c + R*cos(world_ang)
    // z = z_c + offset
    
    double theta_rad = (ang_local - 60.0) * M_PI / 180.0;
    double azimuth = std::atan2(x_(IDX_XC), x_(IDX_YC));
    double world_ang = azimuth + M_PI + theta_rad;
    
    // 用于计算方位角对中心位置的偏导
    double dist_xy = std::sqrt(x_(IDX_XC) * x_(IDX_XC) + x_(IDX_YC) * x_(IDX_YC));
    if (dist_xy < 0.01) dist_xy = 0.01;  // 防止除零
    
    // d(azimuth)/d(x_c) = y_c / (x_c^2 + y_c^2)
    // d(azimuth)/d(y_c) = -x_c / (x_c^2 + y_c^2)
    double daz_dxc = x_(IDX_YC) / (dist_xy * dist_xy);
    double daz_dyc = -x_(IDX_XC) / (dist_xy * dist_xy);
    
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(MEAS_DIM, STATE_DIM);
    
    // d(theta)/d(ang) = 1
    H(0, IDX_ANG) = 1.0;
    
    // x = x_c + R*sin(world_ang), world_ang = azimuth + pi + theta_rad
    // d(x)/d(ang) = R*cos(world_ang) * d(theta_rad)/d(ang) = R*cos(world_ang) * pi/180
    // d(x)/d(x_c) = 1 + R*cos(world_ang) * d(azimuth)/d(x_c)
    // d(x)/d(y_c) = R*cos(world_ang) * d(azimuth)/d(y_c)
    H(1, IDX_ANG) = config_.radius * std::cos(world_ang) * M_PI / 180.0;
    H(1, IDX_XC) = 1.0 + config_.radius * std::cos(world_ang) * daz_dxc;
    H(1, IDX_YC) = config_.radius * std::cos(world_ang) * daz_dyc;
    
    // y = y_c + R*cos(world_ang)
    // d(y)/d(ang) = -R*sin(world_ang) * pi/180
    // d(y)/d(x_c) = -R*sin(world_ang) * d(azimuth)/d(x_c)
    // d(y)/d(y_c) = 1 - R*sin(world_ang) * d(azimuth)/d(y_c)
    H(2, IDX_ANG) = -config_.radius * std::sin(world_ang) * M_PI / 180.0;
    H(2, IDX_XC) = -config_.radius * std::sin(world_ang) * daz_dxc;
    H(2, IDX_YC) = 1.0 - config_.radius * std::sin(world_ang) * daz_dyc;
    
    // d(z)/d(ang) = 0, d(z)/d(z_c) = 1
    H(3, IDX_ZC) = 1.0;
    
    return H;
}

Eigen::MatrixXd OutpostPredictorV4::getR() const
{
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(MEAS_DIM, MEAS_DIM);
    R(0, 0) = config_.r_theta;
    R(1, 1) = config_.r_x;
    R(2, 2) = config_.r_y;
    R(3, 3) = config_.r_z;
    return R;
}

// ============================================================================
// 预瞄计算
// ============================================================================
HeightState OutpostPredictorV4::computeNextHeight(HeightState current_height, int direction) const{
    int val = static_cast<int>(current_height);
    // 需要考虑 armor_arrangement_reversed，与 nextHeightState 保持一致
    if ((direction > 0 && !config_.armor_arrangement_reversed) || (config_.armor_arrangement_reversed && direction < 0)) {
        // 正向分布顺时针 或 逆向分布逆时针：Low->Middle->High->Low
        return static_cast<HeightState>((val + 1) % 3);
    } else {
        // 正向分布逆时针 或 逆向分布顺时针：Low->High->Middle->Low
        return static_cast<HeightState>((val + 2) % 3);
    }
}
AimResultV4 OutpostPredictorV4::computeAim() const
{
    AimResultV4 result;
    result.valid = false;
    
    // ========== 静止确认：直接返回当前观测位置 ==========
    if (static_confirmed_) {
        // 装甲板完全静止，直接瞄准最近的有效观测位置
        if (!obs_history_.empty()) {
            const auto& last_obs = obs_history_.back();
            if (last_obs.valid) {
                result.aim_position = last_obs.position;
                result.aim_height_state = current_height_state_;
                result.fire_condition_1 = true;  // 静止目标可以直接开火
                result.bullet_flight_time = last_obs.position.norm() / config_.v_bullet;
                result.distance = last_obs.position.norm();
                result.t_gimbal_camera = last_t_gimbal_camera_;
                result.r_body_gimbal = last_r_body_gimbal_;
                result.valid = true;
                
                // 调试输出（每100帧一次）
                static int static_aim_count = 0;
                if (++static_aim_count % 100 == 0) {
                    std::cout << "[V4] STATIC AIM: pos=(" << result.aim_position.x() 
                              << ", " << result.aim_position.y() 
                              << ", " << result.aim_position.z() << ")" << std::endl;
                }
                
                return result;
            }
        }
        return result;  // 没有有效观测，返回无效结果
    }
    
    if (!ekf_initialized_) {
        return result;
    }
    
    // 1. 估计飞行时间
    // 近似距离为当前观测到的装甲板中心到相机的距离
    Eigen::Vector3d current_armor_pos = computePredictedArmorPosition(
        std::fmod(x_(IDX_ANG), 120.0), height_k_);
    double dist = current_armor_pos.norm();
    double t_fly = dist / config_.v_bullet;
    
    // 2. 估算子弹飞达时的全局相位（包含filter_delay）
    double total_delay = t_fly + config_.t_delay + config_.filter_delay;
    double aim_ang = x_(IDX_ANG) + x_(IDX_OMEGA) * total_delay * 180.0 / M_PI;
    
    // 3. 根据预测的aim_ang直接计算所在的装甲板高度
    // 不依赖height_k_，直接从aim_ang计算
    HeightState aim_height;
    if (!config_.armor_arrangement_reversed) {
        // 正向分布：保持原有逻辑
        if (x_(IDX_OMEGA) > 0) {
            // 正向分布顺时针
            aim_height = computeHeightFromAng(aim_ang);
        } else {
            // 正向分布逆时针
            aim_height = computeHeightFromAng(aim_ang + 120.0);
        }
    } else {
        // 逆向分布：映射已在computeHeightFromAng中正确处理
        if (x_(IDX_OMEGA) > 0) {
            // 逆向分布顺时针
            aim_height = computeHeightFromAng(aim_ang);
        } else {
            // 逆向分布逆时针
            aim_height = computeHeightFromAng(aim_ang + 120.0);
        }
        if(static_confirmed_){
            aim_height = current_height_state_;
        }
    }

    // 计算局部相位 [-60, 60]
    double aim_ang_local = std::fmod(aim_ang, 120.0);
    if (aim_ang_local < 0) aim_ang_local += 120.0;
    if (aim_ang_local >120) aim_ang_local -= 120.0;
    aim_ang_local -= 60.0;
    
    // 4. 检查是否超过瞄准角阈值
    // 关键：基于 omega 的符号（ang 的变化方向）判断
    // omega > 0：ang 递增，aim_ang_local > +threshold 表示装甲板已转过，需要切换
    //           aim_ang_local < -threshold 表示装甲板还没到，不应该切换
    // omega < 0：ang 递减，aim_ang_local < -threshold 表示装甲板已转过，需要切换
    //           aim_ang_local > +threshold 表示装甲板还没到，不应该切换
    // 注意：这里只与 omega 符号有关，与 armor_arrangement_reversed 无关
    bool fire_ok = true;
    bool need_switch = false;
    
    if (x_(IDX_OMEGA) > 0) {
        // omega > 0：ang 递增
        if (aim_ang_local > config_.aim_angle_threshold_deg) {
            // 装甲板已经转过可射击区域，切换到下一个
            need_switch = true;
        } else if (aim_ang_local < -config_.aim_angle_threshold_deg) {
            // 装甲板还没转到可射击区域，等待（不开火）
            fire_ok = false;
        }
    } else {
        // omega < 0：ang 递减
        if (aim_ang_local < -config_.aim_angle_threshold_deg) {
            // 装甲板已经转过可射击区域，切换到下一个
            need_switch = true;
        } else if (aim_ang_local > config_.aim_angle_threshold_deg) {
            // 装甲板还没转到可射击区域，等待（不开火）
            fire_ok = false;
        }
    }
    
    if (need_switch) {
        // 切换到下一个装甲板
        double next_aim_ang;
        if (x_(IDX_OMEGA) > 0) {
            // 顺时针：下一个装甲板在当前sector结束后
            next_aim_ang = aim_ang + (120.0 - (aim_ang_local + 60.0));
        } else {
            // 逆时针：下一个装甲板在当前sector开始前
            next_aim_ang = aim_ang - (aim_ang_local + 60.0);
        }
        
        // 重新计算下一个装甲板的高度和局部相位
        aim_height = computeNextHeight(aim_height, (x_(IDX_OMEGA) > 0) ? 1 : -1);
        aim_ang_local = std::fmod(next_aim_ang, 120.0);
        // 归一化到 [0, 120) 范围，与 armor_arrangement_reversed 无关
        if (aim_ang_local < 0) aim_ang_local += 120.0;
        if (aim_ang_local > 120) aim_ang_local -= 120.0;
        aim_ang_local -= 60.0;  // 转到 [-60, 60] 范围
        
        // 再次检查新装甲板是否在可射击范围
        if (std::abs(aim_ang_local) > config_.aim_angle_threshold_deg) {
            fire_ok = false;
        }
    }
    
    // 5. 计算预瞄位置
    result.aim_position = computePredictedArmorPosition(aim_ang_local + 60.0, aim_height);
    result.aim_height_state = aim_height;
    result.aim_ang_local = aim_ang_local;
    result.fire_condition_1 = fire_ok;
    result.bullet_flight_time = t_fly;
    result.distance = dist;
    result.t_gimbal_camera = last_t_gimbal_camera_;
    result.r_body_gimbal = last_r_body_gimbal_;
    result.imu_yaw = last_imu_yaw_;
    result.imu_pitch = last_imu_pitch_;
    result.valid = true;
    
    // 6. 计算更远预瞄点（total_delay + moving_delay）
    double further_total_delay = total_delay + config_.moving_delay;
    double further_aim_ang = x_(IDX_ANG) + x_(IDX_OMEGA) * further_total_delay * 180.0 / M_PI;
    
    // 计算更远预瞄点的高度状态
    HeightState further_aim_height;
    if (!config_.armor_arrangement_reversed) {
        if (x_(IDX_OMEGA) > 0) {
            further_aim_height = computeHeightFromAng(further_aim_ang);
        } else {
            further_aim_height = computeHeightFromAng(further_aim_ang + 120.0);
        }
    } else {
        if (x_(IDX_OMEGA) > 0) {
            further_aim_height = computeHeightFromAng(further_aim_ang);
        } else {
            further_aim_height = computeHeightFromAng(further_aim_ang + 120.0);
        }
    }
    
    // 计算更远预瞄点的局部相位角
    double further_ang_local = std::fmod(further_aim_ang, 120.0);
    if (further_ang_local < 0) further_ang_local += 120.0;
    if (further_ang_local > 120) further_ang_local -= 120.0;
    further_ang_local -= 60.0;
    
    // 检查更远预瞄点是否需要切换装甲板
    bool further_need_switch = false;
    if (x_(IDX_OMEGA) > 0) {
        if (further_ang_local > config_.aim_angle_threshold_deg) {
            further_need_switch = true;
        }
    } else {
        if (further_ang_local < -config_.aim_angle_threshold_deg) {
            further_need_switch = true;
        }
    }
    
    if (further_need_switch) {
        double further_next_aim_ang;
        if (x_(IDX_OMEGA) > 0) {
            further_next_aim_ang = further_aim_ang + (120.0 - (further_ang_local + 60.0));
        } else {
            further_next_aim_ang = further_aim_ang - (further_ang_local + 60.0);
        }
        
        further_aim_height = computeNextHeight(further_aim_height, (x_(IDX_OMEGA) > 0) ? 1 : -1);
        further_ang_local = std::fmod(further_next_aim_ang, 120.0);
        if (further_ang_local < 0) further_ang_local += 120.0;
        if (further_ang_local > 120) further_ang_local -= 120.0;
        further_ang_local -= 60.0;
    }
    
    result.further_aim_position = computePredictedArmorPosition(further_ang_local + 60.0, further_aim_height);
    result.further_aim_ang = further_ang_local;
    result.further_aim_height_state = further_aim_height;
    
    return result;
}

// ============================================================================
// 状态查询
// ============================================================================

EKFStateV4 OutpostPredictorV4::getEKFState() const
{
    EKFStateV4 state;
    state.ang = x_(IDX_ANG);
    state.omega = x_(IDX_OMEGA);
    state.x_c = x_(IDX_XC);
    state.y_c = x_(IDX_YC);
    state.z_c = x_(IDX_ZC);
    state.height_k = height_k_;
    return state;
}

// ============================================================================
// 辅助方法
// ============================================================================

double OutpostPredictorV4::normalizeAngle360(double angle)
{
    angle = std::fmod(angle, 360.0);
    if (angle < 0) angle += 360.0;
    return angle;
}

double OutpostPredictorV4::normalizeAngle180(double angle)
{
    angle = std::fmod(angle + 180.0, 360.0);
    if (angle < 0) angle += 360.0;
    return angle - 180.0;
}

std::optional<ObservationV4> OutpostPredictorV4::findLastValidObservation() const
{
    // 从历史记录中找到最近的有效观测（不是当前帧）
    if (obs_history_.size() < 2) {
        return std::nullopt;
    }
    
    // 跳过最后一个（当前帧），找倒数第二个
    for (auto it = obs_history_.rbegin() + 1; it != obs_history_.rend(); ++it) {
        if (it->valid) {
            return *it;
        }
    }
    
    return std::nullopt;
}

}  // namespace armor_detector
