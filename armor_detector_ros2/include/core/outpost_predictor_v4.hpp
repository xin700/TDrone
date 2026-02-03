/**
 * @file outpost_predictor_v4.hpp
 * @brief 前哨站预测器 V4 - 基于长宽比的相位观测 + EKF状态估计
 * 
 * V4版本核心特性:
 * 1. **基于长宽比的θ观测**: 通过装甲板像素长宽比计算相位角
 *    - θ_abs = arccos(aspect_ratio / 2.58)
 *    - 结合上升/下降段状态机确定实际相位
 * 
 * 2. **分阶段初始化**: 
 *    - 第一阶段: 旋转方向初始化 (观测cx变化)
 *    - 第二阶段: 高度状态机初始化 (观测z坐标突变)
 *    - 第三阶段: θ观测初始化 (第一圈观测长宽比变化)
 *    - 第四阶段: EKF初始化与运行
 * 
 * 3. **5维状态空间EKF**:
 *    状态向量 X = [ang, ω, x_c, y_c, z_c]^T
 *    - ang: 累计转角（顺时针从0开始，逆时针从120开始）
 *    - ω: 角速度 (rad/s)
 *    - (x_c, y_c, z_c): 中装甲板旋转中心在机架坐标系中的位置
 * 
 * 4. **观测空间**:
 *    Z = [θ, x_w, y_w, z_w]^T
 *    - θ: 当前观测到的装甲板相对于"正面朝向相机"的相位角
 *    - (x_w, y_w, z_w): PnP解算的装甲板中心机架坐标系坐标
 * 
 * 坐标系约定:
 *   【机架坐标系（Body Frame）】
 *   - X轴: 机架右方向
 *   - Y轴: 机架前方向（机头方向）
 *   - Z轴: 机架上方向（重力反方向）
 *   - 右手系: X(右) × Y(前) = Z(上)
 * 
 * 前哨站几何约定 (从上往下看):
 *   - 三个装甲板按顺时针方向为 Low(0) -> Middle(1) -> High(2)
 *   - 装甲板朝外（法向量指向远离转轴）
 *   - 旋转半径 R = 276.7mm
 *   - 相邻装甲板高度差 100mm
 *   - 标准角速度 0.8π rad/s
 * 
 * @author AI Assistant
 * @date 2026-01-29
 */

#ifndef ARMOR_DETECTOR_ROS2__CORE__OUTPOST_PREDICTOR_V4_HPP_
#define ARMOR_DETECTOR_ROS2__CORE__OUTPOST_PREDICTOR_V4_HPP_

#include <Eigen/Dense>
#include <array>
#include <deque>
#include <memory>
#include <optional>
#include <cmath>
#include <fstream>

namespace armor_detector
{

// ============================================================================
// 常量定义
// ============================================================================

/// 前哨站装甲板数量
constexpr int V4_ARMOR_COUNT = 3;

/// 装甲板间隔角度 (120度)
constexpr double V4_ARMOR_ANGLE_SPACING = 120.0;  // 度

/// 标准前哨站旋转速度 (0.8π rad/s ≈ 2.513 rad/s)
constexpr double V4_STANDARD_OMEGA = 0.8 * M_PI;

/// 标准前哨站装甲板半径 (mm -> m)
constexpr double V4_STANDARD_RADIUS = 0.2767;  // m = 276.7mm

/// 标准装甲板高度差 (100mm)
constexpr double V4_HEIGHT_DIFF = 0.10;  // m

/// 装甲板与地面夹角 (75度)
constexpr double V4_TILT_ANGLE_DEG = 75.0;

/// 最大长宽比（装甲板正对时）
constexpr double V4_MAX_ASPECT_RATIO = 2.58;

// ============================================================================
// 枚举定义
// ============================================================================

/**
 * @brief 旋转方向枚举
 */
enum class RotationDirection : int
{
    CLOCKWISE = -1,     ///< 顺时针
    STATIC = 0,         ///< 静止
    COUNTER_CLOCKWISE = 1  ///< 逆时针
};

/**
 * @brief 装甲板高度状态枚举
 */
enum class HeightState : int
{
    LOW = 0,    ///< 低装甲板
    MIDDLE = 1, ///< 中装甲板
    HIGH = 2    ///< 高装甲板
};

/**
 * @brief 相位状态机枚举
 */
enum class PhaseState : int
{
    RISING = 0,    ///< 上升段（长宽比增大）
    PLATEAU = 1,   ///< 平台区（长宽比接近最大值）
    FALLING = 2    ///< 下降段（长宽比减小）
};

/**
 * @brief 初始化阶段枚举
 */
enum class InitPhase : int
{
    NOT_STARTED = 0,           ///< 未开始
    DIRECTION_INIT = 1,        ///< 方向初始化中
    XY_LINE_INIT = 2,          ///< XY线性方程初始化中
    HEIGHT_INIT = 3,           ///< 高度状态机初始化中
    THETA_INIT = 4,            ///< θ观测初始化中（第一圈）
    EKF_RUNNING = 5            ///< EKF运行中
};

// ============================================================================
// 配置结构体
// ============================================================================

/**
 * @brief V4 预测器配置
 */
struct OutpostConfigV4
{
    // === 几何参数 ===
    double radius = V4_STANDARD_RADIUS;           ///< 旋转半径 (m)
    double height_diff = V4_HEIGHT_DIFF;          ///< 相邻装甲板高度差 (m)
    double tilt_angle_deg = V4_TILT_ANGLE_DEG;    ///< 装甲板与地面夹角 (度)
    double max_aspect_ratio = V4_MAX_ASPECT_RATIO; ///< 最大长宽比
    bool armor_arrangement_reversed = false;      ///< 装甲板排列是否反向（true:低-中-高，false:高-中-低）
    
    // === 运动参数 ===
    double standard_omega = V4_STANDARD_OMEGA;    ///< 标准角速度 (rad/s)
    
    // === 初始化参数 ===
    int direction_init_frames = 10;               ///< 方向初始化所需帧数
    double cx_change_threshold = 30.0;            ///< cx变化阈值（像素）
    double z_jump_threshold = 0.14;               ///< z坐标突变阈值 (m)
    double single_z_jump = 0.06;
    
    // === XY 线性拟合参数 ===
    int xy_line_init_frames = 30;                 ///< XY线性方程初始化所需帧数
    double xy_jump_ratio = 0.4;                 ///< XY投影剧变阈值（线段长度的比例）
    int xy_jump_confirm_frames = 3;               ///< XY剧变确认帧数
    int z_filter_window = 5;                      ///< Z滤波滑动窗口大小

    // === 相位观测参数 ===
    double plateau_threshold = 2.35;               ///< 平台区长宽比阈值
    double plateau_exit_ratio = 0.80;             ///< 平台区退出比例
    double plateau_confirm_ratio = 2.4;           ///< 确认进入平台区的长宽比
    
    // === EKF参数 ===
    // 后验误差协方差P初始化
    double sigma_theta_sq = 0.1;                  ///< θ的方差 (rad²)
    double sigma_omega_sq = 1.0;                  ///< ω的方差 ((rad/s)²)
    double sigma_x_sq = 50.0;                     ///< x的方差 (m²)
    double sigma_y_sq = 50.0;                     ///< y的方差 (m²)
    double sigma_z_sq = 50.0;                     ///< z的方差 (m²)
    
    // 过程噪声Q
    double q_theta = 1e-4;                        ///< θ过程噪声
    double q_omega = 1e-2;                        ///< ω过程噪声
    double q_x = 1.0;                             ///< x过程噪声
    double q_y = 1.0;                             ///< y过程噪声
    double q_z = 0.1;                             ///< z过程噪声
    
    // 观测噪声R
    double r_theta = 0.01;                        ///< θ观测噪声
    double r_x = 0.01;                            ///< x观测噪声
    double r_y = 0.01;                            ///< y观测噪声
    double r_z = 0.01;                            ///< z观测噪声
    
    // === 预瞄参数 ===
    double t_delay = 0.05;                        ///< 系统延迟 (s)
    double filter_delay = 0.1;                    ///< 滤波延迟 (s)，用于滑动窗口滤波补偿
    double moving_delay = 0.2;                    ///< 切换运动延迟 (s)，云台切换预瞄时的运动时间
    double v_bullet = 28.0;                       ///< 子弹速度 (m/s)
    double aim_angle_threshold_deg = 55.0;        ///< 瞄准角阈值 (度)
    
    // === 收敛判断 ===
    int ekf_converge_frames = 50;                 ///< EKF收敛所需帧数
    double height_verify_frames = 10;             ///< 高度状态机与EKF交叉验证的帧数
    int height_recovery_frames = 20;              ///< 高度观测恢复的等待帧数
    
    // 协方差收敛阈值（相对于初始值的比例）
    double cov_omega_threshold = 0.1;             ///< omega协方差阈值（初始值的10%）
    double cov_position_threshold = 0.05;         ///< 位置协方差阈值（初始值的5%）
};

// ============================================================================
// 数据结构
// ============================================================================

/**
 * @brief 单帧观测数据
 */
struct ObservationV4
{
    // 原始PnP解算结果（机架坐标系）
    Eigen::Vector3d position{0.0, 0.0, 0.0};  ///< (x_w, y_w, z_w)
    
    // 装甲板像素信息
    double aspect_ratio = -1.0;                ///< 长宽比（-1表示无效）
    double center_pixel_x = 0.0;               ///< 像素中心x坐标 (cx)
    
    // 坐标转换矩阵
    Eigen::Matrix3d t_gimbal_camera = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d r_body_gimbal = Eigen::Matrix3d::Identity();
    
    // 时间戳
    double timestamp = 0.0;
    
    // IMU数据（当前帧对应）
    double imu_yaw = 0.0;                      ///< 当前帧对应的IMU yaw角度 (度)
    double imu_pitch = 0.0;                    ///< 当前帧对应的IMU pitch角度 (度)
    
    // 有效性
    bool valid = false;
};

/**
 * @brief 平台区标定数据
 */
struct PlateauCalibration
{
    double cx_enter = 0.0;    ///< 进入平台区时的cx
    double cx_exit = 0.0;     ///< 退出平台区时的cx
    bool calibrated = false;  ///< 是否已标定
};

/**
 * @brief 预瞄结果
 */
struct AimResultV4
{
    // 预瞄点（机架坐标系）
    Eigen::Vector3d aim_position{0.0, 0.0, 0.0};
    
    // 预瞄装甲板高度状态
    HeightState aim_height_state = HeightState::MIDDLE;
    
    // 预瞄装甲板的局部相位角 (度, -60~60范围)
    double aim_ang_local = 0.0;
    
    // 更远预瞄点（机架坐标系）- 用于云台切换预判
    Eigen::Vector3d further_aim_position{0.0, 0.0, 0.0};
    
    // 更远预瞄点的局部相位角 (度, -60~60范围)
    double further_aim_ang = 0.0;
    
    // 更远预瞄点的高度状态
    HeightState further_aim_height_state = HeightState::MIDDLE;
    
    // 开火条件
    bool fire_condition_1 = false;
    
    // 坐标转换矩阵
    Eigen::Matrix3d t_gimbal_camera = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d r_body_gimbal = Eigen::Matrix3d::Identity();
    
    // 弹道信息
    double bullet_flight_time = 0.0;
    double distance = 0.0;
    
    // IMU数据（当前帧对应）
    double imu_yaw = 0.0;
    double imu_pitch = 0.0;
    
    // 有效性
    bool valid = false;
};

/**
 * @brief EKF状态
 */
struct EKFStateV4
{
    double ang = 0.0;     ///< 累计转角 (度)
    double omega = 0.0;   ///< 角速度 (rad/s)
    double x_c = 0.0;     ///< 中心x坐标 (m)
    double y_c = 0.0;     ///< 中心y坐标 (m)
    double z_c = 0.0;     ///< 中心z坐标 (m)
    HeightState height_k = HeightState::MIDDLE;  ///< 当前装甲板高度状态（由ang决定）
};

// ============================================================================
// 前哨站预测器 V4 类
// ============================================================================

/**
 * @class OutpostPredictorV4
 * @brief 前哨站运动预测与预瞄
 */
class OutpostPredictorV4
{
public:
    /// 状态维度: [ang, ω, x_c, y_c, z_c]
    static constexpr int STATE_DIM = 5;
    
    /// 观测维度: [θ, x_w, y_w, z_w]
    static constexpr int MEAS_DIM = 4;
    
    // 状态索引
    static constexpr int IDX_ANG = 0;
    static constexpr int IDX_OMEGA = 1;
    static constexpr int IDX_XC = 2;
    static constexpr int IDX_YC = 3;
    static constexpr int IDX_ZC = 4;

    // double offset = 0.0;

    /**
     * @brief 构造函数
     */
    OutpostPredictorV4();
    
    /**
     * @brief 带配置的构造函数
     */
    explicit OutpostPredictorV4(const OutpostConfigV4& config);
    
    /**
     * @brief 设置配置
     */
    void setConfig(const OutpostConfigV4& config);
    
    /**
     * @brief 获取配置
     */
    const OutpostConfigV4& getConfig() const { return config_; }
    
    /**
     * @brief 处理新观测
     * @param obs 观测数据
     * @return 是否成功处理
     */
    bool update(const ObservationV4& obs);
    
    /**
     * @brief 计算预瞄结果
     * @return 预瞄结果
     */
    AimResultV4 computeAim() const;
    
    /**
     * @brief 重置预测器
     */
    void reset();
    
    // === 状态查询接口 ===
    
    InitPhase getInitPhase() const { return init_phase_; }
    RotationDirection getRotationDirection() const { return rotation_direction_; }
    HeightState getCurrentHeightState() const { return current_height_state_; }
    PhaseState getPhaseState() const { return phase_state_; }
    bool isEKFInitialized() const { return ekf_initialized_; }
    bool isEKFConverged() const { return ekf_converged_; }
    bool isHeightInitialized() const { return height_initialized_; }
    bool isStaticConfirmed() const { return static_confirmed_; }  ///< 是否确认完全静止
    
    /**
     * @brief 获取EKF状态
     */
    EKFStateV4 getEKFState() const;
    
    /**
     * @brief 获取状态向量（调试用）
     */
    Eigen::VectorXd getStateVector() const { return x_; }
    
    /**
     * @brief 获取协方差矩阵（调试用）
     */
    Eigen::MatrixXd getCovarianceMatrix() const { return P_; }
    
    /**
     * @brief 获取当前观测的theta
     */
    double getCurrentTheta() const { return current_theta_; }
    
    /**
     * @brief 获取最近一次测量残差
     */
    Eigen::Vector4d getLastResidual() const { return last_residual_; }

private:
    // ==================== 初始化相关方法 ====================
    
    /**
     * @brief 更新旋转方向初始化
     */
    void updateDirectionInit(const ObservationV4& obs);
    
    /**
     * @brief 更新高度状态机初始化
     */
    void updateHeightInit(const ObservationV4& obs);
    
    /**
     * @brief 更新θ观测初始化（第一圈）
     */
    void updateThetaInit(const ObservationV4& obs);
    
    /**
     * @brief 初始化EKF
     */
    void initializeEKF(const ObservationV4& obs);
    
    // ==================== θ观测相关方法 ====================
    
    /**
     * @brief 从长宽比计算θ绝对值
     * @return θ_abs (度, 0~60范围)
     */
    double computeThetaAbs(double aspect_ratio) const;
    
    /**
     * @brief 根据相位状态计算带符号的θ
     * @return θ (度, -60~60范围)
     */
    double computeTheta(double aspect_ratio, PhaseState phase) const;
    
    /**
     * @brief 更新相位状态机
     */
    void updatePhaseState(const ObservationV4& obs);
    
    /**
     * @brief 在平台区使用cx插值计算θ
     */
    double computePlateauTheta(double cx) const;
    
    // ==================== 高度状态机相关方法 ====================
    
    /**
     * @brief 更新高度状态机（基于观测）
     */
    void updateHeightStateMachine(const ObservationV4& obs);
    
    /**
     * @brief 从EKF的ang计算高度状态
     */
    HeightState computeHeightFromAng(double ang) const;
    
    /**
     * @brief 高度状态切换逻辑
     */
    HeightState nextHeightState(HeightState current, bool positive_direction) const;
    HeightState prevHeightState(HeightState current, bool positive_direction) const;
    HeightState computeNextHeight(HeightState current_height, int direction) const;
    
    /**
     * @brief 根据观测z值确定最接近的高度状态
     */
    HeightState determineHeightFromZ(double z) const;
    
    /**
     * @brief 检测并确认高度跳动（带防抖）
     * @return 确认的跳动z差（0表示无确认的跳动）
     */
    double detectConfirmedJump(double current_z);
    
    /**
     * @brief 更新EKF高度状态（基于ang跨临界检测）
     */
    void updateEKFHeightState();
    
    // ==================== X 范围跟踪相关方法 ====================
    
    /**
     * @brief 更新 X 范围初始化
     */
    void updateXRangeInit(const ObservationV4& obs);
    
    /**
     * @brief 更新 X 范围（持续扩展）
     */
    void updateXRange(const ObservationV4& obs);
    
    /**
     * @brief 更新 Z 滤波值
     */
    void updateFilteredZ(double z);
    
    /**
     * @brief 基于 X 坐标检测装甲板切换
     * @return 确认的 z 跳变方向（正/负/0）
     */
    double detectArmorSwitchByX(const ObservationV4& obs);
    
    // ==================== EKF相关方法 ====================
    
    /**
     * @brief EKF预测步骤
     */
    void ekfPredict(double dt);
    
    /**
     * @brief EKF更新步骤
     */
    void ekfUpdate(const ObservationV4& obs, double theta);
    
    /**
     * @brief 计算预测的装甲板位置
     */
    Eigen::Vector3d computePredictedArmorPosition(double ang_local, HeightState height) const;
    
    /**
     * @brief 计算状态转移雅可比矩阵F
     */
    Eigen::MatrixXd computeF(double dt) const;
    
    /**
     * @brief 计算观测雅可比矩阵H
     */
    Eigen::MatrixXd computeH(double ang_local) const;
    
    /**
     * @brief 获取过程噪声矩阵Q
     */
    Eigen::MatrixXd getQ() const;
    
    /**
     * @brief 获取观测噪声矩阵R
     */
    Eigen::MatrixXd getR() const;
    
    // ==================== 辅助方法 ====================
    
    /**
     * @brief 角度归一化到 [0, 360)
     */
    static double normalizeAngle360(double angle);
    
    /**
     * @brief 角度归一化到 [-180, 180)
     */
    static double normalizeAngle180(double angle);
    
    /**
     * @brief 找到最近一个有效的历史观测
     */
    std::optional<ObservationV4> findLastValidObservation() const;

    // ==================== 成员变量 ====================
    
    // 配置
    OutpostConfigV4 config_;
    
    // === 初始化状态 ===
    InitPhase init_phase_{InitPhase::NOT_STARTED};
    int init_frame_count_{0};
    
    // === 旋转方向 ===
    RotationDirection rotation_direction_{RotationDirection::STATIC};
    std::deque<double> cx_history_;
    
    // === X 坐标范围（用于检测装甲板切换） ===
    bool x_range_initialized_{false};              ///< X 范围是否已初始化
    std::vector<double> x_init_values_;            ///< X 初始化期间收集的值
    double x_min_{0.0};                            ///< X 的最小值
    double x_max_{0.0};                            ///< X 的最大值
    double last_x_{0.0};                           ///< 上一帧的 x 值
    bool last_x_valid_{false};                     ///< 上一帧 x 是否有效
    
    // === X 剧变检测 ===
    bool pending_x_jump_{false};                   ///< 是否有待确认的 X 剧变
    int pending_x_stable_frames_{0};               ///< X 剧变后稳定帧数
    double pre_jump_filtered_z_{0.0};              ///< 剧变前一帧的滤波 z 值
    
    // === Z 滑动窗口滤波 ===
    std::deque<double> z_filter_window_;           ///< Z 滤波滑动窗口
    double current_filtered_z_{0.0};               ///< 当前滤波后的 z 值
    
    // === 高度状态机 ===
    HeightState current_height_state_{HeightState::MIDDLE};
    HeightState predicted_next_height_{HeightState::MIDDLE};
    bool height_initialized_{false};
    int height_verify_count_{0};
    int height_lost_count_{0};
    bool height_sync_with_ekf_{false};
    
    // === 高度标定数据（新机制）===
    double height_calibration_[3] = {0.0, 0.0, 0.0};  ///< 三个高度档位的平均z值 [LOW, MIDDLE, HIGH]
    bool height_calibrated_{false};                    ///< 是否完成高度标定
    int height_init_jump_count_{0};                    ///< 高度初始化期间的突变次数
    std::vector<double> current_height_group_;         ///< 当前高度组的z值累积
    double last_valid_z_{0.0};                         ///< 上一次有效的z坐标
    
    // === 突变检测（防抖动机制）===
    double pending_jump_z_diff_{0.0};                  ///< 待确认的跳动z差
    int pending_jump_stable_frames_{0};                ///< 跳动后稳定帧数
    bool pending_jump_active_{false};                  ///< 是否有待确认的跳动
    double pending_jump_last_z_{0.0};                  ///< 跳变后记录的z值
    static constexpr int JUMP_CONFIRM_FRAMES = 4;      ///< 确认跳动所需稳定帧数;
    
    // === 相位状态机 ===
    PhaseState phase_state_{PhaseState::RISING};
    bool first_z_jump_detected_{false};
    double max_aspect_ratio_recorded_{0.0};
    double max_aspect_ratio_position_x_{0.0};
    double max_aspect_ratio_position_y_{0.0};
    PlateauCalibration plateau_calib_;
    bool plateau_entered_{false};
    bool plateau_aspect_ratio_peaked_{false};
    
    // === FALLING状态超时检测 ===
    double min_ar_in_falling_{1.0};                    ///< FALLING状态中记录的最小长宽比
    int falling_frames_{0};                            ///< FALLING状态持续帧数
    double falling_start_time_{0.0};                   ///< FALLING状态开始时间戳
    static constexpr double FALLING_TIMEOUT_SEC = 0.25; ///< FALLING状态超时阈值（秒）
    
    // === 静止确认机制 ===
    bool static_confirmed_{false};                     ///< 是否确认完全静止
    double static_start_time_{0.0};                    ///< 静止判定开始时间戳
    double last_static_check_x_{0.0};                  ///< 用于静止检测的上一帧x坐标
    int static_check_frames_{0};                       ///< 静止检测帧数
    static constexpr double STATIC_CONFIRM_SEC = 3.0;  ///< 静止确认阈值（秒）
    static constexpr double STATIC_MOVE_THRESHOLD = 0.02; ///< 静止检测移动阈值（米）
    
    // === EKF高度状态跟踪（新机制：基于ang跨临界）===
    double last_ekf_ang_{0.0};                         ///< 上一次EKF的ang值（用于检测跨临界）

    // === θ观测 ===
    double current_theta_{0.0};
    double last_theta_{0.0};
    bool theta_initialized_{false};
    
    // === EKF状态 ===
    Eigen::VectorXd x_;   // 状态向量
    Eigen::MatrixXd P_;   // 协方差矩阵
    bool ekf_initialized_{false};
    bool ekf_converged_{false};
    int ekf_update_count_{0};
    HeightState height_k_{HeightState::MIDDLE};  // EKF中的高度状态指示
    Eigen::Vector4d last_residual_{Eigen::Vector4d::Zero()};
    
    // === 观测历史 ===
    std::deque<ObservationV4> obs_history_;
    static constexpr size_t MAX_HISTORY_SIZE = 30;
    
    // === 时间 ===
    double last_timestamp_{0.0};
    
    // === 最近有效观测的转换矩阵（用于输出） ===
    Eigen::Matrix3d last_t_gimbal_camera_{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d last_r_body_gimbal_{Eigen::Matrix3d::Identity()};
    
    // === 最近的IMU数据（用于转发） ===
    double last_imu_yaw_{0.0};
    double last_imu_pitch_{0.0};
    
    // === CSV 日志记录 ===
    std::ofstream csv_log_file_;
};

}  // namespace armor_detector

#endif  // ARMOR_DETECTOR_ROS2__CORE__OUTPOST_PREDICTOR_V4_HPP_
