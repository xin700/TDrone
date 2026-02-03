/**
 * @file outpost_predictor_v2.hpp
 * @brief 前哨站预测器 V2 - 基于多假设跟踪的前哨站状态估计
 * 
 * 核心改进:
 * 1. **多假设跟踪**: 同时维护3个EKF (假设观测到的是高/中/低装甲板)
 *    - 无需预先知道装甲板是哪一块
 *    - 通过预测误差自动收敛到正确假设
 *    - 解决了"无人机高度可变，无法用绝对高度判断"的问题
 * 
 * 2. 简化状态向量 (5维): [x_c, y_c, z_c, θ, ω]
 * 3. 精确建模三装甲板的高度差
 * 4. 完善的延迟补偿和预测
 * 5. 可击打窗口计算
 * 
 * 工作原理:
 * - 首次观测时，创建3个假设 (H0: 这是高位装甲板, H1: 中位, H2: 低位)
 * - 每个假设用不同方式初始化EKF (反推出不同的z_center)
 * - 后续观测更新所有假设，并根据预测误差更新置信度
 * - 当某个假设置信度超过阈值时，认定为正确假设
 * 
 * @note 保留原有 outpost_estimator.hpp 不变，新旧方法可切换使用
 */

#ifndef ARMOR_DETECTOR_ROS2__CORE__OUTPOST_PREDICTOR_V2_HPP_
#define ARMOR_DETECTOR_ROS2__CORE__OUTPOST_PREDICTOR_V2_HPP_

#include <Eigen/Dense>
#include <array>
#include <memory>
#include <optional>
#include <vector>

#include "core/extended_kalman_filter.hpp"

namespace armor_detector
{

// ============================================================================
// 常量定义
// ============================================================================

/// 前哨站装甲板数量
constexpr int NUM_OUTPOST_ARMORS = 3;

/// 装甲板间隔角度 (120度 = 2π/3)
constexpr double ARMOR_ANGLE_INTERVAL = 2.0 * M_PI / 3.0;

// ============================================================================
// 配置结构体
// ============================================================================

/**
 * @brief 前哨站几何参数配置
 */
struct OutpostGeometry
{
  /// 装甲板到转轴的水平距离 (m)
  double radius = 0.275;
  
  /// 相邻装甲板的高度差 (m), 最高板比中间板高h, 最低板比中间板低h
  double height_diff = 0.10;
  
  /// 装甲板与水平面的夹角 (rad), 接近90度表示几乎垂直
  double tilt_angle = 75.0 * M_PI / 180.0;
  
  /// 装甲板高度偏移 [最高, 中间, 最低]
  std::array<double, 3> getHeightOffsets() const {
    return {height_diff, 0.0, -height_diff};
  }
};

/**
 * @brief EKF 噪声参数配置
 */
struct EKFNoiseParams
{
  /// 位置过程噪声 (m²)
  double process_pos = 0.001;
  
  /// 角度过程噪声 (rad²)
  double process_theta = 0.01;
  
  /// 角速度过程噪声 (rad²/s²)
  double process_omega = 0.001;
  
  /// 位置测量噪声 (m²)
  double measurement_pos = 0.01;
  
  /// 角度测量噪声 (rad²)
  double measurement_angle = 0.05;
};

/**
 * @brief 预测器配置
 */
struct OutpostPredictorConfig
{
  /// 几何参数
  OutpostGeometry geometry;
  
  /// EKF噪声参数
  EKFNoiseParams noise;
  
  /// 是否使用固定角速度 (前哨站角速度已知)
  bool use_fixed_omega = true;
  
  /// 已知的前哨站角速度 (rad/s), 0.4转/秒 ≈ 2.512 rad/s
  double known_omega = 0.4 * 2 * M_PI;
  
  /// 区分静止/旋转的角速度阈值 (rad/s)
  double omega_threshold = 0.3;
  
  /// 系统总延迟 (s), 用于预测补偿
  double system_delay = 0.08;
  
  /// 子弹速度 (m/s)
  double bullet_speed = 25.0;
  
  /// 最大可击打角度 (rad), 装甲板朝向偏离相机超过此角度则不可打
  double max_orientation_angle = 60.0 * M_PI / 180.0;
  
  /// 卡方检验阈值 (用于异常值剔除)
  double chi_square_threshold = 9.49;  // df=4, 95% confidence
  
  /// 方向检测所需的最小样本数
  int direction_detection_samples = 5;
  
  /// 多假设收敛阈值 (置信度超过此值认为收敛)
  double hypothesis_converge_threshold = 0.7;
  
  /// 多假设收敛所需的最小更新次数
  int min_updates_to_converge = 10;
};

// ============================================================================
// 数据结构
// ============================================================================

/**
 * @brief 装甲板观测数据
 */
struct ArmorObservation
{
  /// 装甲板3D位置 (相机坐标系)
  Eigen::Vector3d position{0.0, 0.0, 0.0};
  
  /// 装甲板朝向角 (rad), 相对于相机坐标系
  double orientation = 0.0;
  
  /// 时间戳 (s)
  double timestamp = 0.0;
  
  /// 观测是否有效
  bool valid = false;
  
  /// 识别出的装甲板ID (0, 1, 2), -1表示未知
  int armor_id = -1;
};

/**
 * @brief 前哨站状态
 */
struct OutpostState
{
  /// 转轴中心位置 (相机坐标系)
  Eigen::Vector3d center{0.0, 0.0, 0.0};
  
  /// 当前旋转相位 (rad)
  double theta = 0.0;
  
  /// 角速度 (rad/s), 正值为逆时针
  double omega = 0.0;
  
  /// 旋转方向: -1=顺时针, 0=静止, 1=逆时针
  int direction = 0;
  
  /// 状态是否有效
  bool valid = false;
  
  /// 状态协方差对角线 (用于评估不确定性)
  Eigen::VectorXd covariance_diag;
};

/**
 * @brief 单个装甲板的预测结果
 */
struct ArmorPrediction
{
  /// 装甲板ID (0, 1, 2)
  int armor_id = -1;
  
  /// 预测位置 (相机坐标系)
  Eigen::Vector3d position{0.0, 0.0, 0.0};
  
  /// 预测朝向角 (rad)
  double orientation = 0.0;
  
  /// 是否在可击打范围内
  bool shootable = false;
  
  /// 相对于正面的角度偏差 (rad), 越小越正对相机
  double orientation_error = M_PI;
  
  /// 距离 (m)
  double distance = 0.0;
};

/**
 * @brief 可击打时间窗口
 */
struct ShootableWindow
{
  /// 装甲板ID
  int armor_id = -1;
  
  /// 进入可打范围的时间 (s), 相对于当前时刻
  double enter_time = 0.0;
  
  /// 离开可打范围的时间 (s)
  double exit_time = 0.0;
  
  /// 窗口是否有效
  bool valid = false;
};

/**
 * @brief 瞄准结果
 */
struct AimResult
{
  /// 目标装甲板ID
  int target_armor_id = -1;
  
  /// 瞄准点 (已补偿延迟)
  Eigen::Vector3d aim_point{0.0, 0.0, 0.0};
  
  /// 瞄准点对应的yaw角 (rad)
  double aim_yaw = 0.0;
  
  /// 瞄准点对应的pitch角 (rad)
  double aim_pitch = 0.0;
  
  /// 到目标的距离 (m)
  double distance = 0.0;
  
  /// 是否建议射击
  bool should_shoot = false;
  
  /// 预测置信度 [0, 1]
  double confidence = 0.0;
  
  /// 预测的击中时间 (s), 相对于当前时刻
  double hit_time = 0.0;
  
  /// 子弹飞行时间 (s)
  double bullet_flight_time = 0.0;
};

/**
 * @brief 装甲板高度类型
 */
enum class ArmorHeight
{
  HIGH = 0,    ///< 最高的装甲板
  MIDDLE = 1,  ///< 中间的装甲板
  LOW = 2      ///< 最低的装甲板
};

/**
 * @brief 单个假设的状态
 */
struct Hypothesis
{
  /// 该假设认为观测的装甲板是哪一块
  ArmorHeight assumed_height = ArmorHeight::MIDDLE;
  
  /// 该假设的EKF滤波器
  std::unique_ptr<ExtendedKalmanFilter> ekf;
  
  /// 该假设的置信度 [0, 1]
  double confidence = 1.0 / 3.0;
  
  /// 该假设的累计更新次数
  int num_updates = 0;
  
  /// 最近一次的预测误差 (m)
  double last_prediction_error = 0.0;
};

/**
 * @brief 多假设跟踪器状态
 */
struct MultiHypothesisState
{
  /// 是否已收敛到单一假设
  bool converged = false;
  
  /// 收敛后的最佳假设ID (0, 1, 2 对应 HIGH, MIDDLE, LOW)
  int best_hypothesis_id = -1;
  
  /// 各假设的置信度
  std::array<double, 3> confidences = {1.0/3.0, 1.0/3.0, 1.0/3.0};
  
  /// 各假设的更新次数
  std::array<int, 3> update_counts = {0, 0, 0};
  
  /// 判定结果: 实际观测的是哪个装甲板
  ArmorHeight determined_height = ArmorHeight::MIDDLE;
};

// ============================================================================
// 前哨站预测器 V2
// ============================================================================

/**
 * @class OutpostPredictorV2
 * @brief 前哨站状态估计和预测
 * 
 * 状态向量 X = [x_c, y_c, z_c, θ, ω]^T (5维)
 * 观测向量 Z = [x_a, y_a, z_a, θ_armor]^T (4维)
 * 
 * 主要功能:
 * 1. 接收装甲板观测,估计前哨站状态
 * 2. 预测未来某时刻各装甲板位置
 * 3. 计算可击打窗口
 * 4. 提供补偿延迟后的瞄准点
 */
class OutpostPredictorV2
{
public:
  /// 状态维度
  static constexpr int STATE_DIM = 5;
  
  /// 观测维度
  static constexpr int MEAS_DIM = 4;

  /**
   * @brief 默认构造函数
   */
  OutpostPredictorV2();

  /**
   * @brief 带配置的构造函数
   * @param config 预测器配置
   */
  explicit OutpostPredictorV2(const OutpostPredictorConfig& config);

  /**
   * @brief 设置配置
   * @param config 预测器配置
   */
  void setConfig(const OutpostPredictorConfig& config);

  /**
   * @brief 获取当前配置
   * @return 配置引用
   */
  const OutpostPredictorConfig& getConfig() const { return config_; }

  /**
   * @brief 设置时间步长
   * @param dt 时间步长 (s)
   */
  void setDeltaTime(double dt) { dt_ = dt; }

  /**
   * @brief 用第一个观测初始化
   * @param obs 装甲板观测
   * @param initial_direction 初始方向猜测 (-1, 0, 1)
   */
  void initialize(const ArmorObservation& obs, int initial_direction = 0);

  /**
   * @brief 处理新的观测
   * @param obs 装甲板观测
   * @return 观测是否被接受 (false表示被卡方检验拒绝)
   */
  bool update(const ArmorObservation& obs);

  /**
   * @brief 仅执行预测步骤 (无观测时)
   */
  void predict();

  /**
   * @brief 获取当前前哨站状态估计
   * @return 状态估计
   */
  OutpostState getState() const;

  /**
   * @brief 预测某时刻的所有装甲板位置
   * @param dt 相对于当前时刻的时间偏移 (s)
   * @return 三个装甲板的预测结果
   */
  std::array<ArmorPrediction, 3> predictArmors(double dt) const;

  /**
   * @brief 计算各装甲板的可击打窗口
   * @return 可击打窗口数组
   */
  std::array<ShootableWindow, 3> computeShootableWindows() const;

  /**
   * @brief 计算瞄准结果 (核心函数)
   * @param additional_delay 额外延迟 (s), 在系统延迟基础上增加
   * @return 瞄准结果, 包含目标选择和瞄准点
   */
  AimResult computeAim(double additional_delay = 0.0) const;

  /**
   * @brief 重置预测器状态
   */
  void reset();

  /**
   * @brief 检查是否已初始化
   * @return true 如果已初始化
   */
  bool isInitialized() const { return initialized_; }

  /**
   * @brief 获取旋转方向
   * @return -1=顺时针, 0=静止/未知, 1=逆时针
   */
  int getDirection() const { return direction_; }

  /**
   * @brief 获取最近一次卡方检验值
   * @return 卡方值
   */
  double getLastChiSquare() const { return last_chi_square_; }

  /**
   * @brief 获取EKF状态向量 (调试用)
   * @return 状态向量
   */
  Eigen::VectorXd getStateVector() const;

  /**
   * @brief 获取EKF协方差矩阵 (调试用)
   * @return 协方差矩阵
   */
  Eigen::MatrixXd getCovarianceMatrix() const;

  /**
   * @brief 获取多假设跟踪状态
   * @return 多假设状态信息
   */
  MultiHypothesisState getMultiHypothesisState() const;

  /**
   * @brief 检查多假设是否已收敛
   * @return true 如果已收敛到单一假设
   */
  bool isConverged() const;

  /**
   * @brief 获取当前判定的装甲板高度类型
   * @return 装甲板高度类型
   */
  ArmorHeight getDeterminedHeight() const;

  /**
   * @brief 计算下一个可打窗口的时间
   * @return 距离下一个可打窗口的时间 (s), -1表示当前已可打
   */
  double computeNextWindowTime() const;

  /**
   * @brief 获取距离上次观测的时间
   * @return 时间差 (s)
   */
  double getTimeSinceLastObservation() const;

private:
  // ==================== 成员变量 ====================
  
  /// 配置
  OutpostPredictorConfig config_;
  
  /// 当前活动的EKF滤波器 (收敛后使用)
  std::unique_ptr<ExtendedKalmanFilter> ekf_;
  
  /// 多假设数组 (收敛前使用)
  std::array<Hypothesis, 3> hypotheses_;
  
  /// 多假设是否已收敛
  bool converged_{false};
  
  /// 收敛后的最佳假设ID
  int best_hypothesis_id_{-1};
  
  /// 时间步长 (s)
  double dt_{0.01};
  
  /// 是否已初始化
  bool initialized_{false};
  
  /// 旋转方向: -1=顺时针, 0=未知, 1=逆时针
  int direction_{0};
  
  /// 上一次的theta值 (用于方向检测)
  double last_theta_{0.0};
  
  /// 方向检测: theta变化累积
  double theta_change_sum_{0.0};
  
  /// 方向检测: 已收集的样本数
  int direction_samples_{0};
  
  /// 上一次卡方检验值
  double last_chi_square_{0.0};
  
  /// 上一次观测时间戳
  double last_obs_timestamp_{0.0};
  
  /// 状态转移矩阵
  Eigen::MatrixXd F_;
  
  /// 过程噪声矩阵
  Eigen::MatrixXd Q_;
  
  /// 观测噪声矩阵
  Eigen::MatrixXd R_;
  
  /// 观测雅可比矩阵
  Eigen::MatrixXd H_;

  // ==================== 私有方法 ====================
  
  /**
   * @brief 根据高度识别装甲板ID
   * @param z_armor 装甲板高度
   * @param z_center 中心高度估计
   * @return 装甲板ID (0, 1, 2)
   */
  int identifyArmorByHeight(double z_armor, double z_center) const;

  /**
   * @brief 设置状态转移矩阵
   */
  void setupTransitionMatrix();

  /**
   * @brief 设置过程噪声矩阵
   */
  void setupProcessNoise();

  /**
   * @brief 设置观测噪声矩阵
   */
  void setupMeasurementNoise();

  /**
   * @brief 计算观测雅可比矩阵
   * @param armor_id 观测到的装甲板ID
   */
  void computeObservationJacobian(int armor_id);

  /**
   * @brief 观测方程: 从状态预测观测
   * @param state 状态向量
   * @param armor_id 装甲板ID
   * @return 预测的观测向量
   */
  Eigen::VectorXd observationFunction(const Eigen::VectorXd& state, int armor_id) const;

  /**
   * @brief 从状态计算某个装甲板的位置
   * @param state 状态向量
   * @param armor_id 装甲板ID (0, 1, 2)
   * @return 装甲板位置
   */
  Eigen::Vector3d computeArmorPosition(const Eigen::VectorXd& state, int armor_id) const;

  /**
   * @brief 从状态计算某个装甲板的朝向角
   * @param state 状态向量
   * @param armor_id 装甲板ID
   * @return 朝向角 (rad)
   */
  double computeArmorOrientation(const Eigen::VectorXd& state, int armor_id) const;

  /**
   * @brief 将角度归一化到 [-π, π]
   * @param angle 输入角度
   * @return 归一化后的角度
   */
  static double normalizeAngle(double angle);

  /**
   * @brief 计算角度差 (考虑周期性)
   * @param a 角度a
   * @param b 角度b
   * @return a - b, 归一化到 [-π, π]
   */
  static double angleDiff(double a, double b);

  /**
   * @brief 选择最佳目标装甲板
   * @param predictions 三个装甲板的预测
   * @param hit_delay 击中延迟 (s)
   * @return 选中的装甲板ID, -1表示无可打目标
   */
  int selectTarget(const std::array<ArmorPrediction, 3>& predictions, double hit_delay) const;

  /**
   * @brief 将状态向量预测到未来某时刻
   * @param current_state 当前状态
   * @param dt 时间偏移
   * @return 预测的状态
   */
  Eigen::VectorXd predictState(const Eigen::VectorXd& current_state, double dt) const;

  // ==================== 多假设跟踪私有方法 ====================
  
  /**
   * @brief 初始化多假设 (创建3个不同假设的EKF)
   * @param obs 第一个观测
   * @param initial_direction 初始方向猜测
   */
  void initializeHypotheses(const ArmorObservation& obs, int initial_direction);
  
  /**
   * @brief 为单个假设创建并初始化EKF
   * @param obs 观测数据
   * @param assumed_height 假设该观测是哪个高度的装甲板
   * @param initial_direction 初始方向猜测
   * @return 初始化好的EKF
   */
  std::unique_ptr<ExtendedKalmanFilter> createEKFForHypothesis(
    const ArmorObservation& obs, 
    ArmorHeight assumed_height,
    int initial_direction);
  
  /**
   * @brief 更新所有假设
   * @param obs 新观测
   * @return 最佳假设的更新是否成功
   */
  bool updateAllHypotheses(const ArmorObservation& obs);
  
  /**
   * @brief 更新单个EKF
   * @param obs 观测数据
   * @param ekf 要更新的EKF
   * @param assumed_height 假设的装甲板高度
   * @return 观测是否被接受
   */
  bool updateSingleEKF(
    const ArmorObservation& obs, 
    ExtendedKalmanFilter* ekf, 
    ArmorHeight assumed_height);
  
  /**
   * @brief 更新假设的置信度 (基于预测误差)
   */
  void updateHypothesisConfidences();
  
  /**
   * @brief 检查假设是否收敛，如果收敛则切换到单EKF模式
   */
  void checkConvergence();
  
  /**
   * @brief 获取当前最佳假设的EKF
   * @return EKF指针
   */
  ExtendedKalmanFilter* getBestEKF() const;
  
  /**
   * @brief 获取当前最佳假设ID
   * @return 假设ID (0=HIGH, 1=MIDDLE, 2=LOW)
   */
  int getBestHypothesisId() const;
};

}  // namespace armor_detector

#endif  // ARMOR_DETECTOR_ROS2__CORE__OUTPOST_PREDICTOR_V2_HPP_
