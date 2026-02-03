/**
 * @file outpost_predictor_v3.hpp
 * @brief 前哨站预测器 V3 - 基于极坐标观测的前哨站状态估计
 * 
 * V3版本改进要点:
 * 1. **正确的观测模型**: 使用极坐标观测 (yaw, pitch, distance, orientation)
 *    - 避免了笛卡尔坐标系下的非线性问题
 *    - 更符合相机测量的物理特性
 * 
 * 2. **简化的状态空间**: 
 *    状态向量 X = [θ_center, ω, x_c, y_c, z_c]^T (5维)
 *    - θ_center: 前哨站当前旋转相位(基准装甲板的朝向角)
 *    - ω: 角速度 (rad/s)，正值为逆时针(从上往下看)
 *    - (x_c, y_c, z_c): 前哨站旋转中心在预测器内部坐标系中的位置
 * 
 * 3. **鲁棒的装甲板识别**:
 *    - 基于高度差和角度连续性同时判断
 *    - 支持装甲板切换时的状态修正
 * 
 * 4. **改进的瞄准策略**:
 *    - 参考rm.cv.fans的direct/indirect瞄准
 *    - 正确计算可击打窗口和提前量
 * 
 * 坐标系约定:
 *   【预测器内部坐标系（相机坐标系）】
 *   - X轴: 向右
 *   - Y轴: 向下
 *   - Z轴: 向前 (光轴方向)
 * 
 *   【外部接口坐标系（机架坐标系）】
 *   - X轴: 向右
 *   - Y轴: 向上
 *   - Z轴: 向后 (机头前方是-Z)
 * 
 *   【转换关系】
 *   - X_cam = X_body
 *   - Y_cam = -Y_body
 *   - Z_cam = -Z_body
 * 
 * 前哨站几何约定 (从上往下看):
 *   - 三个装甲板按顺时针方向为 Low -> Middle -> High
 *   - 装甲板朝外 (法向量指向远离转轴)
 *   - θ_center = 0 时，High装甲板正对相机
 *   - ω > 0: 逆时针旋转 (从上往下看)
 *   - ω < 0: 顺时针旋转
 * 
 * @note 保留原有 outpost_predictor_v2.hpp 不变，V2和V3可切换使用
 * @note 节点层负责机架坐标系与预测器内部坐标系之间的转换
 */

#ifndef ARMOR_DETECTOR_ROS2__CORE__OUTPOST_PREDICTOR_V3_HPP_
#define ARMOR_DETECTOR_ROS2__CORE__OUTPOST_PREDICTOR_V3_HPP_

#include <Eigen/Dense>
#include <array>
#include <deque>
#include <memory>
#include <optional>
#include <vector>
#include <functional>

namespace armor_detector
{

// ============================================================================
// 常量定义
// ============================================================================

/// 前哨站装甲板数量
constexpr int OUTPOST_ARMOR_COUNT = 3;

/// 装甲板间隔角度 (120度 = 2π/3)
constexpr double ARMOR_ANGLE_SPACING = 2.0 * M_PI / 3.0;  // 120°

/// 标准前哨站旋转速度 (0.4转/秒 = 0.8π rad/s)
constexpr double STANDARD_OUTPOST_OMEGA = 0.8 * M_PI;

/// 标准前哨站装甲板半径
constexpr double STANDARD_OUTPOST_RADIUS = 0.2767;  // m

/// 标准装甲板高度差 (最高与中间相差100mm)
constexpr double STANDARD_HEIGHT_DIFF = 0.10;  // m

// ============================================================================
// 配置结构体
// ============================================================================

/**
 * @brief 前哨站几何参数
 */
struct OutpostGeometryV3
{
  /// 装甲板到转轴的水平距离 (m)
  double radius = STANDARD_OUTPOST_RADIUS;
  
  /// 相邻装甲板的高度差 (m)
  /// 从上往下俯视顺时针方向: Low(-h) -> Middle(0) -> High(+h)
  double height_diff = STANDARD_HEIGHT_DIFF;
  
  /// 装甲板与地面的夹角 (rad)
  /// 75度表示装甲板略微朝下
  double tilt_angle = 75.0 * M_PI / 180.0;
  
  /**
   * @brief 获取各装甲板的高度偏移 [High, Middle, Low]
   * 相对于中心轴的高度偏移
   */
  std::array<double, 3> getHeightOffsets() const {
    // 从上往下看顺时针: Low(-h) -> Middle(0) -> High(+h)
    // 但数组索引按 High(0), Middle(1), Low(2) 排列
    return {height_diff, 0.0, -height_diff};
  }
  
  /**
   * @brief 获取各装甲板的相位偏移 (相对于θ_center)
   * 从上往下看顺时针排列: Low -> Middle -> High
   * 使用逆时针正角度约定，所以 High在θ_center, Middle在θ_center-120°, Low在θ_center-240°
   */
  std::array<double, 3> getPhaseOffsets() const {
    // High(0): 0°, Middle(1): -120°, Low(2): -240° (即+120°)
    return {0.0, -ARMOR_ANGLE_SPACING, -2.0 * ARMOR_ANGLE_SPACING};
  }
};

/**
 * @brief EKF 噪声参数
 */
struct EKFNoiseV3
{
  // === 过程噪声 (预测不确定性) ===
  /// 角度过程噪声 (rad²)
  double q_theta = 0.01;
  
  /// 角速度过程噪声 (rad²/s²)
  double q_omega = 0.001;
  
  /// 位置过程噪声 (m²)
  double q_position = 0.001;
  
  // === 观测噪声 ===
  /// 方位角观测噪声 (rad²) - yaw
  double r_yaw = 0.01;
  
  /// 俯仰角观测噪声 (rad²) - pitch
  double r_pitch = 0.01;
  
  /// 距离观测噪声 (m²)
  double r_distance = 0.01;
  
  /// 装甲板朝向观测噪声 (rad²)
  double r_orientation = 0.05;
};

/**
 * @brief V3 预测器配置
 */
struct OutpostConfigV3
{
  /// 几何参数
  OutpostGeometryV3 geometry;
  
  /// EKF噪声参数
  EKFNoiseV3 noise;
  
  /// 是否使用固定角速度
  bool use_fixed_omega = true;
  
  /// 已知角速度 (rad/s)
  double known_omega = STANDARD_OUTPOST_OMEGA;
  
  /// 角速度阈值 (低于此值认为静止)
  double omega_threshold = 0.5;
  
  /// 系统延迟 (s)
  double system_delay = 0.05;
  
  /// 子弹速度 (m/s)
  double bullet_speed = 28.0;
  
  /// 最大可击打朝向角 (rad)
  /// 装甲板法向量与视线夹角小于此值时可击打
  double max_shootable_angle = 60.0 * M_PI / 180.0;
  
  /// 多假设收敛阈值
  double hypothesis_converge_threshold = 0.8;
  
  /// 收敛所需最小更新次数
  int min_updates_to_converge = 15;
  
  /// 最大丢失帧数
  int max_lost_frames = 30;
  
  /// 卡方检验阈值 (df=4)
  double chi_square_threshold = 9.49;
  
  /// 用于装甲板切换判断的角度阈值 (rad)
  double switch_angle_threshold = M_PI / 3.0;  // 60°
};

// ============================================================================
// 数据结构
// ============================================================================

/**
 * @brief 装甲板观测 (极坐标形式)
 */
struct ArmorObservationV3
{
  /// 方位角 (rad) - 水平面内相对于相机光轴的角度
  double yaw = 0.0;
  
  /// 俯仰角 (rad) - 垂直面内相对于水平面的角度
  double pitch = 0.0;
  
  /// 距离 (m)
  double distance = 0.0;
  
  /// 装甲板朝向角 (rad) - 装甲板法向量在水平面的投影角
  double orientation = 0.0;
  
  /// 装甲板在相机坐标系中的位置 (供参考，主要用于可视化)
  Eigen::Vector3d position{0.0, 0.0, 0.0};
  
  /// 时间戳 (s)
  double timestamp = 0.0;
  
  /// 是否有效
  bool valid = false;
};

/**
 * @brief 装甲板类型枚举
 */
enum class ArmorType : int
{
  HIGH = 0,     ///< 最高的装甲板
  MIDDLE = 1,   ///< 中间的装甲板
  LOW = 2       ///< 最低的装甲板
};

/**
 * @brief 单个假设的状态
 */
struct HypothesisV3
{
  /// 该假设认为观测的装甲板类型
  ArmorType assumed_type = ArmorType::MIDDLE;
  
  /// 状态向量 [θ_center, ω, x_c, y_c, z_c]
  Eigen::VectorXd state;
  
  /// 协方差矩阵
  Eigen::MatrixXd covariance;
  
  /// 置信度 [0, 1]
  double confidence = 1.0 / 3.0;
  
  /// 累计更新次数
  int update_count = 0;
  
  /// 最近一次的残差范数
  double last_residual_norm = 0.0;
  
  /// 最近一次的马氏距离
  double last_mahalanobis_distance = 0.0;
};

/**
 * @brief 预测的装甲板状态
 */
struct PredictedArmorV3
{
  /// 装甲板类型
  ArmorType type = ArmorType::MIDDLE;
  
  /// 位置 (相机坐标系)
  Eigen::Vector3d position{0.0, 0.0, 0.0};
  
  /// 朝向角 (rad)
  double orientation = 0.0;
  
  /// 与相机视线的夹角 (rad) - 越小越正对
  double facing_angle = M_PI;
  
  /// 距离 (m)
  double distance = 0.0;
  
  /// 是否可击打
  bool shootable = false;
};

/**
 * @brief 瞄准结果
 */
struct AimResultV3
{
  /// 目标装甲板类型
  ArmorType target_type = ArmorType::MIDDLE;
  
  /// 瞄准点 (相机坐标系)
  Eigen::Vector3d aim_point{0.0, 0.0, 0.0};
  
  /// 瞄准的yaw角 (rad)
  double aim_yaw = 0.0;
  
  /// 瞄准的pitch角 (rad)
  double aim_pitch = 0.0;
  
  /// 距离 (m)
  double distance = 0.0;
  
  /// 是否应该射击
  bool should_shoot = false;
  
  /// 置信度
  double confidence = 0.0;
  
  /// 预计击中时间 (s)
  double hit_time = 0.0;
  
  /// 瞄准模式: "direct"=直接瞄准可见装甲板, "indirect"=等待下一块装甲板
  std::string aim_mode = "none";
};

/**
 * @brief 前哨站状态
 */
struct OutpostStateV3
{
  /// 旋转中心位置
  Eigen::Vector3d center{0.0, 0.0, 0.0};
  
  /// 当前相位 (rad)
  double theta = 0.0;
  
  /// 角速度 (rad/s)
  double omega = 0.0;
  
  /// 旋转方向: -1=顺时针, 0=静止, 1=逆时针
  int direction = 0;
  
  /// 是否有效
  bool valid = false;
  
  /// 是否已收敛
  bool converged = false;
  
  /// 最佳假设的类型
  ArmorType best_hypothesis_type = ArmorType::MIDDLE;
  
  /// 各假设的置信度
  std::array<double, 3> hypothesis_confidences = {1.0/3.0, 1.0/3.0, 1.0/3.0};
};

// ============================================================================
// 前哨站预测器 V3 类
// ============================================================================

/**
 * @class OutpostPredictorV3
 * @brief 前哨站状态估计和预测器
 * 
 * 核心算法:
 * 1. 多假设跟踪: 同时维护3个EKF假设 (观测的是High/Middle/Low)
 * 2. 基于残差的假设竞争: 使用马氏距离更新置信度
 * 3. 装甲板切换检测: 基于角度跳变识别切换事件
 * 4. 极坐标观测模型: 更好地处理相机测量噪声
 */
class OutpostPredictorV3
{
public:
  /// 状态维度: [θ_center, ω, x_c, y_c, z_c]
  static constexpr int STATE_DIM = 5;
  
  /// 观测维度: [yaw, pitch, distance, orientation]
  static constexpr int MEAS_DIM = 4;
  
  // 状态索引
  static constexpr int IDX_THETA = 0;
  static constexpr int IDX_OMEGA = 1;
  static constexpr int IDX_XC = 2;
  static constexpr int IDX_YC = 3;
  static constexpr int IDX_ZC = 4;

  /**
   * @brief 构造函数
   */
  OutpostPredictorV3();
  
  /**
   * @brief 带配置的构造函数
   */
  explicit OutpostPredictorV3(const OutpostConfigV3& config);
  
  /**
   * @brief 设置配置
   */
  void setConfig(const OutpostConfigV3& config);
  
  /**
   * @brief 获取配置
   */
  const OutpostConfigV3& getConfig() const { return config_; }
  
  /**
   * @brief 初始化预测器
   * @param obs 第一个观测
   * @param initial_direction 初始旋转方向猜测 (-1, 0, 1)
   */
  void initialize(const ArmorObservationV3& obs, int initial_direction = 0);
  
  /**
   * @brief 处理新观测
   * @param obs 装甲板观测
   * @return 是否成功处理
   */
  bool update(const ArmorObservationV3& obs);
  
  /**
   * @brief 仅执行预测 (无观测时调用)
   * @param dt 时间步长 (s)
   */
  void predict(double dt);
  
  /**
   * @brief 获取当前前哨站状态
   */
  OutpostStateV3 getState() const;
  
  /**
   * @brief 预测未来时刻的装甲板位置
   * @param dt 相对于当前时刻的时间偏移 (s)
   */
  std::array<PredictedArmorV3, 3> predictArmors(double dt) const;
  
  /**
   * @brief 计算瞄准结果
   * @param additional_delay 额外延迟 (s)
   */
  AimResultV3 computeAim(double additional_delay = 0.0) const;
  
  /**
   * @brief 重置预测器
   */
  void reset();
  
  /**
   * @brief 检查是否已初始化
   */
  bool isInitialized() const { return initialized_; }
  
  /**
   * @brief 检查是否已收敛
   */
  bool isConverged() const { return converged_; }
  
  /**
   * @brief 获取旋转方向
   */
  int getDirection() const { return direction_; }
  
  /**
   * @brief 获取当前跟踪的装甲板类型
   */
  ArmorType getTrackedArmorType() const;
  
  /**
   * @brief 获取状态向量 (调试用)
   */
  Eigen::VectorXd getStateVector() const;
  
  /**
   * @brief 获取协方差矩阵 (调试用)
   */
  Eigen::MatrixXd getCovarianceMatrix() const;

private:
  // ==================== 内部方法 ====================
  
  /**
   * @brief 初始化多假设
   */
  void initializeHypotheses(const ArmorObservationV3& obs, int initial_direction);
  
  /**
   * @brief 从观测反推状态 (用于初始化)
   */
  Eigen::VectorXd observationToState(const ArmorObservationV3& obs, ArmorType assumed_type) const;
  
  /**
   * @brief EKF预测步骤
   */
  void ekfPredict(HypothesisV3& hyp, double dt);
  
  /**
   * @brief EKF更新步骤
   * @return 是否接受该观测
   */
  bool ekfUpdate(HypothesisV3& hyp, const ArmorObservationV3& obs);
  
  /**
   * @brief 状态转移函数
   */
  Eigen::VectorXd stateTransition(const Eigen::VectorXd& state, double dt) const;
  
  /**
   * @brief 计算状态转移雅可比矩阵
   */
  Eigen::MatrixXd computeF(double dt) const;
  
  /**
   * @brief 观测函数: 从状态计算预期观测
   */
  Eigen::VectorXd observationFunction(const Eigen::VectorXd& state, ArmorType type) const;
  
  /**
   * @brief 计算观测雅可比矩阵
   */
  Eigen::MatrixXd computeH(const Eigen::VectorXd& state, ArmorType type) const;
  
  /**
   * @brief 从状态计算装甲板位置
   */
  Eigen::Vector3d computeArmorPosition(const Eigen::VectorXd& state, ArmorType type) const;
  
  /**
   * @brief 从状态计算装甲板朝向角
   */
  double computeArmorOrientation(const Eigen::VectorXd& state, ArmorType type) const;
  
  /**
   * @brief 更新假设置信度
   */
  void updateHypothesisConfidences();
  
  /**
   * @brief 检查是否收敛
   */
  void checkConvergence();
  
  /**
   * @brief 获取最佳假设索引
   */
  int getBestHypothesisIndex() const;
  
  /**
   * @brief 检测装甲板切换
   */
  bool detectArmorSwitch(const ArmorObservationV3& obs);
  
  /**
   * @brief 处理装甲板切换
   */
  void handleArmorSwitch(ArmorType new_type);
  
  /**
   * @brief 更新方向估计
   */
  void updateDirectionEstimate(const ArmorObservationV3& obs);
  
  /**
   * @brief 角度归一化到 [-π, π]
   */
  static double normalizeAngle(double angle);
  
  /**
   * @brief 计算角度差 (a - b), 结果在 [-π, π]
   */
  static double angleDiff(double a, double b);
  
  /**
   * @brief 找到最接近目标的角度 (处理周期性)
   */
  static double getClosestAngle(double angle, double target);
  
  /**
   * @brief 笛卡尔坐标转极坐标
   */
  static void cartesianToPolar(const Eigen::Vector3d& xyz, double& yaw, double& pitch, double& dist);
  
  /**
   * @brief 极坐标转笛卡尔坐标
   */
  static Eigen::Vector3d polarToCartesian(double yaw, double pitch, double dist);
  
  /**
   * @brief 获取过程噪声矩阵
   */
  Eigen::MatrixXd getQ(double dt) const;
  
  /**
   * @brief 获取观测噪声矩阵
   */
  Eigen::MatrixXd getR() const;
  
  /**
   * @brief 选择直接瞄准目标
   */
  AimResultV3 selectDirectTarget(const std::array<PredictedArmorV3, 3>& armors) const;
  
  /**
   * @brief 计算间接瞄准点 (等待下一块装甲板)
   */
  AimResultV3 computeIndirectAim(const std::array<PredictedArmorV3, 3>& armors) const;

  // ==================== 成员变量 ====================
  
  /// 配置
  OutpostConfigV3 config_;
  
  /// 多假设数组
  std::array<HypothesisV3, 3> hypotheses_;
  
  /// 是否已收敛
  bool converged_{false};
  
  /// 收敛后的最佳假设索引
  int best_hypothesis_idx_{-1};
  
  /// 是否已初始化
  bool initialized_{false};
  
  /// 旋转方向: -1=顺时针, 0=未知, 1=逆时针
  int direction_{0};
  
  /// 当前跟踪的装甲板类型
  ArmorType tracked_armor_type_{ArmorType::MIDDLE};
  
  /// 方向检测相关
  double last_orientation_{0.0};
  double orientation_change_sum_{0.0};
  int direction_samples_{0};
  
  /// 时间相关
  double last_obs_timestamp_{0.0};
  double last_predict_timestamp_{0.0};
  
  /// 丢失计数
  int lost_count_{0};
  
  /// 最近的观测历史 (用于装甲板切换检测)
  std::deque<ArmorObservationV3> obs_history_;
  static constexpr size_t MAX_HISTORY_SIZE = 10;
};

}  // namespace armor_detector

#endif  // ARMOR_DETECTOR_ROS2__CORE__OUTPOST_PREDICTOR_V3_HPP_
