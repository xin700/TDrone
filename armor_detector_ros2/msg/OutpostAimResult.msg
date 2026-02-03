# OutpostAimResult.msg
# 前哨站预瞄结果消息 V4
# 包含预瞄点位置、开火条件、坐标转换矩阵等

# 消息头（包含时间戳和坐标系）
std_msgs/Header header

# ==================== 预瞄点位置（机架坐标系） ====================
# X_aim_pred, Y_aim_pred, Z_aim_pred (米)
geometry_msgs/Point aim_position

# 预瞄装甲板的高度状态 (0=Low, 1=Middle, 2=High)
int32 aim_height_state

# 预瞄装甲板的局部相位角 (度, -60~60范围)
float64 aim_ang_local

# ==================== 更远预瞄点位置（机架坐标系） ====================
# 用于云台切换状态预判，预测total_delay + moving_delay时间后的装甲板位置
geometry_msgs/Point further_aim_position

# 更远预瞄点的局部相位角 (度, -60~60范围)
float64 further_aim_ang

# 更远预瞄点的高度状态 (0=Low, 1=Middle, 2=High)
int32 further_aim_height_state

# ==================== IMU数据（当前帧对应） ====================
# 当前帧对应的IMU yaw角度 (度, 左正右负)
float64 imu_yaw

# 当前帧对应的IMU pitch角度 (度, 上正下负)
float64 imu_pitch

# ==================== 开火条件 ====================
# 开火条件1：预瞄装甲板在可瞄准角度范围内
bool fire_condition_1

# ==================== 坐标转换矩阵 ====================
# T_gimbal_camera: 相机坐标系 → 云台坐标系 的转换矩阵 (3x3，按行存储)
float64[9] t_gimbal_camera

# R_body_gimbal: 云台坐标系 → 机架坐标系 的旋转矩阵 (3x3，按行存储) 
float64[9] r_body_gimbal

# ==================== EKF状态信息 ====================
# EKF是否已初始化
bool ekf_initialized

# EKF是否已收敛
bool ekf_converged

# 当前累计转角 ang (度)
float64 ang

# 当前角速度 omega (rad/s)
float64 omega

# 前哨站旋转中心位置（机架坐标系）
geometry_msgs/Point center_position

# 当前观测的装甲板高度状态 (0=Low, 1=Middle, 2=High)
int32 current_height_state

# EKF状态中的装甲板高度状态 height_k (0=Low, 1=Middle, 2=High)
int32 ekf_height_state

# ==================== 初始化状态信息 ====================
# 旋转方向: -1=顺时针, 0=静止, 1=逆时针
int32 rotation_direction

# 高度状态机是否已初始化
bool height_initialized

# θ观测是否已初始化（第一圈已完成）
bool theta_initialized

# 相位状态: 0=上升段, 1=平台区, 2=下降段
int32 phase_state

# ==================== 子弹飞行信息 ====================
# 估计的子弹飞行时间 (秒)
float64 bullet_flight_time

# 距离 (米)
float64 distance

# ==================== 调试信息 ====================
# 当前观测的theta (度，-60~60范围)
float64 observed_theta

# 当前观测的长宽比
float64 aspect_ratio

# 最近一次的测量残差
float64[4] measurement_residual

# 消息是否有效
bool valid
