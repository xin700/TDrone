# GimbalCommand.msg
# 云台控制指令消息
# 包含目标yaw和pitch角度

# 消息头（包含时间戳）
std_msgs/Header header

# ==================== 目标角度（绝对值，度） ====================
# 目标yaw角度（度）：左正右负
float64 yaw

# 目标pitch角度（度）：上正下负
float64 pitch

# ==================== 状态信息 ====================
# 瞄准状态：0=初始化中, 1=追踪中, 2=切换中
int32 aiming_state

# 是否允许开火
bool fire_allowed

# ==================== 调试信息 ====================
# 相对yaw角度变化（度）
float64 delta_yaw

# 相对pitch角度变化（度）
float64 delta_pitch

# 目标距离（米）
float64 distance

# pitch重力补偿角（度）
float64 gravity_compensation
