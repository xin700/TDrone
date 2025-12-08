# Command.msg
# 控制指令消息
# 用于发送给下位机的瞄准和开火指令

# 消息头（包含时间戳）
std_msgs/Header header

# 是否接管控制
bool control

# 是否开火
bool fire

# 目标 yaw 角（弧度）
float64 yaw

# 目标 pitch 角（弧度）
float64 pitch

# yaw 速度前馈（rad/s）
float64 yaw_velocity

# pitch 速度前馈（rad/s）
float64 pitch_velocity
