# OutpostState.msg
# 前哨站状态预测结果
# 包含前哨站旋转中心、角速度等信息
# Requirements: 7.2

# 消息头（包含时间戳和坐标系）
std_msgs/Header header

# 旋转中心位置（米）
geometry_msgs/Point center

# 旋转中心速度（米/秒）
geometry_msgs/Vector3 velocity

# 旋转半径（米，约 0.275m）
float64 radius

# 当前旋转角度（弧度）
float64 theta

# 角速度（弧度/秒）
float64 omega

# 旋转方向: -1 = 顺时针, 1 = 逆时针, 0 = 未知
int32 direction

# 估计是否有效
bool valid
