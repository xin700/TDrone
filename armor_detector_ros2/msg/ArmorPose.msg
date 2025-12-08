# ArmorPose.msg
# 装甲板位姿解算结果
# 包含装甲板在三维空间中的位置和姿态信息

# 消息头（包含时间戳和坐标系）
std_msgs/Header header

# 相对云台的 yaw 角（弧度）
float64 yaw

# 相对云台的 pitch 角（弧度）
float64 pitch

# 距离（米）
float64 distance

# 世界坐标系位置（米）
geometry_msgs/Point position

# 装甲板朝向角（弧度）
float64 armor_yaw

# 颜色ID: 0=蓝色, 1=红色, 2=灰色, 3=紫色
int32 color_id

# 标签ID（数字分类结果）
int32 tag_id

# 装甲板类型: 0=小装甲板, 1=大装甲板
int32 armor_type

# 解算是否有效
bool valid
