# ArmorPoseArray.msg
# 装甲板位姿解算结果数组
# 包含一帧图像中所有装甲板的位姿

# 消息头（包含时间戳和坐标系）
std_msgs/Header header

# 解算得到的装甲板位姿数组
armor_detector_ros2/ArmorPose[] poses

# 解算耗时（毫秒）
float32 solve_time_ms
