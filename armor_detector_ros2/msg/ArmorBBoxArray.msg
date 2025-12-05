# ArmorBBoxArray.msg
# 装甲板检测结果数组
# 包含一帧图像中检测到的所有装甲板

# 消息头，包含时间戳和坐标系信息
std_msgs/Header header

# 检测到的装甲板数组
armor_detector_ros2/ArmorBBox[] armors

# 检测耗时（毫秒）
float32 detection_time_ms
