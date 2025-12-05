# ArmorBBox.msg
# 单个装甲板检测结果
# 包含4个角点坐标、分类信息和置信度

# 装甲板四个角点坐标（按顺序：左上、右上、右下、左下）
armor_detector_ros2/Point2f[4] corners

# 装甲板中心点坐标
armor_detector_ros2/Point2f center

# 置信度分数 (0.0 - 1.0)
float32 confidence

# 颜色ID: 0=蓝色, 1=红色, 2=灰色, 3=紫色
int32 color_id

# 标签ID（数字分类结果）:
# 1-5: 步兵1-5号
# 0: 基地
# 6: 哨兵
# 7: 前哨站
int32 tag_id

# 边界框矩形（用于快速碰撞检测）
int32 rect_x      # 左上角X
int32 rect_y      # 左上角Y
int32 rect_width  # 宽度
int32 rect_height # 高度
