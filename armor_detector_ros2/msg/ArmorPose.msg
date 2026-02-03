# ArmorPose.msg
# 装甲板位姿解算结果
# 包含装甲板在三维空间中的位置和姿态信息
#
# 坐标系说明：
# - 相机坐标系：X右、Y下、Z前（光轴方向）- OpenCV标准
# - 云台坐标系：X右、Y前、Z上（当yaw=pitch=0时与机架重合）
# - 机架坐标系（Body Frame）：固定在无人机机身，不随云台转动
#   * X轴：机架右方向
#   * Y轴：机架前方向（机头方向）
#   * Z轴：机架上方向（重力反方向）
#   * 右手系：X(右) × Y(前) = Z(上)
#
# IMU定义：
# - yaw: 向右为正（绕Z轴，从上往下看顺时针）
# - pitch: 向上为正（绕X轴，抬头为正）

# 消息头（包含时间戳和坐标系）
std_msgs/Header header

# ==================== 机架坐标系（主要输出） ====================
# 目标相对机架的 yaw 角（弧度）正值=目标在右边
float64 yaw

# 目标相对机架的 pitch 角（弧度）正值=目标在上边
float64 pitch

# 距离（米）
float64 distance

# 机架坐标系位置（米）
geometry_msgs/Point position

# 装甲板法向量的 yaw 角（弧度）
float64 armor_yaw

# 装甲板法向量的 pitch 角（弧度）
float64 armor_pitch

# ==================== 相机坐标系（原始PnP输出） ====================
# 相机坐标系位置（米）
geometry_msgs/Point position_cam

# PnP解算的旋转向量（用于重投影验证）
# 3元素: rvec[0], rvec[1], rvec[2]
float64[3] rvec

# ==================== 坐标系转换矩阵（用于逆变换） ====================
# T_gimbal_camera: 相机坐标系 → 云台坐标系 的转换矩阵 (3x3，按行存储)
# 相机: X右, Y下, Z前 → 云台: X右, Y前, Z上
# 逆变换: T_camera_gimbal = T_gimbal_camera^T (正交矩阵)
float64[9] t_gimbal_camera

# R_body_gimbal: 云台坐标系 → 机架坐标系 的旋转矩阵 (3x3，按行存储)
# 依赖于当前IMU的yaw和pitch角度
# 逆变换: R_gimbal_body = R_body_gimbal^T (正交矩阵)
float64[9] r_body_gimbal

# ==================== 4个角点3D位置（机架坐标系） ====================
# 角点顺序：左上(0)、左下(1)、右下(2)、右上(3)
geometry_msgs/Point[4] corners_3d

# ==================== 元信息 ====================
# 颜色ID: 0=蓝色, 1=红色, 2=灰色, 3=紫色
int32 color_id

# 标签ID（数字分类结果）
int32 tag_id

# 装甲板类型: 0=小装甲板, 1=大装甲板
int32 armor_type

# 解算是否有效
bool valid

# ==================== 图像空间信息 ====================
# 装甲板像素宽高比 (平均宽度 / 平均高度)
# 计算方法: 
#   平均宽度 = (|右上.x - 左上.x| + |右下.x - 左下.x|) / 2
#   平均高度 = (|左下.y - 左上.y| + |右下.y - 右上.y|) / 2
float64 aspect_ratio

# 装甲板中心的像素坐标 (u, v)
# 计算方法: 四个角点的平均值
float64 center_pixel_x
float64 center_pixel_y
