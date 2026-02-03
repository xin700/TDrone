#!/usr/bin/env python3
"""Web可视化仪表盘 - ROS2 装甲板检测与预测系统 V4
优化版本：
1. 直接订阅 visualizer_v4 发布的已绘制图像，避免重复绘制
2. 保留原始数据订阅用于图表更新（aim_result、imu等）
3. 大幅降低Web节点的计算负担
4. 保持视频流和数据更新的独立性
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import QuaternionStamped
from armor_detector_ros2.msg import ArmorBBoxArray, ArmorPoseArray, OutpostAimResult
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from flask import Flask, render_template_string, Response, jsonify, request, send_file
from flask_socketio import SocketIO, emit
import threading
import time
import json
from collections import deque
import argparse
import os
import glob
import yaml

class WebVisualizerNode(Node):
    def __init__(self):
        super().__init__('web_visualizer_node')
        self.bridge = CvBridge()
        
        # 使用Best Effort QoS减少延迟
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 低延迟 QoS（用于需要实时更新的数据）
        qos_low_latency = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # 只保留最新的一条，减少延迟
        )
        
        # 订阅话题
        self.get_logger().info('开始创建订阅...')
        
        # 订阅 visualizer_v4 发布的已绘制图像（核心：避免重复绘制）
        self.image_sub = self.create_subscription(
            Image, '/visualizer_v4/image_visualized', self.image_callback, qos_best_effort)
        self.get_logger().info('已订阅: /visualizer_v4/image_visualized (已绘制图像)')
        
        # 订阅原始数据用于图表更新
        self.aim_sub = self.create_subscription(
            OutpostAimResult, '/predictor_v4/aim_result', self.aim_callback, qos_low_latency)
        self.get_logger().info('已订阅: /predictor_v4/aim_result (用于图表)')
        
        self.imu_sub = self.create_subscription(
            QuaternionStamped, '/imu/quaternion', self.imu_callback, qos_best_effort)
        self.get_logger().info('已订阅: /imu/quaternion (用于图表)')
        
        # 数据存储（使用缓存 + 时间戳，支持异步更新）
        self.latest_frame = None
        self.latest_aim_result = None  # 仅用于图表
        self.latest_imu = None  # 仅用于图表
        self.frame_lock = threading.Lock()
        
        # 不再需要 armors、poses 的数据存储和序列号（由 visualizer_v4 处理）
        # 仅保留 aim 的更新序列号用于图表更新检测
        self.aim_update_seq = 0
        
        # 预编码的JPEG（避免每次请求都编码）
        self.encoded_frame = None
        self.frame_seq = 0
        
        # 帧率控制
        self.last_frame_time = 0
        self.target_fps = 30  # 目标帧率
        self.frame_interval = 1.0 / self.target_fps
        
        # 调试计数器
        self.image_callback_count = 0
        self.aim_callback_count = 0
        # 不再需要 armor_callback_count 和 pose_callback_count
        
        # 时间序列数据（最近100个点）
        self.aim_history = {
            'time': deque(maxlen=100),
            'x': deque(maxlen=100), 'y': deque(maxlen=100), 'z': deque(maxlen=100),
            'theta': deque(maxlen=100), 'omega': deque(maxlen=100)
        }
        
        # IMU时间序列数据（最近300个点，显示更长时间范围）
        self.imu_history = {
            'time': deque(maxlen=100),
            'roll': deque(maxlen=100), 'pitch': deque(maxlen=100), 'yaw': deque(maxlen=100)
        }
        
        # IMU降采样计数器（每20个数据点记录1个，将200Hz降到10Hz）
        self.imu_sample_counter = 0
        self.imu_sample_rate = 20  # 每20个数据记录1个，匹配前端10Hz更新频率
        
        # 轨迹历史（最近200个点）
        self.trajectory_points = deque(maxlen=200)
        
        # 相机内参（默认值，将尝试从YAML加载）
        self.camera_matrix = np.array([
            [1280.0, 0, 640.0],
            [0, 1024.0, 512.0],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        # 尝试从 YAML 文件加载相机内参
        self._load_camera_intrinsics()
        
        self.get_logger().info('Web Visualizer Node 已启动')
    
    def _load_camera_intrinsics(self):
        """从 YAML 文件加载相机内参"""
        # 搜索相机内参配置文件的路径列表
        possible_paths = [
            # 优先从环境变量获取
            os.environ.get('CAMERA_INTRINSICS_PATH', ''),
            # 相对于脚本位置
            os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'camera_intrinsics.yaml'),
            # 常见的安装路径
            '/home/user/droneAim/TDrone/armor_detector_ros2/config/camera_intrinsics.yaml',
            os.path.expanduser('~/droneAim26/TDrone/armor_detector_ros2/config/camera_intrinsics.yaml'),
        ]
        
        for yaml_path in possible_paths:
            if not yaml_path or not os.path.exists(yaml_path):
                continue
            
            try:
                # 使用 OpenCV FileStorage 读取（与 C++ 端保持一致）
                fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
                if not fs.isOpened():
                    continue
                
                # 尝试读取不同格式的 camera_matrix
                camera_matrix = None
                for key in ['camera_matrix', 'K', 'intrinsic_matrix']:
                    node = fs.getNode(key)
                    if not node.empty():
                        camera_matrix = node.mat()
                        break
                
                if camera_matrix is not None and camera_matrix.shape == (3, 3):
                    self.camera_matrix = camera_matrix.astype(np.float64)
                    self.get_logger().info(f'相机内参已从YAML加载: {yaml_path}')
                    self.get_logger().info(f'  fx={self.camera_matrix[0,0]:.1f}, fy={self.camera_matrix[1,1]:.1f}, '
                                          f'cx={self.camera_matrix[0,2]:.1f}, cy={self.camera_matrix[1,2]:.1f}')
                    
                    # 尝试读取畸变系数
                    for key in ['distortion_coefficients', 'D', 'dist_coeffs']:
                        node = fs.getNode(key)
                        if not node.empty():
                            dist = node.mat()
                            if dist is not None:
                                self.dist_coeffs = dist.flatten().astype(np.float64)
                                break
                    
                    fs.release()
                    return  # 成功加载，退出
                
                fs.release()
            except Exception as e:
                self.get_logger().warn(f'加载相机内参失败 ({yaml_path}): {e}')
        
        self.get_logger().warn('未能从YAML加载相机内参，使用默认值')
        self.get_logger().warn(f'  默认值: fx={self.camera_matrix[0,0]:.1f}, fy={self.camera_matrix[1,1]:.1f}')
    
    def image_callback(self, msg):
        """图像回调 - 直接接收已绘制的可视化图像"""
        self.image_callback_count += 1
        
        if self.image_callback_count == 1:
            self.get_logger().info('=== 首次接收到已绘制图像！ ===')
        if self.image_callback_count % 100 == 0:
            self.get_logger().info(f'已接收 {self.image_callback_count} 帧已绘制图像')
        
        try:
            # 直接转换图像，无需再次绘制标注
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 预编码JPEG（在回调中完成，减少请求延迟）
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            encoded = buffer.tobytes()
            
            with self.frame_lock:
                self.latest_frame = frame
                self.encoded_frame = encoded
                self.frame_seq += 1
        except Exception as e:
            import traceback
            self.get_logger().error(f'图像处理错误: {e}\\n{traceback.format_exc()}')
    
    def aim_callback(self, msg):
        """预测结果回调 - 仅用于图表数据更新"""
        self.aim_callback_count += 1
        with self.frame_lock:
            self.latest_aim_result = msg
            self.aim_update_seq += 1
        
        # 记录时间序列数据（用于图表显示）
        t = time.time()
        self.aim_history['time'].append(t)
        self.aim_history['x'].append(msg.aim_position.x)
        self.aim_history['y'].append(msg.aim_position.y)
        self.aim_history['z'].append(msg.aim_position.z)
        self.aim_history['theta'].append(msg.observed_theta)
        self.aim_history['omega'].append(msg.omega)
    
    def imu_callback(self, msg):
        """IMU数据回调"""
        self.latest_imu = msg
        
        # 降采样：每imu_sample_rate个数据记录1个
        self.imu_sample_counter += 1
        if self.imu_sample_counter < self.imu_sample_rate:
            return  # 跳过此次记录
        self.imu_sample_counter = 0  # 重置计数器
        
        # 记录时间序列数据
        t = time.time()
        self.imu_history['time'].append(t)
        
        # 四元数转欧拉角（ZYX顺序：yaw-pitch-roll）
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w
        )
        self.imu_history['roll'].append(math.degrees(roll))
        self.imu_history['pitch'].append(math.degrees(pitch))
        self.imu_history['yaw'].append(math.degrees(yaw))
    
    def body_to_camera(self, p_body, aim_msg):
        """机架坐标系转相机坐标系"""
        # 提取旋转矩阵 R_body_gimbal (3x3)
        R_bg = np.array(aim_msg.r_body_gimbal).reshape(3, 3)
        # 提取转换矩阵 T_gimbal_camera (3x3)
        T_gc = np.array(aim_msg.t_gimbal_camera).reshape(3, 3)
        
        # 相机相对云台旋转中心的偏移（云台坐标系）
        # 必须与 pose_solver.cpp 中的 camera_offset_gimbal 保持一致！
        camera_offset_gimbal = np.array([0.0, 0.0, -0.10])
        
        # 机架 -> 云台 -> 相机
        p_gimbal = R_bg.T @ p_body
        # 减去相机偏移后再转换到相机坐标系
        p_gimbal_no_offset = p_gimbal - camera_offset_gimbal
        p_cam = T_gc.T @ p_gimbal_no_offset
        return p_cam
    
    def quaternion_to_euler(self, x, y, z, w):
        """四元数转欧拉角（ZYX顺序：yaw-pitch-roll）"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def project_point(self, p_cam):
        """将3D点投影到图像平面"""
        if p_cam[2] < 0.1:
            return None, False
        
        # 投影
        p_2d = self.camera_matrix @ p_cam
        u = int(p_2d[0] / p_2d[2])
        v = int(p_2d[1] / p_2d[2])
        return (u, v), True
    
    def draw_visualization(self, frame):
        """绘制完整的可视化信息（类似visualizer_node_v4）
        
        【已废弃】此函数不再使用，所有绘制工作由 visualizer_v4 节点完成
        Web前端直接订阅 /visualizer_v4/image_visualized 话题获取已绘制图像
        
        保留此函数仅为向后兼容，实际不会被调用
        """
        # 直接返回原图，不再进行任何绘制
        return frame
        
        # 构建pose字典，方便通过像素坐标匹配
        pose_dict = {}
        if poses_snapshot is not None:
            for pose in poses_snapshot.poses:
                # 使用中心像素坐标作为key（四舍五入到整数）
                key = (int(pose.center_pixel_x), int(pose.center_pixel_y))
                pose_dict[key] = pose
        
        # 绘制装甲板
        for armor in armors_snapshot.armors:
            # 颜色映射：0=蓝色, 1=红色, 2=灰色, 3=紫色
            if armor.color_id == 0:  # BLUE
                color = (255, 0, 0)
            elif armor.color_id == 1:  # RED
                color = (0, 0, 255)
            else:
                color = (128, 128, 128)
            
            # 绘制四个角点
            for corner in armor.corners:
                pt = (int(corner.x), int(corner.y))
                cv2.circle(frame, pt, 4, color, -1)
            
            # 绘制中心点
            center = (int(armor.center.x), int(armor.center.y))
            cv2.circle(frame, center, 5, color, -1)
            
            # 标注 tag_id
            label = f"ID-{armor.tag_id}"
            cv2.putText(frame, label, (center[0] - 30, center[1] - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # 查找对应的位姿数据并绘制机架坐标系坐标
            matched_pose = None
            min_dist = 50  # 最大匹配距离（像素）
            for (px, py), pose in pose_dict.items():
                dist = abs(px - center[0]) + abs(py - center[1])
                if dist < min_dist:
                    min_dist = dist
                    matched_pose = pose
            
            if matched_pose is not None and matched_pose.valid:
                # 绘制机架坐标系坐标 (x, y, z) - 黄色
                pos = matched_pose.position
                coord_text = f"Body:({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
                # 在中心点下方显示坐标
                cv2.putText(frame, coord_text, (center[0] - 80, center[1] + 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                
                # 绘制相机坐标系坐标 (x, y, z) - 绿色
                pos_cam = matched_pose.position_cam
                coord_cam_text = f"Cam:({pos_cam.x:.2f}, {pos_cam.y:.2f}, {pos_cam.z:.2f})"
                cv2.putText(frame, coord_cam_text, (center[0] - 80, center[1] + 42),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                
                # 显示距离
                dist_text = f"d={matched_pose.distance:.2f}m"
                cv2.putText(frame, dist_text, (center[0] - 40, center[1] + 58),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # 绘制预测器状态信息（左上角）
        if aim_snapshot is not None:
            aim = aim_snapshot
            y = 30
            line_height = 22
            
            # 标题
            cv2.putText(frame, "Predictor V4", (20, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            y += line_height
            
            # === 初始化状态区域 ===
            # 旋转方向
            dir_str = "CW" if aim.rotation_direction == -1 else ("CCW" if aim.rotation_direction == 1 else "?")
            dir_color = (0, 255, 0) if aim.rotation_direction != 0 else (128, 128, 128)
            cv2.putText(frame, f"Dir: {dir_str}", (20, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, dir_color, 1)
            
            # 高度初始化
            h_init_str = "OK" if aim.height_initialized else "..."
            h_init_color = (0, 255, 0) if aim.height_initialized else (128, 128, 128)
            cv2.putText(frame, f"H-Init: {h_init_str}", (120, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, h_init_color, 1)
            
            # Theta初始化
            t_init_str = "OK" if aim.theta_initialized else "..."
            t_init_color = (0, 255, 0) if aim.theta_initialized else (128, 128, 128)
            cv2.putText(frame, f"T-Init: {t_init_str}", (220, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, t_init_color, 1)
            y += line_height
            
            # EKF状态
            if aim.ekf_initialized:
                ekf_status = "CONV" if aim.ekf_converged else "RUN"
                color = (0, 255, 0) if aim.ekf_converged else (0, 200, 255)
            else:
                ekf_status = "WAIT"
                color = (128, 128, 128)
            cv2.putText(frame, f"EKF: {ekf_status}", (20, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # 相位状态
            phase_names = {0: 'RISE', 1: 'PLAT', 2: 'FALL'}
            phase_str = phase_names.get(aim.phase_state, '?')
            cv2.putText(frame, f"Phase: {phase_str}", (120, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            y += line_height
            
            # === 观测数据区域 ===
            # 观测角theta（度）
            cv2.putText(frame, f"theta: {aim.observed_theta:.1f}", (20, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            
            # 角速度omega
            cv2.putText(frame, f"omega: {aim.omega:.3f}", (120, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            y += line_height
            
            # 高度状态（观测）
            height_names = {0: 'LOW', 1: 'MID', 2: 'HIGH'}
            height_colors = {0: (255, 100, 100), 1: (0, 255, 0), 2: (0, 200, 255)}
            if aim.height_initialized:
                obs_h = aim.current_height_state
                h_name = height_names.get(obs_h, '?')
                h_color = height_colors.get(obs_h, (255, 255, 255))
                cv2.putText(frame, f"Obs H: {h_name}", (20, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, h_color, 2)
            else:
                cv2.putText(frame, "Obs H: ---", (20, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
            y += line_height
            
            # 预瞄点位置
            cv2.putText(frame, f"Aim: ({aim.aim_position.x:.2f}, {aim.aim_position.y:.2f}, {aim.aim_position.z:.2f})",
                       (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            y += line_height
            
            # 开火条件
            fire_str = "FIRE" if aim.fire_condition_1 else "WAIT"
            fire_color = (0, 255, 0) if aim.fire_condition_1 else (0, 0, 255)
            cv2.putText(frame, f"Fire: {fire_str}", (20, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, fire_color, 2)
        
        # 绘制预瞄点（传递快照）
        self.draw_aim_point(frame, aim_snapshot)
        
        # 绘制轨迹
        self.draw_trajectory(frame)
        
        # 绘制前哨站模型
        # self.draw_outpost_model(frame)  # 已禁用
        
        # 右上角调试信息 - 数据同步监控
        h, w = frame.shape[:2]
        debug_x = w - 250
        debug_y = 30
        debug_line_height = 18
        
        cv2.putText(frame, "=== Sync Debug ===", (debug_x, debug_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        debug_y += debug_line_height
        
        # 回调计数（帧数统计）
        cv2.putText(frame, f"Video: {getattr(self, 'image_callback_count', 0)}", (debug_x, debug_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        debug_y += debug_line_height
        
        cv2.putText(frame, f"Aim: {self.aim_callback_count}", (debug_x, debug_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        debug_y += debug_line_height
        
        cv2.putText(frame, f"Armor: {getattr(self, 'armor_callback_count', 0)}", (debug_x, debug_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        debug_y += debug_line_height
        
        cv2.putText(frame, f"Pose: {getattr(self, 'pose_callback_count', 0)}", (debug_x, debug_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        debug_y += debug_line_height
        
        # 显示数据更新状态（NEW 表示有新数据）
        armors_status = "NEW" if self.armors_update_seq != self.last_drawn_armors_seq else "OLD"
        aim_status = "NEW" if self.aim_update_seq != self.last_drawn_aim_seq else "OLD"
        armors_color = (0, 255, 0) if armors_status == "NEW" else (100, 100, 100)
        aim_color = (0, 255, 0) if aim_status == "NEW" else (100, 100, 100)
        
        cv2.putText(frame, f"Armor: {armors_status}", (debug_x, debug_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, armors_color, 1)
        debug_y += debug_line_height
        
        cv2.putText(frame, f"Aim: {aim_status}", (debug_x, debug_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, aim_color, 1)
        debug_y += debug_line_height
        
        # 显示时间戳对齐情况（用于诊断数据同步）
        if aim_snapshot is not None and hasattr(aim_snapshot, 'header'):
            aim_stamp = aim_snapshot.header.stamp
            aim_sec = aim_stamp.sec + aim_stamp.nanosec * 1e-9
            # 计算aim数据的"年龄"
            if hasattr(self, '_last_aim_sec'):
                dt = aim_sec - self._last_aim_sec
                cv2.putText(frame, f"Aim dt: {dt*1000:.1f}ms", (debug_x, debug_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                debug_y += debug_line_height
            self._last_aim_sec = aim_sec
        
        return frame
    
    def draw_aim_point(self, frame, aim_result=None):
        """绘制预瞄点
        
        Args:
            frame: 图像帧
            aim_result: 预测结果快照（如果为None则使用self.latest_aim_result）
        """
        if aim_result is None:
            with self.frame_lock:
                aim_result = self.latest_aim_result
        
        if aim_result is None:
            return
        
        aim = aim_result
        p_body = np.array([aim.aim_position.x, aim.aim_position.y, aim.aim_position.z])
        p_cam = self.body_to_camera(p_body, aim)
        
        uv, valid = self.project_point(p_cam)
        if not valid or uv is None:
            return
        
        u, v = uv
        if u < 0 or u >= frame.shape[1] or v < 0 or v >= frame.shape[0]:
            return
        
        # 根据开火条件选择样式
        if aim.fire_condition_1:
            # 绿色十字准星 + 圆圈
            color = (0, 255, 0)
            size = 30
            cv2.line(frame, (u - size, v), (u + size, v), color, 2)
            cv2.line(frame, (u, v - size), (u, v + size), color, 2)
            cv2.circle(frame, (u, v), size, color, 2)
            cv2.circle(frame, (u, v), 8, color, -1)
        else:
            # 橙色点
            color = (0, 165, 255)
            cv2.circle(frame, (u, v), 12, color, -1)
            cv2.circle(frame, (u, v), 15, color, 2)
        
        # 标注距离
        dist_text = f"{p_cam[2]:.2f}m"
        cv2.putText(frame, dist_text, (u + 20, v + 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 标注瞄准高度
        height_names = {0: 'L', 1: 'M', 2: 'H'}
        height_colors = {0: (255, 100, 100), 1: (0, 255, 0), 2: (0, 200, 255)}
        height_str = height_names.get(aim.aim_height_state, '?')
        height_color = height_colors.get(aim.aim_height_state, (255, 255, 255))
        cv2.putText(frame, height_str, (u + 20, v + 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, height_color, 2)
    
    def draw_trajectory(self, frame):
        """绘制历史轨迹"""
        if len(self.trajectory_points) < 2:
            return
        
        projected_points = []
        for p_cam in self.trajectory_points:
            uv, valid = self.project_point(p_cam)
            if valid and uv is not None:
                u, v = uv
                if 0 <= u < frame.shape[1] and 0 <= v < frame.shape[0]:
                    projected_points.append((u, v))
        
        # 绘制轨迹线（颜色渐变：蓝→黄）
        for i in range(1, len(projected_points)):
            alpha = i / len(projected_points)
            color = (int(255 * (1 - alpha)), int(255 * alpha), 0)
            cv2.line(frame, projected_points[i-1], projected_points[i], color, 1)
    
    def draw_outpost_model(self, frame):
        """绘制前哨站三装甲板模型"""
        if self.latest_aim_result is None:
            return
        
        aim = self.latest_aim_result
        
        # 前哨站几何参数
        radius = 0.2767
        height_diff = 0.10
        phase_spacing = 2.0 * np.pi / 3.0
        
        # 旋转中心
        cx, cy, cz = aim.center_position.x, aim.center_position.y, aim.center_position.z
        theta = np.deg2rad(aim.ang)
        
        # 三个装甲板的高度和相位偏移
        height_offsets = [-height_diff, 0.0, height_diff]  # L, M, H
        phase_offsets = [0.0, -phase_spacing, -2.0 * phase_spacing]
        colors = [(255, 100, 100), (0, 255, 0), (0, 200, 255)]  # L=蓝, M=绿, H=橙
        names = ['L', 'M', 'H']
        
        # 绘制旋转中心
        center_body = np.array([cx, cy, cz])
        center_cam = self.body_to_camera(center_body, aim)
        center_uv, valid = self.project_point(center_cam)
        
        if valid and center_uv is not None:
            u, v = center_uv
            if 0 <= u < frame.shape[1] and 0 <= v < frame.shape[0]:
                cv2.drawMarker(frame, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
                cv2.putText(frame, 'C', (u + 10, v - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # 绘制三个装甲板位置
        for i in range(3):
            angle = theta + phase_offsets[i]
            armor_x = cx + radius * np.cos(angle)
            armor_y = cy + radius * np.sin(angle)
            armor_z = cz + height_offsets[i]
            
            armor_body = np.array([armor_x, armor_y, armor_z])
            armor_cam = self.body_to_camera(armor_body, aim)
            armor_uv, valid = self.project_point(armor_cam)
            
            if valid and armor_uv is not None:
                u, v = armor_uv
                if 0 <= u < frame.shape[1] and 0 <= v < frame.shape[0]:
                    cv2.circle(frame, (u, v), 8, colors[i], -1)
                    cv2.circle(frame, (u, v), 10, colors[i], 2)
                    cv2.putText(frame, names[i], (u + 12, v + 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, colors[i], 2)
    
    def get_frame_jpeg(self):
        """获取JPEG编码的帧"""
        with self.frame_lock:
            if self.latest_frame is None:
                # 返回黑色占位图
                placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder, "Waiting for image...", (200, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                _, buffer = cv2.imencode('.jpg', placeholder)
            else:
                _, buffer = cv2.imencode('.jpg', self.latest_frame, 
                                        [cv2.IMWRITE_JPEG_QUALITY, 85])
            return buffer.tobytes()
    
    def get_statistics(self):
        """获取统计数据"""
        if self.latest_armors is None:
            return {'count': 0}
        
        stats = {'count': len(self.latest_armors.armors), 'by_color': {}}
        color_names = {0: 'BLUE', 1: 'RED', 2: 'GRAY', 3: 'PURPLE'}
        for armor in self.latest_armors.armors:
            color_name = color_names.get(armor.color_id, 'UNKNOWN')
            stats['by_color'][color_name] = stats['by_color'].get(color_name, 0) + 1
        
        return stats
    
    def get_predictor_state(self):
        """获取预测器状态"""
        if self.latest_aim_result is None:
            return None
        
        aim = self.latest_aim_result
        return {
            'ekf_converged': aim.ekf_converged,
            'theta': float(aim.observed_theta),
            'omega': float(aim.omega),
            'rotation_direction': int(aim.rotation_direction),
            'fire_condition': aim.fire_condition_1,
            'aim_x': float(aim.aim_position.x),
            'aim_y': float(aim.aim_position.y),
            'aim_z': float(aim.aim_position.z)
        }
    
    def get_time_series(self):
        """获取时间序列数据"""
        if len(self.aim_history['time']) == 0:
            return None
        
        # 计算相对时间（秒）
        times = list(self.aim_history['time'])
        base_time = times[0]
        rel_times = [t - base_time for t in times]
        
        return {
            'time': rel_times,
            'aim_x': list(self.aim_history['x']),
            'aim_y': list(self.aim_history['y']),
            'aim_z': list(self.aim_history['z']),
            'theta': list(self.aim_history['theta']),
            'omega': list(self.aim_history['omega'])
        }
    
    def get_imu_state(self):
        """获取当前IMU状态"""
        if self.latest_imu is None:
            return None
        
        roll, pitch, yaw = self.quaternion_to_euler(
            self.latest_imu.quaternion.x,
            self.latest_imu.quaternion.y,
            self.latest_imu.quaternion.z,
            self.latest_imu.quaternion.w
        )
        
        return {
            'quaternion': {
                'w': float(self.latest_imu.quaternion.w),
                'x': float(self.latest_imu.quaternion.x),
                'y': float(self.latest_imu.quaternion.y),
                'z': float(self.latest_imu.quaternion.z)
            },
            'euler': {
                'roll': float(math.degrees(roll)),
                'pitch': float(math.degrees(pitch)),
                'yaw': float(math.degrees(yaw))
            }
        }
    
    def get_imu_time_series(self):
        """获取IMU时间序列数据"""
        if len(self.imu_history['time']) == 0:
            return None
        
        # 计算相对时间（秒）
        times = list(self.imu_history['time'])
        base_time = times[0]
        rel_times = [t - base_time for t in times]
        
        return {
            'time': rel_times,

            'roll': list(self.imu_history['roll']),
            'pitch': list(self.imu_history['pitch']),
            'yaw': list(self.imu_history['yaw'])
        }


# Flask应用
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")
ros_node = None

@app.route('/frame')
def get_single_frame():
    """获取单帧图像 - 直接返回预编码数据"""
    if ros_node is None:
        return Response(status=503)
    
    with ros_node.frame_lock:
        encoded = ros_node.encoded_frame
        seq = ros_node.frame_seq
    
    if encoded is None:
        return Response(status=204)
    
    response = Response(encoded, mimetype='image/jpeg')
    response.headers['X-Frame-Seq'] = str(seq)
    response.headers['Cache-Control'] = 'no-cache'
    return response

# 保留MJPEG流作为备选
def gen_frames():
    """生成视频流 - MJPEG模式（备选）"""
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, 80]
    last_frame_seq = -1
    
    while True:
        if ros_node is None:
            time.sleep(0.05)
            continue
        
        with ros_node.frame_lock:
            frame = ros_node.latest_frame
            current_seq = ros_node.frame_seq
            # 使用预编码的帧（如果可用）
            encoded = ros_node.encoded_frame
        
        # 使用frame_seq来检测新帧
        if frame is not None and current_seq != last_frame_seq:
            last_frame_seq = current_seq
            # 如果有预编码的帧直接使用，否则编码
            if encoded is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + encoded + b'\r\n')
            else:
                _, buffer = cv2.imencode('.jpg', frame, encode_params)
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        
        time.sleep(0.02)  # 50fps检查频率

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/stats')
def api_stats():
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'})
    return jsonify(ros_node.get_statistics())

@app.route('/api/predictor')
def api_predictor():
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'})
    state = ros_node.get_predictor_state()
    return jsonify(state if state else {})

@app.route('/api/timeseries')
def api_timeseries():
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'})
    data = ros_node.get_time_series()
    return jsonify(data if data else {})

@app.route('/api/imu')
def api_imu():
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'})
    state = ros_node.get_imu_state()
    return jsonify(state if state else {})

@app.route('/api/imu/timeseries')
def api_imu_timeseries():
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'})
    data = ros_node.get_imu_time_series()
    return jsonify(data if data else {})

@app.route('/api/debug')
def api_debug():
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'})
    return jsonify({
        'image_callback_count': ros_node.image_callback_count,
        'armor_callback_count': ros_node.armor_callback_count,
        'pose_callback_count': ros_node.pose_callback_count,
        'latest_frame': 'Yes' if ros_node.latest_frame is not None else 'No',
        'latest_armors': 'Yes' if ros_node.latest_armors is not None else 'No',
        'latest_poses': 'Yes' if ros_node.latest_poses is not None else 'No',
        'latest_imu': 'Yes' if ros_node.latest_imu is not None else 'No'
    })

HTML_TEMPLATE = '''TRUNCATED_FOR_BREVITY'''

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>装甲板检测可视化</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.js"></script>
    <script src="https://cdn.socket.io/4.5.4/socket.io.min.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, sans-serif;
            background: #f5f5f5;
            color: #333;
        }
        .header {
            background: white;
            padding: 15px 30px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            margin-bottom: 20px;
        }
        .header h1 {
            font-size: 24px;
            font-weight: 600;
            color: #333;
        }
        .header .subtitle {
            font-size: 14px;
            color: #666;
            margin-top: 5px;
        }
        .container {
            max-width: 1800px;
            margin: 0 auto;
            padding: 0 20px;
        }
        .row {
            display: flex;
            gap: 20px;
            margin-bottom: 20px;
        }
        .panel {
            background: white;
            border-radius: 4px;
            padding: 20px;
            box-shadow: 0 1px 3px rgba(0,0,0,0.1);
        }
        .panel-title {
            font-size: 16px;
            font-weight: 600;
            color: #333;
            margin-bottom: 15px;
            padding-bottom: 10px;
            border-bottom: 1px solid #e0e0e0;
        }
        .video-panel {
            flex: 3;
        }
        .stats-panel {
            flex: 1;
            min-width: 250px;
        }
        #videoStream {
            width: 100%;
            height: auto;
            border-radius: 4px;
            display: block;
        }
        .stat-item {
            padding: 8px 0;
            border-bottom: 1px solid #f0f0f0;
        }
        .stat-label {
            font-size: 13px;
            color: #666;
        }
        .stat-value {
            font-size: 18px;
            font-weight: 600;
            color: #333;
            margin-top: 2px;
        }
        .status-badge {
            display: inline-block;
            padding: 4px 12px;
            border-radius: 12px;
            font-size: 12px;
            font-weight: 600;
        }
        .badge-success { background: #e8f5e9; color: #2e7d32; }
        .badge-warning { background: #fff3e0; color: #f57c00; }
        .badge-error { background: #ffebee; color: #c62828; }
        .chart-container {
            position: relative;
            height: 300px;
            margin-top: 10px;
        }
        .full-width { flex: 1; }
        .video-list {
            max-height: 400px;
            overflow-y: auto;
        }
        .video-item {
            padding: 10px;
            margin: 5px 0;
            background: #f9f9f9;
            border-radius: 4px;
            cursor: pointer;
            transition: background 0.2s;
        }
        .video-item:hover {
            background: #e8e8e8;
        }
        .video-item-name {
            font-size: 14px;
            color: #333;
            font-weight: 500;
        }
        .video-item-info {
            font-size: 12px;
            color: #666;
            margin-top: 3px;
        }
        .video-player {
            margin-top: 15px;
        }
        #videoPlayer {
            width: 100%;
            border-radius: 4px;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>装甲板检测预测可视化</h1>
        <div class="subtitle">可视化工具</div>
    </div>
    
    <div class="container">
        <!-- 第一行：实时画面 + 预测器状态 + IMU状态 -->
        <div class="row">
            <div class="panel video-panel">
                <div class="panel-title">视频流 <span id="fpsDisplay" style="font-weight:normal;color:#666;font-size:12px;"></span></div>
                <img id="videoStream" alt="Video Stream" style="background:#000;">
            </div>
            <div class="panel stats-panel">
                <div class="panel-title">检测</div>
                <div class="stat-item">
                    <div class="stat-label">装甲板数量</div>
                    <div class="stat-value" id="armorCount">0</div>
                </div>
                <div class="stat-item" id="colorStats"></div>
                
                <div class="panel-title" style="margin-top: 20px;">预测器状态</div>
                <div class="stat-item">
                    <div class="stat-label">EKF状态</div>
                    <div id="ekfStatus">-</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">观测角 θ (deg)</div>
                    <div class="stat-value" id="theta">-</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">角速度 ω (rad/s)</div>
                    <div class="stat-value" id="omega">-</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">开火条件</div>
                    <div id="fireCond">-</div>
                </div>
                
                <div class="panel-title" style="margin-top: 20px;">IMU 姿态</div>
                <div class="stat-item">
                    <div class="stat-label">Roll (deg)</div>
                    <div class="stat-value" id="imuRoll">-</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">Pitch (deg)</div>
                    <div class="stat-value" id="imuPitch">-</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">Yaw (deg)</div>
                    <div class="stat-value" id="imuYaw">-</div>
                </div>
            </div>
        </div>
        
        <!-- 第二行：IMU 欧拉角曲线 -->
        <div class="row">
            <div class="panel full-width">
                <div class="panel-title">IMU 姿态角 (Roll, Pitch, Yaw)</div>
                <div class="chart-container">
                    <canvas id="imuEulerChart"></canvas>
                </div>
            </div>
        </div>
        
        <!-- 第三行：预瞄坐标曲线 -->
        <div class="row">
            <div class="panel full-width">
                <div class="panel-title">预瞄坐标 (X, Y, Z)</div>
                <div class="chart-container">
                    <canvas id="aimChart"></canvas>
                </div>
            </div>
        </div>
        
        <!-- 第四行：角度和角速度曲线 -->
        <div class="row">
            <div class="panel" style="flex: 1;">
                <div class="panel-title">观测角 θ (deg)</div>
                <div class="chart-container">
                    <canvas id="thetaChart"></canvas>
                </div>
            </div>
            <div class="panel" style="flex: 1;">
                <div class="panel-title">角速度 ω (rad/s)</div>
                <div class="chart-container">
                    <canvas id="omegaChart"></canvas>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        // 图表配置
        const commonOptions = {
            responsive: true,
            maintainAspectRatio: false,
            plugins: { legend: { position: 'top' } },
            scales: {
                x: { title: { display: true, text: '时间 (s)' } },
                y: { title: { display: true, text: '值' } }
            }
        };
        
        // 初始化图表
        const aimChart = new Chart(document.getElementById('aimChart'), {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    { label: 'X (m)', data: [], borderColor: '#f44336', backgroundColor: 'rgba(244, 67, 54, 0.1)', borderWidth: 2 },
                    { label: 'Y (m)', data: [], borderColor: '#4caf50', backgroundColor: 'rgba(76, 175, 80, 0.1)', borderWidth: 2 },
                    { label: 'Z (m)', data: [], borderColor: '#2196f3', backgroundColor: 'rgba(33, 150, 243, 0.1)', borderWidth: 2 }
                ]
            },
            options: commonOptions
        });
        
        const thetaChart = new Chart(document.getElementById('thetaChart'), {
            type: 'line',
            data: {
                labels: [],
                datasets: [{ label: 'θ', data: [], borderColor: '#ff9800', backgroundColor: 'rgba(255, 152, 0, 0.1)', borderWidth: 2 }]
            },
            options: commonOptions
        });
        
        const omegaChart = new Chart(document.getElementById('omegaChart'), {
            type: 'line',
            data: {
                labels: [],
                datasets: [{ label: 'ω', data: [], borderColor: '#9c27b0', backgroundColor: 'rgba(156, 39, 176, 0.1)', borderWidth: 2 }]
            },
            options: commonOptions
        });
        
        // IMU 欧拉角图表
        const imuEulerChart = new Chart(document.getElementById('imuEulerChart'), {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    { label: 'Roll (deg)', data: [], borderColor: '#e91e63', backgroundColor: 'rgba(233, 30, 99, 0.1)', borderWidth: 2 },
                    { label: 'Pitch (deg)', data: [], borderColor: '#00bcd4', backgroundColor: 'rgba(0, 188, 212, 0.1)', borderWidth: 2 },
                    { label: 'Yaw (deg)', data: [], borderColor: '#8bc34a', backgroundColor: 'rgba(139, 195, 74, 0.1)', borderWidth: 2 }
                ]
            },
            options: commonOptions
        });
        
        // ========== 视频帧拉取模式 ==========
        const videoImg = document.getElementById('videoStream');
        const fpsDisplay = document.getElementById('fpsDisplay');
        let frameCount = 0;
        let lastFpsTime = performance.now();
        let isLoading = false;
        let lastSeq = -1;
        
        function fetchFrame() {
            if (isLoading) return;
            isLoading = true;
            
            const img = new Image();
            img.onload = () => {
                videoImg.src = img.src;
                frameCount++;
                isLoading = false;
                // 立即请求下一帧，不等待
                setTimeout(fetchFrame, 0);
            };
            img.onerror = () => {
                isLoading = false;
                setTimeout(fetchFrame, 30);
            };
            // 添加时间戳防止缓存
            img.src = '/frame?' + Date.now();
        }
        
        // 启动帧拉取
        fetchFrame();
        
        // FPS显示
        setInterval(() => {
            const now = performance.now();
            const elapsed = (now - lastFpsTime) / 1000;
            const fps = frameCount / elapsed;
            fpsDisplay.textContent = `(${fps.toFixed(1)} FPS)`;
            frameCount = 0;
            lastFpsTime = now;
        }, 1000);
        
        // HTTP轮询更新数据（200ms间隔，减少请求频率）
        setInterval(() => {
            // 更新统计信息
            fetch('/api/stats').then(r => r.json()).then(data => {
                document.getElementById('armorCount').textContent = data.count || 0;
                const colorDiv = document.getElementById('colorStats');
                if (data.by_color) {
                    let html = '';
                    for (const [color, count] of Object.entries(data.by_color)) {
                        html += `<div style="margin: 5px 0;"><span style="color:#666;">${color}:</span> <strong>${count}</strong></div>`;
                    }
                    colorDiv.innerHTML = html;
                }
            }).catch(() => {});
            
            // 更新预测器状态
            fetch('/api/predictor').then(r => r.json()).then(data => {
                if (data.ekf_converged !== undefined) {
                    const ekfBadge = data.ekf_converged 
                        ? '<span class="status-badge badge-success">已收敛</span>'
                        : '<span class="status-badge badge-warning">初始化</span>';
                    document.getElementById('ekfStatus').innerHTML = ekfBadge;
                    
                    document.getElementById('theta').textContent = data.theta ? data.theta.toFixed(1) : '-';
                    document.getElementById('omega').textContent = data.omega ? data.omega.toFixed(3) : '-';
                    
                    const fireBadge = data.fire_condition
                        ? '<span class="status-badge badge-success">满足</span>'
                        : '<span class="status-badge badge-error">不满足</span>';
                    document.getElementById('fireCond').innerHTML = fireBadge;
                }
            }).catch(() => {});
            
            // 更新IMU状态
            fetch('/api/imu').then(r => r.json()).then(data => {
                if (data && data.euler) {
                    document.getElementById('imuRoll').textContent = data.euler.roll.toFixed(2);
                    document.getElementById('imuPitch').textContent = data.euler.pitch.toFixed(2);
                    document.getElementById('imuYaw').textContent = data.euler.yaw.toFixed(2);
                }
            }).catch(() => {});
        }, 200);  // 200ms = 5Hz更新状态
        
        // 图表数据更新（较低频率）
        setInterval(() => {
            // 更新预瞄坐标曲线
            fetch('/api/timeseries').then(r => r.json()).then(data => {
                if (data && data.time && data.time.length > 0) {
                    const labels = data.time.map(t => t.toFixed(2));
                    
                    aimChart.data.labels = labels;
                    aimChart.data.datasets[0].data = data.aim_x;
                    aimChart.data.datasets[1].data = data.aim_y;
                    aimChart.data.datasets[2].data = data.aim_z;
                    aimChart.update('none');
                    
                    thetaChart.data.labels = labels;
                    thetaChart.data.datasets[0].data = data.theta;
                    thetaChart.update('none');
                    
                    omegaChart.data.labels = labels;
                    omegaChart.data.datasets[0].data = data.omega;
                    omegaChart.update('none');
                }
            }).catch(() => {});
            
            // 更新IMU欧拉角曲线图
            fetch('/api/imu/timeseries').then(r => r.json()).then(data => {
                if (data && data.time && data.time.length > 0) {
                    const labels = data.time.map(t => t.toFixed(2));
                    
                    imuEulerChart.data.labels = labels;
                    imuEulerChart.data.datasets[0].data = data.roll;
                    imuEulerChart.data.datasets[1].data = data.pitch;
                    imuEulerChart.data.datasets[2].data = data.yaw;
                    imuEulerChart.update('none');
                }
            }).catch(() => {});
        }, 500);  // 500ms = 2Hz更新图表
    </script>
</body>
</html>
'''

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

def ros_spin():
    """ROS spin线程"""
    from rclpy.executors import MultiThreadedExecutor
    print('[DEBUG] ROS spin线程已启动')
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    print('[DEBUG] Executor已添加节点，开始spin...')
    try:
        executor.spin()
    finally:
        print('[DEBUG] Executor停止')
        executor.shutdown()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5000, help='Flask服务端口')
    args = parser.parse_args()
    
    global ros_node
    rclpy.init()
    ros_node = WebVisualizerNode()
    
    # 等待一下让ROS节点完全初始化
    time.sleep(1)
    
    # 启动ROS spin线程
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    print('[INFO] ROS spin线程已启动')
    
    time.sleep(1)  # 等待线程初始化
    
    print(f"[INFO] Web可视化服务已启动（优化版 V3）")
    print(f"[INFO] 访问地址: http://0.0.0.0:{args.port}")
    print(f"[INFO] 新功能: 机架坐标系坐标显示 | 预瞄点/轨迹绘制")
    
    # 使用gevent或直接Flask
    try:
        from gevent import pywsgi
        print("[INFO] 使用 gevent 高性能服务器")
        server = pywsgi.WSGIServer(('0.0.0.0', args.port), app)
        server.serve_forever()
    except ImportError:
        print("[INFO] 使用 Flask 内置服务器 (threaded)")
        app.run(host='0.0.0.0', port=args.port, debug=False, threaded=True)

if __name__ == '__main__':
    main()
