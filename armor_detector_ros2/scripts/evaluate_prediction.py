#!/usr/bin/env python3
"""
前哨站预测性能评估脚本

功能：
1. 订阅检测结果和预测结果
2. 记录时间序列数据
3. 计算预测误差统计
4. 生成评估报告

使用方法：
    ros2 run armor_detector_ros2 evaluate_prediction.py
    
    # 或者直接运行
    python3 evaluate_prediction.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import json
import os
from datetime import datetime
from dataclasses import dataclass, asdict
from typing import List, Optional

# 消息类型
from armor_detector_ros2.msg import ArmorPoseArray, OutpostState


@dataclass
class DataPoint:
    """单个数据点"""
    timestamp: float
    # 检测数据
    detected_yaw: Optional[float] = None
    detected_pitch: Optional[float] = None
    detected_distance: Optional[float] = None
    detected_x: Optional[float] = None
    detected_y: Optional[float] = None
    detected_z: Optional[float] = None
    detected_u: Optional[float] = None  # 图像坐标
    detected_v: Optional[float] = None
    # 预测数据
    predicted_theta: Optional[float] = None
    predicted_omega: Optional[float] = None
    predicted_center_x: Optional[float] = None
    predicted_center_y: Optional[float] = None
    predicted_center_z: Optional[float] = None
    predicted_armor_x: Optional[float] = None
    predicted_armor_y: Optional[float] = None
    predicted_armor_z: Optional[float] = None
    predicted_u: Optional[float] = None
    predicted_v: Optional[float] = None
    predicted_valid: bool = False
    direction: int = 0


class PredictionEvaluator(Node):
    def __init__(self):
        super().__init__('prediction_evaluator')
        
        # 参数
        self.declare_parameter('output_dir', '/ros2_ws/output/evaluation')
        self.declare_parameter('fx', 1280.0)
        self.declare_parameter('fy', 1280.0)
        self.declare_parameter('cx', 640.0)
        self.declare_parameter('cy', 512.0)
        self.declare_parameter('outpost_radius', 0.275)
        
        self.output_dir = self.get_parameter('output_dir').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        self.radius = self.get_parameter('outpost_radius').value
        
        # 数据存储
        self.data_points: List[DataPoint] = []
        self.current_point: Optional[DataPoint] = None
        self.frame_count = 0
        
        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # 订阅检测结果
        self.pose_sub = self.create_subscription(
            ArmorPoseArray,
            '/solver/armor_poses',
            self.pose_callback,
            qos
        )
        
        # 订阅预测结果
        self.prediction_sub = self.create_subscription(
            OutpostState,
            '/outpost/prediction',
            self.prediction_callback,
            qos
        )
        
        # 定时器：定期输出统计信息
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        
        # 确保输出目录存在
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info(f'预测评估器已启动，输出目录: {self.output_dir}')
    
    def pose_to_3d(self, yaw, pitch, distance):
        """将球坐标转换为笛卡尔坐标"""
        x = distance * np.cos(pitch) * np.sin(-yaw)
        y = distance * np.cos(pitch) * np.cos(yaw)
        z = distance * np.sin(pitch)
        return x, y, z
    
    def project_to_image(self, x, y, z):
        """将3D坐标投影到图像坐标"""
        if y > 0.1:
            u = self.cx + self.fx * (x / y)
            v = self.cy - self.fy * (z / y)
            return u, v
        return None, None
    
    def pose_callback(self, msg: ArmorPoseArray):
        """处理检测结果"""
        self.frame_count += 1
        timestamp = self.get_clock().now().nanoseconds / 1e9
        
        # 创建新的数据点
        self.current_point = DataPoint(timestamp=timestamp)
        
        # 查找前哨站装甲板 (tag_id = 7)
        for pose in msg.poses:
            if pose.valid and pose.tag_id == 7:
                self.current_point.detected_yaw = pose.yaw
                self.current_point.detected_pitch = pose.pitch
                self.current_point.detected_distance = pose.distance
                
                # 计算3D坐标
                x, y, z = self.pose_to_3d(pose.yaw, pose.pitch, pose.distance)
                self.current_point.detected_x = x
                self.current_point.detected_y = y
                self.current_point.detected_z = z
                
                # 计算图像坐标
                u, v = self.project_to_image(x, y, z)
                self.current_point.detected_u = u
                self.current_point.detected_v = v
                break
    
    def prediction_callback(self, msg: OutpostState):
        """处理预测结果"""
        if self.current_point is None:
            return
        
        self.current_point.predicted_valid = msg.valid
        self.current_point.predicted_theta = msg.theta
        self.current_point.predicted_omega = msg.omega
        self.current_point.direction = msg.direction
        self.current_point.predicted_center_x = msg.center.x
        self.current_point.predicted_center_y = msg.center.y
        self.current_point.predicted_center_z = msg.center.z
        
        # 计算预测的装甲板位置
        armor_x = msg.center.x + self.radius * np.sin(msg.theta)
        armor_y = msg.center.y - self.radius * np.cos(msg.theta)
        armor_z = msg.center.z
        
        self.current_point.predicted_armor_x = armor_x
        self.current_point.predicted_armor_y = armor_y
        self.current_point.predicted_armor_z = armor_z
        
        # 计算图像坐标
        u, v = self.project_to_image(armor_x, armor_y, armor_z)
        self.current_point.predicted_u = u
        self.current_point.predicted_v = v
        
        # 保存数据点
        self.data_points.append(self.current_point)
        self.current_point = None
    
    def compute_statistics(self):
        """计算统计数据"""
        if len(self.data_points) < 10:
            return None
        
        # 筛选有效数据点（同时有检测和预测）
        valid_points = [p for p in self.data_points 
                       if p.detected_u is not None and p.predicted_u is not None and p.predicted_valid]
        
        if len(valid_points) < 5:
            return None
        
        # 计算图像坐标误差
        u_errors = [p.predicted_u - p.detected_u for p in valid_points]
        v_errors = [p.predicted_v - p.detected_v for p in valid_points]
        pixel_errors = [np.sqrt((p.predicted_u - p.detected_u)**2 + (p.predicted_v - p.detected_v)**2) 
                       for p in valid_points]
        
        # 计算3D坐标误差
        x_errors = [p.predicted_armor_x - p.detected_x for p in valid_points]
        y_errors = [p.predicted_armor_y - p.detected_y for p in valid_points]
        z_errors = [p.predicted_armor_z - p.detected_z for p in valid_points]
        dist_3d_errors = [np.sqrt((p.predicted_armor_x - p.detected_x)**2 + 
                                  (p.predicted_armor_y - p.detected_y)**2 +
                                  (p.predicted_armor_z - p.detected_z)**2) 
                        for p in valid_points]
        
        # omega 统计
        omegas = [p.predicted_omega for p in valid_points]
        
        stats = {
            'total_frames': self.frame_count,
            'valid_points': len(valid_points),
            'detection_rate': len(valid_points) / self.frame_count if self.frame_count > 0 else 0,
            
            # 图像坐标误差 (像素)
            'u_error_mean': np.mean(u_errors),  # 正=预测在右, 负=预测在左
            'u_error_std': np.std(u_errors),
            'v_error_mean': np.mean(v_errors),
            'v_error_std': np.std(v_errors),
            'pixel_error_mean': np.mean(pixel_errors),
            'pixel_error_std': np.std(pixel_errors),
            'pixel_error_max': np.max(pixel_errors),
            'pixel_error_median': np.median(pixel_errors),
            
            # 3D坐标误差 (米)
            'x_error_mean': np.mean(x_errors),
            'x_error_std': np.std(x_errors),
            'y_error_mean': np.mean(y_errors),
            'y_error_std': np.std(y_errors),
            'z_error_mean': np.mean(z_errors),
            'z_error_std': np.std(z_errors),
            'dist_3d_error_mean': np.mean(dist_3d_errors),
            'dist_3d_error_std': np.std(dist_3d_errors),
            'dist_3d_error_max': np.max(dist_3d_errors),
            
            # omega 统计
            'omega_mean': np.mean(omegas),
            'omega_std': np.std(omegas),
            'omega_min': np.min(omegas),
            'omega_max': np.max(omegas),
            
            # 预测方向分析
            # 对于顺时针旋转(omega<0)，负值表示预测超前
            # 对于逆时针旋转(omega>0)，正值表示预测超前
            'u_error_positive_ratio': sum(1 for e in u_errors if e > 0) / len(u_errors),
            'prediction_ahead_ratio': sum(1 for e, o in zip(u_errors, omegas) if (e < 0 and o < 0) or (e > 0 and o > 0)) / len(u_errors),
        }
        
        return stats
    
    def print_stats(self):
        """打印统计信息"""
        stats = self.compute_statistics()
        if stats is None:
            self.get_logger().info(f'数据点不足，当前: {len(self.data_points)} 帧')
            return
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'预测性能统计 (帧数: {stats["total_frames"]}, 有效点: {stats["valid_points"]})')
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'检测率: {stats["detection_rate"]*100:.1f}%')
        self.get_logger().info(f'像素误差: {stats["pixel_error_mean"]:.1f} ± {stats["pixel_error_std"]:.1f} px (max: {stats["pixel_error_max"]:.1f})')
        # 对于顺时针旋转(omega<0)，负值表示预测超前；对于逆时针旋转(omega>0)，正值表示预测超前
        ahead_direction = "负=超前" if stats["omega_mean"] < 0 else "正=超前"
        self.get_logger().info(f'  U方向: {stats["u_error_mean"]:.1f} ± {stats["u_error_std"]:.1f} px ({ahead_direction})')
        self.get_logger().info(f'  V方向: {stats["v_error_mean"]:.1f} ± {stats["v_error_std"]:.1f} px')
        self.get_logger().info(f'3D误差: {stats["dist_3d_error_mean"]*100:.1f} ± {stats["dist_3d_error_std"]*100:.1f} cm')
        self.get_logger().info(f'角速度: {stats["omega_mean"]:.2f} ± {stats["omega_std"]:.2f} rad/s')
        self.get_logger().info(f'预测超前比例: {stats["prediction_ahead_ratio"]*100:.1f}%')
        self.get_logger().info('=' * 60)
    
    def save_results(self):
        """保存结果到文件"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # 保存原始数据
        data_file = os.path.join(self.output_dir, f'prediction_data_{timestamp}.json')
        with open(data_file, 'w') as f:
            json.dump([asdict(p) for p in self.data_points], f, indent=2)
        
        # 保存统计结果
        stats = self.compute_statistics()
        if stats:
            stats_file = os.path.join(self.output_dir, f'prediction_stats_{timestamp}.json')
            with open(stats_file, 'w') as f:
                json.dump(stats, f, indent=2)
        
        self.get_logger().info(f'结果已保存到: {self.output_dir}')
        return data_file, stats
    
    def destroy_node(self):
        """节点销毁时保存数据"""
        self.save_results()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PredictionEvaluator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
