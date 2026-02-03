#!/usr/bin/env python3
"""
云台控制节点测试脚本

用法：
    python3 test_gimbal_control.py

功能：
    1. 发布测试GimbalCommand消息
    2. 循环发送不同角度的云台指令
    3. 可选测试模式：静态、扫描、圆周
"""

import rclpy
from rclpy.node import Node
from armor_detector_ros2.msg import GimbalCommand
from std_msgs.msg import Header
import math
import time


class GimbalControlTester(Node):
    def __init__(self):
        super().__init__('gimbal_control_tester')
        
        # 创建发布者
        self.publisher = self.create_publisher(
            GimbalCommand,
            '/aiming/gimbal_command',
            10
        )
        
        # 创建定时器（100Hz）
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        # 测试参数
        self.test_mode = 'static'  # 'static', 'sweep', 'circle', 'manual'
        self.start_time = time.time()
        self.counter = 0
        
        self.get_logger().info('云台控制测试节点已启动')
        self.get_logger().info('测试模式: {}'.format(self.test_mode))
        self.print_usage()
    
    def print_usage(self):
        self.get_logger().info('==================== 测试模式 ====================')
        self.get_logger().info('1. static  - 固定角度测试 (yaw=0, pitch=0)')
        self.get_logger().info('2. sweep   - 扫描测试 (yaw: -30° ~ +30°)')
        self.get_logger().info('3. circle  - 圆周测试 (yaw: 0° ~ 360°)')
        self.get_logger().info('4. manual  - 手动模式 (需要修改代码设置角度)')
        self.get_logger().info('================================================')
        self.get_logger().info('修改 test_mode 变量来切换测试模式')
    
    def timer_callback(self):
        msg = GimbalCommand()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'body_frame'
        
        elapsed = time.time() - self.start_time
        
        if self.test_mode == 'static':
            # 静态测试：固定角度
            msg.yaw = 0.0
            msg.pitch = 0.0
            
        elif self.test_mode == 'sweep':
            # 扫描测试：-30° ~ +30° 正弦扫描
            msg.yaw = 30.0 * math.sin(elapsed * 0.5)  # 周期约12.5秒
            msg.pitch = 0.0
            
        elif self.test_mode == 'circle':
            # 圆周测试：0° ~ 360° 匀速旋转
            msg.yaw = (elapsed * 30.0) % 360.0  # 30°/s
            if msg.yaw > 180.0:
                msg.yaw -= 360.0  # 转换到 [-180, 180]
            msg.pitch = 0.0
            
        elif self.test_mode == 'manual':
            # 手动模式：设置固定角度
            msg.yaw = 10.0  # 修改这里设置目标yaw
            msg.pitch = 5.0  # 修改这里设置目标pitch
        
        # 其他字段
        msg.aiming_state = 1  # TRACKING
        msg.fire_allowed = False
        msg.delta_yaw = 0.0
        msg.delta_pitch = 0.0
        msg.distance = 5.0
        msg.gravity_compensation = 0.0
        
        # 发布消息
        self.publisher.publish(msg)
        
        # 每100次打印一次（每秒一次）
        self.counter += 1
        if self.counter % 100 == 0:
            self.get_logger().info(
                '[{}] 发送: yaw={:.2f}°, pitch={:.2f}°'.format(
                    self.test_mode, msg.yaw, msg.pitch
                )
            )


def main(args=None):
    rclpy.init(args=args)
    
    tester = GimbalControlTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('测试中断')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
