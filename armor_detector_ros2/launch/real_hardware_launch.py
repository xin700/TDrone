"""
真实硬件系统 ROS2 启动文件

功能：
1. real_hardware_publisher_node: 从真实相机采集图像，从串口读取IMU数据
   - 发布到 /camera/image_raw (与mock_camera_node兼容)
   - 发布到 /imu/quaternion (与mock_imu_node兼容)
   - 发布到 /bullet_speed
   - 发布到 /aim_request
   - 提供 /pipeline_control 服务控制检测流水线启停

使用方法：
    # 使用默认参数启动（真实硬件）
    ros2 launch armor_detector_ros2 real_hardware_launch.py
    
    # 指定相机序列号
    ros2 launch armor_detector_ros2 real_hardware_launch.py camera_sn:=FGK25050153
    
    # 指定串口设备
    ros2 launch armor_detector_ros2 real_hardware_launch.py serial_device:=/dev/ttyACM0
    
    # 使用视频文件模拟（用于测试）
    ros2 launch armor_detector_ros2 real_hardware_launch.py use_simulated_camera:=true simulated_video_path:=/path/to/video.mp4
    
参数说明：
    camera_sn: 大恒相机序列号 (默认 FGK25050153)
    exposure_time: 曝光时间 μs (默认 3000)
    img_width: 图像宽度 (默认 1280)
    img_height: 图像高度 (默认 1024)
    fps: 目标帧率 (默认 60)
    serial_device: 串口设备路径 (默认 /dev/ttyACM0)
    serial_baud: 串口波特率 (默认 115200)
    imu_rate: IMU发布频率 (默认 500 Hz)
    use_simulated_camera: 使用视频文件代替真实相机 (默认 false)
    simulated_video_path: 模拟视频文件路径

服务接口：
    /pipeline_control (std_srvs/SetBool): 
        - True: 启动检测流水线
        - False: 停止检测流水线（数据采集继续）
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('armor_detector_ros2')
    
    # ==================== 声明启动参数 ====================
    # 相机参数
    camera_sn_arg = DeclareLaunchArgument(
        'camera_sn',
        default_value='FGK25050153',
        description='大恒相机序列号'
    )
    
    exposure_time_arg = DeclareLaunchArgument(
        'exposure_time',
        default_value='3000',
        description='曝光时间 (μs)'
    )
    
    img_width_arg = DeclareLaunchArgument(
        'img_width',
        default_value='1280',
        description='图像宽度'
    )
    
    img_height_arg = DeclareLaunchArgument(
        'img_height',
        default_value='1024',
        description='图像高度'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='60.0',
        description='目标帧率'
    )
    
    # 串口参数
    serial_device_arg = DeclareLaunchArgument(
        'serial_device',
        default_value='/dev/ttyACM0',
        description='串口设备路径'
    )
    
    serial_baud_arg = DeclareLaunchArgument(
        'serial_baud',
        default_value='115200',
        description='串口波特率'
    )
    
    imu_rate_arg = DeclareLaunchArgument(
        'imu_rate',
        default_value='500.0',
        description='IMU发布频率 (Hz)'
    )
    
    # 模拟模式参数
    use_simulated_camera_arg = DeclareLaunchArgument(
        'use_simulated_camera',
        default_value='false',
        description='使用视频文件代替真实相机'
    )
    
    simulated_video_path_arg = DeclareLaunchArgument(
        'simulated_video_path',
        default_value='',
        description='模拟视频文件路径'
    )
    
    # ==================== 定义节点 ====================
    # 真实硬件发布节点
    real_hardware_publisher_node = Node(
        package='armor_detector_ros2',
        executable='real_hardware_publisher_node',
        name='real_hardware_publisher_node',
        output='screen',
        parameters=[{
            'camera_sn': LaunchConfiguration('camera_sn'),
            'exposure_time': LaunchConfiguration('exposure_time'),
            'img_width': LaunchConfiguration('img_width'),
            'img_height': LaunchConfiguration('img_height'),
            'fps': LaunchConfiguration('fps'),
            'serial_device': LaunchConfiguration('serial_device'),
            'serial_baud': LaunchConfiguration('serial_baud'),
            'imu_rate': LaunchConfiguration('imu_rate'),
            'use_simulated_camera': LaunchConfiguration('use_simulated_camera'),
            'simulated_video_path': LaunchConfiguration('simulated_video_path'),
        }]
    )
    
    # ==================== 返回启动描述 ====================
    return LaunchDescription([
        # 启动参数
        camera_sn_arg,
        exposure_time_arg,
        img_width_arg,
        img_height_arg,
        fps_arg,
        serial_device_arg,
        serial_baud_arg,
        imu_rate_arg,
        use_simulated_camera_arg,
        simulated_video_path_arg,
        # 节点
        real_hardware_publisher_node,
    ])
