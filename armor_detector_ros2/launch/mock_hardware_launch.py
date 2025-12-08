"""
模拟硬件系统 ROS2 启动文件

启动三个模拟硬件节点：
1. mock_camera_node: 从视频文件读取帧并发布到 /camera/image_raw
2. mock_imu_node: 发布模拟 IMU 四元数到 /imu/quaternion
3. mock_cboard_node: 发布模拟弹速，接收控制指令

使用方法：
    ros2 launch armor_detector_ros2 mock_hardware_launch.py
    
    # 使用自定义视频文件：
    ros2 launch armor_detector_ros2 mock_hardware_launch.py video_path:=/path/to/video.mp4
    
    # 使用自定义弹速：
    ros2 launch armor_detector_ros2 mock_hardware_launch.py bullet_speed:=28.0
    
    # 使用旋转 IMU 模式：
    ros2 launch armor_detector_ros2 mock_hardware_launch.py imu_mode:=rotating

Requirements: 9.3
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
    
    # 默认视频路径
    default_video = '/ros2_ws/videos/1.mp4'
    
    # ==================== 声明启动参数 ====================
    # 相机参数
    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value=default_video,
        description='输入视频文件路径'
    )
    
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='30.0',
        description='相机发布帧率'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='是否循环播放视频'
    )
    
    # IMU 参数
    imu_rate_arg = DeclareLaunchArgument(
        'imu_rate',
        default_value='200.0',
        description='IMU 发布频率 (Hz)'
    )
    
    imu_mode_arg = DeclareLaunchArgument(
        'imu_mode',
        default_value='static',
        description='IMU 模式: static, rotating, oscillating'
    )
    
    rotation_speed_arg = DeclareLaunchArgument(
        'rotation_speed',
        default_value='0.5',
        description='旋转速度 (rad/s)'
    )
    
    oscillation_amplitude_arg = DeclareLaunchArgument(
        'oscillation_amplitude',
        default_value='0.3',
        description='振荡幅度 (rad)'
    )
    
    oscillation_frequency_arg = DeclareLaunchArgument(
        'oscillation_frequency',
        default_value='1.0',
        description='振荡频率 (Hz)'
    )
    
    # CBoard 参数
    cboard_rate_arg = DeclareLaunchArgument(
        'cboard_rate',
        default_value='100.0',
        description='CBoard 发布频率 (Hz)'
    )
    
    bullet_speed_arg = DeclareLaunchArgument(
        'bullet_speed',
        default_value='25.0',
        description='模拟弹速 (m/s)'
    )
    
    bullet_speed_noise_arg = DeclareLaunchArgument(
        'bullet_speed_noise',
        default_value='0.5',
        description='弹速噪声标准差 (m/s)'
    )
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='0',
        description='CBoard 模式'
    )
    
    # ==================== 定义节点 ====================
    # 模拟相机节点
    mock_camera_node = Node(
        package='armor_detector_ros2',
        executable='mock_camera_node',
        name='mock_camera_node',
        output='screen',
        parameters=[{
            'video_path': LaunchConfiguration('video_path'),
            'fps': LaunchConfiguration('camera_fps'),
            'loop': LaunchConfiguration('loop'),
        }]
    )
    
    # 模拟 IMU 节点
    mock_imu_node = Node(
        package='armor_detector_ros2',
        executable='mock_imu_node',
        name='mock_imu_node',
        output='screen',
        parameters=[{
            'publish_rate': LaunchConfiguration('imu_rate'),
            'mode': LaunchConfiguration('imu_mode'),
            'rotation_speed': LaunchConfiguration('rotation_speed'),
            'oscillation_amplitude': LaunchConfiguration('oscillation_amplitude'),
            'oscillation_frequency': LaunchConfiguration('oscillation_frequency'),
        }]
    )
    
    # 模拟 CBoard 节点
    mock_cboard_node = Node(
        package='armor_detector_ros2',
        executable='mock_cboard_node',
        name='mock_cboard_node',
        output='screen',
        parameters=[{
            'publish_rate': LaunchConfiguration('cboard_rate'),
            'bullet_speed': LaunchConfiguration('bullet_speed'),
            'bullet_speed_noise': LaunchConfiguration('bullet_speed_noise'),
            'mode': LaunchConfiguration('mode'),
        }]
    )
    
    # ==================== 返回启动描述 ====================
    return LaunchDescription([
        # 启动参数
        video_path_arg,
        camera_fps_arg,
        loop_arg,
        imu_rate_arg,
        imu_mode_arg,
        rotation_speed_arg,
        oscillation_amplitude_arg,
        oscillation_frequency_arg,
        cboard_rate_arg,
        bullet_speed_arg,
        bullet_speed_noise_arg,
        mode_arg,
        # 节点
        mock_camera_node,
        mock_imu_node,
        mock_cboard_node,
    ])
