"""
装甲板检测系统ROS2启动文件

启动三个节点：
1. video_publisher_node: 从视频文件读取帧并发布
2. detector_node: 进行装甲板关键点检测和数字分类
3. visualizer_node: 可视化检测结果并写入视频文件

使用方法：
    ros2 launch armor_detector_ros2 detector_launch.py
    
    # 使用自定义视频文件：
    ros2 launch armor_detector_ros2 detector_launch.py video_path:=/path/to/video.mp4
    
    # 使用自定义输出路径：
    ros2 launch armor_detector_ros2 detector_launch.py output_path:=/path/to/output.mp4
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
    
    # 默认配置文件路径
    default_config = os.path.join(pkg_share, 'config', 'detector_config.yaml')
    
    # 默认模型路径 (使用 Docker 容器内路径 ~/droneAim/TDrone)
    default_armor_model = '/home/user/droneAim/TDrone/models/BRpoints_nano.xml'
    default_classifier_model = '/home/user/droneAim/TDrone/models/classifier.xml'
    default_video = '/home/user/droneAim/TDrone/videos/sample.avi'
    default_output = '/home/user/droneAim/TDrone/output/detection_result.mp4'
    
    # ==================== 声明启动参数 ====================
    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value=default_video,
        description='输入视频文件路径'
    )
    
    output_path_arg = DeclareLaunchArgument(
        'output_path',
        default_value=default_output,
        description='输出视频文件路径'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30.0',
        description='视频帧率'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='是否循环播放视频'
    )
    
    # ==================== 定义节点 ====================
    # 视频发布节点
    video_publisher_node = Node(
        package='armor_detector_ros2',
        executable='video_publisher_node',
        name='video_publisher_node',
        output='screen',
        parameters=[{
            'video_path': LaunchConfiguration('video_path'),
            'fps': LaunchConfiguration('fps'),
            'loop': LaunchConfiguration('loop'),
        }]
    )
    
    # 检测器节点
    detector_node = Node(
        package='armor_detector_ros2',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[{
            'armor_model_path': default_armor_model,
            'classifier_model_path': default_classifier_model,
            'color_flag': -1,  # 不过滤颜色
        }]
    )
    
    # 可视化节点
    visualizer_node = Node(
        package='armor_detector_ros2',
        executable='visualizer_node',
        name='visualizer_node',
        output='screen',
        parameters=[{
            'output_video_path': LaunchConfiguration('output_path'),
            'fps': LaunchConfiguration('fps'),
        }]
    )
    
    # ==================== 返回启动描述 ====================
    return LaunchDescription([
        # 启动参数
        video_path_arg,
        output_path_arg,
        fps_arg,
        loop_arg,
        # 节点
        video_publisher_node,
        detector_node,
        visualizer_node,
    ])
