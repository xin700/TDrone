"""
装甲板检测系统 - 零拷贝版本启动文件

特点：
1. 所有节点在同一进程中运行（ComponentContainer）
2. 启用进程内通信（use_intra_process_comms: True）
3. 图像消息通过 unique_ptr 传递，实现零拷贝

性能优势：
- 图像传输延迟从毫秒级降到微秒级
- 减少CPU内存拷贝开销
- 降低内存带宽占用

使用方法：
    ros2 launch armor_detector_ros2 detector_zero_copy_launch.py
    
    # 使用自定义视频文件：
    ros2 launch armor_detector_ros2 detector_zero_copy_launch.py video_path:=/path/to/video.mp4
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # 默认配置
    default_armor_model = '/home/xin/ROS2Drone/OrangeAim-Drone/utils/models/armor_yolo_x.xml'
    default_classifier_model = '/home/xin/ROS2Drone/OrangeAim-Drone/utils/models/classifier.xml'
    default_video = '/home/xin/ROS2Drone/OrangeAim-Drone/utils/红方前哨站公路视角全速.mp4'
    default_output = '/home/xin/ROS2Drone/ws/output/detection_zero_copy.mp4'
    
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
    
    # ==================== 组件容器（同进程运行） ====================
    container = ComposableNodeContainer(
        name='detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 视频发布组件
            ComposableNode(
                package='armor_detector_ros2',
                plugin='armor_detector::VideoPublisherComponent',
                name='video_publisher',
                parameters=[{
                    'video_path': LaunchConfiguration('video_path'),
                    'fps': LaunchConfiguration('fps'),
                    'loop': LaunchConfiguration('loop'),
                }],
                extra_arguments=[{'use_intra_process_comms': True}],  # 启用零拷贝
            ),
            # 检测器组件
            ComposableNode(
                package='armor_detector_ros2',
                plugin='armor_detector::DetectorComponent',
                name='detector',
                parameters=[{
                    'armor_model_path': default_armor_model,
                    'classifier_model_path': default_classifier_model,
                    'color_flag': -1,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],  # 启用零拷贝
            ),
            # 可视化组件
            ComposableNode(
                package='armor_detector_ros2',
                plugin='armor_detector::VisualizerComponent',
                name='visualizer',
                parameters=[{
                    'output_video_path': LaunchConfiguration('output_path'),
                    'fps': LaunchConfiguration('fps'),
                }],
                extra_arguments=[{'use_intra_process_comms': True}],  # 启用零拷贝
            ),
        ],
        output='screen',
    )
    
    return LaunchDescription([
        video_path_arg,
        output_path_arg,
        fps_arg,
        loop_arg,
        container,
    ])
