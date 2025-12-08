"""
前哨站预测评估启动文件

启动前哨站流水线 + 评估节点，用于收集和分析预测性能数据

使用方法：
    ros2 launch armor_detector_ros2 outpost_evaluation_launch.py
    ros2 launch armor_detector_ros2 outpost_evaluation_launch.py video_path:=/path/to/video.mp4
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('armor_detector_ros2')
    
    # 默认路径
    default_video = '/ros2_ws/videos/3.avi'
    default_output = '/ros2_ws/output/evaluation'
    
    # 启动参数
    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value=default_video,
        description='输入视频文件路径'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=default_output,
        description='评估结果输出目录'
    )
    
    # 包含前哨站流水线
    outpost_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'outpost_pipeline_launch.py')
        ),
        launch_arguments={
            'video_path': LaunchConfiguration('video_path'),
            'loop': 'false',
        }.items()
    )
    
    # 评估节点
    evaluator_node = Node(
        package='armor_detector_ros2',
        executable='evaluate_prediction.py',
        name='prediction_evaluator',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'fx': 1280.0,
            'fy': 1280.0,
            'cx': 640.0,
            'cy': 512.0,
            'outpost_radius': 0.275,
        }]
    )
    
    return LaunchDescription([
        video_path_arg,
        output_dir_arg,
        outpost_pipeline,
        evaluator_node,
    ])
