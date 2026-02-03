"""
开源模型装甲板检测系统ROS2启动文件

使用rm.cv.fans的0526.onnx模型进行装甲板检测

启动三个节点：
1. video_publisher_node: 从视频文件读取帧并发布
2. detector_opensource_node: 使用0526.onnx进行装甲板检测
3. visualizer_node: 可视化检测结果并写入视频文件

使用方法：
    ros2 launch armor_detector_ros2 detector_opensource_launch.py
    
    # 使用自定义视频文件：
    ros2 launch armor_detector_ros2 detector_opensource_launch.py video_path:=/path/to/video.mp4
    
    # 使用自定义模型：
    ros2 launch armor_detector_ros2 detector_opensource_launch.py model_path:=/path/to/model.onnx
    
    # 检测红色装甲板（默认）：
    ros2 launch armor_detector_ros2 detector_opensource_launch.py color_flag:=1
    
    # 检测蓝色装甲板：
    ros2 launch armor_detector_ros2 detector_opensource_launch.py color_flag:=0
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
    
    # 默认路径配置
    default_model = '/home/user/droneAim/TDrone/armor_detector_ros2/models/0526.onnx'
    default_video = '/home/user/droneAim/TDrone/videos/r1.avi'
    default_output = '/home/user/droneAim/TDrone/output/opensource_detection_result.mp4'
    
    # ==================== 声明启动参数 ====================
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model,
        description='0526.onnx开源模型路径'
    )
    
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
    
    color_flag_arg = DeclareLaunchArgument(
        'color_flag',
        default_value='1',
        description='检测颜色: 0=蓝色, 1=红色, -1=不过滤'
    )
    
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.6',
        description='置信度阈值'
    )
    
    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold',
        default_value='0.45',
        description='NMS阈值'
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
    
    # 开源模型检测器节点
    detector_opensource_node = Node(
        package='armor_detector_ros2',
        executable='detector_opensource_node',
        name='detector_opensource_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'color_flag': LaunchConfiguration('color_flag'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'nms_threshold': LaunchConfiguration('nms_threshold'),
            'use_pixel_color_correction': True,
        }]
    )
    
    # 可视化节点
    visualizer_node = Node(
        package='armor_detector_ros2',
        executable='visualizer_node',
        name='visualizer_node',
        output='screen',
        parameters=[{
            'output_path': LaunchConfiguration('output_path'),
        }]
    )
    
    # ==================== 构建启动描述 ====================
    return LaunchDescription([
        # 启动参数
        model_path_arg,
        video_path_arg,
        output_path_arg,
        fps_arg,
        loop_arg,
        color_flag_arg,
        conf_threshold_arg,
        nms_threshold_arg,
        
        # 节点
        video_publisher_node,
        detector_opensource_node,
        visualizer_node,
    ])
