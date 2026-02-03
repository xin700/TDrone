"""
完整检测+解算流水线 ROS2 启动文件

启动节点：
1. video_publisher_node: 从视频文件读取帧并发布
2. imu_file_publisher_node / mock_imu_node: IMU 数据（文件或模拟）
3. detector_node: 进行装甲板关键点检测和数字分类
4. solver_node: PnP 位姿解算
5. visualizer_node: 可视化检测和解算结果并写入视频文件

使用方法：
    ros2 launch armor_detector_ros2 full_pipeline_launch.py
    
    # 使用自定义视频文件：
    ros2 launch armor_detector_ros2 full_pipeline_launch.py video_path:=/path/to/video.mp4
    
    # 使用录制的 IMU 文件：
    ros2 launch armor_detector_ros2 full_pipeline_launch.py \\
        use_imu_file:=true \\
        imu_file_path:=/home/user/Videos/drone_camera_xxx.txt
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('armor_detector_ros2')
    
    # 默认模型路径 (使用 Docker 容器内路径 ~/droneAim/TDrone)
    default_armor_model = '/home/user/droneAim/TDrone/models/BRpoints_nano.xml'
    default_classifier_model = '/home/user/droneAim/TDrone/models/classifier.xml'
    default_video = '/home/user/droneAim/TDrone/videos/sample.avi'
    default_output = '/home/user/droneAim/TDrone/output/full_pipeline_result.mp4'
    default_camera_config = os.path.join(pkg_share, 'config', 'camera_intrinsics.yaml')
    
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
    
    show_pose_arg = DeclareLaunchArgument(
        'show_pose_info',
        default_value='true',
        description='是否显示位姿信息'
    )
    
    show_ekf_arg = DeclareLaunchArgument(
        'show_ekf_info',
        default_value='false',
        description='是否显示 EKF 跟踪信息'
    )
    
    show_trajectory_arg = DeclareLaunchArgument(
        'show_trajectory',
        default_value='false',
        description='是否显示目标轨迹'
    )
    
    # IMU 相关参数
    use_imu_file_arg = DeclareLaunchArgument(
        'use_imu_file',
        default_value='false',
        description='是否使用 IMU 文件（而非模拟 IMU）'
    )
    
    imu_file_path_arg = DeclareLaunchArgument(
        'imu_file_path',
        default_value='',
        description='IMU 数据文件路径'
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
    
    # 模拟 IMU 节点（当不使用 IMU 文件时）
    mock_imu_node = Node(
        package='armor_detector_ros2',
        executable='mock_imu_node',
        name='mock_imu_node',
        output='screen',
        parameters=[{
            'publish_rate': 200.0,
        }],
        condition=UnlessCondition(LaunchConfiguration('use_imu_file'))
    )
    
    # IMU 文件播放节点（当使用 IMU 文件时）
    imu_file_publisher_node = Node(
        package='armor_detector_ros2',
        executable='imu_file_publisher_node',
        name='imu_file_publisher_node',
        output='screen',
        parameters=[{
            'imu_file_path': LaunchConfiguration('imu_file_path'),
            'time_offset': 0.0,
            'publish_rate': 0.0,  # 跟随视频
            'interpolate': True,
            'loop': LaunchConfiguration('loop'),
        }],
        condition=IfCondition(LaunchConfiguration('use_imu_file'))
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
    
    # 位姿解算节点
    solver_node = Node(
        package='armor_detector_ros2',
        executable='solver_node',
        name='solver_node',
        output='screen',
        parameters=[{
            # 默认相机内参（1280x720）
            'fx': 1280.0,
            'fy': 1280.0,
            'cx': 640.0,
            'cy': 360.0,
            'k1': 0.0,
            'k2': 0.0,
            'p1': 0.0,
            'p2': 0.0,
            'k3': 0.0,
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
            'show_pose_info': LaunchConfiguration('show_pose_info'),
            'show_ekf_info': LaunchConfiguration('show_ekf_info'),
            'show_trajectory': LaunchConfiguration('show_trajectory'),
            'trajectory_length': 50,
        }]
    )
    
    # ==================== 返回启动描述 ====================
    return LaunchDescription([
        # 启动参数
        video_path_arg,
        output_path_arg,
        fps_arg,
        loop_arg,
        show_pose_arg,
        show_ekf_arg,
        show_trajectory_arg,
        use_imu_file_arg,
        imu_file_path_arg,
        # 节点
        video_publisher_node,
        mock_imu_node,
        imu_file_publisher_node,
        detector_node,
        solver_node,
        visualizer_node,
    ])
