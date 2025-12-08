"""
前哨站预测流水线 ROS2 启动文件

启动节点：
1. video_publisher_node: 从视频文件读取帧并发布
2. mock_imu_node: 模拟 IMU 数据
3. detector_node: 进行装甲板关键点检测和数字分类
4. solver_node: PnP 位姿解算
5. outpost_predictor_node: 前哨站状态预测
6. visualizer_node: 可视化检测、解算和预测结果

使用方法：
    ros2 launch armor_detector_ros2 outpost_pipeline_launch.py
    
    # 使用自定义视频文件：
    ros2 launch armor_detector_ros2 outpost_pipeline_launch.py video_path:=/path/to/video.mp4

Requirements: 7.1, 7.2, 7.3
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
    
    # 默认模型路径 (使用 Docker 容器内路径 /ros2_ws)
    default_armor_model = '/ros2_ws/models/BRpoints_nano.xml'
    default_classifier_model = '/ros2_ws/models/classifier.xml'
    default_video = '/ros2_ws/videos/outpost_sample.mp4'
    default_output = '/ros2_ws/output/outpost_prediction_result.mp4'
    
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
        default_value='-1.0',
        description='视频发布帧率（-1表示使用原始帧率）'
    )
    
    output_fps_arg = DeclareLaunchArgument(
        'output_fps',
        default_value='30.0',
        description='输出视频帧率'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='是否循环播放视频'
    )
    
    show_outpost_arg = DeclareLaunchArgument(
        'show_outpost_info',
        default_value='true',
        description='是否显示前哨站预测信息'
    )
    
    show_trajectory_arg = DeclareLaunchArgument(
        'show_trajectory',
        default_value='true',
        description='是否显示预测轨迹'
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
    
    # 模拟 IMU 节点
    mock_imu_node = Node(
        package='armor_detector_ros2',
        executable='mock_imu_node',
        name='mock_imu_node',
        output='screen',
        parameters=[{
            'publish_rate': 200.0,
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
    
    # 位姿解算节点
    solver_node = Node(
        package='armor_detector_ros2',
        executable='solver_node',
        name='solver_node',
        output='screen',
        parameters=[{
            # 默认相机内参（1280x1024）
            'fx': 1280.0,
            'fy': 1280.0,
            'cx': 640.0,
            'cy': 512.0,  # 1024/2 = 512
            'k1': 0.0,
            'k2': 0.0,
            'p1': 0.0,
            'p2': 0.0,
            'k3': 0.0,
        }]
    )
    
    # 前哨站预测节点
    outpost_predictor_node = Node(
        package='armor_detector_ros2',
        executable='outpost_predictor_node',
        name='outpost_predictor_node',
        output='screen',
        parameters=[{
            'outpost_tag_id': 7,  # 前哨站装甲板 tag_id
            'chi_square_threshold': 11.07,  # 卡方检验阈值
            'initial_direction': 0,  # 初始旋转方向
            'prediction_dt': 0.01,  # 预测时间步长
            'min_observations_for_valid': 2,  # 最小有效观测数（降低，因为前哨站旋转时检测不连续）
            'observation_timeout': 5.0,  # 超时时间（秒），前哨站旋转一圈约2秒
            'accept_any_tag': True,  # 接受任何 tag_id（用于测试，因为分类器可能无法正确识别前哨站）
            'prediction_ahead_time': 0.2,  # 预测超前时间（秒），补偿系统延迟
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
            'fps': LaunchConfiguration('output_fps'),  # 输出视频使用固定帧率
            'show_pose_info': True,
            'show_ekf_info': False,
            'show_trajectory': LaunchConfiguration('show_trajectory'),
            'show_outpost_info': LaunchConfiguration('show_outpost_info'),
            'trajectory_length': 100,
        }]
    )
    
    # ==================== 返回启动描述 ====================
    return LaunchDescription([
        # 启动参数
        video_path_arg,
        output_path_arg,
        fps_arg,
        output_fps_arg,
        loop_arg,
        show_outpost_arg,
        show_trajectory_arg,
        # 节点
        video_publisher_node,
        mock_imu_node,
        detector_node,
        solver_node,
        outpost_predictor_node,
        visualizer_node,
    ])
