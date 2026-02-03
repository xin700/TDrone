"""
Solver 节点调试专用 Launch 文件

启动节点：
1. video_publisher_node: 从视频文件读取帧并发布
2. imu_file_publisher_node / mock_imu_node: IMU 数据（文件或模拟）
3. detector_node: 进行装甲板检测
4. solver_node: PnP 位姿解算
5. solver_visualizer_node: 专用调试可视化

用途：
- 单独调试 solver_node 的 PnP 解算
- 验证角点到 3D 位姿的转换
- 检查 yaw/pitch/distance 计算

使用方法：
    # 使用模拟 IMU（默认）：
    ros2 launch armor_detector_ros2 solver_debug_launch.py
    
    # 使用录制的 IMU 文件：
    ros2 launch armor_detector_ros2 solver_debug_launch.py \\
        use_imu_file:=true \\
        imu_file_path:=/home/user/Videos/drone_camera_20251208_223633.txt
    
    # 使用自定义视频文件：
    ros2 launch armor_detector_ros2 solver_debug_launch.py video_path:=/path/to/video.mp4
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('armor_detector_ros2')
    
    # 默认模型路径
    default_armor_model = '/home/user/droneAim/TDrone/models/BRpoints_nano.xml'
    default_classifier_model = '/home/user/droneAim/TDrone/models/classifier.xml'
    default_video = '/home/user/droneAim/TDrone/videos/sample.avi'
    default_output = '/home/user/droneAim/TDrone/output/solver_debug.avi'
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
        default_value='100.0',
        description='视频帧率'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='是否循环播放视频'
    )
    
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='false',
        description='是否显示实时窗口（SSH环境建议false）'
    )
    
    show_axis_arg = DeclareLaunchArgument(
        'show_axis',
        default_value='true',
        description='是否显示 3D 坐标轴'
    )
    
    show_grid_arg = DeclareLaunchArgument(
        'show_grid',
        default_value='true',
        description='是否显示网格'
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
    
    imu_time_offset_arg = DeclareLaunchArgument(
        'imu_time_offset',
        default_value='0.0',
        description='IMU 与视频的时间偏移（秒）'
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
            'yaw': 0.0,
            'pitch': 0.0,
            'roll': 0.0,
            'publish_rate': 100.0,
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
            'time_offset': LaunchConfiguration('imu_time_offset'),
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
    
    # 解算器节点
    solver_node = Node(
        package='armor_detector_ros2',
        executable='solver_node',
        name='solver_node',
        output='screen',
        parameters=[{
            'camera_intrinsics_path': default_camera_config,
        }]
    )

    detector_opensource_node = Node(
        package='armor_detector_ros2',
        executable='detector_opensource_node',
        name='detector_opensource_node',
        output='screen',
        parameters=[{
            'model_path': '/home/user/droneAim/TDrone/armor_detector_ros2/models/0526.onnx',
            'color_flag': 1,
            'conf_threshold': 0.8,
            'nms_threshold': 0.6,
            'use_pixel_color_correction': True,
        }]
    )
    # Solver 专用可视化节点
    solver_visualizer_node = Node(
        package='armor_detector_ros2',
        executable='solver_visualizer_node',
        name='solver_visualizer_node',
        output='screen',
        parameters=[{
            'output_path': LaunchConfiguration('output_path'),
            'output_fps': LaunchConfiguration('fps'),
            'show_window': LaunchConfiguration('show_window'),
            'show_axis': LaunchConfiguration('show_axis'),
            'show_grid': LaunchConfiguration('show_grid'),
            # 相机内参 (与 camera_intrinsics.yaml 保持一致)
            'fx': 3083.8105952278529,
            'fy': 3087.5133740683185,
            'cx': 632.7590339498046,
            'cy': 506.76799711496106,
            # 畸变系数
            'k1': -0.029973359424498877,
            'k2': 0.6368251830188471,
            'p1': -0.0018583704879155746,
            'p2': 0.0016903907748413662,
            'k3': -3.2254813681718064,
        }]
    )
    
    return LaunchDescription([
        video_path_arg,
        output_path_arg,
        fps_arg,
        loop_arg,
        show_window_arg,
        show_axis_arg,
        show_grid_arg,
        use_imu_file_arg,
        imu_file_path_arg,
        imu_time_offset_arg,
        video_publisher_node,
        mock_imu_node,
        imu_file_publisher_node,
        detector_opensource_node,
        solver_node,
        solver_visualizer_node,
    ])
