"""
前哨站预测流水线 V2 ROS2 启动文件

启动节点：
1. video_publisher_node: 从视频文件读取帧并发布
2. imu_file_publisher_node / mock_imu_node: IMU 数据（文件或模拟）
3. detector_node: 进行装甲板关键点检测和数字分类
4. solver_node: PnP 位姿解算
5. outpost_predictor_node_v2: 新版前哨站状态预测 (5维状态向量)
6. visualizer_node: 可视化检测、解算和预测结果

使用方法：
    ros2 launch armor_detector_ros2 outpost_pipeline_v2_launch.py
    
    # 使用自定义视频文件：
    ros2 launch armor_detector_ros2 outpost_pipeline_v2_launch.py video_path:=/path/to/video.mp4
    
    # 使用录制的 IMU 文件：
    ros2 launch armor_detector_ros2 outpost_pipeline_v2_launch.py \\
        use_imu_file:=true \\
        imu_file_path:=/home/user/Videos/drone_camera_xxx.txt
    
    # 调整前哨站参数：
    ros2 launch armor_detector_ros2 outpost_pipeline_v2_launch.py \
        outpost_radius:=0.3 \
        outpost_omega:=2.5 \
        bullet_speed:=26.0

新版预测器特性：
- 5维状态向量 [x_c, y_c, z_c, θ, ω]（移除冗余的速度状态）
- 显式建模3个装甲板的高度差异
- 可打窗口计算
- 延迟补偿（系统延迟 + 弹道飞行时间）
- 卡方检验的观测验证
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
    
    # 默认路径
    default_armor_model = '/home/user/droneAim/TDrone/models/BRpoints_nano.xml'
    default_classifier_model = '/home/user/droneAim/TDrone/models/classifier.xml'
    default_video = '/home/user/droneAim/TDrone/videos/outpost_sample.mp4'
    default_output = '/home/user/droneAim/TDrone/output/outpost_v2_result.mp4'
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
    
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='false',
        description='是否实时显示可视化窗口'
    )
    
    # ==================== 前哨站几何参数 ====================
    outpost_radius_arg = DeclareLaunchArgument(
        'outpost_radius',
        default_value='0.275',
        description='前哨站装甲板旋转半径 (m)'
    )
    
    outpost_height_diff_arg = DeclareLaunchArgument(
        'outpost_height_diff',
        default_value='0.0',
        description='装甲板高度差 (m)，如果三块装甲板等高则为0'
    )
    
    outpost_tilt_angle_arg = DeclareLaunchArgument(
        'outpost_tilt_angle',
        default_value='1.48',
        description='装甲板倾斜角 (rad)，约85度'
    )
    
    # ==================== 前哨站运动参数 ====================
    outpost_omega_arg = DeclareLaunchArgument(
        'outpost_omega',
        default_value='2.513',  # 0.4 * 2π
        description='前哨站已知角速度 (rad/s)'
    )
    
    use_fixed_omega_arg = DeclareLaunchArgument(
        'use_fixed_omega',
        default_value='true',
        description='是否使用固定角速度（标准前哨站为true）'
    )
    
    # ==================== 弹道参数 ====================
    bullet_speed_arg = DeclareLaunchArgument(
        'bullet_speed',
        default_value='28.0',
        description='子弹速度 (m/s)'
    )
    
    system_delay_arg = DeclareLaunchArgument(
        'system_delay',
        default_value='0.05',
        description='系统延迟 (s)，包括检测、通信、电机响应等'
    )
    
    # ==================== 瞄准参数 ====================
    max_orientation_angle_arg = DeclareLaunchArgument(
        'max_orientation_angle',
        default_value='0.65',
        description='最大可打角度 (rad)，约37度'
    )
    
    # ==================== 噪声参数 ====================
    process_pos_noise_arg = DeclareLaunchArgument(
        'process_pos_noise',
        default_value='0.001',
        description='位置过程噪声'
    )
    
    process_theta_noise_arg = DeclareLaunchArgument(
        'process_theta_noise',
        default_value='0.01',
        description='角度过程噪声'
    )
    
    measurement_pos_noise_arg = DeclareLaunchArgument(
        'measurement_pos_noise',
        default_value='0.02',
        description='位置观测噪声'
    )
    
    measurement_angle_noise_arg = DeclareLaunchArgument(
        'measurement_angle_noise',
        default_value='0.05',
        description='角度观测噪声'
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
            'publish_rate': 0.0,
            'interpolate': True,
            'loop': LaunchConfiguration('loop'),
        }],
        condition=IfCondition(LaunchConfiguration('use_imu_file'))
    )
    
    # 检测器节点
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
    
    # 位姿解算节点
    solver_node = Node(
        package='armor_detector_ros2',
        executable='solver_node',
        name='solver_node',
        output='screen',
        parameters=[{
            'armor_width': 0.135,
            'armor_height': 0.055,
        }]
    )
    
    # 新版前哨站预测器节点 (V2)
    outpost_predictor_node_v2 = Node(
        package='armor_detector_ros2',
        executable='outpost_predictor_node_v2',
        name='outpost_predictor_v2',
        output='screen',
        parameters=[{
            # 几何参数
            'geometry.radius': LaunchConfiguration('outpost_radius'),
            'geometry.height_diff': LaunchConfiguration('outpost_height_diff'),
            'geometry.tilt_angle': LaunchConfiguration('outpost_tilt_angle'),
            # 运动参数
            'known_omega': LaunchConfiguration('outpost_omega'),
            'use_fixed_omega': LaunchConfiguration('use_fixed_omega'),
            'omega_threshold': 0.1,
            # 噪声参数
            'noise.process_pos': LaunchConfiguration('process_pos_noise'),
            'noise.process_theta': LaunchConfiguration('process_theta_noise'),
            'noise.process_omega': 0.001,
            'noise.measurement_pos': LaunchConfiguration('measurement_pos_noise'),
            'noise.measurement_angle': LaunchConfiguration('measurement_angle_noise'),
            # 弹道参数
            'bullet_speed': LaunchConfiguration('bullet_speed'),
            'system_delay': LaunchConfiguration('system_delay'),
            # 瞄准参数
            'max_orientation_angle': LaunchConfiguration('max_orientation_angle'),
            'chi_square_threshold': 15.0,
            'direction_detection_samples': 10,
            # 重置参数（装甲板切换时会短暂丢失，不应重置）
            'max_lost_count': 30,  # 增加到30帧（约1秒）
            'reset_on_lost': False,  # 默认不重置，继续预测
        }]
    )
    
    # 新版可视化节点 (V2)
    visualizer_node = Node(
        package='armor_detector_ros2',
        executable='visualizer_node_v2',
        name='visualizer_node_v2',
        output='screen',
        parameters=[{
            'output_path': LaunchConfiguration('output_path'),
            'output_fps': LaunchConfiguration('output_fps'),
            'show_convergence_info': True,
            'show_armor_positions': True,
            'show_trajectory_prediction': True,
            'show_shootable_window': True,
            'show_window': LaunchConfiguration('show_window'),
            # 相机内参从统一的YAML文件加载
            'camera_intrinsics_path': default_camera_config,
        }]
    )
    
    # ==================== 返回 LaunchDescription ====================
    return LaunchDescription([
        # 参数声明
        video_path_arg,
        output_path_arg,
        fps_arg,
        output_fps_arg,
        loop_arg,
        show_window_arg,
        outpost_radius_arg,
        outpost_height_diff_arg,
        outpost_tilt_angle_arg,
        outpost_omega_arg,
        use_fixed_omega_arg,
        bullet_speed_arg,
        system_delay_arg,
        max_orientation_angle_arg,
        process_pos_noise_arg,
        process_theta_noise_arg,
        measurement_pos_noise_arg,
        measurement_angle_noise_arg,
        use_imu_file_arg,
        imu_file_path_arg,
        # 节点
        video_publisher_node,
        mock_imu_node,
        imu_file_publisher_node,
        detector_opensource_node,
        solver_node,
        outpost_predictor_node_v2,
        visualizer_node,
    ])
