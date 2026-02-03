"""
前哨站预测流水线 V3 ROS2 启动文件

启动节点：
1. video_publisher_node: 从视频文件读取帧并发布
2. imu_file_publisher_node / mock_imu_node: IMU 数据（文件或模拟）
3. detector_node: 进行装甲板关键点检测和数字分类
4. solver_node: PnP 位姿解算
5. outpost_predictor_node_v3: 新版前哨站状态预测 (极坐标观测模型)
6. visualizer_node: 可视化检测、解算和预测结果

使用方法：
    ros2 launch armor_detector_ros2 outpost_pipeline_v3_launch.py
    
    # 使用自定义视频文件：
    ros2 launch armor_detector_ros2 outpost_pipeline_v3_launch.py video_path:=/path/to/video.mp4
    
    # 使用录制的 IMU 文件：
    ros2 launch armor_detector_ros2 outpost_pipeline_v3_launch.py \\
        use_imu_file:=true \\
        imu_file_path:=/home/user/Videos/drone_camera_xxx.txt
    
    # 调整前哨站参数：
    ros2 launch armor_detector_ros2 outpost_pipeline_v3_launch.py \
        outpost_radius:=0.2767 \
        outpost_omega:=2.513 \
        bullet_speed:=26.0

V3 预测器特性 (相比 V2 改进):
- 极坐标观测模型 [yaw, pitch, distance, orientation]：匹配相机测量特性
- 状态向量 [θ_center, ω, x_c, y_c, z_c]：角度优先，便于预测
- Mahalanobis距离的多假设置信度更新：更鲁棒的假设选择
- 装甲板切换检测：处理观测目标跳变
- Direct/Indirect 瞄准策略：选择最优击打方案
- 正确的坐标系约定：θ=0时HIGH装甲板正对相机，ω>0逆时针
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
    default_output = '/home/user/droneAim/TDrone/output/outpost_v3_result.mp4'
    default_config = '/home/user/droneAim/TDrone/armor_detector_ros2/config/outpost_predictor_v3_config.yaml'
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
    
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config,
        description='V3预测器配置文件路径'
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
    # 根据用户提供的规格: 276.7mm半径, 100mm高度差, 75°倾斜角
    outpost_radius_arg = DeclareLaunchArgument(
        'outpost_radius',
        default_value='0.2767',
        description='前哨站装甲板旋转半径 (m) - 标准值276.7mm'
    )
    
    outpost_height_diff_arg = DeclareLaunchArgument(
        'outpost_height_diff',
        default_value='0.1',
        description='装甲板高度差 (m) - 标准值100mm (HIGH-MID-LOW排列)'
    )
    
    outpost_tilt_angle_arg = DeclareLaunchArgument(
        'outpost_tilt_angle',
        default_value='1.309',  # 75° = 75 * π / 180 ≈ 1.309 rad
        description='装甲板倾斜角 (rad) - 标准值75度'
    )
    
    # ==================== 前哨站运动参数 ====================
    outpost_omega_arg = DeclareLaunchArgument(
        'outpost_omega',
        default_value='2.513',  # 0.8π rad/s ≈ 2.513 rad/s
        description='前哨站已知角速度 (rad/s) - 标准值0.8π rad/s'
    )
    
    use_fixed_omega_arg = DeclareLaunchArgument(
        'use_fixed_omega',
        default_value='true',
        description='是否使用固定角速度（标准前哨站为true）'
    )
    
    # ==================== 弹道参数 ====================
    bullet_speed_arg = DeclareLaunchArgument(
        'bullet_speed',
        default_value='26.0',
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
        default_value='0.65',  # 约37度
        description='最大可打角度 (rad)，约37度'
    )
    
    aim_advance_time_arg = DeclareLaunchArgument(
        'aim_advance_time',
        default_value='0.15',
        description='瞄准提前量 (s)，indirect模式提前瞄准时间'
    )
    
    # ==================== 噪声参数 (极坐标观测) ====================
    q_theta_arg = DeclareLaunchArgument(
        'q_theta',
        default_value='0.02',
        description='中心角度过程噪声'
    )
    
    q_omega_arg = DeclareLaunchArgument(
        'q_omega',
        default_value='0.01',
        description='角速度过程噪声'
    )
    
    q_position_arg = DeclareLaunchArgument(
        'q_position',
        default_value='0.005',
        description='中心位置过程噪声'
    )
    
    r_yaw_arg = DeclareLaunchArgument(
        'r_yaw',
        default_value='0.02',
        description='yaw角观测噪声 (rad)'
    )
    
    r_pitch_arg = DeclareLaunchArgument(
        'r_pitch',
        default_value='0.02',
        description='pitch角观测噪声 (rad)'
    )
    
    r_distance_arg = DeclareLaunchArgument(
        'r_distance',
        default_value='0.05',
        description='距离观测噪声 (m)'
    )
    
    r_orientation_arg = DeclareLaunchArgument(
        'r_orientation',
        default_value='0.05',
        description='装甲板朝向观测噪声 (rad)'
    )
    
    # ==================== 多假设参数 ====================
    hypothesis_converge_threshold_arg = DeclareLaunchArgument(
        'hypothesis_converge_threshold',
        default_value='0.8',
        description='假设收敛置信度阈值'
    )
    
    min_updates_to_converge_arg = DeclareLaunchArgument(
        'min_updates_to_converge',
        default_value='15',
        description='判定收敛的最小更新次数'
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
            'camera_intrinsics_path': default_camera_config,  # 加载优化后的内参
            'armor_width': 0.135,
            'armor_height': 0.055,
        }]
    )
    
    # V3 前哨站预测器节点 (极坐标观测模型)
    outpost_predictor_node_v3 = Node(
        package='armor_detector_ros2',
        executable='outpost_predictor_node_v3',
        name='outpost_predictor_v3',
        output='screen',
        parameters=[{
            # 配置文件路径
            'config_path': LaunchConfiguration('config_path'),
            # 几何参数
            'geometry.radius': LaunchConfiguration('outpost_radius'),
            'geometry.height_diff': LaunchConfiguration('outpost_height_diff'),
            'geometry.tilt_angle': LaunchConfiguration('outpost_tilt_angle'),
            # 运动参数
            'known_omega': LaunchConfiguration('outpost_omega'),
            'use_fixed_omega': LaunchConfiguration('use_fixed_omega'),
            # 过程噪声 (状态: θ_center, ω, x_c, y_c, z_c)
            'noise.q_theta': LaunchConfiguration('q_theta'),
            'noise.q_omega': LaunchConfiguration('q_omega'),
            'noise.q_position': LaunchConfiguration('q_position'),
            # 观测噪声 (极坐标: yaw, pitch, distance, orientation)
            'noise.r_yaw': LaunchConfiguration('r_yaw'),
            'noise.r_pitch': LaunchConfiguration('r_pitch'),
            'noise.r_distance': LaunchConfiguration('r_distance'),
            'noise.r_orientation': LaunchConfiguration('r_orientation'),
            # 弹道参数
            'bullet_speed': LaunchConfiguration('bullet_speed'),
            'system_delay': LaunchConfiguration('system_delay'),
            # 瞄准参数
            'max_orientation_angle': LaunchConfiguration('max_orientation_angle'),
            'aim_advance_time': LaunchConfiguration('aim_advance_time'),
            # 多假设参数
            'hypothesis_converge_threshold': LaunchConfiguration('hypothesis_converge_threshold'),
            'min_updates_to_converge': LaunchConfiguration('min_updates_to_converge'),
            # 重置参数
            'max_lost_count': 30,
            'reset_on_lost': False,
        }]
    )
    
    # V3 可视化节点 (时间同步版本，解决检测框滞后问题)
    visualizer_node = Node(
        package='armor_detector_ros2',
        executable='visualizer_node_v3',
        name='visualizer_node_v3',
        output='screen',
        parameters=[{
            'output_path': LaunchConfiguration('output_path'),
            'output_fps': LaunchConfiguration('output_fps'),
            'show_predictor_info': True,
            'show_aim_point': True,
            'show_trajectory': True,
            'show_window': LaunchConfiguration('show_window'),
            'sync_tolerance_ms': 50.0,  # 时间同步容差
            # 相机内参从统一的YAML文件加载
            'camera_intrinsics_path': default_camera_config,
        }]
    )
    
    # ==================== 返回 LaunchDescription ====================
    return LaunchDescription([
        # 参数声明
        video_path_arg,
        output_path_arg,
        config_path_arg,
        fps_arg,
        output_fps_arg,
        loop_arg,
        show_window_arg,
        # 几何参数
        outpost_radius_arg,
        outpost_height_diff_arg,
        outpost_tilt_angle_arg,
        # 运动参数
        outpost_omega_arg,
        use_fixed_omega_arg,
        # 弹道参数
        bullet_speed_arg,
        system_delay_arg,
        # 瞄准参数
        max_orientation_angle_arg,
        aim_advance_time_arg,
        # 噪声参数
        q_theta_arg,
        q_omega_arg,
        q_position_arg,
        r_yaw_arg,
        r_pitch_arg,
        r_distance_arg,
        r_orientation_arg,
        # 多假设参数
        hypothesis_converge_threshold_arg,
        min_updates_to_converge_arg,
        # IMU 参数
        use_imu_file_arg,
        imu_file_path_arg,
        # 节点
        video_publisher_node,
        mock_imu_node,
        imu_file_publisher_node,
        detector_opensource_node,
        solver_node,
        outpost_predictor_node_v3,
        visualizer_node,
    ])
