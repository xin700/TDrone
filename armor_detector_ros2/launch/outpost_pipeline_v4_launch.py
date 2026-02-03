"""
前哨站预测流水线 V4 ROS2 启动文件

启动节点：
1. video_publisher_node: 从视频文件读取帧并发布
2. imu_file_publisher_node / mock_imu_node: IMU 数据（文件或模拟）
3. detector_node: 进行装甲板关键点检测和数字分类
4. solver_node: PnP 位姿解算
5. outpost_predictor_node_v4: V4前哨站状态预测 (基于长宽比的相位观测)
6. visualizer_node: 可视化检测、解算和预测结果

使用方法：
    ros2 launch armor_detector_ros2 outpost_pipeline_v4_launch.py
    
    # 使用自定义视频文件：
    ros2 launch armor_detector_ros2 outpost_pipeline_v4_launch.py video_path:=/path/to/video.mp4
    
    # 使用录制的 IMU 文件：
    ros2 launch armor_detector_ros2 outpost_pipeline_v4_launch.py \\
        use_imu_file:=true \\
        imu_file_path:=/home/user/Videos/drone_camera_xxx.txt

V4 预测器特性:
- 基于长宽比的θ观测: θ_abs = arccos(aspect_ratio / 2.58)
- 分阶段初始化: 方向→高度→θ→EKF
- 5维状态空间: [ang, ω, x_c, y_c, z_c]
- 4维观测空间: [θ, x_w, y_w, z_w]
- 高度状态机与EKF交叉验证
- 平台区cx插值θ计算
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
    default_output = '/home/user/droneAim/TDrone/output/outpost_v4_result.mp4'
    default_config = '/home/user/droneAim/TDrone/armor_detector_ros2/config/outpost_predictor_v4_config.yaml'
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
        description='V4预测器配置文件路径'
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
        description='是否实时显示可视化窗口（SSH环境建议false）'
    )
    
    # IMU 相关参数
    use_imu_file_arg = DeclareLaunchArgument(
        'use_imu_file',
        default_value='false',
        description='是否使用IMU文件而非模拟IMU'
    )
    
    imu_file_path_arg = DeclareLaunchArgument(
        'imu_file_path',
        default_value='',
        description='IMU数据文件路径'
    )
    
    # 前哨站几何参数
    outpost_radius_arg = DeclareLaunchArgument(
        'outpost_radius',
        default_value='0.2767',
        description='前哨站旋转半径 (m)'
    )
    
    outpost_height_diff_arg = DeclareLaunchArgument(
        'outpost_height_diff',
        default_value='0.10',
        description='装甲板高度差 (m)'
    )
    
    armor_arrangement_reversed_arg = DeclareLaunchArgument(
        'armor_arrangement_reversed',
        default_value='false',
        description='装甲板排列是否反向：false=高-中-低(默认), true=低-中-高(新前哨站)'
    )
    
    # 运动参数
    outpost_omega_arg = DeclareLaunchArgument(
        'outpost_omega',
        default_value='2.5133',
        description='前哨站角速度 (rad/s)'
    )
    
    # 预瞄参数
    bullet_speed_arg = DeclareLaunchArgument(
        'bullet_speed',
        default_value='28.0',
        description='子弹速度 (m/s)'
    )
    
    system_delay_arg = DeclareLaunchArgument(
        'system_delay',
        default_value='0.05',
        description='系统延迟 (s)'
    )
    
    # ==================== 节点定义 ====================
    
    # 视频发布节点
    video_publisher_node = Node(
        package='armor_detector_ros2',
        executable='video_publisher_node',
        name='video_publisher',
        parameters=[{
            'video_path': LaunchConfiguration('video_path'),
            'fps': LaunchConfiguration('fps'),
            'loop': LaunchConfiguration('loop'),
        }],
        output='screen'
    )
    
    # 模拟 IMU 节点（当不使用IMU文件时）
    mock_imu_node = Node(
        package='armor_detector_ros2',
        executable='mock_imu_node',
        name='mock_imu',
        parameters=[{
            'yaw': 0.0,
            'pitch': 0.0,
            'roll': 0.0,
        }],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_imu_file'))
    )
    
    # IMU 文件发布节点（当使用IMU文件时）
    imu_file_publisher_node = Node(
        package='armor_detector_ros2',
        executable='imu_file_publisher_node',
        name='imu_file_publisher',
        parameters=[{
            'imu_file_path': LaunchConfiguration('imu_file_path'),
        }],
        output='screen',
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
        name='solver',
        parameters=[{
            'camera_intrinsics_path': default_camera_config,
        }],
        output='screen'
    )
    
    # V4 前哨站预测节点
    outpost_predictor_node = Node(
        package='armor_detector_ros2',
        executable='outpost_predictor_node_v4',
        name='outpost_predictor_v4',
        parameters=[{
            # 几何参数
            'geometry.radius': LaunchConfiguration('outpost_radius'),
            'geometry.height_diff': LaunchConfiguration('outpost_height_diff'),
            'geometry.tilt_angle_deg': 75.0,
            'geometry.max_aspect_ratio': 2.58,
            'geometry.armor_arrangement_reversed': LaunchConfiguration('armor_arrangement_reversed'),
            
            # 运动参数
            'motion.standard_omega': LaunchConfiguration('outpost_omega'),
            
            # 初始化参数
            'init.direction_frames': 10,
            'init.cx_change_threshold': 30.0,
            'init.z_jump_threshold': 0.14,
            
            # 相位观测参数
            'phase.plateau_threshold': 2.2,
            'phase.plateau_exit_ratio': 0.80,
            'phase.plateau_confirm_ratio': 2.4,
            
            # EKF参数 - 调优说明:
            # sigma_*_sq: 初始协方差，值越大初始不确定性越高
            # q_*: 过程噪声，值越大越信任观测（状态变化快）
            # r_*: 观测噪声，值越大越信任预测（观测噪声大）
            
            # 初始协方差
            'ekf.sigma_theta_sq': 0.1,      # theta初始不确定性
            'ekf.sigma_omega_sq': 0.01,    # omega初始不确定性（更小，转速基本已知）
            'ekf.sigma_x_sq': 1.0,          # 位置初始不确定性（减小，位置解算更准了）
            'ekf.sigma_y_sq': 1.0,
            'ekf.sigma_z_sq': 10.0,
            
            # 过程噪声 - 前哨站中心几乎不动，转速恒定
            'ekf.q_theta': 1e-2,            # theta变化（由omega驱动）
            'ekf.q_omega': 1e-6,            # omega变化极小（大幅降低！转速恒定）
            'ekf.q_x': 1e-5,                # 中心位置几乎不变
            'ekf.q_y': 1e-5,
            'ekf.q_z': 1e-4,
            
            # 观测噪声 - 关键修改！位置观测现在更准了
            'ekf.r_theta': 0.05,             # theta观测噪声（大幅增大！不要让theta主导更新）
            'ekf.r_x': 0.2,                # 位置观测噪声（大幅减小！更信任位置观测）
            'ekf.r_y': 0.2,
            'ekf.r_z': 1.0,
            
            # 预瞄参数
            'aim.t_delay': LaunchConfiguration('system_delay'),
            'aim.v_bullet': LaunchConfiguration('bullet_speed'),
            'aim.angle_threshold_deg': 60.0,
            
            # 收敛参数
            # ekf_frames: 最小帧数要求，实际收敛还需满足协方差和残差条件
            # - 建议：100-200帧（约3-6秒）确保充分观测前哨站旋转
            # - 太小：可能在状态还在调整时就判定收敛
            # - 太大：延迟射击时机
            'converge.ekf_frames': 100,  # 增加到150帧，确保至少观测1.5圈以上
            'converge.height_verify_frames': 10.0,  # double 类型
            'converge.height_recovery_frames': 20,            
            # 协方差收敛阈值（相对于初始值的比例）
            # - cov_omega_threshold: omega协方差需降到初始值的多少倍以下
            #   值越小要求越严格（收敛越慢），建议范围 0.05-0.2
            # - cov_position_threshold: 位置协方差需降到初始值的多少倍以下
            #   值越小要求越严格（收敛越慢），建议范围 0.01-0.1
            'converge.cov_omega_threshold': 0.1,      # omega降到初始的10%
            'converge.cov_position_threshold': 0.05,  # 位置降到初始的5%        
            }],
        output='screen'
    )
    
    # V4 可视化节点
    visualizer_node = Node(
        package='armor_detector_ros2',
        executable='visualizer_node_v4',
        name='visualizer_v4',
        parameters=[{
            'output_path': LaunchConfiguration('output_path'),
            'output_fps': LaunchConfiguration('output_fps'),
            'show_window': LaunchConfiguration('show_window'),
            'show_predictor_info': True,
            'show_aim_point': True,
            'show_trajectory': True,
            'show_imu_data': True,
            'save_video': True,
            'sync_tolerance_ms': 50.0,
            # 相机内参从统一的YAML文件加载
            'camera_intrinsics_path': default_camera_config,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        # 启动参数
        video_path_arg,
        output_path_arg,
        config_path_arg,
        fps_arg,
        output_fps_arg,
        loop_arg,
        show_window_arg,
        use_imu_file_arg,
        imu_file_path_arg,
        outpost_radius_arg,
        outpost_height_diff_arg,
        armor_arrangement_reversed_arg,
        outpost_omega_arg,
        bullet_speed_arg,
        system_delay_arg,
        
        # 节点
        video_publisher_node,
        mock_imu_node,
        imu_file_publisher_node,
        detector_opensource_node,
        solver_node,
        outpost_predictor_node,
        visualizer_node,
    ])
