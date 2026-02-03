"""
真实硬件完整检测+解算流水线 ROS2 启动文件

启动节点：
1. real_hardware_publisher_node: 从真实大恒相机采集图像，从串口读取IMU数据
2. detector_opensource_node: 进行装甲板检测（使用ONNX模型）
3. solver_node: PnP 位姿解算
4. outpost_predictor_node_v4: V4前哨站状态预测
5. visualizer_node_v4: 可视化检测、解算和预测结果

使用方法：
    ros2 launch armor_detector_ros2 real_hardware_pipeline_launch.py
    
    # 指定相机序列号和曝光时间
    ros2 launch armor_detector_ros2 real_hardware_pipeline_launch.py camera_sn:=FGK25050153 exposure_time:=3000
    
    # 指定输出视频路径
    ros2 launch armor_detector_ros2 real_hardware_pipeline_launch.py output_path:=/path/to/output.mp4
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('armor_detector_ros2')
    
    # 默认路径
    default_output = '/home/user/droneAim/TDrone/output/real_hardware_result.mp4'
    default_camera_config = os.path.join(pkg_share, 'config', 'camera_intrinsics.yaml')
    
    # ==================== 声明启动参数 ====================
    # 相机参数
    camera_sn_arg = DeclareLaunchArgument(
        'camera_sn',
        default_value='FGK25050153',
        description='大恒相机序列号'
    )
    
    exposure_time_arg = DeclareLaunchArgument(
        'exposure_time',
        default_value='3000',
        description='曝光时间 (μs)'
    )
    
    img_width_arg = DeclareLaunchArgument(
        'img_width',
        default_value='1280',
        description='图像宽度'
    )
    
    img_height_arg = DeclareLaunchArgument(
        'img_height',
        default_value='1024',
        description='图像高度'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='60.0',
        description='目标帧率'
    )
    
    # 串口参数
    serial_device_arg = DeclareLaunchArgument(
        'serial_device',
        default_value='/dev/ttyACM0',
        description='串口设备路径'
    )
    
    serial_baud_arg = DeclareLaunchArgument(
        'serial_baud',
        default_value='115200',
        description='串口波特率'
    )
    
    imu_rate_arg = DeclareLaunchArgument(
        'imu_rate',
        default_value='500.0',
        description='IMU发布频率 (Hz)'
    )
    
    # 输出和可视化参数
    output_path_arg = DeclareLaunchArgument(
        'output_path',
        default_value=default_output,
        description='输出视频文件路径'
    )
    
    output_fps_arg = DeclareLaunchArgument(
        'output_fps',
        default_value='30.0',
        description='输出视频帧率'
    )
    
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='false',
        description='是否实时显示可视化窗口'
    )
    
    enable_visualizer_arg = DeclareLaunchArgument(
        'enable_visualizer',
        default_value='true',
        description='是否启用可视化节点'
    )
    
    save_video_arg = DeclareLaunchArgument(
        'save_video',
        default_value='true',
        description='是否保存输出视频'
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
    
    # 真实硬件发布节点
    real_hardware_publisher_node = Node(
        package='armor_detector_ros2',
        executable='real_hardware_publisher_node',
        name='real_hardware_publisher',
        output='screen',
        parameters=[{
            'camera_sn': LaunchConfiguration('camera_sn'),
            'exposure_time': LaunchConfiguration('exposure_time'),
            'img_width': LaunchConfiguration('img_width'),
            'img_height': LaunchConfiguration('img_height'),
            'fps': LaunchConfiguration('fps'),
            'serial_device': LaunchConfiguration('serial_device'),
            'serial_baud': LaunchConfiguration('serial_baud'),
            'imu_rate': LaunchConfiguration('imu_rate'),
        }]
    )
    
    # 检测器节点
    detector_opensource_node = Node(
        package='armor_detector_ros2',
        executable='detector_opensource_node',
        name='detector_opensource_node',
        output='screen',
        parameters=[{
            'model_path': '/home/user/droneAim/TDrone/armor_detector_ros2/models/0526.onnx',
            'color_flag': -1,
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
            'init.direction_frames': 30,
            'init.cx_change_threshold': 30.0,
            'init.z_jump_threshold': 0.14,
            
            # 相位观测参数
            'phase.plateau_threshold': 2.35,
            'phase.plateau_exit_ratio': 0.80,
            'phase.plateau_confirm_ratio': 2.4,
            
            # EKF参数
            # EKF参数 - 调优说明:
            # sigma_*_sq: 初始协方差，值越大初始不确定性越高
            # q_*: 过程噪声，值越大越信任观测（状态变化快）
            # r_*: 观测噪声，值越大越信任预测（观测噪声大）
            'ekf.sigma_theta_sq': 5.0,
            'ekf.sigma_omega_sq': 0.1,
            'ekf.sigma_x_sq': 5.0,
            'ekf.sigma_y_sq': 5.0,
            'ekf.sigma_z_sq': 1.0,

            'ekf.q_theta': 1e-2,
            'ekf.q_omega': 1e-6,
            'ekf.q_x': 1e-5,
            'ekf.q_y': 1e-5,
            'ekf.q_z': 1e-4,

            'ekf.r_theta': 0.1,
            'ekf.r_x': 0.5,
            'ekf.r_y': 0.5,
            'ekf.r_z': 0.5,
            
            # 预瞄参数
            'aim.t_delay': LaunchConfiguration('system_delay'),
            'aim.filter_delay': 0.1,
            'aim.moving_delay': 0.2,
            'aim.v_bullet': LaunchConfiguration('bullet_speed'),
            'aim.angle_threshold_deg': 60.0,
            
            # 收敛参数
            'converge.ekf_frames': 100,
            'converge.height_verify_frames': 10.0,
            'converge.height_recovery_frames': 20,
            'converge.cov_omega_threshold': 0.1,
            'converge.cov_position_threshold': 0.05,
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
            'sync_tolerance_ms': 50.0,
            'save_video': LaunchConfiguration('save_video'),
            # 相机内参从统一的YAML文件加载
            'camera_intrinsics_path': default_camera_config,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_visualizer'))
    )
    
    # 瞄准控制节点
    aiming_node = Node(
        package='armor_detector_ros2',
        executable='aiming_node',
        name='aiming_node',
        parameters=[{
            # 滤波器参数
            'filter.window_size': 5,
            
            # 切换检测参数
            'switch.angle_jump_threshold': 60.0,
            'switch.yaw_match_threshold': 3.0,
            'switch.moving_delay': 0.2,
            
            # 重力补偿参数
            'gravity.g': 9.8,
            'gravity.bullet_speed': LaunchConfiguration('bullet_speed'),
            
            # 相机偏移（相机相对云台旋转中心的偏移，单位：米）
            'camera_offset.x': 0.0,
            'camera_offset.y': 0.0,
            'camera_offset.z': 0.0,
        }],
        output='screen'
    )
    
    # 云台控制节点
    gimbal_control_node = Node(
        package='armor_detector_ros2',
        executable='gimbal_control_node',
        name='gimbal_control_node',
        parameters=[{
            'serial_device': LaunchConfiguration('serial_device'),
            'serial_baud': LaunchConfiguration('serial_baud'),
            'send_rate': 100.0,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        # 启动参数
        camera_sn_arg,
        exposure_time_arg,
        img_width_arg,
        img_height_arg,
        fps_arg,
        serial_device_arg,
        serial_baud_arg,
        imu_rate_arg,
        output_path_arg,
        output_fps_arg,
        show_window_arg,
        enable_visualizer_arg,
        save_video_arg,
        outpost_radius_arg,
        outpost_height_diff_arg,
        armor_arrangement_reversed_arg,
        outpost_omega_arg,
        bullet_speed_arg,
        system_delay_arg,
        
        # 节点
        real_hardware_publisher_node,
        detector_opensource_node,
        solver_node,
        outpost_predictor_node,
        visualizer_node,
        aiming_node,
        gimbal_control_node,
    ])
