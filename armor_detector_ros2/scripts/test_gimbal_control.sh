#!/bin/bash
# 云台控制节点测试脚本
# 使用 ros2 topic pub 手动发布云台指令

echo "================================================"
echo "      云台控制节点测试工具"
echo "================================================"
echo ""
echo "选择测试模式："
echo "1. 静态测试 (yaw=0°, pitch=0°)"
echo "2. 向左转 (yaw=+20°)"
echo "3. 向右转 (yaw=-20°)"
echo "4. 向上抬 (pitch=+15°)"
echo "5. 向下压 (pitch=-15°)"
echo "6. 自定义角度"
echo "7. 查看当前云台指令话题"
echo "8. 监听云台控制节点日志"
echo "0. 退出"
echo ""
read -p "请输入选择 [0-8]: " choice

case $choice in
    1)
        echo "发送静态测试指令: yaw=0°, pitch=0°"
        ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand \
        "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'body_frame'}, \
        yaw: 0.0, pitch: 0.0, aiming_state: 1, fire_allowed: false, \
        delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}"
        ;;
    2)
        echo "发送向左转指令: yaw=+20°"
        ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand \
        "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'body_frame'}, \
        yaw: 20.0, pitch: 0.0, aiming_state: 1, fire_allowed: false, \
        delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}"
        ;;
    3)
        echo "发送向右转指令: yaw=-20°"
        ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand \
        "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'body_frame'}, \
        yaw: -20.0, pitch: 0.0, aiming_state: 1, fire_allowed: false, \
        delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}"
        ;;
    4)
        echo "发送向上抬指令: pitch=+15°"
        ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand \
        "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'body_frame'}, \
        yaw: 0.0, pitch: 15.0, aiming_state: 1, fire_allowed: false, \
        delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}"
        ;;
    5)
        echo "发送向下压指令: pitch=-15°"
        ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand \
        "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'body_frame'}, \
        yaw: 0.0, pitch: -15.0, aiming_state: 1, fire_allowed: false, \
        delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}"
        ;;
    6)
        read -p "请输入yaw角度（度）: " yaw_val
        read -p "请输入pitch角度（度）: " pitch_val
        echo "发送自定义指令: yaw=${yaw_val}°, pitch=${pitch_val}°"
        ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand \
        "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'body_frame'}, \
        yaw: ${yaw_val}, pitch: ${pitch_val}, aiming_state: 1, fire_allowed: false, \
        delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}"
        ;;
    7)
        echo "查看云台指令话题内容..."
        ros2 topic echo /aiming/gimbal_command --once
        ;;
    8)
        echo "监听云台控制节点日志 (Ctrl+C 退出)..."
        ros2 node info /gimbal_control_node
        ;;
    0)
        echo "退出测试工具"
        exit 0
        ;;
    *)
        echo "无效选择"
        ;;
esac

echo ""
echo "================================================"
echo "提示："
echo "1. 确保云台控制节点正在运行"
echo "2. 检查串口连接: /dev/ttyACM0"
echo "3. 查看节点日志: ros2 node list"
echo "4. 检查话题: ros2 topic list | grep gimbal"
echo "================================================"
