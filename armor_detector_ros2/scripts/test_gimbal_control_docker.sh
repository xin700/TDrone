#!/bin/bash
# 云台控制节点测试脚本 (Docker 版本)
# 使用 ros2 topic pub 手动发布云台指令
# 支持 Docker 环境

# Docker 容器名称
CONTAINER_NAME="ros2-vision-dev"

# 检查是否在 Docker 容器中运行
if [ -f /.dockerenv ]; then
    # 在容器内部，直接使用 ros2 命令
    ROS2_CMD="ros2"
    echo "[INFO] 检测到在 Docker 容器内运行"
else
    # 在宿主机上，检查容器是否运行
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        ROS2_CMD="docker exec -it ${CONTAINER_NAME} bash -c"
        echo "[INFO] 检测到 Docker 容器 ${CONTAINER_NAME} 正在运行"
        echo "[INFO] 将通过 Docker 容器执行命令"
    else
        echo "[ERROR] Docker 容器 ${CONTAINER_NAME} 未运行"
        echo "请先启动容器: cd /home/hustlyrm/droneAim26/TDrone && sudo bash docker_run.sh"
        exit 1
    fi
fi

echo "================================================"
echo "      云台控制节点测试工具 (Docker)"
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
echo "8. 监听云台控制节点信息"
echo "9. 查看话题发布频率"
echo "10. 持续监听云台控制节点接收的消息 (Ctrl+C 退出)"
echo "0. 退出"
echo ""
read -p "请输入选择 [0-10]: " choice

# 构建 ROS2 环境设置命令
if [ -f /.dockerenv ]; then
    # 容器内部
    ROS_SETUP="source /opt/ros/humble/setup.bash && source /home/user/droneAim/TDrone/install/setup.bash &&"
else
    # 宿主机上，需要在 docker exec 中设置环境
    ROS_SETUP="source /opt/ros/humble/setup.bash && source /home/user/droneAim/TDrone/install/setup.bash &&"
fi

case $choice in
    1)
        echo "发送静态测试指令: yaw=0°, pitch=0°"
        if [ -f /.dockerenv ]; then
            $ROS_SETUP ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand \
            "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'body_frame'}, \
            yaw: 0.0, pitch: 0.0, aiming_state: 1, fire_allowed: false, \
            delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}"
        else
            docker exec -it ${CONTAINER_NAME} bash -c "$ROS_SETUP ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"body_frame\"}, yaw: 0.0, pitch: 0.0, aiming_state: 1, fire_allowed: false, delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}'"
        fi
        ;;
    2)
        echo "发送向左转指令: yaw=+20°"
        if [ -f /.dockerenv ]; then
            $ROS_SETUP ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand \
            "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'body_frame'}, \
            yaw: 20.0, pitch: 0.0, aiming_state: 1, fire_allowed: false, \
            delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}"
        else
            docker exec -it ${CONTAINER_NAME} bash -c "$ROS_SETUP ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"body_frame\"}, yaw: 20.0, pitch: 0.0, aiming_state: 1, fire_allowed: false, delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}'"
        fi
        ;;
    3)
        echo "发送向右转指令: yaw=-20°"
        if [ -f /.dockerenv ]; then
            $ROS_SETUP ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand \
            "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'body_frame'}, \
            yaw: -20.0, pitch: 0.0, aiming_state: 1, fire_allowed: false, \
            delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}"
        else
            docker exec -it ${CONTAINER_NAME} bash -c "$ROS_SETUP ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"body_frame\"}, yaw: -20.0, pitch: 0.0, aiming_state: 1, fire_allowed: false, delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}'"
        fi
        ;;
    4)
        echo "发送向上抬指令: pitch=+15°"
        if [ -f /.dockerenv ]; then
            $ROS_SETUP ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand \
            "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'body_frame'}, \
            yaw: 0.0, pitch: 15.0, aiming_state: 1, fire_allowed: false, \
            delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}"
        else
            docker exec -it ${CONTAINER_NAME} bash -c "$ROS_SETUP ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"body_frame\"}, yaw: 0.0, pitch: 15.0, aiming_state: 1, fire_allowed: false, delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}'"
        fi
        ;;
    5)
        echo "发送向下压指令: pitch=-15°"
        if [ -f /.dockerenv ]; then
            $ROS_SETUP ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand \
            "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'body_frame'}, \
            yaw: 0.0, pitch: -15.0, aiming_state: 1, fire_allowed: false, \
            delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}"
        else
            docker exec -it ${CONTAINER_NAME} bash -c "$ROS_SETUP ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"body_frame\"}, yaw: 0.0, pitch: -15.0, aiming_state: 1, fire_allowed: false, delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}'"
        fi
        ;;
    6)
        read -p "请输入yaw角度（度）: " yaw_val
        read -p "请输入pitch角度（度）: " pitch_val
        echo "发送自定义指令: yaw=${yaw_val}°, pitch=${pitch_val}°"
        if [ -f /.dockerenv ]; then
            $ROS_SETUP ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand \
            "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'body_frame'}, \
            yaw: ${yaw_val}, pitch: ${pitch_val}, aiming_state: 1, fire_allowed: false, \
            delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}"
        else
            docker exec -it ${CONTAINER_NAME} bash -c "$ROS_SETUP ros2 topic pub --once /aiming/gimbal_command armor_detector_ros2/msg/GimbalCommand '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"body_frame\"}, yaw: ${yaw_val}, pitch: ${pitch_val}, aiming_state: 1, fire_allowed: false, delta_yaw: 0.0, delta_pitch: 0.0, distance: 5.0, gravity_compensation: 0.0}'"
        fi
        ;;
    7)
        echo "查看云台指令话题内容..."
        if [ -f /.dockerenv ]; then
            $ROS_SETUP ros2 topic echo /aiming/gimbal_command --once
        else
            docker exec -it ${CONTAINER_NAME} bash -c "$ROS_SETUP ros2 topic echo /aiming/gimbal_command --once"
        fi
        ;;
    8)
        echo "监听云台控制节点信息..."
        if [ -f /.dockerenv ]; then
            $ROS_SETUP ros2 node info /gimbal_control_node
        else
            docker exec -it ${CONTAINER_NAME} bash -c "$ROS_SETUP ros2 node info /gimbal_control_node"
        fi
        ;;
    9)
        echo "查看话题发布频率 (Ctrl+C 退出)..."
        if [ -f /.dockerenv ]; then
            $ROS_SETUP ros2 topic hz /aiming/gimbal_command
        else
            docker exec -it ${CONTAINER_NAME} bash -c "$ROS_SETUP ros2 topic hz /aiming/gimbal_command"
        fi
        ;;
    10)
        echo "================================================"
        echo "持续监听云台控制节点接收的消息"
        echo "================================================"
        echo "按 Ctrl+C 停止监听"
        echo ""
        if [ -f /.dockerenv ]; then
            $ROS_SETUP ros2 topic echo /aiming/gimbal_command
        else
            docker exec -it ${CONTAINER_NAME} bash -c "$ROS_SETUP ros2 topic echo /aiming/gimbal_command"
        fi
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
echo "1. 确保云台控制节点正在运行 (在 Docker 容器中)"
echo "2. 检查串口连接: /dev/ttyACM0"
if [ -f /.dockerenv ]; then
    echo "3. 查看节点: ros2 node list"
    echo "4. 检查话题: ros2 topic list | grep gimbal"
else
    echo "3. 进入容器: docker exec -it ${CONTAINER_NAME} bash"
    echo "4. 或使用 docker_run.sh 菜单启动节点"
fi
echo "================================================"
