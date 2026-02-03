# 云台控制节点 Docker 测试指南

## 概述
ROS2 节点在 Docker 容器中运行，测试脚本已适配 Docker 环境。

## 测试工具

### 1. Bash 交互式测试脚本
**文件**: `test_gimbal_control_docker.sh`

**特点**:
- 自动检测是否在 Docker 容器内运行
- 支持从宿主机或容器内部执行
- 交互式菜单选择测试模式

**使用方法**:
```bash
# 从宿主机运行（推荐）
cd /home/hustlyrm/droneAim26/TDrone/armor_detector_ros2/scripts
sudo bash test_gimbal_control_docker.sh

# 或从容器内部运行
docker exec -it ros2-vision-dev bash
cd /home/user/droneAim/TDrone/armor_detector_ros2/scripts
./test_gimbal_control_docker.sh
```

### 2. Python 持续测试脚本
**文件**: `test_gimbal_control.py`

**特点**:
- 100Hz 持续发布
- 4种测试模式：static, sweep, circle, manual
- 需要在 Docker 容器内运行

**使用方法**:
```bash
# 进入容器
docker exec -it ros2-vision-dev bash

# 运行脚本
cd /home/user/droneAim/TDrone/armor_detector_ros2/scripts
source /opt/ros/humble/setup.bash
source /home/user/droneAim/TDrone/install/setup.bash
python3 test_gimbal_control.py
```

## 完整测试流程

### 步骤 1: 启动 Docker 容器
```bash
cd /home/hustlyrm/droneAim26/TDrone
sudo bash docker_run.sh
# 选择 d1) 启动 Docker 容器
```

### 步骤 2: 在容器中启动云台控制节点
```bash
# 方法 1: 使用 docker_run.sh 菜单
# 选择 r2) 真实硬件完整流水线

# 方法 2: 手动启动单个节点
docker exec -it ros2-vision-dev bash
source /opt/ros/humble/setup.bash
source /home/user/droneAim/TDrone/install/setup.bash
ros2 run armor_detector_ros2 gimbal_control_node
```

### 步骤 3: 运行测试脚本
```bash
# 终端 2 - 运行 Bash 测试脚本（推荐）
cd /home/hustlyrm/droneAim26/TDrone/armor_detector_ros2/scripts
sudo bash test_gimbal_control_docker.sh

# 或运行 Python 测试脚本
docker exec -it ros2-vision-dev bash
cd /home/user/droneAim/TDrone/armor_detector_ros2/scripts
source /opt/ros/humble/setup.bash
source /home/user/droneAim/TDrone/install/setup.bash
python3 test_gimbal_control.py
```

### 步骤 4: 验证结果
```bash
# 查看话题是否发布
docker exec -it ros2-vision-dev bash -c "source /opt/ros/humble/setup.bash && source /home/user/droneAim/TDrone/install/setup.bash && ros2 topic list | grep gimbal"

# 查看话题内容
docker exec -it ros2-vision-dev bash -c "source /opt/ros/humble/setup.bash && source /home/user/droneAim/TDrone/install/setup.bash && ros2 topic echo /aiming/gimbal_command --once"

# 查看话题频率
docker exec -it ros2-vision-dev bash -c "source /opt/ros/humble/setup.bash && source /home/user/droneAim/TDrone/install/setup.bash && ros2 topic hz /aiming/gimbal_command"

# 查看节点信息
docker exec -it ros2-vision-dev bash -c "source /opt/ros/humble/setup.bash && source /home/user/droneAim/TDrone/install/setup.bash && ros2 node info /gimbal_control_node"
```

## 常见问题排查

### 1. 容器未运行
**问题**: 执行脚本时提示"Docker 容器未运行"
**解决**:
```bash
cd /home/hustlyrm/droneAim26/TDrone
sudo bash docker_run.sh
# 选择 d1) 启动容器
```

### 2. 串口权限问题
**问题**: 容器无法访问 /dev/ttyACM0
**解决**:
```bash
# 检查串口设备
ls -l /dev/ttyACM*

# 修改权限
sudo chmod 666 /dev/ttyACM0

# 或添加到 dialout 组
sudo usermod -aG dialout $USER
```

### 3. ROS2 环境未设置
**问题**: ros2 命令找不到
**解决**: 确保在容器内执行命令前已 source 环境
```bash
source /opt/ros/humble/setup.bash
source /home/user/droneAim/TDrone/install/setup.bash
```

### 4. 云台不响应
**问题**: 发送指令但云台不动
**检查清单**:
1. 云台控制节点是否运行：`ros2 node list | grep gimbal`
2. 串口连接是否正常：检查 /dev/ttyACM0
3. 波特率是否匹配：默认 115200
4. 是否启用控制：`ros2 service call /gimbal_control/pipeline_control std_srvs/srv/SetBool "{data: true}"`
5. 是否收到消息：查看节点日志

## 测试模式说明

### Bash 脚本测试模式
1. **静态测试**: yaw=0°, pitch=0° - 测试回中
2. **向左转**: yaw=+20° - 测试左转
3. **向右转**: yaw=-20° - 测试右转
4. **向上抬**: pitch=+15° - 测试俯仰上
5. **向下压**: pitch=-15° - 测试俯仰下
6. **自定义角度**: 手动输入 yaw 和 pitch

### Python 脚本测试模式
1. **static**: 静态保持 (yaw=10°, pitch=5°)
2. **sweep**: 左右扫描 (yaw: -30° ~ +30°)
3. **circle**: 圆周运动 (yaw 和 pitch 按正弦曲线变化)
4. **manual**: 手动输入角度

## 调试技巧

### 1. 查看所有节点
```bash
docker exec -it ros2-vision-dev bash -c "source /opt/ros/humble/setup.bash && source /home/user/droneAim/TDrone/install/setup.bash && ros2 node list"
```

### 2. 查看所有话题
```bash
docker exec -it ros2-vision-dev bash -c "source /opt/ros/humble/setup.bash && source /home/user/droneAim/TDrone/install/setup.bash && ros2 topic list"
```

### 3. 实时监控话题
```bash
docker exec -it ros2-vision-dev bash -c "source /opt/ros/humble/setup.bash && source /home/user/droneAim/TDrone/install/setup.bash && ros2 topic echo /aiming/gimbal_command"
```

### 4. 进入容器调试
```bash
docker exec -it ros2-vision-dev bash
cd /home/user/droneAim/TDrone
source /opt/ros/humble/setup.bash
source install/setup.bash

# 然后可以执行各种 ros2 命令
ros2 node list
ros2 topic list
ros2 topic echo /aiming/gimbal_command
```

## 注意事项

1. **权限**: 从宿主机运行脚本可能需要 sudo
2. **串口映射**: 确保 docker-compose.yml 中正确映射了串口设备
3. **环境变量**: 容器内必须先 source ROS2 环境
4. **网络**: 确保容器网络配置允许 ROS2 通信
5. **日志**: 查看容器日志以诊断问题：`docker logs ros2-vision-dev`
