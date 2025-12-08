#!/bin/bash
# ==============================================================================
# Docker 环境运行脚本
# ==============================================================================
# 功能：
#   - 在 Docker 容器内编译和运行 ROS2 包
#   - 支持 GUI 可视化
# ==============================================================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# 配置
WS_PATH="/home/xin/ROS2Drone/ws"
CONTAINER_NAME="ros2-vision-dev"
CONTAINER_GUI="ros2-vision-gui"
PKG_NAME="armor_detector_ros2"

# 默认视频
DEFAULT_VIDEO="/home/xin/ROS2Drone/ws/videos/3.avi"

# ==============================================================================
# 函数定义
# ==============================================================================

show_header() {
    clear
    echo -e "${CYAN}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║         ROS2 Vision Docker 运行环境                          ║"
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║  在 Docker 容器内编译和运行，确保环境一致性                  ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

show_menu() {
    echo -e "${YELLOW}请选择操作：${NC}"
    echo ""
    echo -e "  ${CYAN}--- Docker 管理 ---${NC}"
    echo -e "  ${GREEN}1)${NC} 启动 Docker 容器"
    echo -e "  ${GREEN}2)${NC} 停止 Docker 容器"
    echo -e "  ${GREEN}3)${NC} 查看容器状态"
    echo -e "  ${GREEN}4)${NC} 进入容器 Shell"
    echo -e "  ${CYAN}--- 编译 ---${NC}"
    echo -e "  ${GREEN}5)${NC} 在 Docker 内编译 (Release)"
    echo -e "  ${GREEN}6)${NC} 在 Docker 内编译 (Debug + 测试)"
    echo -e "  ${GREEN}7)${NC} 清理 Docker 内的构建文件"
    echo -e "  ${CYAN}--- 运行 ---${NC}"
    echo -e "  ${GREEN}8)${NC} 运行完整流水线 (Docker 内)"
    echo -e "  ${GREEN}9)${NC} 运行测试 (Docker 内)"
    echo -e "  ${GREEN}e)${NC} 运行 EKF 可视化演示 (Docker 内)"
    echo -e "  ${GREEN}o)${NC} 运行前哨站预测可视化 (Docker 内) ${YELLOW}[新功能]${NC}"
    echo -e "  ${CYAN}--- 其他 ---${NC}"
    echo -e "  ${GREEN}s)${NC} 查看系统信息"
    echo -e "  ${GREEN}0)${NC} 退出"
    echo ""
    echo -n -e "${CYAN}请输入选项: ${NC}"
}

# 检查 Docker 是否运行
check_docker() {
    if ! docker info > /dev/null 2>&1; then
        echo -e "${RED}[ERROR] Docker 未运行，请先启动 Docker${NC}"
        return 1
    fi
    return 0
}

# 检查容器是否运行
check_container() {
    local container="$1"
    if docker ps --format '{{.Names}}' | grep -q "^${container}$"; then
        return 0
    fi
    return 1
}

# 启动容器
start_container() {
    check_docker || return 1
    
    echo -e "${BLUE}[INFO] 启动 Docker 容器...${NC}"
    cd "${WS_PATH}"
    
    # 允许 X11 连接
    xhost +local:docker 2>/dev/null || true
    
    docker-compose up -d ros2-vision
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[SUCCESS] 容器已启动${NC}"
        docker ps --filter "name=ros2-vision"
    else
        echo -e "${RED}[ERROR] 启动失败${NC}"
    fi
    
    read -p "按回车键继续..."
}

# 停止容器
stop_container() {
    check_docker || return 1
    
    echo -e "${BLUE}[INFO] 停止 Docker 容器...${NC}"
    cd "${WS_PATH}"
    
    docker-compose down
    
    echo -e "${GREEN}[SUCCESS] 容器已停止${NC}"
    read -p "按回车键继续..."
}

# 查看容器状态
show_container_status() {
    check_docker || return 1
    
    echo -e "${CYAN}========== Docker 容器状态 ==========${NC}"
    echo ""
    docker ps -a --filter "name=ros2-vision"
    echo ""
    
    echo -e "${CYAN}========== Docker Volumes ==========${NC}"
    docker volume ls | grep ros2
    echo ""
    
    read -p "按回车键继续..."
}

# 进入容器 Shell
enter_container() {
    check_docker || return 1
    
    if ! check_container "${CONTAINER_NAME}"; then
        echo -e "${YELLOW}[WARN] 容器未运行，正在启动...${NC}"
        start_container
    fi
    
    echo -e "${BLUE}[INFO] 进入容器 Shell...${NC}"
    echo -e "${YELLOW}[TIP] 输入 'exit' 退出容器${NC}"
    
    docker exec -it ${CONTAINER_NAME} bash
    
    read -p "按回车键继续..."
}

# 在 Docker 内编译
build_in_docker() {
    local build_type="$1"
    check_docker || return 1
    
    if ! check_container "${CONTAINER_NAME}"; then
        echo -e "${YELLOW}[WARN] 容器未运行，正在启动...${NC}"
        cd "${WS_PATH}"
        docker-compose up -d ros2-vision
        sleep 2
    fi
    
    echo -e "${BLUE}[INFO] 在 Docker 内编译 (${build_type})...${NC}"
    
    local cmake_args=""
    if [ "$build_type" == "Debug" ]; then
        cmake_args="-DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON"
    else
        cmake_args="-DCMAKE_BUILD_TYPE=Release"
    fi
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        cd /ros2_ws
        colcon build --packages-select ${PKG_NAME} --cmake-args ${cmake_args}
    "
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[SUCCESS] 编译成功！${NC}"
    else
        echo -e "${RED}[ERROR] 编译失败！${NC}"
    fi
    
    read -p "按回车键继续..."
}

# 清理 Docker 内的构建文件
clean_docker_build() {
    check_docker || return 1
    
    echo -e "${YELLOW}[WARN] 即将清理 Docker 内的构建文件${NC}"
    echo -n -e "${CYAN}确认清理? [y/N]: ${NC}"
    read confirm
    
    if [ "$confirm" == "y" ] || [ "$confirm" == "Y" ]; then
        echo -e "${BLUE}[INFO] 清理 Docker volumes...${NC}"
        cd "${WS_PATH}"
        docker-compose down
        docker volume rm ws_ros2_build_cache ws_ros2_install_cache ws_ros2_log_cache 2>/dev/null || true
        echo -e "${GREEN}[SUCCESS] 清理完成${NC}"
    else
        echo -e "${BLUE}[INFO] 取消清理${NC}"
    fi
    
    read -p "按回车键继续..."
}

# 在 Docker 内运行完整流水线
run_pipeline_in_docker() {
    local video_path="${1:-$DEFAULT_VIDEO}"
    check_docker || return 1
    
    if ! check_container "${CONTAINER_NAME}"; then
        echo -e "${YELLOW}[WARN] 容器未运行，正在启动...${NC}"
        cd "${WS_PATH}"
        docker-compose up -d ros2-vision
        sleep 2
    fi
    
    echo -e "${BLUE}[INFO] 在 Docker 内运行完整流水线...${NC}"
    echo -e "${BLUE}[INFO] 输入视频: ${video_path}${NC}"
    
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="/ros2_ws/output/docker_pipeline_${output_name}.mp4"
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        source /ros2_ws/install/setup.bash 2>/dev/null || true
        mkdir -p /ros2_ws/output
        ros2 launch ${PKG_NAME} full_pipeline_launch.py \
            video_path:=${video_path} \
            output_path:=${output_path} \
            fps:=30.0 \
            loop:=false \
            show_pose_info:=true
    "
    
    echo -e "${GREEN}[INFO] 处理完成${NC}"
    read -p "按回车键继续..."
}

# 在 Docker 内运行测试
run_tests_in_docker() {
    check_docker || return 1
    
    if ! check_container "${CONTAINER_NAME}"; then
        echo -e "${YELLOW}[WARN] 容器未运行，正在启动...${NC}"
        cd "${WS_PATH}"
        docker-compose up -d ros2-vision
        sleep 2
    fi
    
    echo -e "${BLUE}[INFO] 在 Docker 内运行测试...${NC}"
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        cd /ros2_ws
        colcon test --packages-select ${PKG_NAME}
        colcon test-result --verbose
    "
    
    read -p "按回车键继续..."
}

# 运行 EKF 可视化演示
run_ekf_demo_in_docker() {
    local video_path="${1:-$DEFAULT_VIDEO}"
    check_docker || return 1
    
    if ! check_container "${CONTAINER_NAME}"; then
        echo -e "${YELLOW}[WARN] 容器未运行，正在启动...${NC}"
        cd "${WS_PATH}"
        docker-compose up -d ros2-vision
        sleep 2
    fi
    
    echo -e "${CYAN}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║              EKF 跟踪可视化演示 (Docker)                     ║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════════════╝${NC}"
    
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="/ros2_ws/output/ekf_demo_${output_name}.mp4"
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        source /ros2_ws/install/setup.bash 2>/dev/null || true
        mkdir -p /ros2_ws/output
        ros2 launch ${PKG_NAME} full_pipeline_launch.py \
            video_path:=${video_path} \
            output_path:=${output_path} \
            fps:=30.0 \
            loop:=false \
            show_pose_info:=true \
            show_ekf_info:=true \
            show_trajectory:=true
    "
    
    echo -e "${GREEN}[INFO] 处理完成${NC}"
    read -p "按回车键继续..."
}

# 运行前哨站预测可视化演示 (Docker)
run_outpost_demo_in_docker() {
    local video_path="${1:-/home/xin/ROS2Drone/OrangeAim-Drone/utils/蓝方前哨站狙击点视角全速.mp4}"
    check_docker || return 1
    
    if ! check_container "${CONTAINER_NAME}"; then
        echo -e "${YELLOW}[WARN] 容器未运行，正在启动...${NC}"
        cd "${WS_PATH}"
        docker-compose up -d ros2-vision
        sleep 2
    fi
    
    echo -e "${CYAN}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║              前哨站预测可视化演示 (Docker)                   ║${NC}"
    echo -e "${CYAN}╠══════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}║  功能说明:                                                   ║${NC}"
    echo -e "${CYAN}║  - 使用 EKF 估计前哨站旋转中心和角速度                       ║${NC}"
    echo -e "${CYAN}║  - 显示旋转中心位置、半径、角度、角速度                      ║${NC}"
    echo -e "${CYAN}║  - 显示旋转方向 (顺时针/逆时针)                              ║${NC}"
    echo -e "${CYAN}║  - 可选显示预测轨迹                                          ║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${YELLOW}选择可视化选项：${NC}"
    echo -e "  ${GREEN}1)${NC} 仅显示前哨站预测信息"
    echo -e "  ${GREEN}2)${NC} 显示预测信息 + 预测轨迹"
    echo -e "  ${GREEN}3)${NC} 返回主菜单"
    echo ""
    echo -n -e "${CYAN}请选择 [1-3]: ${NC}"
    read outpost_choice
    
    local show_trajectory="false"
    case $outpost_choice in
        1)
            show_trajectory="false"
            ;;
        2)
            show_trajectory="true"
            ;;
        3)
            return
            ;;
        *)
            show_trajectory="false"
            ;;
    esac
    
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="/ros2_ws/output/outpost_demo_${output_name}.mp4"
    
    echo -e "${BLUE}[INFO] 启动前哨站预测流水线...${NC}"
    echo -e "${BLUE}[INFO] 输入视频: ${video_path}${NC}"
    echo -e "${BLUE}[INFO] 输出视频: ${output_path}${NC}"
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        source /ros2_ws/install/setup.bash 2>/dev/null || true
        mkdir -p /ros2_ws/output
        ros2 launch ${PKG_NAME} outpost_pipeline_launch.py \
            video_path:=${video_path} \
            output_path:=${output_path} \
            fps:=30.0 \
            loop:=false \
            show_outpost_info:=true \
            show_trajectory:=${show_trajectory}
    "
    
    echo -e "${GREEN}[INFO] 处理完成${NC}"
    read -p "按回车键继续..."
}

# 显示系统信息
show_system_info() {
    check_docker || return 1
    
    echo -e "${CYAN}========== Docker 环境信息 ==========${NC}"
    echo ""
    
    if check_container "${CONTAINER_NAME}"; then
        docker exec ${CONTAINER_NAME} bash -c "
            echo '--- ROS2 版本 ---'
            echo \"ROS_DISTRO: \${ROS_DISTRO}\"
            echo ''
            echo '--- OpenVINO 版本 ---'
            source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null
            python3 -c \"from openvino.runtime import Core; c = Core(); print(f'版本: {c.get_versions()[\"CPU\"].build_number}')\" 2>/dev/null || echo '未检测到'
            echo ''
            echo '--- OpenCV 版本 ---'
            pkg-config --modversion opencv4 2>/dev/null || echo '未检测到'
            echo ''
            echo '--- 已安装的 ROS2 包 ---'
            source /ros2_ws/install/setup.bash 2>/dev/null
            ros2 pkg list 2>/dev/null | grep armor || echo '未编译'
        "
    else
        echo -e "${YELLOW}[WARN] 容器未运行${NC}"
    fi
    
    echo ""
    read -p "按回车键继续..."
}

# ==============================================================================
# 主循环
# ==============================================================================
main() {
    while true; do
        show_header
        show_menu
        read choice
        
        case $choice in
            1) start_container ;;
            2) stop_container ;;
            3) show_container_status ;;
            4) enter_container ;;
            5) build_in_docker "Release" ;;
            6) build_in_docker "Debug" ;;
            7) clean_docker_build ;;
            8) run_pipeline_in_docker ;;
            9) run_tests_in_docker ;;
            e|E) run_ekf_demo_in_docker ;;
            o|O) run_outpost_demo_in_docker ;;
            s|S) show_system_info ;;
            0)
                echo -e "${GREEN}再见！${NC}"
                exit 0
                ;;
            *)
                echo -e "${RED}[ERROR] 无效选项${NC}"
                sleep 1
                ;;
        esac
    done
}

# 运行主函数
main
