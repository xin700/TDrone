#!/bin/bash
# ==============================================================================
# Docker 环境运行脚本 (适配 run_detector.sh 所有功能)
# ==============================================================================
# 功能：
#   - 在 Docker 容器内编译和运行 ROS2 包
#   - 支持 GUI 可视化
#   - 完整匹配 run_detector.sh 的所有功能
# ==============================================================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# 配置 (容器内路径)
WS_PATH="/home/user/droneAim/TDrone"
CONTAINER_NAME="ros2-vision-dev"
CONTAINER_GUI="ros2-vision-gui"
PKG_NAME="armor_detector_ros2"
OUTPUT_DIR="${WS_PATH}/output"

# 默认测试视频 (容器内路径)
VIDEO_ARMOR="${WS_PATH}/videos/sample.avi"
VIDEO_OUTPOST="${WS_PATH}/videos/outpost_sample.mp4"

# ==============================================================================
# 函数定义
# ==============================================================================

show_header() {
    clear
    echo -e "${CYAN}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║         装甲板检测系统 (ROS2 Docker)                         ║"
    echo "║  检测 -> 解算 -> 预测 -> 可视化                              ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

show_menu() {
    echo -e "${YELLOW}请选择操作：${NC}"
    echo ""
    echo -e "  ${CYAN}--- Docker 管理 ---${NC}"
    echo -e "  ${GREEN}d1)${NC} 启动 Docker 容器"
    echo -e "  ${GREEN}d2)${NC} 停止 Docker 容器"
    echo -e "  ${GREEN}d3)${NC} 查看容器状态"
    echo -e "  ${GREEN}d4)${NC} 进入容器 Shell"
    echo ""
    echo -e "  ${CYAN}--- 编译 ---${NC}"
    echo -e "  ${GREEN}1)${NC} 编译 (Release)"
    echo -e "  ${GREEN}c)${NC} 清理编译文件"
    echo ""
    echo -e "  ${CYAN}--- 运行流水线 ---${NC}"
    echo -e "  ${GREEN}2)${NC} 运行完整流水线 (普通装甲板)"
    echo -e "  ${GREEN}3)${NC} 运行前哨站预测流水线 (V1)"
    echo -e "  ${GREEN}4)${NC} 运行前哨站预测流水线 (V2 多假设追踪)"
    echo -e "  ${GREEN}5)${NC} 运行前哨站预测流水线 (V3 极坐标观测)"
    echo -e "  ${GREEN}6)${NC} 运行前哨站预测流水线 (V4 θ观测模型) ${CYAN}[推荐]${NC}"
    echo -e "  ${GREEN}7)${NC} 选择视频运行"
    echo ""
    echo -e "  ${CYAN}--- 真实硬件 ---${NC}"
    echo -e "  ${GREEN}r1)${NC} 真实硬件数据采集 (仅数据流)"
    echo -e "  ${GREEN}r2)${NC} 真实硬件完整流水线 ${CYAN}[实战推荐]${NC}"
    echo -e "  ${GREEN}r3)${NC} 控制流水线启停"
    echo -e "  ${GREEN}r4)${NC} 查看ROS2话题"
    echo ""
    echo -e "  ${CYAN}--- 节点调试 ---${NC}"
    echo -e "  ${GREEN}D)${NC} Solver 节点调试 (PnP解算可视化)"
    echo -e "  ${GREEN}n1)${NC} 启动云台控制节点"
    echo -e "  ${GREEN}n2)${NC} 启动瞄准节点"
    echo -e "  ${GREEN}n3)${NC} 启动云台控制+瞄准节点"
    echo ""
    echo -e "  ${CYAN}--- 其他 ---${NC}"
    echo -e "  ${GREEN}8)${NC} 运行测试"
    echo -e "  ${GREEN}9)${NC} 查看输出视频"
    echo -e "  ${GREEN}s)${NC} 查看容器系统信息"
    echo -e "  ${GREEN}0)${NC} 退出"
    echo ""
    echo -n -e "${CYAN}选项: ${NC}"
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
    
    # 允许 X11 连接（用于 GUI）
    xhost +local:docker 2>/dev/null || true
    
    # 启动容器
    cd "$(dirname "$(readlink -f "$0")")"
    docker compose up -d ros2-vision
    
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
    cd "$(dirname "$(readlink -f "$0")")"
    
    docker compose down
    
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
    local build_type="${1:-Release}"
    check_docker || return 1
    
    if ! check_container "${CONTAINER_NAME}"; then
        echo -e "${YELLOW}[WARN] 容器未运行，正在启动...${NC}"
        cd "$(dirname "$(readlink -f "$0")")"
        docker compose up -d ros2-vision
        sleep 2
    fi
    
    # 询问并行编译数量
    echo -e "${CYAN}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║              编译资源配置                                    ║${NC}"
    echo -e "${CYAN}╠══════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}║  NUC编译建议: 使用 1-2 个并行任务防止系统卡死               ║${NC}"
    echo -e "${CYAN}║  如果系统内存 >= 16GB，可以尝试 4 个并行任务                ║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -n -e "${CYAN}并行编译任务数 (建议1-2，默认2): ${NC}"
    read parallel_input
    local parallel_workers="${parallel_input:-2}"
    
    echo -e "${BLUE}[INFO] 编译配置: ${build_type}, 并行任务=${parallel_workers}${NC}"
    echo -e "${YELLOW}[TIP] 编译过程可能需要5-15分钟，请耐心等待...${NC}"
    echo -e "${YELLOW}[TIP] 如果系统卡死，请重启后使用更小的并行数（如1）${NC}"
    echo ""
    
    # 显示当前系统资源
    echo -e "${CYAN}--- 编译前系统资源 ---${NC}"
    docker exec ${CONTAINER_NAME} bash -c "
        echo 'CPU核心数: '\$(nproc)
        echo ' 内存使用: '\$(free -h | awk '/^Mem:/{print \$3\"/\"\$2}')
        echo 'Swap使用: '\$(free -h | awk '/^Swap:/{print \$3\"/\"\$2}')
    "
    echo ""
    
    echo -e "${BLUE}[INFO] 开始编译...${NC}"
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        cd ${WS_PATH}
        
        # 限制编译并行度，防止OOM
        colcon build \
            --packages-select ${PKG_NAME} \
            --parallel-workers ${parallel_workers} \
            --cmake-args -DCMAKE_BUILD_TYPE=${build_type} \
            --event-handlers console_direct+
    "
    
    local exit_code=$?
    
    # 显示编译后系统资源
    echo ""
    echo -e "${CYAN}--- 编译后系统资源 ---${NC}"
    docker exec ${CONTAINER_NAME} bash -c "
        echo 'CPU核心数: '\$(nproc)
        echo '内存使用: '\$(free -h | awk '/^Mem:/{print \$3\"/\"\$2}')
        echo 'Swap使用: '\$(free -h | awk '/^Swap:/{print \$3\"/\"\$2}')
    " 2>/dev/null || echo -e "${RED}无法获取资源信息（容器可能崩溃）${NC}"
    echo ""
    
    if [ $exit_code -eq 0 ]; then
        echo -e "${GREEN}[OK] 编译成功${NC}"
    else
        echo -e "${RED}[ERROR] 编译失败 (退出码: ${exit_code})${NC}"
        echo -e "${YELLOW}[TIP] 如果是内存不足，请尝试:${NC}"
        echo -e "${YELLOW}      1. 减少并行任务数 (--parallel-workers 1)${NC}"
        echo -e "${YELLOW}      2. 增加交换空间 (sudo fallocate -l 8G /swapfile)${NC}"
        echo -e "${YELLOW}      3. 关闭其他应用释放内存${NC}"
    fi
    
    read -p "按回车继续..."
}

# 清理 Docker 内的构建文件
clean_docker_build() {
    check_docker || return 1
    
    echo -e "${YELLOW}[WARN] 即将清理 Docker 内的构建文件${NC}"
    echo -n -e "${CYAN}确认清理? [y/N]: ${NC}"
    read confirm
    
    if [ "$confirm" == "y" ] || [ "$confirm" == "Y" ]; then
        if check_container "${CONTAINER_NAME}"; then
            echo -e "${BLUE}[INFO] 清理容器内构建文件...${NC}"
            docker exec -it ${CONTAINER_NAME} bash -c "
                cd ${WS_PATH}
                rm -rf build install log
            "
        fi
        echo -e "${GREEN}[OK] 清理完成${NC}"
    fi
    read -p "按回车继续..."
}

# 确保容器运行
ensure_container_running() {
    if ! check_container "${CONTAINER_NAME}"; then
        echo -e "${YELLOW}[WARN] 容器未运行，正在启动...${NC}"
        cd "$(dirname "$(readlink -f "$0")")"
        docker compose up -d ros2-vision
        sleep 3
        echo -e "${GREEN}[OK] 容器已启动${NC}"
    fi
}

# 运行完整流水线（检测+解算+可视化）
run_full_pipeline() {
    local video_path="$1"
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="${OUTPUT_DIR}/result_${output_name}.mp4"
    
    # 询问 fps
    echo -n -e "${CYAN}视频帧率 (默认30.0): ${NC}"
    read fps_input
    local fps="${fps_input:-30.0}"
    
    echo -e "${BLUE}[INFO] 启动完整流水线${NC}"
    echo -e "${BLUE}[INFO] 输入: ${video_path}${NC}"
    echo -e "${BLUE}[INFO] 输出: ${output_path}${NC}"
    echo -e "${BLUE}[INFO] FPS: ${fps}${NC}"
    
    ensure_container_running
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        source ${WS_PATH}/install/setup.bash
        mkdir -p ${OUTPUT_DIR}
        ros2 launch ${PKG_NAME} full_pipeline_launch.py \
            video_path:='${video_path}' \
            output_path:='${output_path}' \
            fps:=${fps} \
            loop:=false \
            show_pose_info:=true \
            use_imu_file:=false
    "
    
    echo -e "${GREEN}[OK] 完成: ${output_path}${NC}"
    read -p "按回车继续..."
}

# 运行前哨站预测流水线 V1
run_outpost_pipeline() {
    local video_path="$1"
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="${OUTPUT_DIR}/outpost_${output_name}.mp4"
    
    # 询问 fps
    echo -n -e "${CYAN}视频帧率 (默认30.0): ${NC}"
    read fps_input
    local fps="${fps_input:-30.0}"
    
    echo -e "${BLUE}[INFO] 启动前哨站预测流水线 (V1)${NC}"
    echo -e "${BLUE}[INFO] 输入: ${video_path}${NC}"
    echo -e "${BLUE}[INFO] 输出: ${output_path}${NC}"
    echo -e "${BLUE}[INFO] FPS: ${fps}${NC}"
    
    ensure_container_running
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        source ${WS_PATH}/install/setup.bash
        mkdir -p ${OUTPUT_DIR}
        ros2 launch ${PKG_NAME} outpost_pipeline_launch.py \
            video_path:='${video_path}' \
            output_path:='${output_path}' \
            fps:=${fps} \
            loop:=false \
            show_outpost_info:=true \
            show_trajectory:=true \
            use_imu_file:=false
    "
    
    echo -e "${GREEN}[OK] 完成: ${output_path}${NC}"
    read -p "按回车继续..."
}

# 运行前哨站预测 V2 流水线（多假设追踪）
run_outpost_v2_pipeline() {
    local video_path="$1"
    local show_window="${2:-false}"
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="${OUTPUT_DIR}/outpost_v2_${output_name}.mp4"
    
    # 询问 fps
    echo -n -e "${CYAN}视频帧率 (默认30.0): ${NC}"
    read fps_input
    local fps="${fps_input:-30.0}"
    
    echo -e "${BLUE}[INFO] 启动前哨站预测流水线 V2 (多假设追踪)${NC}"
    echo -e "${CYAN}  - 3个并行EKF假设 (高/中/低)${NC}"
    echo -e "${CYAN}  - 自动收敛检测${NC}"
    echo -e "${CYAN}  - 三装甲板位置可视化${NC}"
    echo -e "${BLUE}[INFO] 输入: ${video_path}${NC}"
    echo -e "${BLUE}[INFO] 输出: ${output_path}${NC}"
    echo -e "${BLUE}[INFO] FPS: ${fps}${NC}"
    
    ensure_container_running
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        source ${WS_PATH}/install/setup.bash
        mkdir -p ${OUTPUT_DIR}
        ros2 launch ${PKG_NAME} outpost_pipeline_v2_launch.py \
            video_path:='${video_path}' \
            output_path:='${output_path}' \
            fps:=${fps} \
            loop:=false \
            show_window:=${show_window} \
            show_outpost_info:=true \
            use_imu_file:=false
    "
    
    echo -e "${GREEN}[OK] 完成: ${output_path}${NC}"
    read -p "按回车继续..."
}

# 运行前哨站预测 V3 流水线（极坐标观测）
run_outpost_v3_pipeline() {
    local video_path="$1"
    local show_window="${2:-false}"
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="${OUTPUT_DIR}/outpost_v3_${output_name}.mp4"
    
    # 询问 fps
    echo -n -e "${CYAN}视频帧率 (默认30.0): ${NC}"
    read fps_input
    local fps="${fps_input:-30.0}"
    
    echo -e "${BLUE}[INFO] 启动前哨站预测流水线 V3 (极坐标观测)${NC}"
    echo -e "${BLUE}[INFO] 输入: ${video_path}${NC}"
    echo -e "${BLUE}[INFO] 输出: ${output_path}${NC}"
    echo -e "${BLUE}[INFO] FPS: ${fps}${NC}"
    
    ensure_container_running
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        source ${WS_PATH}/install/setup.bash
        mkdir -p ${OUTPUT_DIR}
        ros2 launch ${PKG_NAME} outpost_pipeline_v3_launch.py \
            video_path:='${video_path}' \
            output_path:='${output_path}' \
            fps:=${fps} \
            loop:=false \
            show_window:=${show_window} \
            show_outpost_info:=true \
            use_imu_file:=false
    "
    
    echo -e "${GREEN}[OK] 完成: ${output_path}${NC}"
    read -p "按回车继续..."
}

# 运行前哨站预测 V4 流水线（θ观测模型）
run_outpost_v4_pipeline() {
    local video_path="$1"
    local show_window="${2:-false}"
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="${OUTPUT_DIR}/outpost_v4_${output_name}.mp4"
    
    # 询问 fps
    echo -n -e "${CYAN}视频帧率 (默认30.0): ${NC}"
    read fps_input
    local fps="${fps_input:-30.0}"
    
    # 询问装甲板排列
    echo -n -e "${CYAN}装甲板排列反向? (俯视顺时针低-中-高) [y/N]: ${NC}"
    read reversed_input
    local armor_reversed="false"
    if [ "$reversed_input" == "y" ] || [ "$reversed_input" == "Y" ]; then
        armor_reversed="true"
    fi
    
    echo -e "${BLUE}[INFO] 启动前哨站预测流水线 V4 (θ观测模型) ${CYAN}[推荐]${NC}"
    echo -e "${BLUE}[INFO] 输入: ${video_path}${NC}"
    echo -e "${BLUE}[INFO] 输出: ${output_path}${NC}"
    echo -e "${BLUE}[INFO] FPS: ${fps}${NC}"
    echo -e "${BLUE}[INFO] 装甲板排列: ${armor_reversed} (${armor_reversed}=低-中-高, false=高-中-低)${NC}"
    
    ensure_container_running
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        source ${WS_PATH}/install/setup.bash
        # 重启 ROS2 daemon 以清除之前的参数缓存
        ros2 daemon stop 2>/dev/null || true
        ros2 daemon start 2>/dev/null || true
        mkdir -p ${OUTPUT_DIR}
        ros2 launch ${PKG_NAME} outpost_pipeline_v4_launch.py \
            video_path:='${video_path}' \
            output_path:='${output_path}' \
            outpost_omega:=2.513 \
            armor_arrangement_reversed:=${armor_reversed} \
            fps:=${fps} \
            loop:=false \
            show_window:=${show_window} \
            show_outpost_info:=true \
            use_imu_file:=false
    "
    
    echo -e "${GREEN}[OK] 完成: ${output_path}${NC}"
    read -p "按回车继续..."
}

# Solver 节点调试
run_solver_debug() {
    local video_path="$1"
    local show_window="${2:-true}"
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="${OUTPUT_DIR}/solver_debug_${output_name}.mp4"
    
    # 询问 fps
    echo -n -e "${CYAN}视频帧率 (默认30.0): ${NC}"
    read fps_input
    local fps="${fps_input:-30.0}"
    
    echo -e "${BLUE}[INFO] 启动 Solver 节点调试 (PnP解算可视化)${NC}"
    echo -e "${BLUE}[INFO] 输入: ${video_path}${NC}"
    echo -e "${BLUE}[INFO] 输出: ${output_path}${NC}"
    echo -e "${BLUE}[INFO] FPS: ${fps}${NC}"
    
    ensure_container_running
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        source ${WS_PATH}/install/setup.bash
        mkdir -p ${OUTPUT_DIR}
        ros2 launch ${PKG_NAME} solver_debug_launch.py \
            video_path:='${video_path}' \
            output_path:='${output_path}' \
            fps:=${fps} \
            loop:=false \
            show_window:=${show_window} \
            show_axis:=true \
            show_grid:=true \
            use_imu_file:=false
    "
    
    echo -e "${GREEN}[OK] 完成: ${output_path}${NC}"
    read -p "按回车继续..."
}

# 选择视频运行
select_video() {
    echo -e "${YELLOW}可用的视频文件：${NC}"
    
    # 在容器内列出视频文件
    ensure_container_running
    docker exec ${CONTAINER_NAME} bash -c "ls -lh ${WS_PATH}/videos/*.{avi,mp4,mkv} 2>/dev/null || echo '无视频文件'"
    
    echo ""
    echo -n -e "${CYAN}输入视频路径 (相对于 ${WS_PATH}): ${NC}"
    read video_input
    
    if [ -z "$video_input" ]; then
        echo -e "${RED}[ERROR] 未输入视频路径${NC}"
        read -p "按回车继续..."
        return
    fi
    
    # 构建完整路径
    if [[ "$video_input" = /* ]]; then
        # 绝对路径
        video_path="$video_input"
    else
        # 相对路径
        video_path="${WS_PATH}/${video_input}"
    fi
    
    # 选择流水线类型
    echo ""
    echo -e "${YELLOW}选择流水线类型：${NC}"
    echo -e "  ${GREEN}1)${NC} 完整流水线 (普通装甲板)"
    echo -e "  ${GREEN}2)${NC} 前哨站预测 V1"
    echo -e "  ${GREEN}3)${NC} 前哨站预测 V2 (多假设)"
    echo -e "  ${GREEN}4)${NC} 前哨站预测 V3 (极坐标)"
    echo -e "  ${GREEN}5)${NC} 前哨站预测 V4 (θ观测) ${CYAN}[推荐]${NC}"
    echo -e "  ${GREEN}6)${NC} Solver 调试"
    echo -n -e "${CYAN}选择: ${NC}"
    read pipeline_choice
    
    case $pipeline_choice in
        1) run_full_pipeline "$video_path" ;;
        2) run_outpost_pipeline "$video_path" ;;
        3) run_outpost_v2_pipeline "$video_path" false ;;
        4) run_outpost_v3_pipeline "$video_path" false ;;
        5) run_outpost_v4_pipeline "$video_path" false ;;
        6) run_solver_debug "$video_path" true ;;
        *) echo -e "${RED}无效选项${NC}"; read -p "按回车继续..." ;;
    esac
}

# 运行测试
run_tests() {
    echo -e "${BLUE}[INFO] 运行测试...${NC}"
    ensure_container_running
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source ${WS_PATH}/install/setup.bash
        cd ${WS_PATH}
        colcon test --packages-select ${PKG_NAME}
        colcon test-result --verbose
    "
    
    read -p "按回车继续..."
}

# 查看输出视频
view_output() {
    ensure_container_running
    
    echo -e "${YELLOW}输出视频：${NC}"
    docker exec ${CONTAINER_NAME} bash -c "ls -lht ${OUTPUT_DIR}/*.mp4 2>/dev/null | head -10 || echo '无输出文件'"
    
    echo ""
    echo -n -e "${CYAN}输入文件名在宿主机播放 (或回车返回): ${NC}"
    read filename
    
    if [ -n "$filename" ]; then
        # 构建宿主机路径
        local host_path="$(dirname "$(readlink -f "$0")")/output/${filename}"
        if [ -f "$host_path" ]; then
            xdg-open "$host_path" 2>/dev/null || vlc "$host_path" 2>/dev/null || echo -e "${RED}无法打开视频${NC}"
        else
            echo -e "${RED}文件不存在: $host_path${NC}"
        fi
    fi
    
    read -p "按回车继续..."
}

# 查看系统信息
show_system_info() {
    ensure_container_running
    
    echo -e "${CYAN}========== 容器系统信息 ==========${NC}"
    docker exec ${CONTAINER_NAME} bash -c "
        echo '--- ROS2 版本 ---'
        echo \"ROS_DISTRO: \$ROS_DISTRO\"
        echo ''
        echo '--- OpenVINO 版本 ---'
        ls /opt/intel/ 2>/dev/null || echo 'OpenVINO 未找到'
        echo ''
        echo '--- Python 版本 ---'
        python3 --version
        echo ''
        echo '--- 工作目录 ---'
        ls -la ${WS_PATH}/
    "
    
    read -p "按回车继续..."
}

# 在 Docker 内运行完整流水线
run_pipeline_in_docker() {
    local video_path="${1:-$DEFAULT_VIDEO}"
    check_docker || return 1
    
    if ! check_container "${CONTAINER_NAME}"; then
        echo -e "${YELLOW}[WARN] 容器未运行，正在启动...${NC}"
        cd "${WS_PATH}"
        docker compose up -d ros2-vision
        sleep 2
    fi
    
    echo -e "${BLUE}[INFO] 在 Docker 内运行完整流水线...${NC}"
    echo -e "${BLUE}[INFO] 输入视频: ${video_path}${NC}"
    
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="/home/user/droneAim/TDrone/output/docker_pipeline_${output_name}.mp4"
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        source /home/user/droneAim/install/setup.bash 2>/dev/null || true
        mkdir -p /home/user/droneAim/TDrone/output
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
        docker compose up -d ros2-vision
        sleep 2
    fi
    
    echo -e "${BLUE}[INFO] 在 Docker 内运行测试...${NC}"
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        cd /home/user/droneAim
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
        docker compose up -d ros2-vision
        sleep 2
    fi
    
    echo -e "${CYAN}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║              EKF 跟踪可视化演示 (Docker)                     ║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════════════╝${NC}"
    
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="/home/user/droneAim/TDrone/output/ekf_demo_${output_name}.mp4"
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        source /home/user/droneAim/install/setup.bash 2>/dev/null || true
        mkdir -p /home/user/droneAim/TDrone/output
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
    local video_path="${1:-/home/user/ROS2Drone/OrangeAim-Drone/utils/蓝方前哨站狙击点视角全速.mp4}"
    check_docker || return 1
    
    if ! check_container "${CONTAINER_NAME}"; then
        echo -e "${YELLOW}[WARN] 容器未运行，正在启动...${NC}"
        cd "${WS_PATH}"
        docker compose up -d ros2-vision
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
    local output_path="/home/user/droneAim/TDrone/output/outpost_demo_${output_name}.mp4"
    
    echo -e "${BLUE}[INFO] 启动前哨站预测流水线...${NC}"
    echo -e "${BLUE}[INFO] 输入视频: ${video_path}${NC}"
    echo -e "${BLUE}[INFO] 输出视频: ${output_path}${NC}"
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        source /home/user/droneAim/install/setup.bash 2>/dev/null || true
        mkdir -p /home/user/droneAim/TDrone/output
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

# ==================== 真实硬件数据流函数 ====================

# 启动真实硬件数据采集（仅数据流，不启动流水线）
run_real_hardware_data() {
    echo -e "${CYAN}╔═════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║              真实硬件数据采集模式                           ║${NC}"
    echo -e "${CYAN}╠═════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}║  功能说明:                                                  ║${NC}"
    echo -e "${CYAN}║  - 从真实相机采集图像流                                     ║${NC}"
    echo -e "${CYAN}║  - 从串口读取IMU数据                                        ║${NC}"
    echo -e "${CYAN}║  - 发布到 /camera/image_raw 和 /imu/quaternion              ║${NC}"
    echo -e "${CYAN}║  - 提供 /pipeline_control 服务控制流水线启停                ║${NC}"
    echo -e "${CYAN}╚═════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    
    # 询问相机参数
    echo -n -e "${CYAN}相机序列号 (默认 FGK25050153): ${NC}"
    read camera_sn_input
    local camera_sn="${camera_sn_input:-FGK25050153}"
    
    echo -n -e "${CYAN}曝光时间 μs (默认 3000): ${NC}"
    read exposure_input
    local exposure_time="${exposure_input:-3000}"
    
    echo -n -e "${CYAN}帧率 (默认 60): ${NC}"
    read fps_input
    local fps="${fps_input:-60.0}"
    
    # 询问串口参数
    echo -n -e "${CYAN}串口设备 (默认 /dev/ttyACM0): ${NC}"
    read serial_input
    local serial_device="${serial_input:-/dev/ttyACM0}"
    
    echo -e "${BLUE}[INFO] 启动真实硬件数据采集...${NC}"
    echo -e "${BLUE}[INFO] 相机: ${camera_sn}, 曝光: ${exposure_time}μs, FPS: ${fps}${NC}"
    echo -e "${BLUE}[INFO] 串口: ${serial_device}${NC}"
    echo -e "${YELLOW}[TIP] 按 Ctrl+C 停止数据采集${NC}"
    echo ""
    
    ensure_container_running
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        source ${WS_PATH}/install/setup.bash
        ros2 launch ${PKG_NAME} real_hardware_launch.py \
            camera_sn:='${camera_sn}' \
            exposure_time:=${exposure_time} \
            fps:=${fps} \
            serial_device:='${serial_device}'
    "
    
    read -p "按回车继续..."
}

# 启动真实硬件完整流水线
run_real_hardware_pipeline() {
    echo -e "${CYAN}╔═════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║              真实硬件完整流水线                             ║${NC}"
    echo -e "${CYAN}╠═════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}║  功能说明:                                                  ║${NC}"
    echo -e "${CYAN}║  - 从真实相机和串口采集数据                                 ║${NC}"
    echo -e "${CYAN}║  - 自动运行检测 -> 解算 -> 预测流水线                       ║${NC}"
    echo -e "${CYAN}║  - 可通过服务控制流水线启停                                 ║${NC}"
    echo -e "${CYAN}╚═════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    
    # 询问相机参数
    echo -n -e "${CYAN}相机序列号 (默认 FGK25050153): ${NC}"
    read camera_sn_input
    local camera_sn="${camera_sn_input:-FGK25050153}"
    
    echo -n -e "${CYAN}曝光时间 μs (默认 3000): ${NC}"
    read exposure_input
    local exposure_time="${exposure_input:-3000}"
    
    echo -n -e "${CYAN}帧率 (默认 60): ${NC}"
    read fps_input
    local fps="${fps_input:-60.0}"
    
    # 询问串口参数
    echo -n -e "${CYAN}串口设备 (默认 /dev/ttyACM0): ${NC}"
    read serial_input
    local serial_device="${serial_input:-/dev/ttyACM0}"
    
    # 询问装甲板排列
    echo -n -e "${CYAN}装甲板排列反向? (俯视顺时针低-中-高) [y/N]: ${NC}"
    read reversed_input
    local armor_reversed="false"
    if [ "$reversed_input" == "y" ] || [ "$reversed_input" == "Y" ]; then
        armor_reversed="true"
    fi
    
    # 询问可视化
    echo -n -e "${CYAN}是否启用可视化? [y/N]: ${NC}"
    read vis_input
    local enable_vis="false"
    if [ "$vis_input" == "y" ] || [ "$vis_input" == "Y" ]; then
        enable_vis="true"
    fi
    
    # 询问是否保存视频
    echo -n -e "${CYAN}是否保存输出视频? [y/N]: ${NC}"
    read save_input
    local save_video="false"
    local output_path_arg=""
    if [ "$save_input" == "y" ] || [ "$save_input" == "Y" ]; then
        save_video="true"
        local output_name=$(date +"%Y%m%d_%H%M%S")
        local output_path="${OUTPUT_DIR}/real_hardware_${output_name}.mp4"
        output_path_arg="output_path:='${output_path}'"
    fi
    
    echo -e "${BLUE}[INFO] 启动真实硬件完整流水线...${NC}"
    echo -e "${BLUE}[INFO] 相机: ${camera_sn}, 曝光: ${exposure_time}μs, FPS: ${fps}${NC}"
    echo -e "${BLUE}[INFO] 串口: ${serial_device}${NC}"
    echo -e "${BLUE}[INFO] 装甲板排列: ${armor_reversed} (true=低-中-高, false=高-中-低)${NC}"
    echo -e "${BLUE}[INFO] 可视化: ${enable_vis}${NC}"
    echo -e "${BLUE}[INFO] 保存视频: ${save_video}${NC}"
    if [ "$save_video" == "true" ]; then
        echo -e "${BLUE}[INFO] 输出: ${output_path}${NC}"
    fi
    echo -e "${YELLOW}[TIP] 按 Ctrl+C 停止流水线${NC}"
    echo -e "${YELLOW}[TIP] 可使用 'ros2 service call /pipeline_control std_srvs/srv/SetBool \"{data: false}\"' 停止检测${NC}"
    echo ""
    
    ensure_container_running
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
        source ${WS_PATH}/install/setup.bash
        # 重启 ROS2 daemon 以清除之前的参数缓存
        ros2 daemon stop 2>/dev/null || true
        ros2 daemon start 2>/dev/null || true
        mkdir -p ${OUTPUT_DIR}
        ros2 launch ${PKG_NAME} real_hardware_pipeline_launch.py \
            camera_sn:='${camera_sn}' \
            exposure_time:=${exposure_time} \
            fps:=${fps} \
            serial_device:='${serial_device}' \
            armor_arrangement_reversed:=${armor_reversed} \
            enable_visualizer:=${enable_vis} \
            save_video:=${save_video} \
            ${output_path_arg}
    "
    
    read -p "按回车继续..."
}

# 控制流水线启停
control_pipeline() {
    echo -e "${CYAN}╔═════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║              流水线控制                                     ║${NC}"
    echo -e "${CYAN}╚═════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${YELLOW}选择操作：${NC}"
    echo -e "  ${GREEN}1)${NC} 启动检测流水线"
    echo -e "  ${GREEN}2)${NC} 停止检测流水线"
    echo -e "  ${GREEN}3)${NC} 返回"
    echo -n -e "${CYAN}选择: ${NC}"
    read ctrl_choice
    
    ensure_container_running
    
    case $ctrl_choice in
        1)
            echo -e "${BLUE}[INFO] 启动检测流水线...${NC}"
            docker exec ${CONTAINER_NAME} bash -c "
                source /opt/ros/humble/setup.bash
                source ${WS_PATH}/install/setup.bash 2>/dev/null || true
                ros2 service call /pipeline_control std_srvs/srv/SetBool '{data: true}'
            "
            echo -e "${GREEN}[OK] 检测流水线已启动${NC}"
            ;;
        2)
            echo -e "${BLUE}[INFO] 停止检测流水线...${NC}"
            docker exec ${CONTAINER_NAME} bash -c "
                source /opt/ros/humble/setup.bash
                source ${WS_PATH}/install/setup.bash 2>/dev/null || true
                ros2 service call /pipeline_control std_srvs/srv/SetBool '{data: false}'
            "
            echo -e "${GREEN}[OK] 检测流水线已停止（数据采集继续）${NC}"
            ;;
        3)
            return
            ;;
        *)
            echo -e "${RED}无效选项${NC}"
            ;;
    esac
    
    read -p "按回车继续..."
}

# 启动云台控制节点
run_gimbal_control_node() {
    echo -e "${CYAN}╔═════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║              启动云台控制节点                               ║${NC}"
    echo -e "${CYAN}╠═════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}║  功能说明:                                                  ║${NC}"
    echo -e "${CYAN}║  - 接收 GimbalCommand 消息                                  ║${NC}"
    echo -e "${CYAN}║  - 通过串口发送云台控制指令                                 ║${NC}"
    echo -e "${CYAN}║  - 订阅话题: /aiming/gimbal_command                         ║${NC}"
    echo -e "${CYAN}║  - 提供服务: /gimbal_control/pipeline_control              ║${NC}"
    echo -e "${CYAN}╚═════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    
    # 询问串口参数
    echo -n -e "${CYAN}串口设备 (默认 /dev/ttyACM0): ${NC}"
    read serial_input
    local serial_device="${serial_input:-/dev/ttyACM0}"
    
    echo -n -e "${CYAN}波特率 (默认 115200): ${NC}"
    read baudrate_input
    local baudrate="${baudrate_input:-115200}"
    
    echo -n -e "${CYAN}是否启用仿真模式 (无串口)? [y/N]: ${NC}"
    read sim_input
    local simulation_mode="false"
    if [ "$sim_input" == "y" ] || [ "$sim_input" == "Y" ]; then
        simulation_mode="true"
    fi
    
    echo -e "${BLUE}[INFO] 启动云台控制节点...${NC}"
    echo -e "${BLUE}[INFO] 串口: ${serial_device}, 波特率: ${baudrate}${NC}"
    echo -e "${BLUE}[INFO] 仿真模式: ${simulation_mode}${NC}"
    echo -e "${YELLOW}[TIP] 按 Ctrl+C 停止节点${NC}"
    echo -e "${YELLOW}[TIP] 可使用测试脚本发送云台指令：${NC}"
    echo -e "${YELLOW}      sudo bash armor_detector_ros2/scripts/test_gimbal_control_docker.sh${NC}"
    echo ""
    
    ensure_container_running
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source ${WS_PATH}/install/setup.bash
        ros2 run ${PKG_NAME} gimbal_control_node \
            --ros-args \
            -p serial_device:='${serial_device}' \
            -p baudrate:=${baudrate} \
            -p simulation_mode:=${simulation_mode}
    "
    
    read -p "按回车继续..."
}

# 启动瞄准节点
run_aiming_node() {
    echo -e "${CYAN}╔═════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║              启动瞄准节点                                   ║${NC}"
    echo -e "${CYAN}╠═════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}║  功能说明:                                                  ║${NC}"
    echo -e "${CYAN}║  - 接收预测结果 OutpostAimResult                            ║${NC}"
    echo -e "${CYAN}║  - 计算云台控制角度                                         ║${NC}"
    echo -e "${CYAN}║  - 发布 GimbalCommand 消息                                  ║${NC}"
    echo -e "${CYAN}║  - 3状态机: INITIALIZING/TRACKING/SWITCHING                 ║${NC}"
    echo -e "${CYAN}╚═════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    
    echo -e "${BLUE}[INFO] 启动瞄准节点...${NC}"
    echo -e "${YELLOW}[TIP] 按 Ctrl+C 停止节点${NC}"
    echo -e "${YELLOW}[TIP] 需要先运行预测节点以提供瞄准数据${NC}"
    echo ""
    
    ensure_container_running
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source ${WS_PATH}/install/setup.bash
        ros2 run ${PKG_NAME} aiming_node
    "
    
    read -p "按回车继续..."
}

# 启动云台控制+瞄准节点
run_aiming_and_gimbal_nodes() {
    echo -e "${CYAN}╔═════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║              启动瞄准+云台控制节点                          ║${NC}"
    echo -e "${CYAN}╠═════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}║  功能说明:                                                  ║${NC}"
    echo -e "${CYAN}║  - 同时启动瞄准节点和云台控制节点                           ║${NC}"
    echo -e "${CYAN}║  - 完整的瞄准控制链路                                       ║${NC}"
    echo -e "${CYAN}╚═════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    
    # 询问串口参数
    echo -n -e "${CYAN}串口设备 (默认 /dev/ttyACM0): ${NC}"
    read serial_input
    local serial_device="${serial_input:-/dev/ttyACM0}"
    
    echo -n -e "${CYAN}是否启用仿真模式 (无串口)? [y/N]: ${NC}"
    read sim_input
    local simulation_mode="false"
    if [ "$sim_input" == "y" ] || [ "$sim_input" == "Y" ]; then
        simulation_mode="true"
    fi
    
    echo -e "${BLUE}[INFO] 启动瞄准+云台控制节点...${NC}"
    echo -e "${BLUE}[INFO] 串口: ${serial_device}${NC}"
    echo -e "${BLUE}[INFO] 仿真模式: ${simulation_mode}${NC}"
    echo -e "${YELLOW}[TIP] 按 Ctrl+C 停止节点${NC}"
    echo ""
    
    ensure_container_running
    
    # 在容器中后台启动两个节点
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source ${WS_PATH}/install/setup.bash
        
        # 启动云台控制节点（后台）
        ros2 run ${PKG_NAME} gimbal_control_node \
            --ros-args \
            -p serial_device:='${serial_device}' \
            -p baudrate:=115200 \
            -p simulation_mode:=${simulation_mode} &
        GIMBAL_PID=\$!
        
        sleep 1
        
        # 启动瞄准节点（前台）
        ros2 run ${PKG_NAME} aiming_node
        
        # 清理
        kill \$GIMBAL_PID 2>/dev/null || true
    "
    
    read -p "按回车继续..."
}

# 查看ROS2话题
view_topics() {
    ensure_container_running
    
    echo -e "${CYAN}========== ROS2 话题列表 ==========${NC}"
    docker exec ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source ${WS_PATH}/install/setup.bash 2>/dev/null || true
        ros2 topic list
    "
    echo ""
    echo -e "${CYAN}========== 话题发布频率 ==========${NC}"
    echo -e "${YELLOW}[TIP] 监控主要话题 5 秒...${NC}"
    docker exec ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash
        source ${WS_PATH}/install/setup.bash 2>/dev/null || true
        timeout 5 ros2 topic hz /camera/image_raw 2>/dev/null &
        timeout 5 ros2 topic hz /imu/quaternion 2>/dev/null &
        wait
    " 2>/dev/null || true
    
    read -p "按回车继续..."
}
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
            source /home/user/droneAim/install/setup.bash 2>/dev/null
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
            # Docker 管理
            d1|D1) start_container ;;
            d2|D2) stop_container ;;
            d3|D3) show_container_status ;;
            d4|D4) enter_container ;;
            
            # 编译
            1) build_in_docker "Release" ;;
            c|C) clean_docker_build ;;
            
            # 运行流水线
            2) run_full_pipeline "${VIDEO_ARMOR}" ;;
            3) run_outpost_pipeline "${VIDEO_OUTPOST}" ;;
            4) run_outpost_v2_pipeline "${VIDEO_OUTPOST}" false ;;
            5) run_outpost_v3_pipeline "${VIDEO_OUTPOST}" false ;;
            6) run_outpost_v4_pipeline "${VIDEO_OUTPOST}" false ;;
            7) select_video ;;
            
            # 真实硬件
            r1|R1) run_real_hardware_data ;;
            r2|R2) run_real_hardware_pipeline ;;
            r3|R3) control_pipeline ;;
            r4|R4) view_topics ;;

            # 调试
            d|D) run_solver_debug "${VIDEO_ARMOR}" true ;;
            n1|N1) run_gimbal_control_node ;;
            n2|N2) run_aiming_node ;;
            n3|N3) run_aiming_and_gimbal_nodes ;;
            
            # 其他
            8) run_tests ;;
            9) view_output ;;
            s|S) show_system_info ;;
            
            0) echo -e "${GREEN}再见！${NC}"; exit 0 ;;
            *) echo -e "${RED}无效选项${NC}"; sleep 1 ;;
        esac
    done
}

# 运行主函数
main
