#!/bin/bash
# Web可视化仪表盘启动脚本 (Docker 版本)

CONTAINER_NAME="ros2-vision-dev"
DOCKER_WS_PATH="/home/user/droneAim/TDrone"
SCRIPT_REL_PATH="armor_detector_ros2/scripts/web_visualizer/web_dashboard.py"
PORT=5000

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TDRONE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

check_docker() {
    docker info > /dev/null 2>&1 || { echo -e "${RED}[ERROR] Docker 未运行${NC}"; return 1; }
}

check_container() {
    docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"
}

ensure_container() {
    if ! check_container; then
        echo -e "${YELLOW}[INFO] 容器未运行，正在启动...${NC}"
        cd "${TDRONE_DIR}"
        docker compose up -d ros2-vision
        sleep 3
        check_container || { echo -e "${RED}[ERROR] 无法启动容器${NC}"; return 1; }
        echo -e "${GREEN}[OK] 容器已启动${NC}"
    fi
}

check_deps_in_docker() {
    echo -e "${YELLOW}检查容器内依赖...${NC}"
    docker exec ${CONTAINER_NAME} bash -c "
        python3 -c 'import flask' 2>/dev/null || {
            echo '使用清华镜像源安装 Flask...'
            pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple flask flask-socketio
        }
        python3 -c 'import flask_socketio' 2>/dev/null || {
            echo '使用清华镜像源安装 Flask-SocketIO...'
            pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple flask-socketio
        }
    " || {
        echo -e "${YELLOW}如果镜像源失败，可以手动进入容器安装:${NC}"
        echo -e "${CYAN}  docker exec -it ${CONTAINER_NAME} bash${NC}"
        echo -e "${CYAN}  pip3 install flask flask-socketio${NC}"
        return 1
    }
    echo -e "${GREEN}依赖检查完成${NC}"
}

show_access_info() {
    local IP=$(hostname -I | awk '{print $1}')
    echo -e "  ${YELLOW}在PC浏览器访问:${NC}"
    echo -e "  ${CYAN}http://${IP}:${PORT}${NC}"
    echo ""
}

start_dashboard() {
    check_docker || return 1
    ensure_container || return 1
    
    # 检查是否已运行
    if docker exec ${CONTAINER_NAME} pgrep -f 'web_dashboard.py' > /dev/null 2>&1; then
        echo -e "${YELLOW}仪表盘已在运行中${NC}"
        show_access_info
        return 0
    fi
    
    check_deps_in_docker || return 1
    
    echo -e "${GREEN}启动Web可视化仪表盘 (Docker内)...${NC}"
    
    # 使用 bash -c 在同一个 shell 中 source 环境并运行
    docker exec -d ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash && \
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true && \
        source ${DOCKER_WS_PATH}/install/setup.bash && \
        cd ${DOCKER_WS_PATH} && \
        python3 ${DOCKER_WS_PATH}/${SCRIPT_REL_PATH} --port ${PORT} > /tmp/web_dashboard.log 2>&1
    "
    
    sleep 2
    
    if docker exec ${CONTAINER_NAME} pgrep -f 'web_dashboard.py' > /dev/null 2>&1; then
        echo -e "${GREEN}============================================${NC}"
        echo -e "${GREEN}  Web可视化仪表盘已启动！${NC}"
        echo -e "${GREEN}============================================${NC}"
        show_access_info
        echo -e "${GREEN}============================================${NC}"
    else
        echo -e "${RED}启动失败，查看日志:${NC}"
        docker exec ${CONTAINER_NAME} cat /tmp/web_dashboard.log 2>/dev/null | tail -30
    fi
}

start_foreground() {
    check_docker || return 1
    ensure_container || return 1
    check_deps_in_docker || return 1
    
    echo -e "${GREEN}前台启动 (Ctrl+C 停止)...${NC}"
    show_access_info
    echo ""
    
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash && \
        source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true && \
        source ${DOCKER_WS_PATH}/install/setup.bash && \
        python3 ${DOCKER_WS_PATH}/${SCRIPT_REL_PATH} --port ${PORT}
    "
}

stop_dashboard() {
    check_docker || return 1
    check_container || { echo -e "${YELLOW}容器未运行${NC}"; return 0; }
    
    echo -e "${YELLOW}停止仪表盘...${NC}"
    docker exec ${CONTAINER_NAME} pkill -f 'web_dashboard.py' 2>/dev/null || true
    echo -e "${GREEN}已停止${NC}"
}

show_status() {
    check_docker || return 1
    check_container || { echo -e "${YELLOW}容器未运行${NC}"; return 0; }
    
    if docker exec ${CONTAINER_NAME} pgrep -f 'web_dashboard.py' > /dev/null 2>&1; then
        echo -e "${GREEN}仪表盘运行中${NC}"
        show_access_info
    else
        echo -e "${YELLOW}仪表盘未运行${NC}"
    fi
}

show_log() {
    check_docker || return 1
    check_container || { echo -e "${YELLOW}容器未运行${NC}"; return 0; }
    
    echo -e "${CYAN}=== 日志 (Ctrl+C 退出) ===${NC}"
    docker exec -it ${CONTAINER_NAME} tail -f /tmp/web_dashboard.log 2>/dev/null || \
        echo -e "${YELLOW}日志不存在${NC}"
}

case "$1" in
    stop) stop_dashboard ;;
    status) show_status ;;
    log) show_log ;;
    fg|foreground) start_foreground ;;
    restart) stop_dashboard; sleep 1; start_dashboard ;;
    *) start_dashboard ;;
esac
