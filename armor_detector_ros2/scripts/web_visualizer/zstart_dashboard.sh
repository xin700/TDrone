#!/bin/bash
# Web可视化仪表盘启动脚本
# 
# 使用方法:
#   ./start_dashboard.sh          # 启动（后台运行）
#   ./start_dashboard.sh stop     # 停止
#   ./start_dashboard.sh status   # 查看状态
#   ./start_dashboard.sh log      # 查看日志

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="/tmp/web_dashboard.pid"
LOG_FILE="/tmp/web_dashboard.log"
PORT=5000

# 颜色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

check_deps() {
    echo -e "${YELLOW}检查依赖...${NC}"
    
    # 检查Python包
    python3 -c "import flask" 2>/dev/null || {
        echo -e "${RED}缺少 flask，正在安装...${NC}"
        pip3 install flask flask-socketio
    }
    
    python3 -c "import flask_socketio" 2>/dev/null || {
        echo -e "${RED}缺少 flask-socketio，正在安装...${NC}"
        pip3 install flask-socketio
    }
    
    echo -e "${GREEN}依赖检查完成${NC}"
}

start_dashboard() {
    if [ -f "$PID_FILE" ]; then
        PID=$(cat "$PID_FILE")
        if ps -p $PID > /dev/null 2>&1; then
            echo -e "${YELLOW}仪表盘已在运行中 (PID: $PID)${NC}"
            echo -e "访问地址: http://$(hostname -I | awk '{print $1}'):$PORT"
            return 0
        fi
    fi
    
    check_deps
    
    echo -e "${GREEN}启动Web可视化仪表盘...${NC}"
    
    # source ROS2 环境
    source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/galactic/setup.bash 2>/dev/null
    
    # source 工作空间
    if [ -f "$SCRIPT_DIR/../../../install/setup.bash" ]; then
        source "$SCRIPT_DIR/../../../install/setup.bash"
    fi
    
    # 后台启动
    nohup python3 "$SCRIPT_DIR/web_dashboard.py" --port $PORT > "$LOG_FILE" 2>&1 &
    echo $! > "$PID_FILE"
    
    sleep 2
    
    if ps -p $(cat "$PID_FILE") > /dev/null 2>&1; then
        IP=$(hostname -I | awk '{print $1}')
        echo -e "${GREEN}============================================${NC}"
        echo -e "${GREEN}  Web可视化仪表盘已启动！${NC}"
        echo -e "${GREEN}============================================${NC}"
        echo -e "  PID: $(cat $PID_FILE)"
        echo -e "  日志: $LOG_FILE"
        echo -e ""
        echo -e "  ${YELLOW}在PC浏览器访问:${NC}"
        echo -e "  ${GREEN}http://$IP:$PORT${NC}"
        echo -e "${GREEN}============================================${NC}"
    else
        echo -e "${RED}启动失败，请查看日志: $LOG_FILE${NC}"
        cat "$LOG_FILE"
    fi
}

stop_dashboard() {
    if [ -f "$PID_FILE" ]; then
        PID=$(cat "$PID_FILE")
        if ps -p $PID > /dev/null 2>&1; then
            echo -e "${YELLOW}停止仪表盘 (PID: $PID)...${NC}"
            kill $PID
            rm -f "$PID_FILE"
            echo -e "${GREEN}已停止${NC}"
        else
            echo -e "${YELLOW}进程不存在，清理PID文件${NC}"
            rm -f "$PID_FILE"
        fi
    else
        echo -e "${YELLOW}仪表盘未运行${NC}"
    fi
}

show_status() {
    if [ -f "$PID_FILE" ]; then
        PID=$(cat "$PID_FILE")
        if ps -p $PID > /dev/null 2>&1; then
            IP=$(hostname -I | awk '{print $1}')
            echo -e "${GREEN}仪表盘运行中${NC}"
            echo -e "  PID: $PID"
            echo -e "  访问: http://$IP:$PORT"
        else
            echo -e "${RED}仪表盘已停止 (残留PID文件)${NC}"
        fi
    else
        echo -e "${YELLOW}仪表盘未运行${NC}"
    fi
}

show_log() {
    if [ -f "$LOG_FILE" ]; then
        tail -f "$LOG_FILE"
    else
        echo -e "${YELLOW}日志文件不存在${NC}"
    fi
}

case "$1" in
    stop)
        stop_dashboard
        ;;
    status)
        show_status
        ;;
    log)
        show_log
        ;;
    restart)
        stop_dashboard
        sleep 1
        start_dashboard
        ;;
    *)
        start_dashboard
        ;;
esac
