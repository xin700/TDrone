#!/bin/bash
# ==============================================================================
# 装甲板检测系统启动脚本 (简化版)
# ==============================================================================
# 功能：编译、运行完整流水线（检测+解算+预测+可视化）、测试
# ==============================================================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# 配置 (使用 Docker 容器内路径 /ros2_ws)
WS_PATH="/ros2_ws"
PKG_NAME="armor_detector_ros2"
OUTPUT_DIR="${WS_PATH}/output"

# 测试视频 (放在 /ros2_ws/videos 目录下)
VIDEO_ARMOR="${WS_PATH}/videos/sample.avi"
VIDEO_OUTPOST="${WS_PATH}/videos/outpost_sample.mp4"

# ==============================================================================
# 函数定义
# ==============================================================================

show_header() {
    clear
    echo -e "${CYAN}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║         装甲板检测系统 (ROS2)                                ║"
    echo "║  检测 -> 解算 -> 预测 -> 可视化                              ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

show_menu() {
    echo -e "${YELLOW}请选择操作：${NC}"
    echo ""
    echo -e "  ${GREEN}1)${NC} 编译"
    echo -e "  ${GREEN}2)${NC} 运行完整流水线 (普通装甲板)"
    echo -e "  ${GREEN}3)${NC} 运行前哨站预测流水线"
    echo -e "  ${GREEN}4)${NC} 选择视频运行"
    echo -e "  ${GREEN}5)${NC} 运行测试"
    echo -e "  ${GREEN}6)${NC} 查看输出视频"
    echo -e "  ${GREEN}7)${NC} 清理编译"
    echo -e "  ${GREEN}0)${NC} 退出"
    echo ""
    echo -n -e "${CYAN}选项: ${NC}"
}

setup_ros() {
    if [ -z "$ROS_DISTRO" ]; then
        source /opt/ros/humble/setup.bash 2>/dev/null
    fi
    source "${WS_PATH}/install/setup.bash" 2>/dev/null
}

# 编译
build_package() {
    echo -e "${BLUE}[INFO] 编译中...${NC}"
    setup_ros
    colcon build --packages-select ${PKG_NAME} --cmake-args -DCMAKE_BUILD_TYPE=Release
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[OK] 编译成功${NC}"
        source "${WS_PATH}/install/setup.bash"
    else
        echo -e "${RED}[ERROR] 编译失败${NC}"
    fi
    read -p "按回车继续..."
}

# 运行完整流水线（检测+解算+可视化）
run_full_pipeline() {
    local video_path="$1"
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="${OUTPUT_DIR}/result_${output_name}.mp4"
    
    echo -e "${BLUE}[INFO] 启动完整流水线${NC}"
    echo -e "${BLUE}[INFO] 输入: ${video_path}${NC}"
    echo -e "${BLUE}[INFO] 输出: ${output_path}${NC}"
    
    mkdir -p "${OUTPUT_DIR}"
    setup_ros
    
    ros2 launch ${PKG_NAME} full_pipeline_launch.py \
        video_path:="${video_path}" \
        output_path:="${output_path}" \
        fps:=30.0 \
        loop:=false \
        show_pose_info:=true
    
    echo -e "${GREEN}[OK] 完成: ${output_path}${NC}"
    read -p "按回车继续..."
}

# 运行前哨站预测流水线（检测+解算+预测+可视化）
run_outpost_pipeline() {
    local video_path="$1"
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="${OUTPUT_DIR}/outpost_${output_name}.mp4"
    
    echo -e "${BLUE}[INFO] 启动前哨站预测流水线${NC}"
    echo -e "${BLUE}[INFO] 输入: ${video_path}${NC}"
    echo -e "${BLUE}[INFO] 输出: ${output_path}${NC}"
    
    mkdir -p "${OUTPUT_DIR}"
    setup_ros
    
    ros2 launch ${PKG_NAME} outpost_pipeline_launch.py \
        video_path:="${video_path}" \
        output_path:="${output_path}" \
        fps:=30.0 \
        loop:=false \
        show_outpost_info:=true \
        show_trajectory:=true
    
    echo -e "${GREEN}[OK] 完成: ${output_path}${NC}"
    read -p "按回车继续..."
}

# 选择视频
select_video() {
    echo -e "${YELLOW}选择视频：${NC}"
    echo -e "  ${GREEN}1)${NC} 普通装甲板视频 (3.avi)"
    echo -e "  ${GREEN}2)${NC} 前哨站视频 (蓝方前哨站)"
    echo -e "  ${GREEN}3)${NC} 自定义路径"
    echo ""
    echo -n -e "${CYAN}选择: ${NC}"
    read video_choice
    
    local selected_video=""
    case $video_choice in
        1) selected_video="${VIDEO_ARMOR}" ;;
        2) selected_video="${VIDEO_OUTPOST}" ;;
        3)
            echo -n -e "${CYAN}输入路径: ${NC}"
            read custom_path
            if [ -f "${custom_path}" ]; then
                selected_video="${custom_path}"
            else
                echo -e "${RED}[ERROR] 文件不存在${NC}"
                read -p "按回车继续..."
                return
            fi
            ;;
        *) return ;;
    esac
    
    echo ""
    echo -e "${YELLOW}选择流水线：${NC}"
    echo -e "  ${GREEN}1)${NC} 完整流水线 (检测+解算+可视化)"
    echo -e "  ${GREEN}2)${NC} 前哨站预测 (检测+解算+预测+可视化)"
    echo ""
    echo -n -e "${CYAN}选择: ${NC}"
    read mode_choice
    
    case $mode_choice in
        1) run_full_pipeline "${selected_video}" ;;
        2) run_outpost_pipeline "${selected_video}" ;;
        *) run_full_pipeline "${selected_video}" ;;
    esac
}

# 运行测试
run_tests() {
    echo -e "${BLUE}[INFO] 运行测试...${NC}"
    setup_ros
    colcon test --packages-select ${PKG_NAME}
    colcon test-result --verbose
    read -p "按回车继续..."
}

# 查看输出
view_output() {
    if [ -d "${OUTPUT_DIR}" ]; then
        echo -e "${YELLOW}输出视频：${NC}"
        ls -lht "${OUTPUT_DIR}"/*.mp4 2>/dev/null | head -10
        echo ""
        echo -n -e "${CYAN}输入文件名播放 (或回车返回): ${NC}"
        read filename
        if [ -n "$filename" ] && [ -f "${OUTPUT_DIR}/${filename}" ]; then
            xdg-open "${OUTPUT_DIR}/${filename}" 2>/dev/null
        fi
    else
        echo -e "${YELLOW}[WARN] 无输出文件${NC}"
    fi
    read -p "按回车继续..."
}

# 清理
clean_build() {
    echo -n -e "${YELLOW}确认清理? [y/N]: ${NC}"
    read confirm
    if [ "$confirm" == "y" ] || [ "$confirm" == "Y" ]; then
        rm -rf "${WS_PATH}/build" "${WS_PATH}/install" "${WS_PATH}/log"
        echo -e "${GREEN}[OK] 清理完成${NC}"
    fi
    read -p "按回车继续..."
}

# ==============================================================================
# 主循环
# ==============================================================================
main() {
    cd "${WS_PATH}"
    while true; do
        show_header
        show_menu
        read choice
        
        case $choice in
            1) build_package ;;
            2) run_full_pipeline "${VIDEO_ARMOR}" ;;
            3) run_outpost_pipeline "${VIDEO_OUTPOST}" ;;
            4) select_video ;;
            5) run_tests ;;
            6) view_output ;;
            7) clean_build ;;
            0) echo -e "${GREEN}再见！${NC}"; exit 0 ;;
            *) echo -e "${RED}无效选项${NC}"; sleep 1 ;;
        esac
    done
}

main
