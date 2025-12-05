#!/bin/bash
# ==============================================================================
# 装甲板检测系统启动脚本
# ==============================================================================
# 功能：
#   - 编译ROS2包
#   - 启动检测系统
#   - 提供交互式菜单选择不同功能
# ==============================================================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 工作空间路径
WS_PATH="/home/xin/ROS2Drone/ws"
PKG_NAME="armor_detector_ros2"
OUTPUT_DIR="${WS_PATH}/output"

# 默认配置
DEFAULT_VIDEO="/home/xin/ROS2Drone/OrangeAim-Drone/utils/红方前哨站公路视角全速.mp4"
DEFAULT_VIDEO2="/home/xin/ROS2Drone/OrangeAim-Drone/utils/蓝方前哨站狙击点视角全速.mp4"

# ==============================================================================
# 函数定义
# ==============================================================================

# 显示标题
show_header() {
    clear
    echo -e "${CYAN}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║         装甲板关键点检测与分类系统 (ROS2版本)                ║"
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║  基于 YOLOX + OpenVINO 的实时装甲板检测                      ║"
    echo "║  支持4角点关键点检测和数字分类                               ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

# 显示主菜单
show_main_menu() {
    echo -e "${YELLOW}请选择操作：${NC}"
    echo ""
    echo -e "  ${GREEN}1)${NC} 编译ROS2包"
    echo -e "  ${CYAN}--- 检测系统 ---${NC}"
    echo -e "  ${GREEN}2)${NC} 运行检测系统 (多进程版本)"
    echo -e "  ${GREEN}3)${NC} 运行检测系统 (零拷贝版本) ${YELLOW}[推荐]${NC}"
    echo -e "  ${GREEN}4)${NC} 选择视频文件运行"
    echo -e "  ${CYAN}--- 单独节点 ---${NC}"
    echo -e "  ${GREEN}5)${NC} 单独运行视频发布节点"
    echo -e "  ${GREEN}6)${NC} 单独运行检测节点"
    echo -e "  ${GREEN}7)${NC} 单独运行可视化节点"
    echo -e "  ${CYAN}--- 其他 ---${NC}"
    echo -e "  ${GREEN}8)${NC} 查看输出视频"
    echo -e "  ${GREEN}9)${NC} 清理编译文件"
    echo -e "  ${GREEN}s)${NC} 查看系统信息"
    echo -e "  ${GREEN}0)${NC} 退出"
    echo ""
    echo -n -e "${CYAN}请输入选项: ${NC}"
}

# 编译包
build_package() {
    echo -e "${BLUE}[INFO] 开始编译ROS2包...${NC}"
    cd "${WS_PATH}"
    
    # 检查是否source了ROS2环境
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${YELLOW}[WARN] 未检测到ROS2环境，尝试source...${NC}"
        source /opt/ros/humble/setup.bash 2>/dev/null || \
        source /opt/ros/iron/setup.bash 2>/dev/null || \
        source /opt/ros/rolling/setup.bash 2>/dev/null
    fi
    
    echo -e "${BLUE}[INFO] ROS2版本: ${ROS_DISTRO}${NC}"
    
    # 编译
    colcon build --packages-select ${PKG_NAME} --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[SUCCESS] 编译成功！${NC}"
        # source安装目录
        source "${WS_PATH}/install/setup.bash"
    else
        echo -e "${RED}[ERROR] 编译失败！${NC}"
    fi
    
    echo ""
    read -p "按回车键继续..."
}

# 运行检测系统（多进程版本）
run_detection() {
    local video_path="$1"
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="${OUTPUT_DIR}/detection_${output_name}.mp4"
    
    echo -e "${BLUE}[INFO] 启动检测系统 (多进程版本)...${NC}"
    echo -e "${BLUE}[INFO] 输入视频: ${video_path}${NC}"
    echo -e "${BLUE}[INFO] 输出视频: ${output_path}${NC}"
    
    mkdir -p "${OUTPUT_DIR}"
    
    source /opt/ros/humble/setup.bash 2>/dev/null || \
    source /opt/ros/iron/setup.bash 2>/dev/null
    source "${WS_PATH}/install/setup.bash"
    
    ros2 launch ${PKG_NAME} detector_launch.py \
        video_path:="${video_path}" \
        output_path:="${output_path}" \
        fps:=30.0 \
        loop:=false
    
    echo ""
    echo -e "${GREEN}[INFO] 检测完成，输出视频保存在: ${output_path}${NC}"
    read -p "按回车键继续..."
}

# 运行检测系统（零拷贝版本 - 同进程多线程）
run_detection_zero_copy() {
    local video_path="$1"
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="${OUTPUT_DIR}/detection_zc_${output_name}.mp4"
    
    echo -e "${CYAN}[INFO] 启动检测系统 (零拷贝版本)...${NC}"
    echo -e "${CYAN}[INFO] 特点: 同进程运行，图像零拷贝传输${NC}"
    echo -e "${BLUE}[INFO] 输入视频: ${video_path}${NC}"
    echo -e "${BLUE}[INFO] 输出视频: ${output_path}${NC}"
    
    mkdir -p "${OUTPUT_DIR}"
    
    source /opt/ros/humble/setup.bash 2>/dev/null || \
    source /opt/ros/iron/setup.bash 2>/dev/null
    source "${WS_PATH}/install/setup.bash"
    
    ros2 launch ${PKG_NAME} detector_zero_copy_launch.py \
        video_path:="${video_path}" \
        output_path:="${output_path}" \
        fps:=30.0 \
        loop:=false
    
    echo ""
    echo -e "${GREEN}[INFO] 检测完成，输出视频保存在: ${output_path}${NC}"
    read -p "按回车键继续..."
}

# 选择视频文件并选择运行模式
select_video() {
    echo -e "${YELLOW}可用的测试视频：${NC}"
    echo ""
    echo -e "  ${GREEN}1)${NC} 红方前哨站公路视角全速.mp4"
    echo -e "  ${GREEN}2)${NC} 蓝方前哨站狙击点视角全速.mp4"
    echo -e "  ${GREEN}3)${NC} 输入自定义路径"
    echo ""
    echo -n -e "${CYAN}请选择视频 [1-3]: ${NC}"
    read video_choice
    
    local selected_video=""
    case $video_choice in
        1) selected_video="${DEFAULT_VIDEO}" ;;
        2) selected_video="${DEFAULT_VIDEO2}" ;;
        3)
            echo -n -e "${CYAN}请输入视频文件完整路径: ${NC}"
            read custom_path
            if [ -f "${custom_path}" ]; then
                selected_video="${custom_path}"
            else
                echo -e "${RED}[ERROR] 文件不存在: ${custom_path}${NC}"
                read -p "按回车键继续..."
                return
            fi
            ;;
        *)
            echo -e "${RED}[ERROR] 无效选项${NC}"
            read -p "按回车键继续..."
            return
            ;;
    esac
    
    # 选择运行模式
    echo ""
    echo -e "${YELLOW}选择运行模式：${NC}"
    echo -e "  ${GREEN}1)${NC} 多进程版本 (标准ROS2通信)"
    echo -e "  ${GREEN}2)${NC} 零拷贝版本 (同进程零拷贝) ${YELLOW}[推荐]${NC}"
    echo ""
    echo -n -e "${CYAN}请选择模式 [1-2]: ${NC}"
    read mode_choice
    
    case $mode_choice in
        1) run_detection "${selected_video}" ;;
        2) run_detection_zero_copy "${selected_video}" ;;
        *) run_detection_zero_copy "${selected_video}" ;;  # 默认零拷贝
    esac
}

# 单独运行节点
run_single_node() {
    local node_name="$1"
    
    source /opt/ros/humble/setup.bash 2>/dev/null || \
    source /opt/ros/iron/setup.bash 2>/dev/null
    source "${WS_PATH}/install/setup.bash"
    
    case $node_name in
        "video")
            echo -e "${BLUE}[INFO] 启动视频发布节点...${NC}"
            ros2 run ${PKG_NAME} video_publisher_node \
                --ros-args -p video_path:="${WS_PATH}/videos/video.avi" -p fps:=30.0 -p loop:=true
            ;;
        "detector")
            echo -e "${BLUE}[INFO] 启动检测节点...${NC}"
            ros2 run ${PKG_NAME} detector_node \
                --ros-args \
                -p armor_model_path:="/home/xin/ROS2Drone/OrangeAim-Drone/utils/models/armor_yolo_x.xml" \
                -p classifier_model_path:="/home/xin/ROS2Drone/OrangeAim-Drone/utils/models/classifier.xml"
            ;;
        "visualizer")
            echo -e "${BLUE}[INFO] 启动可视化节点...${NC}"
            ros2 run ${PKG_NAME} visualizer_node \
                --ros-args -p output_video_path:="${OUTPUT_DIR}/output.mp4" -p fps:=30.0
            ;;
    esac
    
    echo ""
    read -p "按回车键继续..."
}

# 查看输出视频
view_output() {
    echo -e "${YELLOW}输出目录内容：${NC}"
    echo ""
    
    if [ -d "${OUTPUT_DIR}" ]; then
        ls -lh "${OUTPUT_DIR}"
        echo ""
        
        # 列出所有视频文件
        videos=(${OUTPUT_DIR}/*.mp4)
        if [ ${#videos[@]} -gt 0 ] && [ -f "${videos[0]}" ]; then
            echo -e "${CYAN}选择要播放的视频:${NC}"
            select video in "${videos[@]}" "返回"; do
                if [ "$video" == "返回" ]; then
                    break
                elif [ -f "$video" ]; then
                    echo -e "${BLUE}[INFO] 播放: ${video}${NC}"
                    # 使用系统默认播放器
                    xdg-open "$video" 2>/dev/null || \
                    vlc "$video" 2>/dev/null || \
                    ffplay "$video" 2>/dev/null || \
                    echo -e "${RED}[ERROR] 未找到可用的视频播放器${NC}"
                    break
                fi
            done
        else
            echo -e "${YELLOW}[WARN] 没有找到视频文件${NC}"
        fi
    else
        echo -e "${YELLOW}[WARN] 输出目录不存在${NC}"
    fi
    
    echo ""
    read -p "按回车键继续..."
}

# 清理编译文件
clean_build() {
    echo -e "${YELLOW}[WARN] 即将删除 build, install, log 目录${NC}"
    echo -n -e "${CYAN}确认删除? [y/N]: ${NC}"
    read confirm
    
    if [ "$confirm" == "y" ] || [ "$confirm" == "Y" ]; then
        cd "${WS_PATH}"
        rm -rf build install log
        echo -e "${GREEN}[SUCCESS] 清理完成${NC}"
    else
        echo -e "${BLUE}[INFO] 取消清理${NC}"
    fi
    
    echo ""
    read -p "按回车键继续..."
}

# 显示系统信息
show_system_info() {
    echo -e "${CYAN}========== 系统信息 ==========${NC}"
    echo ""
    
    # ROS2版本
    echo -e "${YELLOW}ROS2 环境:${NC}"
    if [ -n "$ROS_DISTRO" ]; then
        echo "  ROS_DISTRO: ${ROS_DISTRO}"
    else
        echo "  未检测到ROS2环境"
    fi
    echo ""
    
    # OpenVINO版本
    echo -e "${YELLOW}OpenVINO:${NC}"
    if command -v python3 &> /dev/null; then
        python3 -c "from openvino.runtime import Core; c = Core(); print(f'  版本: {c.get_versions()[\"CPU\"].build_number}')" 2>/dev/null || \
        echo "  未检测到OpenVINO"
    fi
    echo ""
    
    # OpenCV版本
    echo -e "${YELLOW}OpenCV:${NC}"
    pkg-config --modversion opencv4 2>/dev/null && echo "" || \
    echo "  未检测到OpenCV"
    echo ""
    
    # 模型文件
    echo -e "${YELLOW}模型文件:${NC}"
    ls -lh /home/xin/ROS2Drone/OrangeAim-Drone/utils/models/*.xml 2>/dev/null || \
    echo "  模型文件不存在"
    echo ""
    
    # 测试视频
    echo -e "${YELLOW}测试视频:${NC}"
    ls -lh /home/xin/ROS2Drone/OrangeAim-Drone/utils/*.mp4 2>/dev/null || \
    echo "  测试视频不存在"
    echo ""
    
    read -p "按回车键继续..."
}

# ==============================================================================
# 主循环
# ==============================================================================
main() {
    while true; do
        show_header
        show_main_menu
        read choice
        
        case $choice in
            1)
                build_package
                ;;
            2)
                run_detection "${DEFAULT_VIDEO}"
                ;;
            3)
                run_detection_zero_copy "${DEFAULT_VIDEO}"
                ;;
            4)
                select_video
                ;;
            5)
                run_single_node "video"
                ;;
            6)
                run_single_node "detector"
                ;;
            7)
                run_single_node "visualizer"
                ;;
            8)
                view_output
                ;;
            9)
                clean_build
                ;;
            s|S)
                show_system_info
                ;;
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
