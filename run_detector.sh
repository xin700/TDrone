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

# 配置 (使用 Docker 容器内路径 ~/droneAim/TDrone)
WS_PATH="/home/user/droneAim/TDrone"
PKG_NAME="armor_detector_ros2"
OUTPUT_DIR="${WS_PATH}/output"

# 测试视频 (放在 ~/droneAim/TDrone/videos 目录下)
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
    echo -e "  ${GREEN}3)${NC} 运行前哨站预测流水线 (V1)"
    echo -e "  ${GREEN}4)${NC} 运行前哨站预测流水线 (V2 多假设追踪)"
    echo -e "  ${GREEN}5)${NC} 运行前哨站预测流水线 (V3 极坐标观测)"
    echo -e "  ${GREEN}6)${NC} 运行前哨站预测流水线 (V4 θ观测模型) ${CYAN}[推荐]${NC}"
    echo -e "  ${GREEN}7)${NC} 选择视频运行"
    echo -e "  ${GREEN}8)${NC} 运行测试"
    echo -e "  ${GREEN}9)${NC} 查看输出视频"
    echo -e "  ${GREEN}c)${NC} 清理编译"
    echo ""
    echo -e "  ${CYAN}--- 节点调试 ---${NC}"
    echo -e "  ${GREEN}d)${NC} Solver 节点调试 (PnP解算可视化)"
    echo ""
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
    local use_imu_file="${2:-false}"
    local imu_file_path="${3:-}"
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
    if [ "$use_imu_file" == "true" ] && [ -n "$imu_file_path" ]; then
        echo -e "${BLUE}[INFO] IMU: ${imu_file_path}${NC}"
    else
        echo -e "${BLUE}[INFO] IMU: 模拟 (静态)${NC}"
    fi
    
    mkdir -p "${OUTPUT_DIR}"
    setup_ros
    
    ros2 launch ${PKG_NAME} full_pipeline_launch.py \
        video_path:="${video_path}" \
        output_path:="${output_path}" \
        fps:="${fps}" \
        loop:=false \
        show_pose_info:=true \
        use_imu_file:="${use_imu_file}" \
        imu_file_path:="${imu_file_path}"
    
    echo -e "${GREEN}[OK] 完成: ${output_path}${NC}"
    read -p "按回车继续..."
}

# 运行前哨站预测流水线（检测+解算+预测+可视化）
run_outpost_pipeline() {
    local video_path="$1"
    local use_imu_file="${2:-false}"
    local imu_file_path="${3:-}"
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
    if [ "$use_imu_file" == "true" ] && [ -n "$imu_file_path" ]; then
        echo -e "${BLUE}[INFO] IMU: ${imu_file_path}${NC}"
    else
        echo -e "${BLUE}[INFO] IMU: 模拟 (静态)${NC}"
    fi
    
    mkdir -p "${OUTPUT_DIR}"
    setup_ros
    
    ros2 launch ${PKG_NAME} outpost_pipeline_launch.py \
        video_path:="${video_path}" \
        output_path:="${output_path}" \
        fps:="${fps}" \
        loop:=false \
        show_outpost_info:=true \
        show_trajectory:=true \
        use_imu_file:="${use_imu_file}" \
        imu_file_path:="${imu_file_path}"
    
    echo -e "${GREEN}[OK] 完成: ${output_path}${NC}"
    read -p "按回车继续..."
}

# 运行前哨站预测 V2 流水线（多假设追踪）
run_outpost_v2_pipeline() {
    local video_path="$1"
    local show_window="${2:-false}"
    local use_imu_file="${3:-false}"
    local imu_file_path="${4:-}"
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
    if [ "$show_window" == "true" ]; then
        echo -e "${CYAN}  - 实时显示窗口: 开启${NC}"
    fi
    echo -e "${BLUE}[INFO] 输入: ${video_path}${NC}"
    echo -e "${BLUE}[INFO] 输出: ${output_path}${NC}"
    echo -e "${BLUE}[INFO] FPS: ${fps}${NC}"
    if [ "$use_imu_file" == "true" ] && [ -n "$imu_file_path" ]; then
        echo -e "${BLUE}[INFO] IMU: ${imu_file_path}${NC}"
    else
        echo -e "${BLUE}[INFO] IMU: 模拟 (静态)${NC}"
    fi
    
    mkdir -p "${OUTPUT_DIR}"
    setup_ros
    
    ros2 launch ${PKG_NAME} outpost_pipeline_v2_launch.py \
        video_path:="${video_path}" \
        output_path:="${output_path}" \
        fps:="${fps}" \
        loop:=false \
        show_window:="${show_window}" \
        show_outpost_info:=true \
        show_trajectory:=true \
        use_imu_file:="${use_imu_file}" \
        imu_file_path:="${imu_file_path}"
    
    echo -e "${GREEN}[OK] 完成: ${output_path}${NC}"
    read -p "按回车继续..."
}

# 运行前哨站预测 V3 流水线（极坐标观测模型）
run_outpost_v3_pipeline() {
    local video_path="$1"
    local show_window="${2:-false}"
    local use_imu_file="${3:-false}"
    local imu_file_path="${4:-}"
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="${OUTPUT_DIR}/outpost_v3_${output_name}.mp4"
    
    # 询问 fps
    echo -n -e "${CYAN}视频帧率 (默认30.0): ${NC}"
    read fps_input
    local fps="${fps_input:-30.0}"
    
    echo -e "${BLUE}[INFO] 启动前哨站预测流水线 V3 (极坐标观测)${NC}"
    echo -e "${CYAN}  - 极坐标观测模型 [yaw, pitch, dist, orient]${NC}"
    echo -e "${CYAN}  - 状态向量 [θ_center, ω, x_c, y_c, z_c]${NC}"
    echo -e "${CYAN}  - Mahalanobis距离多假设跟踪${NC}"
    echo -e "${CYAN}  - 装甲板切换检测与处理${NC}"
    echo -e "${CYAN}  - Direct/Indirect瞄准策略${NC}"
    if [ "$show_window" == "true" ]; then
        echo -e "${CYAN}  - 实时显示窗口: 开启${NC}"
    fi
    echo -e "${BLUE}[INFO] 输入: ${video_path}${NC}"
    echo -e "${BLUE}[INFO] 输出: ${output_path}${NC}"
    echo -e "${BLUE}[INFO] FPS: ${fps}${NC}"
    if [ "$use_imu_file" == "true" ] && [ -n "$imu_file_path" ]; then
        echo -e "${BLUE}[INFO] IMU: ${imu_file_path}${NC}"
    else
        echo -e "${BLUE}[INFO] IMU: 模拟 (静态)${NC}"
    fi
    
    # 询问角速度设置
    echo ""
    echo -e "${YELLOW}角速度设置：${NC}"
    echo -e "  ${GREEN}1)${NC} 标准转速 (0.8π ≈ 2.513 rad/s)"
    echo -e "  ${GREEN}2)${NC} 慢速 (0.6π ≈ 1.885 rad/s)"
    echo -e "  ${GREEN}3)${NC} 自动估计 (关闭固定角速度)"
    echo -e "  ${GREEN}4)${NC} 自定义转速"
    echo -n -e "${CYAN}选择 [1]: ${NC}"
    read omega_choice
    
    local use_fixed="true"
    local omega_value="2.513"
    
    case $omega_choice in
        2) omega_value="1.885" ;;
        3) use_fixed="false" ;;
        4)
            echo -n -e "${CYAN}输入转速 (rad/s): ${NC}"
            read omega_value
            ;;
        *) omega_value="2.513" ;;
    esac
    
    mkdir -p "${OUTPUT_DIR}"
    setup_ros
    
    echo -e "${BLUE}[INFO] 使用角速度: ${omega_value} rad/s (固定=${use_fixed})${NC}"
    
    ros2 launch ${PKG_NAME} outpost_pipeline_v3_launch.py \
        video_path:="${video_path}" \
        output_path:="${output_path}" \
        fps:="${fps}" \
        loop:=false \
        show_window:="${show_window}" \
        use_fixed_omega:="${use_fixed}" \
        outpost_omega:="${omega_value}" \
        use_imu_file:="${use_imu_file}" \
        imu_file_path:="${imu_file_path}"
    
    echo -e "${GREEN}[OK] 完成: ${output_path}${NC}"
    read -p "按回车继续..."
}

# 运行前哨站预测 V4 流水线（θ观测模型）
run_outpost_v4_pipeline() {
    local video_path="$1"
    local show_window="${2:-false}"
    local use_imu_file="${3:-false}"
    local imu_file_path="${4:-}"
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="${OUTPUT_DIR}/outpost_v4_${output_name}.mp4"
    
    # 询问 fps
    echo -n -e "${CYAN}视频帧率 (默认30.0): ${NC}"
    read fps_input
    local fps="${fps_input:-30.0}"
    
    echo -e "${BLUE}[INFO] 启动前哨站预测流水线 V4 (θ观测模型)${NC}"
    echo -e "${CYAN}  - 基于长宽比的θ观测: θ_abs = arccos(aspect_ratio / 2.58)${NC}"
    echo -e "${CYAN}  - 分阶段初始化: 方向→高度→θ→EKF${NC}"
    echo -e "${CYAN}  - 5维状态空间: [ang, ω, x_c, y_c, z_c]${NC}"
    echo -e "${CYAN}  - 高度状态机与EKF交叉验证${NC}"
    echo -e "${CYAN}  - 平台区插值θ计算${NC}"
    if [ "$show_window" == "true" ]; then
        echo -e "${CYAN}  - 实时显示窗口: 开启${NC}"
    fi
    echo -e "${BLUE}[INFO] 输入: ${video_path}${NC}"
    echo -e "${BLUE}[INFO] 输出: ${output_path}${NC}"
    echo -e "${BLUE}[INFO] FPS: ${fps}${NC}"
    if [ "$use_imu_file" == "true" ] && [ -n "$imu_file_path" ]; then
        echo -e "${BLUE}[INFO] IMU: ${imu_file_path}${NC}"
    else
        echo -e "${BLUE}[INFO] IMU: 模拟 (静态)${NC}"
    fi
    
    # 询问角速度设置
    echo ""
    echo -e "${YELLOW}角速度设置：${NC}"
    echo -e "  ${GREEN}1)${NC} 标准转速 (0.8π ≈ 2.513 rad/s)"
    echo -e "  ${GREEN}2)${NC} 慢速 (0.6π ≈ 1.885 rad/s)"
    echo -e "  ${GREEN}3)${NC} 自定义转速"
    echo -n -e "${CYAN}选择 [1]: ${NC}"
    read omega_choice
    
    local omega_value="2.513"
    
    case $omega_choice in
        2) omega_value="1.885" ;;
        3)
            echo -n -e "${CYAN}输入转速 (rad/s): ${NC}"
            read omega_value
            ;;
        *) omega_value="2.513" ;;
    esac
    
    mkdir -p "${OUTPUT_DIR}"
    setup_ros
    
    echo -e "${BLUE}[INFO] 使用角速度: ${omega_value} rad/s${NC}"
    
    ros2 launch ${PKG_NAME} outpost_pipeline_v4_launch.py \
        video_path:="${video_path}" \
        output_path:="${output_path}" \
        fps:="${fps}" \
        loop:=false \
        show_window:="${show_window}" \
        outpost_omega:="${omega_value}" \
        use_imu_file:="${use_imu_file}" \
        imu_file_path:="${imu_file_path}"
    
    echo -e "${GREEN}[OK] 完成: ${output_path}${NC}"
    read -p "按回车继续..."
}

# 选择视频
select_video() {
    echo -e "${YELLOW}选择视频：${NC}"
    echo -e "  ${GREEN}1)${NC} 普通装甲板视频 (3.avi)"
    echo -e "  ${GREEN}2)${NC} 前哨站视频 (蓝方前哨站)"
    echo -e "  ${GREEN}3)${NC} 从 ~/Videos 选择录制的视频"
    echo -e "  ${GREEN}4)${NC} 自定义路径"
    echo ""
    echo -n -e "${CYAN}选择: ${NC}"
    read video_choice
    
    local selected_video=""
    local selected_imu=""
    
    case $video_choice in
        1) selected_video="${VIDEO_ARMOR}" ;;
        2) selected_video="${VIDEO_OUTPOST}" ;;
        3)
            # 列出 ~/Videos 中的视频文件
            echo ""
            echo -e "${YELLOW}可用的录制视频：${NC}"
            local videos=($(ls -1 /home/user/Videos/*.avi 2>/dev/null | sort))
            if [ ${#videos[@]} -eq 0 ]; then
                echo -e "${RED}[ERROR] 没有找到视频文件${NC}"
                read -p "按回车继续..."
                return
            fi
            
            local i=1
            for v in "${videos[@]}"; do
                local basename=$(basename "$v")
                local imu_file="${v%.avi}.txt"
                if [ -f "$imu_file" ]; then
                    echo -e "  ${GREEN}${i})${NC} ${basename} ${CYAN}[有IMU]${NC}"
                else
                    echo -e "  ${GREEN}${i})${NC} ${basename}"
                fi
                ((i++))
            done
            
            echo ""
            echo -n -e "${CYAN}选择视频序号: ${NC}"
            read video_num
            
            if [ "$video_num" -ge 1 ] && [ "$video_num" -le ${#videos[@]} ] 2>/dev/null; then
                selected_video="${videos[$((video_num-1))]}"
                # 自动查找对应的 IMU 文件
                local imu_file="${selected_video%.avi}.txt"
                if [ -f "$imu_file" ]; then
                    selected_imu="$imu_file"
                    echo -e "${GREEN}[INFO] 找到匹配的 IMU 文件: $(basename $imu_file)${NC}"
                fi
            else
                echo -e "${RED}[ERROR] 无效选择${NC}"
                read -p "按回车继续..."
                return
            fi
            ;;
        4)
            echo -n -e "${CYAN}输入视频路径: ${NC}"
            read custom_path
            if [ -f "${custom_path}" ]; then
                selected_video="${custom_path}"
                # 尝试查找同名 IMU 文件
                local imu_file="${custom_path%.*}.txt"
                if [ -f "$imu_file" ]; then
                    selected_imu="$imu_file"
                    echo -e "${GREEN}[INFO] 找到匹配的 IMU 文件: $(basename $imu_file)${NC}"
                fi
            else
                echo -e "${RED}[ERROR] 文件不存在${NC}"
                read -p "按回车继续..."
                return
            fi
            ;;
        *) return ;;
    esac
    
    # 询问是否使用 IMU 文件
    local use_imu="false"
    if [ -n "$selected_imu" ]; then
        echo -n -e "${CYAN}使用关联的 IMU 数据? [Y/n]: ${NC}"
        read use_imu_choice
        if [ "$use_imu_choice" != "n" ] && [ "$use_imu_choice" != "N" ]; then
            use_imu="true"
        fi
    else
        echo -e "${YELLOW}[INFO] 未找到匹配的 IMU 文件，将使用模拟 IMU${NC}"
    fi
    
    echo ""
    echo -e "${YELLOW}选择流水线：${NC}"
    echo -e "  ${GREEN}1)${NC} 完整流水线 (检测+解算+可视化)"
    echo -e "  ${GREEN}2)${NC} 前哨站预测 V1 (检测+解算+预测+可视化)"
    echo -e "  ${GREEN}3)${NC} 前哨站预测 V2 (多假设追踪)"
    echo -e "  ${GREEN}4)${NC} 前哨站预测 V2 + 实时显示窗口"
    echo -e "  ${GREEN}5)${NC} 前哨站预测 V3 (极坐标观测)"
    echo -e "  ${GREEN}6)${NC} 前哨站预测 V3 + 实时显示窗口"
    echo -e "  ${GREEN}7)${NC} 前哨站预测 V4 (θ观测模型) ${CYAN}[推荐]${NC}"
    echo -e "  ${GREEN}8)${NC} 前哨站预测 V4 + 实时显示窗口"
    echo -e "  ${GREEN}9)${NC} Solver 调试 (PnP解算可视化)"
    echo ""
    echo -n -e "${CYAN}选择: ${NC}"
    read mode_choice
    
    case $mode_choice in
        1) run_full_pipeline "${selected_video}" "${use_imu}" "${selected_imu}" ;;
        2) run_outpost_pipeline "${selected_video}" "${use_imu}" "${selected_imu}" ;;
        3) run_outpost_v2_pipeline "${selected_video}" false "${use_imu}" "${selected_imu}" ;;
        4) run_outpost_v2_pipeline "${selected_video}" true "${use_imu}" "${selected_imu}" ;;
        5) run_outpost_v3_pipeline "${selected_video}" false "${use_imu}" "${selected_imu}" ;;
        6) run_outpost_v3_pipeline "${selected_video}" true "${use_imu}" "${selected_imu}" ;;
        7) run_outpost_v4_pipeline "${selected_video}" false "${use_imu}" "${selected_imu}" ;;
        8) run_outpost_v4_pipeline "${selected_video}" true "${use_imu}" "${selected_imu}" ;;
        9) run_solver_debug "${selected_video}" true "${use_imu}" "${selected_imu}" ;;
        *) run_full_pipeline "${selected_video}" "${use_imu}" "${selected_imu}" ;;
    esac
}

# ==============================================================================
# Solver 节点调试
# ==============================================================================
run_solver_debug() {
    local video_path="$1"
    local show_window="${2:-true}"
    local use_imu_file="${3:-false}"
    local imu_file_path="${4:-}"
    
    # 如果没有传入视频路径，进行视频选择
    if [ -z "$video_path" ] || [ "$video_path" == "${VIDEO_ARMOR}" ]; then
        echo -e "${YELLOW}选择调试视频：${NC}"
        echo ""
        
        # 列出 /home/user/droneAim/TDrone/videos 中的视频文件
        local videos=($(ls -1 /home/user/droneAim/TDrone/videos/*.{avi,mp4,mov} 2>/dev/null | sort))
        if [ ${#videos[@]} -eq 0 ]; then
            echo -e "${RED}[ERROR] 在 /home/user/droneAim/TDrone/videos 中没有找到视频文件${NC}"
            read -p "按回车继续..."
            return
        fi
        
        local i=1
        for v in "${videos[@]}"; do
            local basename=$(basename "$v")
            local imu_file="${v%.*}.txt"
            if [ -f "$imu_file" ]; then
                echo -e "  ${GREEN}${i})${NC} ${basename} ${CYAN}[有IMU]${NC}"
            else
                echo -e "  ${GREEN}${i})${NC} ${basename}"
            fi
            ((i++))
        done
        
        echo ""
        echo -n -e "${CYAN}选择视频序号: ${NC}"
        read video_num
        
        if [ "$video_num" -ge 1 ] && [ "$video_num" -le ${#videos[@]} ] 2>/dev/null; then
            video_path="${videos[$((video_num-1))]}"
            # 自动查找对应的 IMU 文件
            local auto_imu_file="${video_path%.*}.txt"
            if [ -f "$auto_imu_file" ]; then
                imu_file_path="$auto_imu_file"
                use_imu_file="true"
                echo -e "${GREEN}[INFO] 找到匹配的 IMU 文件: $(basename $auto_imu_file)${NC}"
                echo -n -e "${CYAN}使用此 IMU 文件? [Y/n]: ${NC}"
                read use_imu_choice
                if [ "$use_imu_choice" == "n" ] || [ "$use_imu_choice" == "N" ]; then
                    use_imu_file="false"
                    imu_file_path=""
                fi
            fi
        else
            echo -e "${RED}[ERROR] 无效选择${NC}"
            read -p "按回车继续..."
            return
        fi
        echo ""
    fi
    
    local output_name=$(date +"%Y%m%d_%H%M%S")
    local output_path="${OUTPUT_DIR}/solver_debug_${output_name}.avi"
    
    echo -e "${BLUE}[INFO] 启动 Solver 调试模式${NC}"
    echo -e "${CYAN}  - 检测 (detector_node)${NC}"
    echo -e "${CYAN}  - PnP解算 (solver_node)${NC}"
    echo -e "${CYAN}  - 可视化调试 (solver_visualizer_node)${NC}"
    echo ""
    echo -e "${YELLOW}可视化内容：${NC}"
    echo -e "  - 装甲板角点 (0,1,2,3 标注)"
    echo -e "  - PnP 解算结果 (yaw, pitch, distance)"
    echo -e "  - 3D 位置 (X, Y, Z)"
    echo -e "  - 坐标轴投影"
    echo -e "  - IMU 状态"
    echo ""
    echo -e "${YELLOW}快捷键：${NC}"
    echo -e "  [A] 切换坐标轴显示"
    echo -e "  [G] 切换网格显示"
    echo -e "  [Q/ESC] 退出"
    echo ""
    
    # 如果没有传入 IMU 文件，询问是否使用
    local imu_time_offset="0.0"
    if [ "$use_imu_file" != "true" ]; then
        echo -e "${YELLOW}IMU 数据源：${NC}"
        echo -e "  ${GREEN}1)${NC} 模拟 IMU (静态)"
        echo -e "  ${GREEN}2)${NC} 从文件读取 IMU 数据"
        echo -n -e "${CYAN}选择 [1]: ${NC}"
        read imu_choice
        
        if [ "$imu_choice" == "2" ]; then
            use_imu_file="true"
            
            # 尝试自动匹配 IMU 文件
            local video_basename=$(basename "${video_path}" | sed 's/\.[^.]*$//')
            local auto_imu_file="/home/user/droneAim/TDrone/videos/${video_basename}.txt"
            
            if [ -f "$auto_imu_file" ]; then
                echo -e "${GREEN}[INFO] 找到匹配的 IMU 文件: ${auto_imu_file}${NC}"
                echo -n -e "${CYAN}使用此文件? [Y/n]: ${NC}"
                read use_auto
                if [ "$use_auto" != "n" ] && [ "$use_auto" != "N" ]; then
                    imu_file_path="$auto_imu_file"
                fi
            fi
            
            if [ -z "$imu_file_path" ]; then
                echo -e "${YELLOW}可用的 IMU 文件：${NC}"
                ls -1 /home/user/droneAim/TDrone/videos/*.txt 2>/dev/null | head -10
                echo ""
                echo -n -e "${CYAN}输入 IMU 文件路径: ${NC}"
                read imu_file_path
                
                if [ ! -f "$imu_file_path" ]; then
                    echo -e "${RED}[ERROR] 文件不存在: ${imu_file_path}${NC}"
                    echo -e "${YELLOW}[INFO] 使用模拟 IMU${NC}"
                    use_imu_file="false"
                fi
            fi
        fi
    fi
    
    if [ "$use_imu_file" == "true" ] && [ -n "$imu_file_path" ]; then
        echo -n -e "${CYAN}IMU 时间偏移（秒，默认0）: ${NC}"
        read offset_input
        if [ -n "$offset_input" ]; then
            imu_time_offset="$offset_input"
        fi
    fi
    
    # 询问 fps
    echo -n -e "${CYAN}视频帧率 (默认30.0): ${NC}"
    read fps_input
    local fps="${fps_input:-30.0}"
    
    echo ""
    echo -e "${BLUE}[INFO] 输入: ${video_path}${NC}"
    echo -e "${BLUE}[INFO] 输出: ${output_path}${NC}"
    echo -e "${BLUE}[INFO] FPS: ${fps}${NC}"
    if [ "$use_imu_file" == "true" ] && [ -n "$imu_file_path" ]; then
        echo -e "${BLUE}[INFO] IMU: ${imu_file_path} (偏移: ${imu_time_offset}s)${NC}"
    else
        echo -e "${BLUE}[INFO] IMU: 模拟 (静态)${NC}"
    fi
    
    mkdir -p "${OUTPUT_DIR}"
    setup_ros
    
    ros2 launch ${PKG_NAME} solver_debug_launch.py \
        video_path:="${video_path}" \
        output_path:="${output_path}" \
        fps:="${fps}" \
        loop:=false \
        show_window:="${show_window}" \
        show_axis:=true \
        show_grid:=true \
        use_imu_file:="${use_imu_file}" \
        imu_file_path:="${imu_file_path}" \
        imu_time_offset:="${imu_time_offset}"
    
    echo -e "${GREEN}[OK] 完成: ${output_path}${NC}"
    read -p "按回车继续..."
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
            4) run_outpost_v2_pipeline "${VIDEO_OUTPOST}" false ;;
            5) run_outpost_v3_pipeline "${VIDEO_OUTPOST}" false ;;
            6) run_outpost_v4_pipeline "${VIDEO_OUTPOST}" false ;;
            7) select_video ;;
            8) run_tests ;;
            9) view_output ;;
            c|C) clean_build ;;
            d|D) run_solver_debug "${VIDEO_ARMOR}" true ;;
            0) echo -e "${GREEN}再见！${NC}"; exit 0 ;;
            *) echo -e "${RED}无效选项${NC}"; sleep 1 ;;
        esac
    done
}

main
