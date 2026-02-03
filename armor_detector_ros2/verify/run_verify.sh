#!/bin/bash
# 编译和运行PnP验证程序

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 默认路径
MODEL_PATH="${SCRIPT_DIR}/../models/0526.onnx"
VIDEO_PATH="${SCRIPT_DIR}/../../videos/r1.avi"

# 解析参数
if [ ! -z "$1" ]; then
    VIDEO_PATH="$1"
fi
if [ ! -z "$2" ]; then
    MODEL_PATH="$2"
fi

# 颜色过滤: 0=蓝, 1=红, 2=全部
COLOR_FLAG="${3:-2}"

# 编译
echo "=== 编译 verify_pnp ==="
mkdir -p build
cd build

# 设置OpenVINO环境
source /opt/intel/openvino_2023/setupvars.sh 2>/dev/null || \
source /opt/intel/openvino/setupvars.sh 2>/dev/null || \
echo "Warning: OpenVINO setupvars.sh not found, using system paths"

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

if [ $? -ne 0 ]; then
    echo "编译失败!"
    exit 1
fi

echo ""
echo "=== 运行 verify_pnp ==="
echo "视频: $VIDEO_PATH"
echo "模型: $MODEL_PATH"
echo "颜色: $COLOR_FLAG (0=蓝, 1=红, 2=全部)"
echo ""

./verify_pnp "$VIDEO_PATH" "$MODEL_PATH" "$COLOR_FLAG"
