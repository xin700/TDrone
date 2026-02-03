#!/bin/bash
# Environment Verification Script for ROS2 Vision Migration
# This script verifies that the Docker environment is properly configured

set -e

echo "=========================================="
echo "ROS2 Vision Environment Verification"
echo "=========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
}

fail() {
    echo -e "${RED}[FAIL]${NC} $1"
    exit 1
}

warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# 1. Check ROS2 environment
echo ""
echo "1. Checking ROS2 environment..."
if [ -z "$ROS_DISTRO" ]; then
    fail "ROS_DISTRO not set. Source ROS2 setup first."
else
    pass "ROS_DISTRO=$ROS_DISTRO"
fi

# Check ros2 command
if command -v ros2 &> /dev/null; then
    pass "ros2 command available"
else
    fail "ros2 command not found"
fi

# 2. Check OpenVINO
echo ""
echo "2. Checking OpenVINO..."
if [ -d "/opt/intel/openvino_2023" ]; then
    pass "OpenVINO 2023 installed"
else
    warn "OpenVINO 2023 not found at /opt/intel/openvino_2023"
fi

# Check OpenVINO environment
if [ -n "$INTEL_OPENVINO_DIR" ] || [ -f "/opt/intel/openvino_2023/setupvars.sh" ]; then
    pass "OpenVINO environment available"
else
    warn "OpenVINO environment not configured"
fi

# 3. Check required libraries
echo ""
echo "3. Checking required libraries..."

# Check Eigen
if pkg-config --exists eigen3 2>/dev/null || [ -d "/usr/include/eigen3" ]; then
    pass "Eigen3 available"
else
    fail "Eigen3 not found"
fi

# Check OpenCV
if pkg-config --exists opencv4 2>/dev/null; then
    OPENCV_VERSION=$(pkg-config --modversion opencv4)
    pass "OpenCV $OPENCV_VERSION available"
else
    warn "OpenCV pkg-config not found, may still be available"
fi

# Check nlohmann-json
if [ -f "/usr/include/nlohmann/json.hpp" ]; then
    pass "nlohmann-json available"
else
    warn "nlohmann-json not found at expected location"
fi

# Check GTest
if [ -d "/usr/include/gtest" ]; then
    pass "GTest available"
else
    fail "GTest not found"
fi

# 4. Check hardware access
echo ""
echo "4. Checking hardware access..."

# Check GPU access
if [ -d "/dev/dri" ]; then
    pass "GPU device (/dev/dri) accessible"
else
    warn "GPU device not accessible"
fi

# Check for video devices
if ls /dev/video* 1> /dev/null 2>&1; then
    VIDEO_DEVICES=$(ls /dev/video* 2>/dev/null | wc -l)
    pass "Video devices found: $VIDEO_DEVICES"
else
    warn "No video devices found"
fi

# Check for serial ports
if ls /dev/ttyUSB* 1> /dev/null 2>&1 || ls /dev/ttyACM* 1> /dev/null 2>&1; then
    pass "Serial ports available"
else
    warn "No serial ports found"
fi

# 5. Check workspace
echo ""
echo "5. Checking workspace..."

if [ -d "~/droneAim/TDrone/src" ]; then
    pass "Workspace source directory exists"
else
    warn "Workspace source directory not found"
fi

if [ -d "~/droneAim/TDrone/videos" ]; then
    VIDEO_COUNT=$(ls ~/droneAim/TDrone/videos/*.mp4 ~/droneAim/TDrone/videos/*.avi 2>/dev/null | wc -l)
    pass "Videos directory exists ($VIDEO_COUNT video files)"
else
    warn "Videos directory not found"
fi

# 6. Test ROS2 node communication
echo ""
echo "6. Testing ROS2 node communication..."

# Start a simple test
timeout 5 ros2 topic list > /dev/null 2>&1 && pass "ROS2 topic list works" || warn "ROS2 topic list failed (may need DDS setup)"

echo ""
echo "=========================================="
echo "Environment verification complete!"
echo "=========================================="
