# ROS2 Vision Migration - Docker Development Environment

## Overview

This Docker environment provides a consistent development setup for the ROS2 Vision Migration project, including:

- ROS2 Humble (perception variant)
- OpenVINO 2024 for neural network inference
- OpenCV 4.x for image processing
- Eigen3, Sophus, Ceres for mathematical operations
- GTest and RapidCheck for testing

**Note**: This environment is designed to be headless (no GUI). Test results and visualizations are saved to video files in the `output/` directory.

## Directory Structure

```
ws/
├── armor_detector_ros2/    # ROS2 package source code
├── models/                 # OpenVINO models (symlink to armor_detector_ros2/models)
│   ├── BRpoints_nano.xml   # Armor detection model
│   └── classifier.xml      # Number classifier model
├── videos/                 # Input test videos
│   ├── sample.avi          # Sample armor video
│   └── outpost_sample.mp4  # Sample outpost video
├── output/                 # Output videos and results
├── docker-compose.yml      # Docker configuration
├── Dockerfile              # Docker image definition
└── run_detector.sh         # Interactive run script
```

## Quick Start

### 1. Prepare Models and Videos

Before running, ensure you have the required models and videos:

```bash
# Models should be in ws/models/ (or ws/armor_detector_ros2/models/)
ls ws/models/
# Should show: BRpoints_nano.xml, BRpoints_nano.bin, classifier.xml, classifier.bin

# Videos should be in ws/videos/
ls ws/videos/
# Place your test videos here
```

### 2. Build the Docker Image

```bash
cd ws
docker-compose build
```

### 3. Start the Development Container

```bash
# For development (headless, recommended)
docker-compose up -d ros2-vision

# For hardware testing (with full device access)
docker-compose up -d ros2-vision-hardware

# For GUI applications (optional, requires X11 setup)
xhost +local:docker
docker-compose up -d ros2-vision-gui
```

### 4. Enter the Container

```bash
docker-compose exec ros2-vision bash
```

### 5. Build the Project

Inside the container:
```bash
source /opt/ros/humble/setup.bash
cd /ros2_ws
colcon build --packages-select armor_detector_ros2
source install/setup.bash
```

### 5. Run Environment Verification

```bash
# Run the verification script
bash /ros2_ws/src/armor_detector_ros2/../scripts/verify_environment.sh

# Or run the GTest-based verification
colcon build --packages-select armor_detector_ros2 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select armor_detector_ros2 --ctest-args -R test_environment
colcon test-result --verbose
```

## Services

| Service | Purpose | GUI | Hardware Access |
|---------|---------|-----|-----------------|
| `ros2-vision` | Standard development | No | GPU only |
| `ros2-vision-gui` | Visualization/debugging | Yes | GPU only |
| `ros2-vision-hardware` | Hardware testing | No | Full `/dev` |

### ros2-vision (Default)
- Headless development environment
- Source code mounted for hot reload
- GPU access for OpenVINO
- Output saved to `./output/` directory

### ros2-vision-gui (Optional)
- X11 forwarding for GUI applications
- Use only when visualization is needed
- Requires `xhost +local:docker` on host

### ros2-vision-hardware
- Full device access (`/dev` mounted)
- For testing with real cameras and serial ports
- Use when connecting to actual hardware

## Volume Mounts

| Host Path | Container Path | Purpose |
|-----------|----------------|---------|
| `./` | `/ros2_ws` | Entire workspace |
| `./models` | `/ros2_ws/models` | OpenVINO models |
| `./videos` | `/ros2_ws/videos` | Input test videos |
| `./output` | `/ros2_ws/output` | Output videos and results |

## Running Pipelines

### Full Pipeline (Detection + Pose Solving + Visualization)

```bash
# Inside container
ros2 launch armor_detector_ros2 full_pipeline_launch.py \
    video_path:=/ros2_ws/videos/sample.avi \
    output_path:=/ros2_ws/output/result.mp4
```

### Outpost Prediction Pipeline

```bash
# Inside container
ros2 launch armor_detector_ros2 outpost_pipeline_launch.py \
    video_path:=/ros2_ws/videos/outpost_sample.mp4 \
    output_path:=/ros2_ws/output/outpost_result.mp4
```

### Using Custom Paths

All launch files accept custom paths via launch arguments:

```bash
ros2 launch armor_detector_ros2 outpost_pipeline_launch.py \
    video_path:=/ros2_ws/videos/your_video.mp4 \
    output_path:=/ros2_ws/output/your_output.mp4 \
    fps:=-1.0  # Use original video frame rate
```

## Headless Testing

All tests are designed to work without GUI. Results are saved to files:

```bash
# Run full pipeline and save output to video
ros2 launch armor_detector_ros2 full_pipeline_launch.py

# View results on host machine
vlc ./output/full_pipeline_result.mp4
```

## Troubleshooting

### OpenVINO not found
```bash
source /opt/intel/openvino_2023/setupvars.sh
```

### Permission denied for devices
Use the `ros2-vision-hardware` service or add your user to the appropriate groups:
```bash
sudo usermod -aG dialout $USER  # For serial ports
sudo usermod -aG video $USER    # For cameras
```

### Build cache issues
```bash
# Clear build cache
docker-compose down -v
docker-compose up -d ros2-vision
```

### Output directory not writable
```bash
# On host, ensure output directory exists with correct permissions
mkdir -p ./output
chmod 777 ./output
```

## Requirements

- Docker 20.10+
- Docker Compose 2.0+
- Intel GPU (optional, for GPU acceleration)
- X11 server (only for `ros2-vision-gui` service)
