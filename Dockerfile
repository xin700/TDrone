# ROS2 Vision Migration Docker Environment
# Based on ros:humble-perception for ROS2 Humble with OpenCV and cv_bridge
FROM ros:humble-perception

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Configure Chinese mirror sources for faster downloads
RUN sed -i 's|http://archive.ubuntu.com|https://mirrors.aliyun.com|g' /etc/apt/sources.list && \
    sed -i 's|http://security.ubuntu.com|https://mirrors.aliyun.com|g' /etc/apt/sources.list

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Build tools
    build-essential \
    cmake \
    git \
    wget \
    curl \
    gnupg \
    ca-certificates \
    # Development tools
    gdb \
    vim \
    # Math libraries
    libeigen3-dev \
    # Ceres Solver dependencies
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    # JSON library
    nlohmann-json3-dev \
    # Testing frameworks
    libgtest-dev \
    libgmock-dev \
    # YAML parsing
    libyaml-cpp-dev \
    # Format library
    libfmt-dev \
    # Serial communication
    libserial-dev \
    # Video/Image dependencies
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    # X11 for GUI
    libx11-dev \
    x11-apps \
    # Hardware access
    udev \
    usbutils \
    # Python tools
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install OpenVINO 2024 from APT (more reliable than 2023)
RUN wget -qO - https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | gpg --dearmor -o /usr/share/keyrings/intel-openvino-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/intel-openvino-archive-keyring.gpg] https://apt.repos.intel.com/openvino/2024 ubuntu22 main" | tee /etc/apt/sources.list.d/intel-openvino-2024.list && \
    apt-get update && \
    apt-get install -y openvino && \
    rm -rf /var/lib/apt/lists/*

# Set OpenVINO environment variables
ENV OpenVINO_DIR=/opt/intel/openvino_2024
ENV LD_LIBRARY_PATH=/opt/intel/openvino_2024/runtime/lib/intel64:$LD_LIBRARY_PATH

# Install RapidCheck from source (not available in Ubuntu 22.04 repos)
RUN cd /tmp && \
    git clone https://github.com/emil-e/rapidcheck.git && \
    cd rapidcheck && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DRC_ENABLE_GTEST=ON && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    rm -rf /tmp/rapidcheck

# Install Sophus (Lie group library)
RUN cd /tmp && \
    git clone https://github.com/strasdat/Sophus.git && \
    cd Sophus && \
    git checkout 1.22.10 && \
    mkdir build && cd build && \
    cmake .. -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/Sophus

# Install Ceres Solver
RUN cd /tmp && \
    git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && \
    git checkout 2.1.0 && \
    mkdir build && cd build && \
    cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/ceres-solver

# Setup udev rules for hardware access (cameras, serial ports)
RUN echo 'SUBSYSTEM=="usb", MODE="0666"' > /etc/udev/rules.d/99-usb.rules && \
    echo 'KERNEL=="ttyUSB*", MODE="0666"' >> /etc/udev/rules.d/99-usb.rules && \
    echo 'KERNEL=="ttyACM*", MODE="0666"' >> /etc/udev/rules.d/99-usb.rules

# Create workspace directory
WORKDIR /ros2_ws

# Setup entrypoint
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

# Source ROS2 and OpenVINO setup in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true" >> /root/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /root/.bashrc

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
