cmake_minimum_required(VERSION 3.10)
project(verify_pnp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# OpenVINO
find_package(OpenVINO REQUIRED)

# 编译可执行文件
add_executable(verify_pnp verify_pnp.cpp)

target_link_libraries(verify_pnp
    ${OpenCV_LIBS}
    openvino::runtime
)

# 打印配置信息
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenVINO found")
