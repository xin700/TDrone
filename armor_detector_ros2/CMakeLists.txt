cmake_minimum_required(VERSION 3.8)
project(armor_detector_ros2)

# 设置C++标准为C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译选项：Release模式优化
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ==================== 查找依赖 ====================
# ROS2核心依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)

# OpenCV依赖 - 只使用需要的组件
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio dnn highgui)

# 解决gdal库的curl/libtiff依赖问题
find_package(CURL REQUIRED)
find_package(TIFF REQUIRED)

# OpenVINO依赖 - 用于神经网络推理
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

# Eigen3依赖 - 用于矩阵运算
find_package(Eigen3 REQUIRED)

# ==================== 生成自定义消息 ====================
# 定义装甲板检测结果的消息类型
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmorBBox.msg"
  "msg/ArmorBBoxArray.msg"
  "msg/Point2f.msg"
  DEPENDENCIES std_msgs
)

# 获取生成的消息类型目标
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ==================== 头文件路径 ====================
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ==================== 视频发布节点 ====================
# 功能: 读取视频文件，逐帧发布到ROS2话题
add_executable(video_publisher_node
  src/nodes/video_publisher_node.cpp
)
target_link_libraries(video_publisher_node
  ${OpenCV_LIBS}
  CURL::libcurl
  TIFF::TIFF
)
ament_target_dependencies(video_publisher_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

# ==================== 检测器节点 ====================
# 功能: 订阅图像话题，进行装甲板关键点检测和数字分类，发布检测结果
add_executable(detector_node
  src/nodes/detector_node.cpp
  src/core/ArmorInferer.cpp
  src/core/NumberClassifier.cpp
)
target_link_libraries(detector_node
  ${OpenCV_LIBS}
  openvino::runtime
  CURL::libcurl
  TIFF::TIFF
)
ament_target_dependencies(detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)
# 链接自定义消息
target_link_libraries(detector_node ${cpp_typesupport_target})

# ==================== 可视化节点 ====================
# 功能: 订阅检测结果和图像，可视化并写入视频文件
add_executable(visualizer_node
  src/visualize/visualizer_node.cpp
)
target_link_libraries(visualizer_node
  ${OpenCV_LIBS}
  CURL::libcurl
  TIFF::TIFF
)
ament_target_dependencies(visualizer_node
  rclcpp
  sensor_msgs
  cv_bridge
)
# 链接自定义消息
target_link_libraries(visualizer_node ${cpp_typesupport_target})

# ==================== 组件库（零拷贝版本）====================
# 将所有组件编译为一个共享库，支持进程内零拷贝通信

# 视频发布组件
add_library(video_publisher_component SHARED
  src/components/video_publisher_component.cpp
)
target_link_libraries(video_publisher_component
  ${OpenCV_LIBS}
  CURL::libcurl
  TIFF::TIFF
)
ament_target_dependencies(video_publisher_component
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
)
rclcpp_components_register_nodes(video_publisher_component "armor_detector::VideoPublisherComponent")

# 检测器组件
add_library(detector_component SHARED
  src/components/detector_component.cpp
  src/core/ArmorInferer.cpp
  src/core/NumberClassifier.cpp
)
target_link_libraries(detector_component
  ${OpenCV_LIBS}
  openvino::runtime
  CURL::libcurl
  TIFF::TIFF
  ${cpp_typesupport_target}
)
ament_target_dependencies(detector_component
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
)
rclcpp_components_register_nodes(detector_component "armor_detector::DetectorComponent")

# 可视化组件
add_library(visualizer_component SHARED
  src/components/visualizer_component.cpp
)
target_link_libraries(visualizer_component
  ${OpenCV_LIBS}
  CURL::libcurl
  TIFF::TIFF
  ${cpp_typesupport_target}
)
ament_target_dependencies(visualizer_component
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
)
rclcpp_components_register_nodes(visualizer_component "armor_detector::VisualizerComponent")

# ==================== 安装目标 ====================
# 安装可执行文件
install(TARGETS
  video_publisher_node
  detector_node
  visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装组件库
install(TARGETS
  video_publisher_component
  detector_component
  visualizer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装启动文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装模型文件
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
