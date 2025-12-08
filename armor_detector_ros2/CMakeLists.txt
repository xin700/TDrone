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
find_package(geometry_msgs REQUIRED)
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
  "msg/ArmorPose.msg"
  "msg/ArmorPoseArray.msg"
  "msg/Point2f.msg"
  "msg/Command.msg"
  "msg/OutpostState.msg"
  DEPENDENCIES std_msgs geometry_msgs
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

# ==================== 位姿解算节点 ====================
# 功能: 订阅检测结果和 IMU 数据，进行 PnP 位姿解算，发布位姿结果
add_executable(solver_node
  src/nodes/solver_node.cpp
  src/core/pose_solver.cpp
)
target_link_libraries(solver_node
  ${OpenCV_LIBS}
  CURL::libcurl
  TIFF::TIFF
)
ament_target_dependencies(solver_node
  rclcpp
  geometry_msgs
)
# 链接自定义消息
target_link_libraries(solver_node ${cpp_typesupport_target})

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

# ==================== 模拟硬件节点 ====================
# 功能: 模拟相机，从视频文件读取帧并发布到 /camera/image_raw
add_executable(mock_camera_node
  src/mock/mock_camera_node.cpp
)
target_link_libraries(mock_camera_node
  ${OpenCV_LIBS}
  CURL::libcurl
  TIFF::TIFF
)
ament_target_dependencies(mock_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# 功能: 模拟 IMU，发布四元数到 /imu/quaternion
add_executable(mock_imu_node
  src/mock/mock_imu_node.cpp
)
ament_target_dependencies(mock_imu_node
  rclcpp
  geometry_msgs
)

# 功能: 模拟 CBoard，发布弹速，接收控制指令
add_executable(mock_cboard_node
  src/mock/mock_cboard_node.cpp
)
ament_target_dependencies(mock_cboard_node
  rclcpp
  std_msgs
)

# ==================== 前哨站预测节点 ====================
# 功能: 订阅装甲板位姿，使用 EKF 估计前哨站旋转状态，发布预测结果
# Requirements: 7.1, 7.2, 7.3
add_executable(outpost_predictor_node
  src/nodes/outpost_predictor_node.cpp
  src/core/outpost_estimator.cpp
  src/core/extended_kalman_filter.cpp
)
target_include_directories(outpost_predictor_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(outpost_predictor_node
  rclcpp
  geometry_msgs
)
# 链接自定义消息
target_link_libraries(outpost_predictor_node ${cpp_typesupport_target})

# ==================== 安装目标 ====================
# 安装可执行文件
install(TARGETS
  video_publisher_node
  detector_node
  solver_node
  visualizer_node
  mock_camera_node
  mock_imu_node
  mock_cboard_node
  outpost_predictor_node
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

# 安装脚本文件
install(PROGRAMS
  scripts/evaluate_prediction.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装模型文件
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# ==================== CRC16 库 ====================
# 功能: CRC16 校验模块，用于下位机通信
add_library(crc16 STATIC
  src/core/crc16.cpp
)
target_include_directories(crc16 PUBLIC
  include
)

# ==================== PoseSolver 库 ====================
# 功能: PnP 位姿解算模块
add_library(pose_solver STATIC
  src/core/pose_solver.cpp
)
target_include_directories(pose_solver PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(pose_solver
  ${OpenCV_LIBS}
)

# ==================== ExtendedKalmanFilter 库 ====================
# 功能: 扩展卡尔曼滤波模块，用于状态估计
add_library(extended_kalman_filter STATIC
  src/core/extended_kalman_filter.cpp
)
target_include_directories(extended_kalman_filter PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ==================== OutpostEstimator 库 ====================
# 功能: 前哨站状态估计模块，使用 EKF 估计旋转中心和角速度
# Requirements: 7.1, 7.2, 7.3, 7.4
add_library(outpost_estimator STATIC
  src/core/outpost_estimator.cpp
)
target_include_directories(outpost_estimator PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(outpost_estimator
  extended_kalman_filter
)

# ==================== 测试配置 ====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Environment verification test
  ament_add_gtest(test_environment
    test/test_environment.cpp
  )
  target_include_directories(test_environment PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_environment
    ${OpenCV_LIBS}
    openvino::runtime
    CURL::libcurl
    TIFF::TIFF
  )
  ament_target_dependencies(test_environment
    rclcpp
    std_msgs
  )
  
  # CRC16 unit test
  ament_add_gtest(test_crc16
    test/test_crc16.cpp
  )
  target_link_libraries(test_crc16
    crc16
  )
  
  # Mock camera node test
  ament_add_gtest(test_mock_camera
    test/test_mock_camera.cpp
  )
  target_include_directories(test_mock_camera PRIVATE
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(test_mock_camera
    ${OpenCV_LIBS}
    CURL::libcurl
    TIFF::TIFF
  )
  ament_target_dependencies(test_mock_camera
    rclcpp
    sensor_msgs
    cv_bridge
  )
  
  # Mock IMU node test
  ament_add_gtest(test_mock_imu
    test/test_mock_imu.cpp
  )
  ament_target_dependencies(test_mock_imu
    rclcpp
    geometry_msgs
  )
  
  # Mock CBoard node test
  ament_add_gtest(test_mock_cboard
    test/test_mock_cboard.cpp
  )
  ament_target_dependencies(test_mock_cboard
    rclcpp
    std_msgs
  )
  
  # PoseSolver unit test
  ament_add_gtest(test_pose_solver
    test/test_pose_solver.cpp
  )
  target_include_directories(test_pose_solver PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_pose_solver
    pose_solver
    ${OpenCV_LIBS}
  )
  # Define test data directory path
  target_compile_definitions(test_pose_solver PRIVATE
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/test_data"
  )
  
  # Solver node integration test
  ament_add_gtest(test_solver_node
    test/test_solver_node.cpp
  )
  target_include_directories(test_solver_node PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_solver_node
    pose_solver
    ${OpenCV_LIBS}
    ${cpp_typesupport_target}
  )
  ament_target_dependencies(test_solver_node
    rclcpp
    geometry_msgs
  )
  
  # ExtendedKalmanFilter unit test
  # Tests Property 7: EKF 更新协方差减小
  # Tests Property 10: EKF 预测模型一致性
  # Tests Property 24: 卡方检验异常值拒绝
  # Requirements: 7.1
  ament_add_gtest(test_ekf
    test/test_ekf.cpp
  )
  target_include_directories(test_ekf PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_ekf
    extended_kalman_filter
  )
  
  # OutpostEstimator unit test
  # Tests Property 21: 前哨站 EKF 收敛性
  # Tests Property 22: 前哨站输出完整性
  # Tests Property 23: 前哨站方向检测
  # Tests Property 24: 卡方检验异常值拒绝
  # Requirements: 7.1, 7.2, 7.3, 7.4
  ament_add_gtest(test_outpost_estimator
    test/test_outpost_estimator.cpp
  )
  target_include_directories(test_outpost_estimator PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_outpost_estimator
    outpost_estimator
    extended_kalman_filter
  )
  
  # OutpostPredictorNode integration test
  # Tests outpost_predictor_node functionality
  # Requirements: 7.1, 7.2, 7.3
  ament_add_gtest(test_outpost_predictor_node
    test/test_outpost_predictor_node.cpp
  )
  target_include_directories(test_outpost_predictor_node PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_outpost_predictor_node
    ${cpp_typesupport_target}
  )
  ament_target_dependencies(test_outpost_predictor_node
    rclcpp
    geometry_msgs
  )
endif()

ament_package()
