/**
 * @file real_hardware_publisher_node.cpp
 * @brief 真实硬件数据发布 ROS2 节点 - 使用大恒相机SDK
 * 
 * 功能：
 * - 实时采集大恒相机视频流，发布到 /camera/image_raw 话题
 * - 实时采集串口 IMU 数据，发布到 /imu/quaternion 话题
 * - 保证相机帧与 IMU 数据的时间同步
 * - 发布弹速到 /bullet_speed 话题
 * - 发布模式请求到 /aim_request 话题
 * 
 * 发布话题：
 * - /camera/image_raw (sensor_msgs/Image): 相机原始图像
 * - /imu/quaternion (geometry_msgs/QuaternionStamped): IMU 姿态四元数
 * - /bullet_speed (std_msgs/Float32): 实际弹速
 * - /aim_request (std_msgs/Bool): 瞄准请求标志
 * 
 * 服务：
 * - /pipeline_control (std_srvs/SetBool): 控制检测流水线启停
 * 
 * 参数：
 * - camera_sn: 相机序列号（默认 "FGK25050153"）
 * - serial_device: 串口设备路径（默认 "/dev/ttyACM0"）
 * - serial_baud: 串口波特率（默认 115200）
 * - exposure_time: 曝光时间 μs（默认 3000）
 * - img_width: 图像宽度（默认 1280）
 * - img_height: 图像高度（默认 1024）
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>

// Boost ASIO for serial port
#include <boost/asio.hpp>

// 大恒相机驱动
#include "driver/DaHengCamera.h"

using namespace std::chrono_literals;

// ==================== 串口数据结构定义 ====================
namespace SerialProtocol {
    struct IMU_Flag {
        int8_t start_flag;              // 起始标志，固定为'!'
        float pitch_now;                // 当前pitch角度（度）
        float yaw_now;                  // 当前yaw角度（度）
        float roll_now;                 // 当前roll角度（度）
        float actual_bullet_speed;      // 实际弹速（m/s）
        uint8_t aim_request;            // 瞄准请求标志（0/1）
        uint8_t mode_want;              // 模式请求
        uint8_t enemy_color;            // 敌方颜色（0:红 1:蓝）
        uint16_t crc16;                 // CRC16校验码
    } __attribute__((packed));
}

// ==================== CRC16校验实现 ====================
namespace CRC16_UTILS {
    const uint16_t CRC_INIT = 0xffff;
    const uint16_t wCRC_Table[256] = {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
    };
    
    uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC) {
        uint8_t chData;
        if (pchMessage == nullptr) {
            return 0xFFFF;
        }
        while (dwLength--) {
            chData = *pchMessage++;
            wCRC = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
        }
        return wCRC;
    }
    
    bool Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) {
        uint16_t wExpected = 0;
        if ((pchMessage == nullptr) || (dwLength <= 2)) {
            return false;
        }
        wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
        return ((wExpected & 0xff) == pchMessage[dwLength - 2] && 
                ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
    }
}

// ==================== 欧拉角转四元数 ====================
void eulerToQuaternion(float roll, float pitch, float yaw, 
                       double& qw, double& qx, double& qy, double& qz) {
    // 将角度转换为弧度
    double roll_rad = roll * M_PI / 180.0;
    double pitch_rad = pitch * M_PI / 180.0;
    double yaw_rad = yaw * M_PI / 180.0;
    
    // 计算四元数
    double cy = std::cos(yaw_rad * 0.5);
    double sy = std::sin(yaw_rad * 0.5);
    double cp = std::cos(pitch_rad * 0.5);
    double sp = std::sin(pitch_rad * 0.5);
    double cr = std::cos(roll_rad * 0.5);
    double sr = std::sin(roll_rad * 0.5);
    
    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
}

/**
 * @class RealHardwarePublisherNode
 * @brief 真实硬件数据发布节点 - 使用大恒相机SDK
 * 
 * 集成大恒相机和串口IMU的数据采集与ROS2发布
 */
class RealHardwarePublisherNode : public rclcpp::Node
{
public:
    RealHardwarePublisherNode() : Node("real_hardware_publisher_node"),
                                   io_context_(),
                                   serial_port_(io_context_)
    {
        // ==================== 参数声明 ====================
        // 相机参数
        this->declare_parameter<std::string>("camera_sn", "FGK25050153");
        this->declare_parameter<int>("exposure_time", 3000);
        this->declare_parameter<int>("img_width", 1280);
        this->declare_parameter<int>("img_height", 1024);
        this->declare_parameter<double>("gain", 16.0);
        
        // 串口参数
        this->declare_parameter<std::string>("serial_device", "/dev/ttyACM0");
        this->declare_parameter<int>("serial_baud", 115200);
        
        // 白平衡参数
        this->declare_parameter<double>("balance_ratio_red", 1.71);
        this->declare_parameter<double>("balance_ratio_blue", 2.17);
        
        // 获取参数
        camera_sn_ = this->get_parameter("camera_sn").as_string();
        exposure_time_ = this->get_parameter("exposure_time").as_int();
        img_width_ = this->get_parameter("img_width").as_int();
        img_height_ = this->get_parameter("img_height").as_int();
        gain_ = this->get_parameter("gain").as_double();
        
        serial_device_ = this->get_parameter("serial_device").as_string();
        serial_baud_ = this->get_parameter("serial_baud").as_int();
        
        balance_ratio_red_ = this->get_parameter("balance_ratio_red").as_double();
        balance_ratio_blue_ = this->get_parameter("balance_ratio_blue").as_double();
        
        RCLCPP_INFO(this->get_logger(), "===== 真实硬件数据发布节点配置 =====");
        RCLCPP_INFO(this->get_logger(), "相机序列号: %s", camera_sn_.c_str());
        RCLCPP_INFO(this->get_logger(), "曝光时间: %d μs", exposure_time_);
        RCLCPP_INFO(this->get_logger(), "图像尺寸: %dx%d", img_width_, img_height_);
        RCLCPP_INFO(this->get_logger(), "增益: %.1f dB", gain_);
        RCLCPP_INFO(this->get_logger(), "串口设备: %s", serial_device_.c_str());
        RCLCPP_INFO(this->get_logger(), "串口波特率: %d", serial_baud_);
        
        // ==================== 创建发布者 ====================
        // 相机图像发布者 - 与mock_camera_node使用相同话题
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/video/image_raw", 
            rclcpp::QoS(10).reliable()
        );
        
        // IMU四元数发布者 - 与mock_imu_node使用相同话题
        imu_pub_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
            "/imu/quaternion", 
            rclcpp::QoS(10).reliable()
        );
        
        // 弹速发布者
        bullet_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/bullet_speed",
            rclcpp::QoS(10).reliable()
        );
        
        // 瞄准请求发布者
        aim_request_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/aim_request",
            rclcpp::QoS(10).reliable()
        );
        
        // ==================== 创建服务 ====================
        // 流水线控制服务
        pipeline_control_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "/pipeline_control",
            std::bind(&RealHardwarePublisherNode::pipeline_control_callback, this,
                      std::placeholders::_1, std::placeholders::_2)
        );
        
        // ==================== 初始化硬件 ====================
        // 初始化串口
        if (!initSerial()) {
            RCLCPP_ERROR(this->get_logger(), "串口初始化失败！");
            // 串口失败不退出，允许只使用相机
        } else {
            RCLCPP_INFO(this->get_logger(), "串口初始化成功");
            serial_initialized_ = true;
        }
        
        // 初始化大恒相机
        if (!initDaHengCamera()) {
            RCLCPP_ERROR(this->get_logger(), "大恒相机初始化失败！");
            rclcpp::shutdown();
            return;
        }
        camera_initialized_ = true;
        
        // ==================== 启动工作线程 ====================
        // 启动串口读取线程
        if (serial_initialized_) {
            serial_thread_ = std::thread(&RealHardwarePublisherNode::serialReadThread, this);
        }
        
        // 启动相机采集线程
        camera_thread_ = std::thread(&RealHardwarePublisherNode::cameraThread, this);
        
        RCLCPP_INFO(this->get_logger(), "真实硬件数据发布节点已启动");
        RCLCPP_INFO(this->get_logger(), "发布话题: /camera/image_raw, /imu/quaternion, /bullet_speed, /aim_request");
        RCLCPP_INFO(this->get_logger(), "服务: /pipeline_control (控制检测流水线启停)");
    }
    
    ~RealHardwarePublisherNode()
    {
        // 停止线程
        should_stop_ = true;
        
        // 等待线程结束
        if (serial_thread_.joinable()) {
            serial_thread_.join();
        }
        if (camera_thread_.joinable()) {
            camera_thread_.join();
        }
        
        // 关闭串口
        if (serial_port_.is_open()) {
            serial_port_.close();
        }
        
        // 相机会在析构时自动释放
        camera_.reset();
        
        RCLCPP_INFO(this->get_logger(), "真实硬件数据发布节点已关闭");
    }

private:
    // ==================== 串口初始化 ====================
    bool initSerial()
    {
        try {
            serial_port_.open(serial_device_);
            
            serial_port_.set_option(boost::asio::serial_port_base::baud_rate(serial_baud_));
            serial_port_.set_option(boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none));
            serial_port_.set_option(boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none));
            serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));
            serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "打开串口失败: %s", e.what());
            return false;
        }
    }
    
    // ==================== 大恒相机初始化 ====================
    bool initDaHengCamera()
    {
        RCLCPP_INFO(this->get_logger(), "正在初始化大恒相机...");
        
        try {
            // 创建相机对象
            camera_ = std::make_unique<DRIVER::DaHengCamera>();
            
            // 初始化相机库
            GX_STATUS status = camera_->initLib();
            if (status != GX_STATUS_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "初始化相机库失败, 错误码: %d", status);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "相机库初始化成功");
            
            // 打开相机设备
            status = camera_->openDevice(camera_sn_.c_str());
            if (status != GX_STATUS_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "打开相机设备失败, SN: %s, 错误码: %d", 
                            camera_sn_.c_str(), status);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "相机设备打开成功, SN: %s", camera_sn_.c_str());
            
            // 设置ROI参数
            camera_->setRoiParam(img_width_, img_height_, 0, 0);
            RCLCPP_INFO(this->get_logger(), "设置ROI: %dx%d", img_width_, img_height_);
            
            // 设置曝光和增益参数
            // 参数: AutoExposure, AutoGain, ExposureTime, AutoExposureTimeMin, AutoExposureTimeMax,
            //       Gain, AutoGainMin, AutoGainMax, GrayValueMin, GrayValueMax, TRIGGER_SOURCE_LINE2
            camera_->setExposureGainParam(
                false,           // AutoExposure - 不使用自动曝光
                false,           // AutoGain - 不使用自动增益
                exposure_time_,  // ExposureTime - 曝光时间
                500,             // AutoExposureTimeMin
                8000,            // AutoExposureTimeMax
                gain_,           // Gain - 增益
                5,               // AutoGainMin
                13,              // AutoGainMax
                28,              // GrayValueMin
                28,              // GrayValueMax
                false            // TRIGGER_SOURCE_LINE2 - 使用软件触发
            );
            RCLCPP_INFO(this->get_logger(), "设置曝光时间: %d μs, 增益: %.1f dB", 
                        exposure_time_, gain_);
            
            // 设置白平衡参数
            camera_->setWhiteBalanceParam(
                true,                           // WhiteBalanceOn
                balance_ratio_red_,             // BalanceRatioRed
                balance_ratio_blue_,            // BalanceRatioBlue
                1.0,                            // BalanceRatioGreen
                GX_AWB_LAMP_HOUSE_ADAPTIVE      // LightSource
            );
            RCLCPP_INFO(this->get_logger(), "设置白平衡: R=%.2f, B=%.2f", 
                        balance_ratio_red_, balance_ratio_blue_);
            
            // 设置自动曝光区域
            camera_->setAAROIParam(img_width_ / 2, img_height_ / 2, 320, 256);
            
            // 开始采集
            status = camera_->acquisitionStart();
            if (status != GX_STATUS_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "启动相机采集失败, 错误码: %d", status);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "相机采集已启动");
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "初始化相机异常: %s", e.what());
            return false;
        }
    }
    
    // ==================== 串口读取线程 ====================
    void serialReadThread()
    {
        uint8_t read_data_length = sizeof(SerialProtocol::IMU_Flag);
        std::unique_ptr<uint8_t[]> buffer(new uint8_t[read_data_length]);
        
        RCLCPP_INFO(this->get_logger(), "串口读取线程启动");
        
        while (!should_stop_) {
            try {
                // 同步读取数据
                boost::system::error_code ec;
                size_t len = boost::asio::read(serial_port_, 
                                              boost::asio::buffer(buffer.get(), read_data_length),
                                              ec);
                
                if (ec) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                        "读取串口数据失败: %s", ec.message().c_str());
                    std::this_thread::sleep_for(10ms);
                    continue;
                }
                
                // 检查起始标志
                if (buffer[0] != '!') {
                    // 寻找下一个起始标志
                    continue;
                }
                
                // 验证CRC16
                SerialProtocol::IMU_Flag* imu_data = 
                    reinterpret_cast<SerialProtocol::IMU_Flag*>(buffer.get());
                
                if (!CRC16_UTILS::Verify_CRC16_Check_Sum(buffer.get(), read_data_length)) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                        "CRC16校验失败");
                    continue;
                }
                
                // 更新最新IMU数据
                {
                    std::lock_guard<std::mutex> lock(imu_mutex_);
                    latest_imu_ = *imu_data;
                    imu_updated_ = true;
                }
                
                // 发布IMU数据（保持高频率发布，供solver_node缓存匹配使用）
                publishIMUData(imu_data);
                
                imu_received_count_++;
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "串口读取异常: %s", e.what());
                std::this_thread::sleep_for(100ms);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "串口读取线程结束");
    }
    
    // ==================== 相机采集线程 ====================
    void cameraThread()
    {
        RCLCPP_INFO(this->get_logger(), "相机采集线程启动");
        
        cv::Mat frame;
        std::chrono::steady_clock::time_point time_stamp;
        
        // 帧率控制 - 目标60fps
        const int target_fps = 60;
        const auto frame_duration = std::chrono::microseconds(1000000 / target_fps);
        auto last_frame_time = std::chrono::steady_clock::now();
        
        while (!should_stop_) {
            // 使用大恒相机SDK获取图像
            camera_->ProcGetImage(&frame, &time_stamp);
            
            if (frame.empty()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "获取图像失败");
                std::this_thread::sleep_for(1ms);
                continue;
            }
            
            // 帧率控制 - 等待到下一帧时间
            auto now = std::chrono::steady_clock::now();
            auto elapsed = now - last_frame_time;
            if (elapsed < frame_duration) {
                std::this_thread::sleep_for(frame_duration - elapsed);
            }
            last_frame_time = std::chrono::steady_clock::now();
            
            // 发布图像（使用相机SDK提供的采集时间戳）
            publishImage(frame, time_stamp);
            
            camera_frame_count_++;
            if (camera_frame_count_ % 100 == 0) {
                RCLCPP_INFO(this->get_logger(), "已发布 %d 帧图像", camera_frame_count_);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "相机采集线程结束");
    }
    
    // ==================== 发布图像 ====================
    void publishImage(const cv::Mat& frame, const std::chrono::steady_clock::time_point& capture_time)
    {
        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",
            frame
        ).toImageMsg();
        
        // 直接使用当前时刻作为时间戳
        // 注意：不要使用 capture_time 往前推算，因为 IMU 时间戳也是用 now()
        // 两者必须使用相同的时间基准才能正确匹配
        auto now = this->get_clock()->now();
        msg->header.stamp = now;
        msg->header.frame_id = "camera_frame";
        
        image_pub_->publish(*msg);
    }
    
    // ==================== 发布IMU数据 ====================
    void publishIMUData(const SerialProtocol::IMU_Flag* imu_data)
    {
        // 发布四元数
        auto quat_msg = geometry_msgs::msg::QuaternionStamped();
        quat_msg.header.stamp = this->get_clock()->now();
        quat_msg.header.frame_id = "imu_frame";
        
        // 将欧拉角转换为四元数
        double qw, qx, qy, qz;
        eulerToQuaternion(imu_data->roll_now, imu_data->pitch_now, imu_data->yaw_now,
                          qw, qx, qy, qz);
        
        quat_msg.quaternion.w = qw;
        quat_msg.quaternion.x = qx;
        quat_msg.quaternion.y = qy;
        quat_msg.quaternion.z = qz;
        
        imu_pub_->publish(quat_msg);
        
        // 发布弹速
        auto bullet_msg = std_msgs::msg::Float32();
        bullet_msg.data = imu_data->actual_bullet_speed;
        bullet_speed_pub_->publish(bullet_msg);
        
        // 发布瞄准请求
        auto aim_msg = std_msgs::msg::Bool();
        aim_msg.data = (imu_data->aim_request != 0);
        aim_request_pub_->publish(aim_msg);
    }
    
    // ==================== 流水线控制服务回调 ====================
    void pipeline_control_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        pipeline_enabled_ = request->data;
        
        if (pipeline_enabled_) {
            RCLCPP_INFO(this->get_logger(), "检测流水线已启动");
            response->message = "Pipeline started";
        } else {
            RCLCPP_INFO(this->get_logger(), "检测流水线已停止");
            response->message = "Pipeline stopped";
        }
        
        response->success = true;
    }

    // ==================== 成员变量 ====================
    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bullet_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr aim_request_pub_;
    
    // 服务
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pipeline_control_srv_;
    
    // 大恒相机
    std::unique_ptr<DRIVER::DaHengCamera> camera_;
    std::string camera_sn_;
    int exposure_time_;
    int img_width_;
    int img_height_;
    double gain_;
    double balance_ratio_red_;
    double balance_ratio_blue_;
    
    // 串口相关
    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_port_;
    std::string serial_device_;
    int serial_baud_;
    
    // IMU数据缓存
    SerialProtocol::IMU_Flag latest_imu_;
    std::mutex imu_mutex_;
    bool imu_updated_ = false;
    
    // 线程控制
    std::thread serial_thread_;
    std::thread camera_thread_;
    std::atomic<bool> should_stop_{false};
    
    // 初始化状态
    bool serial_initialized_ = false;
    bool camera_initialized_ = false;
    
    // 流水线控制
    std::atomic<bool> pipeline_enabled_{true};
    
    // 统计
    int imu_received_count_ = 0;
    int camera_frame_count_ = 0;
};

/**
 * @brief 主函数
 */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealHardwarePublisherNode>());
    rclcpp::shutdown();
    return 0;
}
