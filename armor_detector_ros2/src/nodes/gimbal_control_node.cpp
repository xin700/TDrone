/**
 * @file gimbal_control_node.cpp
 * @brief 云台控制节点
 * 
 * 功能：
 *   - 订阅瞄准节点发布的云台指令 (/aiming/gimbal_command)
 *   - 通过串口发送指令到云台控制板
 *   - 支持pipeline_control服务控制
 * 
 * 通信协议：
 *   - 发送数据结构：SerialWriteData (包含云台角度设定和射击标志)
 *   - 使用CRC16校验确保数据完整性
 */

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstring>

#include "armor_detector_ros2/msg/gimbal_command.hpp"

namespace armor_detector
{

// ============================================================================
// 串口数据结构（与下位机协议保持一致）
// ============================================================================

namespace SerialData {
    struct SerialWriteData {
        int8_t start_flag;            // 起始标志，固定为'!'
        uint8_t shoot_flag;           // 射击标志（0:不射击 1:射击）
        float pitch_setpoint;         // pitch角度设定值（度）
        float yaw_setpoint;           // yaw角度设定值（度）
        uint16_t crc16;               // CRC16校验码
    } __attribute__((packed));
}

// ============================================================================
// CRC16校验工具
// ============================================================================

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
    
    void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) {
        uint16_t wCRC = 0;
        if ((pchMessage == nullptr) || (dwLength <= 2)) {
            return;
        }
        wCRC = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
        pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
        pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
    }
    
    void addCRC16(SerialData::SerialWriteData* data) {
        if (!data) return;
        Append_CRC16_Check_Sum((uint8_t*)data, sizeof(SerialData::SerialWriteData));
    }
}

// ============================================================================
// 云台控制节点
// ============================================================================

class GimbalControlNode : public rclcpp::Node
{
public:
    GimbalControlNode() : Node("gimbal_control_node")
    {
        RCLCPP_INFO(this->get_logger(), "初始化云台控制节点...");
        
        // 声明参数
        this->declare_parameter("serial_device", "/dev/ttyACM0");
        this->declare_parameter("serial_baud", 115200);
        this->declare_parameter("send_rate", 100.0);  // 发送频率 (Hz)
        
        // 加载参数
        serial_device_ = this->get_parameter("serial_device").as_string();
        serial_baud_ = this->get_parameter("serial_baud").as_int();
        send_rate_ = this->get_parameter("send_rate").as_double();
        
        // 初始化串口
        if (!initSerial()) {
            RCLCPP_ERROR(this->get_logger(), "串口初始化失败！节点将以仿真模式运行");
            simulation_mode_ = true;
        }
        
        // 创建订阅者
        gimbal_cmd_sub_ = this->create_subscription<armor_detector_ros2::msg::GimbalCommand>(
            "/aiming/gimbal_command", 10,
            std::bind(&GimbalControlNode::gimbalCommandCallback, this, std::placeholders::_1));
        
        // 创建pipeline_control服务
        pipeline_control_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "/gimbal_control/pipeline_control",
            std::bind(&GimbalControlNode::pipelineControlCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        // 创建定时发送器
        double send_period_ms = 1000.0 / send_rate_;
        send_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(send_period_ms)),
            std::bind(&GimbalControlNode::sendTimerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "云台控制节点初始化完成");
        RCLCPP_INFO(this->get_logger(), "======================================");
        RCLCPP_INFO(this->get_logger(), "  串口设备: %s", serial_device_.c_str());
        RCLCPP_INFO(this->get_logger(), "  波特率: %d", serial_baud_);
        RCLCPP_INFO(this->get_logger(), "  发送频率: %.1f Hz", send_rate_);
        RCLCPP_INFO(this->get_logger(), "  模式: %s", simulation_mode_ ? "仿真" : "真实");
        RCLCPP_INFO(this->get_logger(), "  订阅: /aiming/gimbal_command");
        RCLCPP_INFO(this->get_logger(), "  服务: /gimbal_control/pipeline_control");
        RCLCPP_INFO(this->get_logger(), "======================================");
    }
    
    ~GimbalControlNode()
    {
        if (serial_port_ && serial_port_->is_open()) {
            serial_port_->close();
        }
    }

private:
    bool initSerial()
    {
        try {
            io_context_ = std::make_unique<boost::asio::io_context>();
            serial_port_ = std::make_unique<boost::asio::serial_port>(*io_context_);
            
            serial_port_->open(serial_device_);
            
            serial_port_->set_option(boost::asio::serial_port_base::baud_rate(serial_baud_));
            serial_port_->set_option(boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none));
            serial_port_->set_option(boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none));
            serial_port_->set_option(boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));
            serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
            
            RCLCPP_INFO(this->get_logger(), "串口打开成功: %s @ %d baud", 
                       serial_device_.c_str(), serial_baud_);
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "串口打开失败: %s", e.what());
            return false;
        }
    }
    
    void pipelineControlCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        pipeline_enabled_ = request->data;
        
        response->success = true;
        response->message = pipeline_enabled_ ? "Gimbal control enabled" : "Gimbal control disabled";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }
    
    void gimbalCommandCallback(const armor_detector_ros2::msg::GimbalCommand::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        
        // 检查目标位置是否发生变化（阈值：0.1度）
        const double position_threshold = 0.1;
        bool position_changed = (std::abs(msg->yaw - latest_yaw_) > position_threshold) ||
                               (std::abs(msg->pitch - latest_pitch_) > position_threshold);
        bool fire_changed = (msg->fire_allowed != latest_fire_allowed_);
        
        // 只有当位置或射击状态变化时才标记为有新指令
        if (position_changed || fire_changed) {
            latest_yaw_ = msg->yaw;
            latest_pitch_ = msg->pitch;
            latest_fire_allowed_ = msg->fire_allowed;
            cmd_received_ = true;
            last_cmd_time_ = this->now();
            
            RCLCPP_DEBUG(this->get_logger(), 
                "收到新指令: yaw=%.2f° pitch=%.2f° shoot=%d",
                latest_yaw_, latest_pitch_, latest_fire_allowed_);
        }
    }
    
    void sendTimerCallback()
    {
        if (!pipeline_enabled_) {
            return;
        }
        
        // 检查是否有新指令需要发送
        double yaw, pitch;
        bool shoot;
        bool has_new_cmd = false;
        
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            
            // 只有收到新指令才发送
            if (!cmd_received_) {
                return;
            }
            
            // 检查指令是否超时（100ms）
            auto elapsed = (this->now() - last_cmd_time_).seconds();
            if (elapsed > 0.1) {
                cmd_received_ = false;  // 清除超时指令
                return;
            }
            
            yaw = latest_yaw_;
            pitch = latest_pitch_;
            shoot = latest_fire_allowed_;
            has_new_cmd = true;
            
            // 发送后立即清除标志，避免重复发送
            cmd_received_ = false;
        }
        
        if (has_new_cmd) {
            // 发送云台指令（只发送一次）
            sendGimbalCommand(static_cast<float>(pitch), static_cast<float>(yaw), shoot);
        }
    }
    
    void sendGimbalCommand(float pitch, float yaw, bool shoot)
    {
        SerialData::SerialWriteData write_data;
        write_data.start_flag = '!';
        write_data.pitch_setpoint = pitch;
        write_data.yaw_setpoint = yaw;
        write_data.shoot_flag = shoot ? 1 : 0;
        write_data.crc16 = 0;
        
        // 添加CRC16校验
        CRC16_UTILS::addCRC16(&write_data);
        
        if (simulation_mode_) {
            // 仿真模式：打印每次发送的日志（因为现在只在变化时才发送）
            RCLCPP_INFO(this->get_logger(), 
                "[SIM] 发送新云台指令: yaw=%.2f° pitch=%.2f° shoot=%d",
                yaw, pitch, shoot ? 1 : 0);
            sent_count_++;
            return;
        }
        
        // 真实模式：发送到串口
        try {
            boost::asio::write(*serial_port_, 
                              boost::asio::buffer(&write_data, sizeof(SerialData::SerialWriteData)));
            sent_count_++;
            
            // 每次发送都打印日志（因为现在只在目标变化时发送）
            RCLCPP_INFO(this->get_logger(), 
                "→ 发送云台指令 #%d: yaw=%.2f° pitch=%.2f° shoot=%d",
                sent_count_, yaw, pitch, shoot ? 1 : 0);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "发送串口数据失败: %s", e.what());
        }
    }
    
    // 成员变量
    rclcpp::Subscription<armor_detector_ros2::msg::GimbalCommand>::SharedPtr gimbal_cmd_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pipeline_control_srv_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    
    // 串口
    std::unique_ptr<boost::asio::io_context> io_context_;
    std::unique_ptr<boost::asio::serial_port> serial_port_;
    std::string serial_device_;
    int serial_baud_;
    double send_rate_;
    bool simulation_mode_{false};
    
    // 状态
    bool pipeline_enabled_{true};
    std::mutex cmd_mutex_;
    double latest_yaw_{0.0};
    double latest_pitch_{0.0};
    bool latest_fire_allowed_{false};
    bool cmd_received_{false};
    rclcpp::Time last_cmd_time_;
    int sent_count_{0};
};

}  // namespace armor_detector

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<armor_detector::GimbalControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
