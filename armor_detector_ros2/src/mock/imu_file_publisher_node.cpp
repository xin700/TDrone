/**
 * @file imu_file_publisher_node.cpp
 * @brief 从文件读取 IMU 数据并发布的 ROS2 节点
 * 
 * 功能：
 * 1. 从 txt 文件读取录制的 IMU 四元数数据
 * 2. 与视频同步发布 IMU 数据
 * 3. 支持时间戳匹配和插值
 * 
 * 文件格式：
 *   每行：timestamp w x y z
 *   例如：200.733343802 0.486388445 0.513251245 0.513248384 0.486390322
 * 
 * 订阅话题：
 * - /video/image_raw (sensor_msgs/Image): 用于同步时间戳
 * 
 * 发布话题：
 * - /imu/quaternion (geometry_msgs/QuaternionStamped): IMU 姿态四元数
 * 
 * 参数：
 * - imu_file_path: IMU 数据文件路径
 * - time_offset: 时间偏移（秒），用于对齐视频和 IMU 时间
 * - publish_rate: 独立发布模式的频率（Hz），0 表示跟随视频
 * - interpolate: 是否启用线性插值
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <mutex>

using namespace std::chrono_literals;

/**
 * @brief IMU 数据点结构体
 */
struct IMUData {
    double timestamp;
    double w, x, y, z;
};

/**
 * @class IMUFilePublisherNode
 * @brief 从文件读取并发布 IMU 数据的节点
 */
class IMUFilePublisherNode : public rclcpp::Node
{
public:
    IMUFilePublisherNode() : Node("imu_file_publisher_node")
    {
        // ==================== 参数声明 ====================
        this->declare_parameter<std::string>("imu_file_path", "");
        this->declare_parameter<double>("time_offset", 0.0);
        this->declare_parameter<double>("publish_rate", 0.0);  // 0 = 跟随视频
        this->declare_parameter<bool>("interpolate", true);
        this->declare_parameter<bool>("loop", false);
        
        // 获取参数
        imu_file_path_ = this->get_parameter("imu_file_path").as_string();
        time_offset_ = this->get_parameter("time_offset").as_double();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        interpolate_ = this->get_parameter("interpolate").as_bool();
        loop_ = this->get_parameter("loop").as_bool();
        
        // 加载 IMU 数据
        if (!loadIMUData(imu_file_path_)) {
            RCLCPP_ERROR(this->get_logger(), "无法加载 IMU 数据文件: %s", imu_file_path_.c_str());
            RCLCPP_ERROR(this->get_logger(), "请检查文件路径是否正确");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "============================================");
        RCLCPP_INFO(this->get_logger(), "  IMU 文件播放节点");
        RCLCPP_INFO(this->get_logger(), "============================================");
        RCLCPP_INFO(this->get_logger(), "  文件: %s", imu_file_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  数据点数: %zu", imu_data_.size());
        RCLCPP_INFO(this->get_logger(), "  时间范围: %.3f - %.3f 秒", 
                   imu_data_.front().timestamp, imu_data_.back().timestamp);
        RCLCPP_INFO(this->get_logger(), "  时间偏移: %.3f 秒", time_offset_);
        RCLCPP_INFO(this->get_logger(), "  插值: %s", interpolate_ ? "开启" : "关闭");
        RCLCPP_INFO(this->get_logger(), "============================================");
        
        // ==================== 创建发布者 ====================
        quaternion_pub_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
            "/imu/quaternion",
            rclcpp::QoS(10).reliable()
        );
        
        // ==================== 创建订阅者（同步模式）====================
        if (publish_rate_ <= 0) {
            RCLCPP_INFO(this->get_logger(), "模式: 视频同步（跟随 /video/image_raw）");
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/video/image_raw", 10,
                std::bind(&IMUFilePublisherNode::imageCallback, this, std::placeholders::_1));
        } else {
            // ==================== 独立发布模式 ====================
            RCLCPP_INFO(this->get_logger(), "模式: 独立发布 (%.1f Hz)", publish_rate_);
            auto period = std::chrono::duration<double>(1.0 / publish_rate_);
            timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                std::bind(&IMUFilePublisherNode::timerCallback, this));
            start_time_ = this->get_clock()->now();
        }
        
        RCLCPP_INFO(this->get_logger(), "IMU 文件播放节点已启动");
    }

private:
    /**
     * @brief 从文件加载 IMU 数据
     */
    bool loadIMUData(const std::string& filepath)
    {
        if (filepath.empty()) {
            RCLCPP_ERROR(this->get_logger(), "IMU 文件路径为空");
            return false;
        }
        
        std::ifstream file(filepath);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开文件: %s", filepath.c_str());
            return false;
        }
        
        imu_data_.clear();
        std::string line;
        int line_num = 0;
        
        while (std::getline(file, line)) {
            line_num++;
            if (line.empty() || line[0] == '#') continue;  // 跳过空行和注释
            
            std::istringstream iss(line);
            IMUData data;
            
            if (iss >> data.timestamp >> data.w >> data.x >> data.y >> data.z) {
                // 验证四元数
                double norm = std::sqrt(data.w*data.w + data.x*data.x + 
                                       data.y*data.y + data.z*data.z);
                if (std::abs(norm - 1.0) > 0.1) {
                    RCLCPP_WARN_ONCE(this->get_logger(), 
                                    "第 %d 行四元数未归一化 (norm=%.3f)，将自动归一化", 
                                    line_num, norm);
                    data.w /= norm;
                    data.x /= norm;
                    data.y /= norm;
                    data.z /= norm;
                }
                imu_data_.push_back(data);
            } else {
                RCLCPP_WARN(this->get_logger(), "无法解析第 %d 行: %s", line_num, line.c_str());
            }
        }
        
        file.close();
        
        if (imu_data_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "文件中没有有效的 IMU 数据");
            return false;
        }
        
        // 确保按时间戳排序
        std::sort(imu_data_.begin(), imu_data_.end(),
                 [](const IMUData& a, const IMUData& b) {
                     return a.timestamp < b.timestamp;
                 });
        
        return true;
    }
    
    /**
     * @brief 图像回调（同步模式）
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!first_image_received_) {
            first_image_received_ = true;
            first_image_time_ = msg->header.stamp;
            RCLCPP_INFO(this->get_logger(), "收到第一帧图像，开始同步 IMU 数据");
        }
        
        // 计算相对于第一帧的时间
        double video_time = (rclcpp::Time(msg->header.stamp) - first_image_time_).seconds();
        
        // 应用时间偏移，映射到 IMU 文件时间
        double imu_time = imu_data_.front().timestamp + video_time + time_offset_;
        
        // 发布对应时间的 IMU 数据
        publishIMUAtTime(imu_time, msg->header.stamp);
    }
    
    /**
     * @brief 定时器回调（独立模式）
     */
    void timerCallback()
    {
        double elapsed = (this->get_clock()->now() - start_time_).seconds();
        double imu_time = imu_data_.front().timestamp + elapsed + time_offset_;
        
        // 检查是否超出范围
        if (imu_time > imu_data_.back().timestamp) {
            if (loop_) {
                start_time_ = this->get_clock()->now();
                current_index_ = 0;
                RCLCPP_INFO(this->get_logger(), "IMU 数据循环播放");
            } else {
                RCLCPP_INFO_ONCE(this->get_logger(), "IMU 数据播放完成");
                return;
            }
        }
        
        publishIMUAtTime(imu_time, this->get_clock()->now());
    }
    
    /**
     * @brief 发布指定时间的 IMU 数据
     */
    void publishIMUAtTime(double target_time, rclcpp::Time stamp)
    {
        if (imu_data_.empty()) return;
        
        // 边界检查
        if (target_time <= imu_data_.front().timestamp) {
            publishIMUData(imu_data_.front(), stamp);
            return;
        }
        if (target_time >= imu_data_.back().timestamp) {
            publishIMUData(imu_data_.back(), stamp);
            return;
        }
        
        // 二分查找最近的数据点
        auto it = std::lower_bound(imu_data_.begin(), imu_data_.end(), target_time,
                                   [](const IMUData& data, double t) {
                                       return data.timestamp < t;
                                   });
        
        if (it == imu_data_.begin()) {
            publishIMUData(*it, stamp);
            return;
        }
        
        auto prev_it = std::prev(it);
        
        if (interpolate_) {
            // 线性插值
            double t = (target_time - prev_it->timestamp) / 
                      (it->timestamp - prev_it->timestamp);
            
            IMUData interpolated;
            interpolated.timestamp = target_time;
            
            // 四元数球面线性插值 (SLERP)
            double dot = prev_it->w * it->w + prev_it->x * it->x + 
                        prev_it->y * it->y + prev_it->z * it->z;
            
            // 处理反向四元数
            double sign = (dot < 0) ? -1.0 : 1.0;
            dot = std::abs(dot);
            
            if (dot > 0.9995) {
                // 接近平行，使用线性插值
                interpolated.w = prev_it->w + t * (sign * it->w - prev_it->w);
                interpolated.x = prev_it->x + t * (sign * it->x - prev_it->x);
                interpolated.y = prev_it->y + t * (sign * it->y - prev_it->y);
                interpolated.z = prev_it->z + t * (sign * it->z - prev_it->z);
            } else {
                // SLERP
                double theta = std::acos(dot);
                double sin_theta = std::sin(theta);
                double w1 = std::sin((1 - t) * theta) / sin_theta;
                double w2 = std::sin(t * theta) / sin_theta * sign;
                
                interpolated.w = w1 * prev_it->w + w2 * it->w;
                interpolated.x = w1 * prev_it->x + w2 * it->x;
                interpolated.y = w1 * prev_it->y + w2 * it->y;
                interpolated.z = w1 * prev_it->z + w2 * it->z;
            }
            
            // 归一化
            double norm = std::sqrt(interpolated.w*interpolated.w + interpolated.x*interpolated.x +
                                   interpolated.y*interpolated.y + interpolated.z*interpolated.z);
            interpolated.w /= norm;
            interpolated.x /= norm;
            interpolated.y /= norm;
            interpolated.z /= norm;
            
            publishIMUData(interpolated, stamp);
        } else {
            // 使用最近的数据点
            if (target_time - prev_it->timestamp < it->timestamp - target_time) {
                publishIMUData(*prev_it, stamp);
            } else {
                publishIMUData(*it, stamp);
            }
        }
    }
    
    /**
     * @brief 发布 IMU 数据
     */
    void publishIMUData(const IMUData& data, rclcpp::Time stamp)
    {
        auto msg = geometry_msgs::msg::QuaternionStamped();
        msg.header.stamp = stamp;
        msg.header.frame_id = "imu_frame";
        msg.quaternion.w = data.w;
        msg.quaternion.x = data.x;
        msg.quaternion.y = data.y;
        msg.quaternion.z = data.z;
        
        quaternion_pub_->publish(msg);
        publish_count_++;
        
        // 定期输出调试信息
        if (publish_count_ % 100 == 0) {
            // 计算欧拉角用于显示
            double sinp = 2.0 * (data.w * data.y - data.z * data.x);
            double pitch = (std::abs(sinp) >= 1.0) ? 
                          std::copysign(90.0, sinp) : 
                          std::asin(sinp) * 180.0 / M_PI;
            
            double siny_cosp = 2.0 * (data.w * data.z + data.x * data.y);
            double cosy_cosp = 1.0 - 2.0 * (data.y * data.y + data.z * data.z);
            double yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
            
            RCLCPP_DEBUG(this->get_logger(), 
                        "IMU [%d]: t=%.3f, yaw=%.1f°, pitch=%.1f°",
                        publish_count_, data.timestamp, yaw, pitch);
        }
    }
    
    // ==================== 成员变量 ====================
    // 发布者和订阅者
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr quaternion_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // IMU 数据
    std::vector<IMUData> imu_data_;
    std::string imu_file_path_;
    
    // 参数
    double time_offset_;
    double publish_rate_;
    bool interpolate_;
    bool loop_;
    
    // 状态
    bool first_image_received_ = false;
    rclcpp::Time first_image_time_;
    rclcpp::Time start_time_;
    size_t current_index_ = 0;
    int publish_count_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUFilePublisherNode>());
    rclcpp::shutdown();
    return 0;
}
