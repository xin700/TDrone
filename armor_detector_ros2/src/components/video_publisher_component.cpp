/**
 * @file video_publisher_component.cpp
 * @brief 视频发布组件 - 支持进程内零拷贝通信
 * 
 * 组件化节点，可与其他组件在同一进程中运行，实现零拷贝图像传输
 * 
 * 发布话题：
 * - /video/image_raw (sensor_msgs/Image): 原始图像（unique_ptr零拷贝）
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

namespace armor_detector
{

/**
 * @class VideoPublisherComponent
 * @brief 视频发布组件类
 * 
 * 特点：
 * - 使用 unique_ptr 发布消息，支持进程内零拷贝
 * - 可作为组件加载到 ComponentContainer 中
 */
class VideoPublisherComponent : public rclcpp::Node
{
public:
    /**
     * @brief 组件构造函数
     * @param options 节点选项，包含进程内通信配置
     */
    explicit VideoPublisherComponent(const rclcpp::NodeOptions& options)
        : Node("video_publisher_component", options)
    {
        // ==================== 参数声明 ====================
        this->declare_parameter<std::string>("video_path", "");
        this->declare_parameter<double>("fps", 30.0);
        this->declare_parameter<bool>("loop", true);

        video_path_ = this->get_parameter("video_path").as_string();
        fps_ = this->get_parameter("fps").as_double();
        loop_ = this->get_parameter("loop").as_bool();

        if (video_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "视频路径为空！请设置 video_path 参数");
            return;
        }

        // ==================== 打开视频文件 ====================
        cap_.open(video_path_);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开视频文件: %s", video_path_.c_str());
            return;
        }

        video_fps_ = cap_.get(cv::CAP_PROP_FPS);
        total_frames_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_COUNT));
        frame_width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        frame_height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));

        RCLCPP_INFO(this->get_logger(), "视频信息: %dx%d, %.1f FPS, %d帧", 
                   frame_width_, frame_height_, video_fps_, total_frames_);
        RCLCPP_INFO(this->get_logger(), "进程内通信: %s", 
                   options.use_intra_process_comms() ? "启用(零拷贝)" : "禁用");

        // ==================== 创建发布者（启用进程内通信） ====================
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/video/image_raw", 
            rclcpp::QoS(10).reliable()
        );

        // ==================== 创建定时器 ====================
        auto period = std::chrono::duration<double>(1.0 / fps_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&VideoPublisherComponent::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "视频发布组件已启动");
    }

private:
    /**
     * @brief 定时器回调 - 使用 unique_ptr 实现零拷贝发布
     */
    void timer_callback()
    {
        cv::Mat frame;
        
        if (!cap_.read(frame)) {
            if (loop_) {
                cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
                RCLCPP_INFO(this->get_logger(), "视频循环播放");
                return;
            } else {
                RCLCPP_INFO(this->get_logger(), "视频播放完毕");
                timer_->cancel();
                return;
            }
        }

        // 使用 unique_ptr 创建消息（支持零拷贝）
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        
        // 填充消息头
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "camera_frame";
        
        // 填充图像数据
        msg->height = frame.rows;
        msg->width = frame.cols;
        msg->encoding = "bgr8";
        msg->is_bigendian = false;
        msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
        
        // 直接移动数据（避免拷贝）
        size_t size = frame.step * frame.rows;
        msg->data.resize(size);
        memcpy(&msg->data[0], frame.data, size);

        // 使用 unique_ptr 发布（进程内零拷贝）
        image_pub_->publish(std::move(msg));

        frame_count_++;
        if (frame_count_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "已发布 %d 帧", frame_count_);
        }
    }

    // 成员变量
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;

    std::string video_path_;
    double fps_;
    bool loop_;
    double video_fps_;
    int total_frames_;
    int frame_width_;
    int frame_height_;
    int frame_count_ = 0;
};

}  // namespace armor_detector

// 注册组件
RCLCPP_COMPONENTS_REGISTER_NODE(armor_detector::VideoPublisherComponent)
