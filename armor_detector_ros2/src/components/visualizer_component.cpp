/**
 * @file visualizer_component.cpp
 * @brief 可视化组件 - 支持进程内零拷贝通信
 * 
 * 组件化节点，将检测结果写入视频文件
 * 使用 unique_ptr 接收消息，支持零拷贝
 * 
 * 订阅话题：
 * - /detector/image_with_stamp (sensor_msgs/Image): 已绘制检测结果的图像
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

namespace armor_detector
{

/**
 * @class VisualizerComponent
 * @brief 可视化组件类
 * 
 * 特点：
 * - 使用 unique_ptr 接收消息实现零拷贝
 * - 可与其他组件在同一进程中运行
 */
class VisualizerComponent : public rclcpp::Node
{
public:
    /**
     * @brief 组件构造函数
     * @param options 节点选项，包含进程内通信配置
     */
    explicit VisualizerComponent(const rclcpp::NodeOptions& options)
        : Node("visualizer_component", options)
    {
        // 参数
        this->declare_parameter<std::string>("output_video_path", "output.mp4");
        this->declare_parameter<double>("fps", 30.0);
        
        output_video_path_ = this->get_parameter("output_video_path").as_string();
        fps_ = this->get_parameter("fps").as_double();

        RCLCPP_INFO(this->get_logger(), "可视化配置:");
        RCLCPP_INFO(this->get_logger(), "  输出视频: %s", output_video_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  帧率: %.2f FPS", fps_);
        RCLCPP_INFO(this->get_logger(), "  进程内通信: %s", 
                   options.use_intra_process_comms() ? "启用(零拷贝)" : "禁用");

        // 订阅已绘制的图像
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/detector/image_with_stamp", 
            rclcpp::QoS(100).reliable(),
            std::bind(&VisualizerComponent::image_callback, this, std::placeholders::_1)
        );

        // 超时检测定时器
        timeout_timer_ = this->create_wall_timer(1s, [this]() {
            if (frame_count_ < 10) return;
            
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                now - last_image_time_).count();
            
            if (elapsed > 2 && !finished_) {
                RCLCPP_INFO(this->get_logger(), "检测到视频结束（2秒无新帧）");
                finish_writing();
            }
        });

        last_image_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "可视化组件已启动");
    }

    ~VisualizerComponent()
    {
        finish_writing();
    }

private:
    /**
     * @brief 完成视频写入
     */
    void finish_writing()
    {
        if (!finished_) {
            finished_ = true;
            if (video_writer_.isOpened()) {
                video_writer_.release();
                RCLCPP_INFO(this->get_logger(), 
                    "视频写入完成，共 %d 帧，保存到: %s", 
                    frame_count_, output_video_path_.c_str());
            }
        }
    }

    /**
     * @brief 图像回调函数 - 支持零拷贝接收
     * @param msg 输入图像消息
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        last_image_time_ = std::chrono::steady_clock::now();
        
        // 转换图像
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge错误: %s", e.what());
            return;
        }

        // 初始化视频写入器
        if (!video_writer_.isOpened()) {
            int fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
            std::string out_path = output_video_path_;
            if (out_path.size() > 4 && out_path.substr(out_path.size()-4) == ".mp4") {
                out_path = out_path.substr(0, out_path.size()-4) + ".avi";
                output_video_path_ = out_path;
            }
            
            video_writer_.open(out_path, fourcc, fps_, frame.size());
            if (!video_writer_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "无法创建视频: %s", out_path.c_str());
                return;
            }
            RCLCPP_INFO(this->get_logger(), "开始写入视频: %s (%dx%d)", 
                       out_path.c_str(), frame.cols, frame.rows);
        }

        // 添加帧号信息
        char info[64];
        snprintf(info, sizeof(info), "Frame: %d", frame_count_);
        cv::putText(frame, info, cv::Point(10, frame.rows - 20), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

        // 写入视频
        video_writer_.write(frame);
        frame_count_++;

        if (frame_count_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "已写入 %d 帧", frame_count_);
        }
    }

    // 成员变量
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;

    cv::VideoWriter video_writer_;
    std::string output_video_path_;
    double fps_;
    int frame_count_ = 0;
    bool finished_ = false;
    
    std::chrono::steady_clock::time_point last_image_time_;
};

}  // namespace armor_detector

// 注册组件
RCLCPP_COMPONENTS_REGISTER_NODE(armor_detector::VisualizerComponent)
