/**
 * @file visualizer_node.cpp
 * @brief 检测结果可视化ROS2节点（简化版）
 * 
 * 功能：接收已绘制的图像并写入视频文件
 * detector_node 已经在图像上绘制了检测结果，这里只负责写入
 * 
 * 订阅话题：
 * - /detector/image_with_stamp (sensor_msgs/Image): 已绘制检测结果的图像
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

/**
 * @class VisualizerNode
 * @brief 可视化节点 - 只负责将已绘制的图像写入视频
 */
class VisualizerNode : public rclcpp::Node
{
public:
    VisualizerNode() : Node("visualizer_node")
    {
        // 参数
        this->declare_parameter<std::string>("output_video_path", "output.mp4");
        this->declare_parameter<double>("fps", 30.0);
        
        output_video_path_ = this->get_parameter("output_video_path").as_string();
        fps_ = this->get_parameter("fps").as_double();

        RCLCPP_INFO(this->get_logger(), "可视化配置:");
        RCLCPP_INFO(this->get_logger(), "  输出视频: %s", output_video_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  帧率: %.2f FPS", fps_);

        // 订阅已绘制的图像（使用reliable QoS）
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/detector/image_with_stamp", 
            rclcpp::QoS(100).reliable(),
            std::bind(&VisualizerNode::image_callback, this, std::placeholders::_1)
        );

        // 超时检测定时器
        timeout_timer_ = this->create_wall_timer(1s, [this]() {
            if (frame_count_ < 10) return;
            
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                now - last_image_time_).count();
            
            if (elapsed > 2 && !finished_) {
                RCLCPP_INFO(this->get_logger(), "检测到视频结束（2秒无新帧）");
                finish_and_exit();
            }
        });

        last_image_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "可视化节点已启动");
    }

    ~VisualizerNode()
    {
        finish_and_exit();
    }

private:
    void finish_and_exit()
    {
        if (!finished_) {
            finished_ = true;
            if (video_writer_.isOpened()) {
                video_writer_.release();
                RCLCPP_INFO(this->get_logger(), 
                    "视频写入完成，共 %d 帧，保存到: %s", 
                    frame_count_, output_video_path_.c_str());
            }
            rclcpp::shutdown();
        }
    }

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

        // 直接写入（图像已经被detector绘制过了）
        video_writer_.write(frame);
        frame_count_++;

        if (frame_count_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "已写入 %d 帧", frame_count_);
        }
    }

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;

    // 视频写入
    cv::VideoWriter video_writer_;
    std::string output_video_path_;
    double fps_;
    int frame_count_ = 0;
    bool finished_ = false;
    
    std::chrono::steady_clock::time_point last_image_time_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
