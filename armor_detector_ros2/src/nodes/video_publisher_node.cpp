/**
 * @file video_publisher_node.cpp
 * @brief 视频发布ROS2节点
 * 
 * 功能：从视频文件读取帧并发布到ROS2话题
 * 
 * 发布话题：
 * - /video/image_raw (sensor_msgs/Image): 原始图像
 * 
 * 参数：
 * - video_path: 视频文件路径
 * - fps: 发布帧率（默认30）
 * - loop: 是否循环播放（默认true）
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

/**
 * @class VideoPublisherNode
 * @brief 视频发布节点类
 * 
 * 使用OpenCV读取视频文件，通过定时器按设定帧率发布图像
 */
class VideoPublisherNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * 
     * 初始化流程：
     * 1. 声明和获取参数
     * 2. 打开视频文件
     * 3. 创建发布者
     * 4. 创建定时器
     */
    VideoPublisherNode() : Node("video_publisher_node")
    {
        // ==================== 参数声明 ====================
        this->declare_parameter<std::string>("video_path", "");
        this->declare_parameter<double>("fps", 30.0);
        this->declare_parameter<bool>("loop", true);

        // 获取参数
        video_path_ = this->get_parameter("video_path").as_string();
        fps_ = this->get_parameter("fps").as_double();
        loop_ = this->get_parameter("loop").as_bool();

        // 验证视频路径
        if (video_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "视频路径为空！请设置 video_path 参数");
            rclcpp::shutdown();
            return;
        }

        // ==================== 打开视频文件 ====================
        cap_.open(video_path_);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开视频文件: %s", video_path_.c_str());
            rclcpp::shutdown();
            return;
        }

        // 获取视频信息
        video_fps_ = cap_.get(cv::CAP_PROP_FPS);
        total_frames_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_COUNT));
        frame_width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        frame_height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));

        // 如果 fps <= 0，使用视频原始帧率
        if (fps_ <= 0) {
            fps_ = video_fps_;
        }

        RCLCPP_INFO(this->get_logger(), "视频信息:");
        RCLCPP_INFO(this->get_logger(), "  路径: %s", video_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  原始帧率: %.2f FPS", video_fps_);
        RCLCPP_INFO(this->get_logger(), "  发布帧率: %.2f FPS", fps_);
        RCLCPP_INFO(this->get_logger(), "  总帧数: %d", total_frames_);
        RCLCPP_INFO(this->get_logger(), "  分辨率: %dx%d", frame_width_, frame_height_);
        RCLCPP_INFO(this->get_logger(), "  循环播放: %s", loop_ ? "是" : "否");

        // ==================== 创建发布者 ====================
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/video/image_raw", 
            rclcpp::QoS(10).reliable()
        );

        // ==================== 创建定时器 ====================
        // 根据设定的fps计算定时器周期
        auto period = std::chrono::duration<double>(1.0 / fps_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&VideoPublisherNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "视频发布节点已启动");
        
        // 记录起始时间（用于计算视频时间戳）
        start_time_ = this->get_clock()->now();
    }

private:
    /**
     * @brief 定时器回调函数
     * 
     * 每次调用读取一帧并发布
     */
    void timer_callback()
    {
        cv::Mat frame;
        
        // 读取帧
        if (!cap_.read(frame)) {
            // 视频结束
            if (loop_) {
                // 循环播放：重置到第一帧
                cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
                frame_count_ = 0;  // 重置帧计数
                RCLCPP_INFO(this->get_logger(), "视频播放完毕，重新开始");
                return;
            } else {
                // 不循环：发布结束信号并关闭节点
                RCLCPP_INFO(this->get_logger(), "视频播放完毕");
                rclcpp::shutdown();
                return;
            }
        }

        // 转换为ROS消息
        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",
            frame
        ).toImageMsg();

        // 【关键修改】使用帧序号计算时间戳，而不是实际时钟
        // 这样即使处理速度慢，IMU也能正确匹配到对应帧
        double video_time = frame_count_ / fps_;
        msg->header.stamp = start_time_ + rclcpp::Duration::from_seconds(video_time);
        msg->header.frame_id = "camera_frame";

        // 发布
        image_pub_->publish(*msg);

        // 更新帧计数
        frame_count_++;
        if (frame_count_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "已发布 %d 帧", frame_count_);
        }
    }

    // ==================== 成员变量 ====================
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;  ///< 图像发布者
    rclcpp::TimerBase::SharedPtr timer_;                                ///< 定时器

    cv::VideoCapture cap_;  ///< 视频捕获对象

    std::string video_path_;  ///< 视频文件路径
    double fps_;              ///< 发布帧率
    bool loop_;               ///< 是否循环播放

    double video_fps_;        ///< 视频原始帧率
    int total_frames_;        ///< 视频总帧数
    int frame_width_;         ///< 帧宽度
    int frame_height_;        ///< 帧高度
    int frame_count_ = 0;     ///< 已发布帧数
    rclcpp::Time start_time_; ///< 视频起始时间（用于计算时间戳）
};

/**
 * @brief 主函数
 */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
