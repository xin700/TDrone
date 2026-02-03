/**
 * @file detector_opensource_node.cpp
 * @brief 基于开源模型(0526.onnx)的装甲板检测ROS2节点
 * 
 * 使用rm.cv.fans的0526.onnx模型，适配TDrone项目消息结构
 * 
 * 优化：使用最新帧缓存机制，避免帧堆积导致延迟累积
 * 
 * 订阅话题：
 * - /video/image_raw (sensor_msgs/Image): 输入图像
 * 
 * 发布话题：
 * - /detector/armors (ArmorBBoxArray): 检测结果
 * - /detector/image_with_stamp (sensor_msgs/Image): 带时间戳的原图（用于可视化同步）
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include "core/OpenSourceInferer.hpp"
#include "armor_detector_ros2/msg/armor_b_box.hpp"
#include "armor_detector_ros2/msg/armor_b_box_array.hpp"
#include "armor_detector_ros2/msg/point2f.hpp"

using namespace std::chrono_literals;

// ==================== 颜色校正函数 ====================
inline int correctColorByPixel(const cv::Mat& frame, const cv::Point2f corners[4]) {
    float min_x = std::min({corners[0].x, corners[1].x, corners[2].x, corners[3].x});
    float max_x = std::max({corners[0].x, corners[1].x, corners[2].x, corners[3].x});
    float min_y = std::min({corners[0].y, corners[1].y, corners[2].y, corners[3].y});
    float max_y = std::max({corners[0].y, corners[1].y, corners[2].y, corners[3].y});
    
    min_x = std::max(0.0f, min_x);
    min_y = std::max(0.0f, min_y);
    max_x = std::min(static_cast<float>(frame.cols - 1), max_x);
    max_y = std::min(static_cast<float>(frame.rows - 1), max_y);
    
    if (max_x <= min_x || max_y <= min_y) {
        return 0;
    }
    
    cv::Rect roi(static_cast<int>(min_x), static_cast<int>(min_y),
                 static_cast<int>(max_x - min_x), static_cast<int>(max_y - min_y));
    cv::Mat armor_region = frame(roi);
    
    long long red_sum = 0, blue_sum = 0;
    for (int y = 0; y < armor_region.rows; y++) {
        for (int x = 0; x < armor_region.cols; x++) {
            cv::Vec3b pixel = armor_region.at<cv::Vec3b>(y, x);
            red_sum += pixel[2];
            blue_sum += pixel[0];
        }
    }
    
    return (red_sum > blue_sum) ? 1 : 0;
}

// ==================== 类别名称映射 ====================
inline std::string getTagName(int tag_id) {
    static const std::vector<std::string> tag_names = {
        "Base", "Hero", "Eng", "Inf3", "Inf4", "Inf5", "Outpost", "Sentry", "Unknown"
    };
    if (tag_id >= 0 && tag_id < static_cast<int>(tag_names.size())) {
        return tag_names[tag_id];
    }
    return "Unknown";
}

inline std::string getColorName(int color_id) {
    static const std::vector<std::string> color_names = {"B", "R", "N", "P"};
    if (color_id >= 0 && color_id < static_cast<int>(color_names.size())) {
        return color_names[color_id];
    }
    return "?";
}

inline cv::Scalar getDrawColor(int color_id) {
    switch (color_id) {
        case 0: return cv::Scalar(255, 0, 0);
        case 1: return cv::Scalar(0, 0, 255);
        case 2: return cv::Scalar(128, 128, 128);
        case 3: return cv::Scalar(255, 0, 255);
        default: return cv::Scalar(0, 255, 255);
    }
}

/**
 * @class DetectorOpenSourceNode
 * @brief 开源模型装甲板检测节点类
 * 
 * 使用独立推理线程 + 最新帧缓存，避免帧堆积
 */
class DetectorOpenSourceNode : public rclcpp::Node
{
public:
    DetectorOpenSourceNode() : Node("detector_opensource_node")
    {
        // ==================== 参数声明 ====================
        this->declare_parameter<std::string>("model_path", "");
        this->declare_parameter<int>("color_flag", -1);
        this->declare_parameter<double>("conf_threshold", 0.6);
        this->declare_parameter<double>("nms_threshold", 0.45);
        this->declare_parameter<bool>("use_pixel_color_correction", true);

        // 获取参数
        std::string model_path = this->get_parameter("model_path").as_string();
        color_flag_ = this->get_parameter("color_flag").as_int();
        float conf_threshold = this->get_parameter("conf_threshold").as_double();
        float nms_threshold = this->get_parameter("nms_threshold").as_double();
        use_pixel_color_correction_ = this->get_parameter("use_pixel_color_correction").as_bool();

        // 验证模型路径
        if (model_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "模型路径为空！请设置 model_path 参数");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "=================================");
        RCLCPP_INFO(this->get_logger(), "开源模型检测器配置:");
        RCLCPP_INFO(this->get_logger(), "  模型路径: %s", model_path.c_str());
        RCLCPP_INFO(this->get_logger(), "  颜色过滤: %d (%s)", color_flag_, 
                    color_flag_ == 0 ? "检测蓝色" : (color_flag_ == 1 ? "检测红色" : "不过滤"));
        RCLCPP_INFO(this->get_logger(), "  置信度阈值: %.2f", conf_threshold);
        RCLCPP_INFO(this->get_logger(), "  NMS阈值: %.2f", nms_threshold);
        RCLCPP_INFO(this->get_logger(), "  像素颜色校正: %s", use_pixel_color_correction_ ? "启用" : "禁用");
        RCLCPP_INFO(this->get_logger(), "  模式: 独立推理线程 + 最新帧缓存");
        RCLCPP_INFO(this->get_logger(), "=================================");

        // ==================== 初始化检测器 ====================
        try {
            inferer_ = std::make_unique<DETECTOR::OpenSourceInferer>(model_path);
            inferer_->setColorFlag(color_flag_);
            inferer_->setConfThreshold(conf_threshold);
            inferer_->setNmsThreshold(nms_threshold);
            RCLCPP_INFO(this->get_logger(), "开源模型检测器初始化成功");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "检测器初始化失败: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // ==================== 创建发布者 ====================
        armor_pub_ = this->create_publisher<armor_detector_ros2::msg::ArmorBBoxArray>(
            "/detector/armors", 
            rclcpp::QoS(10).reliable()
        );

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/detector/image_with_stamp",
            rclcpp::QoS(10).reliable()
        );

        // ==================== 创建订阅者 ====================
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/video/image_raw",
            rclcpp::QoS(10).reliable(),
            std::bind(&DetectorOpenSourceNode::image_callback, this, std::placeholders::_1)
        );

        // ==================== 启动推理线程 ====================
        running_ = true;
        infer_thread_ = std::thread(&DetectorOpenSourceNode::inferenceLoop, this);

        RCLCPP_INFO(this->get_logger(), "开源模型检测节点已启动，等待图像...");
    }

    ~DetectorOpenSourceNode()
    {
        running_ = false;
        frame_cv_.notify_all();
        if (infer_thread_.joinable()) {
            infer_thread_.join();
        }
    }

private:
    /**
     * @brief 图像回调 - 只缓存最新帧，不做推理
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 转换为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge转换失败: %s", e.what());
            return;
        }

        // 缓存最新帧（加锁）
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = cv_ptr->image.clone();
            latest_header_ = msg->header;
            latest_msg_ = msg;
            has_new_frame_ = true;
        }
        frame_cv_.notify_one();
        
        received_frames_++;
    }

    /**
     * @brief 推理线程主循环
     */
    void inferenceLoop()
    {
        cv::Mat frame;
        std_msgs::msg::Header header;
        sensor_msgs::msg::Image::SharedPtr original_msg;

        while (running_) {
            // 等待新帧
            {
                std::unique_lock<std::mutex> lock(frame_mutex_);
                frame_cv_.wait(lock, [this] { return has_new_frame_ || !running_; });
                
                if (!running_) break;
                
                // 取出最新帧
                frame = std::move(latest_frame_);
                header = latest_header_;
                original_msg = latest_msg_;
                has_new_frame_ = false;
            }

            if (frame.empty()) continue;

            auto start_time = std::chrono::high_resolution_clock::now();

            // 执行推理
            DETECTOR::ArmorBBoxes detections;
            try {
                detections = (*inferer_)(frame);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "检测失败: %s", e.what());
                continue;
            }

            // 像素颜色校正
            if (use_pixel_color_correction_) {
                for (auto& det : detections) {
                    int corrected_color = correctColorByPixel(frame, det.corners);
                    det.color_id = corrected_color;
                }
            }

            auto end_time = std::chrono::high_resolution_clock::now();
            float detection_time_ms = std::chrono::duration<float, std::milli>(
                end_time - start_time
            ).count();

            // 构建并发布检测结果
            armor_detector_ros2::msg::ArmorBBoxArray armor_array_msg;
            armor_array_msg.header = header;
            armor_array_msg.detection_time_ms = detection_time_ms;

            for (const auto& det : detections) {
                armor_detector_ros2::msg::ArmorBBox armor_msg;

                for (int i = 0; i < 4; i++) {
                    armor_msg.corners[i].x = det.corners[i].x;
                    armor_msg.corners[i].y = det.corners[i].y;
                }

                armor_msg.center.x = det.center.x;
                armor_msg.center.y = det.center.y;
                armor_msg.confidence = det.confidence;
                armor_msg.color_id = det.color_id;
                armor_msg.tag_id = det.tag_id;
                armor_msg.rect_x = det.rect.x;
                armor_msg.rect_y = det.rect.y;
                armor_msg.rect_width = det.rect.width;
                armor_msg.rect_height = det.rect.height;

                armor_array_msg.armors.push_back(armor_msg);
            }

            armor_pub_->publish(armor_array_msg);

            // 发布原始图像
            if (original_msg) {
                image_pub_->publish(*original_msg);
            }

            processed_frames_++;

            // 统计日志
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration<float>(now - last_log_time_).count();
            if (elapsed >= 1.0f) {
                float process_fps = processed_frames_ / elapsed;
                float receive_fps = received_frames_ / elapsed;
                int dropped = received_frames_ - processed_frames_;
                
                RCLCPP_INFO(this->get_logger(), 
                    "[接收: %.1f fps, 处理: %.1f fps, 丢弃: %d帧] 检测耗时: %.2f ms",
                    receive_fps, process_fps, dropped, detection_time_ms);
                
                processed_frames_ = 0;
                received_frames_ = 0;
                last_log_time_ = now;
            }
        }
    }

    // 检测器
    std::unique_ptr<DETECTOR::OpenSourceInferer> inferer_;
    bool use_pixel_color_correction_ = true;
    int color_flag_ = -1;

    // 最新帧缓存
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    cv::Mat latest_frame_;
    std_msgs::msg::Header latest_header_;
    sensor_msgs::msg::Image::SharedPtr latest_msg_;
    bool has_new_frame_ = false;

    // 推理线程
    std::thread infer_thread_;
    std::atomic<bool> running_{false};

    // 统计
    int processed_frames_ = 0;
    int received_frames_ = 0;
    std::chrono::steady_clock::time_point last_log_time_ = std::chrono::steady_clock::now();

    // ROS2 发布者/订阅者
    rclcpp::Publisher<armor_detector_ros2::msg::ArmorBBoxArray>::SharedPtr armor_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectorOpenSourceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
