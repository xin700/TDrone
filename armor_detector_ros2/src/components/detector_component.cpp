/**
 * @file detector_component.cpp
 * @brief 装甲板检测组件 - 支持进程内零拷贝通信
 * 
 * 组件化节点，实现装甲板关键点检测和数字分类
 * 使用 unique_ptr 接收和发布消息，支持零拷贝
 * 
 * 订阅话题：
 * - /video/image_raw (sensor_msgs/Image): 输入图像
 * 
 * 发布话题：
 * - /detector/armors (ArmorBBoxArray): 检测结果
 * - /detector/image_with_stamp (sensor_msgs/Image): 带检测结果的图像
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>

#include "core/ArmorInferer.hpp"
#include "armor_detector_ros2/msg/armor_b_box.hpp"
#include "armor_detector_ros2/msg/armor_b_box_array.hpp"
#include "armor_detector_ros2/msg/point2f.hpp"

using namespace std::chrono_literals;

namespace armor_detector
{

// ==================== 类别名称映射 ====================
/**
 * @brief 获取装甲板类别名称
 * @param tag_id 类别ID (0-7)
 * @return 类别名称字符串
 */
inline std::string getTagName(int tag_id) {
    static const std::vector<std::string> tag_names = {
        "Base",      // 0: 基地
        "Hero",      // 1: 英雄
        "Eng",       // 2: 工程
        "Inf3",      // 3: 步兵3
        "Inf4",      // 4: 步兵4
        "Inf5",      // 5: 步兵5
        "Sentry",    // 6: 哨兵
        "Outpost"    // 7: 前哨站
    };
    if (tag_id >= 0 && tag_id < static_cast<int>(tag_names.size())) {
        return tag_names[tag_id];
    }
    return "Unknown";
}

/**
 * @brief 获取装甲板颜色名称
 * @param color_id 颜色ID (0-3)
 * @return 颜色名称字符串
 */
inline std::string getColorName(int color_id) {
    static const std::vector<std::string> color_names = {
        "B",   // 0: 蓝色
        "R",   // 1: 红色
        "N",   // 2: 灰色/无色
        "P"    // 3: 紫色
    };
    if (color_id >= 0 && color_id < static_cast<int>(color_names.size())) {
        return color_names[color_id];
    }
    return "?";
}

/**
 * @brief 获取绘制颜色（根据装甲板颜色）
 * @param color_id 颜色ID
 * @return OpenCV BGR颜色
 */
inline cv::Scalar getDrawColor(int color_id) {
    switch (color_id) {
        case 0: return cv::Scalar(255, 0, 0);     // 蓝色装甲板 -> 蓝色线框
        case 1: return cv::Scalar(0, 0, 255);     // 红色装甲板 -> 红色线框
        case 2: return cv::Scalar(128, 128, 128); // 灰色装甲板 -> 灰色线框
        case 3: return cv::Scalar(255, 0, 255);   // 紫色装甲板 -> 紫色线框
        default: return cv::Scalar(0, 255, 255);  // 默认黄色
    }
}

/**
 * @class DetectorComponent
 * @brief 装甲板检测组件类
 * 
 * 特点：
 * - 使用 unique_ptr 消息实现零拷贝
 * - 可与其他组件在同一进程中运行
 */
class DetectorComponent : public rclcpp::Node
{
public:
    /**
     * @brief 组件构造函数
     * @param options 节点选项，包含进程内通信配置
     */
    explicit DetectorComponent(const rclcpp::NodeOptions& options)
        : Node("detector_component", options)
    {
        // ==================== 参数声明 ====================
        this->declare_parameter<std::string>("armor_model_path", "");
        this->declare_parameter<std::string>("classifier_model_path", "");
        this->declare_parameter<int>("color_flag", -1);

        armor_model_path_ = this->get_parameter("armor_model_path").as_string();
        classifier_model_path_ = this->get_parameter("classifier_model_path").as_string();
        int color_flag = this->get_parameter("color_flag").as_int();

        if (armor_model_path_.empty() || classifier_model_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "模型路径为空！请设置参数");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "进程内通信: %s", 
                   options.use_intra_process_comms() ? "启用(零拷贝)" : "禁用");

        // ==================== 初始化检测器 ====================
        try {
            armor_inferer_ = std::make_unique<DETECTOR::ArmorInferer>(
                armor_model_path_, 
                classifier_model_path_
            );
            armor_inferer_->setColorFlag(color_flag);
            RCLCPP_INFO(this->get_logger(), "检测器初始化成功");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "检测器初始化失败: %s", e.what());
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

        // ==================== 创建订阅者（支持 unique_ptr 回调） ====================
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/video/image_raw",
            rclcpp::QoS(10).reliable(),
            std::bind(&DetectorComponent::image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "检测组件已启动，等待图像...");
    }

private:
    /**
     * @brief 图像回调函数 - 支持零拷贝接收
     * @param msg 输入图像消息（进程内为零拷贝）
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        // 转换为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge转换失败: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image.clone();
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        
        // 执行检测
        DETECTOR::ArmorBBoxes detections;
        try {
            detections = (*armor_inferer_)(frame);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "检测失败: %s", e.what());
            return;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        float detection_time_ms = std::chrono::duration<float, std::milli>(
            end_time - start_time
        ).count();

        // ==================== 发布检测结果（使用 unique_ptr） ====================
        auto armor_array_msg = std::make_unique<armor_detector_ros2::msg::ArmorBBoxArray>();
        armor_array_msg->header = msg->header;
        armor_array_msg->detection_time_ms = detection_time_ms;

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
            armor_array_msg->armors.push_back(armor_msg);
        }

        armor_pub_->publish(std::move(armor_array_msg));

        // ==================== 绘制检测结果 ====================
        for (const auto& det : detections) {
            // 获取绘制颜色（根据装甲板颜色）
            cv::Scalar draw_color = getDrawColor(det.color_id);
            
            // 绘制四边形轮廓
            cv::line(frame, det.corners[0], det.corners[1], draw_color, 2);
            cv::line(frame, det.corners[1], det.corners[2], draw_color, 2);
            cv::line(frame, det.corners[2], det.corners[3], draw_color, 2);
            cv::line(frame, det.corners[3], det.corners[0], draw_color, 2);
            
            // 组合标签：颜色+类别，例如 "R-Hero", "B-Sentry"
            std::string label = getColorName(det.color_id) + "-" + getTagName(det.tag_id);
            
            // 绘制标签背景框（提高可读性）
            int baseline = 0;
            cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
            cv::Point text_pos(det.corners[0].x, det.corners[0].y - 5);
            cv::rectangle(frame, 
                         cv::Point(text_pos.x - 2, text_pos.y - text_size.height - 2),
                         cv::Point(text_pos.x + text_size.width + 2, text_pos.y + 2),
                         cv::Scalar(0, 0, 0), -1);
            
            // 绘制文字标签
            cv::putText(frame, label, text_pos, 
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2);
            
            // 绘制置信度
            char conf_text[16];
            snprintf(conf_text, sizeof(conf_text), "%.0f%%", det.confidence * 100);
            cv::putText(frame, conf_text, 
                       cv::Point(det.corners[1].x, det.corners[1].y - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }

        char info_text[128];
        snprintf(info_text, sizeof(info_text), "Armors: %zu | Time: %.1fms", 
                detections.size(), detection_time_ms);
        cv::putText(frame, info_text, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

        // ==================== 发布图像（使用 unique_ptr 零拷贝） ====================
        auto vis_msg = std::make_unique<sensor_msgs::msg::Image>();
        vis_msg->header = msg->header;
        vis_msg->height = frame.rows;
        vis_msg->width = frame.cols;
        vis_msg->encoding = "bgr8";
        vis_msg->is_bigendian = false;
        vis_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
        
        size_t size = frame.step * frame.rows;
        vis_msg->data.resize(size);
        memcpy(&vis_msg->data[0], frame.data, size);

        image_pub_->publish(std::move(vis_msg));

        // 统计
        frame_count_++;
        total_detection_time_ += detection_time_ms;

        if (frame_count_ % 100 == 0) {
            float avg_time = total_detection_time_ / frame_count_;
            RCLCPP_INFO(this->get_logger(), 
                "检测统计 - 帧数: %d, 平均耗时: %.2f ms",
                frame_count_, avg_time);
        }
    }

    // 成员变量
    rclcpp::Publisher<armor_detector_ros2::msg::ArmorBBoxArray>::SharedPtr armor_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    std::unique_ptr<DETECTOR::ArmorInferer> armor_inferer_;

    std::string armor_model_path_;
    std::string classifier_model_path_;

    int frame_count_ = 0;
    float total_detection_time_ = 0.0f;
};

}  // namespace armor_detector

// 注册组件
RCLCPP_COMPONENTS_REGISTER_NODE(armor_detector::DetectorComponent)
