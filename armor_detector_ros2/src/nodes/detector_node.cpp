/**
 * @file detector_node.cpp
 * @brief 装甲板检测ROS2节点
 * 
 * 功能：订阅图像话题，进行装甲板关键点检测和数字分类，发布检测结果
 * 
 * 订阅话题：
 * - /video/image_raw (sensor_msgs/Image): 输入图像
 * 
 * 发布话题：
 * - /detector/armors (ArmorBBoxArray): 检测结果
 * - /detector/image_with_stamp (sensor_msgs/Image): 带时间戳的原图（用于可视化同步）
 * 
 * 参数：
 * - armor_model_path: 装甲板检测模型路径
 * - classifier_model_path: 数字分类模型路径
 */

#include <rclcpp/rclcpp.hpp>
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
 * @class DetectorNode
 * @brief 装甲板检测节点类
 * 
 * 核心功能：
 * 1. 订阅图像话题
 * 2. 使用ArmorInferer进行检测
 * 3. 发布检测结果
 */
class DetectorNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     */
    DetectorNode() : Node("detector_node")
    {
        // ==================== 参数声明 ====================
        this->declare_parameter<std::string>("armor_model_path", "");
        this->declare_parameter<std::string>("classifier_model_path", "");
        this->declare_parameter<int>("color_flag", -1);  // -1表示不过滤颜色

        // 获取参数
        armor_model_path_ = this->get_parameter("armor_model_path").as_string();
        classifier_model_path_ = this->get_parameter("classifier_model_path").as_string();
        int color_flag = this->get_parameter("color_flag").as_int();

        // 验证模型路径
        if (armor_model_path_.empty() || classifier_model_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "模型路径为空！请设置参数");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "模型配置:");
        RCLCPP_INFO(this->get_logger(), "  装甲板检测模型: %s", armor_model_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  数字分类模型: %s", classifier_model_path_.c_str());

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
            std::bind(&DetectorNode::image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "检测节点已启动，等待图像...");
    }

private:
    /**
     * @brief 图像回调函数
     * @param msg 输入图像消息
     * 
     * 处理流程：
     * 1. 转换ROS消息为OpenCV格式
     * 2. 执行检测
     * 3. 转换检测结果为ROS消息
     * 4. 发布结果
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 记录开始时间
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
        
        // BGR -> RGB 转换（与原始代码一致，模型需要RGB输入）
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        
        // 执行检测
        DETECTOR::ArmorBBoxes detections;
        try {
            detections = (*armor_inferer_)(frame);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "检测失败: %s", e.what());
            return;
        }

        // 记录结束时间
        auto end_time = std::chrono::high_resolution_clock::now();
        float detection_time_ms = std::chrono::duration<float, std::milli>(
            end_time - start_time
        ).count();

        // ==================== 构建检测结果消息 ====================
        armor_detector_ros2::msg::ArmorBBoxArray armor_array_msg;
        armor_array_msg.header = msg->header;
        armor_array_msg.detection_time_ms = detection_time_ms;

        for (const auto& det : detections) {
            armor_detector_ros2::msg::ArmorBBox armor_msg;

            // 填充角点
            for (int i = 0; i < 4; i++) {
                armor_msg.corners[i].x = det.corners[i].x;
                armor_msg.corners[i].y = det.corners[i].y;
            }

            // 填充中心点
            armor_msg.center.x = det.center.x;
            armor_msg.center.y = det.center.y;

            // 填充其他信息
            armor_msg.confidence = det.confidence;
            armor_msg.color_id = det.color_id;
            armor_msg.tag_id = det.tag_id;
            armor_msg.rect_x = det.rect.x;
            armor_msg.rect_y = det.rect.y;
            armor_msg.rect_width = det.rect.width;
            armor_msg.rect_height = det.rect.height;

            armor_array_msg.armors.push_back(armor_msg);
        }

        // 发布检测结果
        armor_pub_->publish(armor_array_msg);

        // ==================== 在图像上绘制检测结果 ====================
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

        // 添加统计信息
        char info_text[128];
        snprintf(info_text, sizeof(info_text), "Armors: %zu | Time: %.1fms", 
                detections.size(), detection_time_ms);
        cv::putText(frame, info_text, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        // 转回 BGR 用于显示和保存
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

        // 发布已绘制的图像
        auto vis_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        image_pub_->publish(*vis_msg);

        // 更新统计
        frame_count_++;
        total_detection_time_ += detection_time_ms;

        // 定期输出统计信息
        if (frame_count_ % 100 == 0) {
            float avg_time = total_detection_time_ / frame_count_;
            RCLCPP_INFO(this->get_logger(), 
                "检测统计 - 帧数: %d, 平均耗时: %.2f ms, 当前检测数: %zu",
                frame_count_, avg_time, detections.size());
        }
    }

    // ==================== 成员变量 ====================
    // 发布者
    rclcpp::Publisher<armor_detector_ros2::msg::ArmorBBoxArray>::SharedPtr armor_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    
    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // 检测器
    std::unique_ptr<DETECTOR::ArmorInferer> armor_inferer_;

    // 参数
    std::string armor_model_path_;
    std::string classifier_model_path_;

    // 统计
    int frame_count_ = 0;
    float total_detection_time_ = 0.0f;
};

/**
 * @brief 主函数
 */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectorNode>());
    rclcpp::shutdown();
    return 0;
}
