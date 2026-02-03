/**
 * @file OpenSourceInferer.cpp
 * @brief 开源装甲板检测模型推理器实现 (0526.onnx)
 * 
 * 基于rm.cv.fans的0526.onnx模型
 * 使用OpenVINO进行推理，输出与TDrone项目兼容的ArmorBBoxes
 */

#include "core/OpenSourceInferer.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace DETECTOR
{
    /**
     * @brief 构造函数实现
     */
    OpenSourceInferer::OpenSourceInferer(const std::string& model_path)
    {
        initModel(model_path);
    }

    /**
     * @brief 模型初始化
     * 
     * OpenVINO初始化流程：
     * 1. 读取ONNX模型文件
     * 2. 配置预处理（BGR->RGB转换，归一化到[0,1]）
     * 3. 编译模型到CPU
     * 4. 创建推理请求
     */
    void OpenSourceInferer::initModel(const std::string& model_path)
    {
        std::cout << "[OpenSourceInferer] 加载模型: " << model_path << std::endl;
        
        // 读取ONNX模型
        model = core.read_model(model_path);
        
        // 配置预处理
        ov::preprocess::PrePostProcessor ppp(model);
        
        // 输入张量配置：NHWC格式，BGR颜色
        ppp.input().tensor()
            .set_element_type(ov::element::u8)
            .set_layout("NHWC")
            .set_color_format(ov::preprocess::ColorFormat::BGR);
        
        // 预处理步骤：转换为float32，BGR->RGB，归一化到[0,1]
        ppp.input().preprocess()
            .convert_element_type(ov::element::f32)
            .convert_color(ov::preprocess::ColorFormat::RGB)
            .scale({255.0, 255.0, 255.0});
        
        // 模型输入布局
        ppp.input().model().set_layout("NCHW");
        
        // 输出配置
        ppp.output().tensor().set_element_type(ov::element::f32);
        
        // 构建预处理后的模型
        model = ppp.build();
        
        // 编译模型，优化延迟
        compiled_model = core.compile_model(
            model,
            "GPU",
            ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)
        );
        
        // 创建推理请求
        infer_request = compiled_model.create_infer_request();
        
        std::cout << "[OpenSourceInferer] 模型加载完成" << std::endl;
    }

    /**
     * @brief 执行检测
     */
    ArmorBBoxes OpenSourceInferer::operator()(cv::Mat& img)
    {
        ArmorBBoxes results;
        
        if (img.empty()) {
            std::cerr << "[OpenSourceInferer] 输入图像为空!" << std::endl;
            return results;
        }
        
        // 保存原始尺寸
        orig_width = img.cols;
        orig_height = img.rows;
        
        // 计算缩放比例（从640x640映射回原图）
        scale_x = static_cast<float>(orig_width) / INPUT_W;
        scale_y = static_cast<float>(orig_height) / INPUT_H;
        
        // ==================== 图像预处理 ====================
        cv::Mat resized;
        cv::resize(img, resized, cv::Size(INPUT_W, INPUT_H));
        
        // 创建输入张量
        ov::Shape input_shape = {1, static_cast<size_t>(INPUT_H), static_cast<size_t>(INPUT_W), 3};
        ov::Tensor input_tensor(
            ov::element::u8,
            input_shape,
            resized.data
        );
        
        // 设置输入并推理
        infer_request.set_input_tensor(input_tensor);
        infer_request.infer();
        
        // ==================== 获取输出 ====================
        auto output = infer_request.get_output_tensor(0);
        ov::Shape output_shape = output.get_shape();
        
        // 输出格式: [1, num_anchors, 22]
        int num_anchors = output_shape[1];
        int output_dim = output_shape[2];
        
        cv::Mat output_buffer(num_anchors, output_dim, CV_32F, output.data<float>());
        
        // ==================== 后处理 ====================
        std::vector<ArmorBBox> candidates;
        std::vector<cv::Rect> boxes;
        std::vector<float> confidences;
        
        for (int i = 0; i < num_anchors; i++) {
            // 获取置信度 (index 8)
            float confidence = output_buffer.at<float>(i, 8);
            confidence = sigmoid(confidence);
            
            if (confidence < conf_threshold) {
                continue;
            }
            
            // 获取颜色分类 (index 9-12)
            cv::Mat color_scores = output_buffer.row(i).colRange(9, 13);
            cv::Point color_id;
            cv::minMaxLoc(color_scores, nullptr, nullptr, nullptr, &color_id);
            int detected_color = color_id.x;
            
            // 过滤灰色和紫色
            if (detected_color == 2 || detected_color == 3) {
                continue;
            }
            
            // 颜色过滤
            // color_flag: 0=检测蓝色(过滤红色), 1=检测红色(过滤蓝色)
            if (color_flag == 0 && detected_color == 1) {
                continue;  // 检测蓝色时过滤红色
            }
            if (color_flag == 1 && detected_color == 0) {
                continue;  // 检测红色时过滤蓝色
            }
            
            // 获取类别分类 (index 13-21)
            cv::Mat class_scores = output_buffer.row(i).colRange(13, 22);
            cv::Point class_id;
            cv::minMaxLoc(class_scores, nullptr, nullptr, nullptr, &class_id);
            int detected_class = class_id.x;
            
            // 获取角点坐标 (index 0-7)
            // 0526模型输出顺序: 左上(0,1), 左下(2,3), 右下(4,5), 右上(6,7)
            float x0 = output_buffer.at<float>(i, 0);  // 左上x
            float y0 = output_buffer.at<float>(i, 1);  // 左上y
            float x1 = output_buffer.at<float>(i, 2);  // 左下x
            float y1 = output_buffer.at<float>(i, 3);  // 左下y
            float x2 = output_buffer.at<float>(i, 4);  // 右下x
            float y2 = output_buffer.at<float>(i, 5);  // 右下y
            float x3 = output_buffer.at<float>(i, 6);  // 右上x
            float y3 = output_buffer.at<float>(i, 7);  // 右上y
            
            // 创建ArmorBBox
            ArmorBBox bbox;
            
            // 角点顺序调整为TDrone格式: 左上(0), 右上(1), 右下(2), 左下(3)
            // 并映射回原图坐标
            bbox.corners[0] = cv::Point2f(x0 * scale_x, y0 * scale_y);  // 左上
            bbox.corners[1] = cv::Point2f(x1 * scale_x, y1 * scale_y);  // 左下
            bbox.corners[2] = cv::Point2f(x2 * scale_x, y2 * scale_y);  // 右下
            bbox.corners[3] = cv::Point2f(x3 * scale_x, y3 * scale_y);  // 右上
            
            // 计算中心点
            bbox.center = cv::Point2f(
                (bbox.corners[0].x + bbox.corners[1].x + bbox.corners[2].x + bbox.corners[3].x) / 4.0f,
                (bbox.corners[0].y + bbox.corners[1].y + bbox.corners[2].y + bbox.corners[3].y) / 4.0f
            );
            
            // 计算边界矩形
            float min_x = std::min({bbox.corners[0].x, bbox.corners[1].x, bbox.corners[2].x, bbox.corners[3].x});
            float max_x = std::max({bbox.corners[0].x, bbox.corners[1].x, bbox.corners[2].x, bbox.corners[3].x});
            float min_y = std::min({bbox.corners[0].y, bbox.corners[1].y, bbox.corners[2].y, bbox.corners[3].y});
            float max_y = std::max({bbox.corners[0].y, bbox.corners[1].y, bbox.corners[2].y, bbox.corners[3].y});
            
            bbox.rect = cv::Rect(
                static_cast<int>(min_x),
                static_cast<int>(min_y),
                static_cast<int>(max_x - min_x),
                static_cast<int>(max_y - min_y)
            );
            
            // 尺寸过滤
            float width = max_x - min_x;
            float height = max_y - min_y;
            if (width < 10 || height < 10 || width > 500 || height > 500) {
                continue;
            }
            
            // 长宽比过滤 (装甲板通常是扁的)
            float ratio = std::max(width, height) / std::min(width, height);
            if (ratio < 1.0f || ratio > 6.0f) {
                continue;
            }
            
            // 填充其他字段
            bbox.confidence = confidence;
            bbox.color_id = detected_color;
            bbox.tag_id = detected_class;
            
            candidates.push_back(bbox);
            boxes.push_back(bbox.rect);
            confidences.push_back(confidence);
        }
        
        // ==================== NMS去重 ====================
        if (candidates.empty()) {
            return results;
        }
        
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);
        
        for (int idx : indices) {
            if (idx >= 0 && idx < static_cast<int>(candidates.size())) {
                results.push_back(candidates[idx]);
            }
        }
        
        return results;
    }

} // namespace DETECTOR
