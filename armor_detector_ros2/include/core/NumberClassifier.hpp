/**
 * @file NumberClassifier.hpp
 * @brief 装甲板数字分类器头文件
 * 
 * 使用OpenVINO推理框架对装甲板上的数字进行分类
 * 输入：原始图像 + 装甲板四角点坐标
 * 输出：数字分类ID (0-7)
 * 
 * 分类映射：
 * - 类别索引0 -> ID 1 (步兵1)
 * - 类别索引1 -> ID 2 (步兵2)
 * - 类别索引2 -> ID 3 (步兵3)
 * - 类别索引3 -> ID 4 (步兵4)
 * - 类别索引4 -> ID 5 (步兵5)
 * - 类别索引5 -> ID 0 (基地)
 * - 类别索引6 -> ID 7 (前哨站)
 * - 类别索引7 -> ID 6 (哨兵)
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <vector>
#include <string>
#include <stdexcept>

/**
 * @class NumberClassifier
 * @brief 数字分类器类
 * 
 * 基于OpenVINO的装甲板数字分类器
 * 使用透视变换将装甲板区域裁剪并归一化后送入CNN进行分类
 */
class NumberClassifier {
private:
    ov::Core core_;                    ///< OpenVINO核心对象
    ov::CompiledModel compiled_model_; ///< 编译后的模型
    ov::InferRequest infer_request_;   ///< 推理请求对象

    int input_size_ = 28;              ///< 输入图像尺寸 (28x28)
    bool normalize_ = true;            ///< 是否进行归一化 (0-255 -> 0-1)
    
    /**
     * @brief 类别名称映射
     * 索引顺序对应模型输出的8个类别
     * 值为实际的机器人编号
     */
    std::vector<std::string> class_names_ = {"1", "2", "3", "4", "5", "0", "7", "6"};
    
    std::string model_path_;           ///< 模型文件路径

    /**
     * @brief 图像预处理
     * @param image 原始图像
     * @param points 装甲板四角点坐标
     * @return 预处理后的blob数据 [1,1,28,28]
     * 
     * 处理步骤：
     * 1. 扩展角点范围（增加边缘信息）
     * 2. 透视变换到28x28
     * 3. 转换为灰度图
     * 4. 归一化到0-1范围
     */
    cv::Mat preprocess(cv::Mat& image, const std::vector<cv::Point2f>& points);

    /**
     * @brief Softmax计算
     * @param data 原始logits数据
     * @param length 数据长度
     * @return 概率分布向量
     */
    std::vector<float> softmax(const float* data, size_t length);

    /**
     * @brief 后处理
     * @param output_tensor 模型输出张量
     * @return 分类ID
     */
    int postprocess(const ov::Tensor& output_tensor);

public:
    /**
     * @brief 构造函数
     * @param model_path 分类器模型文件路径 (.xml)
     */
    explicit NumberClassifier(const std::string& model_path);

    /**
     * @brief 执行分类
     * @param image 原始图像
     * @param points 装甲板四角点坐标
     * @return 分类ID (0-7)
     */
    int classify(cv::Mat& image, const std::vector<cv::Point2f>& points);
};
