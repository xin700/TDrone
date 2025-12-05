/**
 * @file NumberClassifier.cpp
 * @brief 装甲板数字分类器实现
 * 
 * 实现基于OpenVINO的数字分类功能
 * 核心功能：
 * 1. 图像预处理：透视变换、灰度转换、归一化
 * 2. CNN推理：使用OpenVINO进行分类
 * 3. 后处理：Softmax概率计算和类别选择
 */

#include "core/NumberClassifier.hpp"
#include <algorithm>
#include <cmath>

/**
 * @brief 构造函数实现
 * 
 * 初始化流程：
 * 1. 读取XML模型文件
 * 2. 编译模型到CPU设备
 * 3. 创建推理请求
 * 4. 验证模型输出维度与类别数匹配
 */
NumberClassifier::NumberClassifier(const std::string& model_path) 
    : model_path_(model_path) 
{
    // 读取模型文件（.xml格式，需要同目录下有对应的.bin权重文件）
    std::shared_ptr<ov::Model> model = core_.read_model(model_path_);
    
    // 编译模型到CPU，也可以改为"GPU"使用集成显卡加速
    compiled_model_ = core_.compile_model(model, "CPU");
    
    // 创建推理请求对象
    infer_request_ = compiled_model_.create_infer_request();

    // 验证模型输出维度
    ov::Shape output_shape = compiled_model_.output().get_shape();
    if (output_shape.back() != class_names_.size()) {
        throw std::invalid_argument(
            "Class count mismatch: model output " + 
            std::to_string(output_shape.back()) + 
            " vs expected " + std::to_string(class_names_.size())
        );
    }
}

/**
 * @brief 图像预处理实现
 * 
 * 处理步骤详解：
 * 1. 扩展角点范围：向外扩展50%，包含更多装甲板边缘信息
 * 2. 边界检查：确保扩展后的坐标在图像范围内
 * 3. 透视变换：使用3点仿射变换将装甲板区域变换到28x28正方形
 * 4. 灰度转换：BGR -> Gray
 * 5. 归一化：像素值从0-255映射到0-1
 * 6. 创建blob：转换为NCHW格式 [1,1,28,28]
 */
cv::Mat NumberClassifier::preprocess(cv::Mat& image, const std::vector<cv::Point2f>& points) {
    // 输入验证
    if (image.empty()) {
        throw std::runtime_error("Input image is empty");
    }
    
    // 复制角点并进行扩展（增加边缘信息）
    cv::Point2f srcPoints[4] = {points[0], points[1], points[2], points[3]};
    
    // 计算每条边的高度和宽度差异
    float height1 = (srcPoints[3].y - srcPoints[0].y) / 2;  // 左边高度差
    float height2 = (srcPoints[2].y - srcPoints[1].y) / 2;  // 右边高度差
    float weight1 = (srcPoints[3].x - srcPoints[0].x) / 2;  // 左边宽度差
    float weight2 = (srcPoints[2].x - srcPoints[1].x) / 2;  // 右边宽度差

    // 向外扩展角点（扩展50%边距）
    srcPoints[0].x -= weight1 / 2;
    srcPoints[3].x += weight1 / 2;
    srcPoints[0].y -= height1 / 2;
    srcPoints[3].y += height1 / 2;

    srcPoints[1].x -= weight2 / 2;
    srcPoints[2].x += weight2 / 2;
    srcPoints[1].y -= height2 / 2;
    srcPoints[2].y += height2 / 2;

    // 边界裁剪：确保坐标在图像范围内
    for (int i = 0; i < 4; ++i) {
        srcPoints[i].x = std::max(0.0f, std::min(srcPoints[i].x, static_cast<float>(image.cols)));
        srcPoints[i].y = std::max(0.0f, std::min(srcPoints[i].y, static_cast<float>(image.rows)));
    }

    // 定义目标角点（28x28正方形）
    cv::Point2f dstPoints[4] = {
        cv::Point2f(0, 0),                          // 左上
        cv::Point2f(input_size_, 0),                // 右上
        cv::Point2f(input_size_, input_size_),      // 右下
        cv::Point2f(0, input_size_)                 // 左下
    };
    
    // 计算仿射变换矩阵（使用前3个点）
    cv::Mat M = cv::getAffineTransform(srcPoints, dstPoints);
    
    // 执行仿射变换
    cv::Mat affine_img;
    cv::warpAffine(image, affine_img, M, cv::Size(input_size_, input_size_));
    
    // 转换为灰度图（数字分类不需要颜色信息）
    // 注意：虽然输入是 RGB 格式，但原始训练时使用的是 BGR2GRAY
    // 为了与训练数据保持一致，必须使用 BGR2GRAY（等效于 R/B 通道互换后的灰度）
    cv::cvtColor(affine_img, affine_img, cv::COLOR_BGR2GRAY);
    
    // 归一化到0-1范围
    if (normalize_) {
        affine_img.convertTo(affine_img, CV_32F, 1.0 / 255.0);
    }
    
    // 创建blob：将HW格式转换为NCHW格式
    return cv::dnn::blobFromImage(affine_img);
}

/**
 * @brief Softmax计算实现
 * 
 * 将原始logits转换为概率分布
 * 使用数值稳定的实现：减去最大值防止溢出
 */
std::vector<float> NumberClassifier::softmax(const float* data, size_t length) {
    std::vector<float> exp_values(length);
    
    // 找到最大值（数值稳定性）
    float max_val = *std::max_element(data, data + length);

    // 计算指数和求和
    float sum = 0.0f;
    for (size_t i = 0; i < length; ++i) {
        exp_values[i] = std::exp(data[i] - max_val);
        sum += exp_values[i];
    }

    // 归一化
    for (auto& val : exp_values) {
        val /= sum;
    }
    
    return exp_values;
}

/**
 * @brief 后处理实现
 * 
 * 从模型输出中提取分类结果：
 * 1. 获取原始logits
 * 2. 计算softmax概率
 * 3. 选择最大概率的类别
 * 4. 映射到实际的机器人编号
 */
int NumberClassifier::postprocess(const ov::Tensor& output_tensor) {
    const float* output_data = output_tensor.data<const float>();
    size_t num_classes = output_tensor.get_size();

    // 计算概率分布
    auto probabilities = softmax(output_data, num_classes);
    
    // 找到最大概率的索引
    int max_index = std::max_element(probabilities.begin(), probabilities.end()) - probabilities.begin();
    
    // 边界检查
    if (max_index < 0 || static_cast<size_t>(max_index) >= class_names_.size()) {
        throw std::runtime_error("Invalid class index: " + std::to_string(max_index));
    }

    // 返回对应的机器人编号
    return std::stoi(class_names_[max_index]);
}

/**
 * @brief 分类主函数实现
 * 
 * 完整的分类流程：
 * 1. 预处理图像
 * 2. 设置输入张量
 * 3. 执行推理
 * 4. 后处理获取结果
 */
int NumberClassifier::classify(cv::Mat& image, const std::vector<cv::Point2f>& points) {
    // 预处理
    cv::Mat input_blob = preprocess(image, points);
    
    // 创建输入张量
    ov::Tensor input_tensor(
        ov::element::f32,
        compiled_model_.input().get_shape(),
        input_blob.ptr<float>()
    );
    
    // 设置输入并执行推理
    infer_request_.set_input_tensor(input_tensor);
    infer_request_.infer();

    // 获取输出并后处理
    ov::Tensor output_tensor = infer_request_.get_output_tensor();
    return postprocess(output_tensor);
}
