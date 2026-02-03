/**
 * @file OpenSourceInferer.hpp
 * @brief 开源装甲板检测模型推理器 (0526.onnx)
 * 
 * 使用OpenVINO加载0526.onnx模型进行装甲板4角点检测
 * 与现有TDrone项目消息结构兼容
 * 
 * 0526.onnx模型特点：
 * - 输入尺寸：640x640 (RGB格式)
 * - 输出维度：22 = 8(corners) + 1(objectness) + 4(colors) + 9(classes)
 * - 颜色分类：蓝(0)/红(1)/灰(2)/紫(3)
 * - 类别分类：0-Base, 1-Hero, 2-Eng, 3-Inf3, 4-Inf4, 5-Inf5, 6-Outpost, 7-Sentry, 8-Unknown
 */

#pragma once

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include "core/BBoxes.h"
#include <memory>
#include <vector>

namespace DETECTOR
{
    /**
     * @class OpenSourceInferer
     * @brief 开源模型推理器类
     * 
     * 单阶段装甲板检测器，直接输出角点坐标和分类结果
     * 使用OpenVINO进行CPU加速推理
     */
    class OpenSourceInferer
    {
    public:
        /**
         * @brief 构造函数
         * @param model_path ONNX模型路径 (.onnx)
         * 
         * 初始化OpenVINO推理环境，加载0526.onnx模型
         * 该模型同时输出角点坐标、颜色和类别
         */
        explicit OpenSourceInferer(const std::string& model_path);
        
        /**
         * @brief 析构函数
         */
        ~OpenSourceInferer() = default;

        /**
         * @brief 执行检测
         * @param img 输入图像（BGR格式）
         * @return 检测到的装甲板列表
         * 
         * 处理流程：
         * 1. 图像预处理（缩放到640x640、RGB转换、归一化）
         * 2. OpenVINO推理
         * 3. 解码输出获取角点坐标
         * 4. NMS去重
         * 5. 坐标映射回原图
         */
        ArmorBBoxes operator()(cv::Mat& img);

        /**
         * @brief 设置敌方颜色标志
         * @param flag_ 颜色标志 (0=蓝方敌人/检测红色, 1=红方敌人/检测蓝色)
         * 
         * 用于过滤非目标颜色的装甲板
         */
        void setColorFlag(int flag_) { color_flag = flag_; }
        
        /**
         * @brief 设置置信度阈值
         * @param thresh 置信度阈值 (0.0-1.0)
         */
        void setConfThreshold(float thresh) { conf_threshold = thresh; }
        
        /**
         * @brief 设置NMS阈值
         * @param thresh NMS阈值 (0.0-1.0)
         */
        void setNmsThreshold(float thresh) { nms_threshold = thresh; }

    private:
        // ==================== 模型常量 ====================
        static constexpr int INPUT_W = 640;           // 输入图像宽度
        static constexpr int INPUT_H = 640;           // 输入图像高度
        static constexpr int NUM_COLORS = 4;          // 颜色数：蓝/红/灰/紫
        static constexpr int NUM_CLASSES = 9;         // 类别数
        // 输出维度：8(corners) + 1(objectness) + 4(colors) + 9(classes) = 22
        static constexpr int OUTPUT_DIM = 8 + 1 + NUM_COLORS + NUM_CLASSES;
        
        // ==================== 推理参数 ====================
        float conf_threshold = 0.5f;     // 置信度阈值
        float nms_threshold = 0.45f;     // NMS阈值
        int color_flag = -1;             // 颜色过滤标志 (-1不过滤)
        
        // ==================== OpenVINO组件 ====================
        ov::Core core;                      ///< OpenVINO核心
        std::shared_ptr<ov::Model> model;   ///< 原始网络模型
        ov::CompiledModel compiled_model;   ///< 编译后的可执行模型
        ov::InferRequest infer_request;     ///< 推理请求
        
        // ==================== 图像变换 ====================
        float scale_x = 1.0f;               ///< X方向缩放比例
        float scale_y = 1.0f;               ///< Y方向缩放比例
        int orig_width = 0;                 ///< 原始图像宽度
        int orig_height = 0;                ///< 原始图像高度
        
        /**
         * @brief 初始化模型
         * @param model_path 模型文件路径
         */
        void initModel(const std::string& model_path);
        
        /**
         * @brief Sigmoid激活函数
         */
        inline double sigmoid(double x) const {
            if (x > 0)
                return 1.0 / (1.0 + std::exp(-x));
            else
                return std::exp(x) / (1.0 + std::exp(x));
        }
    };

} // namespace DETECTOR
