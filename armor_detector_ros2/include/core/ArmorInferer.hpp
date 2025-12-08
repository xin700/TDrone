/**
 * @file ArmorInferer.hpp
 * @brief 装甲板关键点检测器头文件
 * 
 * 使用OpenVINO加载YOLOX模型进行装甲板4角点检测
 * 与 OrangeAim-Drone 保持一致，使用 BRpoints_nano 模型
 * 
 * 输入：原始BGR图像（任意尺寸）
 * 输出：检测到的装甲板列表（ArmorBBoxes）
 * 
 * BRpoints_nano 模型特点：
 * - 输入尺寸：416x416
 * - 输出维度：12 = 8(corners) + 1(objectness) + 2(colors) + 1(class)
 * - 颜色分类：蓝(0)/红(1)
 * - 支持NMS去重和候选框合并
 */

#pragma once

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include "core/BBoxes.h"
#include "core/NumberClassifier.hpp"

namespace DETECTOR
{
    /**
     * @class ArmorInferer
     * @brief 装甲板推理器类
     * 
     * 单阶段装甲板检测器，集成关键点检测和数字分类功能
     * 使用OpenVINO进行GPU/CPU加速推理
     */
    class ArmorInferer
    {
    public:
        /**
         * @brief 构造函数
         * @param armor_model_file 装甲板检测模型路径 (.xml)
         * @param classifier_model_file 数字分类模型路径 (.xml)
         * 
         * 初始化OpenVINO推理环境，加载两个模型：
         * 1. YOLOX装甲板关键点检测模型
         * 2. 数字分类CNN模型
         */
        explicit ArmorInferer(const std::string& armor_model_file, 
                             const std::string& classifier_model_file);

        /**
         * @brief 执行检测
         * @param img 输入图像（BGR格式）
         * @return 检测到的装甲板列表
         * 
         * 处理流程：
         * 1. 图像预处理（缩放、填充）
         * 2. YOLOX推理获取候选框
         * 3. 解码输出获取角点坐标
         * 4. NMS去重
         * 5. 对每个检测框进行数字分类
         */
        ArmorBBoxes operator()(cv::Mat& img);

        /**
         * @brief 设置敌方颜色标志
         * @param flag_ 颜色标志 (0=蓝方敌人, 1=红方敌人)
         * 
         * 用于过滤非目标颜色的装甲板
         * 当前实现中未启用颜色过滤
         */
        void setColorFlag(int flag_)
        {
            this->color_flag = flag_;
        };

    private:
        std::unique_ptr<NumberClassifier> number_classifier;  ///< 数字分类器

        ov::Core core;                      ///< OpenVINO核心
        std::shared_ptr<ov::Model> model;   ///< 原始网络模型
        ov::CompiledModel compiled_model;   ///< 编译后的可执行模型
        ov::InferRequest infer_request;     ///< 推理请求
        ov::Tensor input_tensor;            ///< 输入张量

        /**
         * @brief 图像变换矩阵
         * 用于将模型输出坐标映射回原图坐标
         */
        Eigen::Matrix<float, 3, 3> transfrom_matrix;

        /**
         * @brief 初始化模型
         * @param model_file 模型文件路径
         */
        void initModel(const std::string& model_file);

        /**
         * @brief 颜色过滤标志
         * 0: 蓝色, 1: 红色, 2: 灰色, 3: 紫色
         * -1: 不过滤
         */
        int color_flag = -1;
    };

    /**
     * @struct GridAndStride
     * @brief YOLOX网格和步长信息
     * 
     * 用于解码YOLOX输出，将特征图位置映射到原图坐标
     */
    struct GridAndStride
    {
        int grid0;   ///< 网格X索引
        int grid1;   ///< 网格Y索引
        int stride;  ///< 当前特征层的步长
    };

} // namespace DETECTOR
