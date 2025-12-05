/**
 * @file BBoxes.h
 * @brief 装甲板边界框数据结构定义
 * 
 * 定义了装甲板检测结果的核心数据结构，包含：
 * - ArmorBBox: 单个装甲板的检测结果
 * - ArmorBBoxes: 装甲板检测结果数组
 * 
 * 这些数据结构用于存储YOLOX模型的输出，以及后续的数字分类结果
 */

#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace DETECTOR
{
    /**
     * @class ArmorBBox
     * @brief 装甲板边界框类
     * 
     * 存储单个装甲板的所有检测信息：
     * - 4个角点坐标（用于PnP解算）
     * - 中心点坐标
     * - 边界矩形（用于快速碰撞检测）
     * - 置信度、颜色和标签分类
     */
    class ArmorBBox
    {
    public:
        /**
         * @brief 默认构造函数，初始化所有成员为默认值
         */
        ArmorBBox() : center(0, 0), rect(0, 0, 0, 0), area(0.0f), confidence(0.0f), color_id(0), tag_id(0)
        {
            // 初始化4个角点为原点
            for (int i = 0; i < 4; i++)
            {
                corners[i] = cv::Point2f(0, 0);
            }
            points.clear();
        }

        /**
         * @brief 装甲板四个角点坐标
         * 顺序：左上(0) -> 右上(1) -> 右下(2) -> 左下(3)
         * 用于PnP位姿解算
         */
        cv::Point2f corners[4];

        /**
         * @brief 所有候选角点集合
         * 当多个检测框被合并时，存储所有候选角点用于平均计算
         * 可以降低角点检测的随机误差
         */
        std::vector<cv::Point2f> points;

        /**
         * @brief 装甲板中心点坐标
         * 由四个角点求平均得到
         */
        cv::Point2f center;

        /**
         * @brief 外接矩形框
         * 用于快速碰撞检测和IOU计算
         */
        cv::Rect rect;

        /**
         * @brief 外接矩形面积
         * 用于NMS排序
         */
        float area;

        /**
         * @brief 检测置信度 (0.0 - 1.0)
         * YOLOX模型输出的objectness分数
         */
        float confidence;

        /**
         * @brief 颜色分类ID
         * 0: 蓝色
         * 1: 红色
         * 2: 灰色（熄灭）
         * 3: 紫色
         */
        int color_id;

        /**
         * @brief 数字分类ID（机器人编号）
         * 1-5: 步兵1-5号
         * 0: 基地
         * 6: 哨兵
         * 7: 前哨站
         */
        int tag_id;
    };

    /**
     * @typedef ArmorBBoxes
     * @brief 装甲板检测结果数组类型
     */
    typedef std::vector<ArmorBBox> ArmorBBoxes;

} // namespace DETECTOR
