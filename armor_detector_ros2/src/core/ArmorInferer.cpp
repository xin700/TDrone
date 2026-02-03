/**
 * @file ArmorInferer.cpp
 * @brief 装甲板关键点检测器实现
 * 
 * 基于YOLOX的单阶段装甲板检测器
 * 特点：
 * - 直接回归4个角点坐标（无需anchor）
 * - 多尺度特征融合（8/16/32步长）
 * - 集成颜色分类
 * - NMS去重和候选框合并
 */

#include "core/ArmorInferer.hpp"
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <cmath>

namespace DETECTOR
{
    // ==================== 模型参数 ====================
    // 与 OrangeAim-Drone 保持一致，使用 BRpoints_nano 模型
    static constexpr int INPUT_W = 416;           // 输入图像宽度
    static constexpr int INPUT_H = 416;           // 输入图像高度
    // BRpoints_nano 模型输出: [1, 3549, 12]
    // 12 = 8(corners: x1,y1,x2,y2,x3,y3,x4,y4) + 1(objectness) + 2(colors) + 1(class)
    // 颜色: 0=蓝, 1=红
    // 类别: 由 NumberClassifier 重新分类
    static constexpr int NUM_CLASSES = 1;         // 类别数（BRpoints_nano）
    static constexpr int NUM_COLORS = 2;          // 颜色数（BRpoints_nano: 蓝/红）
    static constexpr int OUTPUT_DIM = 9 + NUM_COLORS + NUM_CLASSES;  // 12维
    static constexpr int TOPK = 128;              // 保留的最大候选框数
    static constexpr float NMS_THRESH = 0.3;      // NMS阈值
    static constexpr float BBOX_CONF_THRESH = 0.5; // 置信度阈值
    static constexpr float MERGE_CONF_ERROR = 0.15; // 合并时允许的置信度差异
    static constexpr float MERGE_MIN_IOU = 0.9;   // 合并时要求的最小IOU

    /**
     * @brief 构造函数实现
     * 
     * 初始化装甲板检测模型和数字分类器
     */
    ArmorInferer::ArmorInferer(const std::string& armor_model_file, 
                               const std::string& classifier_model_file)
    {
        initModel(armor_model_file);
        number_classifier = std::make_unique<NumberClassifier>(classifier_model_file);
    }

    /**
     * @brief 模型初始化
     * 
     * OpenVINO初始化流程：
     * 1. 读取IR模型文件
     * 2. 配置预处理（设置输入输出精度为float32）
     * 3. 编译模型到CPU
     * 4. 创建推理请求
     */
    void ArmorInferer::initModel(const std::string& model_file)
    {
        // 启用CPU性能分析
        core.set_property("CPU", ov::enable_profiling(true));
        
        // 读取模型
        model = core.read_model(model_file);
        
        // 配置预处理
        ov::preprocess::PrePostProcessor ppp(model);
        ppp.input().tensor().set_element_type(ov::element::f32);
        ppp.output().tensor().set_element_type(ov::element::f32);
        ppp.build();
        
        // 编译模型，优化延迟
        compiled_model = core.compile_model(
            model,
            "CPU",
            ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)
        );
        
        // 创建推理请求
        infer_request = compiled_model.create_infer_request();
    }

    /**
     * @brief 找到数组中最大值的索引
     * @param ptr 数据指针
     * @param len 数据长度
     * @return 最大值索引
     */
    static inline int argmax(const float* ptr, int len)
    {
        int max_arg = 0;
        for (int i = 1; i < len; i++) {
            if (ptr[i] > ptr[max_arg])
                max_arg = i;
        }
        return max_arg;
    }

    /**
     * @brief 等比例缩放图像到模型输入尺寸
     * @param img 输入图像
     * @param transform_matrix 输出的坐标变换矩阵（用于将检测结果映射回原图）
     * @return 缩放后的图像
     * 
     * 处理流程：
     * 1. 计算缩放比例（保持长宽比）
     * 2. 缩放图像
     * 3. 计算填充大小（居中填充）
     * 4. 添加黑边
     * 5. 计算逆变换矩阵
     */
    inline cv::Mat scaledResize(const cv::Mat& img, Eigen::Matrix<float, 3, 3>& transform_matrix)
    {
        // 计算缩放比例
        float r = std::min(INPUT_W / (img.cols * 1.0), INPUT_H / (img.rows * 1.0));
        int unpad_w = r * img.cols;
        int unpad_h = r * img.rows;

        // 计算填充大小（均分到两边）
        int dw = INPUT_W - unpad_w;
        int dh = INPUT_H - unpad_h;
        dw /= 2;
        dh /= 2;

        // 构建逆变换矩阵（从模型输出坐标到原图坐标）
        transform_matrix << 1.0 / r, 0, -dw / r,
                           0, 1.0 / r, -dh / r,
                           0, 0, 1;

        // 缩放图像
        cv::Mat re;
        cv::resize(img, re, cv::Size(unpad_w, unpad_h));
        
        // 添加黑边填充
        cv::Mat out;
        cv::copyMakeBorder(re, out, dh, dh, dw, dw, cv::BORDER_CONSTANT);
        
        return out;
    }

    /**
     * @brief 生成YOLOX的网格和步长信息
     * @param target_w 目标宽度
     * @param target_h 目标高度
     * @param strides 各层步长
     * @param grid_strides 输出的网格步长列表
     * 
     * YOLOX使用3个尺度的特征图：
     * - 52x52 (步长8): 检测小目标
     * - 26x26 (步长16): 检测中等目标
     * - 13x13 (步长32): 检测大目标
     */
    static void generate_grids_and_stride(
        const int target_w, 
        const int target_h, 
        std::vector<int>& strides, 
        std::vector<GridAndStride>& grid_strides)
    {
        for (auto stride : strides) {
            int num_grid_w = target_w / stride;
            int num_grid_h = target_h / stride;

            for (int g1 = 0; g1 < num_grid_h; g1++) {
                for (int g0 = 0; g0 < num_grid_w; g0++) {
                    GridAndStride grid_stride = {g0, g1, stride};
                    grid_strides.emplace_back(grid_stride);
                }
            }
        }
    }

    /**
     * @brief 生成YOLOX检测候选框
     * @param grid_strides 网格步长信息
     * @param feat_ptr 特征图数据指针
     * @param transform_matrix 坐标变换矩阵
     * @param prob_threshold 置信度阈值
     * @param bboxes 输出的候选框列表
     * @param color_flag 颜色过滤标志
     * 
     * 每个anchor的输出格式：
     * [x1,y1,x2,y2,x3,y3,x4,y4, objectness, color*2, class*1]
     * = 8 + 1 + 2 + 1 = 12维
     */
    static void generateYoloxProposals(
        std::vector<GridAndStride> grid_strides, 
        const float* feat_ptr,
        Eigen::Matrix<float, 3, 3>& transform_matrix, 
        float prob_threshold,
        ArmorBBoxes& bboxes, 
        int& color_flag)
    {
        const int num_anchors = grid_strides.size();
        
        // 遍历所有anchor
        for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++)
        {
            const int grid0 = grid_strides[anchor_idx].grid0;
            const int grid1 = grid_strides[anchor_idx].grid1;
            const int stride = grid_strides[anchor_idx].stride;
            // 使用模型实际输出维度计算位置
            const int basic_pos = anchor_idx * OUTPUT_DIM;

            // 解码4个角点坐标（相对于网格中心的偏移）
            float x_1 = (feat_ptr[basic_pos + 0] + grid0) * stride;
            float y_1 = (feat_ptr[basic_pos + 1] + grid1) * stride;
            float x_2 = (feat_ptr[basic_pos + 2] + grid0) * stride;
            float y_2 = (feat_ptr[basic_pos + 3] + grid1) * stride;
            float x_3 = (feat_ptr[basic_pos + 4] + grid0) * stride;
            float y_3 = (feat_ptr[basic_pos + 5] + grid1) * stride;
            float x_4 = (feat_ptr[basic_pos + 6] + grid0) * stride;
            float y_4 = (feat_ptr[basic_pos + 7] + grid1) * stride;

            // 解码objectness和分类
            float box_objectness = feat_ptr[basic_pos + 8];
            
            // 获取颜色概率（用于调试）
            float blue_prob = feat_ptr[basic_pos + 9];
            float red_prob = feat_ptr[basic_pos + 10];
            int box_color = argmax(feat_ptr + basic_pos + 9, NUM_COLORS);
            int box_class = argmax(feat_ptr + basic_pos + 9 + NUM_COLORS, NUM_CLASSES);

            float box_prob = box_objectness;
            
            // 置信度过滤
            if (box_prob >= prob_threshold)
            {
                // 检查坐标有效性（必须在合理范围内）
                // 允许少量边界外溢（模型输出可能略微超出）
                bool valid_coords = true;
                float coords[] = {x_1, y_1, x_2, y_2, x_3, y_3, x_4, y_4};
                for (int k = 0; k < 8; k++) {
                    float limit = (k % 2 == 0) ? INPUT_W : INPUT_H;
                    // 允许±50像素的边界外溢
                    if (std::isnan(coords[k]) || std::isinf(coords[k]) ||
                        coords[k] < -50 || coords[k] > limit + 50) {
                        valid_coords = false;
                        break;
                    }
                }
                
                if (!valid_coords) continue;  // 跳过无效检测

                ArmorBBox bbox;

                // 使用Eigen矩阵进行坐标变换（从模型输出坐标到原图坐标）
                Eigen::Matrix<float, 3, 4> apex_norm;
                Eigen::Matrix<float, 3, 4> apex_dst;

                apex_norm << x_1, x_2, x_3, x_4,
                            y_1, y_2, y_3, y_4,
                            1, 1, 1, 1;

                apex_dst = transform_matrix * apex_norm;

                // 提取变换后的角点坐标
                for (int i = 0; i < 4; i++) {
                    bbox.corners[i] = cv::Point2f(apex_dst(0, i), apex_dst(1, i));
                    bbox.points.push_back(bbox.corners[i]);
                }

                // 计算外接矩形
                std::vector<cv::Point2f> tmp(bbox.corners, bbox.corners + 4);
                bbox.rect = cv::boundingRect(tmp);
                bbox.color_id = box_color;
                bbox.confidence = box_prob;
                bbox.area = bbox.rect.area();
                
                // 调试：输出颜色概率
                // printf("[DEBUG] Color probs: blue=%.3f, red=%.3f -> %s\n", 
                //        blue_prob, red_prob, box_color == 0 ? "BLUE" : "RED");
                
                bboxes.push_back(bbox);
            }
        }
    }

    /**
     * @brief 快速排序（按置信度降序）
     */
    static void qsort_descent_inplace(ArmorBBoxes& facebboxes, int left, int right)
    {
        int i = left;
        int j = right;
        float p = facebboxes[(left + right) / 2].confidence;

        while (i <= j) {
            while (facebboxes[i].confidence > p) i++;
            while (facebboxes[j].confidence < p) j--;

            if (i <= j) {
                std::swap(facebboxes[i], facebboxes[j]);
                i++;
                j--;
            }
        }
        
        if (left < j) qsort_descent_inplace(facebboxes, left, j);
        if (i < right) qsort_descent_inplace(facebboxes, i, right);
    }

    static void qsort_descent_inplace(ArmorBBoxes& bboxes)
    {
        if (bboxes.empty()) return;
        qsort_descent_inplace(bboxes, 0, bboxes.size() - 1);
    }

    /**
     * @brief 计算两个边界框的交集面积
     */
    static inline float intersection_area(const ArmorBBox& a, const ArmorBBox& b)
    {
        cv::Rect_<float> inter = a.rect & b.rect;
        return inter.area();
    }

    /**
     * @brief NMS非极大值抑制
     * @param faceobjects 候选框列表（需预先按置信度排序）
     * @param picked 输出的保留索引
     * @param nms_threshold NMS阈值
     * 
     * 特点：支持候选框合并，当两个框高度重叠且属性相似时，
     * 会将被抑制框的角点加入保留框的points列表，用于后续平均
     */
    static void nms_sorted_bboxes(
        ArmorBBoxes& faceobjects, 
        std::vector<int>& picked, 
        float nms_threshold)
    {
        picked.clear();
        const int n = faceobjects.size();

        // 预计算所有框的面积
        std::vector<float> areas(n);
        for (int i = 0; i < n; i++) {
            areas[i] = faceobjects[i].rect.area();
        }

        for (int i = 0; i < n; i++) {
            ArmorBBox& a = faceobjects[i];
            int keep = 1;
            
            for (int j = 0; j < (int)picked.size(); j++) {
                ArmorBBox& b = faceobjects[picked[j]];
                
                // 计算IOU
                float inter_area = intersection_area(a, b);
                float union_area = areas[i] + areas[picked[j]] - inter_area;
                float iou = inter_area / union_area;

                if (iou > nms_threshold || std::isnan(iou)) {
                    keep = 0;
                    
                    // 合并候选框（收集角点用于平均）
                    if (iou > MERGE_MIN_IOU &&
                        std::abs(a.confidence - b.confidence) < MERGE_CONF_ERROR &&
                        a.tag_id == b.tag_id &&
                        a.color_id == b.color_id)
                    {
                        for (int k = 0; k < 4; k++) {
                            b.points.push_back(a.corners[k]);
                        }
                    }
                }
            }
            
            if (keep) picked.push_back(i);
        }
    }

    /**
     * @brief 解码模型输出
     * @param prob 模型输出数据
     * @param objects 输出的检测结果
     * @param transform_matrix 坐标变换矩阵
     * @param color_flag 颜色过滤标志
     */
    void decodeOutputs(
        const float* prob, 
        ArmorBBoxes& objects, 
        Eigen::Matrix<float, 3, 3>& transform_matrix, 
        int& color_flag)
    {
        ArmorBBoxes proposals;
        std::vector<int> strides = {8, 16, 32};
        std::vector<GridAndStride> grid_strides;

        // 生成网格信息
        generate_grids_and_stride(INPUT_W, INPUT_H, strides, grid_strides);
        
        // 生成候选框
        generateYoloxProposals(grid_strides, prob, transform_matrix, 
                               BBOX_CONF_THRESH, proposals, color_flag);
        
        // 按置信度排序
        qsort_descent_inplace(proposals);

        // TopK过滤
        if (proposals.size() >= TOPK)
            proposals.resize(TOPK);
        
        // NMS
        std::vector<int> picked;
        nms_sorted_bboxes(proposals, picked, NMS_THRESH);
        
        // 收集结果
        int count = picked.size();
        objects.resize(count);
        for (int i = 0; i < count; i++) {
            objects[i] = proposals[picked[i]];
        }
    }

    /**
     * @brief 检测主函数
     * 
     * 完整检测流程：
     * 1. 图像预处理（缩放填充）
     * 2. 数据格式转换（HWC -> CHW，归一化）
     * 3. OpenVINO推理
     * 4. 解码输出
     * 5. 角点平均（当有合并的候选框时）
     * 6. 数字分类
     */
    ArmorBBoxes ArmorInferer::operator()(cv::Mat& img)
    {
        // 图像预处理
        cv::Mat pre_img = scaledResize(img, transfrom_matrix);

        // 转换为float并分离通道
        cv::Mat pre;
        cv::Mat pre_split[3];
        pre_img.convertTo(pre, CV_32F);  // 不归一化，保持[0,255]
        cv::split(pre, pre_split);

        // 填充输入张量（CHW格式）
        input_tensor = infer_request.get_input_tensor(0);
        infer_request.set_input_tensor(input_tensor);
        float* tensor_data = input_tensor.data<float_t>();
        auto img_offset = INPUT_H * INPUT_W;
        
        for (int c = 0; c < 3; c++) {
            memcpy(tensor_data, pre_split[c].data, INPUT_H * INPUT_W * sizeof(float));
            tensor_data += img_offset;
        }
        
        // 执行推理
        infer_request.infer();
        
        // 获取输出
        ov::Tensor output_tensor = infer_request.get_output_tensor();
        float* output = output_tensor.data<float_t>();

        // 解码输出
        ArmorBBoxes bboxes;
        decodeOutputs(output, bboxes, transfrom_matrix, this->color_flag);

        // 后处理：角点平均和数字分类
        for (auto bbox = bboxes.begin(); bbox != bboxes.end(); ++bbox) {
            // 当有多个合并的候选框时，对角点进行平均
            if ((*bbox).points.size() >= 8) {
                auto N = (*bbox).points.size();
                cv::Point2f pts_final[4] = {
                    cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f),
                    cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f)
                };
                
                for (size_t i = 0; i < N; i++) {
                    pts_final[i % 4] += (*bbox).points[i];
                }

                float count_per_corner = static_cast<float>(N) / 4.0f;
                for (int i = 0; i < 4; i++) {
                    pts_final[i].x = pts_final[i].x / count_per_corner;
                    pts_final[i].y = pts_final[i].y / count_per_corner;
                }
                
                (*bbox).corners[0] = pts_final[0];
                (*bbox).corners[1] = pts_final[1];
                (*bbox).corners[2] = pts_final[2];
                (*bbox).corners[3] = pts_final[3];
                (*bbox).center = ((*bbox).corners[0] + (*bbox).corners[1] + 
                                  (*bbox).corners[2] + (*bbox).corners[3]) / 4;
            }
            else {
                (*bbox).center = ((*bbox).corners[0] + (*bbox).corners[1] + 
                                  (*bbox).corners[2] + (*bbox).corners[3]) / 4;
            }
            
            // 数字分类
            (*bbox).tag_id = number_classifier->classify(
                img, 
                {(*bbox).corners[0], (*bbox).corners[1], 
                 (*bbox).corners[2], (*bbox).corners[3]}
            );
        }
        
        return bboxes;
    }

} // namespace DETECTOR
