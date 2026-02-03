/**
 * @file verify_pnp.cpp
 * @brief 独立验证程序：ONNX模型检测 + PnP解算 + 重投影可视化
 * 
 * 功能：
 * 1. 使用OpenVINO加载0526.onnx模型检测装甲板角点
 * 2. 使用OpenCV solvePnP解算相机坐标系下的位姿
 * 3. 将3D坐标系重投影到图像上验证解算效果
 * 
 * 编译: mkdir build && cd build && cmake .. && make
 * 运行: ./verify_pnp <video_path> <model_path>
 */

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <iomanip>

// ==================== 模型参数 ====================
constexpr int INPUT_W = 640;
constexpr int INPUT_H = 640;
constexpr int OUTPUT_DIM = 22;  // 8corners + 1obj + 4colors + 9classes
constexpr float CONF_THRESHOLD = 0.5f;
constexpr float NMS_THRESHOLD = 0.45f;

// ==================== 装甲板尺寸(米) ====================
constexpr float SMALL_ARMOR_WIDTH = 0.135f;
constexpr float SMALL_ARMOR_HEIGHT = 0.055f;
constexpr float LARGE_ARMOR_WIDTH = 0.225f;
constexpr float LARGE_ARMOR_HEIGHT = 0.055f;

// ==================== 检测结果结构 ====================
struct ArmorDetection {
    std::vector<cv::Point2f> corners;  // 4个角点: 左上、左下、右下、右上
    float confidence;
    int color_id;   // 0=蓝, 1=红
    int class_id;   // 0-8
    bool is_large;  // 是否大装甲板
};

// ==================== 推理性能统计 ====================
struct InferenceProfile {
    double preprocess_ms = 0;   // 预处理（resize）
    double inference_ms = 0;     // 模型推理
    double decode_ms = 0;        // 输出解析
    double nms_ms = 0;           // NMS
    double total_ms = 0;         // 总时间
};

// ==================== PnP结果结构 ====================
struct PnPResult {
    cv::Mat rvec;           // 旋转向量
    cv::Mat tvec;           // 位移向量
    cv::Mat rotation_mat;   // 旋转矩阵
    bool valid;
};

/**
 * @brief OpenVINO模型推理类
 */
class ArmorDetector {
public:
    ArmorDetector(const std::string& model_path) {
        // 初始化OpenVINO
        ov::Core core;
        auto model = core.read_model(model_path);
        
        // 预处理配置
        ov::preprocess::PrePostProcessor ppp(model);
        ppp.input()
            .tensor()
            .set_element_type(ov::element::u8)
            .set_layout("NHWC")
            .set_color_format(ov::preprocess::ColorFormat::BGR);
        ppp.input()
            .preprocess()
            .convert_element_type(ov::element::f32)
            .convert_color(ov::preprocess::ColorFormat::RGB)
            .scale(255.0f);
        ppp.input()
            .model()
            .set_layout("NCHW");
        
        model = ppp.build();
        compiled_model_ = core.compile_model(model, "CPU");
        infer_request_ = compiled_model_.create_infer_request();
        
        std::cout << "[ArmorDetector] 模型加载成功: " << model_path << std::endl;
    }
    
    std::vector<ArmorDetection> detect(const cv::Mat& frame, int color_flag, InferenceProfile& profile) {
        auto t_start = std::chrono::high_resolution_clock::now();
        std::vector<ArmorDetection> results;
        
        // 记录原始尺寸
        int orig_w = frame.cols;
        int orig_h = frame.rows;
        float scale_x = static_cast<float>(orig_w) / INPUT_W;
        float scale_y = static_cast<float>(orig_h) / INPUT_H;
        
        // 1. 预处理：缩放到模型输入尺寸
        auto t1 = std::chrono::high_resolution_clock::now();
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size(INPUT_W, INPUT_H));
        
        // 创建输入tensor
        ov::Tensor input_tensor(ov::element::u8, {1, INPUT_H, INPUT_W, 3}, resized.data);
        infer_request_.set_input_tensor(input_tensor);
        auto t2 = std::chrono::high_resolution_clock::now();
        profile.preprocess_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
        
        // 2. 模型推理
        auto t3 = std::chrono::high_resolution_clock::now();
        infer_request_.infer();
        auto t4 = std::chrono::high_resolution_clock::now();
        profile.inference_ms = std::chrono::duration<double, std::milli>(t4 - t3).count();
        
        // 3. 解析输出
        auto t5 = std::chrono::high_resolution_clock::now();
        auto output = infer_request_.get_output_tensor();
        const float* output_data = output.data<float>();
        auto shape = output.get_shape();
        int num_detections = shape[1];
        
        // 解析输出
        std::vector<ArmorDetection> candidates;
        std::vector<float> scores;
        std::vector<cv::Rect> boxes;
        
        for (int i = 0; i < num_detections; i++) {
            const float* det = output_data + i * OUTPUT_DIM;
            
            float obj_conf = det[8];
            if (obj_conf < CONF_THRESHOLD) continue;
            
            // 解析颜色 (indices 9-12)
            int color_id = 0;
            float max_color = det[9];
            for (int c = 1; c < 4; c++) {
                if (det[9 + c] > max_color) {
                    max_color = det[9 + c];
                    color_id = c;
                }
            }
            
            // 颜色过滤: 0=蓝, 1=红, 2=全部
            if (color_flag == 0 && color_id != 0) continue;
            if (color_flag == 1 && color_id != 1) continue;
            
            // 解析类别 (indices 13-21)
            int class_id = 0;
            float max_class = det[13];
            for (int c = 1; c < 9; c++) {
                if (det[13 + c] > max_class) {
                    max_class = det[13 + c];
                    class_id = c;
                }
            }
            
            // 解析角点 (indices 0-7: x0,y0,x1,y1,x2,y2,x3,y3)
            // 模型输出顺序: 左上、左下、右下、右上
            ArmorDetection armor;
            armor.corners.resize(4);
            for (int j = 0; j < 4; j++) {
                armor.corners[j].x = det[j * 2] * scale_x;
                armor.corners[j].y = det[j * 2 + 1] * scale_y;
            }
            
            armor.confidence = obj_conf;
            armor.color_id = color_id;
            armor.class_id = class_id;
            
            // 判断大小装甲板 (1=hero, 7=outpost -> 大装甲板)
            armor.is_large = (class_id == 1 || class_id == 7);
            
            candidates.push_back(armor);
            scores.push_back(obj_conf);
            
            // 计算bbox用于NMS
            float min_x = std::min({armor.corners[0].x, armor.corners[1].x, 
                                    armor.corners[2].x, armor.corners[3].x});
            float max_x = std::max({armor.corners[0].x, armor.corners[1].x, 
                                    armor.corners[2].x, armor.corners[3].x});
            float min_y = std::min({armor.corners[0].y, armor.corners[1].y, 
                                    armor.corners[2].y, armor.corners[3].y});
            float max_y = std::max({armor.corners[0].y, armor.corners[1].y, 
                                    armor.corners[2].y, armor.corners[3].y});
            boxes.emplace_back(min_x, min_y, max_x - min_x, max_y - min_y);
        }
        auto t6 = std::chrono::high_resolution_clock::now();
        profile.decode_ms = std::chrono::duration<double, std::milli>(t6 - t5).count();
        
        // 4. NMS
        auto t7 = std::chrono::high_resolution_clock::now();
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, scores, CONF_THRESHOLD, NMS_THRESHOLD, indices);
        
        for (int idx : indices) {
            results.push_back(candidates[idx]);
        }
        auto t8 = std::chrono::high_resolution_clock::now();
        profile.nms_ms = std::chrono::duration<double, std::milli>(t8 - t7).count();
        
        // 总时间
        auto t_end = std::chrono::high_resolution_clock::now();
        profile.total_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        
        return results;
    }

private:
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;
};

/**
 * @brief PnP解算器
 */
class PnPSolver {
public:
    PnPSolver(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs)
        : camera_matrix_(camera_matrix), dist_coeffs_(dist_coeffs) {
        
        // 初始化3D点 - 装甲板坐标系原点在中心，X左Y上Z外
        // 角点顺序必须与检测器输出一致: 左上(0)、左下(1)、右下(2)、右上(3)
        float sw = SMALL_ARMOR_WIDTH / 2;
        float sh = SMALL_ARMOR_HEIGHT / 2;
        float lw = LARGE_ARMOR_WIDTH / 2;
        float lh = LARGE_ARMOR_HEIGHT / 2;
        
        // 小装甲板3D点
        points_small_ = {
            cv::Point3f( sw,  sh, 0),  // 左上: X+, Y+
            cv::Point3f( sw, -sh, 0),  // 左下: X+, Y-
            cv::Point3f(-sw, -sh, 0),  // 右下: X-, Y-
            cv::Point3f(-sw,  sh, 0)   // 右上: X-, Y+
        };
        
        // 大装甲板3D点
        points_large_ = {
            cv::Point3f( lw,  lh, 0),  // 左上
            cv::Point3f( lw, -lh, 0),  // 左下
            cv::Point3f(-lw, -lh, 0),  // 右下
            cv::Point3f(-lw,  lh, 0)   // 右上
        };
    }
    
    PnPResult solve(const std::vector<cv::Point2f>& corners, bool is_large) {
        PnPResult result;
        result.valid = false;
        
        const auto& points_3d = is_large ? points_large_ : points_small_;
        
        // PnP解算
        bool success = cv::solvePnP(
            points_3d, corners,
            camera_matrix_, dist_coeffs_,
            result.rvec, result.tvec,
            false, cv::SOLVEPNP_IPPE
        );
        
        if (success) {
            // 旋转向量转旋转矩阵
            cv::Rodrigues(result.rvec, result.rotation_mat);
            
            // 修复Z轴方向歧义：确保Z轴（法向量）始终朝向相机
            // 装甲板坐标系的Z轴是旋转矩阵的第三列
            cv::Vec3d z_axis(result.rotation_mat.at<double>(0, 2),
                             result.rotation_mat.at<double>(1, 2),
                             result.rotation_mat.at<double>(2, 2));
            
            // 位移向量指向装甲板，所以装甲板的法向量应该指向相机（与tvec方向相反）
            // 如果Z轴与tvec方向一致（点积为正），说明法向量朝外，需要翻转
            cv::Vec3d tvec_dir(result.tvec.at<double>(0),
                              result.tvec.at<double>(1),
                              result.tvec.at<double>(2));
            
            double dot = z_axis.dot(tvec_dir);
            if (dot > 0) {
                // 翻转旋转矩阵：Z轴取反，同时X轴也取反以保持右手系
                result.rotation_mat.col(0) *= -1;  // X轴取反
                result.rotation_mat.col(2) *= -1;  // Z轴取反
                
                // 更新旋转向量
                cv::Rodrigues(result.rotation_mat, result.rvec);
            }
            
            result.valid = true;
        }
        
        return result;
    }
    
    const cv::Mat& getCameraMatrix() const { return camera_matrix_; }
    const cv::Mat& getDistCoeffs() const { return dist_coeffs_; }
    const std::vector<cv::Point3f>& getPoints3D(bool is_large) const {
        return is_large ? points_large_ : points_small_;
    }

private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::vector<cv::Point3f> points_small_;
    std::vector<cv::Point3f> points_large_;
};

/**
 * @brief 计算装甲板的像素宽高比
 * @param corners 角点数组 [左上(0), 左下(1), 右下(2), 右上(3)]
 * @return 宽高比 (平均宽度 / 平均高度)
 */
double calculateAspectRatio(const std::vector<cv::Point2f>& corners) {
    // 上边宽度: 左上(0) 到 右上(3) 的水平距离
    double top_width = std::abs(corners[3].x - corners[0].x);
    // 下边宽度: 左下(1) 到 右下(2) 的水平距离
    double bottom_width = std::abs(corners[2].x - corners[1].x);
    // 平均宽度
    double avg_width = (top_width + bottom_width) / 2.0;
    
    // 左边高度: 左上(0) 到 左下(1) 的竖直距离
    double left_height = std::abs(corners[1].y - corners[0].y);
    // 右边高度: 右上(3) 到 右下(2) 的竖直距离
    double right_height = std::abs(corners[2].y - corners[3].y);
    // 平均高度
    double avg_height = (left_height + right_height) / 2.0;
    
    // 避免除以零
    if (avg_height < 1e-6) return -1.0;
    
    return avg_width / avg_height;
}

/**
 * @brief 获取装甲板中心的X坐标（用于判断最左侧）
 */
double getArmorCenterX(const std::vector<cv::Point2f>& corners) {
    return (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0;
}

/**
 * @brief 可视化类
 */
class Visualizer {
public:
    /**
     * @brief 绘制检测结果
     */
    static void drawDetection(cv::Mat& frame, const ArmorDetection& det) {
        // 颜色
        cv::Scalar color = (det.color_id == 0) ? cv::Scalar(255, 128, 0) : cv::Scalar(0, 128, 255);
        
        // 绘制四边形
        for (int i = 0; i < 4; i++) {
            cv::line(frame, det.corners[i], det.corners[(i + 1) % 4], color, 1);
        }
        
        // 绘制角点
        const char* corner_labels[] = {"LT", "LB", "RB", "RT"};
        for (int i = 0; i < 4; i++) {
            cv::circle(frame, det.corners[i], 4, cv::Scalar(0, 255, 0), -1);
            cv::putText(frame, corner_labels[i], det.corners[i] + cv::Point2f(5, -5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        }
        
        // 绘制信息
        cv::Point2f center = (det.corners[0] + det.corners[2]) * 0.5f;
        char info[64];
        snprintf(info, sizeof(info), "%s C%d conf:%.2f", 
                det.is_large ? "L" : "S", det.class_id, det.confidence);
        cv::putText(frame, info, center + cv::Point2f(-40, -20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
    }
    
    /**
     * @brief 绘制PnP解算结果和3D坐标轴
     */
    static void drawPnPResult(cv::Mat& frame, const PnPResult& pnp,
                              const PnPSolver& solver, bool is_large) {
        if (!pnp.valid) return;
        
        // 坐标轴长度（米）
        const float axis_len = 0.1f;
        
        // 定义坐标轴端点（装甲板坐标系）
        std::vector<cv::Point3f> axis_points = {
            cv::Point3f(0, 0, 0),           // 原点
            cv::Point3f(axis_len, 0, 0),    // X轴端点（左）
            cv::Point3f(0, axis_len, 0),    // Y轴端点（上）
            cv::Point3f(0, 0, axis_len)     // Z轴端点（外/朝向相机）
        };
        
        // 投影到图像
        std::vector<cv::Point2f> axis_2d;
        cv::projectPoints(axis_points, pnp.rvec, pnp.tvec,
                         solver.getCameraMatrix(), solver.getDistCoeffs(), axis_2d);
        
        // 绘制坐标轴
        cv::Point origin = axis_2d[0];
        cv::arrowedLine(frame, origin, axis_2d[1], cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.2);  // X: 红
        cv::arrowedLine(frame, origin, axis_2d[2], cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.2);  // Y: 绿
        cv::arrowedLine(frame, origin, axis_2d[3], cv::Scalar(255, 0, 0), 2, cv::LINE_AA, 0, 0.2);  // Z: 蓝
        
        // 标注轴名称
        // cv::putText(frame, "X", axis_2d[1] + cv::Point2f(5, 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
        // cv::putText(frame, "Y", axis_2d[2] + cv::Point2f(5, 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        // cv::putText(frame, "Z", axis_2d[3] + cv::Point2f(5, 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
        
        // 重投影3D角点验证
        const auto& points_3d = solver.getPoints3D(is_large);
        std::vector<cv::Point2f> reproj_corners;
        cv::projectPoints(points_3d, pnp.rvec, pnp.tvec,
                         solver.getCameraMatrix(), solver.getDistCoeffs(), reproj_corners);
        
        // 绘制重投影角点（黄色圆圈）
        for (const auto& pt : reproj_corners) {
            cv::circle(frame, pt, 6, cv::Scalar(0, 255, 255), 2);
        }
        
        // 绘制位移和旋转信息
        double tx = pnp.tvec.at<double>(0);
        double ty = pnp.tvec.at<double>(1);
        double tz = pnp.tvec.at<double>(2);
        double dist = std::sqrt(tx*tx + ty*ty + tz*tz);
        
        char tvec_str[128];
        snprintf(tvec_str, sizeof(tvec_str), "T:[%.3f, %.3f, %.3f] D:%.2fm", tx, ty, tz, dist);
        cv::putText(frame, tvec_str, origin + cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1);
        
        // 旋转矩阵
        char rvec_str[128];
        snprintf(rvec_str, sizeof(rvec_str), "R:[%.2f, %.2f, %.2f]", 
                pnp.rvec.at<double>(0), pnp.rvec.at<double>(1), pnp.rvec.at<double>(2));
        cv::putText(frame, rvec_str, origin + cv::Point(10, 50),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);
    }
    
    /**
     * @brief 绘制信息面板
     */
    static void drawInfoPanel(cv::Mat& frame, int det_count, double fps, double infer_ms, double pnp_ms) {
        int panel_h = 80;
        cv::rectangle(frame, cv::Point(0, 0), cv::Point(300, panel_h), cv::Scalar(0, 0, 0), -1);
        
        char info[128];
        snprintf(info, sizeof(info), "FPS: %.1f", fps);
        cv::putText(frame, info, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        
        snprintf(info, sizeof(info), "Detections: %d", det_count);
        cv::putText(frame, info, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        
        snprintf(info, sizeof(info), "Infer: %.1fms  PnP: %.2fms", infer_ms, pnp_ms);
        cv::putText(frame, info, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
    }
};

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "用法: " << argv[0] << " <video_path> <model_path> [color_flag]" << std::endl;
        std::cout << "  color_flag: 0=蓝色, 1=红色, 2=全部(默认)" << std::endl;
        return 1;
    }
    
    std::string video_path = argv[1];
    std::string model_path = argv[2];
    int color_flag = (argc > 3) ? std::stoi(argv[3]) : 2;
    
    // 打开视频
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        std::cerr << "无法打开视频: " << video_path << std::endl;
        return 1;
    }
    
    int frame_w = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int frame_h = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double video_fps = cap.get(cv::CAP_PROP_FPS);
    std::cout << "视频: " << frame_w << "x" << frame_h << " @ " << video_fps << "fps" << std::endl;
    
    // 相机内参（默认值，根据实际标定结果修改）
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        1280.0,    0.0, frame_w / 2.0,
           0.0, 1024.0, frame_h / 2.0,
           0.0,    0.0,           1.0);

        //      data: [1289.0, 0.0, 638.8,
        //  0.0, 561.0, 517.0,
        //  0.0, 0.0, 1.0]
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    
    // 初始化检测器和PnP解算器
    ArmorDetector detector(model_path);
    PnPSolver pnp_solver(camera_matrix, dist_coeffs);
    
    // 创建输出视频
    std::string output_path = "/home/user/droneAim/TDrone/armor_detector_ros2/verify/verify_output.mp4";
    cv::VideoWriter writer(output_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                          video_fps, cv::Size(frame_w, frame_h));
    
    if (!writer.isOpened()) {
        std::cerr << "警告: 无法创建输出视频文件: " << output_path << std::endl;
        std::cerr << "将仅显示实时画面，不保存视频" << std::endl;
    } else {
        std::cout << "输出视频将保存到: " << output_path << std::endl;
    }
    
    cv::Mat frame;
    int frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();
    
    // 性能统计
    double total_infer_ms = 0;
    double total_pnp_ms = 0;
    double total_draw_ms = 0;
    double total_frame_ms = 0;
    
    // 推理细节统计
    double total_preprocess_ms = 0;
    double total_inference_ms = 0;
    double total_decode_ms = 0;
    double total_nms_ms = 0;
    
    std::cout << "开始处理... 按 'q' 退出, 空格暂停" << std::endl;
    
    // 打开日志文件记录rvec/tvec
    std::ofstream log_file("/home/user/droneAim/TDrone/output/verify_pnp_log.csv");
    log_file << "frame,rvec0,rvec1,rvec2,tvec0,tvec1,tvec2,corners" << std::endl;
    
    // 打开宽高比记录文件
    std::ofstream aspect_ratio_file("/home/user/droneAim/TDrone/output/aspect_ratio_log.csv");
    aspect_ratio_file << "frame,aspect_ratio,avg_width,avg_height,center_x,num_detections" << std::endl;
    
    bool paused = false;
    while (true) {
        auto frame_start = std::chrono::high_resolution_clock::now();
        
        if (!paused) {
            if (!cap.read(frame)) break;
            frame_count++;
        }
        
        // 检测
        InferenceProfile infer_profile;
        auto t1 = std::chrono::high_resolution_clock::now();
        auto detections = detector.detect(frame, color_flag, infer_profile);
        auto t2 = std::chrono::high_resolution_clock::now();
        double infer_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
        
        // PnP解算
        auto t3 = std::chrono::high_resolution_clock::now();
        std::vector<PnPResult> pnp_results;
        for (const auto& det : detections) {
            pnp_results.push_back(pnp_solver.solve(det.corners, det.is_large));
        }
        auto t4 = std::chrono::high_resolution_clock::now();
        double pnp_ms = std::chrono::duration<double, std::milli>(t4 - t3).count();
        
        // 计算并记录宽高比（只保留最左侧装甲板）
        double aspect_ratio = -1.0;
        double recorded_avg_width = 0.0;
        double recorded_avg_height = 0.0;
        double recorded_center_x = 0.0;
        
        if (!detections.empty()) {
            // 找到最左侧的装甲板
            size_t leftmost_idx = 0;
            double min_center_x = getArmorCenterX(detections[0].corners);
            
            for (size_t i = 1; i < detections.size(); i++) {
                double center_x = getArmorCenterX(detections[i].corners);
                if (center_x < min_center_x) {
                    min_center_x = center_x;
                    leftmost_idx = i;
                }
            }
            
            // 计算最左侧装甲板的宽高比
            const auto& corners = detections[leftmost_idx].corners;
            
            double top_width = std::abs(corners[3].x - corners[0].x);
            double bottom_width = std::abs(corners[2].x - corners[1].x);
            recorded_avg_width = (top_width + bottom_width) / 2.0;
            
            double left_height = std::abs(corners[1].y - corners[0].y);
            double right_height = std::abs(corners[2].y - corners[3].y);
            recorded_avg_height = (left_height + right_height) / 2.0;
            
            recorded_center_x = min_center_x;
            
            if (recorded_avg_height > 1e-6) {
                aspect_ratio = recorded_avg_width / recorded_avg_height;
            }
        }
        
        // 记录到文件
        aspect_ratio_file << frame_count << ","
                         << std::fixed << std::setprecision(4)
                         << aspect_ratio << ","
                         << recorded_avg_width << ","
                         << recorded_avg_height << ","
                         << recorded_center_x << ","
                         << detections.size() << std::endl;
        
        // 可视化 - 直接在原图上绘制避免clone
        auto t5 = std::chrono::high_resolution_clock::now();
        for (size_t i = 0; i < detections.size(); i++) {
            Visualizer::drawDetection(frame, detections[i]);
            Visualizer::drawPnPResult(frame, pnp_results[i], pnp_solver, detections[i].is_large);
            
            // 记录rvec/tvec到日志文件
            if (pnp_results[i].valid) {
                log_file << frame_count << ","
                         << std::fixed << std::setprecision(6)
                         << pnp_results[i].rvec.at<double>(0) << ","
                         << pnp_results[i].rvec.at<double>(1) << ","
                         << pnp_results[i].rvec.at<double>(2) << ","
                         << pnp_results[i].tvec.at<double>(0) << ","
                         << pnp_results[i].tvec.at<double>(1) << ","
                         << pnp_results[i].tvec.at<double>(2) << ",\"";
                // 记录角点
                for (int j = 0; j < 4; j++) {
                    log_file << "(" << detections[i].corners[j].x << "," 
                             << detections[i].corners[j].y << ")";
                    if (j < 3) log_file << ";";
                }
                log_file << "\"" << std::endl;
            }
        }
        auto t6 = std::chrono::high_resolution_clock::now();
        double draw_ms = std::chrono::duration<double, std::milli>(t6 - t5).count();
        
        // 计算实时FPS（基于单帧处理时间）
        auto frame_end = std::chrono::high_resolution_clock::now();
        double frame_ms = std::chrono::duration<double, std::milli>(frame_end - frame_start).count();
        double instant_fps = 1000.0 / frame_ms;
        
        // 累加统计数据
        total_infer_ms += infer_ms;
        total_pnp_ms += pnp_ms;
        total_draw_ms += draw_ms;
        total_frame_ms += frame_ms;
        
        // 累加推理细节
        total_preprocess_ms += infer_profile.preprocess_ms;
        total_inference_ms += infer_profile.inference_ms;
        total_decode_ms += infer_profile.decode_ms;
        total_nms_ms += infer_profile.nms_ms;
        
        // 计算平均FPS
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();
        double avg_fps = frame_count / elapsed;
        
        Visualizer::drawInfoPanel(frame, detections.size(), instant_fps, infer_ms, pnp_ms);
        
        // 在右上角显示平均FPS
        char avg_fps_str[64];
        snprintf(avg_fps_str, sizeof(avg_fps_str), "Avg: %.1f fps", avg_fps);
        cv::putText(frame, avg_fps_str, cv::Point(frame.cols - 200, 60),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(100, 255, 100), 2);
        
        // 写入输出视频
        if (writer.isOpened()) {
            writer.write(frame);
        }
        
        // 显示 - 使用最小延迟
        cv::imshow("PnP Verification", frame);
        
        int key = cv::waitKey(paused ? 0 : 1);
        if (key == 'q' || key == 27) break;
        if (key == ' ') paused = !paused;
        if (key == 's' && paused) {
            // 单帧步进
            if (cap.read(frame)) frame_count++;
        }
    }
    
    cap.release();
    writer.release();
    cv::destroyAllWindows();
    
    // 输出性能统计
    std::cout << "\n========== 性能统计 ==========" << std::endl;
    std::cout << "处理完成，共 " << frame_count << " 帧" << std::endl;
    
    if (frame_count > 0) {
        double avg_infer_ms = total_infer_ms / frame_count;
        double avg_pnp_ms = total_pnp_ms / frame_count;
        double avg_draw_ms = total_draw_ms / frame_count;
        double avg_frame_ms = total_frame_ms / frame_count;
        
        auto end_time = std::chrono::steady_clock::now();
        double total_elapsed = std::chrono::duration<double>(end_time - start_time).count();
        double overall_fps = frame_count / total_elapsed;
        
        std::cout << "\n各环节平均耗时:" << std::endl;
        std::cout << "  模型推理总计: " << std::fixed << std::setprecision(2) 
                  << avg_infer_ms << " ms  (" << (avg_infer_ms / avg_frame_ms * 100) << "%)" << std::endl;
        
        // 推理细节
        double avg_preprocess_ms = total_preprocess_ms / frame_count;
        double avg_inference_ms = total_inference_ms / frame_count;
        double avg_decode_ms = total_decode_ms / frame_count;
        double avg_nms_ms = total_nms_ms / frame_count;
        
        std::cout << "    - 预处理:   " << avg_preprocess_ms << " ms  (" 
                  << (avg_preprocess_ms / avg_infer_ms * 100) << "%)" << std::endl;
        std::cout << "    - 模型推理: " << avg_inference_ms << " ms  (" 
                  << (avg_inference_ms / avg_infer_ms * 100) << "%)" << std::endl;
        std::cout << "    - 输出解析: " << avg_decode_ms << " ms  (" 
                  << (avg_decode_ms / avg_infer_ms * 100) << "%)" << std::endl;
        std::cout << "    - NMS:      " << avg_nms_ms << " ms  (" 
                  << (avg_nms_ms / avg_infer_ms * 100) << "%)" << std::endl;
        
        std::cout << "  PnP解算:    " << avg_pnp_ms << " ms  (" 
                  << (avg_pnp_ms / avg_frame_ms * 100) << "%)" << std::endl;
        std::cout << "  可视化绘制: " << avg_draw_ms << " ms  (" 
                  << (avg_draw_ms / avg_frame_ms * 100) << "%)" << std::endl;
        std::cout << "  其他开销:   " << (avg_frame_ms - avg_infer_ms - avg_pnp_ms - avg_draw_ms) 
                  << " ms  (" << ((avg_frame_ms - avg_infer_ms - avg_pnp_ms - avg_draw_ms) / avg_frame_ms * 100) << "%)" << std::endl;
        std::cout << "  单帧总耗时: " << avg_frame_ms << " ms" << std::endl;
        
        std::cout << "\nFPS统计:" << std::endl;
        std::cout << "  理论FPS (基于单帧): " << (1000.0 / avg_frame_ms) << " fps" << std::endl;
        std::cout << "  实际FPS (整体):     " << overall_fps << " fps" << std::endl;
        
        std::cout << "\n总处理时间: " << std::fixed << std::setprecision(1) 
                  << total_elapsed << " 秒" << std::endl;
    }
    
    if (writer.isOpened()) {
        std::cout << "\n输出视频: " << output_path << std::endl;
    }
    std::cout << "============================\n" << std::endl;
    
    return 0;
}
