#ifndef YOLO_NPU_H
#define YOLO_NPU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#define YOLO_NPU_API __declspec(dllexport)
#else
#define YOLO_NPU_API __attribute__((visibility("default")))
#endif

/** 与 libyolo_npu.so 同步，用于确认板端是否部署了最新 SO */
#define YOLO_NPU_API_VERSION 3

/** 返回 API 版本字符串，如 "3" */
YOLO_NPU_API const char* yolo_npu_api_version(void);

/**
 * 单个目标检测结果
 */
typedef struct {
    int x1, y1, x2, y2; // 矩形框左上角(x1, y1)和右下角(x2, y2)
    int cls_id;         // 类别ID
    float prob;         // 置信度/概率
} yolo_det_t;

/**
 * 图像识别后的完整信息结构体
 */
typedef struct {
    uint8_t* data;      // 图像数据指针（NV12格式）
    int width;          // 图像宽度
    int height;         // 图像高度
    int count;          // 识别到的目标数量
    yolo_det_t detections[128]; // 识别到的目标列表
} yolo_image_info_t;

/**
 * 初始化推理环境
 * @param model_path 模型文件路径 (.rknn)
 * @param labels_path 标签文件路径 (.txt)
 * @return 推理句柄
 */
YOLO_NPU_API void* yolo_npu_create(const char* model_path, const char* labels_path);

/**
 * 销毁推理环境
 */
YOLO_NPU_API void yolo_npu_destroy(void* handle);

/**
 * 执行推理并获取结构化信息
 * @param out_info 输出的识别信息结构体，包含图像指针、宽高、目标总数及检测详情
 */
YOLO_NPU_API int yolo_npu_detect(void* handle, uint8_t* nv12_data, int width, int height, yolo_image_info_t* out_info);

/**
 * 获取指定索引的目标检测详情
 * @param info 推理结果结构体
 * @param index 目标索引 (0 ~ count-1)
 * @param out_det 输出的目标详情结构体（包含坐标、类别ID、概率）
 * @return 0 成功, -1 失败
 */
YOLO_NPU_API int yolo_npu_get_result(yolo_image_info_t* info, int index, yolo_det_t* out_det);

/**
 * 直接运行缓冲区数据处理
 * @param model_path 模型路径
 * @param data 图像数据缓冲区（NV12/YUV420SP，与 width/height 对应）
 * @param size 缓冲区总大小，须 >= width*height*3/2（单帧），可为多帧连续数据
 * @param width 图像宽度（像素）
 * @param height 图像高度（像素）
 * @param out_path 保存识别结果的路径
 * @param save_flag 是否保存标志 (1: 保存, 0: 不保存)
 * @param append_flag 追加标志 (1: 追加模式, 0: 覆盖模式)
 * @param draw_flag 是否画框标志 (1: 画框, 0: 不画框)
 * @param out_info 输出识别到的结构化信息（可选，可传nullptr）
 * @return 0 成功, -1 失败
 */
YOLO_NPU_API int yolo_npu_run_buffer(const char* model_path, uint8_t* data, size_t size, int width, int height, const char* out_path, int save_flag, int append_flag, int draw_flag, yolo_image_info_t* out_info);

/**
 * 外部画框与标注相关函数
 */
YOLO_NPU_API void yolo_npu_draw_rectangle(uint8_t* nv12_data, int width, int height, int x, int y, int w, int h, unsigned int color, int thickness);
YOLO_NPU_API void yolo_npu_draw_text(uint8_t* nv12_data, int width, int height, const char* text, int x, int y, unsigned int color, int fontsize);
YOLO_NPU_API const char* yolo_npu_coco_cls_to_name(int cls_id);

#ifdef __cplusplus
}
#endif

#endif // YOLO_NPU_H
