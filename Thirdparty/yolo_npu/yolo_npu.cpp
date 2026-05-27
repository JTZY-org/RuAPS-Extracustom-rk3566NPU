// RKNN YOLOv8 推理库

#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>

#include "yolov8.h"
#include "postprocess.h"
#include "yolo_npu.h"

extern "C" {
#include "image_drawing.h"
#include "image_utils.h"
}

struct YoloNpuHandle {
    rknn_app_context_t rknn_ctx;
};

extern "C" YOLO_NPU_API const char* yolo_npu_api_version(void) {
    return "3";
}

extern "C" YOLO_NPU_API void* yolo_npu_create(const char* model_path, const char* labels_path) {
    if (!model_path) {
        return nullptr;
    }
    if (init_post_process(labels_path) != 0) {
        std::cerr << "yolo_npu_create: 标签文件加载失败，推理仍继续（画框文字可能显示 null）\n";
    }
    auto* h = new YoloNpuHandle();
    std::memset(&h->rknn_ctx, 0, sizeof(h->rknn_ctx));
    if (init_yolov8_model(model_path, &h->rknn_ctx) != 0) {
        std::cerr << "yolo_npu_create: init_yolov8_model 失败\n";
        deinit_post_process();
        delete h;
        return nullptr;
    }
    return h;
}

extern "C" YOLO_NPU_API void yolo_npu_destroy(void* handle) {
    if (!handle) {
        return;
    }
    auto* h = static_cast<YoloNpuHandle*>(handle);
    release_yolov8_model(&h->rknn_ctx);
    deinit_post_process();
    delete h;
}

extern "C" YOLO_NPU_API int yolo_npu_detect(void* handle, uint8_t* nv12_data, int width, int height, yolo_image_info_t* out_info) {
    if (!handle || !nv12_data || width < 2 || height < 2 || !out_info) {
        return -1;
    }
    auto* h = static_cast<YoloNpuHandle*>(handle);
    const uint32_t cap_w = static_cast<uint32_t>(width);
    const uint32_t cap_h = static_cast<uint32_t>(height);
    const size_t frame_bytes = static_cast<size_t>(cap_w) * cap_h * 3 / 2;

    image_buffer_t img {};
    img.width = static_cast<int>(cap_w);
    img.height = static_cast<int>(cap_h);
    img.height_stride = img.height;
    img.format = IMAGE_FORMAT_YUV420SP_NV12;
    img.size = static_cast<int>(frame_bytes);
    img.fd = -1;
    img.virt_addr = nv12_data;
    img.width_stride = static_cast<int>(cap_w);

    object_detect_result_list od_results {};
    if (inference_yolov8_model(&h->rknn_ctx, &img, &od_results) != 0) {
        return -1;
    }

    // 填充结构化信息
    out_info->data = nv12_data;
    out_info->width = width;
    out_info->height = height;
    out_info->count = od_results.count;
    if (out_info->count > 128) out_info->count = 128;

    for (int i = 0; i < out_info->count; i++) {
        object_detect_result* det = &od_results.results[i];
        out_info->detections[i].x1 = det->box.left;
        out_info->detections[i].y1 = det->box.top;
        out_info->detections[i].x2 = det->box.right;
        out_info->detections[i].y2 = det->box.bottom;
        out_info->detections[i].cls_id = det->cls_id;
        out_info->detections[i].prob = det->prop;
    }

    return 0;
}

extern "C" YOLO_NPU_API int yolo_npu_get_result(yolo_image_info_t* info, int index, yolo_det_t* out_det) {
    if (!info || !out_det || index < 0 || index >= info->count) {
        return -1;
    }
    *out_det = info->detections[index];
    return 0;
}

extern "C" YOLO_NPU_API int yolo_npu_run_buffer(const char* model_path, uint8_t* data, size_t size, int width, int height, const char* out_path, int save_flag, int append_flag, int draw_flag, yolo_image_info_t* out_info) {
    if (!model_path || !data || width < 2 || height < 2 || size == 0) {
        std::cerr << "yolo_npu_run_buffer: 参数无效:"
                  << " model_path=" << (model_path ? model_path : "(null)")
                  << " data=" << static_cast<const void*>(data)
                  << " size=" << size
                  << " width=" << width
                  << " height=" << height << "\n";
        if (!model_path) {
            std::cerr << "  -> model_path 为空\n";
        }
        if (!data) {
            std::cerr << "  -> data 为空，请确认 conInYOLOdata.data 已赋值\n";
        }
        if (width < 2 || height < 2) {
            std::cerr << "  -> width/height 未设置或为 0，请确认 conInYOLOdata.width/height\n";
        }
        if (size == 0) {
            std::cerr << "  -> size 为 0\n";
        }
        std::cerr << "  -> 若以上看起来正常，请检查 dlsym 函数指针类型是否与 yolo_npu.h 完全一致(含 draw_flag、size_t)\n";
        return -1;
    }

    if (out_info) {
        std::memset(out_info, 0, sizeof(*out_info));
    }

    void* handle = yolo_npu_create(model_path, nullptr);
    if (!handle) {
        std::cerr << "yolo_npu_run_buffer: yolo_npu_create 失败, model=" << model_path << "\n";
        return -1;
    }

    FILE* fp = nullptr;
    if (save_flag && out_path) {
        fp = std::fopen(out_path, append_flag ? "ab" : "wb");
        if (!fp) {
            std::cerr << "yolo_npu_run_buffer: 无法打开输出文件 " << out_path
                      << " (" << std::strerror(errno) << ")\n";
            yolo_npu_destroy(handle);
            return -1;
        }
    } else if (save_flag) {
        std::cerr << "yolo_npu_run_buffer: save_flag=1 但 out_path 为空\n";
    }

    const size_t frame_bytes = static_cast<size_t>(width) * static_cast<size_t>(height) * 3 / 2;
    if (frame_bytes == 0 || size < frame_bytes) {
        std::cerr << "yolo_npu_run_buffer: 缓冲区过小 size=" << size
                  << " 需要至少 NV12 一帧 " << frame_bytes
                  << " (width=" << width << " height=" << height << ")\n";
        if (fp) {
            std::fclose(fp);
        }
        yolo_npu_destroy(handle);
        return -1;
    }

    const size_t n_frames = size / frame_bytes;
    if (size % frame_bytes != 0) {
        std::cerr << "yolo_npu_run_buffer: 警告 size=" << size
                  << " 不是帧大小 " << frame_bytes << " 的整数倍，仅处理前 " << n_frames << " 帧\n";
    }
    int ret = 0;

    for (size_t i = 0; i < n_frames; i++) {
        uint8_t* frame_ptr = data + i * frame_bytes;
        yolo_image_info_t info;
        if (yolo_npu_detect(handle, frame_ptr, width, height, &info) != 0) {
            ret = -1;
            break;
        }

        // 如果外部需要结构体信息，将最后一帧的结果拷贝出去
        if (out_info) {
            *out_info = info;
        }

        if (draw_flag) {
            image_buffer_t img {};
            img.width = width;
            img.height = height;
            img.height_stride = height;
            img.format = IMAGE_FORMAT_YUV420SP_NV12;
            img.size = static_cast<int>(frame_bytes);
            img.fd = -1;
            img.virt_addr = frame_ptr;
            img.width_stride = width;

            char text[256];
            for (int j = 0; j < info.count; j++) {
                yolo_det_t* det = &info.detections[j];
                draw_rectangle(&img, det->x1, det->y1, det->x2 - det->x1, det->y2 - det->y1, COLOR_BLUE, 2);
                std::snprintf(text, sizeof(text), "%s %.0f%%", coco_cls_to_name(det->cls_id), det->prob * 100.f);
                draw_text(&img, text, det->x1, det->y1 - 12, COLOR_RED, 10);
            }
        }

        if (save_flag && fp) {
            if (std::fwrite(frame_ptr, 1, frame_bytes, fp) != frame_bytes) {
                ret = -1;
                break;
            }
        }
    }

    if (fp) std::fclose(fp);
    yolo_npu_destroy(handle);
    return ret;
}
