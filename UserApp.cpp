#include "UserApp.hpp"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>
#include <chrono>

#include "yolo_npu.h"
#include "im2d.hpp"

#define USERAPP_ENABLE_PRINT_INIT 1
#ifndef USERAPP_ENABLE_PRINT_INIT
#define USERAPP_ENABLE_PRINT_INIT 1
#endif

#define USERAPP_ENABLE_PRINT_EXCHANGE 0
#ifndef USERAPP_ENABLE_PRINT_EXCHANGE
#define USERAPP_ENABLE_PRINT_EXCHANGE 1
#endif

#define USERAPP_ENABLE_RGA_ROTATION 1

#define APP_INIT_COUT              \
     if (USERAPP_ENABLE_PRINT_INIT) \
     std::cout
#define APP_INIT_CERR              \
     if (USERAPP_ENABLE_PRINT_INIT) \
     std::cerr

#define APP_EXCH_COUT                  \
     if (USERAPP_ENABLE_PRINT_EXCHANGE) \
     std::cout
#define APP_EXCH_CERR                  \
     if (USERAPP_ENABLE_PRINT_EXCHANGE) \
     std::cerr

#ifndef USERAPP_DEFAULT_YOLO_MODEL
#define USERAPP_DEFAULT_YOLO_MODEL "/etc/rknn/yolov8n.rknn"
#endif

#ifndef USERAPP_DEFAULT_YOLO_LABELS
#define USERAPP_DEFAULT_YOLO_LABELS "/etc/rknn/coco_80_labels_list.txt"
#endif

namespace
{
    uint8_t *g_frameBuffer = nullptr;
    size_t g_frameBufferSize = 0;
    int g_frameWidth = 0;
    int g_frameHeight = 0;
    unsigned int g_framePixFormat = 0;
    rga_buffer_handle_t g_dstRgaHandle = 0;

    void *g_yoloHandle = nullptr;
    std::vector<std::string> g_labels;
    std::mutex g_yoloMutex;

    std::string getEnvOrDefault(const char *name, const char *fallback)
    {
        const char *value = std::getenv(name);
        if (value != nullptr && value[0] != '\0')
        {
            return value;
        }
        return fallback;
    }

    std::vector<std::string> loadLabels(const std::string &labelsPath)
    {
        std::vector<std::string> labels;
        std::ifstream infile(labelsPath);
        std::string line;
        while (std::getline(infile, line))
        {
            if (!line.empty() && line.back() == '\r')
            {
                line.pop_back();
            }
            labels.push_back(line);
        }
        return labels;
    }

    const char *labelName(int clsId)
    {
        if (clsId >= 0 && static_cast<size_t>(clsId) < g_labels.size() && !g_labels[clsId].empty())
        {
            return g_labels[clsId].c_str();
        }
        return "unknown";
    }

    void destroyYolo()
    {
        std::lock_guard<std::mutex> lock(g_yoloMutex);
        if (g_yoloHandle != nullptr)
        {
            yolo_npu_destroy(g_yoloHandle);
            g_yoloHandle = nullptr;
        }
    }

    bool g_exiting = false;

    void exit_handler()
    {
        g_exiting = true;
        destroyYolo();
    }
} // namespace

extern "C" void UserAppInit(V4L2Tools::V4l2Info vinfo)
{
    APP_INIT_COUT << "[UserApp] init RKNN YOLO APP" << std::endl;

    static std::once_flag s_atexit_flag;
    std::call_once(s_atexit_flag, []() {
        std::atexit(exit_handler);
    });

    g_frameWidth = vinfo.ImgWidth;
    g_frameHeight = vinfo.ImgHeight;
    g_framePixFormat = vinfo.PixFormat;
    const size_t frameSize = static_cast<size_t>(g_frameWidth) * static_cast<size_t>(g_frameHeight) * 3 / 2;
#if USERAPP_ENABLE_RGA_ROTATION
    if (g_dstRgaHandle != 0)
    {
        releasebuffer_handle(g_dstRgaHandle);
        g_dstRgaHandle = 0;
    }
#endif
    if (g_frameBuffer != nullptr)
    {
        free(g_frameBuffer);
    }
    g_frameBufferSize = frameSize;
    if (posix_memalign((void **)&g_frameBuffer, 4096, g_frameBufferSize) != 0)
    {
        g_frameBuffer = nullptr;
        g_frameBufferSize = 0;
    }
#if USERAPP_ENABLE_RGA_ROTATION
    else
    {
        g_dstRgaHandle = importbuffer_virtualaddr(g_frameBuffer, g_frameBufferSize);
    }
#endif

    APP_INIT_COUT << "[UserApp] RKNN input buffer: "
                  << frameSize << " bytes, " << g_frameWidth << "x" << g_frameHeight << std::endl;

    if (g_framePixFormat != V4L2_PIX_FMT_NV12)
    {
        APP_INIT_COUT << "[UserApp] Warning: camera pixfmt is not V4L2_PIX_FMT_NV12; YOLO expects NV12 input" << std::endl;
    }

    const std::string modelPath = getEnvOrDefault("YOLO_NPU_MODEL", USERAPP_DEFAULT_YOLO_MODEL);
    const std::string labelsPath = getEnvOrDefault("YOLO_NPU_LABELS", USERAPP_DEFAULT_YOLO_LABELS);

    g_labels = loadLabels(labelsPath);
    if (g_labels.empty())
    {
        APP_INIT_COUT << "[UserApp] Warning: labels not loaded from " << labelsPath << std::endl;
    }

    destroyYolo();
    {
        std::lock_guard<std::mutex> lock(g_yoloMutex);
        g_yoloHandle = yolo_npu_create(modelPath.c_str(), labelsPath.c_str());
    }

    if (g_yoloHandle == nullptr)
    {
        APP_INIT_CERR << "[UserApp] Failed to init YOLO_NPU, model=" << modelPath << std::endl;
    }
    else
    {
        APP_INIT_COUT << "[UserApp] YOLO_NPU ready, api=" << yolo_npu_api_version()
                      << ", model=" << modelPath << std::endl;
    }
}

extern "C" void UserAppExChange(UserAppData data)
{
    const auto &frame = data.cameraFrame;
    if (frame.data != nullptr && frame.size >= g_frameBufferSize && g_frameBuffer != nullptr && g_yoloHandle != nullptr)
    {
        if (!yolo_npu_is_busy(g_yoloHandle))
        {
#if USERAPP_ENABLE_RGA_ROTATION
            // Rotate the NV12 input frame by 180 degrees using hardware RGA
            rga_buffer_handle_t src_handle = importbuffer_virtualaddr(const_cast<uint8_t *>(frame.data), g_frameBufferSize);
            if (src_handle != 0 && g_dstRgaHandle != 0)
            {
                rga_buffer_t src_img = wrapbuffer_handle(src_handle, g_frameWidth, g_frameHeight, RK_FORMAT_YCbCr_420_SP);
                rga_buffer_t dst_img = wrapbuffer_handle(g_dstRgaHandle, g_frameWidth, g_frameHeight, RK_FORMAT_YCbCr_420_SP);

                IM_STATUS ret = imrotate(src_img, dst_img, IM_HAL_TRANSFORM_ROT_180);
                if (ret != IM_STATUS_SUCCESS)
                {
                    APP_EXCH_CERR << "[UserApp] RGA imrotate failed: " << ret << std::endl;
                }
                releasebuffer_handle(src_handle);
            }
            else
            {
                APP_EXCH_CERR << "[UserApp] Failed to import src_handle or dst_handle was null!" << std::endl;
            }
#else
            std::copy(frame.data, frame.data + g_frameBufferSize, g_frameBuffer);
#endif

            yolo_npu_detect_async(g_yoloHandle, g_frameBuffer, g_frameWidth, g_frameHeight, [pushCallback = data.pushBoradcastData](const yolo_image_info_t& info) {
                auto appendUint16 = [](std::vector<uint8_t> &vec, uint16_t val)
                {
                    vec.push_back(static_cast<uint8_t>((val >> 0) & 0xFF));
                    vec.push_back(static_cast<uint8_t>((val >> 8) & 0xFF));
                };

                // Calculate and print FPS every 1.0 second
                static auto lastTime = std::chrono::steady_clock::now();
                static int frameCount = 0;
                static std::mutex fpsMutex;
                {
                    std::lock_guard<std::mutex> fpsLock(fpsMutex);
                    frameCount++;
                    auto currentTime = std::chrono::steady_clock::now();
                    std::chrono::duration<double> elapsed = currentTime - lastTime;
                    if (elapsed.count() >= 1.0)
                    {
                        double fps = frameCount / elapsed.count();
                        APP_EXCH_COUT << "[UserApp] Total System FPS (Dual-Context Zero-Copy): " << fps << std::endl;
                        frameCount = 0;
                        lastTime = currentTime;
                    }
                }

                for (int i = 0; i < info.count; ++i)
                {
                    const yolo_det_t &det = info.detections[i];
                    APP_EXCH_COUT << "[UserApp] det[" << i << "]"
                                  << " cls_id=" << det.cls_id
                                  << " type=" << labelName(det.cls_id)
                                  << " prob=" << det.prob
                                  << " box=(" << det.x1 << "," << det.y1 << ")-(" << det.x2 << "," << det.y2 << ")"
                                  << std::endl;

                    std::vector<uint8_t> broadcastData;
                    broadcastData.reserve(15);     // 1 byte header + 14 bytes detection payload
                    broadcastData.push_back(0xFE); // Header

                    uint16_t targetId = static_cast<uint16_t>(i + 1);
                    uint16_t targetType = static_cast<uint16_t>(det.cls_id);
                    uint16_t targetConfidence = static_cast<uint16_t>(det.prob * 100.0f);

                    appendUint16(broadcastData, targetId);
                    appendUint16(broadcastData, targetType);
                    appendUint16(broadcastData, targetConfidence);
                    appendUint16(broadcastData, static_cast<uint16_t>(det.x1));
                    appendUint16(broadcastData, static_cast<uint16_t>(det.y1));
                    appendUint16(broadcastData, static_cast<uint16_t>(det.x2));
                    appendUint16(broadcastData, static_cast<uint16_t>(det.y2));

                    if (pushCallback)
                    {
                        pushCallback(broadcastData);
                    }
                }
            });
        }
    }
    else if (frame.data != nullptr && frame.size < g_frameBufferSize)
    {
        APP_EXCH_CERR << "[UserApp] Frame is smaller than expected NV12 size: "
                      << frame.size << " < " << g_frameBufferSize << std::endl;
    }

    // APM control (executed synchronously on the caller thread, takes <1ms)
    {
        if (*data.APMData._RC_Channel_Raw[7] <= 2050 && *data.APMData._RC_Channel_Raw[7] >= 1900)
        {
            data.APMData.APMControllerARM();
        }
        else
        {
            data.APMData.APMControllerDISARM();
        }
    }
}

__attribute__((destructor)) static void UserAppCleanup()
{
    if (!g_exiting)
    {
        destroyYolo();
    }
#if USERAPP_ENABLE_RGA_ROTATION
    if (g_dstRgaHandle != 0)
    {
        releasebuffer_handle(g_dstRgaHandle);
        g_dstRgaHandle = 0;
    }
#endif
    if (g_frameBuffer != nullptr)
    {
        free(g_frameBuffer);
        g_frameBuffer = nullptr;
        g_frameBufferSize = 0;
    }
}
