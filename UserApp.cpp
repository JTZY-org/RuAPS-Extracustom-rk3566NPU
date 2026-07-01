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
#include <thread>
#include <condition_variable>

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
    // Triple buffering for NV12 frames (2 for active workers, 1 for active writing)
    uint8_t *g_frameBuffers[3] = {nullptr, nullptr, nullptr};
    size_t g_frameBufferSize = 0;
    int g_frameWidth = 0;
    int g_frameHeight = 0;
    unsigned int g_framePixFormat = 0;
    rga_buffer_handle_t g_dstRgaHandles[3] = {0, 0, 0};

    // Buffer state management
    int g_writeBufferIdx = 2;

    // Dual Context State
    void *g_yoloHandles[2] = {nullptr, nullptr};
    bool g_workerBusy[2] = {false, false};
    int g_workerBufferIdx[2] = {0, 1};
    std::mutex g_workerMutex[2];
    std::condition_variable g_workerCv[2];
    std::thread g_workerThreads[2];
    std::atomic<bool> g_workerRunning[2] = {false, false};
    std::function<void(std::vector<uint8_t>)> g_workerPushCallback[2];
    std::mutex g_schedulerMtx;

    std::vector<std::string> g_labels;
    std::mutex g_globalMutex;

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
        std::lock_guard<std::mutex> lock(g_globalMutex);
        for (int i = 0; i < 2; i++)
        {
            if (g_yoloHandles[i] != nullptr)
            {
                yolo_npu_destroy(g_yoloHandles[i]);
                g_yoloHandles[i] = nullptr;
            }
        }
    }

    void workerThreadFunc(int id)
    {
        auto appendUint16 = [](std::vector<uint8_t> &vec, uint16_t val)
        {
            vec.push_back(static_cast<uint8_t>((val >> 0) & 0xFF));
            vec.push_back(static_cast<uint8_t>((val >> 8) & 0xFF));
        };

        while (g_workerRunning[id])
        {
            // Block until worker is marked busy
            {
                std::unique_lock<std::mutex> lock(g_workerMutex[id]);
                g_workerCv[id].wait(lock, [id] { return g_workerBusy[id] || !g_workerRunning[id]; });
                if (!g_workerRunning[id])
                {
                    break;
                }
            }

            int bufIdx = -1;
            std::function<void(std::vector<uint8_t>)> pushCallback;

            {
                std::lock_guard<std::mutex> lock(g_schedulerMtx);
                bufIdx = g_workerBufferIdx[id];
                pushCallback = g_workerPushCallback[id];
            }

            if (bufIdx != -1 && g_yoloHandles[id] != nullptr)
            {
                yolo_image_info_t info;
                std::memset(&info, 0, sizeof(info));

                // Running inference using the thread's own independent context.
                // In Zero-Copy mode, this is completely thread-safe because memory pools are separate.
                if (yolo_npu_detect(g_yoloHandles[id], g_frameBuffers[bufIdx], g_frameWidth, g_frameHeight, &info) == 0)
                {
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
#if (USERAPP_ENABLE_PRINT_EXCHANGE)
                            {
                                std::string hexStr;
                                char hexBuf[16];
                                for (uint8_t b : broadcastData)
                                {
                                    snprintf(hexBuf, sizeof(hexBuf), "%02X", b);
                                    hexStr += hexBuf;
                                }
                                std::cout << "[UserApp] Sent broadcast data (Hex): " << hexStr << std::endl;
                            }
#endif
                        }
                    }
                }
                else
                {
                    APP_EXCH_CERR << "[UserApp] Worker " << id << " YOLO detect failed" << std::endl;
                }
            }

            // Mark worker as idle under scheduler mutex
            {
                std::lock_guard<std::mutex> lock(g_schedulerMtx);
                g_workerBusy[id] = false;
            }
        }
    }
} // namespace

extern "C" void UserAppInit(V4L2Tools::V4l2Info vinfo)
{
    APP_INIT_COUT << "[UserApp] init RKNN YOLO APP with Dual-Context Zero-Copy Pipeline" << std::endl;

    g_frameWidth = vinfo.ImgWidth;
    g_frameHeight = vinfo.ImgHeight;
    g_framePixFormat = vinfo.PixFormat;
    const size_t frameSize = static_cast<size_t>(g_frameWidth) * static_cast<size_t>(g_frameHeight) * 3 / 2;

    // Stop workers if running
    for (int i = 0; i < 2; i++)
    {
        if (g_workerRunning[i])
        {
            g_workerRunning[i] = false;
            g_workerCv[i].notify_all();
            if (g_workerThreads[i].joinable())
            {
                g_workerThreads[i].join();
            }
        }
    }

#if USERAPP_ENABLE_RGA_ROTATION
    for (int i = 0; i < 3; i++)
    {
        if (g_dstRgaHandles[i] != 0)
        {
            releasebuffer_handle(g_dstRgaHandles[i]);
            g_dstRgaHandles[i] = 0;
        }
    }
#endif

    for (int i = 0; i < 3; i++)
    {
        if (g_frameBuffers[i] != nullptr)
        {
            free(g_frameBuffers[i]);
            g_frameBuffers[i] = nullptr;
        }
    }

    g_frameBufferSize = frameSize;
    bool memAllocSuccess = true;
    for (int i = 0; i < 3; i++)
    {
        if (posix_memalign((void **)&g_frameBuffers[i], 4096, g_frameBufferSize) != 0)
        {
            g_frameBuffers[i] = nullptr;
            memAllocSuccess = false;
        }
#if USERAPP_ENABLE_RGA_ROTATION
        else
        {
            g_dstRgaHandles[i] = importbuffer_virtualaddr(g_frameBuffers[i], g_frameBufferSize);
        }
#endif
    }

    if (!memAllocSuccess)
    {
        g_frameBufferSize = 0;
        APP_INIT_CERR << "[UserApp] Failed to allocate triple buffers!" << std::endl;
        return;
    }

    APP_INIT_COUT << "[UserApp] RKNN triple input buffers allocated: "
                  << frameSize << " bytes each, " << g_frameWidth << "x" << g_frameHeight << std::endl;

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
        std::lock_guard<std::mutex> lock(g_globalMutex);
        g_yoloHandles[0] = yolo_npu_create(modelPath.c_str(), labelsPath.c_str());
        g_yoloHandles[1] = yolo_npu_create(modelPath.c_str(), labelsPath.c_str());
    }

    if (g_yoloHandles[0] == nullptr || g_yoloHandles[1] == nullptr)
    {
        APP_INIT_CERR << "[UserApp] Failed to init Dual YOLO_NPU Contexts" << std::endl;
    }
    else
    {
        APP_INIT_COUT << "[UserApp] Dual YOLO_NPU ready, model=" << modelPath << std::endl;
        
        // Start background workers
        g_writeBufferIdx = 2;
        g_workerBufferIdx[0] = 0;
        g_workerBufferIdx[1] = 1;
        for (int i = 0; i < 2; i++)
        {
            g_workerBusy[i] = false;
            g_workerRunning[i] = true;
            g_workerThreads[i] = std::thread(workerThreadFunc, i);
        }
    }
}

extern "C" void UserAppExChange(UserAppData data)
{
    const auto &frame = data.cameraFrame;
    if (frame.data == nullptr || frame.size == 0 || g_frameBuffers[0] == nullptr || g_frameBuffers[1] == nullptr || g_frameBuffers[2] == nullptr)
    {
        return;
    }

    if (frame.size < g_frameBufferSize)
    {
        APP_EXCH_CERR << "[UserApp] Frame is smaller than expected NV12 size: "
                      << frame.size << " < " << g_frameBufferSize << std::endl;
        return;
    }

    // 1. Find an idle worker
    int targetWorker = -1;
    {
        std::lock_guard<std::mutex> lock(g_schedulerMtx);
        for (int i = 0; i < 2; i++)
        {
            if (!g_workerBusy[i])
            {
                targetWorker = i;
                g_workerBusy[i] = true; // Mark as busy immediately
                break;
            }
        }
    }

    // If both workers are busy, we skip this frame (NPU is fully saturated at 100%)
    if (targetWorker == -1)
    {
        goto apm_control;
    }

    // 2. Quick copy or rotate the input frame into the active write buffer
    {
#if USERAPP_ENABLE_RGA_ROTATION
        rga_buffer_handle_t src_handle = importbuffer_virtualaddr(const_cast<uint8_t *>(frame.data), g_frameBufferSize);
        if (src_handle != 0 && g_dstRgaHandles[g_writeBufferIdx] != 0)
        {
            rga_buffer_t src_img = wrapbuffer_handle(src_handle, g_frameWidth, g_frameHeight, RK_FORMAT_YCbCr_420_SP);
            rga_buffer_t dst_img = wrapbuffer_handle(g_dstRgaHandles[g_writeBufferIdx], g_frameWidth, g_frameHeight, RK_FORMAT_YCbCr_420_SP);

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
        std::copy(frame.data, frame.data + g_frameBufferSize, g_frameBuffers[g_writeBufferIdx]);
#endif
    }

    // 3. Swap the buffers and wake up the target worker
    {
        std::lock_guard<std::mutex> lock(g_schedulerMtx);
        std::swap(g_writeBufferIdx, g_workerBufferIdx[targetWorker]);
        g_workerPushCallback[targetWorker] = data.pushBoradcastData;
    }
    
    {
        std::lock_guard<std::mutex> lock(g_workerMutex[targetWorker]);
        // Nothing to change, just wake up
    }
    g_workerCv[targetWorker].notify_one();

apm_control:
    // 4. APM control (executed synchronously on the caller thread, takes <1ms)
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
    // Stop workers
    for (int i = 0; i < 2; i++)
    {
        if (g_workerRunning[i])
        {
            g_workerRunning[i] = false;
            g_workerCv[i].notify_all();
            if (g_workerThreads[i].joinable())
            {
                g_workerThreads[i].join();
            }
        }
    }

    destroyYolo();

#if USERAPP_ENABLE_RGA_ROTATION
    for (int i = 0; i < 3; i++)
    {
        if (g_dstRgaHandles[i] != 0)
        {
            releasebuffer_handle(g_dstRgaHandles[i]);
            g_dstRgaHandles[i] = 0;
        }
    }
#endif

    for (int i = 0; i < 3; i++)
    {
        if (g_frameBuffers[i] != nullptr)
        {
            free(g_frameBuffers[i]);
            g_frameBuffers[i] = nullptr;
        }
    }
    g_frameBufferSize = 0;
}
