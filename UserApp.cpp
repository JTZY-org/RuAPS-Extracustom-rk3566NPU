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

#include "yolo_npu.h"

#define USERAPP_ENABLE_PRINT_INIT 1
#ifndef USERAPP_ENABLE_PRINT_INIT
#define USERAPP_ENABLE_PRINT_INIT 1
#endif

#define USERAPP_ENABLE_PRINT_EXCHANGE 0
#ifndef USERAPP_ENABLE_PRINT_EXCHANGE
#define USERAPP_ENABLE_PRINT_EXCHANGE 1
#endif

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
    if (USERAPP_ENABLE_PRINT_EXCHNGE) \
    std::cerrA

#ifndef USERAPP_DEFAULT_YOLO_MODEL
#define USERAPP_DEFAULT_YOLO_MODEL "/etc/rknn/yolov8n.rknn"
#endif

#ifndef USERAPP_DEFAULT_YOLO_LABELS
#define USERAPP_DEFAULT_YOLO_LABELS "/etc/rknn/coco_80_labels_list.txt"
#endif

namespace
{
    std::vector<uint8_t> g_frameBuffer;
    int g_frameWidth = 0;
    int g_frameHeight = 0;
    unsigned int g_framePixFormat = 0;

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
} // namespace

extern "C" void UserAppInit(V4L2Tools::V4l2Info vinfo)
{
    APP_INIT_COUT << "[UserApp] init RKNN YOLO APP" << std::endl;

    g_frameWidth = vinfo.ImgWidth;
    g_frameHeight = vinfo.ImgHeight;
    g_framePixFormat = vinfo.PixFormat;

    const size_t frameSize = static_cast<size_t>(g_frameWidth) * static_cast<size_t>(g_frameHeight) * 3 / 2;
    g_frameBuffer.assign(frameSize, 0);

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
    if (frame.data == nullptr || frame.size == 0 || g_frameBuffer.empty())
    {
        return;
    }

    static std::atomic<uint64_t> frameCounter{0};
    const uint64_t currentFrameId = ++frameCounter;

    if (currentFrameId % 10 != 0)
    {
        return;
    }

    if (frame.size < g_frameBuffer.size())
    {
        APP_EXCH_CERR << "[UserApp] Frame is smaller than expected NV12 size: "
                      << frame.size << " < " << g_frameBuffer.size() << std::endl;
        return;
    }

    std::copy(frame.data, frame.data + g_frameBuffer.size(), g_frameBuffer.data());

    std::lock_guard<std::mutex> lock(g_yoloMutex);
    if (g_yoloHandle == nullptr)
    {
        return;
    }

    yolo_image_info_t info;
    std::memset(&info, 0, sizeof(info));
    if (yolo_npu_detect(g_yoloHandle, g_frameBuffer.data(), g_frameWidth, g_frameHeight, &info) != 0)
    {
        APP_EXCH_CERR << "[UserApp] YOLO detect failed at frame " << currentFrameId << std::endl;
        return;
    }

    APP_EXCH_COUT << "[UserApp] frame=" << currentFrameId << " detections=" << info.count << std::endl;

    auto appendUint16 = [](std::vector<uint8_t> &vec, uint16_t val)
    {
        vec.push_back(static_cast<uint8_t>((val >> 0) & 0xFF));
        vec.push_back(static_cast<uint8_t>((val >> 8) & 0xFF));
    };

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
        broadcastData.reserve(15);     // 1 byte header + 14 bytes detection payload (7 fields * 2 bytes)
        broadcastData.push_back(0xFE); // Header

        // Serialize id, type, confidence, and box (x1, y1, x2, y2) as uint16_t
        uint16_t targetId = static_cast<uint16_t>(i + 1);
        uint16_t targetType = static_cast<uint16_t>(det.cls_id);
        uint16_t targetConfidence = static_cast<uint16_t>(det.prob * 100.0f); // 0-100%

        appendUint16(broadcastData, targetId);
        appendUint16(broadcastData, targetType);
        appendUint16(broadcastData, targetConfidence);
        appendUint16(broadcastData, static_cast<uint16_t>(det.x1));
        appendUint16(broadcastData, static_cast<uint16_t>(det.y1));
        appendUint16(broadcastData, static_cast<uint16_t>(det.x2));
        appendUint16(broadcastData, static_cast<uint16_t>(det.y2));

        if (data.pushBoradcastData)
        {
            data.pushBoradcastData(broadcastData);

#if(USERAPP_ENABLE_PRINT_EXCHANGE)
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

__attribute__((destructor)) static void UserAppCleanup()
{
    destroyYolo();
}
