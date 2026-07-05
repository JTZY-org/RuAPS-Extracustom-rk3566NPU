#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <functional>
#include "yolo_npu.h"
#include "PerformanceMonitor.hpp"

class YoloEngine
{
public:
    YoloEngine();
    ~YoloEngine();

    bool initialize(const std::string &modelPath = "", const std::string &labelsPath = "");
    void cleanup();

    bool isBusy() const;

    // Direct NPU trigger with zero-copy forwarding
    bool detectAsync(const uint8_t *frameData, int width, int height, yolo_npu_callback_t callback);

    const std::string &getLabel(int clsId) const;

private:
    std::string getEnvOrDefault(const char *name, const char *fallback);

private:
    void *m_yoloHandle;
    std::vector<std::string> m_labels;
    mutable std::mutex m_mutex;
    std::string m_unknownLabel;

    PerformanceMonitor m_perfMonitor;
    std::chrono::steady_clock::time_point m_lastFpsPrint;
};
