#include "YoloEngine.hpp"
#include "src/config.hpp"
#include "src/image/RgaProcessor.hpp"
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <utility>

YoloEngine::YoloEngine()
    : m_yoloHandle(nullptr), m_unknownLabel("unknown")
{
    m_lastFpsPrint = std::chrono::steady_clock::now();
}

YoloEngine::~YoloEngine()
{
    cleanup();
}

std::string YoloEngine::getEnvOrDefault(const char *name, const char *fallback)
{
    const char *value = std::getenv(name);
    if (value != nullptr && value[0] != '\0')
    {
        return value;
    }
    return fallback;
}

bool YoloEngine::initialize(const std::string &modelPath, const std::string &labelsPath)
{
    cleanup();

    std::string finalModelPath = modelPath;
    std::string finalLabelsPath = labelsPath;

    if (finalModelPath.empty())
    {
        finalModelPath = getEnvOrDefault("YOLO_NPU_MODEL", config::DEFAULT_YOLO_MODEL);
    }
    if (finalLabelsPath.empty())
    {
        finalLabelsPath = getEnvOrDefault("YOLO_NPU_LABELS", config::DEFAULT_YOLO_LABELS);
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    std::ifstream infile(finalLabelsPath);
    if (!infile.is_open())
    {
        LOG_INIT_ERR << "[YoloEngine] Failed to open labels file: " << finalLabelsPath << std::endl;
    }
    else
    {
        std::string line;
        while (std::getline(infile, line))
        {
            if (!line.empty() && line.back() == '\r')
            {
                line.pop_back();
            }
            m_labels.push_back(line);
        }
    }

    m_yoloHandle = yolo_npu_create(finalModelPath.c_str(), finalLabelsPath.c_str());
    if (m_yoloHandle == nullptr)
    {
        LOG_INIT_ERR << "[YoloEngine] Failed to initialize YOLO NPU model: " << finalModelPath << std::endl;
        return false;
    }

    LOG_INIT << "[YoloEngine] Loaded model: " << finalModelPath << ", API: " << yolo_npu_api_version() << std::endl;
    m_perfMonitor.reset();
    return true;
}

void YoloEngine::cleanup()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_yoloHandle != nullptr)
    {
        yolo_npu_destroy(m_yoloHandle);
        m_yoloHandle = nullptr;
    }
    m_labels.clear();
}

bool YoloEngine::isBusy() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_yoloHandle == nullptr)
    {
        return true;
    }
    return yolo_npu_is_busy(m_yoloHandle);
}

bool YoloEngine::detectAsync(const uint8_t *frameData, int width, int height, yolo_npu_callback_t callback)
{
    if (frameData == nullptr)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_yoloHandle == nullptr || yolo_npu_is_busy(m_yoloHandle))
    {
        return false;
    }

    if (config::ENABLE_PRINT_EXCHANGE)
    {
        return yolo_npu_detect_async(m_yoloHandle, const_cast<uint8_t *>(frameData), width, height, [this, cb = std::move(callback)](const yolo_image_info_t &info)
                                     {
            m_perfMonitor.tickFrame();
            
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - m_lastFpsPrint).count() >= 1)
            {
                LOG_EXCH << "[YoloEngine] FPS: " << m_perfMonitor.getFPS() << std::endl;
                m_lastFpsPrint = now;
            }

            if (cb)
            {
                cb(info);
            } });
    }
    else
    {
        return yolo_npu_detect_async(m_yoloHandle, const_cast<uint8_t *>(frameData), width, height, std::move(callback));
    }
}

const std::string &YoloEngine::getLabel(int clsId) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (clsId >= 0 && static_cast<size_t>(clsId) < m_labels.size() && !m_labels[clsId].empty())
    {
        return m_labels[clsId];
    }
    return m_unknownLabel;
}
