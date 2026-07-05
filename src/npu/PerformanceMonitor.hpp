#pragma once

#include <chrono>
#include <mutex>

class PerformanceMonitor
{
public:
    PerformanceMonitor()
        : m_frameCount(0), m_fps(0.0)
    {
        reset();
    }

    ~PerformanceMonitor() {}

    void reset()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_lastTime = std::chrono::steady_clock::now();
        m_frameCount = 0;
        m_fps = 0.0;
    }

    void tickFrame()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_frameCount++;
        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = currentTime - m_lastTime;
        if (elapsed.count() >= 1.0)
        {
            m_fps = m_frameCount / elapsed.count();
            m_frameCount = 0;
            m_lastTime = currentTime;
        }
    }

    double getFPS()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_fps;
    }

private:
    std::chrono::steady_clock::time_point m_lastTime;
    int m_frameCount;
    double m_fps;
    std::mutex m_mutex;
};
