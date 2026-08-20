#pragma once

#include "Public/PLGUserDefine.hpp"
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>

class ServoController
{
public:
    ServoController();
    ~ServoController();
    
    void start();
    void stop();
    void updateData(const UserAppData &data);

private:
    void run();

    std::thread m_thread;
    std::atomic<bool> m_running;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    int m_ch10_pwm;
    void (*m_servoFunc)(int, int);
    bool m_hasData;
    std::chrono::steady_clock::time_point m_lastFrameTime;
};
