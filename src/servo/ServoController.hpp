#pragma once

#include "Public/PLGUserDefine.hpp"
#include <thread>
#include <atomic>

class ServoController
{
public:
    ServoController();
    ~ServoController();
    
    void updateData(const UserAppData &data);

private:
    void run();

    std::thread m_thread;
    std::atomic<bool> m_running;
    std::atomic<int*> m_pwm_ptr;
    std::atomic<void (*)(int, int)> m_servoFunc;
};
