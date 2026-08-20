#include "ServoController.hpp"

ServoController::ServoController() 
    : m_running(false)
    , m_ch10_pwm(1500)
    , m_servoFunc(nullptr)
    , m_hasData(false) 
{
}

ServoController::~ServoController()
{
    stop();
}

void ServoController::start()
{
    if (m_running) return;
    m_running = true;
    m_lastFrameTime = std::chrono::steady_clock::now();
    m_thread = std::thread(&ServoController::run, this);
}

void ServoController::stop()
{
    if (!m_running) return;
    m_running = false;
    m_cv.notify_all();
    if (m_thread.joinable())
    {
        m_thread.join();
    }
}

void ServoController::updateData(const UserAppData &data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_servoFunc = data.APMData.APMControllerServo;
    if (data.APMData._RC_Channel_Raw[9] != nullptr)
    {
        m_ch10_pwm = *data.APMData._RC_Channel_Raw[9];
        m_hasData = true;
    }
    m_lastFrameTime = std::chrono::steady_clock::now();
}

void ServoController::run()
{
    while (m_running)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cv.wait_for(lock, std::chrono::milliseconds(20), [this]() { return !m_running; });
        
        if (!m_running)
        {
            break;
        }
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastFrameTime).count();
        if (elapsed > 200)
        {
            continue;
        }
        
        void (*servoFunc)(int, int) = m_servoFunc;
        int pwm = m_ch10_pwm;
        bool hasData = m_hasData;
        lock.unlock();
        
        if (hasData && servoFunc != nullptr)
        {
            if (pwm >= 800 && pwm <= 2200)
            {
                servoFunc(4, pwm);
            }
        }
    }
}
