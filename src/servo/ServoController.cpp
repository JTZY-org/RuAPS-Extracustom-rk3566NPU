#include "ServoController.hpp"
#include <chrono>

ServoController::ServoController()
    : m_running(true), m_pwm_ptr(nullptr), m_servoFunc(nullptr)
{
    m_thread = std::thread(&ServoController::run, this);
}

ServoController::~ServoController()
{
    m_running = false;
    if (m_thread.joinable())
    {
        m_thread.join();
    }
}

void ServoController::updateData(const UserAppData &data)
{
    m_servoFunc = data.APMData.APMControllerServo;
    m_pwm_ptr = data.APMData._RC_Channel_Raw[9];
}

void ServoController::run()
{
    double filtered_val = 1500.0;
    bool first_run = true;
    auto lastTime = std::chrono::steady_clock::now();
    int lastSentPwm = -1;

    while (m_running)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // 30Hz
        
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;

        void (*func)(int, int) = m_servoFunc;
        int* pwm_ptr = m_pwm_ptr;
        if (func && pwm_ptr && m_running)
        {
            int raw_val = *pwm_ptr;
            if (raw_val >= 800 && raw_val <= 2200)
            {
                if (first_run)
                {
                    filtered_val = raw_val;
                    first_run = false;
                }
                else
                {
                    // LPF formula: y(t) = y(t-1) + alpha * (x(t) - y(t-1))
                    // Cutoff frequency fc = 1.0 Hz -> tau = 1 / (2 * pi * fc)
                    double tau = 1.0 / (2.0 * 3.141592653589793 * 1.0);
                    double alpha = dt / (tau + dt);
                    filtered_val = filtered_val + alpha * (raw_val - filtered_val);
                }
                
                int rounded_pwm = static_cast<int>(filtered_val + 0.5);
                if (rounded_pwm != lastSentPwm)
                {
                    func(4, rounded_pwm);
                    lastSentPwm = rounded_pwm;
                }
            }
        }
    }
}
