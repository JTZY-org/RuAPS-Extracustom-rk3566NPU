#pragma once

#include <deque>
#include <vector>
#include <iostream>
#include <iomanip>
#include "Public/PLGUserDefine.hpp"
#include "src/config.hpp"

#include <chrono>

class FlightController
{
public:
    enum FlightState
    {
        STATE_IDLE,
        STATE_PENDING_ARM,
        STATE_PENDING_DISARM,
        STATE_LANDING
    };

    FlightState m_state = STATE_IDLE;
    std::chrono::steady_clock::time_point m_lowAltStartTime{};

public:
    // Business Logic 1: State Machine execution & Action Implementation
    void updateState(UserAppData &data)
    {
        // 1. Process deferred Actions (Command Implementation Separated)
        if (m_state == STATE_PENDING_ARM)
        {
            std::cout << "[FlightController] Executing Action: ARM\n";
            if (data.APMData.APMControllerARM)
            {
                data.APMData.APMControllerARM();
            }
            m_state = STATE_IDLE; // Back to idle after execution
            return;
        }

        if (m_state == STATE_PENDING_DISARM)
        {
            std::cout << "[FlightController] Executing Action: DISARM\n";
            if (data.APMData.APMControllerDISARM)
            {
                data.APMData.APMControllerDISARM();
            }
            m_state = STATE_IDLE; // Back to idle after execution
            return;
        }

        // 2. Landing Telemetry Verification
        if (m_state == STATE_LANDING)
        {
            // If timer start time is zero, it's the beginning of landing. Trigger physical moving.
            if (m_lowAltStartTime.time_since_epoch().count() == 0)
            {
                std::cout << "[FlightController] Executing Action: START LANDING (Move to Z=0)\n";
                if (data.APMData.APMControllerPosition)
                {
                    data.APMData.APMControllerPosition(0, 0, 0, true);
                }
                m_lowAltStartTime = std::chrono::steady_clock::now();
            }

            double currentAlt = 999.0;
            if (data.APMData._NAV_Relative_Pos[2] != nullptr)
            {
                currentAlt = *data.APMData._NAV_Relative_Pos[2];
            }

            // Check if fused Z height is close to ground (within 15 cm)
            if (std::abs(currentAlt) <= 8.0)
            {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::steady_clock::now() - m_lowAltStartTime)
                                   .count();
                LOG_EXCH << "[FlightController] Landing check. Elapsed: "
                         << elapsed << " ms / 800 ms (Alt: " << currentAlt << " cm)" << std::endl;

                if (elapsed >= 800)
                {
                    std::cout << "[FlightController] Ground touchdown confirmed stably for 800ms. Disarming...\n";
                    if (data.APMData.APMControllerDISARM)
                    {
                        data.APMData.APMControllerDISARM();
                    }
                    m_state = STATE_IDLE;
                    m_lowAltStartTime = {}; // Reset timer
                }
            }
            else
            {
                // Reset timer to current time to prevent pre-mature disarm if alt fluctuates
                m_lowAltStartTime = std::chrono::steady_clock::now();
            }
        }
    }

    // Business Logic 2: Receives and processes broadcast packets (Command Notification Separated)
    void processCmd(UserAppData &data)
    {
        if (data.BoradCastRecv == nullptr)
        {
            return;
        }

        while (!data.BoradCastRecv->empty())
        {
            auto packet = std::move(data.BoradCastRecv->front());
            data.BoradCastRecv->pop_front();

            // Print hex representation
            LOG_EXCH << "[FlightController] Recv Broadcast (HEX):";
            for (uint8_t byte : packet)
            {
                LOG_EXCH << " " << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
            }
            LOG_EXCH << std::dec << std::endl;

            // Unified Pre-check Filter: Reject packets unless it is emergency CC 00/01, OR (vehicle is ARMED AND (IDLE or duplicate LAND))
            if (!(packet.size() >= 2 && packet[0] == 0xCC && (packet[1] == 0x01 || packet[1] == 0x00)) &&
                ((data.APMData._SYS_ARMFlag == nullptr || *data.APMData._SYS_ARMFlag) ||
                 (m_state != STATE_IDLE && !(packet.size() >= 2 && packet[0] == 0xB0 && packet[1] == 0x01))))
            {
                LOG_EXCH << "[FlightController] COMMAND REJECTED - State conflict or disarmed." << std::endl;
                continue;
            }

            // Command Notification / Parsing only (sets pending state, no physical calls here)
            if (packet.size() >= 2 && packet[0] == 0xCC)
            {
                m_state = (packet[1] == 0x01) ? STATE_PENDING_ARM : STATE_PENDING_DISARM;
            }
            else if (packet.size() >= 2 && packet[0] == 0xB0 && packet[1] == 0x01)
            {
                if (m_state != STATE_LANDING)
                {
                    m_state = STATE_LANDING;
                    m_lowAltStartTime = {}; // Reset timer start point to trigger landing initial action
                }
            }
        }
    }
};
