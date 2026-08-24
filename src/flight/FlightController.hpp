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
            if (data.APMData.APMControllerSpeed)
            {
                data.APMData.APMControllerSpeed(0, 0, 50, 0.0f);
            }

            double currentAlt = 999.0;
            if (data.APMData._NAV_Relative_Pos[2] != nullptr)
            {
                currentAlt = *data.APMData._NAV_Relative_Pos[2];
            }

            // Check if fused Z height is close to ground (less than 3 cm)
            if (currentAlt <= 3.0)
            {
                std::cout << "[FlightController] Ground touchdown confirmed (<3cm). Disarming...\n";
                if (data.APMData.APMControllerDISARM)
                {
                    data.APMData.APMControllerDISARM();
                }
                m_state = STATE_IDLE;
            }
        }
    }

    // Business Logic 2: Receives and processes broadcast packets (Command Notification Separated)
    void processCmd(std::deque<std::vector<uint8_t>> &recvQueue, UserAppData &data)
    {
        while (!recvQueue.empty())
        {
            auto packet = std::move(recvQueue.front());
            recvQueue.pop_front();

            // Print hex representation
            LOG_EXCH << "[FlightController] Recv Broadcast (HEX):";
            for (uint8_t byte : packet)
            {
                LOG_EXCH << " " << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
            }
            LOG_EXCH << std::dec << std::endl;

            // Unified Pre-check Filter: Reject packets unless it is emergency CC 00/01, OR (vehicle is ARMED AND (IDLE or duplicate LAND))
            // Note: _SYS_DISARMFlag == true indicates DISARMED (locked), false indicates ARMED (unlocked).
            bool isDisarmed = (data.APMData._SYS_DISARMFlag == nullptr || *data.APMData._SYS_DISARMFlag);
            if (!(packet.size() >= 2 && packet[0] == 0xCC && (packet[1] == 0x01 || packet[1] == 0x00)) &&
                (isDisarmed ||
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
