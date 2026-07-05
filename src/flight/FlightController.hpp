#pragma once

#include <deque>
#include <vector>
#include <iostream>
#include <iomanip>
#include "Public/PLGUserDefine.hpp"
#include "src/config.hpp"

class FlightController
{
public:
    // Handles the message queue popping, printing, and triggers directly
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

            // Trigger action directly on context callbacks without helper wrappers
            if (packet.size() >= 2 && packet[0] == 0xCC)
            {
                if (packet[1] == 0x01 && data.APMData.APMControllerARM)
                {
                    std::cout << "arm\n";
                    data.APMData.APMControllerARM();
                }
                else if (packet[1] == 0x00 && data.APMData.APMControllerDISARM)
                {
                    std::cout << "disarm\n";
                    data.APMData.APMControllerDISARM();
                }
            }
        }
    }
};
