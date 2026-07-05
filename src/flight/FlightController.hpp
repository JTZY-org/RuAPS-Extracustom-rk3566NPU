#pragma once

#include "Public/PLGUserDefine.hpp"

class FlightController
{
public:
    void update(ControllerData &apm)
    {
        if (apm._RC_Channel_Raw[7] != nullptr)
        {
            int rc7 = *apm._RC_Channel_Raw[7];
            if (rc7 <= 2050 && rc7 >= 1900)
            {
                if (apm.APMControllerARM)
                {
                    apm.APMControllerARM();
                }
            }
            else
            {
                if (apm.APMControllerDISARM)
                {
                    apm.APMControllerDISARM();
                }
            }
        }
    }
};
