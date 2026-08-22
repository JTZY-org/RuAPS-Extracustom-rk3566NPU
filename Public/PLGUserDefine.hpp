#pragma once
#include <functional>
#include <vector>
#include <deque>
#include "V4L2ToolKit.hpp"

#include <vector>
#include <mutex>
#include <stdexcept>
#include <optional>

struct ControllerData
{
    // bool isAPMDown;

    int *_Accel_ClippedTimes;
    float *_Accel_Accelration[3];
    float *_Accel_VIBE[3];
    int *_Accel_RawG[3];
    float *_Accel_GForce;

    float *_ATT_Quaterion[4];
    float *_ATT_EulerAngle[3];
    float *_Gyro_AngleRate[3];
    int *_Mag_RawL[3];
    float *_ATT_EulerAngleYawV;

    float *_Baro_Temp;
    float *_Baro_PressureHPA;
    float *_Baro_AGLAltitudeCM;
    double *_RangeFinder_AGLAltCM;

    bool *_SYS_ARMFlag; // false: armed (unlocked), true: disarmed (locked)
    int *_SYS_TimeInfo[10];
    uint16_t *_SYS_PreARMFlag;
    uint16_t *_SYS_FailSafeFlag;
    int *_SYS_APMStatus;
    // No Flightmode Provided

    double *_NAV_Speed[3];
    double *_NAV_Global_Speed[2];
    float *_NAV_Relative_Head;
    int *_NAV_Global_Pos[3];
    int *_NAV_Global_HOME[2];
    double *_NAV_Relative_Pos[3];
    int *_NAV_Global_SATCount;
    float *_NAV_Global_Head;
    int *_NAV_GPS_HDOP;

    int *_SEN_FLOW_Quality;

    int *_RC_Channel_Raw[16];
    int *_EF_Channel_Raw[16];

    float *_GYRO_CYCLE_TIME;
    float *_Battery_Voltage;
    float *_Battery_Voltage_Single;

    double *_CPU_Core_Temp;

    void (*APMControllerARM)(void);
    void (*APMControllerDISARM)(void);
    void (*APMControllerGPSPosition)(int lat, int lng, float yawdeg, int alt);
    void (*APMControllerPosition)(int x, int y, int z, float yawdeg, bool resetHome);
    void (*APMControllerSpeed)(int x, int y, int z, float yawdegs);
    void (*APMControllerServo)(int pin, int PWMInUs);
};

struct UserAppData
{
    ControllerData APMData;
    V4L2Tools::V4l2Data cameraFrame;
    void (*pushBroadcastData)(std::vector<uint8_t>);
    std::deque<std::vector<uint8_t>> (*getBroadcastRecv)();
};