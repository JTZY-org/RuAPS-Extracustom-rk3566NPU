#pragma once
#include <functional>
#include <vector>
#include "V4L2ToolKit.hpp"

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

    float *_Baro_Temp;
    float *_Baro_PressureHPA;
    float *_Baro_AGLAltitudeCM;
    float *_RangeFinder_AGLAltCM;

    bool *_SYS_ARMFlag;
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

    int *_RC_Channel_Raw[16];
    int *_EF_Channel_Raw[16];

    uint64_t *_GYRO_CYCLE_TIME;
    float *_Battery_Voltage;
    float *_Battery_Voltage_Single;

    double *_CPU_Core_Temp;

    std::function<void(void)> APMControllerARM;
    std::function<void(void)> APMControllerDISARM;
    std::function<void(int x, int y, int z, bool resetHome)> APMControllerPosition;
    std::function<void(int x, int y, int z)> APMControllerSpeed;
    std::function<void(int pin, int PWMInUs)> APMControllerServo;
};

struct UserAppData
{
    ControllerData APMData;
    V4L2Tools::V4l2Data cameraFrame;
    std::deque<std::vector<uint8_t>> *BoradCastRecv;
    std::function<void(std::vector<uint8_t>)> pushBoradcastData;
};