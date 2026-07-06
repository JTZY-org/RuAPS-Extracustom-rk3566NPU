#include "UserApp.hpp"
#include <deque>
#include <vector>
#include <iostream>
#include <iomanip>
#include "src/config.hpp"
#include "src/image/RgaProcessor.hpp"
#include "src/npu/YoloEngine.hpp"
#include "src/flight/FlightController.hpp"
#include "src/npu/ProtocolSerializer.hpp"

namespace
{
    RgaProcessor g_rgaProcessor;
    YoloEngine g_yoloEngine;
    FlightController g_flightController;
}

extern "C" void UserAppInit(V4L2Tools::V4l2Info vinfo)
{
    g_rgaProcessor.initialize(vinfo.ImgWidth, vinfo.ImgHeight, vinfo.PixFormat);
    g_yoloEngine.initialize();
}

extern "C" void UserAppExChange(UserAppData data)
{
    // 1. Process and pop incoming broadcast command messages internally
    g_flightController.processCmd(data);

    // 2. Telemetry and Landing state machine checks
    g_flightController.updateState(data);

    // 3. Image Preprocessing (RGA Hardware Rotation, bypassed internally if NPU is busy)
    uint8_t *rotatedFrame = g_rgaProcessor.rotate180(data.cameraFrame, g_yoloEngine.isBusy());

    // 4. Asynchronous NPU Detection
    g_yoloEngine.detectAsync(
        rotatedFrame, g_rgaProcessor.getWidth(), g_rgaProcessor.getHeight(),
        [pushCallback = data.pushBoradcastData](const yolo_image_info_t &info)
        {
            // 5. Broadcast YOLO Target Detections
            ProtocolSerializer::broadcast(info, pushCallback, g_yoloEngine);
        });
}
