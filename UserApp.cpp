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
#include "src/python/PythonEngine.hpp"

namespace
{
    RgaProcessor g_rgaProcessor;
    YoloEngine g_yoloEngine;
    FlightController g_flightController;
    PythonEngine g_pythonEngine;
}

extern "C" void UserAppInit(V4L2Tools::V4l2Info vinfo)
{
    g_rgaProcessor.initialize(vinfo.ImgWidth, vinfo.ImgHeight, vinfo.PixFormat);
    g_yoloEngine.initialize();
    g_pythonEngine.initialize(vinfo);
}

extern "C" void UserAppExChange(UserAppData data)
{
    std::vector<std::vector<uint8_t>> broadcastPackets;
    if (data.BoradCastRecv != nullptr)
    {
        for (const auto &packet : *data.BoradCastRecv)
        {
            broadcastPackets.push_back(packet);
        }
    }

    // 1. Process and pop incoming broadcast command messages internally
    g_flightController.processCmd(data);

    // 2. Telemetry and Landing state machine checks
    g_flightController.updateState(data);

    // 3. Image Preprocessing (RGA Hardware Rotation, bypassed internally if NPU is busy)
    uint8_t *rotatedFrame = g_rgaProcessor.rotate180(data.cameraFrame, g_yoloEngine.isBusy());

    // Convert rotated NV12 frame to BGR24 using RGA hardware for Python OpenCV
    if (g_rgaProcessor.convertToBgr24(rotatedFrame))
    {
        data.cameraFrame.data = g_rgaProcessor.getBgrBuffer();
        data.cameraFrame.size = g_rgaProcessor.getBgrBufferSize();
    }

    // 4. Call Python engine loop to run Python OpenCV and handle commands
    g_pythonEngine.execute(data, broadcastPackets);

    // 5. Asynchronous NPU Detection
    g_yoloEngine.detectAsync(
        rotatedFrame, g_rgaProcessor.getWidth(), g_rgaProcessor.getHeight(),
        [pushCallback = data.pushBoradcastData](const yolo_image_info_t &info)
        {
            // 6. Broadcast YOLO Target Detections
            ProtocolSerializer::broadcast(info, pushCallback, g_yoloEngine);
        });
}
