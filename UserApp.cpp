#include "UserApp.hpp"
#include <deque>
#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <unistd.h>
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

    int getFlipAngle()
    {
        static int cachedAngle = -1; // Initialize to -1 to trigger initial print
        static int frameCounter = 0;

        if (frameCounter++ % 30 == 0)
        {
            int newAngle = 180; // default fallback
            if (access("/etc/rknn/flip90", F_OK) == 0)
            {
                newAngle = 90;
            }
            else if (access("/etc/rknn/flip180", F_OK) == 0)
            {
                newAngle = 180;
            }
            else if (access("/etc/rknn/flip270", F_OK) == 0)
            {
                newAngle = 270;
            }
            else if (access("/etc/rknn/flip0", F_OK) == 0)
            {
                newAngle = 0;
            }
            else
            {
                bool found = false;
                for (const std::string &path : {"/etc/rknn/flipxxx", "/etc/rknn/flip"})
                {
                    if (access(path.c_str(), F_OK) == 0)
                    {
                        std::ifstream ifs(path);
                        if (ifs.is_open())
                        {
                            int val = -1;
                            if (ifs >> val)
                            {
                                if (val == 90 || val == 180 || val == 270 || val == 0)
                                {
                                    newAngle = val;
                                    found = true;
                                    break;
                                }
                            }
                        }
                    }
                }
                if (!found)
                {
                    newAngle = 180; // default fallback
                }
            }

            if (newAngle != cachedAngle)
            {
                std::cout << "[UserApp] Flip configuration detected/updated: " << newAngle << " degrees" << std::endl;
                cachedAngle = newAngle;
            }
        }
        return cachedAngle;
    }
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
    std::deque<std::vector<uint8_t>> recvQueue;
    if (data.getBroadcastRecv != nullptr)
    {
        recvQueue = data.getBroadcastRecv();
        for (const auto &packet : recvQueue)
        {
            broadcastPackets.push_back(packet);
        }
    }

    // 1. Process and pop incoming broadcast command messages internally
    g_flightController.processCmd(recvQueue, data);

    // 2. Telemetry and Landing state machine checks
    g_flightController.updateState(data);

    // 3. Image Preprocessing (RGA Hardware Rotation)
    int currentAngle = getFlipAngle();
    uint8_t *rotatedFrame = g_rgaProcessor.rotateFrame(data.cameraFrame, currentAngle);

    bool isActuallyRotated = (rotatedFrame == g_rgaProcessor.getFrameBuffer() && currentAngle != 0);
    int finalWidth = (isActuallyRotated && (currentAngle == 90 || currentAngle == 270)) ? g_rgaProcessor.getHeight() : g_rgaProcessor.getWidth();
    int finalHeight = (isActuallyRotated && (currentAngle == 90 || currentAngle == 270)) ? g_rgaProcessor.getWidth() : g_rgaProcessor.getHeight();

    // Convert rotated NV12 frame to BGR24 using RGA hardware for Python OpenCV
    if (g_rgaProcessor.convertToBgr24(rotatedFrame))
    {
        data.cameraFrame.data = g_rgaProcessor.getBgrBuffer();
        data.cameraFrame.size = g_rgaProcessor.getBgrBufferSize();
        data.cameraFrame.width = finalWidth;
        data.cameraFrame.height = finalHeight;
    }

    // 4. Call Python engine loop to run Python OpenCV and handle commands
    g_pythonEngine.execute(data, broadcastPackets);

    // 5. Asynchronous NPU Detection
    g_yoloEngine.detectAsync(
        rotatedFrame, finalWidth, finalHeight,
        [pushCallback = data.pushBroadcastData](const yolo_image_info_t &info)
        {
            // 6. Broadcast YOLO Target Detections
            ProtocolSerializer::broadcast(info, pushCallback, g_yoloEngine);
        });
}
