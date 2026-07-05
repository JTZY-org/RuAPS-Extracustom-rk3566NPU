#pragma once

#include <vector>
#include <cstdint>
#include <functional>
#include <iostream>
#include "yolo_npu.h"
#include "YoloEngine.hpp"
#include "src/config.hpp"

struct DetectionBox
{
    uint16_t id;
    uint16_t type;
    uint16_t confidence;
    uint16_t x1;
    uint16_t y1;
    uint16_t x2;
    uint16_t y2;
};

class ProtocolSerializer
{
public:
    inline static std::vector<uint8_t> serialize(const DetectionBox &det)
    {
        auto appendUint16 = [](std::vector<uint8_t> &vec, uint16_t val)
        {
            vec.push_back(static_cast<uint8_t>((val >> 0) & 0xFF));
            vec.push_back(static_cast<uint8_t>((val >> 8) & 0xFF));
        };

        std::vector<uint8_t> broadcastData;
        broadcastData.reserve(15);
        broadcastData.push_back(0xFE);

        appendUint16(broadcastData, det.id);
        appendUint16(broadcastData, det.type);
        appendUint16(broadcastData, det.confidence);
        appendUint16(broadcastData, det.x1);
        appendUint16(broadcastData, det.y1);
        appendUint16(broadcastData, det.x2);
        appendUint16(broadcastData, det.y2);

        return broadcastData;
    }

    inline static void broadcast(const yolo_image_info_t &info,
                                 const std::function<void(std::vector<uint8_t>)> &pushCallback,
                                 const YoloEngine &yoloEngine)
    {
        for (int i = 0; i < info.count; ++i)
        {
            const yolo_det_t &det = info.detections[i];

            LOG_EXCH << "[ProtocolSerializer] Det[" << i << "] Class=" << yoloEngine.getLabel(det.cls_id)
                     << " Conf=" << det.prob
                     << " Box=(" << det.x1 << "," << det.y1 << ")-(" << det.x2 << "," << det.y2 << ")"
                     << std::endl;

            DetectionBox boxDet;
            boxDet.id = static_cast<uint16_t>(i + 1);
            boxDet.type = static_cast<uint16_t>(det.cls_id);
            boxDet.confidence = static_cast<uint16_t>(det.prob * 100.0f);
            boxDet.x1 = static_cast<uint16_t>(det.x1);
            boxDet.y1 = static_cast<uint16_t>(det.y1);
            boxDet.x2 = static_cast<uint16_t>(det.x2);
            boxDet.y2 = static_cast<uint16_t>(det.y2);

            std::vector<uint8_t> broadcastData = serialize(boxDet);
            if (pushCallback)
            {
                pushCallback(broadcastData);
            }
        }
    }
};
