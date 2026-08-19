#pragma once

#include <vector>
#include <cstdint>
#include <functional>
#include <iostream>
#include "yolo_npu.h"
#include "YoloEngine.hpp"
#include "src/config.hpp"

#include <mutex>
#include "SortTracker.hpp"

struct TrackedBox
{
    int track_id;
    int class_id;
    float confidence;
    int x1;
    int y1;
    int x2;
    int y2;
};

struct DetectionBox
{
    uint16_t id;
    uint16_t type;
    uint16_t confidence;
    uint16_t x1;
    uint16_t y1;
    uint16_t x2;
    uint16_t y2;
    uint16_t track_id;
};

class ProtocolSerializer
{
private:
    inline static std::vector<TrackedBox> s_latestDetections;
    inline static std::mutex s_detectionsMutex;

public:
    inline static std::vector<TrackedBox> getLatestDetections()
    {
        std::lock_guard<std::mutex> lock(s_detectionsMutex);
        return s_latestDetections;
    }

    inline static void setLatestDetections(const std::vector<TrackedBox>& dets)
    {
        std::lock_guard<std::mutex> lock(s_detectionsMutex);
        s_latestDetections = dets;
    }

    inline static std::vector<uint8_t> serialize(const DetectionBox &det)
    {
        auto appendUint16 = [](std::vector<uint8_t> &vec, uint16_t val)
        {
            vec.push_back(static_cast<uint8_t>((val >> 0) & 0xFF));
            vec.push_back(static_cast<uint8_t>((val >> 8) & 0xFF));
        };

        std::vector<uint8_t> broadcastData;
        broadcastData.reserve(17);
        broadcastData.push_back(0xFE);

        appendUint16(broadcastData, det.id);
        appendUint16(broadcastData, det.type);
        appendUint16(broadcastData, det.confidence);
        appendUint16(broadcastData, det.x1);
        appendUint16(broadcastData, det.y1);
        appendUint16(broadcastData, det.x2);
        appendUint16(broadcastData, det.y2);
        appendUint16(broadcastData, det.track_id);

        return broadcastData;
    }

    inline static void broadcast(const yolo_image_info_t &info,
                                 const std::function<void(std::vector<uint8_t>)> &pushCallback,
                                 const YoloEngine &yoloEngine)
    {
        static SortTracker tracker(5, 1, 0.3f);
        static std::mutex trackerMutex;

        std::vector<yolo_det_t> currentDets;
        for (int i = 0; i < info.count; ++i)
        {
            currentDets.push_back(info.detections[i]);
        }

        std::vector<std::pair<int, yolo_det_t>> trackedObjs;
        {
            std::lock_guard<std::mutex> lock(trackerMutex);
            trackedObjs = tracker.update(currentDets, info.data, info.width, info.height);
        }

        std::vector<TrackedBox> currentTracks;
        for (size_t i = 0; i < trackedObjs.size(); ++i)
        {
            int trackId = trackedObjs[i].first;
            const yolo_det_t &det = trackedObjs[i].second;

            LOG_EXCH << "[ProtocolSerializer] TrackID=" << trackId
                     << " Class=" << yoloEngine.getLabel(det.cls_id)
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
            boxDet.track_id = static_cast<uint16_t>(trackId);

            std::vector<uint8_t> broadcastData = serialize(boxDet);
            if (pushCallback)
            {
                pushCallback(broadcastData);
            }

            TrackedBox tb;
            tb.track_id = trackId;
            tb.class_id = det.cls_id;
            tb.confidence = det.prob;
            tb.x1 = det.x1;
            tb.y1 = det.y1;
            tb.x2 = det.x2;
            tb.y2 = det.y2;
            currentTracks.push_back(tb);
        }
        setLatestDetections(currentTracks);
    }
};
