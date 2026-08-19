#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "yolo_npu.h"

// HSV Color structure
struct HSV {
    float h = 0.0f; // Hue (0 - 360)
    float s = 0.0f; // Saturation (0 - 1)
    float v = 0.0f; // Value (0 - 1)
};

// Convert RGB to HSV
inline HSV rgb2hsv(uint8_t r, uint8_t g, uint8_t b) {
    float rf = r / 255.0f;
    float gf = g / 255.0f;
    float bf = b / 255.0f;

    float maxVal = std::max({rf, gf, bf});
    float minVal = std::min({rf, gf, bf});
    float delta = maxVal - minVal;

    HSV hsv;
    hsv.v = maxVal;
    hsv.s = (maxVal > 0.0f) ? (delta / maxVal) : 0.0f;

    if (delta == 0.0f) {
        hsv.h = 0.0f;
    } else {
        if (maxVal == rf) {
            hsv.h = 60.0f * (fmod(((gf - bf) / delta), 6.0f));
        } else if (maxVal == gf) {
            hsv.h = 60.0f * (((bf - rf) / delta) + 2.0f);
        } else if (maxVal == bf) {
            hsv.h = 60.0f * (((rf - gf) / delta) + 4.0f);
        }
        if (hsv.h < 0.0f) hsv.h += 360.0f;
    }
    return hsv;
}

// Convert NV12 YUV pixel to RGB
inline void nv12_to_rgb(uint8_t y_val, uint8_t u_val, uint8_t v_val, uint8_t &r, uint8_t &g, uint8_t &b) {
    int c = y_val - 16;
    int d = u_val - 128;
    int e = v_val - 128;

    // Standard ITU-R BT.601 formula for YUV to RGB
    int r_temp = (298 * c + 409 * e + 128) >> 8;
    int g_temp = (298 * c - 100 * d - 208 * e + 128) >> 8;
    int b_temp = (298 * c + 516 * d + 128) >> 8;

    r = std::clamp(r_temp, 0, 255);
    g = std::clamp(g_temp, 0, 255);
    b = std::clamp(b_temp, 0, 255);
}

// Extract average HSV of a bounding box from NV12 data using "Focus Center Method" (中心收縮法)
inline HSV get_box_average_hsv(const yolo_det_t& box, const uint8_t* nv12_data, int img_w, int img_h) {
    if (!nv12_data) return {0, 0, 0};

    // 1. Calculate boundaries of the box
    int x1 = std::max(0, box.x1);
    int y1 = std::max(0, box.y1);
    int x2 = std::min(img_w - 1, box.x2);
    int y2 = std::min(img_h - 1, box.y2);

    int box_w = x2 - x1;
    int box_h = y2 - y1;

    // 2. Shrink to center 50% region to filter out background noise
    int sx1 = x1 + box_w * 0.25f;
    int sy1 = y1 + box_h * 0.25f;
    int sx2 = x2 - box_w * 0.25f;
    int sy2 = y2 - box_h * 0.25f;

    // Ensure we have a valid region size
    if (sx2 < sx1) std::swap(sx1, sx2);
    if (sy2 < sy1) std::swap(sy1, sy2);

    float sumH = 0.0f, sumS = 0.0f, sumV = 0.0f;
    int count = 0;

    // 3. Step-based pixel sampling to minimize memory reads (e.g., sample ~5x5 grid)
    int step_x = std::max(1, (sx2 - sx1) / 5);
    int step_y = std::max(1, (sy2 - sy1) / 5);

    int uv_offset = img_w * img_h;

    for (int y = sy1; y <= sy2; y += step_y) {
        for (int x = sx1; x <= sx2; x += step_x) {
            // Read Y value
            uint8_t y_val = nv12_data[y * img_w + x];

            // Read U/V values (NV12 format: U and V are interleaved in the second plane)
            // U/V are sub-sampled 2x2, so we find U/V block at (y/2, x/2)
            int uv_index = uv_offset + (y / 2) * img_w + (x / 2) * 2;
            uint8_t u_val = nv12_data[uv_index];
            uint8_t v_val = nv12_data[uv_index + 1];

            uint8_t r, g, b;
            nv12_to_rgb(y_val, u_val, v_val, r, g, b);

            HSV hsv = rgb2hsv(r, g, b);
            sumH += hsv.h;
            sumS += hsv.s;
            sumV += hsv.v;
            count++;
        }
    }

    HSV avg;
    if (count > 0) {
        avg.h = sumH / count;
        avg.s = sumS / count;
        avg.v = sumV / count;
    }
    return avg;
}

// 1D Kalman Filter with Constant Velocity model
struct Kalman1D {
    float p = 0.0f; // Position
    float v = 0.0f; // Velocity
    
    // Covariance matrix P
    float p00 = 10.0f;
    float p01 = 0.0f;
    float p11 = 10.0f;
    
    // Hyperparameters
    float q00 = 1.0f;    // Process noise for position
    float q11 = 0.01f;   // Process noise for velocity
    float r = 10.0f;     // Measurement noise

    void predict() {
        p = p + v;
        float p00_pred = p00 + 2.0f * p01 + p11 + q00;
        float p01_pred = p01 + p11;
        float p11_pred = p11 + q11;
        
        p00 = p00_pred;
        p01 = p01_pred;
        p11 = p11_pred;
    }

    void update(float z) {
        float y = z - p;
        float s = p00 + r;
        if (std::abs(s) < 1e-6f) return;
        
        float k0 = p00 / s;
        float k1 = p01 / s;
        
        p = p + k0 * y;
        v = v + k1 * y;
        
        float p00_new = (1.0f - k0) * p00;
        float p01_new = (1.0f - k0) * p01;
        float p11_new = -k1 * p01 + p11;
        
        p00 = p00_new;
        p01 = p01_new;
        p11 = p11_new;
    }
};

// Represents a tracked object in the SORT algorithm
class KalmanBoxTracker {
public:
    int id;
    int cls_id;
    float prob;
    int age = 0;
    int time_since_update = 0;
    int hits = 0;
    
    Kalman1D kx; // center x
    Kalman1D ky; // center y
    Kalman1D kw; // width
    Kalman1D kh; // height

    HSV avg_hsv;
    bool has_color = false;

    KalmanBoxTracker(int track_id, const yolo_det_t& det, const uint8_t* nv12_data, int img_w, int img_h) {
        id = track_id;
        cls_id = det.cls_id;
        prob = det.prob;
        
        float cx = (det.x1 + det.x2) / 2.0f;
        float cy = (det.y1 + det.y2) / 2.0f;
        float w = det.x2 - det.x1;
        float h = det.y2 - det.y1;
        
        kx.p = cx;
        ky.p = cy;
        kw.p = w;
        kh.p = h;

        if (nv12_data) {
            avg_hsv = get_box_average_hsv(det, nv12_data, img_w, img_h);
            has_color = true;
        }
    }

    void predict() {
        kx.predict();
        ky.predict();
        kw.predict();
        kh.predict();
        
        age++;
        time_since_update++;
    }

    void update(const yolo_det_t& det, const uint8_t* nv12_data, int img_w, int img_h) {
        float cx = (det.x1 + det.x2) / 2.0f;
        float cy = (det.y1 + det.y2) / 2.0f;
        float w = det.x2 - det.x1;
        float h = det.y2 - det.y1;
        
        kx.update(cx);
        ky.update(cy);
        kw.update(w);
        kh.update(h);
        
        cls_id = det.cls_id;
        prob = det.prob;
        time_since_update = 0;
        hits++;

        // Exponential Moving Average (EMA) color update
        if (nv12_data) {
            HSV new_hsv = get_box_average_hsv(det, nv12_data, img_w, img_h);
            if (!has_color) {
                avg_hsv = new_hsv;
                has_color = true;
            } else {
                float alpha = 0.15f; // EMA weight
                
                // Interpolate Hue on the 360-degree circle
                float diff = new_hsv.h - avg_hsv.h;
                if (diff > 180.0f) diff -= 360.0f;
                else if (diff < -180.0f) diff += 360.0f;
                
                avg_hsv.h = avg_hsv.h + alpha * diff;
                if (avg_hsv.h < 0.0f) avg_hsv.h += 360.0f;
                if (avg_hsv.h >= 360.0f) avg_hsv.h -= 360.0f;
                
                avg_hsv.s = (1.0f - alpha) * avg_hsv.s + alpha * new_hsv.s;
                avg_hsv.v = (1.0f - alpha) * avg_hsv.v + alpha * new_hsv.v;
            }
        }
    }

    yolo_det_t get_state() const {
        yolo_det_t det;
        det.cls_id = cls_id;
        det.prob = prob;
        
        float cx = kx.p;
        float cy = ky.p;
        float w = std::max(1.0f, kw.p);
        float h = std::max(1.0f, kh.p);
        
        det.x1 = static_cast<int>(cx - w / 2.0f);
        det.y1 = static_cast<int>(cy - h / 2.0f);
        det.x2 = static_cast<int>(cx + w / 2.0f);
        det.y2 = static_cast<int>(cy + h / 2.0f);
        
        return det;
    }
};

// SORT Tracker manager with color verification
class SortTracker {
private:
    std::vector<KalmanBoxTracker> trackers;
    int next_id = 1;
    int max_age;
    int min_hits;
    float iou_threshold;

    float calculate_iou(const yolo_det_t& box1, const yolo_det_t& box2) {
        float x1 = std::max(static_cast<float>(box1.x1), static_cast<float>(box2.x1));
        float y1 = std::max(static_cast<float>(box1.y1), static_cast<float>(box2.y1));
        float x2 = std::min(static_cast<float>(box1.x2), static_cast<float>(box2.x2));
        float y2 = std::min(static_cast<float>(box1.y2), static_cast<float>(box2.y2));
        
        float w = std::max(0.0f, x2 - x1);
        float h = std::max(0.0f, y2 - y1);
        float intersection = w * h;
        
        float area1 = (box1.x2 - box1.x1) * (box1.y2 - box1.y1);
        float area2 = (box2.x2 - box2.x1) * (box2.y2 - box2.y1);
        float union_area = area1 + area2 - intersection;
        
        if (union_area <= 0.0f) return 0.0f;
        return intersection / union_area;
    }

public:
    SortTracker(int max_age = 5, int min_hits = 1, float iou_threshold = 0.3f)
        : max_age(max_age), min_hits(min_hits), iou_threshold(iou_threshold) {}

    // Main update function: takes new frame detections, raw NV12 image pointer, and dimensions
    std::vector<std::pair<int, yolo_det_t>> update(const std::vector<yolo_det_t>& detections, 
                                                   const uint8_t* nv12_data = nullptr, 
                                                   int img_w = 0, int img_h = 0) {
        // 1. Predict locations of all current trackers
        for (auto& trk : trackers) {
            trk.predict();
        }

        // 2. Compute color features for new detections if image is provided
        std::vector<HSV> det_hsvs;
        if (nv12_data) {
            det_hsvs.reserve(detections.size());
            for (const auto& det : detections) {
                det_hsvs.push_back(get_box_average_hsv(det, nv12_data, img_w, img_h));
            }
        }

        // 3. Associate detections to existing trackers
        std::vector<bool> matched_detections(detections.size(), false);
        std::vector<bool> matched_trackers(trackers.size(), false);
        std::vector<std::pair<int, int>> matches; // pairs of (tracker_idx, detection_idx)

        if (!trackers.empty() && !detections.empty()) {
            struct Association {
                int trk_idx;
                int det_idx;
                float cost; // We will use modified IOU (penalized by color difference)
            };
            std::vector<Association> pairs;
            for (size_t t = 0; t < trackers.size(); ++t) {
                for (size_t d = 0; d < detections.size(); ++d) {
                    if (trackers[t].cls_id == detections[d].cls_id) {
                        float iou = calculate_iou(trackers[t].get_state(), detections[d]);
                        
                        // Apply color filter if colors are available and object has visible color
                        if (iou >= iou_threshold && nv12_data && trackers[t].has_color) {
                            const HSV& t_hsv = trackers[t].avg_hsv;
                            const HSV& d_hsv = det_hsvs[d];

                            // Verify both are not low saturation/value (like black, white, gray)
                            if (t_hsv.s > 0.15f && t_hsv.v > 0.15f && d_hsv.s > 0.15f && d_hsv.v > 0.15f) {
                                float h_diff = std::abs(t_hsv.h - d_hsv.h);
                                if (h_diff > 180.0f) h_diff = 360.0f - h_diff;

                                // Reject association if colors are totally different (Hue diff > 60 degrees)
                                if (h_diff > 60.0f) {
                                    iou = 0.0f; 
                                }
                            }
                        }

                        if (iou >= iou_threshold) {
                            pairs.push_back({static_cast<int>(t), static_cast<int>(d), iou});
                        }
                    }
                }
            }

            // Sort pairs by IOU descending
            std::sort(pairs.begin(), pairs.end(), [](const Association& a, const Association& b) {
                return a.cost > b.cost;
            });

            // Greedy assignment
            for (const auto& pair : pairs) {
                if (!matched_trackers[pair.trk_idx] && !matched_detections[pair.det_idx]) {
                    matched_trackers[pair.trk_idx] = true;
                    matched_detections[pair.det_idx] = true;
                    matches.push_back({pair.trk_idx, pair.det_idx});
                }
            }
        }

        // 4. Update matched trackers
        for (const auto& match : matches) {
            trackers[match.first].update(detections[match.second], nv12_data, img_w, img_h);
        }

        // 5. Create new trackers for unmatched detections
        for (size_t d = 0; d < detections.size(); ++d) {
            if (!matched_detections[d]) {
                trackers.push_back(KalmanBoxTracker(next_id++, detections[d], nv12_data, img_w, img_h));
            }
        }

        // 6. Filter and collect active tracks
        std::vector<std::pair<int, yolo_det_t>> active_tracks;
        auto it = trackers.begin();
        while (it != trackers.end()) {
            if (it->time_since_update > max_age) {
                it = trackers.erase(it);
            } else {
                if (it->time_since_update < 1 && (it->hits >= min_hits || it->age <= min_hits)) {
                    active_tracks.push_back({it->id, it->get_state()});
                }
                ++it;
            }
        }

        return active_tracks;
    }
};
