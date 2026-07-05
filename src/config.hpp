#pragma once

#include <iostream>

namespace config
{
    // Feature and diagnostic toggles
    inline constexpr bool ENABLE_PRINT_INIT = true;
    inline constexpr bool ENABLE_PRINT_EXCHANGE = true;
    inline constexpr bool ENABLE_RGA_ROTATION = true;

    // Default paths
    inline const char *DEFAULT_YOLO_MODEL = "/etc/rknn/yolov8n.rknn";
    inline const char *DEFAULT_YOLO_LABELS = "/etc/rknn/coco_80_labels_list.txt";
}

// Log utility macros matching target print levels
#define LOG_INIT                   \
    if (config::ENABLE_PRINT_INIT) \
    std::cout
#define LOG_INIT_ERR               \
    if (config::ENABLE_PRINT_INIT) \
    std::cerr

#define LOG_EXCH                       \
    if (config::ENABLE_PRINT_EXCHANGE) \
    std::cout
#define LOG_EXCH_ERR                   \
    if (config::ENABLE_PRINT_EXCHANGE) \
    std::cerr
