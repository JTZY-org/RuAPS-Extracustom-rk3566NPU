#pragma once
#include <functional>
#include <vector>
#include "V4L2ToolKit.hpp"

struct UserAppData
{
    V4L2Tools::V4l2Data cameraFrame;
    std::function<void(std::vector<uint8_t>)> pushBoradcastData;
};