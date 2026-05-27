#include "UserApp.hpp"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// Global persistent deep-copy buffer
std::unique_ptr<V4l2Data> g_deepCopiedFrame = nullptr;

extern "C" void UserAppInit(V4L2Tools::V4l2Info vinfo)
{
    std::cout << "[UserApp] init custom APP" << std::endl;
    // Clear/initialize the output YUV file for a fresh test run
    std::ofstream outfile("stream_nv12.yuv", std::ios::binary | std::ios::trunc);
    if (outfile.is_open())
    {
        std::cout << "[UserApp] Initialized fresh stream_nv12.yuv" << std::endl;
        outfile.close();
    }

    // Allocate persistent deep-copy buffer in UserAppInit using configured camera resolution
    int width = vinfo.ImgWidth;
    int height = vinfo.ImgHeight;
    unsigned int size = width * height * 3 / 2; // NV12 size

    g_deepCopiedFrame = std::make_unique<V4l2Data>(
        width,
        height,
        size,
        size,
        vinfo.PixFormat, // pixfmt
        width,           // bytesperline
        false            // ismapping = false ensures memory allocation
    );
    std::cout << "[UserApp] Allocated persistent deep copy buffer in UserAppInit (size: "
              << size << " bytes, Resolution: " << width << "x" << height << ")" << std::endl;
}

extern "C" void UserAppExChange(UserAppData data)
{
    const auto &frame = data.cameraFrame;
    // dont check this frame.isFrameAvaliable
    if (frame.data == nullptr || frame.size == 0)
    {
        return;
    }

    static std::atomic<uint64_t> frameCounter{0};
    uint64_t currentFrameId = ++frameCounter;

    if (currentFrameId % 10 == 0)
    {
        // std::cout << "[UserApp] Selected frame " << currentFrameId << " for deep copy and append stream test." << std::endl;

        if (g_deepCopiedFrame->data != nullptr && frame.data != nullptr)
        {
            std::copy(frame.data, frame.data + frame.size, g_deepCopiedFrame->data);
        }
        else
        {
            std::cerr << "[UserApp] Error: Memory buffer is null!" << std::endl;
            return;
        }

        // Verify deep copy address distinction
        // std::cout << "[UserApp] Deep Copy Verification: "
        //           << "Original buffer address: " << static_cast<const void *>(frame.data)
        //           << ", Deep copied buffer address: " << static_cast<const void *>(g_deepCopiedFrame->data)
        //           << std::endl;

        // Append deep copied frame data to the single NV12 stream file
        std::string filename = "/tmp/stream_nv12.yuv";
        std::ofstream outfile(filename, std::ios::binary | std::ios::app);
        if (outfile.is_open())
        {
            outfile.write(reinterpret_cast<const char *>(g_deepCopiedFrame->data), g_deepCopiedFrame->size);
            outfile.close();
            // std::cout << "[UserApp] Successfully appended frame " << currentFrameId
            //           << " (" << g_deepCopiedFrame->size << " bytes) to " << filename << std::endl;
        }
        else
        {
            std::cerr << "[UserApp] Failed to open file " << filename << " for appending!" << std::endl;
        }
    }

    std::vector<uint8_t> testbitdata = {
        0x01,
        0x02,
        0x03,
        0x04,
        0xAA,
        0xBB,
        0xCC,
    };
    // data.pushBoradcastData(testbitdata);
}
