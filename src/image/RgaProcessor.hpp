#pragma once

#include <cstdint>
#include <cstddef>
#include "im2d.hpp"
#include "Public/PLGUserDefine.hpp"

class RgaProcessor
{
public:
    RgaProcessor();
    ~RgaProcessor();

    bool initialize(int width, int height, unsigned int pixFormat);
    void cleanup();

    // Rotates the frame, but skips hardware rotation immediately if Npu is busy
    uint8_t *rotate180(const V4L2Tools::V4l2Data &srcFrame, bool isNpuBusy = false);

    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }
    size_t getBufferSize() const { return m_frameBufferSize; }

private:
    int m_width;
    int m_height;
    unsigned int m_pixFormat;
    size_t m_frameBufferSize;
    uint8_t *m_frameBuffer;
    rga_buffer_handle_t m_dstRgaHandle;
};
