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

    // Rotates the frame using RGA hardware
    uint8_t *rotate180(const V4L2Tools::V4l2Data &srcFrame);
    uint8_t *rotateFrame(const V4L2Tools::V4l2Data &srcFrame, int angle);

    // Converts NV12 frame to BGR24 using RGA hardware acceleration
    bool convertToBgr24(const uint8_t *srcNv12Data);

    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }
    size_t getBufferSize() const { return m_frameBufferSize; }
    uint8_t *getFrameBuffer() const { return m_frameBuffer; }
    uint8_t *getBgrBuffer() const { return m_bgrBuffer; }
    size_t getBgrBufferSize() const { return m_bgrBufferSize; }

private:
    int m_width;
    int m_height;
    unsigned int m_pixFormat;
    size_t m_frameBufferSize;
    uint8_t *m_frameBuffer;
    rga_buffer_handle_t m_dstRgaHandle;

    size_t m_bgrBufferSize;
    uint8_t *m_bgrBuffer;
    rga_buffer_handle_t m_bgrRgaHandle;

    int m_lastRotationAngle;
};
