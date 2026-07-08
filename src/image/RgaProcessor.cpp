#include "RgaProcessor.hpp"
#include "src/config.hpp"
#include <cstdlib>
#include <cstring>
#include <iostream>

RgaProcessor::RgaProcessor()
    : m_width(0), m_height(0), m_pixFormat(0)
    , m_frameBufferSize(0), m_frameBuffer(nullptr), m_dstRgaHandle(0)
    , m_bgrBufferSize(0), m_bgrBuffer(nullptr), m_bgrRgaHandle(0)
{
}

RgaProcessor::~RgaProcessor()
{
    cleanup();
}

bool RgaProcessor::initialize(int width, int height, unsigned int pixFormat)
{
    cleanup();

    m_width = width;
    m_height = height;
    m_pixFormat = pixFormat;
    
    m_frameBufferSize = static_cast<size_t>(m_width) * static_cast<size_t>(m_height) * 3 / 2;
    m_bgrBufferSize = static_cast<size_t>(m_width) * static_cast<size_t>(m_height) * 3;

    if (posix_memalign((void **)&m_frameBuffer, 4096, m_frameBufferSize) != 0)
    {
        m_frameBuffer = nullptr;
        m_frameBufferSize = 0;
        return false;
    }

    m_dstRgaHandle = importbuffer_virtualaddr(m_frameBuffer, m_frameBufferSize);
    if (m_dstRgaHandle == 0)
    {
        cleanup();
        return false;
    }

    if (posix_memalign((void **)&m_bgrBuffer, 4096, m_bgrBufferSize) != 0)
    {
        m_bgrBuffer = nullptr;
        m_bgrBufferSize = 0;
        cleanup();
        return false;
    }

    m_bgrRgaHandle = importbuffer_virtualaddr(m_bgrBuffer, m_bgrBufferSize);
    if (m_bgrRgaHandle == 0)
    {
        cleanup();
        return false;
    }

    return true;
}

void RgaProcessor::cleanup()
{
    if (m_bgrRgaHandle != 0)
    {
        releasebuffer_handle(m_bgrRgaHandle);
        m_bgrRgaHandle = 0;
    }
    if (m_bgrBuffer != nullptr)
    {
        free(m_bgrBuffer);
        m_bgrBuffer = nullptr;
    }
    m_bgrBufferSize = 0;

    if (m_dstRgaHandle != 0)
    {
        releasebuffer_handle(m_dstRgaHandle);
        m_dstRgaHandle = 0;
    }
    if (m_frameBuffer != nullptr)
    {
        free(m_frameBuffer);
        m_frameBuffer = nullptr;
    }
    m_frameBufferSize = 0;
}

uint8_t *RgaProcessor::rotate180(const V4L2Tools::V4l2Data &srcFrame, bool isNpuBusy)
{
    // Skip RGA operations completely if RGA rotation is disabled, NPU is busy, or data is invalid
    if (!config::ENABLE_RGA_ROTATION || isNpuBusy || srcFrame.data == nullptr || srcFrame.size < m_frameBufferSize || m_frameBuffer == nullptr)
    {
        return const_cast<uint8_t *>(srcFrame.data);
    }

    if (m_dstRgaHandle == 0)
    {
        std::memcpy(m_frameBuffer, srcFrame.data, m_frameBufferSize);
        return m_frameBuffer;
    }

    rga_buffer_handle_t src_handle = importbuffer_virtualaddr(const_cast<uint8_t *>(srcFrame.data), m_frameBufferSize);
    if (src_handle == 0)
    {
        std::memcpy(m_frameBuffer, srcFrame.data, m_frameBufferSize);
        return m_frameBuffer;
    }

    rga_buffer_t src_img = wrapbuffer_handle(src_handle, m_width, m_height, RK_FORMAT_YCbCr_420_SP);
    rga_buffer_t dst_img = wrapbuffer_handle(m_dstRgaHandle, m_width, m_height, RK_FORMAT_YCbCr_420_SP);

    IM_STATUS ret = imrotate(src_img, dst_img, IM_HAL_TRANSFORM_ROT_180);
    releasebuffer_handle(src_handle);

    if (ret != IM_STATUS_SUCCESS)
    {
        std::memcpy(m_frameBuffer, srcFrame.data, m_frameBufferSize);
    }

    return m_frameBuffer;
}

bool RgaProcessor::convertToBgr24(const uint8_t *srcNv12Data)
{
    if (srcNv12Data == nullptr || m_bgrBuffer == nullptr || m_bgrRgaHandle == 0)
    {
        return false;
    }

    rga_buffer_handle_t src_handle = 0;
    if (srcNv12Data == m_frameBuffer)
    {
        src_handle = m_dstRgaHandle;
    }
    else
    {
        src_handle = importbuffer_virtualaddr(const_cast<uint8_t *>(srcNv12Data), m_frameBufferSize);
    }

    if (src_handle == 0)
    {
        return false;
    }

    rga_buffer_t src_img = wrapbuffer_handle(src_handle, m_width, m_height, RK_FORMAT_YCbCr_420_SP);
    rga_buffer_t dst_img = wrapbuffer_handle(m_bgrRgaHandle, m_width, m_height, RK_FORMAT_BGR_888);

    IM_STATUS ret = imcopy(src_img, dst_img);

    if (srcNv12Data != m_frameBuffer)
    {
        releasebuffer_handle(src_handle);
    }

    return (ret == IM_STATUS_SUCCESS);
}
