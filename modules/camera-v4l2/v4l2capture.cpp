/*
 * Copyright (C) 2026 Matthias Klumpp <matthias@tenstral.net>
 *
 * Licensed under the GNU Lesser General Public License Version 3
 */

#include "v4l2capture.h"

#include "datactl/streammeta.h"
#include "streams/stream.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <limits>
#include <sys/eventfd.h>
#include <sys/mman.h>
#include <unistd.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
}

namespace
{

QString avErrorString(int errnum)
{
    char buf[AV_ERROR_MAX_STRING_SIZE] = {};
    av_strerror(errnum, buf, sizeof(buf));
    return QString::fromLocal8Bit(buf);
}

AVPixelFormat normalizeJpegPixelFormat(AVPixelFormat format, bool *fullRange)
{
    switch (format) {
    case AV_PIX_FMT_YUVJ420P:
        *fullRange = true;
        return AV_PIX_FMT_YUV420P;
    case AV_PIX_FMT_YUVJ422P:
        *fullRange = true;
        return AV_PIX_FMT_YUV422P;
    case AV_PIX_FMT_YUVJ444P:
        *fullRange = true;
        return AV_PIX_FMT_YUV444P;
    case AV_PIX_FMT_YUVJ440P:
        *fullRange = true;
        return AV_PIX_FMT_YUV440P;
    case AV_PIX_FMT_YUVJ411P:
        *fullRange = true;
        return AV_PIX_FMT_YUV411P;
    default:
        return format;
    }
}

void drainEventFd(int fd)
{
    if (fd < 0)
        return;

    uint64_t value = 0;
    while (::read(fd, &value, sizeof(value)) == sizeof(value)) {
    }
}

} // namespace

namespace V4L2Camera
{

void signalEventFd(int fd)
{
    if (fd < 0)
        return;

    uint64_t value = 1;
    const auto unused = ::write(fd, &value, sizeof(value));
    Q_UNUSED(unused);
}

static size_t estimatedFrameBufferSize(const CaptureMode &mode)
{
    if (mode.sizeImage > 0)
        return mode.sizeImage;
    if (mode.bytesPerLine > 0 && mode.height > 0)
        return static_cast<size_t>(mode.bytesPerLine) * static_cast<size_t>(mode.height);
    if (mode.width > 0 && mode.height > 0 && mode.cvType >= 0)
        return static_cast<size_t>(mode.width) * static_cast<size_t>(mode.height) * CV_ELEM_SIZE(mode.cvType);
    return 0;
}

static int64_t decodedFrameStrideBytes(const CaptureMode &mode)
{
    if (mode.width <= 0 || mode.cvType < 0)
        return 0;
    return static_cast<int64_t>(mode.width) * static_cast<int64_t>(CV_ELEM_SIZE(mode.cvType));
}

static quint32 requestedMMapBufferCount(const CaptureMode &mode)
{
    constexpr quint32 minBuffers = 2;
    constexpr quint32 defaultMinQueue = 15;
    constexpr quint32 hardMaxBuffers = 128;
    constexpr size_t maxQueueBytes = 256ULL * 1024ULL * 1024ULL;

    quint32 requested = defaultMinQueue;
    const auto fps = mode.fps();
    if (fps > static_cast<double>(defaultMinQueue))
        requested = static_cast<quint32>(std::ceil(fps)) + 1;

    requested = std::clamp(requested, minBuffers, hardMaxBuffers);

    const auto frameBytes = estimatedFrameBufferSize(mode);
    if (frameBytes > 0) {
        const auto maxByMemory = std::max<quint32>(
            minBuffers,
            static_cast<quint32>(std::min<size_t>(hardMaxBuffers, maxQueueBytes / frameBytes)));
        requested = std::min(requested, maxByMemory);
    }

    return std::max(requested, minBuffers);
}

EventFd::~EventFd()
{
    close();
}

bool EventFd::open(QString *error)
{
    close();
    m_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (m_fd < 0) {
        if (error != nullptr)
            *error = errnoString();
        return false;
    }
    return true;
}

void EventFd::close()
{
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
}

int EventFd::fd() const
{
    return m_fd;
}

void EventFd::drain() const
{
    drainEventFd(m_fd);
}

MMapBufferPool::~MMapBufferPool()
{
    release();
}

bool MMapBufferPool::request(int fd, const CaptureMode &mode, QString *error)
{
    release();
    m_fd = fd;

    v4l2_requestbuffers req = {};
    req.count = requestedMMapBufferCount(mode);
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        if (error != nullptr)
            *error = QStringLiteral("VIDIOC_REQBUFS failed: %1").arg(errnoString());
        return false;
    }
    if (req.count < 2) {
        if (error != nullptr)
            *error = QStringLiteral("V4L2 driver allocated too few streaming buffers.");
        return false;
    }

    m_buffers.resize(req.count);
    for (quint32 i = 0; i < req.count; ++i) {
        v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            if (error != nullptr)
                *error = QStringLiteral("VIDIOC_QUERYBUF failed for buffer %1: %2").arg(i).arg(errnoString());
            return false;
        }

        m_buffers[i].length = buf.length;
        m_buffers[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if (m_buffers[i].start == MAP_FAILED) {
            m_buffers[i].start = nullptr;
            if (error != nullptr)
                *error = QStringLiteral("mmap failed for V4L2 buffer %1: %2").arg(i).arg(errnoString());
            return false;
        }
    }

    return true;
}

bool MMapBufferPool::queueAll(QString *error) const
{
    if (m_fd < 0) {
        if (error != nullptr)
            *error = QStringLiteral("No V4L2 mmap buffers have been requested.");
        return false;
    }

    for (quint32 i = 0; i < m_buffers.size(); ++i) {
        v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (xioctl(m_fd, VIDIOC_QBUF, &buf) < 0) {
            if (error != nullptr)
                *error = QStringLiteral("VIDIOC_QBUF failed for buffer %1: %2").arg(i).arg(errnoString());
            return false;
        }
    }
    return true;
}

void MMapBufferPool::release()
{
    for (auto &buffer : m_buffers) {
        if (buffer.start != nullptr) {
            munmap(buffer.start, buffer.length);
            buffer.start = nullptr;
            buffer.length = 0;
        }
    }
    m_buffers.clear();

    if (m_fd >= 0) {
        v4l2_requestbuffers req = {};
        req.count = 0;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        xioctl(m_fd, VIDIOC_REQBUFS, &req);
    }
    m_fd = -1;
}

size_t MMapBufferPool::size() const
{
    return m_buffers.size();
}

const MMapBuffer &MMapBufferPool::operator[](size_t index) const
{
    return m_buffers[index];
}

CaptureStreamGuard::~CaptureStreamGuard()
{
    stop();
}

bool CaptureStreamGuard::start(int fd, QString *error)
{
    stop();
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        if (error != nullptr)
            *error = QStringLiteral("VIDIOC_STREAMON failed: %1").arg(errnoString());
        return false;
    }
    m_fd = fd;
    m_streaming = true;
    return true;
}

void CaptureStreamGuard::stop()
{
    if (m_streaming && m_fd >= 0) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(m_fd, VIDIOC_STREAMOFF, &type);
    }
    m_streaming = false;
    m_fd = -1;
}

FrameDecoder::FrameDecoder()
    : m_codecCtx(nullptr),
      m_avFrame(nullptr),
      m_packet(nullptr),
      m_swsCtx(nullptr),
      m_configured(false)
{
}

FrameDecoder::~FrameDecoder()
{
    reset();
}

bool FrameDecoder::configure(const CaptureMode &mode, QString *error)
{
    reset();
    if (!mode.isValid() || !isSupportedFourcc(mode.fourcc)) {
        if (error != nullptr)
            *error = QStringLiteral("Unsupported V4L2 pixel format %1.").arg(mode.fourccString);
        return false;
    }

    m_mode = mode;
    if (mode.fourcc == V4L2_PIX_FMT_MJPEG || mode.fourcc == v4l2_fourcc('M', 'J', 'P', 'G')) {
        const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
        if (codec == nullptr) {
            if (error != nullptr)
                *error = QStringLiteral("No FFmpeg MJPEG decoder is available.");
            return false;
        }

        m_codecCtx = avcodec_alloc_context3(codec);
        if (m_codecCtx == nullptr) {
            if (error != nullptr)
                *error = QStringLiteral("Unable to allocate MJPEG decoder context.");
            return false;
        }
        m_codecCtx->width = mode.width;
        m_codecCtx->height = mode.height;

        const int ret = avcodec_open2(m_codecCtx, codec, nullptr);
        if (ret < 0) {
            if (error != nullptr)
                *error = QStringLiteral("Unable to open MJPEG decoder: %1").arg(avErrorString(ret));
            return false;
        }

        m_avFrame = av_frame_alloc();
        m_packet = av_packet_alloc();
        if (m_avFrame == nullptr || m_packet == nullptr) {
            if (error != nullptr)
                *error = QStringLiteral("Unable to allocate MJPEG decoder frame/packet.");
            return false;
        }
    }

    m_configured = true;
    return true;
}

void FrameDecoder::reset()
{
    if (m_swsCtx != nullptr) {
        sws_freeContext(m_swsCtx);
        m_swsCtx = nullptr;
    }
    if (m_packet != nullptr) {
        av_packet_free(&m_packet);
        m_packet = nullptr;
    }
    if (m_avFrame != nullptr) {
        av_frame_free(&m_avFrame);
        m_avFrame = nullptr;
    }
    if (m_codecCtx != nullptr) {
        avcodec_free_context(&m_codecCtx);
        m_codecCtx = nullptr;
    }
    m_configured = false;
}

bool FrameDecoder::decode(const quint8 *data, size_t size, cv::Mat *out, QString *error)
{
    if (!m_configured) {
        if (error != nullptr)
            *error = QStringLiteral("V4L2 frame decoder is not configured.");
        return false;
    }
    if (data == nullptr || size == 0 || out == nullptr) {
        if (error != nullptr)
            *error = QStringLiteral("Empty V4L2 frame buffer.");
        return false;
    }

    if (m_mode.fourcc == V4L2_PIX_FMT_GREY)
        return decodeGrey(data, size, out, error);
    if (m_mode.fourcc == V4L2_PIX_FMT_YUYV)
        return decodeYuyv(data, size, out, error);
    if (m_mode.fourcc == V4L2_PIX_FMT_MJPEG || m_mode.fourcc == v4l2_fourcc('M', 'J', 'P', 'G'))
        return decodeMjpeg(data, size, out, error);

    if (error != nullptr)
        *error = QStringLiteral("Unsupported V4L2 pixel format %1.").arg(m_mode.fourccString);
    return false;
}

bool FrameDecoder::decodeGrey(const quint8 *data, size_t size, cv::Mat *out, QString *error)
{
    const size_t rowBytes = static_cast<size_t>(m_mode.width);
    const size_t srcStride = m_mode.bytesPerLine > 0 ? m_mode.bytesPerLine : rowBytes;
    const size_t minSize = srcStride * static_cast<size_t>(m_mode.height - 1) + rowBytes;
    if (size < minSize) {
        if (error != nullptr)
            *error = QStringLiteral("GREY frame is too small.");
        return false;
    }

    out->create(m_mode.height, m_mode.width, CV_8UC1);
    for (int y = 0; y < m_mode.height; ++y)
        std::memcpy(out->ptr(y), data + static_cast<size_t>(y) * srcStride, rowBytes);
    return true;
}

bool FrameDecoder::decodeYuyv(const quint8 *data, size_t size, cv::Mat *out, QString *error)
{
    const int srcStride = static_cast<int>(m_mode.bytesPerLine > 0 ? m_mode.bytesPerLine : m_mode.width * 2);
    const size_t minSize = static_cast<size_t>(srcStride) * static_cast<size_t>(m_mode.height - 1)
        + static_cast<size_t>(m_mode.width) * 2;
    if (size < minSize) {
        if (error != nullptr)
            *error = QStringLiteral("YUYV frame is too small.");
        return false;
    }

    out->create(m_mode.height, m_mode.width, CV_8UC3);
    m_swsCtx = sws_getCachedContext(
        m_swsCtx,
        m_mode.width,
        m_mode.height,
        AV_PIX_FMT_YUYV422,
        m_mode.width,
        m_mode.height,
        AV_PIX_FMT_BGR24,
        SWS_FAST_BILINEAR,
        nullptr,
        nullptr,
        nullptr);
    if (m_swsCtx == nullptr) {
        if (error != nullptr)
            *error = QStringLiteral("Unable to create YUYV swscale context.");
        return false;
    }

    const uint8_t *srcData[4] = {data, nullptr, nullptr, nullptr};
    const int srcLinesize[4] = {srcStride, 0, 0, 0};
    uint8_t *dstData[4] = {out->data, nullptr, nullptr, nullptr};
    const int dstLinesize[4] = {static_cast<int>(out->step), 0, 0, 0};
    sws_scale(m_swsCtx, srcData, srcLinesize, 0, m_mode.height, dstData, dstLinesize);
    return true;
}

bool FrameDecoder::decodeMjpeg(const quint8 *data, size_t size, cv::Mat *out, QString *error)
{
    if (m_codecCtx == nullptr || m_avFrame == nullptr || m_packet == nullptr) {
        if (error != nullptr)
            *error = QStringLiteral("MJPEG decoder is not initialized.");
        return false;
    }

    av_packet_unref(m_packet);
    if (size > static_cast<size_t>(std::numeric_limits<int>::max())) {
        if (error != nullptr)
            *error = QStringLiteral("MJPEG frame is too large for FFmpeg packet decoding.");
        return false;
    }

    constexpr size_t paddingSize = AV_INPUT_BUFFER_PADDING_SIZE;
    if (size > m_mjpegInputBuffer.max_size() - paddingSize) {
        if (error != nullptr)
            *error = QStringLiteral("MJPEG frame is too large to pad for FFmpeg packet decoding.");
        return false;
    }

    // FFmpeg bitstream readers may read past packet->size for optimized parsing.
    // Compressed input must therefore be followed by AV_INPUT_BUFFER_PADDING_SIZE
    // zero bytes; V4L2's mmap buffer only guarantees bytesused valid payload bytes.
    m_mjpegInputBuffer.resize(size + paddingSize);
    std::memcpy(m_mjpegInputBuffer.data(), data, size);
    std::memset(m_mjpegInputBuffer.data() + size, 0, paddingSize);

    m_packet->data = m_mjpegInputBuffer.data();
    m_packet->size = static_cast<int>(size);

    int ret = avcodec_send_packet(m_codecCtx, m_packet);
    m_packet->data = nullptr;
    m_packet->size = 0;
    if (ret < 0) {
        if (error != nullptr)
            *error = QStringLiteral("MJPEG packet decode failed: %1").arg(avErrorString(ret));
        return false;
    }

    ret = avcodec_receive_frame(m_codecCtx, m_avFrame);
    if (ret < 0) {
        if (error != nullptr)
            *error = QStringLiteral("MJPEG decoder did not return a frame: %1").arg(avErrorString(ret));
        return false;
    }

    bool srcFullRange = m_avFrame->color_range == AVCOL_RANGE_JPEG;
    const auto srcFmt = normalizeJpegPixelFormat(static_cast<AVPixelFormat>(m_avFrame->format), &srcFullRange);
    m_swsCtx = sws_getCachedContext(
        m_swsCtx,
        m_avFrame->width,
        m_avFrame->height,
        srcFmt,
        m_mode.width,
        m_mode.height,
        AV_PIX_FMT_BGR24,
        SWS_BILINEAR,
        nullptr,
        nullptr,
        nullptr);
    if (m_swsCtx == nullptr) {
        if (error != nullptr)
            *error = QStringLiteral("Unable to create MJPEG swscale context.");
        av_frame_unref(m_avFrame);
        return false;
    }

    // FFmpeg deprecates YUVJ* formats; use regular YUV and set JPEG/full range explicitly.
    int *invTable = nullptr;
    int *table = nullptr;
    int srcRange = 0;
    int dstRange = 0;
    int brightness = 0;
    int contrast = 0;
    int saturation = 0;
    if (sws_getColorspaceDetails(m_swsCtx, &invTable, &srcRange, &table, &dstRange, &brightness, &contrast, &saturation)
        >= 0) {
        sws_setColorspaceDetails(m_swsCtx, invTable, srcFullRange ? 1 : srcRange, table, 1, brightness, contrast, saturation);
    }

    out->create(m_mode.height, m_mode.width, CV_8UC3);
    uint8_t *dstData[4] = {out->data, nullptr, nullptr, nullptr};
    const int dstLinesize[4] = {static_cast<int>(out->step), 0, 0, 0};
    sws_scale(m_swsCtx, m_avFrame->data, m_avFrame->linesize, 0, m_avFrame->height, dstData, dstLinesize);
    av_frame_unref(m_avFrame);
    return true;
}

void setFrameStreamMetadata(VariantDataStream *stream, const CaptureMode &mode)
{
    stream->setMetadataValue("size", Syntalos::MetaSize(mode.width, mode.height));
    stream->setMetadataValue("framerate", mode.fps());
    stream->setMetadataValue("timeperframe_num", static_cast<int64_t>(mode.timeperframeNum));
    stream->setMetadataValue("timeperframe_den", static_cast<int64_t>(mode.timeperframeDen));
    stream->setMetadataValue("depth", static_cast<int64_t>(CV_MAT_DEPTH(mode.cvType)));
    stream->setMetadataValue("has_color", CV_MAT_CN(mode.cvType) > 1);
    stream->setMetadataValue("fourcc", mode.fourccString.toStdString());
    stream->setMetadataValue("stride", decodedFrameStrideBytes(mode));
    stream->setMetadataValue("v4l2_bytes_per_line", static_cast<int64_t>(mode.bytesPerLine));
    stream->setMetadataValue("colorspace", static_cast<int64_t>(mode.colorspace));
    stream->setMetadataValue("field", static_cast<int64_t>(mode.field));
    stream->setMetadataValue("bayer_pattern", std::string("none"));
    stream->setMetadataValue("timestamp_basis", std::string("v4l2_buffer_timestamp"));
    stream->setMetadataValue("timestamp_clock", std::string("syntalos_run_time"));
    stream->setMetadataValue("timestamp_reference", std::string("syntalos_run_start"));
    stream->setMetadataValue("v4l2_timestamp_clock", std::string("CLOCK_MONOTONIC"));
    stream->setMetadataValue("v4l2_timestamp_type", std::string("monotonic_required"));
    stream->setMetadataValue("v4l2_timestamp_source", std::string("soe_or_eof"));
    stream->setMetadataValue("v4l2_timestamp_sources_accepted", std::string("soe,eof"));
    stream->setMetadataValue("frame_index_source", std::string("v4l2_buffer_sequence"));
}

} // namespace V4L2Camera
