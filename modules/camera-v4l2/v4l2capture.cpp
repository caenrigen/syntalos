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
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/eventfd.h>
#include <sys/mman.h>
#include <utility>
#include <unistd.h>

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

void drainEventFd(int fd)
{
    if (fd < 0)
        return;

    uint64_t value = 0;
    while (::read(fd, &value, sizeof(value)) == sizeof(value)) {
    }
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

EventFd::EventFd(EventFd &&other) noexcept
    : m_fd(other.m_fd)
{
    other.m_fd = -1;
}

EventFd &EventFd::operator=(EventFd &&other) noexcept
{
    if (this != &other) {
        close();
        m_fd = other.m_fd;
        other.m_fd = -1;
    }
    return *this;
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

MMapBufferPool::MMapBufferPool(MMapBufferPool &&other) noexcept
    : m_fd(other.m_fd),
      m_buffers(std::move(other.m_buffers))
{
    other.m_fd = -1;
}

MMapBufferPool &MMapBufferPool::operator=(MMapBufferPool &&other) noexcept
{
    if (this != &other) {
        release();
        m_fd = other.m_fd;
        m_buffers = std::move(other.m_buffers);
        other.m_fd = -1;
    }
    return *this;
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
