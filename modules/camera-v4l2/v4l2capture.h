/*
 * Copyright (C) 2026 Matthias Klumpp <matthias@tenstral.net>
 *
 * Licensed under the GNU Lesser General Public License Version 3
 */

#pragma once

#include "v4l2device.h"

#include <QString>

#include <cstddef>
#include <vector>

class VariantDataStream;

namespace V4L2Camera
{

struct MMapBuffer {
    void *start = nullptr;
    size_t length = 0;
};

void signalEventFd(int fd);
void drainEventFd(int fd);

class EventFd
{
public:
    EventFd() = default;
    ~EventFd();

    EventFd(const EventFd &) = delete;
    EventFd &operator=(const EventFd &) = delete;
    EventFd(EventFd &&other) noexcept;
    EventFd &operator=(EventFd &&other) noexcept;

    bool open(QString *error);
    void close();
    int fd() const;
    void drain() const;

private:
    int m_fd = -1;
};

class MMapBufferPool
{
public:
    MMapBufferPool() = default;
    ~MMapBufferPool();

    MMapBufferPool(const MMapBufferPool &) = delete;
    MMapBufferPool &operator=(const MMapBufferPool &) = delete;
    MMapBufferPool(MMapBufferPool &&other) noexcept;
    MMapBufferPool &operator=(MMapBufferPool &&other) noexcept;

    bool request(int fd, const CaptureMode &mode, QString *error);
    bool queueAll(QString *error) const;
    void release();
    size_t size() const;
    const MMapBuffer &operator[](size_t index) const;

private:
    int m_fd = -1;
    std::vector<MMapBuffer> m_buffers;
};

class CaptureStreamGuard
{
public:
    CaptureStreamGuard() = default;
    ~CaptureStreamGuard();

    CaptureStreamGuard(const CaptureStreamGuard &) = delete;
    CaptureStreamGuard &operator=(const CaptureStreamGuard &) = delete;

    bool start(int fd, QString *error);
    void stop();

private:
    int m_fd = -1;
    bool m_streaming = false;
};

void setFrameStreamMetadata(VariantDataStream *stream, const CaptureMode &mode);

} // namespace V4L2Camera
