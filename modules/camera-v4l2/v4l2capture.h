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
struct AVCodecContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;

namespace V4L2Camera
{

struct MMapBuffer {
    void *start = nullptr;
    size_t length = 0;
};

void signalEventFd(int fd);

class EventFd
{
public:
    EventFd() = default;
    ~EventFd();

    EventFd(const EventFd &) = delete;
    EventFd &operator=(const EventFd &) = delete;

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

class FrameDecoder
{
public:
    FrameDecoder();
    ~FrameDecoder();

    FrameDecoder(const FrameDecoder &) = delete;
    FrameDecoder &operator=(const FrameDecoder &) = delete;

    bool configure(const CaptureMode &mode, QString *error);
    void reset();
    bool decode(const quint8 *data, size_t size, cv::Mat *out, QString *error);

private:
    CaptureMode m_mode;
    AVCodecContext *m_codecCtx;
    AVFrame *m_avFrame;
    AVPacket *m_packet;
    SwsContext *m_swsCtx;
    std::vector<uint8_t> m_mjpegInputBuffer;
    bool m_configured;

    bool decodeGrey(const quint8 *data, size_t size, cv::Mat *out, QString *error);
    bool decodeYuyv(const quint8 *data, size_t size, cv::Mat *out, QString *error);
    bool decodeMjpeg(const quint8 *data, size_t size, cv::Mat *out, QString *error);
};

void setFrameStreamMetadata(VariantDataStream *stream, const CaptureMode &mode);

} // namespace V4L2Camera
