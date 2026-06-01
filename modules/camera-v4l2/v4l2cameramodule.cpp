/*
 * Copyright (C) 2026 Matthias Klumpp <matthias@tenstral.net>
 *
 * Licensed under the GNU Lesser General Public License Version 3
 */

#include "v4l2cameramodule.h"

#include "datactl/frametype.h"
#include "utils/misc.h"
#include "v4l2settingsdialog.h"

#include <QApplication>
#include <QMessageBox>
#include <QMetaObject>
#include <QMutex>
#include <QMutexLocker>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <optional>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <vector>

SYNTALOS_MODULE(V4L2CameraModule)

namespace
{

struct MMapBuffer {
    void *start = nullptr;
    size_t length = 0;
};

struct ScheduledManualReapply {
    quint32 autoControlId = 0;
    std::chrono::steady_clock::time_point dueTime;
};

enum class ManualDependentReapplySource {
    ReportedValue,
    DesiredValue,
};

int xioctl(int fd, unsigned long request, void *arg)
{
    int ret;
    do {
        ret = ioctl(fd, request, arg);
    } while (ret == -1 && errno == EINTR);
    return ret;
}

QString errnoString()
{
    return QString::fromLocal8Bit(std::strerror(errno));
}

size_t estimatedFrameBufferSize(const V4L2Camera::CaptureMode &mode)
{
    if (mode.sizeImage > 0)
        return mode.sizeImage;
    if (mode.bytesPerLine > 0 && mode.height > 0)
        return static_cast<size_t>(mode.bytesPerLine) * static_cast<size_t>(mode.height);
    if (mode.width > 0 && mode.height > 0 && mode.cvType >= 0)
        return static_cast<size_t>(mode.width) * static_cast<size_t>(mode.height) * CV_ELEM_SIZE(mode.cvType);
    return 0;
}

int64_t decodedFrameStrideBytes(const V4L2Camera::CaptureMode &mode)
{
    if (mode.width <= 0 || mode.cvType < 0)
        return 0;
    return static_cast<int64_t>(mode.width) * static_cast<int64_t>(CV_ELEM_SIZE(mode.cvType));
}

quint32 requestedMMapBufferCount(const V4L2Camera::CaptureMode &mode)
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

bool requestMMapBuffers(
    int fd,
    const V4L2Camera::CaptureMode &mode,
    std::vector<MMapBuffer> *buffers,
    QString *error)
{
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

    buffers->resize(req.count);
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

        (*buffers)[i].length = buf.length;
        (*buffers)[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if ((*buffers)[i].start == MAP_FAILED) {
            (*buffers)[i].start = nullptr;
            if (error != nullptr)
                *error = QStringLiteral("mmap failed for V4L2 buffer %1: %2").arg(i).arg(errnoString());
            return false;
        }
    }

    return true;
}

void releaseMMapBuffers(int fd, std::vector<MMapBuffer> *buffers)
{
    for (auto &buffer : *buffers) {
        if (buffer.start != nullptr) {
            munmap(buffer.start, buffer.length);
            buffer.start = nullptr;
            buffer.length = 0;
        }
    }
    buffers->clear();

    if (fd >= 0) {
        v4l2_requestbuffers req = {};
        req.count = 0;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        xioctl(fd, VIDIOC_REQBUFS, &req);
    }
}

bool queueAllBuffers(int fd, size_t count, QString *error)
{
    for (quint32 i = 0; i < count; ++i) {
        v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            if (error != nullptr)
                *error = QStringLiteral("VIDIOC_QBUF failed for buffer %1: %2").arg(i).arg(errnoString());
            return false;
        }
    }
    return true;
}

nanoseconds_t v4l2TimestampToNsec(const timeval &timestamp)
{
    return nanoseconds_t(static_cast<int64_t>(timestamp.tv_sec) * 1000LL * 1000LL * 1000LL
                         + static_cast<int64_t>(timestamp.tv_usec) * 1000LL);
}

void logWarning(QuillLogger *log, const QString &message)
{
    LOG_WARNING(log, "{}", message.toStdString());
}

} // namespace

class V4L2CameraModule : public AbstractModule
{
    Q_OBJECT

private:
    V4L2SettingsDialog *m_settingsDialog;
    std::shared_ptr<DataStream<Frame>> m_outStream;

    std::atomic_bool m_stopped;
    std::atomic_int m_stopEventFd;
    std::atomic_bool m_warnedReadbackMismatch;
    std::atomic_bool m_warnedTimestampFallback;
    std::atomic_bool m_warnedExposurePriority;

    V4L2Camera::DeviceIdentity m_device;
    V4L2Camera::CaptureMode m_requestedMode;
    V4L2Camera::CaptureMode m_effectiveMode;
    QHash<quint32, V4L2Camera::ControlInfo> m_controlMap;

    QMutex m_controlMutex;
    QHash<quint32, qint64> m_desiredControlValues;
    QHash<quint32, qint64> m_pendingControlWrites;
    QHash<quint32, int> m_manualReapplyDelaysMs;
    QList<quint32> m_pendingButtonControls;
    QList<ScheduledManualReapply> m_scheduledManualReapplies;

public:
    explicit V4L2CameraModule(QObject *parent = nullptr)
        : AbstractModule(parent),
          m_settingsDialog(new V4L2SettingsDialog),
          m_stopped(true),
          m_stopEventFd(-1),
          m_warnedReadbackMismatch(false),
          m_warnedTimestampFallback(false),
          m_warnedExposurePriority(false)
    {
        m_outStream = registerOutputPort<Frame>(QStringLiteral("video"), QStringLiteral("Video"));
        addSettingsWindow(m_settingsDialog);

        connect(m_settingsDialog, &V4L2SettingsDialog::controlValueChanged, this, [this](quint32 id, qint64 value) {
            if (m_stopped.load())
                return;
            QMutexLocker locker(&m_controlMutex);
            m_desiredControlValues[id] = value;
            m_pendingControlWrites[id] = value;
        });
        connect(m_settingsDialog, &V4L2SettingsDialog::buttonControlTriggered, this, [this](quint32 id) {
            if (m_stopped.load())
                return;
            QMutexLocker locker(&m_controlMutex);
            m_pendingButtonControls.append(id);
        });
        connect(m_settingsDialog, &V4L2SettingsDialog::manualReapplyDelayChanged, this, [this](quint32 id, int delayMs) {
            QMutexLocker locker(&m_controlMutex);
            m_manualReapplyDelaysMs[id] = delayMs;
        });
    }

    ~V4L2CameraModule() override = default;

    void setName(const QString &name) override
    {
        AbstractModule::setName(name);
        m_settingsDialog->setWindowTitle(QStringLiteral("Settings for %1").arg(name));
    }

    ModuleDriverKind driver() const override
    {
        return ModuleDriverKind::THREAD_DEDICATED;
    }

    ModuleFeatures features() const override
    {
        return ModuleFeature::REALTIME | ModuleFeature::SHOW_SETTINGS;
    }

    bool prepare(const TestSubject &) override
    {
        m_warnedReadbackMismatch = false;
        m_warnedTimestampFallback = false;
        m_warnedExposurePriority = false;
        {
            QMutexLocker locker(&m_controlMutex);
            m_pendingControlWrites.clear();
            m_pendingButtonControls.clear();
            m_manualReapplyDelaysMs = m_settingsDialog->manualReapplyDelaysMs();
        }
        m_scheduledManualReapplies.clear();

        auto wantedDevice = m_settingsDialog->selectedDevice();
        QString matchWarning;
        const auto matchedDevice = V4L2Camera::findDevice(wantedDevice, &matchWarning);
        if (!matchedDevice.has_value()) {
            raiseError(matchWarning.isEmpty() ? QStringLiteral("No V4L2 camera selected.") : matchWarning);
            return false;
        }
        if (!matchWarning.isEmpty()) {
            logWarning(m_log, matchWarning);
            const auto answer = QMessageBox::warning(
                m_settingsDialog,
                QStringLiteral("V4L2 Camera Match"),
                matchWarning + QStringLiteral("\n\nContinue with this camera?"),
                QMessageBox::Yes | QMessageBox::No,
                QMessageBox::No);
            if (answer != QMessageBox::Yes)
                return false;
        }

        m_device = matchedDevice.value();
        m_requestedMode = m_settingsDialog->selectedMode();
        if (!m_requestedMode.isValid()) {
            raiseError(QStringLiteral("No valid V4L2 capture mode selected."));
            return false;
        }

        {
            QMutexLocker locker(&m_controlMutex);
            m_desiredControlValues = m_settingsDialog->desiredControlValues();
            m_manualReapplyDelaysMs = m_settingsDialog->manualReapplyDelaysMs();
        }

        V4L2Camera::Device device;
        QList<V4L2Camera::ControlInfo> controls;
        QString error;
        if (!configureDevice(device, &m_effectiveMode, &controls, &error, true)) {
            raiseError(error);
            return false;
        }

        V4L2Camera::FrameDecoder decoderCheck;
        if (!decoderCheck.configure(m_effectiveMode, &error)) {
            raiseError(error);
            return false;
        }

        updateControlMap(controls);
        m_settingsDialog->replaceControls(controls);
        m_settingsDialog->setEffectiveMode(m_effectiveMode);
        m_settingsDialog->setRunning(true);

        m_outStream->setMetadataValue("size", MetaSize(m_effectiveMode.width, m_effectiveMode.height));
        m_outStream->setMetadataValue("framerate", m_effectiveMode.fps());
        m_outStream->setMetadataValue("timeperframe_num", static_cast<int64_t>(m_effectiveMode.timeperframeNum));
        m_outStream->setMetadataValue("timeperframe_den", static_cast<int64_t>(m_effectiveMode.timeperframeDen));
        m_outStream->setMetadataValue("depth", static_cast<int64_t>(CV_MAT_DEPTH(m_effectiveMode.cvType)));
        m_outStream->setMetadataValue("has_color", CV_MAT_CN(m_effectiveMode.cvType) > 1);
        m_outStream->setMetadataValue("fourcc", m_effectiveMode.fourccString.toStdString());
        m_outStream->setMetadataValue("stride", decodedFrameStrideBytes(m_effectiveMode));
        m_outStream->setMetadataValue("v4l2_bytes_per_line", static_cast<int64_t>(m_effectiveMode.bytesPerLine));
        m_outStream->setMetadataValue("colorspace", static_cast<int64_t>(m_effectiveMode.colorspace));
        m_outStream->setMetadataValue("field", static_cast<int64_t>(m_effectiveMode.field));
        m_outStream->setMetadataValue("bayer_pattern", std::string("none"));
        m_outStream->start();

        statusMessage(QStringLiteral("Waiting."));
        return true;
    }

    void runThread(OptionalWaitCondition *waitCondition) override
    {
        m_stopped = false;

        V4L2Camera::Device device;
        QList<V4L2Camera::ControlInfo> controls;
        QString error;
        bool streaming = false;
        std::vector<MMapBuffer> buffers;
        auto clockSync = initClockSynchronizer(m_effectiveMode.fps());

        const int stopFd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
        m_stopEventFd = stopFd;

        auto cleanup = [&]() {
            if (streaming) {
                int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                xioctl(device.fd(), VIDIOC_STREAMOFF, &type);
                streaming = false;
            }
            releaseMMapBuffers(device.fd(), &buffers);
            if (stopFd >= 0) {
                m_stopEventFd = -1;
                ::close(stopFd);
            }
            safeStopSynchronizer(clockSync);
            QMetaObject::invokeMethod(m_settingsDialog, [this]() { m_settingsDialog->setRunning(false); }, Qt::QueuedConnection);
            m_stopped = true;
        };

        if (stopFd < 0) {
            raiseError(QStringLiteral("Unable to create V4L2 stop eventfd: %1").arg(errnoString()));
            cleanup();
            return;
        }

        if (!clockSync) {
            raiseError(QStringLiteral("Unable to set up clock synchronizer!"));
            cleanup();
            return;
        }
        clockSync->setStrategies(TimeSyncStrategy::SHIFT_TIMESTAMPS_FWD | TimeSyncStrategy::SHIFT_TIMESTAMPS_BWD);
        if (!clockSync->start()) {
            raiseError(QStringLiteral("Unable to start clock synchronizer!"));
            cleanup();
            return;
        }

        const auto preparedEffectiveMode = m_effectiveMode;
        V4L2Camera::CaptureMode runEffectiveMode;
        if (!configureDevice(device, &runEffectiveMode, &controls, &error, true)) {
            raiseError(error);
            cleanup();
            return;
        }
        // Stream metadata was committed in prepare(); abort if the second fd
        // reports different metadata-bearing mode fields before streaming.
        if (!effectiveModeMetadataMatches(preparedEffectiveMode, runEffectiveMode)) {
            raiseError(
                QStringLiteral("V4L2 effective mode changed between prepare and capture. Prepared: %1. Capture: %2.")
                    .arg(effectiveModeMetadataSummary(preparedEffectiveMode), effectiveModeMetadataSummary(runEffectiveMode)));
            cleanup();
            return;
        }
        m_effectiveMode = runEffectiveMode;
        updateControlMap(controls);

        V4L2Camera::FrameDecoder decoder;
        if (!decoder.configure(m_effectiveMode, &error)) {
            raiseError(error);
            cleanup();
            return;
        }

        if (!requestMMapBuffers(device.fd(), m_effectiveMode, &buffers, &error)
            || !queueAllBuffers(device.fd(), buffers.size(), &error)) {
            raiseError(error);
            cleanup();
            return;
        }

        statusMessage(QStringLiteral("%1").arg(m_device.displayName()));
        waitCondition->wait(this);
        if (!m_running) {
            cleanup();
            return;
        }

        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(device.fd(), VIDIOC_STREAMON, &type) < 0) {
            raiseError(QStringLiteral("VIDIOC_STREAMON failed: %1").arg(errnoString()));
            cleanup();
            return;
        }
        streaming = true;

        uint64_t frameCount = 0;
        uint64_t invalidFrameCount = 0;
        uint64_t droppedFrameCount = 0;
        uint64_t sequenceGapCount = 0;
        std::optional<quint32> lastSequence;
        std::optional<nanoseconds_t> lastDriverTimestamp;
        std::optional<microseconds_t> lastFrameTime;
        nanoseconds_t driverToMasterOffset{0};
        bool driverOffsetKnown = false;
        bool useDriverTimestamps = true;
        const auto expectedFrameNs = nanoseconds_t(
            static_cast<int64_t>((1'000'000'000.0 * m_effectiveMode.timeperframeNum) / m_effectiveMode.timeperframeDen));

        auto lastStatusTime = std::chrono::steady_clock::now();
        uint64_t lastStatusFrame = 0;

        while (m_running) {
            applyPendingControlWrites(device);

            pollfd fds[2] = {};
            fds[0].fd = device.fd();
            fds[0].events = POLLIN | POLLERR | POLLHUP;
            fds[1].fd = stopFd;
            fds[1].events = POLLIN;

            const int pollResult = poll(fds, 2, controlPollTimeoutMs());
            if (pollResult < 0) {
                if (errno == EINTR)
                    continue;
                raiseError(QStringLiteral("poll failed while waiting for V4L2 frame: %1").arg(errnoString()));
                break;
            }
            if (pollResult == 0)
                continue;
            if ((fds[1].revents & POLLIN) != 0)
                break;
            if ((fds[0].revents & (POLLHUP | POLLNVAL)) != 0) {
                raiseError(QStringLiteral("V4L2 camera disappeared or became invalid."));
                break;
            }

            while (m_running) {
                v4l2_buffer buf = {};
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;

                if (xioctl(device.fd(), VIDIOC_DQBUF, &buf) < 0) {
                    if (errno == EAGAIN)
                        break;
                    if (errno == ENODEV || errno == EIO) {
                        // Keep EIO fatal for now. V4L2 permits drivers to report transient capture
                        // errors either as EIO here or as V4L2_BUF_FLAG_ERROR on a dequeued buffer.
                        // If EIO becomes common with supported cameras, add bounded stream recovery:
                        // STREAMOFF, requeue all mmap buffers, STREAMON, reset sequence/timestamp state,
                        // and abort only after repeated recovery failures.
                        raiseError(QStringLiteral("V4L2 camera returned a capture error: %1").arg(errnoString()));
                        m_running = false;
                        break;
                    }
                    invalidFrameCount++;
                    logWarning(m_log, QStringLiteral("VIDIOC_DQBUF failed: %1").arg(errnoString()));
                    if (invalidFrameCount > 16) {
                        raiseError(QStringLiteral("Too many V4L2 frame dequeue failures."));
                        m_running = false;
                    }
                    break;
                }

                const auto dequeueMasterNs = m_syTimer->timeSinceStartNsec();
                if (buf.index >= buffers.size()) {
                    raiseError(QStringLiteral("V4L2 driver returned an invalid buffer index."));
                    m_running = false;
                    break;
                }

                if (lastSequence.has_value()) {
                    const quint32 expected = *lastSequence + 1;
                    if (buf.sequence != expected) {
                        const quint32 gap = buf.sequence - expected;
                        sequenceGapCount += gap == 0 ? 1 : gap;
                        logWarning(
                            m_log,
                            QStringLiteral("V4L2 sequence gap detected: expected %1, got %2 (total gaps: %3).")
                                .arg(expected)
                                .arg(buf.sequence)
                                .arg(sequenceGapCount));
                    }
                }
                lastSequence = buf.sequence;

                if ((buf.flags & V4L2_BUF_FLAG_ERROR) != 0) {
                    droppedFrameCount++;
                    invalidFrameCount++;
                    logWarning(m_log, QStringLiteral("Dropping V4L2 frame marked with V4L2_BUF_FLAG_ERROR."));
                } else {
                    cv::Mat image;
                    const auto bytesUsed = buf.bytesused == 0 ? buffers[buf.index].length : static_cast<size_t>(buf.bytesused);
                    const auto *data = static_cast<const quint8 *>(buffers[buf.index].start);
                    if (!decoder.decode(data, bytesUsed, &image, &error)) {
                        invalidFrameCount++;
                        logWarning(m_log, QStringLiteral("Failed to decode V4L2 frame: %1").arg(error));
                    } else if (image.cols != m_effectiveMode.width || image.rows != m_effectiveMode.height) {
                        invalidFrameCount++;
                        logWarning(
                            m_log,
                            QStringLiteral("Decoded V4L2 frame has unexpected dimensions %1x%2.")
                                .arg(image.cols)
                                .arg(image.rows));
                    } else {
                        const bool timestampMonotonic =
                            (buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
                        const auto driverTimestamp = v4l2TimestampToNsec(buf.timestamp);
                        microseconds_t frameTime = nsecToUsec(dequeueMasterNs);

                        if (timestampMonotonic && useDriverTimestamps && driverTimestamp.count() > 0) {
                            if (!driverOffsetKnown) {
                                driverToMasterOffset = dequeueMasterNs - driverTimestamp;
                                driverOffsetKnown = true;
                            }

                            bool sane = true;
                            if (lastDriverTimestamp.has_value()) {
                                const auto delta = driverTimestamp - *lastDriverTimestamp;
                                if (delta.count() <= 0 || (expectedFrameNs.count() > 0 && delta > expectedFrameNs * 20))
                                    sane = false;
                            }

                            if (sane) {
                                frameTime = nsecToUsec(driverTimestamp + driverToMasterOffset);
                                clockSync->processTimestamp(frameTime, nsecToUsec(driverTimestamp));
                                lastDriverTimestamp = driverTimestamp;
                            } else {
                                useDriverTimestamps = false;
                                warnOnce(
                                    &m_warnedTimestampFallback,
                                    QStringLiteral("V4L2 Timestamp Fallback"),
                                    QStringLiteral("Driver timestamps became implausible; using Syntalos dequeue time."));
                            }
                        }

                        if (!timestampMonotonic || !useDriverTimestamps) {
                            frameTime = nsecToUsec(dequeueMasterNs);
                            clockSync->processTimestamp(frameTime, frameTime);
                        }

                        if (lastFrameTime.has_value() && frameTime <= *lastFrameTime)
                            frameTime = *lastFrameTime + microseconds_t(1);
                        lastFrameTime = frameTime;

                        m_outStream->push(Frame(image, frameCount++, frameTime));
                        invalidFrameCount = 0;
                    }
                }

                if (xioctl(device.fd(), VIDIOC_QBUF, &buf) < 0) {
                    raiseError(QStringLiteral("VIDIOC_QBUF failed after frame processing: %1").arg(errnoString()));
                    m_running = false;
                    break;
                }

                if (invalidFrameCount > 16) {
                    raiseError(QStringLiteral("Too many invalid V4L2 frames."));
                    m_running = false;
                    break;
                }
            }

            const auto now = std::chrono::steady_clock::now();
            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastStatusTime);
            if (elapsed.count() >= 2000) {
                const auto frames = frameCount - lastStatusFrame;
                const double fps = static_cast<double>(frames) * 1000.0 / static_cast<double>(elapsed.count());
                statusMessage(
                    QStringLiteral("Capturing %1 fps, sequence gaps %2, dropped %3")
                        .arg(fps, 0, 'f', 1)
                        .arg(sequenceGapCount)
                        .arg(droppedFrameCount));
                lastStatusFrame = frameCount;
                lastStatusTime = now;
            }
        }

        cleanup();
    }

    void stop() override
    {
        statusMessage(QStringLiteral("Cleaning up..."));
        m_running = false;

        const int stopFd = m_stopEventFd.load();
        if (stopFd >= 0) {
            uint64_t value = 1;
            const auto unused = ::write(stopFd, &value, sizeof(value));
            Q_UNUSED(unused);
        }

        while (!m_stopped.load())
            appProcessEvents();

        m_settingsDialog->setRunning(false);
        statusMessage(QStringLiteral("Camera stopped."));
        AbstractModule::stop();
    }

    void serializeSettings(const QString &, QVariantHash &settings, QByteArray &) override
    {
        m_settingsDialog->serializeSettings(settings);
    }

    bool loadSettings(const QString &, const QVariantHash &settings, const QByteArray &) override
    {
        m_settingsDialog->loadSettings(settings);
        return true;
    }

protected:
    void usbHotplugEvent(UsbHotplugEventKind kind) override
    {
        if (!m_stopped)
            return;
        if (kind == UsbHotplugEventKind::DEVICE_ARRIVED || kind == UsbHotplugEventKind::DEVICES_CHANGE
            || kind == UsbHotplugEventKind::DEVICE_LEFT)
            m_settingsDialog->refreshDevices();
    }

private:
    bool configureDevice(
        V4L2Camera::Device &device,
        V4L2Camera::CaptureMode *effectiveMode,
        QList<V4L2Camera::ControlInfo> *controls,
        QString *error,
        bool applyControls)
    {
        if (!device.open(m_device.devicePath, error))
            return false;

        if (!device.applyCaptureMode(m_requestedMode, effectiveMode, error))
            return false;

        auto queriedControls = device.queryControls(nullptr);
        if (applyControls) {
            applyDesiredControls(device, &queriedControls);

            queriedControls = device.queryControls(nullptr);
            if (!device.applyCaptureMode(m_requestedMode, effectiveMode, error))
                return false;
        }

        if (controls != nullptr)
            *controls = queriedControls;
        return true;
    }

    void applyDesiredControls(V4L2Camera::Device &device, QList<V4L2Camera::ControlInfo> *controls)
    {
        QHash<quint32, qint64> desired;
        QHash<quint32, int> manualReapplyDelaysMs;
        {
            QMutexLocker locker(&m_controlMutex);
            desired = m_desiredControlValues;
            manualReapplyDelaysMs = m_manualReapplyDelaysMs;
        }

        if (desired.contains(V4L2_CID_EXPOSURE_AUTO_PRIORITY) && desired.value(V4L2_CID_EXPOSURE_AUTO_PRIORITY) != 0) {
            warnOnce(
                &m_warnedExposurePriority,
                QStringLiteral("Variable FPS Control"),
                QStringLiteral(
                    "V4L2 exposure auto priority is enabled. This can allow variable frame timing; actual FPS will be read back."));
        }

        auto sortedControls = *controls;
        updateControlMap(sortedControls);
        const auto autoIds = V4L2Camera::autoControlIds();
        const auto dependencyTable = V4L2Camera::autoDependencyTable();
        std::sort(sortedControls.begin(), sortedControls.end(), [&autoIds](const auto &a, const auto &b) {
            const auto aPriority = autoIds.contains(a.id) || a.id == V4L2_CID_EXPOSURE_AUTO_PRIORITY ? 0 : 1;
            const auto bPriority = autoIds.contains(b.id) || b.id == V4L2_CID_EXPOSURE_AUTO_PRIORITY ? 0 : 1;
            if (aPriority != bPriority)
                return aPriority < bPriority;
            return a.id < b.id;
        });

        for (const auto &listedControl : sortedControls) {
            const auto control = m_controlMap.value(listedControl.id, listedControl);
            if (!desired.contains(control.id))
                continue;
            if (!control.restorable())
                continue;
            if (V4L2Camera::isManualDependentActive(control.id, desired))
                continue;

            const auto requestedValue = desired.value(control.id);
            const auto result = device.setControlValue(control, requestedValue);
            if (!result.success) {
                logWarning(m_log, result.error);
                continue;
            }

            desired[control.id] = result.readbackValue;
            if (result.changedByDevice) {
                warnOnce(
                    &m_warnedReadbackMismatch,
                    QStringLiteral("V4L2 Control Readback"),
                    QStringLiteral("Control '%1' read back as %2 after requesting %3.")
                        .arg(control.name)
                        .arg(result.readbackValue)
                        .arg(result.requestedValue));
            }

            if (dependencyTable.contains(control.id) && !V4L2Camera::autoControlEnabled(control.id, result.readbackValue)) {
                const auto delayMs = manualReapplyDelaysMs.value(control.id, 0);
                if (delayMs == 0) {
                    reapplyManualDependentControls(
                        device,
                        control,
                        dependencyTable,
                        &desired,
                        nullptr,
                        ManualDependentReapplySource::DesiredValue);
                } else if (delayMs > 0) {
                    // During prepare/startup no frames are being dequeued yet, so waiting here honors saved
                    // startup control state without stalling live acquisition.
                    std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
                    reapplyManualDependentControls(
                        device,
                        control,
                        dependencyTable,
                        &desired,
                        nullptr,
                        ManualDependentReapplySource::DesiredValue);
                }
            }
        }

        {
            QMutexLocker locker(&m_controlMutex);
            m_desiredControlValues = desired;
        }
    }

    static bool sameMenuStructure(
        const QList<V4L2Camera::MenuEntry> &a,
        const QList<V4L2Camera::MenuEntry> &b)
    {
        if (a.size() != b.size())
            return false;

        for (int i = 0; i < a.size(); ++i) {
            if (a.at(i).value != b.at(i).value || a.at(i).name != b.at(i).name)
                return false;
        }

        return true;
    }

    static bool sameControlStructure(
        const V4L2Camera::ControlInfo &a,
        const V4L2Camera::ControlInfo &b)
    {
        return a.id == b.id && a.name == b.name && a.controlClass == b.controlClass && a.type == b.type
            && a.minimum == b.minimum && a.maximum == b.maximum && a.step == b.step
            && a.defaultValue == b.defaultValue && a.supported == b.supported && a.isDisabled() == b.isDisabled()
            && sameMenuStructure(a.menu, b.menu);
    }

    static bool controlInventoryChanged(
        const QHash<quint32, V4L2Camera::ControlInfo> &oldControls,
        const QList<V4L2Camera::ControlInfo> &newControls)
    {
        QSet<quint32> seenIds;
        for (const auto &control : newControls) {
            if (control.isClassMarker())
                continue;

            seenIds.insert(control.id);
            const auto oldIt = oldControls.constFind(control.id);
            if (oldIt == oldControls.constEnd())
                return true;
            if (!sameControlStructure(oldIt.value(), control))
                return true;
        }

        for (auto it = oldControls.constBegin(); it != oldControls.constEnd(); ++it) {
            if (!seenIds.contains(it.key()))
                return true;
        }

        return false;
    }

    static QHash<quint32, V4L2Camera::ControlInfo> controlsById(const QList<V4L2Camera::ControlInfo> &controls)
    {
        QHash<quint32, V4L2Camera::ControlInfo> result;
        for (const auto &control : controls) {
            if (!control.isClassMarker())
                result.insert(control.id, control);
        }
        return result;
    }

    static QList<quint32> sortedControlWriteIds(const QHash<quint32, qint64> &pending)
    {
        auto ids = pending.keys();
        const auto autoIds = V4L2Camera::autoControlIds();
        std::sort(ids.begin(), ids.end(), [&autoIds](quint32 a, quint32 b) {
            const auto aPriority = autoIds.contains(a) || a == V4L2_CID_EXPOSURE_AUTO_PRIORITY ? 0 : 1;
            const auto bPriority = autoIds.contains(b) || b == V4L2_CID_EXPOSURE_AUTO_PRIORITY ? 0 : 1;
            if (aPriority != bPriority)
                return aPriority < bPriority;
            return a < b;
        });
        return ids;
    }

    int controlPollTimeoutMs() const
    {
        constexpr int defaultTimeoutMs = 1000;
        if (m_scheduledManualReapplies.isEmpty())
            return defaultTimeoutMs;

        const auto now = std::chrono::steady_clock::now();
        auto nextDue = m_scheduledManualReapplies.constFirst().dueTime;
        for (const auto &pending : m_scheduledManualReapplies)
            nextDue = std::min(nextDue, pending.dueTime);

        if (nextDue <= now)
            return 0;

        auto remainingMs = std::chrono::duration_cast<std::chrono::milliseconds>(nextDue - now).count();
        if (remainingMs < 1)
            remainingMs = 1;
        return static_cast<int>(std::min<qint64>(defaultTimeoutMs, remainingMs));
    }

    void scheduleManualDependentReapply(quint32 autoControlId, int delayMs)
    {
        for (int i = m_scheduledManualReapplies.size() - 1; i >= 0; --i) {
            if (m_scheduledManualReapplies.at(i).autoControlId == autoControlId)
                m_scheduledManualReapplies.removeAt(i);
        }

        if (delayMs <= 0)
            return;

        ScheduledManualReapply pending;
        pending.autoControlId = autoControlId;
        pending.dueTime = std::chrono::steady_clock::now() + std::chrono::milliseconds(delayMs);
        m_scheduledManualReapplies.append(pending);
    }

    void reapplyManualDependentControls(
        V4L2Camera::Device &device,
        const V4L2Camera::ControlInfo &autoControl,
        const QHash<quint32, QList<quint32>> &dependencyTable,
        QHash<quint32, qint64> *desired,
        QSet<quint32> *affectedRefreshIds,
        ManualDependentReapplySource valueSource)
    {
        const auto dependentIds = dependencyTable.value(autoControl.id);
        if (dependentIds.isEmpty())
            return;

        // Some drivers expose dependent controls as writable before the hardware
        // has physically settled after auto mode is disabled. Delayed callers enter
        // this function only after their configured wait, then query fresh control
        // state before writing either saved desired values or reported manual values.
        auto controls = device.queryControls(nullptr);
        if (controls.isEmpty())
            return;
        updateControlMap(controls);
        const auto queriedControls = controlsById(controls);

        for (const auto dependentId : dependentIds) {
            if (!m_controlMap.contains(dependentId))
                continue;

            if (affectedRefreshIds != nullptr)
                affectedRefreshIds->insert(dependentId);

            const auto dependentIt = queriedControls.constFind(dependentId);
            if (dependentIt == queriedControls.constEnd())
                continue;

            const auto dependent = dependentIt.value();
            if (!dependent.canRead() || !dependent.canWrite())
                continue;

            // Some UVC cameras, e.g. Logitech Webcam C930e, can report a manual
            // value after auto mode is disabled but keep using a different physical
            // state until a manual value is written. During startup restore, keep
            // saved desired values authoritative instead of replacing them with the
            // driver's transient post-auto readback.
            const bool useDesiredValue = valueSource == ManualDependentReapplySource::DesiredValue && desired != nullptr
                && desired->contains(dependent.id);
            const auto writeValue = useDesiredValue ? desired->value(dependent.id) : dependent.currentValue;
            const auto result = device.setControlValue(dependent, writeValue);
            if (!result.success) {
                logWarning(m_log, result.error);
                continue;
            }

            if (desired != nullptr)
                (*desired)[dependent.id] = result.readbackValue;
            if (m_controlMap.contains(dependent.id))
                m_controlMap[dependent.id].currentValue = result.readbackValue;
        }
    }

    void applyDueManualReapplies(
        V4L2Camera::Device &device,
        const QHash<quint32, QList<quint32>> &dependencyTable,
        const QHash<quint32, int> &manualReapplyDelaysMs,
        QHash<quint32, qint64> *desired,
        QSet<quint32> *affectedRefreshIds)
    {
        if (m_scheduledManualReapplies.isEmpty())
            return;

        const auto now = std::chrono::steady_clock::now();
        QList<ScheduledManualReapply> due;
        QList<ScheduledManualReapply> remaining;
        for (const auto &pending : m_scheduledManualReapplies) {
            if (pending.dueTime <= now)
                due.append(pending);
            else
                remaining.append(pending);
        }
        m_scheduledManualReapplies = remaining;

        for (const auto &pending : due) {
            if (manualReapplyDelaysMs.value(pending.autoControlId, 0) < 0)
                continue;
            if (!m_controlMap.contains(pending.autoControlId))
                continue;

            const auto autoControl = m_controlMap.value(pending.autoControlId);
            if (V4L2Camera::autoControlEnabled(
                    autoControl.id,
                    desired->value(autoControl.id, autoControl.currentValue)))
                continue;

            reapplyManualDependentControls(
                device,
                autoControl,
                dependencyTable,
                desired,
                affectedRefreshIds,
                ManualDependentReapplySource::ReportedValue);
        }
    }

    void applyPendingControlWrites(V4L2Camera::Device &device)
    {
        QHash<quint32, qint64> pending;
        QList<quint32> pendingButtons;
        QHash<quint32, qint64> desired;
        QHash<quint32, int> manualReapplyDelaysMs;
        {
            QMutexLocker locker(&m_controlMutex);
            pending = m_pendingControlWrites;
            m_pendingControlWrites.clear();
            pendingButtons = m_pendingButtonControls;
            m_pendingButtonControls.clear();
            desired = m_desiredControlValues;
            manualReapplyDelaysMs = m_manualReapplyDelaysMs;
        }

        const auto hasDueManualReapplies = std::any_of(
            m_scheduledManualReapplies.constBegin(),
            m_scheduledManualReapplies.constEnd(),
            [](const auto &pending) { return pending.dueTime <= std::chrono::steady_clock::now(); });
        if (pending.isEmpty() && pendingButtons.isEmpty() && !hasDueManualReapplies)
            return;

        QSet<quint32> affectedRefreshIds;
        const auto dependencyTable = V4L2Camera::autoDependencyTable();
        for (const auto id : sortedControlWriteIds(pending)) {
            if (!m_controlMap.contains(id))
                continue;

            const auto control = m_controlMap.value(id);
            if (V4L2Camera::isManualDependentActive(control.id, desired))
                continue;

            const auto result = device.setControlValue(control, pending.value(id));
            if (!result.success) {
                logWarning(m_log, result.error);
                continue;
            }

            desired[control.id] = result.readbackValue;
            m_controlMap[control.id].currentValue = result.readbackValue;
            QMetaObject::invokeMethod(
                m_settingsDialog,
                [this, id = control.id, value = result.readbackValue]() { m_settingsDialog->updateControlReadback(id, value); },
                Qt::QueuedConnection);

            if (result.changedByDevice) {
                warnOnce(
                    &m_warnedReadbackMismatch,
                    QStringLiteral("V4L2 Control Readback"),
                    QStringLiteral("Control '%1' read back as %2 after requesting %3.")
                        .arg(control.name)
                        .arg(result.readbackValue)
                        .arg(result.requestedValue));
            }

            affectedRefreshIds.insert(control.id);
            const auto dependentIds = dependencyTable.value(control.id);
            for (const auto dependentId : dependentIds) {
                if (m_controlMap.contains(dependentId))
                    affectedRefreshIds.insert(dependentId);
            }
            if (!dependentIds.isEmpty()) {
                scheduleManualDependentReapply(control.id, 0);
                if (!V4L2Camera::autoControlEnabled(control.id, result.readbackValue)) {
                    const auto delayMs = manualReapplyDelaysMs.value(control.id, 0);
                    if (delayMs == 0)
                        reapplyManualDependentControls(
                            device,
                            control,
                            dependencyTable,
                            &desired,
                            &affectedRefreshIds,
                            ManualDependentReapplySource::ReportedValue);
                    else if (delayMs > 0)
                        scheduleManualDependentReapply(control.id, delayMs);
                }
            }
        }

        for (const auto id : pendingButtons) {
            if (!m_controlMap.contains(id))
                continue;

            const auto control = m_controlMap.value(id);
            QString error;
            if (!device.triggerButtonControl(control, &error))
                logWarning(m_log, error);
        }

        applyDueManualReapplies(device, dependencyTable, manualReapplyDelaysMs, &desired, &affectedRefreshIds);

        {
            QMutexLocker locker(&m_controlMutex);
            m_desiredControlValues = desired;
        }

        if (!affectedRefreshIds.isEmpty()) {
            auto controls = device.queryControls(nullptr);
            const bool inventoryChanged = controlInventoryChanged(m_controlMap, controls);
            updateControlMap(controls);
            {
                QMutexLocker locker(&m_controlMutex);
                if (inventoryChanged) {
                    for (const auto &control : controls) {
                        if (!control.isClassMarker())
                            m_desiredControlValues[control.id] = control.currentValue;
                    }
                } else {
                    for (const auto &control : controls) {
                        if (affectedRefreshIds.contains(control.id))
                            m_desiredControlValues[control.id] = control.currentValue;
                    }
                }
            }
            if (inventoryChanged) {
                QMetaObject::invokeMethod(
                    m_settingsDialog,
                    [this, controls]() { m_settingsDialog->replaceControls(controls); },
                    Qt::QueuedConnection);
            } else {
                QMetaObject::invokeMethod(
                    m_settingsDialog,
                    [this, controls, affectedRefreshIds]() { m_settingsDialog->updateControls(controls, affectedRefreshIds); },
                    Qt::QueuedConnection);
            }
        }
    }

    void updateControlMap(const QList<V4L2Camera::ControlInfo> &controls)
    {
        m_controlMap.clear();
        for (const auto &control : controls) {
            if (!control.isClassMarker())
                m_controlMap.insert(control.id, control);
        }
    }

    static bool effectiveModeMetadataMatches(
        const V4L2Camera::CaptureMode &preparedMode,
        const V4L2Camera::CaptureMode &runMode)
    {
        return preparedMode.fourcc == runMode.fourcc && preparedMode.width == runMode.width
            && preparedMode.height == runMode.height && preparedMode.timeperframeNum == runMode.timeperframeNum
            && preparedMode.timeperframeDen == runMode.timeperframeDen && preparedMode.cvType == runMode.cvType
            && preparedMode.bytesPerLine == runMode.bytesPerLine && preparedMode.colorspace == runMode.colorspace
            && preparedMode.field == runMode.field;
    }

    static QString effectiveModeMetadataSummary(const V4L2Camera::CaptureMode &mode)
    {
        return QStringLiteral("%1 %2x%3 tpf %4/%5 stride %6 colorspace %7 field %8 cvType %9")
            .arg(mode.fourccString)
            .arg(mode.width)
            .arg(mode.height)
            .arg(mode.timeperframeNum)
            .arg(mode.timeperframeDen)
            .arg(mode.bytesPerLine)
            .arg(mode.colorspace)
            .arg(mode.field)
            .arg(mode.cvType);
    }

    void warnOnce(std::atomic_bool *flag, const QString &title, const QString &message)
    {
        if (flag->exchange(true))
            return;

        logWarning(m_log, message);
        QMetaObject::invokeMethod(
            m_settingsDialog,
            [this, title, message]() { QMessageBox::warning(m_settingsDialog, title, message); },
            Qt::QueuedConnection);
    }
};

QString V4L2CameraModuleInfo::id() const
{
    return QStringLiteral("camera-v4l2");
}

QString V4L2CameraModuleInfo::name() const
{
    return QStringLiteral("V4L2 Camera");
}

QString V4L2CameraModuleInfo::summary() const
{
    return QStringLiteral("Capture frames from V4L2 cameras using native streaming I/O.");
}

QString V4L2CameraModuleInfo::description() const
{
    return QStringLiteral("Capture GREY, YUYV, and MJPEG frames from Linux V4L2 devices with native mmap buffers and V4L2 controls.");
}

QString V4L2CameraModuleInfo::authors() const
{
    return QStringLiteral("2026 Matthias Klumpp");
}

QString V4L2CameraModuleInfo::license() const
{
    return QStringLiteral("LGPL-3.0+");
}

ModuleCategories V4L2CameraModuleInfo::categories() const
{
    return ModuleCategory::DEVICES;
}

QColor V4L2CameraModuleInfo::color() const
{
    return QColor::fromRgba(qRgba(29, 158, 246, 180)).darker();
}

AbstractModule *V4L2CameraModuleInfo::createModule(QObject *parent)
{
    return new V4L2CameraModule(parent);
}

#include "v4l2cameramodule.moc"
