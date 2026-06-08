/*
 * Copyright (C) 2026 Matthias Klumpp <matthias@tenstral.net>
 *
 * Licensed under the GNU Lesser General Public License Version 3
 */

#include "v4l2cameramodule.h"

#include "datactl/frametype.h"
#include "syscopeguard.h"
#include "utils/misc.h"
#include "v4l2capture.h"
#include "v4l2controlapplier.h"
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
#include <ctime>
#include <limits>
#include <linux/videodev2.h>
#include <memory>
#include <optional>
#include <poll.h>

SYNTALOS_MODULE(V4L2CameraModule)

namespace
{

nanoseconds_t v4l2TimestampToNsec(const timeval &timestamp)
{
    return nanoseconds_t(static_cast<int64_t>(timestamp.tv_sec) * 1000LL * 1000LL * 1000LL
                         + static_cast<int64_t>(timestamp.tv_usec) * 1000LL);
}

bool currentClockMonotonicNsec(nanoseconds_t *timestamp, QString *error)
{
    timespec ts = {};
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        if (error != nullptr)
            *error = QStringLiteral("clock_gettime(CLOCK_MONOTONIC) failed: %1").arg(V4L2Camera::errnoString());
        return false;
    }

    *timestamp = nanoseconds_t(static_cast<int64_t>(ts.tv_sec) * 1000LL * 1000LL * 1000LL
                               + static_cast<int64_t>(ts.tv_nsec));
    return true;
}

bool syntalosClockCanUseV4L2Monotonic(QString *error)
{
#ifdef SYNTALOS_USE_RAW_MONOTONIC_TIME
    if (error != nullptr) {
        *error = QStringLiteral(
            "camera-v4l2 requires Syntalos master time to use CLOCK_MONOTONIC because V4L2 "
            "V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC timestamps are CLOCK_MONOTONIC. This build uses "
            "CLOCK_MONOTONIC_RAW, so a static timestamp offset would drift during long acquisitions.");
    }
    return false;
#else
    (void)error;
    return true;
#endif
}

QString timestampTypeName(quint32 flags)
{
    switch (flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) {
    case V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN:
        return QStringLiteral("unknown");
    case V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC:
        return QStringLiteral("monotonic");
    case V4L2_BUF_FLAG_TIMESTAMP_COPY:
        return QStringLiteral("copy");
    default:
        return QStringLiteral("unrecognized");
    }
}

QString timestampSourceName(quint32 flags)
{
    switch (flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) {
    case V4L2_BUF_FLAG_TSTAMP_SRC_EOF:
        return QStringLiteral("eof");
    case V4L2_BUF_FLAG_TSTAMP_SRC_SOE:
        return QStringLiteral("soe");
    default:
        return QStringLiteral("unrecognized");
    }
}

bool acceptedTimestampSource(quint32 flags)
{
    const auto source = flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
    return source == V4L2_BUF_FLAG_TSTAMP_SRC_EOF || source == V4L2_BUF_FLAG_TSTAMP_SRC_SOE;
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
    std::atomic_int m_controlEventFd;
    std::atomic_bool m_warnedReadbackMismatch;
    std::atomic_bool m_warnedExposurePriority;

    V4L2Camera::DeviceIdentity m_device;
    V4L2Camera::CaptureMode m_requestedMode;
    V4L2Camera::CaptureMode m_effectiveMode;
    QMutex m_deviceMutex;
    std::unique_ptr<V4L2Camera::Device> m_preparedDevice;
    V4L2Camera::ControlApplier m_controlApplier;

    QMutex m_controlMutex;
    QHash<quint32, qint64> m_desiredControlValues;
    QHash<quint32, qint64> m_pendingControlWrites;
    QHash<quint32, int> m_manualReapplyDelaysMs;
    QList<quint32> m_pendingButtonControls;
    bool m_forceFocusAutoCycleOnRestore;

public:
    explicit V4L2CameraModule(QObject *parent = nullptr)
        : AbstractModule(parent),
          m_settingsDialog(new V4L2SettingsDialog),
          m_stopped(true),
          m_stopEventFd(-1),
          m_controlEventFd(-1),
          m_warnedReadbackMismatch(false),
          m_warnedExposurePriority(false),
          m_forceFocusAutoCycleOnRestore(false)
    {
        m_outStream = registerOutputPort<Frame>(QStringLiteral("video"), QStringLiteral("Video"));
        addSettingsWindow(m_settingsDialog);

        connect(m_settingsDialog, &V4L2SettingsDialog::controlValueChanged, this, [this](quint32 id, qint64 value) {
            if (m_stopped.load())
                return;
            {
                QMutexLocker locker(&m_controlMutex);
                m_desiredControlValues[id] = value;
                m_pendingControlWrites[id] = value;
            }
            wakeControlThread();
        });
        connect(m_settingsDialog, &V4L2SettingsDialog::buttonControlTriggered, this, [this](quint32 id) {
            if (m_stopped.load())
                return;
            {
                QMutexLocker locker(&m_controlMutex);
                m_pendingButtonControls.append(id);
            }
            wakeControlThread();
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
        m_warnedExposurePriority = false;
        {
            QMutexLocker locker(&m_deviceMutex);
            m_preparedDevice.reset();
        }
        {
            QMutexLocker locker(&m_controlMutex);
            m_pendingControlWrites.clear();
            m_pendingButtonControls.clear();
        }
        m_controlApplier.clearScheduledManualReapplies();

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
            m_forceFocusAutoCycleOnRestore = m_settingsDialog->forceFocusAutoCycleOnRestore();
        }

        auto preparedDevice = std::make_unique<V4L2Camera::Device>();
        QString error;
        if (!configureCapture(*preparedDevice, m_effectiveMode, &error)) {
            raiseError(error);
            return false;
        }

        V4L2Camera::FrameDecoder decoderCheck;
        if (!decoderCheck.configure(m_effectiveMode, &error)) {
            raiseError(error);
            return false;
        }

        {
            QMutexLocker locker(&m_deviceMutex);
            m_preparedDevice = std::move(preparedDevice);
        }
        m_settingsDialog->setEffectiveMode(m_effectiveMode);
        m_settingsDialog->setRunning(true);

        V4L2Camera::setFrameStreamMetadata(m_outStream.get(), m_effectiveMode);
        m_outStream->start();

        statusMessage(QStringLiteral("Waiting."));
        return true;
    }

    void runThread(OptionalWaitCondition *waitCondition) override
    {
        m_stopped = false;

        QString error;
        V4L2Camera::CaptureStreamGuard streamGuard;
        V4L2Camera::MMapBufferPool buffers;
        V4L2Camera::EventFd stopEvent;
        V4L2Camera::EventFd controlEvent;
        std::unique_ptr<V4L2Camera::Device> device;
        {
            QMutexLocker locker(&m_deviceMutex);
            device = std::move(m_preparedDevice);
        }

        auto cleanup = syScopeGuard([&]() {
            m_stopEventFd = -1;
            m_controlEventFd = -1;
            streamGuard.stop();
            buffers.release();
            device.reset();
            QMetaObject::invokeMethod(m_settingsDialog, [this]() { m_settingsDialog->setRunning(false); }, Qt::QueuedConnection);
            m_stopped = true;
        });

        if (!stopEvent.open(&error)) {
            raiseError(QStringLiteral("Unable to create V4L2 stop eventfd: %1").arg(error));
            return;
        }
        if (!controlEvent.open(&error)) {
            raiseError(QStringLiteral("Unable to create V4L2 control eventfd: %1").arg(error));
            return;
        }
        m_stopEventFd = stopEvent.fd();
        m_controlEventFd = controlEvent.fd();

        if (device == nullptr || !device->isOpen()) {
            raiseError(QStringLiteral("No prepared V4L2 camera device is available for capture."));
            return;
        }

        V4L2Camera::FrameDecoder decoder;
        if (!decoder.configure(m_effectiveMode, &error)) {
            raiseError(error);
            return;
        }

        if (!buffers.request(device->fd(), m_effectiveMode, &error) || !buffers.queueAll(&error)) {
            raiseError(error);
            return;
        }

        statusMessage(QStringLiteral("%1").arg(m_device.displayName()));
        waitCondition->wait(this);
        if (!m_running) {
            return;
        }

        if (!syntalosClockCanUseV4L2Monotonic(&error)) {
            raiseError(error);
            return;
        }

        nanoseconds_t clockMonotonicBeforeNs;
        nanoseconds_t clockMonotonicAfterNs;
        if (!currentClockMonotonicNsec(&clockMonotonicBeforeNs, &error)) {
            raiseError(error);
            return;
        }
        const auto runSampleNs = m_syTimer->timeSinceStartNsec();
        if (!currentClockMonotonicNsec(&clockMonotonicAfterNs, &error)) {
            raiseError(error);
            return;
        }
        const auto clockMonotonicAtRunSampleNs =
            clockMonotonicBeforeNs + ((clockMonotonicAfterNs - clockMonotonicBeforeNs) / 2);
        const auto clockMonotonicToRunOffsetNs = runSampleNs - clockMonotonicAtRunSampleNs;

        if (!streamGuard.start(device->fd(), &error)) {
            raiseError(error);
            return;
        }

        QString controlError;
        auto controls = device->queryControls(&controlError);
        if (!controlError.isEmpty()) {
            logWarning(m_log, QStringLiteral("Unable to query V4L2 controls after stream start: %1").arg(controlError));
        }
        applyDesiredControls(*device, controls);
        controlError.clear();
        auto refreshedControls = device->queryControls(&controlError);
        if (!controlError.isEmpty()) {
            logWarning(
                m_log,
                QStringLiteral("Unable to refresh V4L2 controls after stream-start restore: %1").arg(controlError));
        }
        m_controlApplier.updateControlMap(refreshedControls);
        QMetaObject::invokeMethod(
            m_settingsDialog,
            [this, controls = std::move(refreshedControls)]() { m_settingsDialog->replaceControls(controls); },
            Qt::QueuedConnection);

        uint64_t frameCount = 0;
        uint64_t invalidFrameCount = 0;
        uint64_t droppedFrameCount = 0;
        uint64_t sequenceGapCount = 0;
        uint64_t sequenceWrapOffset = 0;
        std::optional<quint32> lastSequence;
        std::optional<nanoseconds_t> lastDriverTimestamp;
        std::optional<microseconds_t> lastFrameTime;

        auto lastStatusTime = std::chrono::steady_clock::now();
        uint64_t lastStatusFrame = 0;

        while (m_running) {
            applyPendingControlWrites(*device);

            pollfd fds[3] = {};
            fds[0].fd = device->fd();
            fds[0].events = POLLIN | POLLERR | POLLHUP;
            fds[1].fd = stopEvent.fd();
            fds[1].events = POLLIN;
            fds[2].fd = controlEvent.fd();
            fds[2].events = POLLIN;

            const int pollResult = poll(fds, 3, controlPollTimeoutMs());
            if (pollResult < 0) {
                if (errno == EINTR)
                    continue;
                raiseError(QStringLiteral("poll failed while waiting for V4L2 frame: %1").arg(V4L2Camera::errnoString()));
                break;
            }
            if (pollResult == 0)
                continue;
            if ((fds[1].revents & POLLIN) != 0)
                break;
            if ((fds[2].revents & POLLIN) != 0) {
                controlEvent.drain();
                applyPendingControlWrites(*device);
            }
            if ((fds[0].revents & (POLLHUP | POLLNVAL)) != 0) {
                raiseError(QStringLiteral("V4L2 camera disappeared or became invalid."));
                break;
            }
            if ((fds[0].revents & (POLLIN | POLLERR)) == 0)
                continue;

            while (m_running) {
                v4l2_buffer buf = {};
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;

                if (V4L2Camera::xioctl(device->fd(), VIDIOC_DQBUF, &buf) < 0) {
                    if (errno == EAGAIN)
                        break;
                    if (errno == ENODEV || errno == EIO) {
                        // Keep EIO fatal for now. V4L2 permits drivers to report transient capture
                        // errors either as EIO here or as V4L2_BUF_FLAG_ERROR on a dequeued buffer.
                        // If EIO becomes common with supported cameras, add bounded stream recovery:
                        // STREAMOFF, requeue all mmap buffers, STREAMON, reset sequence/timestamp state,
                        // and abort only after repeated recovery failures.
                        raiseError(QStringLiteral("V4L2 camera returned a capture error: %1").arg(V4L2Camera::errnoString()));
                        m_running = false;
                        break;
                    }
                    invalidFrameCount++;
                    logWarning(m_log, QStringLiteral("VIDIOC_DQBUF failed: %1").arg(V4L2Camera::errnoString()));
                    if (invalidFrameCount > 16) {
                        raiseError(QStringLiteral("Too many V4L2 frame dequeue failures."));
                        m_running = false;
                    }
                    break;
                }

                if (buf.index >= buffers.size()) {
                    raiseError(QStringLiteral("V4L2 driver returned an invalid buffer index."));
                    m_running = false;
                    break;
                }

                const bool timestampMonotonic =
                    (buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
                const auto driverTimestamp = v4l2TimestampToNsec(buf.timestamp);
                if (!timestampMonotonic) {
                    raiseError(
                        QStringLiteral(
                            "V4L2 buffer timestamp type is %1; camera-v4l2 requires "
                            "V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC.")
                            .arg(timestampTypeName(buf.flags)));
                    m_running = false;
                } else if (!acceptedTimestampSource(buf.flags)) {
                    raiseError(
                        QStringLiteral(
                            "V4L2 buffer timestamp source is %1; camera-v4l2 accepts only SOE and EOF timestamps.")
                            .arg(timestampSourceName(buf.flags)));
                    m_running = false;
                } else if (driverTimestamp.count() <= 0) {
                    raiseError(QStringLiteral("V4L2 buffer returned an invalid zero monotonic timestamp."));
                    m_running = false;
                } else if (lastDriverTimestamp.has_value() && driverTimestamp <= *lastDriverTimestamp) {
                    raiseError(QStringLiteral("V4L2 monotonic timestamps are not strictly increasing."));
                    m_running = false;
                } else {
                    lastDriverTimestamp = driverTimestamp;
                }

                if (m_running) {
                    if (lastSequence.has_value() && buf.sequence < *lastSequence
                        && (*lastSequence - buf.sequence) > (std::numeric_limits<quint32>::max() / 2)) {
                        sequenceWrapOffset += uint64_t{1} << 32;
                    }
                    const uint64_t frameIndex = sequenceWrapOffset + buf.sequence;

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
                        const auto bytesUsed =
                            buf.bytesused == 0 ? buffers[buf.index].length : static_cast<size_t>(buf.bytesused);
                        const auto *data = static_cast<const quint8 *>(buffers[buf.index].start);
                        bool fatalDecodeError = false;
                        if (!decoder.decode(data, bytesUsed, &image, &error, &fatalDecodeError)) {
                            invalidFrameCount++;
                            const auto message = QStringLiteral("Failed to decode V4L2 frame: %1").arg(error);
                            if (fatalDecodeError) {
                                raiseError(message);
                                m_running = false;
                            } else {
                                logWarning(m_log, message);
                            }
                        } else {
                            auto frameTime = nsecToUsec(driverTimestamp + clockMonotonicToRunOffsetNs);

                            if (lastFrameTime.has_value() && frameTime <= *lastFrameTime)
                                frameTime = *lastFrameTime + microseconds_t(1);
                            lastFrameTime = frameTime;

                            m_outStream->push(Frame(image, frameIndex, frameTime));
                            frameCount++;
                            invalidFrameCount = 0;
                        }
                    }
                }

                if (V4L2Camera::xioctl(device->fd(), VIDIOC_QBUF, &buf) < 0) {
                    raiseError(QStringLiteral("VIDIOC_QBUF failed after frame processing: %1").arg(V4L2Camera::errnoString()));
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
    }

    void stop() override
    {
        statusMessage(QStringLiteral("Cleaning up..."));
        m_running = false;

        V4L2Camera::signalEventFd(m_stopEventFd.load());
        if (m_stopped.load()) {
            QMutexLocker locker(&m_deviceMutex);
            m_preparedDevice.reset();
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
        if (!m_stopped || hasPreparedDevice())
            return;
        if (kind == UsbHotplugEventKind::DEVICE_ARRIVED || kind == UsbHotplugEventKind::DEVICES_CHANGE
            || kind == UsbHotplugEventKind::DEVICE_LEFT)
            m_settingsDialog->refreshDevices();
    }

private:
    void wakeControlThread()
    {
        V4L2Camera::signalEventFd(m_controlEventFd.load());
    }

    bool hasPreparedDevice()
    {
        QMutexLocker locker(&m_deviceMutex);
        return m_preparedDevice != nullptr;
    }

    bool configureCapture(V4L2Camera::Device &device, V4L2Camera::CaptureMode &effectiveMode, QString *error)
    {
        if (!device.isOpen() && !device.open(m_device.devicePath, error))
            return false;

        if (!device.applyCaptureMode(m_requestedMode, &effectiveMode, error))
            return false;

        return true;
    }

    void applyDesiredControls(V4L2Camera::Device &device, QList<V4L2Camera::ControlInfo> &controls)
    {
        V4L2Camera::ControlRestoreRequest request;
        {
            QMutexLocker locker(&m_controlMutex);
            request.desiredValues = m_desiredControlValues;
            request.manualReapplyDelaysMs = m_manualReapplyDelaysMs;
            request.forceFocusAutoCycleOnRestore = m_forceFocusAutoCycleOnRestore;
        }

        const auto report = m_controlApplier.applyDesiredControls(device, controls, request);
        {
            QMutexLocker locker(&m_controlMutex);
            m_desiredControlValues = report.desiredValues;
        }
        publishControlReportWarnings(report);
    }

    int controlPollTimeoutMs() const
    {
        return m_controlApplier.controlPollTimeoutMs();
    }

    void applyPendingControlWrites(V4L2Camera::Device &device)
    {
        V4L2Camera::PendingControlRequest request;
        {
            QMutexLocker locker(&m_controlMutex);
            request.pendingWrites = m_pendingControlWrites;
            m_pendingControlWrites.clear();
            request.pendingButtons = m_pendingButtonControls;
            m_pendingButtonControls.clear();
            request.desiredValues = m_desiredControlValues;
            request.manualReapplyDelaysMs = m_manualReapplyDelaysMs;
        }

        const auto report = m_controlApplier.applyPendingControlWrites(device, request);
        if (!report.didWork)
            return;

        {
            QMutexLocker locker(&m_controlMutex);
            m_desiredControlValues = report.desiredValues;
        }
        publishControlReportWarnings(report);

        for (const auto &readback : report.readbackUpdates) {
            QMetaObject::invokeMethod(
                m_settingsDialog,
                [this, id = readback.id, value = readback.value]() { m_settingsDialog->updateControlReadback(id, value); },
                Qt::QueuedConnection);
        }

        if (!report.affectedRefreshIds.isEmpty()) {
            if (report.inventoryChanged) {
                QMetaObject::invokeMethod(
                    m_settingsDialog,
                    [this, controls = report.refreshedControls]() { m_settingsDialog->replaceControls(controls); },
                    Qt::QueuedConnection);
            } else {
                QMetaObject::invokeMethod(
                    m_settingsDialog,
                    [this, controls = report.refreshedControls, affectedRefreshIds = report.affectedRefreshIds]() {
                        m_settingsDialog->updateControls(controls, affectedRefreshIds);
                    },
                    Qt::QueuedConnection);
            }
        }
    }

    void publishControlReportWarnings(const V4L2Camera::ControlApplyReport &report)
    {
        for (const auto &message : report.logWarnings)
            logWarning(m_log, message);

        for (const auto &warning : report.userWarnings) {
            auto *flag = warning.kind == V4L2Camera::ControlApplyWarningKind::ExposureAutoPriority
                ? &m_warnedExposurePriority
                : &m_warnedReadbackMismatch;
            warnOnce(flag, warning.title, warning.message);
        }
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
    return QStringLiteral("Capture GREY, YUYV, and MJPEG frames from V4L2 cameras.");
}

QString V4L2CameraModuleInfo::description() const
{
    return QStringLiteral(
        "Capture from Linux V4L2 devices using native mmap streaming. This module currently supports GREY, YUYV, "
        "and MJPEG pixel formats and exposes standard V4L2 controls.");
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
