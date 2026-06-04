/*
 * Copyright (C) 2026 Matthias Klumpp <matthias@tenstral.net>
 *
 * Licensed under the GNU Lesser General Public License Version 3
 */

#include "v4l2device.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSize>
#include <QStringList>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <numeric>
#include <sys/ioctl.h>
#include <unistd.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
}

#ifndef V4L2_FMT_FLAG_EMULATED
#define V4L2_FMT_FLAG_EMULATED 0x0002
#endif

#ifndef V4L2_CTRL_FLAG_NEXT_COMPOUND
#define V4L2_CTRL_FLAG_NEXT_COMPOUND 0x40000000
#endif

namespace V4L2Camera
{

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

} // namespace V4L2Camera

namespace
{

using FrameInterval = QPair<quint32, quint32>;
using V4L2Camera::xioctl;

QString avErrorString(int errnum)
{
    char buf[AV_ERROR_MAX_STRING_SIZE] = {};
    av_strerror(errnum, buf, sizeof(buf));
    return QString::fromLocal8Bit(buf);
}

QString readTrimmedFile(const QString &path)
{
    QFile file(path);
    if (!file.open(QFile::ReadOnly | QFile::Text))
        return {};
    return QString::fromUtf8(file.readAll()).trimmed();
}

int videoNodeNumber(const QString &name)
{
    bool ok = false;
    const auto value = name.mid(QStringLiteral("video").size()).toInt(&ok);
    return ok ? value : std::numeric_limits<int>::max();
}

QString sysfsDevicePathForNode(const QString &nodeName)
{
    const auto linkPath = QStringLiteral("/sys/class/video4linux/%1/device").arg(nodeName);
    const auto resolved = QFileInfo(linkPath).canonicalFilePath();
    return resolved.isEmpty() ? linkPath : resolved;
}

void fillUsbIdentity(const QString &sysfsDevicePath, V4L2Camera::DeviceIdentity *identity)
{
    auto dir = QDir(sysfsDevicePath);
    for (int i = 0; i < 10 && dir.exists(); ++i) {
        if (identity->usbVid.isEmpty())
            identity->usbVid = readTrimmedFile(dir.filePath(QStringLiteral("idVendor")));
        if (identity->usbPid.isEmpty())
            identity->usbPid = readTrimmedFile(dir.filePath(QStringLiteral("idProduct")));
        if (identity->serial.isEmpty())
            identity->serial = readTrimmedFile(dir.filePath(QStringLiteral("serial")));

        if (!identity->usbVid.isEmpty() && !identity->usbPid.isEmpty() && !identity->serial.isEmpty())
            return;

        if (!dir.cdUp())
            return;
    }
}

void addMode(
    QList<V4L2Camera::CaptureMode> *modes,
    QSet<QString> *seen,
    quint32 fourcc,
    const QString &description,
    quint32 formatFlags,
    int width,
    int height,
    quint32 numerator,
    quint32 denominator)
{
    if (width <= 0 || height <= 0 || numerator == 0 || denominator == 0)
        return;

    const auto key = QStringLiteral("%1:%2:%3:%4:%5")
                         .arg(fourcc)
                         .arg(width)
                         .arg(height)
                         .arg(numerator)
                         .arg(denominator);
    if (seen->contains(key))
        return;
    seen->insert(key);

    V4L2Camera::CaptureMode mode;
    mode.fourcc = fourcc;
    mode.fourccString = V4L2Camera::fourccToString(fourcc);
    mode.description = description;
    mode.width = width;
    mode.height = height;
    mode.timeperframeNum = numerator;
    mode.timeperframeDen = denominator;
    mode.flags = formatFlags;
    mode.compressed = (formatFlags & V4L2_FMT_FLAG_COMPRESSED) != 0;
    mode.emulated = (formatFlags & V4L2_FMT_FLAG_EMULATED) != 0;
    mode.cvType = V4L2Camera::cvTypeForFourcc(fourcc);
    modes->append(mode);
}

void addFrameSize(QList<QSize> *sizes, QSet<QString> *seen, int width, int height)
{
    if (width <= 0 || height <= 0)
        return;

    const auto key = QStringLiteral("%1x%2").arg(width).arg(height);
    if (seen->contains(key))
        return;

    seen->insert(key);
    sizes->append(QSize(width, height));
}

bool frameSizeMatchesStep(quint32 value, quint32 min, quint32 max, quint32 step)
{
    if (value < min || value > max)
        return false;
    if (step == 0)
        return true;
    return (value - min) % step == 0;
}

QList<QSize> frameSizesForStepwise(const v4l2_frmsize_stepwise &range)
{
    QList<QSize> sizes;
    QSet<QString> seen;

    const auto addIfValid = [&sizes, &seen, &range](quint32 width, quint32 height) {
        if (!frameSizeMatchesStep(width, range.min_width, range.max_width, range.step_width))
            return;
        if (!frameSizeMatchesStep(height, range.min_height, range.max_height, range.step_height))
            return;
        if (width > static_cast<quint32>(std::numeric_limits<int>::max())
            || height > static_cast<quint32>(std::numeric_limits<int>::max()))
            return;

        addFrameSize(&sizes, &seen, static_cast<int>(width), static_cast<int>(height));
    };

    addIfValid(range.min_width, range.min_height);

    static const QSize commonSizes[] = {
        QSize(128, 96),
        QSize(320, 240),
        QSize(352, 288),
        QSize(360, 280),
        QSize(544, 480),
        QSize(576, 480),
        QSize(640, 480),
        QSize(720, 480),
        QSize(800, 600),
        QSize(960, 720),
        QSize(1024, 768),
        QSize(1280, 720),
        QSize(1280, 960),
        QSize(1440, 1080),
        QSize(1600, 1200),
        QSize(1920, 1080),
        QSize(1920, 1200),
        QSize(2048, 1152),
        QSize(2048, 1536),
        QSize(2560, 1440),
        QSize(3840, 2160),
        QSize(4096, 3072),
        QSize(7680, 4320),
        QSize(7680, 4800),
    };

    for (const auto &size : commonSizes)
        addIfValid(static_cast<quint32>(size.width()), static_cast<quint32>(size.height()));

    addIfValid(range.max_width, range.max_height);

    return sizes;
}

long double intervalSeconds(const FrameInterval &interval)
{
    return static_cast<long double>(interval.first) / static_cast<long double>(interval.second);
}

double intervalFps(const FrameInterval &interval)
{
    return static_cast<double>(interval.second) / static_cast<double>(interval.first);
}

void addFrameInterval(QList<FrameInterval> *intervals, QSet<QString> *seen, quint64 numerator, quint64 denominator)
{
    if (numerator == 0 || denominator == 0)
        return;

    const auto divisor = std::gcd(numerator, denominator);
    numerator /= divisor;
    denominator /= divisor;

    if (numerator > std::numeric_limits<quint32>::max() || denominator > std::numeric_limits<quint32>::max())
        return;

    const auto key = QStringLiteral("%1/%2").arg(numerator).arg(denominator);
    if (seen->contains(key))
        return;

    seen->insert(key);
    intervals->append({static_cast<quint32>(numerator), static_cast<quint32>(denominator)});
}

FrameInterval intervalForFps(long double fps)
{
    if (fps <= 0.0L)
        return {};

    const auto rounded = std::llround(fps);
    if (rounded > 0 && rounded <= std::numeric_limits<quint32>::max()
        && std::fabsl(fps - static_cast<long double>(rounded)) < 0.0001L)
        return {1, static_cast<quint32>(rounded)};

    struct FpsPreset {
        long double fps;
        quint32 numerator;
        quint32 denominator;
    };

    static const FpsPreset presets[] = {
        {23.976L, 1001, 24000},
        {29.970L, 1001, 30000},
        {59.940L, 1001, 60000},
        {119.880L, 1001, 120000},
    };
    for (const auto &preset : presets) {
        if (std::fabsl(fps - preset.fps) < 0.01L)
            return {preset.numerator, preset.denominator};
    }

    constexpr quint64 denominator = 1000000;
    const auto numerator = static_cast<quint64>(std::max<long long>(1, std::llround(denominator / fps)));
    const auto divisor = std::gcd(numerator, denominator);
    return {static_cast<quint32>(numerator / divisor), static_cast<quint32>(denominator / divisor)};
}

FrameInterval intervalForStepwiseFps(long double fps, const v4l2_frmival_stepwise &range, bool alignToStep)
{
    const FrameInterval minInterval = {range.min.numerator, range.min.denominator};
    const FrameInterval maxInterval = {range.max.numerator, range.max.denominator};
    const auto minSeconds = std::min(intervalSeconds(minInterval), intervalSeconds(maxInterval));
    const auto maxSeconds = std::max(intervalSeconds(minInterval), intervalSeconds(maxInterval));

    auto seconds = 1.0L / fps;
    if (alignToStep && range.step.numerator > 0 && range.step.denominator > 0) {
        const auto stepSeconds = static_cast<long double>(range.step.numerator)
            / static_cast<long double>(range.step.denominator);
        if (stepSeconds > 0.0L) {
            const auto steps = std::llround((seconds - minSeconds) / stepSeconds);
            seconds = minSeconds + static_cast<long double>(steps) * stepSeconds;
        }
    }

    seconds = std::clamp(seconds, minSeconds, maxSeconds);
    return intervalForFps(1.0L / seconds);
}

QList<double> fpsSamplesForRange(double minFps, double maxFps)
{
    QList<double> samples;
    if (minFps <= 0.0 || maxFps <= 0.0)
        return samples;
    if (minFps > maxFps)
        std::swap(minFps, maxFps);

    samples.append(minFps);

    double current = std::floor(minFps);
    if (current < 1.0)
        current = 1.0;

    while (current < maxFps) {
        if (current < 20.0)
            current += 1.0;
        else if (current < 100.0)
            current += 10.0;
        else if (current < 1000.0)
            current += 50.0;
        else
            current += 100.0;

        if (current > minFps && current < maxFps)
            samples.append(current);
    }

    samples.append(maxFps);
    return samples;
}

QList<FrameInterval> frameIntervalsForStepwise(const v4l2_frmival_stepwise &range, bool alignToStep)
{
    QList<FrameInterval> intervals;
    QSet<QString> seen;

    const FrameInterval minInterval = {range.min.numerator, range.min.denominator};
    const FrameInterval maxInterval = {range.max.numerator, range.max.denominator};
    if (minInterval.first == 0 || minInterval.second == 0 || maxInterval.first == 0 || maxInterval.second == 0)
        return intervals;

    addFrameInterval(&intervals, &seen, minInterval.first, minInterval.second);
    addFrameInterval(&intervals, &seen, maxInterval.first, maxInterval.second);

    const auto minFps = std::min(intervalFps(minInterval), intervalFps(maxInterval));
    const auto maxFps = std::max(intervalFps(minInterval), intervalFps(maxInterval));

    static const FrameInterval commonIntervals[] = {
        {1,    1     },
        {1,    2     },
        {1,    5     },
        {1,    10    },
        {1,    15    },
        {1,    20    },
        {1,    24    },
        {1001, 24000 },
        {1,    25    },
        {1001, 30000 },
        {1,    30    },
        {1,    50    },
        {1001, 60000 },
        {1,    60    },
        {1,    90    },
        {1001, 120000},
        {1,    120   },
        {1,    240   },
    };

    const auto addTargetFps = [&intervals, &seen, &range, alignToStep, minFps, maxFps](long double fps) {
        if (fps < static_cast<long double>(minFps) - 0.001L || fps > static_cast<long double>(maxFps) + 0.001L)
            return;
        const auto interval = intervalForStepwiseFps(fps, range, alignToStep);
        addFrameInterval(&intervals, &seen, interval.first, interval.second);
    };

    for (const auto &interval : commonIntervals)
        addTargetFps(intervalFps(interval));

    for (const auto fps : fpsSamplesForRange(minFps, maxFps))
        addTargetFps(fps);

    std::sort(intervals.begin(), intervals.end(), [](const FrameInterval &a, const FrameInterval &b) {
        return intervalFps(a) > intervalFps(b);
    });

    return intervals;
}

QList<FrameInterval> frameIntervalsForSize(int fd, quint32 fourcc, int width, int height)
{
    QList<FrameInterval> intervals;
    QSet<QString> seen;

    v4l2_frmivalenum fival = {};
    fival.pixel_format = fourcc;
    fival.width = static_cast<quint32>(width);
    fival.height = static_cast<quint32>(height);

    for (fival.index = 0; xioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &fival) == 0; ++fival.index) {
        if (fival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
            if (fival.discrete.numerator > 0 && fival.discrete.denominator > 0)
                addFrameInterval(&intervals, &seen, fival.discrete.numerator, fival.discrete.denominator);
        } else if (fival.type == V4L2_FRMIVAL_TYPE_STEPWISE || fival.type == V4L2_FRMIVAL_TYPE_CONTINUOUS) {
            intervals = frameIntervalsForStepwise(fival.stepwise, fival.type == V4L2_FRMIVAL_TYPE_STEPWISE);
            break;
        }
    }

    if (intervals.isEmpty())
        addFrameInterval(&intervals, &seen, 1, 30);
    return intervals;
}

bool controlTypeSupported(quint32 type)
{
    switch (type) {
    case V4L2_CTRL_TYPE_INTEGER:
    case V4L2_CTRL_TYPE_BOOLEAN:
    case V4L2_CTRL_TYPE_MENU:
    case V4L2_CTRL_TYPE_INTEGER_MENU:
    case V4L2_CTRL_TYPE_INTEGER64:
    case V4L2_CTRL_TYPE_BITMASK:
    case V4L2_CTRL_TYPE_BUTTON:
        return true;
    default:
        return false;
    }
}

bool fitsInt(qint64 value)
{
    return value >= std::numeric_limits<int>::min() && value <= std::numeric_limits<int>::max();
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

} // namespace

namespace V4L2Camera
{

bool DeviceIdentity::isValid() const
{
    return !devicePath.isEmpty();
}

QString DeviceIdentity::displayName() const
{
    QString label = card.isEmpty() ? devicePath : card;
    if (!serial.isEmpty())
        label = QStringLiteral("%1 [%2]").arg(label, serial);
    return QStringLiteral("%1 (%2)").arg(label, devicePath);
}

QString DeviceIdentity::stableKey() const
{
    return QStringLiteral("%1|%2|%3|%4|%5|%6|%7|%8")
        .arg(driver, card, busInfo, nodeName, QString::number(deviceCaps), usbVid, usbPid, serial);
}

bool DeviceIdentity::exactMatch(const DeviceIdentity &other) const
{
    if (!isValid() || !other.isValid())
        return false;

    return driver == other.driver && card == other.card && busInfo == other.busInfo && nodeName == other.nodeName
        && deviceCaps == other.deviceCaps && usbVid == other.usbVid && usbPid == other.usbPid && serial == other.serial;
}

bool DeviceIdentity::stableMatch(const DeviceIdentity &other) const
{
    if (!isValid() || !other.isValid())
        return false;
    if (usbVid.isEmpty() || usbPid.isEmpty() || serial.isEmpty())
        return false;
    return usbVid == other.usbVid && usbPid == other.usbPid && serial == other.serial;
}

bool DeviceIdentity::weakMatch(const DeviceIdentity &other) const
{
    if (!isValid() || !other.isValid())
        return false;
    if (stableMatch(other))
        return false;
    return !busInfo.isEmpty() && driver == other.driver && card == other.card && busInfo == other.busInfo;
}

bool DeviceMatch::hasDevice() const
{
    return device.isValid() && kind != DeviceMatchKind::None;
}

bool DeviceMatch::trustedForSavedSettings() const
{
    return kind == DeviceMatchKind::Exact || kind == DeviceMatchKind::Stable || kind == DeviceMatchKind::Weak;
}

QVariantHash DeviceIdentity::toVariant() const
{
    return {
        {QStringLiteral("device_path"), devicePath},
        {QStringLiteral("node_name"), nodeName},
        {QStringLiteral("sysfs_path"), sysfsPath},
        {QStringLiteral("driver"), driver},
        {QStringLiteral("card"), card},
        {QStringLiteral("bus_info"), busInfo},
        {QStringLiteral("capabilities"), static_cast<quint64>(capabilities)},
        {QStringLiteral("device_caps"), static_cast<quint64>(deviceCaps)},
        {QStringLiteral("usb_vid"), usbVid},
        {QStringLiteral("usb_pid"), usbPid},
        {QStringLiteral("serial"), serial},
    };
}

DeviceIdentity DeviceIdentity::fromVariant(const QVariantHash &data)
{
    DeviceIdentity identity;
    identity.devicePath = data.value(QStringLiteral("device_path")).toString();
    identity.nodeName = data.value(QStringLiteral("node_name")).toString();
    identity.sysfsPath = data.value(QStringLiteral("sysfs_path")).toString();
    identity.driver = data.value(QStringLiteral("driver")).toString();
    identity.card = data.value(QStringLiteral("card")).toString();
    identity.busInfo = data.value(QStringLiteral("bus_info")).toString();
    identity.capabilities = data.value(QStringLiteral("capabilities")).toUInt();
    identity.deviceCaps = data.value(QStringLiteral("device_caps")).toUInt();
    identity.usbVid = data.value(QStringLiteral("usb_vid")).toString();
    identity.usbPid = data.value(QStringLiteral("usb_pid")).toString();
    identity.serial = data.value(QStringLiteral("serial")).toString();
    return identity;
}

bool CaptureMode::isValid() const
{
    return fourcc != 0 && width > 0 && height > 0 && timeperframeNum > 0 && timeperframeDen > 0;
}

double CaptureMode::fps() const
{
    if (timeperframeNum == 0)
        return 0.0;
    return static_cast<double>(timeperframeDen) / static_cast<double>(timeperframeNum);
}

QString CaptureMode::displayName() const
{
    const auto fpsValue = fps();
    const auto fpsText = qFuzzyCompare(fpsValue, std::round(fpsValue))
        ? QString::number(std::llround(fpsValue))
        : QString::number(fpsValue, 'f', 3);
    return QStringLiteral("%1 %2x%3 @ %4 fps").arg(fourccString).arg(width).arg(height).arg(fpsText);
}

QVariantHash CaptureMode::toVariant() const
{
    return {
        {QStringLiteral("fourcc"), fourccString},
        {QStringLiteral("width"), width},
        {QStringLiteral("height"), height},
        {QStringLiteral("timeperframe_num"), static_cast<quint64>(timeperframeNum)},
        {QStringLiteral("timeperframe_den"), static_cast<quint64>(timeperframeDen)},
        {QStringLiteral("description"), description},
        {QStringLiteral("flags"), static_cast<quint64>(flags)},
        {QStringLiteral("bytes_per_line"), static_cast<quint64>(bytesPerLine)},
        {QStringLiteral("size_image"), static_cast<quint64>(sizeImage)},
        {QStringLiteral("colorspace"), static_cast<quint64>(colorspace)},
        {QStringLiteral("field"), static_cast<quint64>(field)},
    };
}

CaptureMode CaptureMode::fromVariant(const QVariantHash &data)
{
    CaptureMode mode;
    mode.fourccString = data.value(QStringLiteral("fourcc")).toString();
    mode.fourcc = fourccFromString(mode.fourccString);
    mode.width = data.value(QStringLiteral("width")).toInt();
    mode.height = data.value(QStringLiteral("height")).toInt();
    mode.timeperframeNum = data.value(QStringLiteral("timeperframe_num"), 1).toUInt();
    mode.timeperframeDen = data.value(QStringLiteral("timeperframe_den"), 30).toUInt();
    mode.description = data.value(QStringLiteral("description")).toString();
    mode.flags = data.value(QStringLiteral("flags")).toUInt();
    mode.compressed = (mode.flags & V4L2_FMT_FLAG_COMPRESSED) != 0;
    mode.emulated = (mode.flags & V4L2_FMT_FLAG_EMULATED) != 0;
    mode.cvType = cvTypeForFourcc(mode.fourcc);
    mode.bytesPerLine = data.value(QStringLiteral("bytes_per_line")).toUInt();
    mode.sizeImage = data.value(QStringLiteral("size_image")).toUInt();
    mode.colorspace = data.value(QStringLiteral("colorspace")).toUInt();
    mode.field = data.value(QStringLiteral("field"), V4L2_FIELD_NONE).toUInt();
    return mode;
}

bool CaptureMode::sameSelection(const CaptureMode &other) const
{
    return fourcc == other.fourcc && width == other.width && height == other.height
        && timeperframeNum == other.timeperframeNum && timeperframeDen == other.timeperframeDen;
}

bool ControlInfo::isClassMarker() const
{
    return type == V4L2_CTRL_TYPE_CTRL_CLASS;
}

bool ControlInfo::isDisabled() const
{
    return (flags & V4L2_CTRL_FLAG_DISABLED) != 0;
}

bool ControlInfo::isInactive() const
{
    return (flags & V4L2_CTRL_FLAG_INACTIVE) != 0;
}

bool ControlInfo::isReadOnly() const
{
    return (flags & V4L2_CTRL_FLAG_READ_ONLY) != 0;
}

bool ControlInfo::isWriteOnly() const
{
    return (flags & V4L2_CTRL_FLAG_WRITE_ONLY) != 0;
}

bool ControlInfo::isVolatile() const
{
    return (flags & V4L2_CTRL_FLAG_VOLATILE) != 0;
}

bool ControlInfo::isExecuteOnWrite() const
{
#ifdef V4L2_CTRL_FLAG_EXECUTE_ON_WRITE
    return (flags & V4L2_CTRL_FLAG_EXECUTE_ON_WRITE) != 0;
#else
    return false;
#endif
}

bool ControlInfo::isButton() const
{
    return type == V4L2_CTRL_TYPE_BUTTON;
}

bool ControlInfo::canRead() const
{
    return supported && !isClassMarker() && !isButton() && !isDisabled() && !isWriteOnly();
}

bool ControlInfo::canWrite() const
{
    return supported && !isClassMarker() && !isDisabled() && !isReadOnly() && !isInactive();
}

bool ControlInfo::restorable() const
{
    return canWrite() && !isButton() && !isVolatile() && !isExecuteOnWrite();
}

QString ControlInfo::className() const
{
    return controlClassName(controlClass == 0 ? V4L2_CTRL_ID2CLASS(id) : controlClass);
}

QString fourccToString(quint32 fourcc)
{
    char text[5] = {};
    text[0] = static_cast<char>(fourcc & 0xff);
    text[1] = static_cast<char>((fourcc >> 8) & 0xff);
    text[2] = static_cast<char>((fourcc >> 16) & 0xff);
    text[3] = static_cast<char>((fourcc >> 24) & 0xff);
    return QString::fromLatin1(text, 4).trimmed();
}

quint32 fourccFromString(const QString &text)
{
    QByteArray bytes = text.toLatin1();
    while (bytes.size() < 4)
        bytes.append(' ');
    return v4l2_fourcc(bytes[0], bytes[1], bytes[2], bytes[3]);
}

bool isSupportedFourcc(quint32 fourcc)
{
    return fourcc == V4L2_PIX_FMT_GREY || fourcc == V4L2_PIX_FMT_YUYV || fourcc == V4L2_PIX_FMT_MJPEG
        || fourcc == v4l2_fourcc('M', 'J', 'P', 'G');
}

int cvTypeForFourcc(quint32 fourcc)
{
    if (fourcc == V4L2_PIX_FMT_GREY)
        return CV_8UC1;
    if (fourcc == V4L2_PIX_FMT_YUYV || fourcc == V4L2_PIX_FMT_MJPEG || fourcc == v4l2_fourcc('M', 'J', 'P', 'G'))
        return CV_8UC3;
    return -1;
}

QString controlClassName(quint32 controlClass)
{
    switch (controlClass) {
    case V4L2_CTRL_CLASS_USER:
        return QStringLiteral("User");
    case V4L2_CTRL_CLASS_CAMERA:
        return QStringLiteral("Camera");
    case V4L2_CTRL_CLASS_MPEG:
        return QStringLiteral("MPEG");
    case V4L2_CTRL_CLASS_FM_TX:
        return QStringLiteral("FM TX");
    case V4L2_CTRL_CLASS_FLASH:
        return QStringLiteral("Flash");
    case V4L2_CTRL_CLASS_JPEG:
        return QStringLiteral("JPEG");
    case V4L2_CTRL_CLASS_IMAGE_SOURCE:
        return QStringLiteral("Image Source");
    case V4L2_CTRL_CLASS_IMAGE_PROC:
        return QStringLiteral("Image Processing");
    case V4L2_CTRL_CLASS_DV:
        return QStringLiteral("Digital Video");
    case V4L2_CTRL_CLASS_FM_RX:
        return QStringLiteral("FM RX");
    case V4L2_CTRL_CLASS_RF_TUNER:
        return QStringLiteral("RF Tuner");
    case V4L2_CTRL_CLASS_DETECT:
        return QStringLiteral("Detection");
    default:
        return QStringLiteral("Class 0x%1").arg(controlClass, 0, 16);
    }
}

QString controlTypeName(quint32 type)
{
    switch (type) {
    case V4L2_CTRL_TYPE_INTEGER:
        return QStringLiteral("integer");
    case V4L2_CTRL_TYPE_BOOLEAN:
        return QStringLiteral("boolean");
    case V4L2_CTRL_TYPE_MENU:
        return QStringLiteral("menu");
    case V4L2_CTRL_TYPE_BUTTON:
        return QStringLiteral("button");
    case V4L2_CTRL_TYPE_INTEGER64:
        return QStringLiteral("integer64");
    case V4L2_CTRL_TYPE_CTRL_CLASS:
        return QStringLiteral("class");
    case V4L2_CTRL_TYPE_STRING:
        return QStringLiteral("string");
    case V4L2_CTRL_TYPE_BITMASK:
        return QStringLiteral("bitmask");
    case V4L2_CTRL_TYPE_INTEGER_MENU:
        return QStringLiteral("integer menu");
    default:
        return QStringLiteral("type %1").arg(type);
    }
}

const QList<AutoDependencyGroup> &autoDependencyGroups()
{
    static const QList<AutoDependencyGroup> groups = {
        {V4L2_CID_EXPOSURE_AUTO, {V4L2_CID_EXPOSURE_ABSOLUTE}},
        {V4L2_CID_AUTO_WHITE_BALANCE, {V4L2_CID_WHITE_BALANCE_TEMPERATURE, V4L2_CID_RED_BALANCE, V4L2_CID_BLUE_BALANCE}},
        {V4L2_CID_FOCUS_AUTO, {V4L2_CID_FOCUS_ABSOLUTE, V4L2_CID_FOCUS_RELATIVE}},
        {V4L2_CID_AUTOGAIN, {V4L2_CID_GAIN}},
        {V4L2_CID_HUE_AUTO, {V4L2_CID_HUE}},
        {V4L2_CID_CHROMA_AGC, {V4L2_CID_CHROMA_GAIN}},
    };
    return groups;
}

QHash<quint32, QList<quint32>> autoDependencyTable()
{
    QHash<quint32, QList<quint32>> table;
    const auto &groups = autoDependencyGroups();
    table.reserve(groups.size());
    for (const auto &group : groups)
        table.insert(group.autoControlId, group.manualControlIds);
    return table;
}

QSet<quint32> autoControlIds()
{
    QSet<quint32> ids;
    const auto &groups = autoDependencyGroups();
    ids.reserve(groups.size());
    for (const auto &group : groups)
        ids.insert(group.autoControlId);
    return ids;
}

bool autoControlEnabled(quint32 id, qint64 value)
{
    if (id == V4L2_CID_EXPOSURE_AUTO)
        return value != V4L2_EXPOSURE_MANUAL;
    return value != 0;
}

bool isManualDependentActive(quint32 id, const QHash<quint32, qint64> &values)
{
    for (const auto &group : autoDependencyGroups()) {
        if (!group.manualControlIds.contains(id))
            continue;
        const auto autoIt = values.constFind(group.autoControlId);
        if (autoIt != values.constEnd() && autoControlEnabled(group.autoControlId, autoIt.value()))
            return true;
    }
    return false;
}

QList<DeviceIdentity> enumerateDevices(QString *error)
{
    QList<DeviceIdentity> devices;
    QStringList nodeNames = QDir(QStringLiteral("/dev")).entryList({QStringLiteral("video*")}, QDir::System | QDir::Files);
    std::sort(nodeNames.begin(), nodeNames.end(), [](const QString &a, const QString &b) {
        return videoNodeNumber(a) < videoNodeNumber(b);
    });

    QStringList errors;
    for (const auto &nodeName : nodeNames) {
        const auto devicePath = QStringLiteral("/dev/%1").arg(nodeName);
        Device device;
        QString openError;
        if (!device.open(devicePath, &openError)) {
            errors.append(QStringLiteral("%1: %2").arg(devicePath, openError));
            continue;
        }

        DeviceIdentity identity;
        QString queryError;
        if (!device.queryIdentity(&identity, &queryError)) {
            errors.append(QStringLiteral("%1: %2").arg(devicePath, queryError));
            continue;
        }

        const auto caps = identity.deviceCaps == 0 ? identity.capabilities : identity.deviceCaps;
        if ((caps & V4L2_CAP_VIDEO_CAPTURE) == 0 || (caps & V4L2_CAP_STREAMING) == 0)
            continue;

        devices.append(identity);
    }

    if (devices.isEmpty() && error != nullptr)
        *error = errors.isEmpty() ? QStringLiteral("No V4L2 streaming capture devices found.") : errors.join(QStringLiteral("\n"));
    return devices;
}

DeviceMatch matchDevice(const DeviceIdentity &wanted, const QList<DeviceIdentity> &devices, const QString &enumError)
{
    DeviceMatch match;
    if (!wanted.isValid()) {
        if (!devices.isEmpty()) {
            match.device = devices.first();
            match.kind = DeviceMatchKind::FirstAvailable;
        } else {
            match.warning = enumError;
        }
        return match;
    }

    for (const auto &device : devices) {
        if (wanted.exactMatch(device)) {
            match.device = device;
            match.kind = DeviceMatchKind::Exact;
            return match;
        }
    }

    for (const auto &device : devices) {
        if (wanted.stableMatch(device)) {
            match.device = device;
            match.kind = DeviceMatchKind::Stable;
            return match;
        }
    }

    for (const auto &device : devices) {
        if (wanted.weakMatch(device)) {
            match.device = device;
            match.kind = DeviceMatchKind::Weak;
            match.warning = QStringLiteral("Using weak V4L2 camera identity match for %1. Check the selected device.")
                                .arg(device.displayName());
            return match;
        }
    }

    for (const auto &device : devices) {
        if (!wanted.devicePath.isEmpty() && wanted.devicePath == device.devicePath) {
            match.device = device;
            match.kind = DeviceMatchKind::Path;
            match.warning = QStringLiteral("Using %1 by path; saved camera identity did not match exactly.")
                                .arg(device.devicePath);
            return match;
        }
    }

    match.warning = enumError.isEmpty() ? QStringLiteral("Saved V4L2 camera was not found.") : enumError;
    return match;
}

std::optional<DeviceIdentity> findDevice(const DeviceIdentity &wanted, QString *warning)
{
    QString enumError;
    const auto devices = enumerateDevices(&enumError);
    const auto match = matchDevice(wanted, devices, enumError);
    if (warning != nullptr)
        *warning = match.warning;
    if (!match.hasDevice())
        return std::nullopt;
    return match.device;
}

Device::Device()
    : m_fd(-1)
{
}

Device::~Device()
{
    close();
}

bool Device::open(const QString &devicePath, QString *error, bool nonBlocking)
{
    close();

    int flags = O_RDWR | O_CLOEXEC;
    if (nonBlocking)
        flags |= O_NONBLOCK;

    m_fd = ::open(devicePath.toLocal8Bit().constData(), flags);
    if (m_fd < 0) {
        if (error != nullptr)
            *error = QStringLiteral("Unable to open %1: %2").arg(devicePath, errnoString());
        return false;
    }

    m_path = devicePath;
    return true;
}

void Device::close()
{
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
    m_path.clear();
}

bool Device::isOpen() const
{
    return m_fd >= 0;
}

int Device::fd() const
{
    return m_fd;
}

QString Device::path() const
{
    return m_path;
}

bool Device::queryIdentity(DeviceIdentity *identity, QString *error) const
{
    if (m_fd < 0) {
        if (error != nullptr)
            *error = QStringLiteral("Device is not open.");
        return false;
    }

    v4l2_capability cap = {};
    if (xioctl(m_fd, VIDIOC_QUERYCAP, &cap) < 0) {
        if (error != nullptr)
            *error = QStringLiteral("VIDIOC_QUERYCAP failed: %1").arg(errnoString());
        return false;
    }

    const auto nodeName = QFileInfo(m_path).fileName();
    identity->devicePath = m_path;
    identity->nodeName = nodeName;
    identity->sysfsPath = QFileInfo(QStringLiteral("/sys/class/video4linux/%1").arg(nodeName)).canonicalFilePath();
    identity->driver = QString::fromUtf8(reinterpret_cast<const char *>(cap.driver)).trimmed();
    identity->card = QString::fromUtf8(reinterpret_cast<const char *>(cap.card)).trimmed();
    identity->busInfo = QString::fromUtf8(reinterpret_cast<const char *>(cap.bus_info)).trimmed();
    identity->capabilities = cap.capabilities;
    identity->deviceCaps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) != 0 ? cap.device_caps : cap.capabilities;

    fillUsbIdentity(sysfsDevicePathForNode(nodeName), identity);
    return true;
}

QList<CaptureMode> Device::enumerateCaptureModes(QString *error) const
{
    QList<CaptureMode> modes;
    QSet<QString> seen;

    if (m_fd < 0) {
        if (error != nullptr)
            *error = QStringLiteral("Device is not open.");
        return modes;
    }

    v4l2_fmtdesc fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for (fmt.index = 0; xioctl(m_fd, VIDIOC_ENUM_FMT, &fmt) == 0; ++fmt.index) {
        if (!isSupportedFourcc(fmt.pixelformat))
            continue;

        const auto description = QString::fromUtf8(reinterpret_cast<const char *>(fmt.description)).trimmed();

        v4l2_frmsizeenum fsize = {};
        fsize.pixel_format = fmt.pixelformat;
        for (fsize.index = 0; xioctl(m_fd, VIDIOC_ENUM_FRAMESIZES, &fsize) == 0; ++fsize.index) {
            if (fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
                const int width = static_cast<int>(fsize.discrete.width);
                const int height = static_cast<int>(fsize.discrete.height);
                const auto intervals = frameIntervalsForSize(m_fd, fmt.pixelformat, width, height);
                for (const auto &interval : intervals)
                    addMode(&modes, &seen, fmt.pixelformat, description, fmt.flags, width, height, interval.first, interval.second);
            } else if (fsize.type == V4L2_FRMSIZE_TYPE_STEPWISE || fsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS) {
                const auto sizes = frameSizesForStepwise(fsize.stepwise);
                for (const auto &size : sizes) {
                    const auto intervals = frameIntervalsForSize(m_fd, fmt.pixelformat, size.width(), size.height());
                    for (const auto &interval : intervals)
                        addMode(
                            &modes,
                            &seen,
                            fmt.pixelformat,
                            description,
                            fmt.flags,
                            size.width(),
                            size.height(),
                            interval.first,
                            interval.second);
                }
                break;
            }
        }
    }

    std::sort(modes.begin(), modes.end(), [](const CaptureMode &a, const CaptureMode &b) {
        if (a.fourccString != b.fourccString)
            return a.fourccString < b.fourccString;
        const auto aPixels = static_cast<qint64>(a.width) * static_cast<qint64>(a.height);
        const auto bPixels = static_cast<qint64>(b.width) * static_cast<qint64>(b.height);
        if (aPixels != bPixels)
            return aPixels > bPixels;
        if (a.width != b.width)
            return a.width > b.width;
        if (a.height != b.height)
            return a.height > b.height;
        return a.fps() > b.fps();
    });

    if (modes.isEmpty() && error != nullptr)
        *error = QStringLiteral("No supported V4L2 capture modes found. Supported formats are GREY, YUYV, and MJPG.");
    return modes;
}

QList<ControlInfo> Device::queryControls(QString *error) const
{
    QList<ControlInfo> controls;
    if (m_fd < 0) {
        if (error != nullptr)
            *error = QStringLiteral("Device is not open.");
        return controls;
    }

    v4l2_query_ext_ctrl query = {};
    query.id = V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
    while (xioctl(m_fd, VIDIOC_QUERY_EXT_CTRL, &query) == 0) {
        ControlInfo control;
        control.id = query.id;
        control.name = QString::fromUtf8(reinterpret_cast<const char *>(query.name)).trimmed();
        control.controlClass = query.type == V4L2_CTRL_TYPE_CTRL_CLASS ? query.id : V4L2_CTRL_ID2CLASS(query.id);
        control.type = query.type;
        control.minimum = query.minimum;
        control.maximum = query.maximum;
        control.step = query.step == 0 ? 1 : query.step;
        control.defaultValue = query.default_value;
        control.flags = query.flags;
        control.supported = controlTypeSupported(control.type);

        if ((control.type == V4L2_CTRL_TYPE_MENU || control.type == V4L2_CTRL_TYPE_INTEGER_MENU) && control.supported) {
            for (qint64 value = control.minimum; value <= control.maximum; value += control.step) {
                if (!fitsInt(value))
                    continue;

                v4l2_querymenu menu = {};
                menu.id = control.id;
                menu.index = static_cast<quint32>(value);
                if (xioctl(m_fd, VIDIOC_QUERYMENU, &menu) != 0)
                    continue;

                MenuEntry entry;
                entry.value = control.type == V4L2_CTRL_TYPE_INTEGER_MENU ? menu.value : value;
                entry.name = QString::fromUtf8(reinterpret_cast<const char *>(menu.name)).trimmed();
                control.menu.append(entry);
            }
        }

        if (control.canRead())
            readControlValue(&control, nullptr);
        else
            control.currentValue = control.defaultValue;

        controls.append(control);
        query.id |= V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
    }

    if (controls.isEmpty() && error != nullptr && errno != EINVAL)
        *error = QStringLiteral("VIDIOC_QUERY_EXT_CTRL failed: %1").arg(errnoString());
    return controls;
}

bool Device::readControlValue(ControlInfo *control, QString *error) const
{
    if (m_fd < 0) {
        if (error != nullptr)
            *error = QStringLiteral("Device is not open.");
        return false;
    }
    if (control == nullptr || !control->canRead()) {
        if (error != nullptr)
            *error = QStringLiteral("Control is not readable.");
        return false;
    }

    v4l2_ext_control extControl = {};
    extControl.id = control->id;
    v4l2_ext_controls extControls = {};
    extControls.ctrl_class = V4L2_CTRL_ID2CLASS(control->id);
    extControls.count = 1;
    extControls.controls = &extControl;

    if (xioctl(m_fd, VIDIOC_G_EXT_CTRLS, &extControls) == 0) {
        control->currentValue = control->type == V4L2_CTRL_TYPE_INTEGER64 ? extControl.value64 : extControl.value;
        return true;
    }

    if (control->type != V4L2_CTRL_TYPE_INTEGER64) {
        v4l2_control legacy = {};
        legacy.id = control->id;
        if (xioctl(m_fd, VIDIOC_G_CTRL, &legacy) == 0) {
            control->currentValue = legacy.value;
            return true;
        }
    }

    if (error != nullptr)
        *error = QStringLiteral("Failed to read control '%1': %2").arg(control->name, errnoString());
    return false;
}

ControlWriteResult Device::setControlValue(const ControlInfo &control, qint64 value) const
{
    ControlWriteResult result;
    result.id = control.id;
    result.requestedValue = value;

    if (m_fd < 0) {
        result.error = QStringLiteral("Device is not open.");
        return result;
    }
    if (!control.canWrite()) {
        result.error = QStringLiteral("Control '%1' is not writable.").arg(control.name);
        return result;
    }

    v4l2_ext_control extControl = {};
    extControl.id = control.id;
    if (control.type == V4L2_CTRL_TYPE_INTEGER64)
        extControl.value64 = value;
    else
        extControl.value = static_cast<qint32>(value);

    v4l2_ext_controls extControls = {};
    extControls.ctrl_class = V4L2_CTRL_ID2CLASS(control.id);
    extControls.count = 1;
    extControls.controls = &extControl;

    bool writeOk = xioctl(m_fd, VIDIOC_S_EXT_CTRLS, &extControls) == 0;
    if (!writeOk && control.type != V4L2_CTRL_TYPE_INTEGER64) {
        v4l2_control legacy = {};
        legacy.id = control.id;
        legacy.value = static_cast<qint32>(value);
        writeOk = xioctl(m_fd, VIDIOC_S_CTRL, &legacy) == 0;
    }

    if (!writeOk) {
        result.error = QStringLiteral("Failed to write control '%1': %2").arg(control.name, errnoString());
        return result;
    }

    if (!control.canRead()) {
        result.readbackValue = result.requestedValue;
        result.changedByDevice = false;
        result.success = true;
        return result;
    }

    ControlInfo readbackControl = control;
    QString readError;
    if (!readControlValue(&readbackControl, &readError)) {
        result.error = readError;
        return result;
    }

    result.readbackValue = readbackControl.currentValue;
    result.changedByDevice = result.readbackValue != result.requestedValue;
    result.success = true;
    return result;
}

bool Device::triggerButtonControl(const ControlInfo &control, QString *error) const
{
    if (m_fd < 0) {
        if (error != nullptr)
            *error = QStringLiteral("Device is not open.");
        return false;
    }
    if (control.type != V4L2_CTRL_TYPE_BUTTON || !control.canWrite()) {
        if (error != nullptr)
            *error = QStringLiteral("Control '%1' is not a writable button control.").arg(control.name);
        return false;
    }

    v4l2_ext_control extControl = {};
    extControl.id = control.id;
    extControl.value = 0;

    v4l2_ext_controls extControls = {};
    extControls.ctrl_class = V4L2_CTRL_ID2CLASS(control.id);
    extControls.count = 1;
    extControls.controls = &extControl;

    bool writeOk = xioctl(m_fd, VIDIOC_S_EXT_CTRLS, &extControls) == 0;
    if (!writeOk) {
        v4l2_control legacy = {};
        legacy.id = control.id;
        legacy.value = 0;
        writeOk = xioctl(m_fd, VIDIOC_S_CTRL, &legacy) == 0;
    }

    if (!writeOk) {
        if (error != nullptr)
            *error = QStringLiteral("Failed to trigger button control '%1': %2").arg(control.name, errnoString());
        return false;
    }
    return true;
}

bool Device::applyCaptureMode(const CaptureMode &wanted, CaptureMode *effective, QString *error) const
{
    if (m_fd < 0) {
        if (error != nullptr)
            *error = QStringLiteral("Device is not open.");
        return false;
    }
    if (!wanted.isValid() || !isSupportedFourcc(wanted.fourcc)) {
        if (error != nullptr)
            *error = QStringLiteral("Unsupported or invalid V4L2 capture mode.");
        return false;
    }

    v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = static_cast<quint32>(wanted.width);
    fmt.fmt.pix.height = static_cast<quint32>(wanted.height);
    fmt.fmt.pix.pixelformat = wanted.fourcc;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if (xioctl(m_fd, VIDIOC_TRY_FMT, &fmt) < 0) {
        if (error != nullptr)
            *error = QStringLiteral("VIDIOC_TRY_FMT failed for %1: %2").arg(wanted.displayName(), errnoString());
        return false;
    }

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = static_cast<quint32>(wanted.width);
    fmt.fmt.pix.height = static_cast<quint32>(wanted.height);
    fmt.fmt.pix.pixelformat = wanted.fourcc;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(m_fd, VIDIOC_S_FMT, &fmt) < 0) {
        if (error != nullptr)
            *error = QStringLiteral("VIDIOC_S_FMT failed for %1: %2").arg(wanted.displayName(), errnoString());
        return false;
    }

    if (xioctl(m_fd, VIDIOC_G_FMT, &fmt) < 0) {
        if (error != nullptr)
            *error = QStringLiteral("VIDIOC_G_FMT failed after setting format: %1").arg(errnoString());
        return false;
    }

    v4l2_streamparm parm = {};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(m_fd, VIDIOC_G_PARM, &parm) < 0) {
        if (error != nullptr)
            *error = QStringLiteral("VIDIOC_G_PARM failed before setting frame interval for %1: %2")
                         .arg(wanted.displayName(), errnoString());
        return false;
    }

    parm.parm.capture.timeperframe.numerator = wanted.timeperframeNum;
    parm.parm.capture.timeperframe.denominator = wanted.timeperframeDen;
    if (xioctl(m_fd, VIDIOC_S_PARM, &parm) < 0) {
        if (error != nullptr)
            *error = QStringLiteral("VIDIOC_S_PARM failed for %1: %2").arg(wanted.displayName(), errnoString());
        return false;
    }

    parm = {};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(m_fd, VIDIOC_G_PARM, &parm) < 0) {
        if (error != nullptr)
            *error = QStringLiteral("VIDIOC_G_PARM failed after setting frame interval for %1: %2")
                         .arg(wanted.displayName(), errnoString());
        return false;
    }

    if (parm.parm.capture.timeperframe.numerator == 0 || parm.parm.capture.timeperframe.denominator == 0) {
        if (error != nullptr)
            *error = QStringLiteral("Device did not report a valid frame interval after VIDIOC_S_PARM for %1.")
                         .arg(wanted.displayName());
        return false;
    }

    CaptureMode actual = wanted;
    actual.fourcc = fmt.fmt.pix.pixelformat;
    actual.fourccString = fourccToString(actual.fourcc);
    actual.width = static_cast<int>(fmt.fmt.pix.width);
    actual.height = static_cast<int>(fmt.fmt.pix.height);
    actual.bytesPerLine = fmt.fmt.pix.bytesperline;
    actual.sizeImage = fmt.fmt.pix.sizeimage;
    actual.colorspace = fmt.fmt.pix.colorspace;
    actual.field = fmt.fmt.pix.field;
    actual.cvType = cvTypeForFourcc(actual.fourcc);
    actual.compressed = actual.fourcc == V4L2_PIX_FMT_MJPEG || actual.fourcc == v4l2_fourcc('M', 'J', 'P', 'G');
    actual.timeperframeNum = parm.parm.capture.timeperframe.numerator;
    actual.timeperframeDen = parm.parm.capture.timeperframe.denominator;

    if (!isSupportedFourcc(actual.fourcc) || actual.width <= 0 || actual.height <= 0 || actual.fps() <= 0) {
        if (error != nullptr)
            *error = QStringLiteral("Device reported unsupported effective mode after S_FMT/G_FMT.");
        return false;
    }

    if (effective != nullptr)
        *effective = actual;
    return true;
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

} // namespace V4L2Camera
