/*
 * Copyright (C) 2026 Matthias Klumpp <matthias@tenstral.net>
 *
 * Licensed under the GNU Lesser General Public License Version 3
 */

#pragma once

#include <QHash>
#include <QList>
#include <QSet>
#include <QString>
#include <QVariantHash>

#include <linux/videodev2.h>
#include <opencv2/core.hpp>

#include <cstdint>
#include <optional>
#include <vector>

struct AVCodecContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;

namespace V4L2Camera
{

struct DeviceIdentity {
    QString devicePath;
    QString nodeName;
    QString sysfsPath;
    QString driver;
    QString card;
    QString busInfo;
    quint32 capabilities = 0;
    quint32 deviceCaps = 0;
    QString usbVid;
    QString usbPid;
    QString serial;

    bool isValid() const;
    QString displayName() const;
    QString stableKey() const;
    bool exactMatch(const DeviceIdentity &other) const;
    bool stableMatch(const DeviceIdentity &other) const;
    bool weakMatch(const DeviceIdentity &other) const;
    QVariantHash toVariant() const;
    static DeviceIdentity fromVariant(const QVariantHash &data);
};

enum class DeviceMatchKind {
    None,
    FirstAvailable,
    Exact,
    Stable,
    Weak,
    Path
};

struct DeviceMatch {
    DeviceIdentity device;
    DeviceMatchKind kind = DeviceMatchKind::None;
    QString warning;

    bool hasDevice() const;
    bool trustedForSavedSettings() const;
};

struct CaptureMode {
    quint32 fourcc = 0;
    QString fourccString;
    QString description;
    int width = 0;
    int height = 0;
    quint32 timeperframeNum = 1;
    quint32 timeperframeDen = 30;
    quint32 flags = 0;
    bool compressed = false;
    bool emulated = false;
    int cvType = CV_8UC1;
    quint32 bytesPerLine = 0;
    quint32 sizeImage = 0;
    quint32 colorspace = 0;
    quint32 field = V4L2_FIELD_NONE;

    bool isValid() const;
    double fps() const;
    QString displayName() const;
    QVariantHash toVariant() const;
    static CaptureMode fromVariant(const QVariantHash &data);
    bool sameSelection(const CaptureMode &other) const;
};

struct MenuEntry {
    qint64 value = 0;
    QString name;
};

struct ControlInfo {
    quint32 id = 0;
    QString name;
    quint32 controlClass = 0;
    quint32 type = V4L2_CTRL_TYPE_INTEGER;
    qint64 minimum = 0;
    qint64 maximum = 0;
    qint64 step = 1;
    qint64 defaultValue = 0;
    qint64 currentValue = 0;
    quint32 flags = 0;
    QList<MenuEntry> menu;
    bool supported = false;

    bool isClassMarker() const;
    bool isDisabled() const;
    bool isInactive() const;
    bool isReadOnly() const;
    bool isWriteOnly() const;
    bool isVolatile() const;
    bool isExecuteOnWrite() const;
    bool isButton() const;
    bool canRead() const;
    bool canWrite() const;
    bool restorable() const;
    QString className() const;
};

struct ControlWriteResult {
    quint32 id = 0;
    bool success = false;
    bool changedByDevice = false;
    qint64 requestedValue = 0;
    qint64 readbackValue = 0;
    QString error;
};

struct AutoDependencyGroup {
    quint32 autoControlId = 0;
    QList<quint32> manualControlIds;
};

QString fourccToString(quint32 fourcc);
quint32 fourccFromString(const QString &text);
bool isSupportedFourcc(quint32 fourcc);
int cvTypeForFourcc(quint32 fourcc);
QString controlClassName(quint32 controlClass);
QString controlTypeName(quint32 type);
const QList<AutoDependencyGroup> &autoDependencyGroups();
QHash<quint32, QList<quint32>> autoDependencyTable();
QSet<quint32> autoControlIds();
bool autoControlEnabled(quint32 id, qint64 value);
bool isManualDependentActive(quint32 id, const QHash<quint32, qint64> &values);
QList<DeviceIdentity> enumerateDevices(QString *error = nullptr);
DeviceMatch matchDevice(
    const DeviceIdentity &wanted,
    const QList<DeviceIdentity> &devices,
    const QString &enumError = QString());
std::optional<DeviceIdentity> findDevice(const DeviceIdentity &wanted, QString *warning = nullptr);

class Device
{
public:
    Device();
    ~Device();

    Device(const Device &) = delete;
    Device &operator=(const Device &) = delete;

    bool open(const QString &devicePath, QString *error, bool nonBlocking = true);
    void close();
    bool isOpen() const;
    int fd() const;
    QString path() const;

    bool queryIdentity(DeviceIdentity *identity, QString *error) const;
    QList<CaptureMode> enumerateCaptureModes(QString *error) const;
    QList<ControlInfo> queryControls(QString *error) const;
    bool readControlValue(ControlInfo *control, QString *error) const;
    ControlWriteResult setControlValue(const ControlInfo &control, qint64 value) const;
    bool triggerButtonControl(const ControlInfo &control, QString *error) const;
    bool applyCaptureMode(const CaptureMode &wanted, CaptureMode *effective, QString *error) const;

private:
    int m_fd;
    QString m_path;
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

} // namespace V4L2Camera

Q_DECLARE_METATYPE(V4L2Camera::DeviceIdentity)
Q_DECLARE_METATYPE(V4L2Camera::CaptureMode)
Q_DECLARE_METATYPE(V4L2Camera::ControlInfo)
