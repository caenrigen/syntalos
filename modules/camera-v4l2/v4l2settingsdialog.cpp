/*
 * Copyright (C) 2026 Matthias Klumpp <matthias@tenstral.net>
 *
 * Licensed under the GNU Lesser General Public License Version 3
 */

#include "v4l2settingsdialog.h"

#include <QAbstractSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QDebug>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QStringList>
#include <QTabWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

bool fitsInt(qint64 value)
{
    return value >= std::numeric_limits<int>::min() && value <= std::numeric_limits<int>::max();
}

int intStep(qint64 step)
{
    if (step <= 1)
        return 1;
    if (step > std::numeric_limits<int>::max())
        return std::numeric_limits<int>::max();
    return static_cast<int>(step);
}

bool isIntegerControl(const V4L2Camera::ControlInfo &control)
{
    return control.type == V4L2_CTRL_TYPE_INTEGER || control.type == V4L2_CTRL_TYPE_INTEGER64;
}

bool isEditableNumericControl(const V4L2Camera::ControlInfo &control)
{
    return isIntegerControl(control) || control.type == V4L2_CTRL_TYPE_BITMASK;
}

bool isGrabbed(const V4L2Camera::ControlInfo &control)
{
    return (control.flags & V4L2_CTRL_FLAG_GRABBED) != 0;
}

bool hasPayload(const V4L2Camera::ControlInfo &control)
{
#ifdef V4L2_CTRL_FLAG_HAS_PAYLOAD
    return (control.flags & V4L2_CTRL_FLAG_HAS_PAYLOAD) != 0;
#else
    Q_UNUSED(control)
    return false;
#endif
}

qint64 snapControlValue(const V4L2Camera::ControlInfo &control, qint64 value)
{
    if (!isIntegerControl(control) || control.step <= 1)
        return value;

    const auto minValue = static_cast<long double>(control.minimum);
    const auto step = static_cast<long double>(control.step);
    const auto offset = static_cast<long double>(value) - minValue;
    const auto snapped = minValue + std::round(offset / step) * step;
    if (snapped <= static_cast<long double>(std::numeric_limits<qint64>::min()))
        return std::numeric_limits<qint64>::min();
    if (snapped >= static_cast<long double>(std::numeric_limits<qint64>::max()))
        return std::numeric_limits<qint64>::max();
    return static_cast<qint64>(snapped);
}

qint64 clampControlValue(const V4L2Camera::ControlInfo &control, qint64 value)
{
    if (control.type == V4L2_CTRL_TYPE_BUTTON)
        return value;
    if (value < control.minimum)
        return control.minimum;
    if (value > control.maximum)
        return control.maximum;
    const auto snapped = snapControlValue(control, value);
    if (snapped < control.minimum)
        return control.minimum;
    if (snapped > control.maximum)
        return control.maximum;
    return snapped;
}

QString hexId(quint32 value)
{
    return QStringLiteral("0x%1").arg(value, 8, 16, QLatin1Char('0'));
}

int dependencySortKey(quint32 id)
{
    const auto table = V4L2Camera::autoDependencyTable();
    int group = 0;
    for (auto it = table.constBegin(); it != table.constEnd(); ++it, ++group) {
        if (id == it.key())
            return group * 10;
        if (it.value().contains(id))
            return group * 10 + 1;
    }
    return 1000;
}

int valueHexWidth(const V4L2Camera::ControlInfo &control)
{
    const auto maxValue = std::max<qint64>(0, std::max(control.maximum, std::max(control.currentValue, control.defaultValue)));
    return maxValue > std::numeric_limits<quint32>::max() ? 16 : 8;
}

QString hexValue(const V4L2Camera::ControlInfo &control, qint64 value)
{
    return QStringLiteral("0x%1")
        .arg(static_cast<qulonglong>(value), valueHexWidth(control), 16, QLatin1Char('0'));
}

QString symbolicControlName(quint32 id)
{
#define V4L2_CONTROL_NAME_CASE(name) \
    case name:                       \
        return QStringLiteral(#name);

    switch (id) {
#ifdef V4L2_CID_BRIGHTNESS
        V4L2_CONTROL_NAME_CASE(V4L2_CID_BRIGHTNESS)
#endif
#ifdef V4L2_CID_CONTRAST
        V4L2_CONTROL_NAME_CASE(V4L2_CID_CONTRAST)
#endif
#ifdef V4L2_CID_SATURATION
        V4L2_CONTROL_NAME_CASE(V4L2_CID_SATURATION)
#endif
#ifdef V4L2_CID_HUE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_HUE)
#endif
#ifdef V4L2_CID_AUTO_WHITE_BALANCE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_AUTO_WHITE_BALANCE)
#endif
#ifdef V4L2_CID_DO_WHITE_BALANCE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_DO_WHITE_BALANCE)
#endif
#ifdef V4L2_CID_RED_BALANCE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_RED_BALANCE)
#endif
#ifdef V4L2_CID_BLUE_BALANCE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_BLUE_BALANCE)
#endif
#ifdef V4L2_CID_GAMMA
        V4L2_CONTROL_NAME_CASE(V4L2_CID_GAMMA)
#endif
#ifdef V4L2_CID_EXPOSURE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_EXPOSURE)
#endif
#ifdef V4L2_CID_AUTOGAIN
        V4L2_CONTROL_NAME_CASE(V4L2_CID_AUTOGAIN)
#endif
#ifdef V4L2_CID_GAIN
        V4L2_CONTROL_NAME_CASE(V4L2_CID_GAIN)
#endif
#ifdef V4L2_CID_HFLIP
        V4L2_CONTROL_NAME_CASE(V4L2_CID_HFLIP)
#endif
#ifdef V4L2_CID_VFLIP
        V4L2_CONTROL_NAME_CASE(V4L2_CID_VFLIP)
#endif
#ifdef V4L2_CID_POWER_LINE_FREQUENCY
        V4L2_CONTROL_NAME_CASE(V4L2_CID_POWER_LINE_FREQUENCY)
#endif
#ifdef V4L2_CID_HUE_AUTO
        V4L2_CONTROL_NAME_CASE(V4L2_CID_HUE_AUTO)
#endif
#ifdef V4L2_CID_WHITE_BALANCE_TEMPERATURE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_WHITE_BALANCE_TEMPERATURE)
#endif
#ifdef V4L2_CID_SHARPNESS
        V4L2_CONTROL_NAME_CASE(V4L2_CID_SHARPNESS)
#endif
#ifdef V4L2_CID_BACKLIGHT_COMPENSATION
        V4L2_CONTROL_NAME_CASE(V4L2_CID_BACKLIGHT_COMPENSATION)
#endif
#ifdef V4L2_CID_CHROMA_AGC
        V4L2_CONTROL_NAME_CASE(V4L2_CID_CHROMA_AGC)
#endif
#ifdef V4L2_CID_COLOR_KILLER
        V4L2_CONTROL_NAME_CASE(V4L2_CID_COLOR_KILLER)
#endif
#ifdef V4L2_CID_COLORFX
        V4L2_CONTROL_NAME_CASE(V4L2_CID_COLORFX)
#endif
#ifdef V4L2_CID_AUTOBRIGHTNESS
        V4L2_CONTROL_NAME_CASE(V4L2_CID_AUTOBRIGHTNESS)
#endif
#ifdef V4L2_CID_ROTATE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_ROTATE)
#endif
#ifdef V4L2_CID_CHROMA_GAIN
        V4L2_CONTROL_NAME_CASE(V4L2_CID_CHROMA_GAIN)
#endif
#ifdef V4L2_CID_ALPHA_COMPONENT
        V4L2_CONTROL_NAME_CASE(V4L2_CID_ALPHA_COMPONENT)
#endif
#ifdef V4L2_CID_EXPOSURE_AUTO
        V4L2_CONTROL_NAME_CASE(V4L2_CID_EXPOSURE_AUTO)
#endif
#ifdef V4L2_CID_EXPOSURE_ABSOLUTE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_EXPOSURE_ABSOLUTE)
#endif
#ifdef V4L2_CID_EXPOSURE_AUTO_PRIORITY
        V4L2_CONTROL_NAME_CASE(V4L2_CID_EXPOSURE_AUTO_PRIORITY)
#endif
#ifdef V4L2_CID_PAN_RELATIVE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_PAN_RELATIVE)
#endif
#ifdef V4L2_CID_TILT_RELATIVE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_TILT_RELATIVE)
#endif
#ifdef V4L2_CID_PAN_RESET
        V4L2_CONTROL_NAME_CASE(V4L2_CID_PAN_RESET)
#endif
#ifdef V4L2_CID_TILT_RESET
        V4L2_CONTROL_NAME_CASE(V4L2_CID_TILT_RESET)
#endif
#ifdef V4L2_CID_PAN_ABSOLUTE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_PAN_ABSOLUTE)
#endif
#ifdef V4L2_CID_TILT_ABSOLUTE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_TILT_ABSOLUTE)
#endif
#ifdef V4L2_CID_FOCUS_ABSOLUTE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_FOCUS_ABSOLUTE)
#endif
#ifdef V4L2_CID_FOCUS_RELATIVE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_FOCUS_RELATIVE)
#endif
#ifdef V4L2_CID_FOCUS_AUTO
        V4L2_CONTROL_NAME_CASE(V4L2_CID_FOCUS_AUTO)
#endif
#ifdef V4L2_CID_ZOOM_ABSOLUTE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_ZOOM_ABSOLUTE)
#endif
#ifdef V4L2_CID_ZOOM_RELATIVE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_ZOOM_RELATIVE)
#endif
#ifdef V4L2_CID_ZOOM_CONTINUOUS
        V4L2_CONTROL_NAME_CASE(V4L2_CID_ZOOM_CONTINUOUS)
#endif
#ifdef V4L2_CID_PRIVACY
        V4L2_CONTROL_NAME_CASE(V4L2_CID_PRIVACY)
#endif
#ifdef V4L2_CID_IRIS_ABSOLUTE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_IRIS_ABSOLUTE)
#endif
#ifdef V4L2_CID_IRIS_RELATIVE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_IRIS_RELATIVE)
#endif
#ifdef V4L2_CID_AUTO_EXPOSURE_BIAS
        V4L2_CONTROL_NAME_CASE(V4L2_CID_AUTO_EXPOSURE_BIAS)
#endif
#ifdef V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE)
#endif
#ifdef V4L2_CID_WIDE_DYNAMIC_RANGE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_WIDE_DYNAMIC_RANGE)
#endif
#ifdef V4L2_CID_IMAGE_STABILIZATION
        V4L2_CONTROL_NAME_CASE(V4L2_CID_IMAGE_STABILIZATION)
#endif
#ifdef V4L2_CID_ISO_SENSITIVITY
        V4L2_CONTROL_NAME_CASE(V4L2_CID_ISO_SENSITIVITY)
#endif
#ifdef V4L2_CID_ISO_SENSITIVITY_AUTO
        V4L2_CONTROL_NAME_CASE(V4L2_CID_ISO_SENSITIVITY_AUTO)
#endif
#ifdef V4L2_CID_EXPOSURE_METERING
        V4L2_CONTROL_NAME_CASE(V4L2_CID_EXPOSURE_METERING)
#endif
#ifdef V4L2_CID_SCENE_MODE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_SCENE_MODE)
#endif
#ifdef V4L2_CID_3A_LOCK
        V4L2_CONTROL_NAME_CASE(V4L2_CID_3A_LOCK)
#endif
#ifdef V4L2_CID_AUTO_FOCUS_START
        V4L2_CONTROL_NAME_CASE(V4L2_CID_AUTO_FOCUS_START)
#endif
#ifdef V4L2_CID_AUTO_FOCUS_STOP
        V4L2_CONTROL_NAME_CASE(V4L2_CID_AUTO_FOCUS_STOP)
#endif
#ifdef V4L2_CID_AUTO_FOCUS_STATUS
        V4L2_CONTROL_NAME_CASE(V4L2_CID_AUTO_FOCUS_STATUS)
#endif
#ifdef V4L2_CID_AUTO_FOCUS_RANGE
        V4L2_CONTROL_NAME_CASE(V4L2_CID_AUTO_FOCUS_RANGE)
#endif
#ifdef V4L2_CID_PAN_SPEED
        V4L2_CONTROL_NAME_CASE(V4L2_CID_PAN_SPEED)
#endif
#ifdef V4L2_CID_TILT_SPEED
        V4L2_CONTROL_NAME_CASE(V4L2_CID_TILT_SPEED)
#endif
#ifdef V4L2_CID_VBLANK
        V4L2_CONTROL_NAME_CASE(V4L2_CID_VBLANK)
#endif
#ifdef V4L2_CID_HBLANK
        V4L2_CONTROL_NAME_CASE(V4L2_CID_HBLANK)
#endif
#ifdef V4L2_CID_ANALOGUE_GAIN
        V4L2_CONTROL_NAME_CASE(V4L2_CID_ANALOGUE_GAIN)
#endif
#ifdef V4L2_CID_TEST_PATTERN
        V4L2_CONTROL_NAME_CASE(V4L2_CID_TEST_PATTERN)
#endif
    default:
        return {};
    }

#undef V4L2_CONTROL_NAME_CASE
}

QString controlApiId(const V4L2Camera::ControlInfo &control)
{
    const auto symbolicName = symbolicControlName(control.id);
    if (symbolicName.isEmpty())
        return hexId(control.id);
    return QStringLiteral("%1 (%2)").arg(symbolicName, hexId(control.id));
}

QString menuValueName(const V4L2Camera::ControlInfo &control, qint64 value)
{
    for (const auto &entry : control.menu) {
        if (entry.value == value)
            return entry.name;
    }
    return {};
}

QString rawControlValue(const V4L2Camera::ControlInfo &control, qint64 value)
{
    if (control.type == V4L2_CTRL_TYPE_BITMASK)
        return QStringLiteral("%1 / %2").arg(hexValue(control, value), QString::number(value));
    return QString::number(value);
}

QString formatControlValue(const V4L2Camera::ControlInfo &control, qint64 value)
{
    if (control.type == V4L2_CTRL_TYPE_BOOLEAN)
        return value != 0 ? QStringLiteral("true (1)") : QStringLiteral("false (0)");

    if (control.type == V4L2_CTRL_TYPE_MENU || control.type == V4L2_CTRL_TYPE_INTEGER_MENU) {
        const auto name = menuValueName(control, value);
        if (!name.isEmpty())
            return QStringLiteral("%1 [raw: %2]").arg(name, rawControlValue(control, value));
        return QStringLiteral("Unknown [raw: %1]").arg(rawControlValue(control, value));
    }

    if (control.type == V4L2_CTRL_TYPE_BITMASK)
        return rawControlValue(control, value);

    return QString::number(value);
}

QString menuEntryText(const V4L2Camera::ControlInfo &control, const V4L2Camera::MenuEntry &entry)
{
    if (entry.name.isEmpty())
        return QStringLiteral("Raw %1").arg(rawControlValue(control, entry.value));
    return QStringLiteral("%1 [raw: %2]").arg(entry.name, rawControlValue(control, entry.value));
}

QString editorTextForValue(const V4L2Camera::ControlInfo &control, qint64 value)
{
    if (control.type == V4L2_CTRL_TYPE_BITMASK)
        return hexValue(control, value);
    return QString::number(value);
}

bool parseControlValue(const V4L2Camera::ControlInfo &control, const QString &text, qint64 *value)
{
    if (value == nullptr)
        return false;

    const auto trimmed = text.trimmed();
    if (trimmed.isEmpty())
        return false;

    if (control.type == V4L2_CTRL_TYPE_BITMASK) {
        bool ok = false;
        const auto parsed = trimmed.toULongLong(&ok, 0);
        if (!ok || parsed > static_cast<qulonglong>(std::numeric_limits<qint64>::max()))
            return false;
        if (parsed < static_cast<qulonglong>(std::max<qint64>(0, control.minimum))
            || parsed > static_cast<qulonglong>(std::max<qint64>(0, control.maximum)))
            return false;
        *value = static_cast<qint64>(parsed);
        return true;
    }

    bool ok = false;
    const auto parsed = trimmed.toLongLong(&ok, 0);
    if (!ok)
        return false;
    *value = clampControlValue(control, parsed);
    return true;
}

QString controlFlagsText(const V4L2Camera::ControlInfo &control)
{
    QStringList names;
    if ((control.flags & V4L2_CTRL_FLAG_DISABLED) != 0)
        names << QStringLiteral("disabled");
    if ((control.flags & V4L2_CTRL_FLAG_GRABBED) != 0)
        names << QStringLiteral("grabbed");
    if ((control.flags & V4L2_CTRL_FLAG_READ_ONLY) != 0)
        names << QStringLiteral("read-only");
    if ((control.flags & V4L2_CTRL_FLAG_UPDATE) != 0)
        names << QStringLiteral("update");
    if ((control.flags & V4L2_CTRL_FLAG_INACTIVE) != 0)
        names << QStringLiteral("inactive");
    if ((control.flags & V4L2_CTRL_FLAG_SLIDER) != 0)
        names << QStringLiteral("slider");
    if ((control.flags & V4L2_CTRL_FLAG_WRITE_ONLY) != 0)
        names << QStringLiteral("write-only");
    if ((control.flags & V4L2_CTRL_FLAG_VOLATILE) != 0)
        names << QStringLiteral("volatile");
#ifdef V4L2_CTRL_FLAG_HAS_PAYLOAD
    if ((control.flags & V4L2_CTRL_FLAG_HAS_PAYLOAD) != 0)
        names << QStringLiteral("has-payload");
#endif
#ifdef V4L2_CTRL_FLAG_EXECUTE_ON_WRITE
    if ((control.flags & V4L2_CTRL_FLAG_EXECUTE_ON_WRITE) != 0)
        names << QStringLiteral("execute-on-write");
#endif
#ifdef V4L2_CTRL_FLAG_MODIFY_LAYOUT
    if ((control.flags & V4L2_CTRL_FLAG_MODIFY_LAYOUT) != 0)
        names << QStringLiteral("modify-layout");
#endif
#ifdef V4L2_CTRL_FLAG_DYNAMIC_ARRAY
    if ((control.flags & V4L2_CTRL_FLAG_DYNAMIC_ARRAY) != 0)
        names << QStringLiteral("dynamic-array");
#endif
#ifdef V4L2_CTRL_FLAG_HAS_WHICH_MIN_MAX
    if ((control.flags & V4L2_CTRL_FLAG_HAS_WHICH_MIN_MAX) != 0)
        names << QStringLiteral("has-which-min-max");
#endif

    const auto hexFlags = QStringLiteral("0x%1").arg(control.flags, 0, 16);
    if (names.isEmpty())
        return QStringLiteral("%1 (none)").arg(hexFlags);
    return QStringLiteral("%1 (%2)").arg(hexFlags, names.join(QStringLiteral(", ")));
}

QString staticControlStateReason(const V4L2Camera::ControlInfo &control)
{
    if (control.isDisabled())
        return QStringLiteral("Disabled by driver.");
    if (control.isReadOnly())
        return QStringLiteral("Read-only control.");
    if (control.isWriteOnly())
        return QStringLiteral("Write-only control.");
    if (control.isInactive())
        return QStringLiteral("Inactive until a related control or stream state changes.");
    if (isGrabbed(control))
        return QStringLiteral("Temporarily locked by the driver.");
    return {};
}

QString unsupportedControlReason(const V4L2Camera::ControlInfo &control)
{
    if (hasPayload(control))
        return QStringLiteral("Unsupported V4L2 control type: %1 with payload.").arg(V4L2Camera::controlTypeName(control.type));
    if (!control.supported)
        return QStringLiteral("Unsupported V4L2 control type: %1.").arg(V4L2Camera::controlTypeName(control.type));
    return {};
}

QString controlTooltip(const V4L2Camera::ControlInfo &control, const QString &disabledReason = QString())
{
    QStringList lines;
    lines << QStringLiteral("Name: %1").arg(control.name);
    lines << QStringLiteral("API ID: %1").arg(controlApiId(control));
    lines << QStringLiteral("Class: %1").arg(control.className());
    lines << QStringLiteral("Type: %1").arg(V4L2Camera::controlTypeName(control.type));
    lines << QStringLiteral("Current: %1").arg(formatControlValue(control, control.currentValue));
    lines << QStringLiteral("Default: %1").arg(formatControlValue(control, control.defaultValue));
    lines << QStringLiteral("Range: %1 to %2, step %3")
                 .arg(
                     formatControlValue(control, control.minimum),
                     formatControlValue(control, control.maximum),
                     QString::number(control.step));
    lines << QStringLiteral("Flags: %1").arg(controlFlagsText(control));

    QStringList access;
    access << (control.canRead() ? QStringLiteral("readable") : QStringLiteral("not readable"));
    access << (control.canWrite() ? QStringLiteral("writable") : QStringLiteral("not writable"));
    if (control.restorable())
        access << QStringLiteral("resettable");
    lines << QStringLiteral("Access: %1").arg(access.join(QStringLiteral(", ")));

    const auto unsupportedReason = unsupportedControlReason(control);
    const auto stateReason = !disabledReason.isEmpty()
        ? disabledReason
        : (!unsupportedReason.isEmpty() ? unsupportedReason : staticControlStateReason(control));
    if (!stateReason.isEmpty())
        lines << QStringLiteral("State: %1").arg(stateReason);

    return lines.join(QLatin1Char('\n'));
}

bool shouldUseSlider(const V4L2Camera::ControlInfo &control)
{
    if (control.type != V4L2_CTRL_TYPE_INTEGER)
        return false;
    if (!fitsInt(control.minimum) || !fitsInt(control.maximum))
        return false;
    const auto range = control.maximum - control.minimum;
    return (control.flags & V4L2_CTRL_FLAG_SLIDER) != 0 || range <= 10000;
}

QString autoDisabledReason(
    quint32 id,
    const QHash<quint32, V4L2Camera::ControlInfo> &controls,
    const QHash<quint32, qint64> &values)
{
    const auto table = V4L2Camera::autoDependencyTable();
    for (auto it = table.constBegin(); it != table.constEnd(); ++it) {
        if (!it.value().contains(id))
            continue;

        const auto autoControlIt = controls.constFind(it.key());
        const auto autoValueIt = values.constFind(it.key());
        if (autoControlIt == controls.constEnd() || autoValueIt == values.constEnd())
            continue;
        if (!V4L2Camera::autoControlEnabled(it.key(), autoValueIt.value()))
            continue;

        return QStringLiteral("Disabled while %1 is enabled.").arg(autoControlIt->name);
    }
    return {};
}

int defaultFormatRank(const V4L2Camera::CaptureMode &mode)
{
    if (mode.fourcc == V4L2_PIX_FMT_YUYV)
        return 0;
    if (mode.fourcc == V4L2_PIX_FMT_GREY)
        return 1;
    if (mode.fourcc == V4L2_PIX_FMT_MJPEG || mode.fourcc == v4l2_fourcc('M', 'J', 'P', 'G'))
        return 2;
    return 3;
}

bool betterDefaultTieBreak(const V4L2Camera::CaptureMode &a, const V4L2Camera::CaptureMode &b, double targetFps)
{
    if (a.emulated != b.emulated)
        return !a.emulated;
    if (a.compressed != b.compressed)
        return !a.compressed;

    const auto aFormatRank = defaultFormatRank(a);
    const auto bFormatRank = defaultFormatRank(b);
    if (aFormatRank != bFormatRank)
        return aFormatRank < bFormatRank;

    const auto aFpsDistance = std::abs(a.fps() - targetFps);
    const auto bFpsDistance = std::abs(b.fps() - targetFps);
    if (!qFuzzyCompare(aFpsDistance + 1.0, bFpsDistance + 1.0))
        return aFpsDistance < bFpsDistance;

    return a.fps() > b.fps();
}

int preferredExactModeIndex(const QList<V4L2Camera::CaptureMode> &modes, int width, int height, double fps)
{
    int bestIndex = -1;
    for (int i = 0; i < modes.size(); ++i) {
        const auto &mode = modes.at(i);
        if (mode.width != width || mode.height != height)
            continue;
        if (std::abs(mode.fps() - fps) > 0.15)
            continue;
        if (bestIndex < 0 || betterDefaultTieBreak(mode, modes.at(bestIndex), fps))
            bestIndex = i;
    }
    return bestIndex;
}

double fallbackDefaultScore(const V4L2Camera::CaptureMode &mode)
{
    constexpr double targetFps = 30.0;
    constexpr double targetPixels = 1920.0 * 1080.0;
    constexpr double targetAspect = 16.0 / 9.0;

    const auto fps = mode.fps();
    double score = std::abs(fps - targetFps) * 80.0;
    if (fps < 10.0)
        score += 2000.0;
    else if (fps > 60.0)
        score += (fps - 60.0) * 20.0;

    const auto pixels = static_cast<double>(mode.width) * static_cast<double>(mode.height);
    if (pixels <= targetPixels)
        score += (targetPixels - pixels) / 1000.0;
    else
        score += 2000.0 + (pixels - targetPixels) / 250.0;

    const auto aspect = static_cast<double>(mode.width) / static_cast<double>(mode.height);
    score += std::abs(aspect - targetAspect) * 300.0;

    if (mode.emulated)
        score += 1000.0;
    if (mode.compressed)
        score += 250.0;
    score += defaultFormatRank(mode) * 25.0;

    return score;
}

int preferredDefaultModeIndex(const QList<V4L2Camera::CaptureMode> &modes)
{
    static const struct {
        int width;
        int height;
        double fps;
    } preferredModes[] = {
        {1920, 1080, 30.0},
        {1280, 720,  30.0},
        {640,  480,  30.0},
        {1920, 1080, 15.0},
        {1280, 720,  15.0},
        {640,  480,  15.0},
    };

    for (const auto &preferred : preferredModes) {
        const auto index = preferredExactModeIndex(modes, preferred.width, preferred.height, preferred.fps);
        if (index >= 0)
            return index;
    }

    int bestIndex = -1;
    double bestScore = std::numeric_limits<double>::max();
    for (int i = 0; i < modes.size(); ++i) {
        const auto score = fallbackDefaultScore(modes.at(i));
        if (score < bestScore) {
            bestIndex = i;
            bestScore = score;
        }
    }
    return bestIndex;
}

} // namespace

V4L2SettingsDialog::V4L2SettingsDialog(QWidget *parent)
    : QDialog(parent),
      m_tabs(nullptr),
      m_captureTab(nullptr),
      m_deviceCombo(nullptr),
      m_modeCombo(nullptr),
      m_summaryLabel(nullptr),
      m_effectiveLabel(nullptr),
      m_refreshButton(nullptr),
      m_running(false),
      m_blockUiSignals(false),
      m_applyLoadedControlValues(false)
{
    qRegisterMetaType<V4L2Camera::DeviceIdentity>();
    qRegisterMetaType<V4L2Camera::CaptureMode>();
    qRegisterMetaType<V4L2Camera::ControlInfo>();

    buildUi();
    refreshDevices();
}

V4L2Camera::DeviceIdentity V4L2SettingsDialog::selectedDevice() const
{
    return m_deviceCombo->currentData().value<V4L2Camera::DeviceIdentity>();
}

V4L2Camera::CaptureMode V4L2SettingsDialog::selectedMode() const
{
    return m_modeCombo->currentData().value<V4L2Camera::CaptureMode>();
}

QList<V4L2Camera::ControlInfo> V4L2SettingsDialog::controls() const
{
    QList<V4L2Camera::ControlInfo> result;
    result.reserve(m_controls.size());
    for (const auto &control : m_controls)
        result.append(control);
    return result;
}

QHash<quint32, qint64> V4L2SettingsDialog::desiredControlValues() const
{
    return m_desiredValues;
}

void V4L2SettingsDialog::setRunning(bool running)
{
    m_running = running;
    m_deviceCombo->setEnabled(!running);
    m_modeCombo->setEnabled(!running);
    m_refreshButton->setEnabled(!running);
    updateDependencyStates();
}

void V4L2SettingsDialog::setEffectiveMode(const V4L2Camera::CaptureMode &mode)
{
    if (!mode.isValid()) {
        m_effectiveLabel->setText(QStringLiteral("Not configured"));
        return;
    }

    m_effectiveLabel->setText(
        QStringLiteral("%1, stride %2, colorspace %3, field %4")
            .arg(mode.displayName())
            .arg(mode.bytesPerLine)
            .arg(mode.colorspace)
            .arg(mode.field));
}

void V4L2SettingsDialog::updateControlReadback(quint32 id, qint64 value)
{
    if (!m_controls.contains(id))
        return;

    m_controls[id].currentValue = value;
    m_desiredValues[id] = value;
    setControlWidgetValue(id, value);
    updateDependencyStates();
}

void V4L2SettingsDialog::updateControls(const QList<V4L2Camera::ControlInfo> &controls, const QSet<quint32> &affectedIds)
{
    if (affectedIds.isEmpty())
        return;

    QHash<quint32, V4L2Camera::ControlInfo> queriedControls;
    for (const auto &control : controls) {
        if (!control.isClassMarker() && !control.isDisabled())
            queriedControls.insert(control.id, control);
    }

    for (const auto id : affectedIds) {
        const auto it = queriedControls.constFind(id);
        if (it == queriedControls.constEnd() || !m_controlWidgets.contains(id))
            continue;

        m_controls[id] = it.value();
        m_desiredValues[id] = it->currentValue;
        setControlWidgetValue(id, it->currentValue);
    }

    updateDependencyStates();
}

void V4L2SettingsDialog::replaceControls(const QList<V4L2Camera::ControlInfo> &controls)
{
    rebuildControls(controls);
}

void V4L2SettingsDialog::refreshDevices()
{
    const auto previous = selectedDevice();
    m_applyLoadedControlValues = false;

    QString enumError;
    const auto devices = V4L2Camera::enumerateDevices(&enumError);

    QSignalBlocker blocker(m_deviceCombo);
    m_deviceCombo->clear();
    for (const auto &device : devices)
        m_deviceCombo->addItem(device.displayName(), QVariant::fromValue(device));

    if (devices.isEmpty()) {
        if (!m_loadedIdentity.isValid() && previous.isValid())
            m_pendingIdentity = previous;

        QString message = enumError.isEmpty() ? QStringLiteral("No V4L2 capture devices found.") : enumError;
        if (m_pendingIdentity.isValid())
            message = QStringLiteral("%1\nWaiting for previously selected camera: %2.")
                          .arg(message, m_pendingIdentity.displayName());
        m_summaryLabel->setText(message);
        m_modeCombo->clear();
        clearControlTabs();
        return;
    }

    auto indexForDevice = [this](const V4L2Camera::DeviceIdentity &wanted) {
        for (int i = 0; i < m_deviceCombo->count(); ++i) {
            const auto device = m_deviceCombo->itemData(i).value<V4L2Camera::DeviceIdentity>();
            if (device.devicePath == wanted.devicePath)
                return i;
        }
        return -1;
    };

    V4L2Camera::DeviceMatch match;
    if (m_loadedIdentity.isValid()) {
        match = V4L2Camera::matchDevice(m_loadedIdentity, devices, enumError);
        m_applyLoadedControlValues = match.trustedForSavedSettings();
        if (!match.warning.isEmpty()) {
            qWarning().noquote() << "camera-v4l2:" << match.warning;
            QMessageBox::warning(this, QStringLiteral("V4L2 Camera Match"), match.warning);
        }
        if (!match.hasDevice()) {
            m_deviceCombo->setCurrentIndex(-1);
            m_summaryLabel->setText(match.warning);
            m_modeCombo->clear();
            clearControlTabs();
            return;
        }
    } else if (m_pendingIdentity.isValid()) {
        match = V4L2Camera::matchDevice(m_pendingIdentity, devices, enumError);
        if (!match.hasDevice()) {
            m_deviceCombo->setCurrentIndex(-1);
            m_summaryLabel->setText(
                QStringLiteral("Waiting for previously selected V4L2 camera: %1.").arg(m_pendingIdentity.displayName()));
            m_modeCombo->clear();
            clearControlTabs();
            return;
        }
        m_pendingIdentity = {};
    } else if (previous.isValid()) {
        match = V4L2Camera::matchDevice(previous, devices, enumError);
        if (!match.hasDevice()) {
            m_pendingIdentity = previous;
            m_deviceCombo->setCurrentIndex(-1);
            m_summaryLabel->setText(
                QStringLiteral("Previously selected V4L2 camera is not available: %1.").arg(previous.displayName()));
            m_modeCombo->clear();
            clearControlTabs();
            return;
        }
    }
    if (!match.hasDevice())
        match = V4L2Camera::matchDevice({}, devices, enumError);

    m_deviceCombo->setCurrentIndex(indexForDevice(match.device));
    onDeviceChanged(m_deviceCombo->currentIndex());
}

void V4L2SettingsDialog::serializeSettings(QVariantHash &settings) const
{
    auto identity = selectedDevice();
    if (!identity.isValid())
        identity = m_loadedIdentity;

    auto mode = selectedMode();
    if (!mode.isValid())
        mode = m_loadedMode;

    settings.insert(QStringLiteral("device_identity"), identity.toVariant());
    settings.insert(QStringLiteral("capture_mode"), mode.toVariant());
    settings.insert(QStringLiteral("control_values"), serializableControlSettings());
}

void V4L2SettingsDialog::loadSettings(const QVariantHash &settings)
{
    m_loadedIdentity = V4L2Camera::DeviceIdentity::fromVariant(settings.value(QStringLiteral("device_identity")).toHash());
    m_pendingIdentity = {};
    m_loadedMode = V4L2Camera::CaptureMode::fromVariant(settings.value(QStringLiteral("capture_mode")).toHash());
    m_loadedControlValues.clear();

    const auto controlValues = settings.value(QStringLiteral("control_values")).toList();
    for (const auto &entryVar : controlValues) {
        const auto entry = entryVar.toHash();
        const quint32 id = entry.value(QStringLiteral("id")).toUInt();
        if (id == 0)
            continue;
        m_loadedControlValues[id] = entry.value(QStringLiteral("value")).toLongLong();
    }

    refreshDevices();
}

void V4L2SettingsDialog::onDeviceChanged(int index)
{
    if (index < 0)
        return;

    m_pendingIdentity = {};

    const auto deviceInfo = selectedDevice();
    m_controls.clear();
    m_desiredValues.clear();
    clearControlTabs();
    m_modeCombo->clear();

    V4L2Camera::Device device;
    QString error;
    if (!device.open(deviceInfo.devicePath, &error)) {
        m_summaryLabel->setText(error);
        return;
    }

    const auto modes = device.enumerateCaptureModes(&error);
    populateModes(modes);

    const bool applyLoadedValues = m_applyLoadedControlValues;
    m_applyLoadedControlValues = false;
    auto queriedControls = device.queryControls(&error);
    rebuildControls(queriedControls, applyLoadedValues);
    if (applyLoadedValues)
        m_loadedControlValues.clear();
    updateSummary();
}

void V4L2SettingsDialog::onModeChanged(int)
{
    updateSummary();
}

void V4L2SettingsDialog::onRefreshClicked()
{
    refreshDevices();
}

void V4L2SettingsDialog::readControlClass(const QString &className)
{
    const auto deviceInfo = selectedDevice();
    if (!deviceInfo.isValid())
        return;

    V4L2Camera::Device device;
    QString error;
    if (!device.open(deviceInfo.devicePath, &error)) {
        QMessageBox::warning(this, QStringLiteral("V4L2 Camera"), error);
        return;
    }

    auto queriedControls = device.queryControls(&error);
    if (queriedControls.isEmpty() && !error.isEmpty())
        QMessageBox::warning(this, QStringLiteral("V4L2 Camera"), error);

    QSet<quint32> affectedIds;
    for (const auto &control : queriedControls) {
        if (control.isClassMarker() || control.isDisabled())
            continue;
        if (control.className() == className && m_controlWidgets.contains(control.id))
            affectedIds.insert(control.id);
    }
    updateControls(queriedControls, affectedIds);
}

void V4L2SettingsDialog::resetControlClass(const QString &className)
{
    const auto values = m_desiredValues;
    QList<quint32> ids;
    for (auto it = m_controls.begin(); it != m_controls.end(); ++it) {
        const auto &control = it.value();
        if (control.className() != className)
            continue;
        if (!control.restorable())
            continue;
        if (V4L2Camera::isManualDependentActive(control.id, values))
            continue;
        if (values.value(control.id, control.currentValue) == control.defaultValue)
            continue;
        ids.append(control.id);
    }

    std::sort(ids.begin(), ids.end());
    for (const auto id : ids) {
        if (m_controls.contains(id))
            handleControlEdited(id, m_controls.value(id).defaultValue);
    }
}

void V4L2SettingsDialog::buildUi()
{
    setWindowTitle(QStringLiteral("V4L2 Camera Settings"));
    resize(760, 620);

    auto *mainLayout = new QVBoxLayout(this);
    m_tabs = new QTabWidget(this);
    mainLayout->addWidget(m_tabs);

    m_captureTab = new QWidget(this);
    auto *captureLayout = new QVBoxLayout(m_captureTab);
    auto *captureGroup = new QGroupBox(QStringLiteral("Capture"), m_captureTab);
    auto *form = new QFormLayout(captureGroup);

    auto *deviceRow = new QWidget(captureGroup);
    auto *deviceLayout = new QHBoxLayout(deviceRow);
    deviceLayout->setContentsMargins(0, 0, 0, 0);
    m_deviceCombo = new QComboBox(deviceRow);
    m_refreshButton = new QPushButton(QStringLiteral("Refresh"), deviceRow);
    deviceLayout->addWidget(m_deviceCombo, 1);
    deviceLayout->addWidget(m_refreshButton);
    form->addRow(QStringLiteral("Device"), deviceRow);

    m_modeCombo = new QComboBox(captureGroup);
    form->addRow(QStringLiteral("Mode"), m_modeCombo);

    m_effectiveLabel = new QLabel(QStringLiteral("Not configured"), captureGroup);
    m_effectiveLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    form->addRow(QStringLiteral("Effective"), m_effectiveLabel);

    m_summaryLabel = new QLabel(captureGroup);
    m_summaryLabel->setWordWrap(true);
    m_summaryLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    form->addRow(QStringLiteral("Identity"), m_summaryLabel);

    captureLayout->addWidget(captureGroup);
    captureLayout->addStretch();
    m_tabs->addTab(m_captureTab, QStringLiteral("Capture"));

    connect(m_deviceCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &V4L2SettingsDialog::onDeviceChanged);
    connect(m_modeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &V4L2SettingsDialog::onModeChanged);
    connect(m_refreshButton, &QPushButton::clicked, this, &V4L2SettingsDialog::onRefreshClicked);
}

void V4L2SettingsDialog::clearControlTabs()
{
    m_controlWidgets.clear();
    m_classResetButtons.clear();
    while (m_tabs->count() > 1) {
        auto *widget = m_tabs->widget(1);
        m_tabs->removeTab(1);
        widget->deleteLater();
    }
}

void V4L2SettingsDialog::populateModes(const QList<V4L2Camera::CaptureMode> &modes)
{
    QSignalBlocker blocker(m_modeCombo);
    m_modeCombo->clear();
    for (const auto &mode : modes)
        m_modeCombo->addItem(mode.displayName(), QVariant::fromValue(mode));

    int selectedIndex = -1;
    for (int i = 0; i < m_modeCombo->count(); ++i) {
        const auto mode = m_modeCombo->itemData(i).value<V4L2Camera::CaptureMode>();
        if (m_loadedMode.isValid() && m_loadedMode.sameSelection(mode)) {
            selectedIndex = i;
            break;
        }
    }
    const bool loadedModeMissed = m_loadedMode.isValid() && selectedIndex < 0;
    if (selectedIndex < 0 && m_modeCombo->count() > 0)
        selectedIndex = preferredDefaultModeIndex(modes);
    m_modeCombo->setCurrentIndex(selectedIndex);

    if (loadedModeMissed) {
        const auto fallbackMode = selectedMode();
        const auto message = fallbackMode.isValid()
            ? QStringLiteral("Saved V4L2 capture mode is not available: %1. Using %2 instead.")
                  .arg(m_loadedMode.displayName(), fallbackMode.displayName())
            : QStringLiteral("Saved V4L2 capture mode is not available: %1. No supported fallback mode is available.")
                  .arg(m_loadedMode.displayName());
        qWarning().noquote() << "camera-v4l2:" << message;
        QMessageBox::warning(this, QStringLiteral("V4L2 Capture Mode"), message);
    }
}

void V4L2SettingsDialog::rebuildControls(const QList<V4L2Camera::ControlInfo> &controls, bool applyLoadedValues)
{
    clearControlTabs();
    m_controls.clear();
    m_desiredValues.clear();

    auto sorted = controls;
    std::sort(sorted.begin(), sorted.end(), [](const V4L2Camera::ControlInfo &a, const V4L2Camera::ControlInfo &b) {
        if (a.className() != b.className())
            return a.className() < b.className();
        const auto aKey = dependencySortKey(a.id);
        const auto bKey = dependencySortKey(b.id);
        if (aKey != bKey)
            return aKey < bKey;
        return a.name < b.name;
    });

    QHash<QString, QVBoxLayout *> tabLayouts;
    for (auto control : sorted) {
        if (control.isClassMarker() || control.isDisabled())
            continue;

        if (applyLoadedValues && m_loadedControlValues.contains(control.id))
            control.currentValue = clampControlValue(control, m_loadedControlValues.value(control.id));

        m_controls.insert(control.id, control);
        m_desiredValues.insert(control.id, control.currentValue);

        const auto className = control.className();
        if (!tabLayouts.contains(className)) {
            auto *scrollArea = new QScrollArea(m_tabs);
            scrollArea->setWidgetResizable(true);
            auto *page = new QWidget(scrollArea);
            auto *layout = new QVBoxLayout(page);
            layout->setContentsMargins(10, 10, 10, 10);
            layout->setSpacing(6);
            page->setLayout(layout);

            auto *buttonsRow = new QWidget(page);
            auto *buttonsLayout = new QHBoxLayout(buttonsRow);
            buttonsLayout->setContentsMargins(0, 0, 0, 4);
            buttonsLayout->addStretch();
            auto *readButton = new QPushButton(QStringLiteral("Read Tab"), buttonsRow);
            auto *resetButton = new QPushButton(QStringLiteral("Reset Tab"), buttonsRow);
            buttonsLayout->addWidget(readButton);
            buttonsLayout->addWidget(resetButton);
            layout->addWidget(buttonsRow);
            connect(readButton, &QPushButton::clicked, this, [this, className]() {
                readControlClass(className);
            });
            connect(resetButton, &QPushButton::clicked, this, [this, className]() {
                resetControlClass(className);
            });
            m_classResetButtons.insert(className, resetButton);

            scrollArea->setWidget(page);
            m_tabs->addTab(scrollArea, className);
            tabLayouts.insert(className, layout);
        }

        tabLayouts[className]->addWidget(createControlRow(control));
    }

    for (auto *layout : tabLayouts)
        layout->addStretch();
    updateDependencyStates();
}

QWidget *V4L2SettingsDialog::createControlRow(const V4L2Camera::ControlInfo &control)
{
    auto *row = new QWidget(this);
    auto *grid = new QGridLayout(row);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setColumnStretch(1, 1);

    ControlWidgets widgets;
    widgets.row = row;
    auto *nameLabel = new QLabel(control.name, row);
    widgets.nameLabel = nameLabel;
    grid->addWidget(nameLabel, 0, 0);

    widgets.stateLabel = new QLabel(row);
    widgets.stateLabel->setMinimumWidth(70);

    const auto unsupportedReason = unsupportedControlReason(control);
    if (!unsupportedReason.isEmpty()) {
        auto *label = new QLabel(unsupportedReason, row);
        label->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);
        widgets.editor = label;
        grid->addWidget(label, 0, 1);
    } else if (control.type == V4L2_CTRL_TYPE_BOOLEAN) {
        auto *checkBox = new QCheckBox(row);
        checkBox->setChecked(control.currentValue != 0);
        widgets.editor = checkBox;
        widgets.checkBox = checkBox;
        connect(checkBox, &QCheckBox::toggled, this, [this, id = control.id](bool checked) {
            if (!m_blockUiSignals)
                handleControlEdited(id, checked ? 1 : 0);
        });
        grid->addWidget(checkBox, 0, 1);
    } else if (control.type == V4L2_CTRL_TYPE_MENU || control.type == V4L2_CTRL_TYPE_INTEGER_MENU) {
        auto *comboBox = new QComboBox(row);
        for (const auto &entry : control.menu)
            comboBox->addItem(menuEntryText(control, entry), entry.value);
        widgets.editor = comboBox;
        widgets.comboBox = comboBox;
        connect(comboBox, qOverload<int>(&QComboBox::currentIndexChanged), this, [this, id = control.id](int index) {
            if (m_blockUiSignals || index < 0)
                return;
            const auto *combo = qobject_cast<QComboBox *>(sender());
            if (combo == nullptr)
                return;
            handleControlEdited(id, combo->itemData(index).toLongLong());
        });
        grid->addWidget(comboBox, 0, 1);
    } else if (control.type == V4L2_CTRL_TYPE_BUTTON) {
        auto *button = new QPushButton(QStringLiteral("Trigger"), row);
        widgets.editor = button;
        widgets.button = button;
        connect(button, &QPushButton::clicked, this, [this, id = control.id]() {
            if (!m_blockUiSignals)
                Q_EMIT buttonControlTriggered(id);
        });
        grid->addWidget(button, 0, 1);
    } else if (shouldUseSlider(control)) {
        auto *editor = new QWidget(row);
        auto *layout = new QHBoxLayout(editor);
        layout->setContentsMargins(0, 0, 0, 0);

        auto *slider = new QSlider(Qt::Horizontal, editor);
        slider->setRange(static_cast<int>(control.minimum), static_cast<int>(control.maximum));
        slider->setSingleStep(intStep(control.step));
        auto *spinBox = new QSpinBox(editor);
        spinBox->setRange(static_cast<int>(control.minimum), static_cast<int>(control.maximum));
        spinBox->setSingleStep(intStep(control.step));
        spinBox->setKeyboardTracking(false);

        layout->addWidget(slider, 1);
        layout->addWidget(spinBox);
        widgets.editor = editor;
        widgets.slider = slider;
        widgets.spinBox = spinBox;

        connect(slider, &QSlider::valueChanged, this, [this, id = control.id, spinBox, slider](int value) {
            if (m_blockUiSignals)
                return;
            QSignalBlocker blocker(spinBox);
            spinBox->setValue(value);
            if (!slider->isSliderDown())
                handleControlEdited(id, value);
        });
        connect(slider, &QSlider::sliderReleased, this, [this, id = control.id, slider]() {
            if (!m_blockUiSignals)
                handleControlEdited(id, slider->value());
        });
        connect(spinBox, qOverload<int>(&QSpinBox::valueChanged), this, [this, id = control.id, slider](int value) {
            if (m_blockUiSignals)
                return;
            QSignalBlocker blocker(slider);
            slider->setValue(value);
        });
        connect(spinBox, &QAbstractSpinBox::editingFinished, this, [this, id = control.id, spinBox, slider]() {
            if (m_blockUiSignals)
                return;
            QSignalBlocker blocker(slider);
            slider->setValue(spinBox->value());
            handleControlEdited(id, spinBox->value());
        });
        grid->addWidget(editor, 0, 1);
    } else if (control.type == V4L2_CTRL_TYPE_INTEGER && fitsInt(control.minimum) && fitsInt(control.maximum)) {
        auto *spinBox = new QSpinBox(row);
        spinBox->setRange(static_cast<int>(control.minimum), static_cast<int>(control.maximum));
        spinBox->setSingleStep(intStep(control.step));
        spinBox->setKeyboardTracking(false);
        widgets.editor = spinBox;
        widgets.spinBox = spinBox;
        connect(spinBox, &QAbstractSpinBox::editingFinished, this, [this, id = control.id, spinBox]() {
            if (!m_blockUiSignals)
                handleControlEdited(id, spinBox->value());
        });
        grid->addWidget(spinBox, 0, 1);
    } else if (isEditableNumericControl(control)) {
        auto *lineEdit = new QLineEdit(row);
        lineEdit->setClearButtonEnabled(false);
        widgets.editor = lineEdit;
        widgets.lineEdit = lineEdit;
        connect(lineEdit, &QLineEdit::editingFinished, this, [this, id = control.id, lineEdit]() {
            if (m_blockUiSignals || !m_controls.contains(id))
                return;

            qint64 parsedValue = 0;
            const auto &control = m_controls[id];
            if (parseControlValue(control, lineEdit->text(), &parsedValue)) {
                handleControlEdited(id, parsedValue);
            } else {
                setControlWidgetValue(id, m_desiredValues.value(id, control.currentValue));
            }
        });
        grid->addWidget(lineEdit, 0, 1);
    } else {
        auto *label = new QLabel(formatControlValue(control, control.currentValue), row);
        label->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);
        widgets.editor = label;
        grid->addWidget(label, 0, 1);
    }

    widgets.resetButton = new QPushButton(QStringLiteral("Reset"), row);
    connect(widgets.resetButton, &QPushButton::clicked, this, [this, id = control.id]() {
        if (m_controls.contains(id))
            handleControlEdited(id, m_controls.value(id).defaultValue);
    });

    grid->addWidget(widgets.stateLabel, 0, 2);
    grid->addWidget(widgets.resetButton, 0, 3);
    m_controlWidgets.insert(control.id, widgets);
    setControlWidgetValue(control.id, control.currentValue);
    updateControlPresentation(control.id);
    return row;
}

void V4L2SettingsDialog::setControlWidgetValue(quint32 id, qint64 value)
{
    if (!m_controlWidgets.contains(id))
        return;

    const bool oldBlockState = m_blockUiSignals;
    m_blockUiSignals = true;
    auto &widgets = m_controlWidgets[id];
    const auto control = m_controls.value(id);
    if (widgets.checkBox != nullptr) {
        widgets.checkBox->setChecked(value != 0);
    } else if (widgets.comboBox != nullptr) {
        bool found = false;
        for (int i = 0; i < widgets.comboBox->count(); ++i) {
            if (widgets.comboBox->itemData(i).toLongLong() == value) {
                widgets.comboBox->setCurrentIndex(i);
                found = true;
                break;
            }
        }
        if (!found) {
            widgets.comboBox->addItem(QStringLiteral("Unknown [raw: %1]").arg(rawControlValue(control, value)), value);
            widgets.comboBox->setCurrentIndex(widgets.comboBox->count() - 1);
        }
    } else if (widgets.slider != nullptr && widgets.spinBox != nullptr && fitsInt(value)) {
        widgets.slider->setValue(static_cast<int>(value));
        widgets.spinBox->setValue(static_cast<int>(value));
    } else if (widgets.spinBox != nullptr && fitsInt(value)) {
        widgets.spinBox->setValue(static_cast<int>(value));
    } else if (widgets.lineEdit != nullptr) {
        widgets.lineEdit->setText(editorTextForValue(control, value));
    } else if (auto *label = qobject_cast<QLabel *>(widgets.editor)) {
        label->setText(formatControlValue(control, value));
    }
    m_blockUiSignals = oldBlockState;
}

void V4L2SettingsDialog::handleControlEdited(quint32 id, qint64 value)
{
    if (!m_controls.contains(id))
        return;

    auto &control = m_controls[id];
    value = clampControlValue(control, value);
    control.currentValue = value;
    m_desiredValues[id] = value;
    setControlWidgetValue(id, value);
    updateDependencyStates();
    Q_EMIT controlValueChanged(id, value);
}

void V4L2SettingsDialog::updateControlPresentation(quint32 id, const QString &disabledReason)
{
    if (!m_controls.contains(id) || !m_controlWidgets.contains(id))
        return;

    const auto &control = m_controls[id];
    auto &widgets = m_controlWidgets[id];
    const auto tooltip = controlTooltip(control, disabledReason);
    const auto applyTooltip = [&tooltip](QWidget *widget) {
        if (widget != nullptr)
            widget->setToolTip(tooltip);
    };

    applyTooltip(widgets.row);
    applyTooltip(widgets.nameLabel);
    applyTooltip(widgets.editor);
    applyTooltip(widgets.slider);
    applyTooltip(widgets.spinBox);
    applyTooltip(widgets.lineEdit);
    applyTooltip(widgets.comboBox);
    applyTooltip(widgets.checkBox);
    applyTooltip(widgets.button);
    applyTooltip(widgets.stateLabel);

    if (widgets.resetButton != nullptr) {
        widgets.resetButton->setText(QStringLiteral("Reset"));
        widgets.resetButton->setToolTip(
            QStringLiteral("Reset to default value: %1").arg(formatControlValue(control, control.defaultValue)));
    }
}

void V4L2SettingsDialog::updateSummary()
{
    const auto device = selectedDevice();
    if (!device.isValid()) {
        m_summaryLabel->setText(QStringLiteral("No device selected."));
        return;
    }

    const auto mode = selectedMode();
    m_summaryLabel->setText(
        QStringLiteral("%1\nDriver: %2\nBus: %3\nCaps: 0x%4\nUSB: %5:%6 %7\nSelected: %8")
            .arg(
                device.displayName(),
                device.driver,
                device.busInfo,
                QString::number(device.deviceCaps, 16),
                device.usbVid.isEmpty() ? QStringLiteral("-") : device.usbVid,
                device.usbPid.isEmpty() ? QStringLiteral("-") : device.usbPid,
                device.serial.isEmpty() ? QStringLiteral("-") : device.serial,
                mode.isValid() ? mode.displayName() : QStringLiteral("none")));
}

void V4L2SettingsDialog::updateDependencyStates()
{
    QHash<QString, bool> classHasModifiedReset;
    for (auto it = m_controlWidgets.begin(); it != m_controlWidgets.end(); ++it) {
        const auto id = it.key();
        if (!m_controls.contains(id))
            continue;

        const auto &control = m_controls[id];
        const bool autoDisabled = V4L2Camera::isManualDependentActive(id, m_desiredValues);
        const bool writable = control.canWrite() && !isGrabbed(control) && !autoDisabled && (!control.isButton() || m_running);
        const bool readableValue = control.canRead();
        const bool copyableReadOnly = control.isReadOnly() && readableValue;
        const bool modified = m_desiredValues.value(id, control.currentValue) != control.defaultValue;
        if (control.restorable() && !autoDisabled && !isGrabbed(control) && modified)
            classHasModifiedReset.insert(control.className(), true);

        if (it->slider != nullptr)
            it->slider->setEnabled(writable);
        if (it->spinBox != nullptr) {
            it->spinBox->setEnabled(writable || copyableReadOnly);
            it->spinBox->setReadOnly(!writable);
        } else if (it->lineEdit != nullptr) {
            it->lineEdit->setEnabled(writable || copyableReadOnly);
            it->lineEdit->setReadOnly(!writable);
        } else if (qobject_cast<QLabel *>(it->editor) != nullptr) {
            it->editor->setEnabled(true);
        } else if (it->editor != nullptr) {
            it->editor->setEnabled(writable);
        }
        if (it->resetButton != nullptr)
            it->resetButton->setEnabled(control.restorable() && !autoDisabled && !isGrabbed(control) && modified);
        if (it->stateLabel != nullptr) {
            if (!unsupportedControlReason(control).isEmpty())
                it->stateLabel->setText(QStringLiteral("Unsupported"));
            else if (control.isReadOnly())
                it->stateLabel->setText(QStringLiteral("Read-only"));
            else if (autoDisabled)
                it->stateLabel->setText(QStringLiteral("Auto"));
            else if (isGrabbed(control))
                it->stateLabel->setText(QStringLiteral("Locked"));
            else if (control.isButton() && !m_running)
                it->stateLabel->setText(QStringLiteral("Live only"));
            else
                it->stateLabel->clear();
        }

        QString disabledReason;
        if (autoDisabled)
            disabledReason = autoDisabledReason(id, m_controls, m_desiredValues);
        else if (control.isButton() && !m_running)
            disabledReason = QStringLiteral("Button controls are only available while acquisition is running.");
        updateControlPresentation(id, disabledReason);
    }

    for (auto it = m_classResetButtons.begin(); it != m_classResetButtons.end(); ++it) {
        if (it.value() != nullptr)
            it.value()->setEnabled(classHasModifiedReset.value(it.key(), false));
    }
}

QVariantList V4L2SettingsDialog::serializableControlSettings() const
{
    QVariantList result;
    for (const auto &control : m_controls) {
        if (!control.restorable())
            continue;
        if (V4L2Camera::isManualDependentActive(control.id, m_desiredValues))
            continue;

        QVariantList menuEntries;
        for (const auto &entry : control.menu) {
            QVariantHash menuEntry;
            menuEntry.insert(QStringLiteral("value"), entry.value);
            menuEntry.insert(QStringLiteral("name"), entry.name);
            menuEntries.append(menuEntry);
        }

        QVariantHash item;
        item.insert(QStringLiteral("id"), static_cast<quint64>(control.id));
        item.insert(QStringLiteral("name"), control.name);
        item.insert(QStringLiteral("type"), static_cast<quint64>(control.type));
        item.insert(QStringLiteral("value"), m_desiredValues.value(control.id, control.currentValue));
        item.insert(QStringLiteral("default"), control.defaultValue);
        item.insert(QStringLiteral("minimum"), control.minimum);
        item.insert(QStringLiteral("maximum"), control.maximum);
        item.insert(QStringLiteral("step"), control.step);
        item.insert(QStringLiteral("menu"), menuEntries);
        result.append(item);
    }
    return result;
}
