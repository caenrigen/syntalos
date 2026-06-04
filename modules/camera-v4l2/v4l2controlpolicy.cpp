/*
 * Copyright (C) 2026 Matthias Klumpp <matthias@tenstral.net>
 *
 * Licensed under the GNU Lesser General Public License Version 3
 */

#include "v4l2controlpolicy.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace V4L2Camera
{

int clampManualReapplyDelayMs(int value)
{
    return std::clamp(value, -1, 9999);
}

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

bool isIntegerControl(const ControlInfo &control)
{
    return control.type == V4L2_CTRL_TYPE_INTEGER || control.type == V4L2_CTRL_TYPE_INTEGER64;
}

bool isEditableNumericControl(const ControlInfo &control)
{
    return isIntegerControl(control) || control.type == V4L2_CTRL_TYPE_BITMASK;
}

bool isGrabbed(const ControlInfo &control)
{
    return (control.flags & V4L2_CTRL_FLAG_GRABBED) != 0;
}

bool hasPayload(const ControlInfo &control)
{
#ifdef V4L2_CTRL_FLAG_HAS_PAYLOAD
    return (control.flags & V4L2_CTRL_FLAG_HAS_PAYLOAD) != 0;
#else
    Q_UNUSED(control)
    return false;
#endif
}

qint64 snapControlValue(const ControlInfo &control, qint64 value)
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

qint64 clampControlValue(const ControlInfo &control, qint64 value)
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

bool shouldUseSlider(const ControlInfo &control)
{
    if (control.type != V4L2_CTRL_TYPE_INTEGER)
        return false;
    if (!fitsInt(control.minimum) || !fitsInt(control.maximum))
        return false;
    const auto range = control.maximum - control.minimum;
    return (control.flags & V4L2_CTRL_FLAG_SLIDER) != 0 || range <= 10000;
}

int dependencySortKey(quint32 id)
{
    int groupIndex = 0;
    for (const auto &group : autoDependencyGroups()) {
        if (id == group.autoControlId)
            return groupIndex * 10;
        if (group.manualControlIds.contains(id))
            return groupIndex * 10 + 1;
        ++groupIndex;
    }
    return 1000;
}

bool autoDisabled(quint32 id, const QHash<quint32, qint64> &values)
{
    return isManualDependentActive(id, values);
}

static bool sameMenuStructure(const QList<MenuEntry> &a, const QList<MenuEntry> &b)
{
    if (a.size() != b.size())
        return false;

    for (int i = 0; i < a.size(); ++i) {
        if (a.at(i).value != b.at(i).value || a.at(i).name != b.at(i).name)
            return false;
    }

    return true;
}

static bool sameControlStructure(const ControlInfo &a, const ControlInfo &b)
{
    return a.id == b.id && a.name == b.name && a.controlClass == b.controlClass && a.type == b.type
        && a.minimum == b.minimum && a.maximum == b.maximum && a.step == b.step
        && a.defaultValue == b.defaultValue && a.supported == b.supported && a.isDisabled() == b.isDisabled()
        && sameMenuStructure(a.menu, b.menu);
}

QHash<quint32, ControlInfo> controlsById(const QList<ControlInfo> &controls)
{
    QHash<quint32, ControlInfo> result;
    for (const auto &control : controls) {
        if (!control.isClassMarker())
            result.insert(control.id, control);
    }
    return result;
}

bool controlInventoryChanged(const QHash<quint32, ControlInfo> &oldControls, const QList<ControlInfo> &newControls)
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

QList<quint32> sortedControlWriteIds(const QHash<quint32, qint64> &pending)
{
    auto ids = pending.keys();
    const auto autoIds = autoControlIds();
    std::sort(ids.begin(), ids.end(), [&autoIds](quint32 a, quint32 b) {
        const auto aPriority = autoIds.contains(a) || a == V4L2_CID_EXPOSURE_AUTO_PRIORITY ? 0 : 1;
        const auto bPriority = autoIds.contains(b) || b == V4L2_CID_EXPOSURE_AUTO_PRIORITY ? 0 : 1;
        if (aPriority != bPriority)
            return aPriority < bPriority;
        return a < b;
    });
    return ids;
}

QList<ControlInfo> sortedControlsForRestore(const QList<ControlInfo> &controls)
{
    auto sorted = controls;
    const auto autoIds = autoControlIds();
    std::sort(sorted.begin(), sorted.end(), [&autoIds](const auto &a, const auto &b) {
        const auto aPriority = autoIds.contains(a.id) || a.id == V4L2_CID_EXPOSURE_AUTO_PRIORITY ? 0 : 1;
        const auto bPriority = autoIds.contains(b.id) || b.id == V4L2_CID_EXPOSURE_AUTO_PRIORITY ? 0 : 1;
        if (aPriority != bPriority)
            return aPriority < bPriority;
        return a.id < b.id;
    });
    return sorted;
}

} // namespace V4L2Camera
