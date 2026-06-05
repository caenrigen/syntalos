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

static bool isIntegerControl(const ControlInfo &control)
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

static qint64 snapControlValue(const ControlInfo &control, qint64 value)
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

} // namespace V4L2Camera
