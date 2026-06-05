/*
 * Copyright (C) 2026 Matthias Klumpp <matthias@tenstral.net>
 *
 * Licensed under the GNU Lesser General Public License Version 3
 */

#pragma once

#include "v4l2device.h"

namespace V4L2Camera
{

int clampManualReapplyDelayMs(int value);
bool fitsInt(qint64 value);
int intStep(qint64 step);
bool isEditableNumericControl(const ControlInfo &control);
bool isGrabbed(const ControlInfo &control);
bool hasPayload(const ControlInfo &control);
qint64 clampControlValue(const ControlInfo &control, qint64 value);
bool shouldUseSlider(const ControlInfo &control);
int dependencySortKey(quint32 id);

} // namespace V4L2Camera
