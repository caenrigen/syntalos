/*
 * Copyright (C) 2026 Matthias Klumpp <matthias@tenstral.net>
 *
 * Licensed under the GNU Lesser General Public License Version 3
 */

#pragma once

#include "v4l2device.h"

#include <QHash>
#include <QList>
#include <QSet>

namespace V4L2Camera
{

int clampManualReapplyDelayMs(int value);
bool fitsInt(qint64 value);
int intStep(qint64 step);
bool isIntegerControl(const ControlInfo &control);
bool isEditableNumericControl(const ControlInfo &control);
bool isGrabbed(const ControlInfo &control);
bool hasPayload(const ControlInfo &control);
qint64 snapControlValue(const ControlInfo &control, qint64 value);
qint64 clampControlValue(const ControlInfo &control, qint64 value);
bool shouldUseSlider(const ControlInfo &control);
int dependencySortKey(quint32 id);
bool autoDisabled(quint32 id, const QHash<quint32, qint64> &values);
QHash<quint32, ControlInfo> controlsById(const QList<ControlInfo> &controls);
bool controlInventoryChanged(
    const QHash<quint32, ControlInfo> &oldControls,
    const QList<ControlInfo> &newControls);
QList<quint32> sortedControlWriteIds(const QHash<quint32, qint64> &pending);
QList<ControlInfo> sortedControlsForRestore(const QList<ControlInfo> &controls);

} // namespace V4L2Camera
