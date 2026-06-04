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
#include <QStringList>

#include <chrono>

namespace V4L2Camera
{

enum class ControlApplyWarningKind {
    ReadbackMismatch,
    ExposureAutoPriority
};

struct ControlApplyWarning {
    ControlApplyWarningKind kind = ControlApplyWarningKind::ReadbackMismatch;
    QString title;
    QString message;
};

struct ControlReadbackUpdate {
    quint32 id = 0;
    qint64 value = 0;
};

struct ControlApplyReport {
    bool didWork = false;
    bool inventoryChanged = false;
    QHash<quint32, qint64> desiredValues;
    QStringList logWarnings;
    QList<ControlApplyWarning> userWarnings;
    QList<ControlReadbackUpdate> readbackUpdates;
    QList<ControlInfo> refreshedControls;
    QSet<quint32> affectedRefreshIds;
};

struct ControlRestoreRequest {
    QHash<quint32, qint64> desiredValues;
    QHash<quint32, int> manualReapplyDelaysMs;
    bool forceFocusAutoCycleOnRestore = false;
};

struct PendingControlRequest {
    QHash<quint32, qint64> pendingWrites;
    QList<quint32> pendingButtons;
    QHash<quint32, qint64> desiredValues;
    QHash<quint32, int> manualReapplyDelaysMs;
};

class ControlApplier
{
public:
    void clearScheduledManualReapplies();
    void updateControlMap(const QList<ControlInfo> &controls);

    int controlPollTimeoutMs() const;
    ControlApplyReport applyDesiredControls(
        Device &device,
        const QList<ControlInfo> &controls,
        const ControlRestoreRequest &request);
    ControlApplyReport applyPendingControlWrites(Device &device, const PendingControlRequest &request);

private:
    enum class ManualDependentReapplySource {
        ReportedValue,
        DesiredValue,
    };

    struct ScheduledManualReapply {
        quint32 autoControlId = 0;
        std::chrono::steady_clock::time_point dueTime;
    };

    QHash<quint32, ControlInfo> m_controlMap;
    QList<ScheduledManualReapply> m_scheduledManualReapplies;

    bool hasDueManualReapplies() const;
    void scheduleManualDependentReapply(quint32 autoControlId, int delayMs);
    void reapplyManualDependentControls(
        Device &device,
        const ControlInfo &autoControl,
        const QHash<quint32, QList<quint32>> &dependencyTable,
        QHash<quint32, qint64> &desired,
        QSet<quint32> *affectedRefreshIds,
        ManualDependentReapplySource valueSource,
        ControlApplyReport &report);
    void applyDueManualReapplies(
        Device &device,
        const QHash<quint32, QList<quint32>> &dependencyTable,
        const QHash<quint32, int> &manualReapplyDelaysMs,
        QHash<quint32, qint64> &desired,
        QSet<quint32> *affectedRefreshIds,
        ControlApplyReport &report);
};

} // namespace V4L2Camera
