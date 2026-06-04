/*
 * Copyright (C) 2026 Matthias Klumpp <matthias@tenstral.net>
 *
 * Licensed under the GNU Lesser General Public License Version 3
 */

#include "v4l2controlapplier.h"

#include "v4l2controlpolicy.h"

#include <algorithm>
#include <thread>

namespace V4L2Camera
{

static ControlApplyWarning readbackMismatchWarning(const ControlInfo &control, const ControlWriteResult &result)
{
    return {
        ControlApplyWarningKind::ReadbackMismatch,
        QStringLiteral("V4L2 Control Readback"),
        QStringLiteral("Control '%1' read back as %2 after requesting %3.")
            .arg(control.name)
            .arg(result.readbackValue)
            .arg(result.requestedValue),
    };
}

static ControlApplyWarning exposureAutoPriorityWarning()
{
    return {
        ControlApplyWarningKind::ExposureAutoPriority,
        QStringLiteral("Variable FPS Control"),
        QStringLiteral(
            "V4L2 exposure auto priority is enabled. This can allow variable frame timing; actual FPS will be read back."),
    };
}

void ControlApplier::clear()
{
    m_controlMap.clear();
    m_scheduledManualReapplies.clear();
}

void ControlApplier::clearScheduledManualReapplies()
{
    m_scheduledManualReapplies.clear();
}

void ControlApplier::updateControlMap(const QList<ControlInfo> &controls)
{
    m_controlMap = controlsById(controls);
}

const QHash<quint32, ControlInfo> &ControlApplier::controlMap() const
{
    return m_controlMap;
}

int ControlApplier::controlPollTimeoutMs() const
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

bool ControlApplier::hasDueManualReapplies() const
{
    return std::any_of(
        m_scheduledManualReapplies.constBegin(),
        m_scheduledManualReapplies.constEnd(),
        [](const auto &pending) { return pending.dueTime <= std::chrono::steady_clock::now(); });
}

ControlApplyReport ControlApplier::applyDesiredControls(
    Device &device,
    const QList<ControlInfo> &controls,
    const ControlRestoreRequest &request)
{
    ControlApplyReport report;
    report.didWork = true;
    auto desired = request.desiredValues;

    if (desired.contains(V4L2_CID_EXPOSURE_AUTO_PRIORITY) && desired.value(V4L2_CID_EXPOSURE_AUTO_PRIORITY) != 0)
        report.userWarnings.append(exposureAutoPriorityWarning());

    const auto sortedControls = sortedControlsForRestore(controls);
    updateControlMap(sortedControls);
    const auto dependencyTable = autoDependencyTable();

    for (const auto &listedControl : sortedControls) {
        const auto control = m_controlMap.value(listedControl.id, listedControl);
        if (!desired.contains(control.id))
            continue;
        if (!control.restorable())
            continue;
        if (isManualDependentActive(control.id, desired))
            continue;

        const auto requestedValue = desired.value(control.id);
        if (request.forceFocusAutoCycleOnRestore && control.id == V4L2_CID_FOCUS_AUTO
            && !autoControlEnabled(control.id, requestedValue)) {
            const auto enableResult = device.setControlValue(control, 1);
            if (!enableResult.success) {
                report.logWarnings.append(
                    QStringLiteral("Failed to cycle Focus Auto before restoring manual focus mode: %1")
                        .arg(enableResult.error));
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        const auto result = device.setControlValue(control, requestedValue);
        if (!result.success) {
            report.logWarnings.append(result.error);
            continue;
        }

        desired[control.id] = result.readbackValue;
        if (result.changedByDevice)
            report.userWarnings.append(readbackMismatchWarning(control, result));

        if (dependencyTable.contains(control.id) && !autoControlEnabled(control.id, result.readbackValue)) {
            const auto delayMs = request.manualReapplyDelaysMs.value(control.id, 0);
            if (delayMs == 0) {
                reapplyManualDependentControls(
                    device,
                    control,
                    dependencyTable,
                    &desired,
                    nullptr,
                    ManualDependentReapplySource::DesiredValue,
                    &report);
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
                    ManualDependentReapplySource::DesiredValue,
                    &report);
            }
        }
    }

    report.desiredValues = desired;
    return report;
}

void ControlApplier::scheduleManualDependentReapply(quint32 autoControlId, int delayMs)
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

void ControlApplier::reapplyManualDependentControls(
    Device &device,
    const ControlInfo &autoControl,
    const QHash<quint32, QList<quint32>> &dependencyTable,
    QHash<quint32, qint64> *desired,
    QSet<quint32> *affectedRefreshIds,
    ManualDependentReapplySource valueSource,
    ControlApplyReport *report)
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

        // Logitech Webcam C930e reports a manual value after auto mode is disabled
        // but keeps using a different physical state until a manual value is written.
        // During startup restore, keep saved desired values authoritative instead
        // of replacing them with the driver's transient post-auto readback.
        const bool useDesiredValue = valueSource == ManualDependentReapplySource::DesiredValue && desired != nullptr
            && desired->contains(dependent.id);
        const auto writeValue = useDesiredValue ? desired->value(dependent.id) : dependent.currentValue;
        const auto result = device.setControlValue(dependent, writeValue);
        if (!result.success) {
            if (report != nullptr)
                report->logWarnings.append(result.error);
            continue;
        }

        if (desired != nullptr)
            (*desired)[dependent.id] = result.readbackValue;
        if (m_controlMap.contains(dependent.id))
            m_controlMap[dependent.id].currentValue = result.readbackValue;
    }
}

void ControlApplier::applyDueManualReapplies(
    Device &device,
    const QHash<quint32, QList<quint32>> &dependencyTable,
    const QHash<quint32, int> &manualReapplyDelaysMs,
    QHash<quint32, qint64> *desired,
    QSet<quint32> *affectedRefreshIds,
    ControlApplyReport *report)
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
        if (autoControlEnabled(autoControl.id, desired->value(autoControl.id, autoControl.currentValue)))
            continue;

        reapplyManualDependentControls(
            device,
            autoControl,
            dependencyTable,
            desired,
            affectedRefreshIds,
            ManualDependentReapplySource::ReportedValue,
            report);
    }
}

ControlApplyReport ControlApplier::applyPendingControlWrites(Device &device, const PendingControlRequest &request)
{
    ControlApplyReport report;
    auto desired = request.desiredValues;

    if (request.pendingWrites.isEmpty() && request.pendingButtons.isEmpty() && !hasDueManualReapplies()) {
        report.desiredValues = desired;
        return report;
    }
    report.didWork = true;

    QSet<quint32> affectedRefreshIds;
    const auto dependencyTable = autoDependencyTable();
    for (const auto id : sortedControlWriteIds(request.pendingWrites)) {
        if (!m_controlMap.contains(id))
            continue;

        const auto control = m_controlMap.value(id);
        if (isManualDependentActive(control.id, desired))
            continue;

        const auto result = device.setControlValue(control, request.pendingWrites.value(id));
        if (!result.success) {
            report.logWarnings.append(result.error);
            continue;
        }

        desired[control.id] = result.readbackValue;
        m_controlMap[control.id].currentValue = result.readbackValue;
        report.readbackUpdates.append({control.id, result.readbackValue});

        if (result.changedByDevice)
            report.userWarnings.append(readbackMismatchWarning(control, result));

        affectedRefreshIds.insert(control.id);
        const auto dependentIds = dependencyTable.value(control.id);
        for (const auto dependentId : dependentIds) {
            if (m_controlMap.contains(dependentId))
                affectedRefreshIds.insert(dependentId);
        }
        if (!dependentIds.isEmpty()) {
            scheduleManualDependentReapply(control.id, 0);
            if (!autoControlEnabled(control.id, result.readbackValue)) {
                const auto delayMs = request.manualReapplyDelaysMs.value(control.id, 0);
                if (delayMs == 0)
                    reapplyManualDependentControls(
                        device,
                        control,
                        dependencyTable,
                        &desired,
                        &affectedRefreshIds,
                        ManualDependentReapplySource::ReportedValue,
                        &report);
                else if (delayMs > 0)
                    scheduleManualDependentReapply(control.id, delayMs);
            }
        }
    }

    for (const auto id : request.pendingButtons) {
        if (!m_controlMap.contains(id))
            continue;

        const auto control = m_controlMap.value(id);
        QString error;
        if (!device.triggerButtonControl(control, &error))
            report.logWarnings.append(error);
    }

    applyDueManualReapplies(device, dependencyTable, request.manualReapplyDelaysMs, &desired, &affectedRefreshIds, &report);

    if (!affectedRefreshIds.isEmpty()) {
        auto controls = device.queryControls(nullptr);
        report.inventoryChanged = controlInventoryChanged(m_controlMap, controls);
        updateControlMap(controls);
        if (report.inventoryChanged) {
            for (const auto &control : controls) {
                if (!control.isClassMarker())
                    desired[control.id] = control.currentValue;
            }
        } else {
            for (const auto &control : controls) {
                if (affectedRefreshIds.contains(control.id))
                    desired[control.id] = control.currentValue;
            }
        }
        report.refreshedControls = controls;
        report.affectedRefreshIds = affectedRefreshIds;
    }

    report.desiredValues = desired;
    return report;
}

} // namespace V4L2Camera
