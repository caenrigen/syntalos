/*
 * Copyright (C) 2026 Matthias Klumpp <matthias@tenstral.net>
 *
 * Licensed under the GNU Lesser General Public License Version 3
 */

#pragma once

#include "v4l2device.h"

#include <QDialog>
#include <QHash>
#include <QList>

class QCheckBox;
class QComboBox;
class QGridLayout;
class QLabel;
class QPushButton;
class QSlider;
class QSpinBox;
class QTabWidget;

class V4L2SettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit V4L2SettingsDialog(QWidget *parent = nullptr);

    V4L2Camera::DeviceIdentity selectedDevice() const;
    V4L2Camera::CaptureMode selectedMode() const;
    QList<V4L2Camera::ControlInfo> controls() const;
    QHash<quint32, qint64> desiredControlValues() const;

    void setRunning(bool running);
    void setEffectiveMode(const V4L2Camera::CaptureMode &mode);
    void updateControlReadback(quint32 id, qint64 value);
    void replaceControls(const QList<V4L2Camera::ControlInfo> &controls);
    void refreshDevices();

    void serializeSettings(QVariantHash &settings) const;
    void loadSettings(const QVariantHash &settings);

Q_SIGNALS:
    void controlValueChanged(quint32 id, qint64 value);

private Q_SLOTS:
    void onDeviceChanged(int index);
    void onModeChanged(int index);
    void onRefreshClicked();
    void onReadControlsClicked();
    void onResetControlsClicked();

private:
    struct ControlWidgets {
        QWidget *row = nullptr;
        QWidget *editor = nullptr;
        QSlider *slider = nullptr;
        QSpinBox *spinBox = nullptr;
        QComboBox *comboBox = nullptr;
        QCheckBox *checkBox = nullptr;
        QPushButton *button = nullptr;
        QPushButton *resetButton = nullptr;
        QLabel *stateLabel = nullptr;
    };

    QTabWidget *m_tabs;
    QWidget *m_captureTab;
    QComboBox *m_deviceCombo;
    QComboBox *m_modeCombo;
    QLabel *m_summaryLabel;
    QLabel *m_effectiveLabel;
    QPushButton *m_refreshButton;
    QPushButton *m_readControlsButton;
    QPushButton *m_resetControlsButton;

    bool m_running;
    bool m_blockUiSignals;
    V4L2Camera::DeviceIdentity m_loadedIdentity;
    V4L2Camera::CaptureMode m_loadedMode;
    QHash<quint32, qint64> m_loadedControlValues;
    QHash<quint32, V4L2Camera::ControlInfo> m_controls;
    QHash<quint32, qint64> m_desiredValues;
    QHash<quint32, ControlWidgets> m_controlWidgets;

    void buildUi();
    void clearControlTabs();
    void populateModes(const QList<V4L2Camera::CaptureMode> &modes);
    void rebuildControls(const QList<V4L2Camera::ControlInfo> &controls);
    QWidget *createControlRow(const V4L2Camera::ControlInfo &control);
    void setControlWidgetValue(quint32 id, qint64 value);
    void handleControlEdited(quint32 id, qint64 value);
    void updateSummary();
    void updateDependencyStates();
    QVariantList serializableControlSettings() const;
};
