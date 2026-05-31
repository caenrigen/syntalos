/*
 * Copyright (C) 2026 Matthias Klumpp <matthias@tenstral.net>
 *
 * Licensed under the GNU Lesser General Public License Version 3
 */

#include "v4l2settingsdialog.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDebug>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QTabWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <limits>

namespace
{

bool fitsInt(qint64 value)
{
    return value >= std::numeric_limits<int>::min() && value <= std::numeric_limits<int>::max();
}

qint64 clampControlValue(const V4L2Camera::ControlInfo &control, qint64 value)
{
    if (control.type == V4L2_CTRL_TYPE_BUTTON)
        return value;
    if (value < control.minimum)
        return control.minimum;
    if (value > control.maximum)
        return control.maximum;
    return value;
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

QString controlTooltip(const V4L2Camera::ControlInfo &control)
{
    return QStringLiteral("0x%1, %2").arg(control.id, 0, 16).arg(V4L2Camera::controlTypeName(control.type));
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
      m_readControlsButton(nullptr),
      m_resetControlsButton(nullptr),
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
        m_summaryLabel->setText(enumError.isEmpty() ? QStringLiteral("No V4L2 capture devices found.") : enumError);
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
    } else if (previous.isValid()) {
        match = V4L2Camera::matchDevice(previous, devices, enumError);
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

void V4L2SettingsDialog::onReadControlsClicked()
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
    rebuildControls(queriedControls);
}

void V4L2SettingsDialog::onResetControlsClicked()
{
    const auto values = m_desiredValues;
    for (auto it = m_controls.begin(); it != m_controls.end(); ++it) {
        const auto &control = it.value();
        if (!control.restorable())
            continue;
        if (V4L2Camera::isManualDependentActive(control.id, values))
            continue;
        handleControlEdited(control.id, control.defaultValue);
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

    auto *buttonsLayout = new QHBoxLayout;
    buttonsLayout->addStretch();
    m_readControlsButton = new QPushButton(QStringLiteral("Read Current"), this);
    m_resetControlsButton = new QPushButton(QStringLiteral("Reset Controls"), this);
    buttonsLayout->addWidget(m_readControlsButton);
    buttonsLayout->addWidget(m_resetControlsButton);
    mainLayout->addLayout(buttonsLayout);

    connect(m_deviceCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &V4L2SettingsDialog::onDeviceChanged);
    connect(m_modeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &V4L2SettingsDialog::onModeChanged);
    connect(m_refreshButton, &QPushButton::clicked, this, &V4L2SettingsDialog::onRefreshClicked);
    connect(m_readControlsButton, &QPushButton::clicked, this, &V4L2SettingsDialog::onReadControlsClicked);
    connect(m_resetControlsButton, &QPushButton::clicked, this, &V4L2SettingsDialog::onResetControlsClicked);
}

void V4L2SettingsDialog::clearControlTabs()
{
    m_controlWidgets.clear();
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
        selectedIndex = 0;
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

    auto *nameLabel = new QLabel(control.name, row);
    nameLabel->setToolTip(controlTooltip(control));
    grid->addWidget(nameLabel, 0, 0);

    ControlWidgets widgets;
    widgets.row = row;
    widgets.stateLabel = new QLabel(row);
    widgets.stateLabel->setMinimumWidth(70);

    if (control.type == V4L2_CTRL_TYPE_BOOLEAN) {
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
            comboBox->addItem(entry.name, entry.value);
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
    } else if (fitsInt(control.minimum) && fitsInt(control.maximum) && fitsInt(control.currentValue)) {
        auto *editor = new QWidget(row);
        auto *layout = new QHBoxLayout(editor);
        layout->setContentsMargins(0, 0, 0, 0);

        auto *slider = new QSlider(Qt::Horizontal, editor);
        slider->setRange(static_cast<int>(control.minimum), static_cast<int>(control.maximum));
        slider->setSingleStep(static_cast<int>(std::max<qint64>(1, control.step)));
        auto *spinBox = new QSpinBox(editor);
        spinBox->setRange(static_cast<int>(control.minimum), static_cast<int>(control.maximum));
        spinBox->setSingleStep(static_cast<int>(std::max<qint64>(1, control.step)));

        layout->addWidget(slider, 1);
        layout->addWidget(spinBox);
        widgets.editor = editor;
        widgets.slider = slider;
        widgets.spinBox = spinBox;

        connect(slider, &QSlider::valueChanged, this, [this, id = control.id, spinBox](int value) {
            if (m_blockUiSignals)
                return;
            QSignalBlocker blocker(spinBox);
            spinBox->setValue(value);
            handleControlEdited(id, value);
        });
        connect(spinBox, qOverload<int>(&QSpinBox::valueChanged), this, [this, id = control.id, slider](int value) {
            if (m_blockUiSignals)
                return;
            QSignalBlocker blocker(slider);
            slider->setValue(value);
            handleControlEdited(id, value);
        });
        grid->addWidget(editor, 0, 1);
    } else {
        auto *label = new QLabel(QString::number(control.currentValue), row);
        widgets.editor = label;
        grid->addWidget(label, 0, 1);
    }

    widgets.resetButton = new QPushButton(QStringLiteral("Default"), row);
    connect(widgets.resetButton, &QPushButton::clicked, this, [this, id = control.id, defaultValue = control.defaultValue]() {
        handleControlEdited(id, defaultValue);
    });

    grid->addWidget(widgets.stateLabel, 0, 2);
    grid->addWidget(widgets.resetButton, 0, 3);
    m_controlWidgets.insert(control.id, widgets);
    setControlWidgetValue(control.id, control.currentValue);
    return row;
}

void V4L2SettingsDialog::setControlWidgetValue(quint32 id, qint64 value)
{
    if (!m_controlWidgets.contains(id))
        return;

    const bool oldBlockState = m_blockUiSignals;
    m_blockUiSignals = true;
    auto &widgets = m_controlWidgets[id];
    if (widgets.checkBox != nullptr) {
        widgets.checkBox->setChecked(value != 0);
    } else if (widgets.comboBox != nullptr) {
        for (int i = 0; i < widgets.comboBox->count(); ++i) {
            if (widgets.comboBox->itemData(i).toLongLong() == value) {
                widgets.comboBox->setCurrentIndex(i);
                break;
            }
        }
    } else if (widgets.slider != nullptr && widgets.spinBox != nullptr && fitsInt(value)) {
        widgets.slider->setValue(static_cast<int>(value));
        widgets.spinBox->setValue(static_cast<int>(value));
    } else if (auto *label = qobject_cast<QLabel *>(widgets.editor)) {
        label->setText(QString::number(value));
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
    for (auto it = m_controlWidgets.begin(); it != m_controlWidgets.end(); ++it) {
        const auto id = it.key();
        if (!m_controls.contains(id))
            continue;

        const auto &control = m_controls[id];
        const bool autoDisabled = V4L2Camera::isManualDependentActive(id, m_desiredValues);
        const bool writable = control.canWrite() && !autoDisabled && (!control.isButton() || m_running);

        if (it->editor != nullptr)
            it->editor->setEnabled(writable);
        if (it->resetButton != nullptr)
            it->resetButton->setEnabled(control.restorable() && !autoDisabled);
        if (it->stateLabel != nullptr) {
            if (control.isReadOnly())
                it->stateLabel->setText(QStringLiteral("Read-only"));
            else if (control.isInactive())
                it->stateLabel->setText(QStringLiteral("Inactive"));
            else if (autoDisabled)
                it->stateLabel->setText(QStringLiteral("Auto"));
            else if (control.isButton() && !m_running)
                it->stateLabel->setText(QStringLiteral("Live only"));
            else
                it->stateLabel->clear();
        }
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
