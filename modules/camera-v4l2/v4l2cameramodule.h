/*
 * Copyright (C) 2026 Matthias Klumpp <matthias@tenstral.net>
 *
 * Licensed under the GNU Lesser General Public License Version 3
 */

#pragma once

#include "moduleapi.h"
#include <QObject>

SYNTALOS_DECLARE_MODULE

class V4L2CameraModuleInfo : public ModuleInfo
{
public:
    QString id() const final;
    QString name() const final;
    QString summary() const final;
    QString description() const final;
    QString authors() const final;
    QString license() const final;
    ModuleCategories categories() const final;
    QColor color() const final;
    AbstractModule *createModule(QObject *parent = nullptr) final;
};
