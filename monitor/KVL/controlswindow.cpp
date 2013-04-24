/*
 *  controlswindow.cpp
 *
 *  Copyright (c) 2013 Nathaniel Lewis, Robotics Society at UC Merced
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "controlswindow.h"
#include "ui_controlswindow.h"

ControlsWindow::ControlsWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::ControlsWindow)
{
    ui->setupUi(this);
    connect(ui->captureButton, SIGNAL(clicked()), this, SLOT(captureButtonPressed()));
    connect(ui->showResult, SIGNAL(stateChanged(int)), this, SLOT(resultCheckboxChanged(int)));
    connect(ui->resetCamera, SIGNAL(clicked()), this, SLOT(ptzResetPressed()));
    connect(ui->shouldTrack, SIGNAL(stateChanged(int)), this, SLOT(trackingCheckboxChanged(int)));
}

ControlsWindow::~ControlsWindow()
{
    delete ui;
}

void ControlsWindow::captureButtonPressed()
{
    emit capture_frame();
}

void ControlsWindow::resultCheckboxChanged(int i)
{
    emit result_option_changed(i);
}

void ControlsWindow::ptzResetPressed()
{
    emit reset_ptz();
}

void ControlsWindow::trackingCheckboxChanged(int i)
{
    emit tracking_option_changed(i);
}
