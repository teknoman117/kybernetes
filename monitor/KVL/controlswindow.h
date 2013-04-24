/*
 *  controlswindow.h
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

#ifndef CONTROLSWINDOW_H
#define CONTROLSWINDOW_H

#include <QMainWindow>

namespace Ui {
class ControlsWindow;
}

class ControlsWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit ControlsWindow(QWidget *parent = 0);
    ~ControlsWindow();

private:
    Ui::ControlsWindow *ui;

signals:
    void capture_frame();
    void result_option_changed(int);
    void reset_ptz();
    void tracking_option_changed(int);

public slots:
    void captureButtonPressed();
    void resultCheckboxChanged(int);
    void ptzResetPressed();
    void trackingCheckboxChanged(int);

};

#endif // CONTROLSWINDOW_H
