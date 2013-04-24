/*
 *  mainwindow.h
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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QMutex>

#include "frame_downloader.h"
#include "controlswindow.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

    // thread object that runs the frame download on another thread
    QThread          *downloaderThread;

    // Download manager object
    frame_downloader *downloader;

    // Controls window
    ControlsWindow   *window;

    // Image Pixmap (so we can draw on it later)
    QPixmap           pixmap;
    QMutex            pixmapMutex;

    // Points important to calculating the mean of a pixel range
    QPoint            press;
    QPoint            release;

    // Flags
    bool              drawing;
    bool              valid;
    bool              mode; //true = resultant, false = color
    bool              n;

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void mousePressEvent(QMouseEvent *);
    void mouseReleaseEvent(QMouseEvent *);
    void mouseMoveEvent(QMouseEvent *);

    void updateImage(void);
private:
    Ui::MainWindow *ui;

public slots:
    void receive_frame(unsigned int width, unsigned int height, unsigned int size, void *image_buffer);
    void begin_capture();
    void changeMode(int checkboxState);
    void changeTrackingMode(int checkboxState);
    void resetPtz();
};

#endif // MAINWINDOW_H
