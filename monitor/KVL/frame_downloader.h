/*
 *  frame_downloader.h
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

#ifndef FRAME_DOWNLOADER_H
#define FRAME_DOWNLOADER_H

#include <QObject>
#include <QThread>

#include "socket.hpp"

class frame_downloader : public QObject
{
    Q_OBJECT
    kybernetes::network::Socket socket;
    QString                     m_host;
    int                         m_port;
public:
    frame_downloader(std::string host, int port);
    ~frame_downloader();

    void modify_tracking_parameters(uchar y, uchar u, uchar v);
    void set_tracking_enabled(uchar e);
    void reset_ptz();

    bool                        mode;

public slots:
    void run();

signals:
    void frame_grabbed(unsigned int width, unsigned int height, unsigned int size, void *image_buffer);   
};

#endif // FRAME_DOWNLOADER_H
