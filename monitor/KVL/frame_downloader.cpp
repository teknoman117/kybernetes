/*
 *  frame_downloader.cpp
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

#include <iostream>
#include "frame_downloader.h"

#define Y_PADDING 20
#define U_PADDING 20
#define V_PADDING 20

frame_downloader::frame_downloader(std::string host, int port)
    : QObject(), m_host(host.c_str()), m_port(port), mode(false)
{
    // Connect to the frame server
    if(!socket.connectTo(host, port))
    {
        std::cerr << " << Error connecting to " << host << ":" << port << std::endl;
        return;
    }
    std::cout << " >> Connected to " << host << ":" << port << std::endl;
}

frame_downloader::~frame_downloader()
{
    std::cout << " >> Disconnected" << std::endl;
    socket.disconnect();
}

void frame_downloader::run()
{
    // Variables to use
    unsigned int len = 0;
    unsigned int width = 0;
    unsigned int height = 0;
    char         command[2];

    // Image buffer
    void        *image_buffer;

    // Send image grab command
    //qDebug("Beginning Capture");
    command[0] = mode ? 'r' : 'i';
    command[1] = 0;
    if(!socket.write((void *) command, 1))
    {
        std::cerr << "Unable to send command" << std::endl;
        return;
    }
    qDebug("Sent Command");

    // Fetch the width of the image
    if(!socket.read((void *) &width, 4))
    {
        std::cerr << "Unable fetch width" << std::endl;
        return;
    }
    qDebug("Fetched width");

    // Fetch the height of the image
    if(!socket.read((void *) &height, 4))
    {
        std::cerr << "Unable fetch height" << std::endl;
        return;
    }
    qDebug("Fetched height");

    // Fetch the data length of the image
    if(!socket.read((void *) &len, 4))
    {
        std::cerr << "Unable fetch size" << std::endl;
        return;
    }
    qDebug("Fetched Length");

    // Fetch the data of the image
    image_buffer = malloc(width * height * 2);
    if(!socket.read(image_buffer, len))
    {
        std::cerr << "Unable fetch image data" << std::endl;
        return;
    }
    qDebug("Fetched Data");

    // Print frame information
    //std::cout << "Frame: Dimensions = {" << width << ", " << height << "}, Size = " << len << std::endl;

    // Emit event
    //qDebug("Captured");
    emit frame_grabbed(width, height, len, image_buffer);
    thread()->terminate();
}

void frame_downloader::modify_tracking_parameters(uchar y, uchar u, uchar v)
{
    // Create the padded values
    uchar y_max = (y < 245) ? y + Y_PADDING : 255;
    uchar y_min = (y > 10) ? y - Y_PADDING : 0;
    uchar u_max = (u < 245) ? u + U_PADDING : 255;
    uchar u_min = (u > 10) ? u - U_PADDING : 0;
    uchar v_max = (v < 245) ? v + V_PADDING : 255;
    uchar v_min = (v > 10) ? v - V_PADDING : 0;

    // Upload
    qDebug("beginning upload");
    unsigned char command[2];
    command[0] = 'u';
    command[1] = 0;
    if(!socket.write((void *) command, 1))
    {
        std::cerr << "Unable to send command" << std::endl;
        return;
    }

    // Upload colors
    if(!socket.write((void *) &y_max, 1))
    {
        std::cerr << "Unable to write parameter" << std::endl;
        return;
    }

    if(!socket.write((void *) &y_min, 1))
    {
        std::cerr << "Unable to write parameter" << std::endl;
        return;
    }
    if(!socket.write((void *) &u_max, 1))
    {
        std::cerr << "Unable to write parameter" << std::endl;
        return;
    }

    if(!socket.write((void *) &u_min, 1))
    {
        std::cerr << "Unable to write parameter" << std::endl;
        return;
    }
    if(!socket.write((void *) &v_max, 1))
    {
        std::cerr << "Unable to write parameter" << std::endl;
        return;
    }

    if(!socket.write((void *) &v_min, 1))
    {
        std::cerr << "Unable to write parameter" << std::endl;
        return;
    }
    qDebug("Finished uploading parameters");
}

void frame_downloader::set_tracking_enabled(uchar e)
{
    // Variables to use
    char         command[3];

    // Send image grab command
    command[0] = 't';
    command[1] = e;
    command[2] = 0;
    if(!socket.write((void *) command, 2))
    {
        std::cerr << "Unable to send command" << std::endl;
        return;
    }
    qDebug("Tracking Mode Changed");
}

void frame_downloader::reset_ptz()
{
    // Variables to use
    char         command[2];

    // Send image grab command
    command[0] = 'e';
    command[1] = 0;
    if(!socket.write((void *) command, 1))
    {
        std::cerr << "Unable to send command" << std::endl;
        return;
    }
    qDebug("PTZ Reset");
}
