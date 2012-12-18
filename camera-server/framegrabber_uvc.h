//==========================================================================
//
//  Project:        VRS - Vision Recogition System for Linux
//
//  Module:         Camera capture library
//
//  Description:    Captures images from video devices
//                    Supports the V4L2 UVC interface
//                    Supports the V4L2 interface
//
//  Authors:        Nathaniel Lewis <linux.robotdude@gmail.com>
//
//  Homepage:       http://groups.google.com/group/the-linbot-project
//
//--------------------------------------------------------------------------
//
//  VRS - Vision Recognition System for Linux
//  Copyright (c) 2010 Nathaniel Lewis
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307  USA
//  or obtain a copy from the GNU website at http://www.gnu.org/
//
//==========================================================================

#ifndef _FrameGrabber_UVC_
#define _FrameGrabber_UVC_

#define NB_BUFFER 16
#define NORMB_BUFFER 4
#define DHT_SIZE 420
#define HEADERFRAME1 0xaf

#include <vector>
#include <string>
#include <stdint.h>

#include <linux/videodev2.h>

namespace uvc_camera {
    class FrameGrabber_UVC;
};

class uvc_camera::FrameGrabber_UVC {
public:
    // Creation and destruction of the framegrabber
    FrameGrabber_UVC( std::string device, int _width, int _height, int format );
    ~FrameGrabber_UVC();
    void stop();

    // Grabbing frames
    int grab_compressed(std::vector<unsigned char>& data);
    int grab_dummy();
    //int grab(sensor_msgs::Image           &image);
    
    // Grabs an image of the correct format
    int grab(std::vector<unsigned char>& data);

    // Camera controls
    int brightness( int nbrightness );
    int saturation( int nsaturation );
    int contrast( int ncontrast );
    int gain( int ngain );
    
    // Pan, tilt, zoom controls
    int resetPTZ( void );
    int movePTZ(int pan, int tilt, int zoom);

private:
    // Variables required by framegrabber
    int cam;
    std::string videodevice;

    // V4L2 Structures
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    struct v4l2_buffer buf;
    struct v4l2_requestbuffers rb;

    // Temporary buffers
    void *mem[NB_BUFFER];
    std::vector<uint8_t> tmpbuffer;

    // Holds status and settings
    unsigned int isstreaming;
    unsigned int width;
    unsigned int height;
    unsigned int formatIn;
    unsigned int framesizeIn;

    // Background, supportive functions 
    int initV4L2();
    int videoEnable();
    int videoDisable();
    int isControl(int control, struct v4l2_queryctrl *queryctrl);
    int getControl(int control);
    int setControl(int control, int value);
};

#endif
 
