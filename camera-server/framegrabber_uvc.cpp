//==========================================================================
//
//  Project:        VRS - Vision Recogition System for Linux
//
//  Module:         Camera capture library
//
//  Description:    Captures images from video devices
//                    *Supports the V4L2 UVC interface
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

#include "framegrabber_uvc.h"
#include <jpeglib.h>

#include <sys/mman.h>
#include <sys/ioctl.h>

#include <iostream>

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

using namespace uvc_camera;

static unsigned char dht_data[DHT_SIZE] = {
  0xff, 0xc4, 0x01, 0xa2, 0x00, 0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02,
  0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x01, 0x00, 0x03,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
  0x0a, 0x0b, 0x10, 0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05,
  0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7d, 0x01, 0x02, 0x03, 0x00, 0x04,
  0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22,
  0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15,
  0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17,
  0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36,
  0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a,
  0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66,
  0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a,
  0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95,
  0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8,
  0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2,
  0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5,
  0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7,
  0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9,
  0xfa, 0x11, 0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04, 0x07, 0x05,
  0x04, 0x04, 0x00, 0x01, 0x02, 0x77, 0x00, 0x01, 0x02, 0x03, 0x11, 0x04,
  0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71, 0x13, 0x22,
  0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33,
  0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25,
  0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35, 0x36,
  0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a,
  0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66,
  0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a,
  0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94,
  0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
  0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba,
  0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
  0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7,
  0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa
};


// Supports V4L2_PIX_FMT_YUYV and V4L2_PIX_FMT_MJPEG
FrameGrabber_UVC::FrameGrabber_UVC( std::string device, int _width, int _height, int format )
    : videodevice(device), width(_width), height(_height) 
{
    if( (device.length() == 0) || (width == 0) || (height == 0) ) 
    {
        std::cerr << "Error: FrameGrabber_UVC - parameter error\n" << std::endl;
        return;
    }

    this->formatIn = format;

    if( this->initV4L2() < 0 ) 
    {
        std::cerr << "Fatal Error: FrameGrabber_UVC - device init failed" << std::endl;
        close( this->cam );
        return;
    }

    this->framesizeIn = (this->width * this->height << 1);
    switch (this->formatIn) 
    {
        case V4L2_PIX_FMT_MJPEG:
            this->tmpbuffer.resize(0);
            break;
        case V4L2_PIX_FMT_YUYV:
            this->tmpbuffer.resize(this->framesizeIn);
            break;
        default:
            std::cerr << "Fatal Error: FrameGrabber_UVC - unsupported format" << std::endl;
            close( this->cam );
            return;
    }

    if( this->videoEnable() ) {
        std::cerr << "Fatal Error: FrameGrabber_UVC - could not start video stream" << std::endl;
        close( this->cam );
        return;
    }
}

FrameGrabber_UVC::~FrameGrabber_UVC() {
    this->stop();
}

int FrameGrabber_UVC::initV4L2() {
    int i;
    this->cam = open( this->videodevice.c_str(), O_RDWR );
    if( this->cam < 0 ) {
        ////ROS_ERROR("Error: FrameGrabber_UVC::initV4L2() - failed to open device" );
        return -1;
    }
    memset (&this->cap, 0, sizeof (struct v4l2_capability));
    if( ioctl (this->cam, VIDIOC_QUERYCAP, &this->cap) < 0 ) {
        //ROS_ERROR("Error: FrameGrabber_UVC::initV4L2() - unable to query device %s", this->videodevice.c_str());
        return -1;
    }
    if ((this->cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0) {
        //ROS_ERROR("Error: FrameGrabber_UVC::initV4L2() - %s no video capture support", this->videodevice.c_str());
        return -1;
    }
    if (!(this->cap.capabilities & V4L2_CAP_STREAMING)) {
        //ROS_ERROR("Error: FrameGrabber_UVC::initV4L2() - %s no i/o streaming support", this->videodevice.c_str());
        return -1;
    }
    memset (&this->fmt, 0, sizeof (struct v4l2_format));
    this->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    this->fmt.fmt.pix.width = this->width;
    this->fmt.fmt.pix.height = this->height;
    this->fmt.fmt.pix.pixelformat = this->formatIn;
    this->fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if( ioctl(this->cam, VIDIOC_S_FMT, &this->fmt) < 0 ) {
        //ROS_ERROR( "Error: FrameGrabber_UVC::initV4L2() - Unable to set format" );
        return -1;
    }
    if ((this->fmt.fmt.pix.width != this->width) || (this->fmt.fmt.pix.height != this->height)) {
        //ROS_WARN( "Warning: FrameGrabber_UVC::initV4L2() - width %d height %d unavailable", this->fmt.fmt.pix.width, this->fmt.fmt.pix.height );
        this->width = this->fmt.fmt.pix.width;
        this->height = this->fmt.fmt.pix.height;
    }
    if( this->fmt.fmt.pix.pixelformat != this->formatIn ) {
        //ROS_WARN( "Warning: FrameGrabber_UVC::initV4L2() - Unable to get desired format" );
        this->formatIn = this->fmt.fmt.pix.pixelformat;
        //ROS_ERROR( "Got %d\n", this->formatIn );
    }
    memset (&this->rb, 0, sizeof (struct v4l2_requestbuffers));
    this->rb.count = NB_BUFFER;
    this->rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    this->rb.memory = V4L2_MEMORY_MMAP;
    if( ioctl (this->cam, VIDIOC_REQBUFS, &this->rb) < 0 ) {
        //ROS_ERROR( "Error: FrameGrabber_UVC::initV4L2() - Unable to allocate buffers" );
        return -1;
    }
    for (i = 0; i < NB_BUFFER; i++) {
        memset (&this->buf, 0, sizeof (struct v4l2_buffer));
        this->buf.index = i;
        this->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        this->buf.memory = V4L2_MEMORY_MMAP;
        if( ioctl (this->cam, VIDIOC_QUERYBUF, &this->buf) < 0 ) {
            //ROS_ERROR( "Error: FrameGrabber_UVC::initV4L2() - Unable to query buffer" );
            return -1;
        }
        this->mem[i] = mmap (0, this->buf.length, PROT_READ, MAP_SHARED, this->cam, this->buf.m.offset);
        if (this->mem[i] == MAP_FAILED) {
            //ROS_ERROR( "Error: FrameGrabber_UVC::initV4L2() - Unable to map buffer" );
            return -1;
        }
    }
    for (i = 0; i < NB_BUFFER; ++i) {
        memset (&this->buf, 0, sizeof (struct v4l2_buffer));
        this->buf.index = i;
        this->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        this->buf.memory = V4L2_MEMORY_MMAP;
        if( ioctl (this->cam, VIDIOC_QBUF, &this->buf) < 0) {
            //ROS_ERROR( "Error: FrameGrabber_UVC::initV4L2() - Unable to queue buffer" );
            return -1;
        }
    }
    return 0;
}

int FrameGrabber_UVC::videoEnable () {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if( ioctl (this->cam, VIDIOC_STREAMON, &type) < 0) {
        //ROS_ERROR( "Error: FrameGrabber_UVC::videoEnable() - Unable to start capture");
        return 1;
    }
    this->isstreaming = 1;
    return 0;
}

int FrameGrabber_UVC::videoDisable () {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if( ioctl (this->cam, VIDIOC_STREAMOFF, &type) < 0 ) {
        //ROS_ERROR( "Error: FrameGrabber_UVC::videoEnable() -Unable to stop capture" );
        return 1;
    }
    this->isstreaming = 0;
    return 0;
}

void FrameGrabber_UVC::stop() {
    int i;
    if (this->isstreaming) this->videoDisable();
    for (i = 0; i < NB_BUFFER; i++) munmap (this->mem[i], this->buf.length);
    this->tmpbuffer.resize(0);
    close (this->cam);
}

// Grab a JPEG blob from the camera
int FrameGrabber_UVC::grab_compressed(std::vector<unsigned char>& data)
{
    // Ensure that the device is streaming frames
    if(!this->isstreaming) {
        if( this->videoEnable() ) {
            //ROS_ERROR("Fatal Error: FrameGrabber_UVC::uvcGrab() - could not start video stream");
            return 1;
        }
    }

    // Set the buffer type
    memset( &this->buf, 0, sizeof (struct v4l2_buffer) );
    this->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    this->buf.memory = V4L2_MEMORY_MMAP;

    // Dequeue a video buffer frame
    if( ioctl (this->cam, VIDIOC_DQBUF, &this->buf) < 0 ) {
        //ROS_ERROR( "Fatal Error: FrameGrabber_UVC::uvcGrab() - Unable to dequeue buffer" );
        return 1;
    }

    // Figure out how to handle the frame data
    switch (this->formatIn) {
        case V4L2_PIX_FMT_MJPEG:
            data.resize(this->buf.bytesused + DHT_SIZE);
            memcpy (data.data(), this->mem[this->buf.index], HEADERFRAME1);
            memcpy (data.data() + HEADERFRAME1, dht_data, DHT_SIZE);
            memcpy (data.data() + HEADERFRAME1 + DHT_SIZE, static_cast<unsigned char *>(this->mem[this->buf.index]) + HEADERFRAME1, (this->buf.bytesused - HEADERFRAME1));
            break;
        case V4L2_PIX_FMT_YUYV:
            /*if (this->buf.bytesused > this->framesizeIn) {
                memcpy (this->framebuffer, this->mem[this->buf.index], this->framesizeIn);
            } else {
                memcpy (this->framebuffer, this->mem[this->buf.index], this->buf.bytesused);
            }
            break;*/
        default:
            //ROS_ERROR( "Fatal Error: FrameGrabber_UVC::uvcGrab() - camera using unsupported format" );
            return 1;
            break;
    }

    // Requeue the buffer
    if( ioctl (this->cam, VIDIOC_QBUF, &this->buf) < 0 ) {
        //ROS_ERROR( "Fatal Error: FrameGrabber_UVC::uvcGrab() - Unable to requeue buffer" );
        return 1;
    }
    return 0;
}

int FrameGrabber_UVC::grab_dummy()
{
    if(!this->isstreaming) {
        if( this->videoEnable() ) {
            //ROS_ERROR("Fatal Error: FrameGrabber_UVC::uvcGrab() - could not start video stream");
            return 1;
        }
    }
    memset( &this->buf, 0, sizeof (struct v4l2_buffer) );
    this->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    this->buf.memory = V4L2_MEMORY_MMAP;
    if( ioctl (this->cam, VIDIOC_DQBUF, &this->buf) < 0 ) {
        //ROS_ERROR( "Fatal Error: FrameGrabber_UVC::uvcGrab() - Unable to dequeue buffer" );
        return 1;
    }
    if( ioctl (this->cam, VIDIOC_QBUF, &this->buf) < 0 ) {
        //ROS_ERROR( "Fatal Error: FrameGrabber_UVC::uvcGrab() - Unable to requeue buffer" );
        //return 1;
    }
    return 0;
}

/*int FrameGrabber_UVC::grab(sensor_msgs::Image &image)
{
    if(!this->isstreaming) {
        if( this->videoEnable() ) {
            //ROS_ERROR("Fatal Error: FrameGrabber_UVC::uvcGrab() - could not start video stream");
            return 1;
        }
    }
    memset( &this->buf, 0, sizeof (struct v4l2_buffer) );
    this->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    this->buf.memory = V4L2_MEMORY_MMAP;
    if( ioctl (this->cam, VIDIOC_DQBUF, &this->buf) < 0 ) {
        //ROS_ERROR( "Fatal Error: FrameGrabber_UVC::uvcGrab() - Unable to dequeue buffer" );
        return 1;
    }
            
    if(this->formatIn == V4L2_PIX_FMT_YUYV) {
        // Prep the image object for the data
        image.width = this->width;
        image.height = this->height;
        image.is_bigendian = 0;
        image.step = this->width * 2;
        image.encoding = "yuv422";
        
        // Fill in the data
        if (this->buf.bytesused > this->framesizeIn) {
            image.data.resize(this->framesizeIn);
            memcpy (image.data.data(), this->mem[this->buf.index], this->framesizeIn);
        } else {
            image.data.resize(this->buf.bytesused);
            memcpy (image.data.data(), this->mem[this->buf.index], this->buf.bytesused);
        }
    } else if(this->formatIn == V4L2_PIX_FMT_MJPEG) {
        // Copy jpeg data into buffer
        unsigned int is = this->buf.bytesused + DHT_SIZE;
        if(tmpbuffer.size() < is) {
            //ROS_INFO("Resizing temporary buffer to %d byte(s)", is);
            tmpbuffer.resize(is);
        }
        memcpy (tmpbuffer.data(), this->mem[this->buf.index], HEADERFRAME1);
        memcpy (tmpbuffer.data() + HEADERFRAME1, dht_data, DHT_SIZE);
        memcpy (tmpbuffer.data() + HEADERFRAME1 + DHT_SIZE, static_cast<unsigned char*>(this->mem[this->buf.index]) + HEADERFRAME1, (this->buf.bytesused - HEADERFRAME1));
        
        // Decompress into RGB
        struct jpeg_decompress_struct cinfo;
        struct jpeg_error_mgr jerr;
        cinfo.err = jpeg_std_error (&jerr);
        jpeg_create_decompress (&cinfo);
        FILE *stream = fmemopen( tmpbuffer.data(), this->buf.bytesused + DHT_SIZE, "rb" );
        jpeg_stdio_src (&cinfo, stream);
        jpeg_read_header (&cinfo, TRUE);
        int nw = image.width = cinfo.image_width;
        int nh = image.height = cinfo.image_height;
        int ns = nw * nh * 3;
        image.step = nw * 3;
        image.data.resize(ns);
        image.encoding = "rgb8";
        image.is_bigendian = 0;
        unsigned char* ndatptr = image.data.data();
        if( cinfo.out_color_space != JCS_RGB ) {
            //ROS_ERROR( "Fatal Error: Image::decompress() - colorspace not supported" );
            jpeg_destroy_decompress( &cinfo );
            fclose( stream );
            return 1;
        }
        jpeg_start_decompress (&cinfo);
        while (cinfo.output_scanline < cinfo.output_height) {
                jpeg_read_scanlines (&cinfo, &ndatptr, 1);
                ndatptr += image.step;
        }
        jpeg_finish_decompress (&cinfo);
        jpeg_destroy_decompress (&cinfo);
        fclose( stream );
    } else if(this->formatIn == V4L2_PIX_FMT_RGB24) {
        // Prep the image object for the data
        image.width = this->width;
        image.height = this->height;
        image.is_bigendian = 0;
        image.step = this->width * 3;
        image.encoding = "rgb8";
        
        // Fill in the data
        if (this->buf.bytesused > this->framesizeIn)
            memcpy (tmpbuffer.data(), this->mem[this->buf.index], this->framesizeIn);
        else
            memcpy (tmpbuffer.data(), this->mem[this->buf.index], this->buf.bytesused);
        
        // Convert into RGB24
        image.data.resize(this->width * this->height * 3);
        unsigned char *src = tmpbuffer.data();
        unsigned char *dst = image.data.data();
        unsigned int i = 0;
        int y, u, v, r, g, b, z = 0;
        for( i = 0; i < this->width * this->height; i++ ) {
            if(!z) {
                y = src[0] << 8;
            } else {
                y = src[2] << 8;
            }
            u = src[1] - 128;
            v = src[3] - 128;
            r = (y + (359 * v)) >> 8;
            g = (y - (88 * u) - (183 * v)) >> 8;
            b = (y + (454 * u)) >> 8;
            *(dst++) = (r > 255) ? 255 : ((r < 0) ? 0 : r);
            *(dst++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
            *(dst++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);
            if(z++) {
                src += 4;
                z = 0;
            }
        }
    } else {
        //ROS_ERROR( "Fatal Error: FrameGrabber_UVC::uvcGrab() - camera using unsupported format\n" );
        return 1;
    }
    if( ioctl (this->cam, VIDIOC_QBUF, &this->buf) < 0 ) {
        //ROS_ERROR( "Fatal Error: FrameGrabber_UVC::uvcGrab() - Unable to requeue buffer\n" );
        return 1;
    }
    return 0;
}*/

int FrameGrabber_UVC::isControl(int control, struct v4l2_queryctrl *queryctrl) {
    queryctrl->id = control;
    if ( ioctl (this->cam, VIDIOC_QUERYCTRL, queryctrl) < 0) {
        //ROS_ERROR( "FrameGrabber_UVC::isControl() - ioctl querycontrol error\n" );
     } else if (queryctrl->flags & V4L2_CTRL_FLAG_DISABLED) {
        //ROS_ERROR( "FrameGrabber_UVC::isControl() - control %s disabled\n", (char *) queryctrl->name);
    } else if (queryctrl->flags & V4L2_CTRL_TYPE_BOOLEAN) {
        return 1;
    } else if (queryctrl->type & V4L2_CTRL_TYPE_INTEGER) {
        return 0;
    } else {
        //ROS_ERROR( "FrameGrabber_UVC::isControl() - control %s unsupported\n", (char *) queryctrl->name);
    }
    return -1;
}

int FrameGrabber_UVC::getControl(int control) {
    struct v4l2_queryctrl queryctrl;
    struct v4l2_control control_s;
    if (this->isControl (control, &queryctrl) < 0) return -1;
    control_s.id = control;
    if ( ioctl (this->cam, VIDIOC_G_CTRL, &control_s) < 0) {
        //ROS_ERROR( "FrameGrabber_UVC::getControl() - ioctl get control error\n");
        return -1;
    }
    return control_s.value;
}

int FrameGrabber_UVC::setControl(int control, int value) {
    struct v4l2_control control_s;
    struct v4l2_queryctrl queryctrl;
    int min, max;
    //int step, val_def;
    if (this->isControl (control, &queryctrl) < 0) return -1;
    min = queryctrl.minimum;
    max = queryctrl.maximum;
    //step = queryctrl.step;
    //val_def = queryctrl.default_value;
    if ((value >= min) && (value <= max)) {
        control_s.id = control;
        control_s.value = value;
        if (ioctl (this->cam, VIDIOC_S_CTRL, &control_s) < 0) {
            //ROS_ERROR( "FrameGrabber_UVC::setControl() - ioctl set control error\n" );
            return -1;
        }
    }
    return 0;
}

int FrameGrabber_UVC::saturation( int nsaturation ) {
    if( nsaturation ) {         //if nsaturation > 0 it will overwrite current one
        this->setControl(V4L2_CID_SATURATION, nsaturation);
    }
    return this->getControl(V4L2_CID_SATURATION);
}

int FrameGrabber_UVC::brightness( int nbrightness ) {
    if( nbrightness ) {         //if nbrightmess > 0 it will overwrite current one
        this->setControl(V4L2_CID_BRIGHTNESS, nbrightness);
    }
    return this->getControl(V4L2_CID_BRIGHTNESS);
}

int FrameGrabber_UVC::contrast( int ncontrast ) {
    if( ncontrast ) {         //if ncontrast > 0 it will overwrite current one
        this->setControl(V4L2_CID_CONTRAST, ncontrast);
    }
    return this->getControl(V4L2_CID_CONTRAST);
}

int FrameGrabber_UVC::gain( int ngain ) {
    if( ngain ) {         //if ngain > 0 it will overwrite current one
        this->setControl(V4L2_CID_GAIN, ngain);
    }
    return this->getControl(V4L2_CID_GAIN);
}

int FrameGrabber_UVC::resetPTZ( void ) {
    // V4L2 External Control Structures
    struct v4l2_ext_control xctrls;
    struct v4l2_ext_controls ctrls;

    // Set up controls for a total PTZ reset
    xctrls.id = V4L2_CID_TILT_RESET;
    xctrls.value = 1;
    ctrls.count = 1;
    ctrls.controls = &xctrls;

    // Execute
    if ( ioctl(this->cam, VIDIOC_S_EXT_CTRLS, &ctrls) < 0 )
    {
        std::cerr << "VIDIOC_S_EXT_CTRLS - Pan/Tilt error. Are the extended controls available?" << std::endl;
        return 1;
    }
    return 0;
}
 
int FrameGrabber_UVC::movePTZ(int pan, int tilt, int zoom)
{
    // V4L2 External Control Structures
    struct v4l2_ext_control xctrls[3];
    struct v4l2_ext_controls ctrls;

    // Set up controls for a PTZ move
    xctrls[0].id = V4L2_CID_PAN_RELATIVE;
    //xctrls[0].id = V4L2_CID_PAN_ABSOLUTE;
    xctrls[0].value = pan;
    xctrls[1].id = V4L2_CID_TILT_RELATIVE;
    //xctrls[1].id = V4L2_CID_TILT_ABSOLUTE;
    xctrls[1].value = tilt;
    ctrls.count = 2;
    ctrls.controls = xctrls;

    // Execute
    if ( ioctl(this->cam, VIDIOC_S_EXT_CTRLS, &ctrls) < 0 )
    {
        std::cerr << "VIDIOC_S_EXT_CTRLS - Pan/Tilt error. Are the extended controls available?" << std::endl;
        return 1;
    }
    return 0;
}
