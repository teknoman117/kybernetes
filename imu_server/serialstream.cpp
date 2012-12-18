//
//  serialstream.cpp
//  
//
//  Created by Nathaniel Lewis on 3/11/12.
//  Copyright (c) UC Merced Robotics Society. All rights reserved.
//

#include "serialstream.h"

#include <sstream> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <time.h>   // time calls

#include <sys/ioctl.h>

SerialDevice::SerialDevice(std::string port, unsigned int baudrate) throw (SerialDeviceException)
{
    // Open the port
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd < 0)
    {
        // The port failed to open
        std::ostringstream error;
        error << "Failure connecting to port \"" << port << "\"" << std::ends;
        throw SerialDeviceException(error.str());
    } else {
        fcntl(fd, F_SETFL, 0);
    }

    // Get the current port settings
    struct termios settings;
    get_termios(&settings);

    // Set the baudrate
    cfsetispeed(&settings, baudrate);
    cfsetospeed(&settings, baudrate);

    // Set the port flow options
    settings.c_cflag &= ~PARENB;    // set no parity, 1 stop bit, 8 data bits
    settings.c_cflag &= ~CSTOPB;
    settings.c_cflag &= ~CSIZE;
    settings.c_cflag |= (CLOCAL | CREAD | CS8);
    settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    settings.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);
    settings.c_oflag &= ~(OPOST | ONLCR | OCRNL);

    // Set the port settings
    set_termios(&settings);
}

SerialDevice::~SerialDevice()
{
    // Close the port if its open
    if(fd > -1) System::Close(fd);
}

// Port settings control
void SerialDevice::setBaudrate(unsigned int baudrate)
{
    // Get the current settings
    struct termios settings;
    get_termios(&settings);
    
    // Set the baudrate
    cfsetispeed(&settings, baudrate);
    cfsetospeed(&settings, baudrate);
    
    // Set the new settings
    set_termios(&settings);
}

// Raw control of port
void SerialDevice::set_termios(struct termios *settings)
{
    tcsetattr(fd, TCSANOW, settings);
}

void SerialDevice::get_termios(struct termios *settings)
{
    tcgetattr(fd, settings);
}

// Port status reading
unsigned int SerialDevice::available()
{
    unsigned int bytes = 0;
    ioctl(fd, FIONREAD, &bytes);
    return bytes;
}

void SerialDevice::flush(unsigned int buffers)
{
    // Flush selected buffers
    if(buffers & BUFFER_INPUT) tcflush(fd, TCIFLUSH);
    if(buffers & BUFFER_OUTPUT) tcflush(fd, TCOFLUSH);
}

// IO Operations
ssize_t SerialDevice::read(char *s, size_t n)
{
    // Since we are blocking, attempt to get all the bytes
    size_t count = 0;
    while(count < n)
    {
        // Call the read command, adjust for any recalls because of incomplete reception
        size_t ret = System::Read(fd, s + count, n - count);

        // If we got -1, return that
        if(ret < 0) return -1;

        // Add the received bytes to the received count
        count += ret;
    }

    // Returned the received count
    return count;
}

ssize_t SerialDevice::write(char *s, size_t n)
{
    return System::Write(fd, s, n);
}

// Exceptions classes
SerialDeviceException::SerialDeviceException(std::string message)
{
    this->message = message;
}

// Access to some system functions
namespace System {
    // System read function
    ssize_t Read(int fd, void* buf, size_t count)
    {
        return read(fd, buf, count);
    }

    // System write function
    ssize_t Write(int fd, void* buf, size_t count)
    {
        return write(fd, buf, count);
    }

    // System close function
    void Close(int fd)
    {
        close(fd);
    }
}
