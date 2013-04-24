/*
 *  serial.cpp
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

#include <kybernetes/io/serial.hpp>

#include <sstream> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <time.h>   // time calls

#include <sys/ioctl.h>

using namespace kybernetes::io;

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
size_t SerialDevice::read(char *s, size_t n)
{
    // Since we are blocking, attempt to get all the bytes
    size_t count = 0;
    while(count < n)
    {
        // Call the read command, adjust for any recalls because of incomplete reception
        int ret = System::Read(fd, s + count, n - count);

        // If we got -1, return that
        if(ret < 0) return -1;

        // Add the received bytes to the received count
        count += ret;
    }

    // Returned the received count
    return count;
}

size_t SerialDevice::write(char *s, size_t n)
{
    return System::Write(fd, s, n);
}

// token reader
bool SerialDevice::readToken(const char* token, size_t length)
{
    // If there are not enough values to represent the token, well obviously its not there
    if(available() < length)
        return false;
    
    // Check every element if its part of the token
    char b = 0;
    for(size_t i = 0; i < length; i ++)
    {
        // Read a byte from the imu device's buffer
        read(&b, 1);
        //std::cout << "Rcv'd: " << (short) b << " compare to: " << (short) token[i] << std::endl;
        
        // If the byte is not equivalent to that of the token, return false
        if(b != token[i])
        {
            return false;
        }
    }
    
    // return success, we synchronized with the imu data stream
    return true;
}

// Exceptions classes
SerialDeviceException::SerialDeviceException(std::string message)
{
    this->message = message;
}

// Access to some system functions
namespace System {
    // System read function
    size_t Read(int fd, void* buf, size_t count)
    {
        return read(fd, buf, count);
    }

    // System write function
    size_t Write(int fd, void* buf, size_t count)
    {
        return write(fd, buf, count);
    }

    // System close function
    void Close(int fd)
    {
        close(fd);
    }
}
