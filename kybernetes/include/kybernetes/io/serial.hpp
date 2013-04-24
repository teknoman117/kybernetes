/*
 *  serial.hpp
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

#ifndef _serial_h_
#define _serial_h_

#include <string>
#include <termios.h>

#define BUFFER_INPUT 1
#define BUFFER_OUTPUT 2

// Kybernetes namespace
namespace kybernetes
{
    // io namespace
    namespace io
    {
        // A base class to handle serial device exceptions
        class SerialDeviceException {
        public:
            SerialDeviceException(std::string message);
            std::string message;
        };

        // A class to encapsulate a serial port
        class SerialDevice {
        public:
            // Constructor/Deconstructor
            SerialDevice(std::string port, unsigned int baudrate) throw (SerialDeviceException);
            ~SerialDevice();
            
            // Port setup
            void setBaudrate(unsigned int baudrate);
            
            // Raw port settings control
            void set_termios(struct termios *settings);
            void get_termios(struct termios *settings);
            
            // Port status 
            unsigned int available();  // returns bytes currently in buffer
            void         flush(unsigned int buffers);      // flush the buffers
            
            // IO Operations
            size_t read (char *s, size_t n);
            size_t write(char *s, size_t n);
            
            // Utility function for searching for a particular string
            bool readToken(const char* token, size_t length);
        private:
            int  fd;    // The serial port device identifier
        };
    }
}

namespace System {
    size_t Read (int fd, void* buf, size_t count);
    size_t Write(int fd, void* buf, size_t count);
    void   Close(int fd);
}

#endif
