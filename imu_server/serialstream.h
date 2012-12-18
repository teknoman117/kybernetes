//
//  serialstream.h
//  
//
//  Created by Nathaniel Lewis on 3/11/12.
//  Copyright (c) UC Merced Robotics Society. All rights reserved.
//

#ifndef _serialstream_h_
#define _serialstream_h_

#include <string>
#include <termios.h>

#define BUFFER_INPUT 1
#define BUFFER_OUTPUT 2

// A base class to handle serial device exceptions
class SerialDeviceException {
public:
    SerialDeviceException(std::string message);
    std::string message;
};

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
    ssize_t read (char *s, size_t n);
    ssize_t write(char *s, size_t n);
    
private:
    int  fd;    // The serial port device identifier
    
};

namespace System {
    ssize_t Read (int fd, void* buf, size_t count);
    ssize_t Write(int fd, void* buf, size_t count);
    void   Close(int fd);
}

#endif
