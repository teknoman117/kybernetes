/*
 *  nmeagps.cpp
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

#include <kybernetes/sensor/nmeagps.hpp>

#include <iomanip>
#include <sstream>

using namespace kybernetes::io;
using namespace kybernetes::sensor;

// Constructor for the object
NMEAGPS::NMEAGPS(std::string port, unsigned int baudrate = B4800) :
    m_port(port), m_baudrate(baudrate)
{
    // Do some initialization
    
    // Start the processing thread
    this->start();
}

NMEAGPS::~NMEAGPS()
{
    // Stop the processing thread
    this->stop();
}

// Thread control
void NMEAGPS::start()
{
    // Start the processing thread
    m_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&NMEAGPS::do_parsing, this)));
}

void NMEAGPS::stop()
{
    // Interrupt the thread
    m_thread->interrupt();
    
    // Join the thread
    m_thread->join();
}

// Thread which parses the data from the IMU controller
void NMEAGPS::do_parsing()
{
    // Attempt to open a connection to the device
    try
    {
        // Try to open device
        m_device = new SerialDevice(m_port, m_baudrate);
    } catch (SerialDeviceException &e)
    {
        // Alert of error
        std::cerr << "[NMEAGPS:" << m_port << "] Could not open port: " << e.message << std::endl;
        
        // Stop thread
        return;
    }
    
    // Disable all output messages
    //write_nmea("PGRMO,,2");
    
    // Flush the input buffer
    m_device->flush(BUFFER_INPUT);
    
    // Process data from the IMU controller
    try
    {
        // Locals to store currently downloading data
        NMEAGPS::state state;
        std::string    sentence;
        
        // Download the new data (always)
        while(1)
        {
            // Provide an interruption point
            boost::this_thread::interruption_point();
            
            // --- Get a sentence ---
            // align with the beginning of a sentence
            char b = 0;
            while(b != '$') m_device->read(&b, 1);
            
            // Get the sentence
            sentence.clear();
            while(1)
            {
                // Fetch a character from the buffer
                m_device->read(&b, 1);
                
                // If is one of the end characters, drop it or break out
                if(b == '\r') continue;
                else if(b == '\n') break;
                
                // If its not, store the character
                else sentence += b;
            }
            
            // Process string(s)
            // IMPLEMENT!!!!!
            std::cout << "[NMEAGPS:" << m_port << "] Received: " << sentence << std::endl;
            
            // Swap with shared copy
            boost::mutex::scoped_lock lock(m_mutex);
            m_state = state;
            lock.unlock();
            
            // Execute the callback
            if(!m_callback.empty()) m_callback(state);
        }
        
    } catch (boost::thread_interrupted)
    {
        
    }
    
    // Alert
    std::cerr << "[NMEAGPS:" << m_port << "] Stopped updating GPS data" << std::endl;
    
    // Close the link to the device
    delete m_device;
}

void NMEAGPS::write_nmea(std::string nmea)
{
    // First we have to calculate the checksum.  Checksum is every ascii character between $ and * exclusive or'd to each other
    char checksum = 0;
    for(std::string::iterator it = nmea.begin(); it != nmea.end(); ++it)
        checksum ^= *it;
    
    // Calculate the final string
    std::ostringstream message;
    message << "$" << nmea << "*" << std::hex << (unsigned) checksum << "\r\n";
    std::string sentence = message.str();
    
    // Send the message to the gps
    m_device->write((char *) sentence.data(), sentence.size());
}

void NMEAGPS::registerCallback(NMEAGPS::callback c)
{
    m_callback = c;
}

