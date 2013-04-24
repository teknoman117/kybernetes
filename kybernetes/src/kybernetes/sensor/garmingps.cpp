/*
 *  garmingps.cpp
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

#include <kybernetes/sensor/garmingps.hpp>

#include <iomanip>
#include <sstream>

using namespace kybernetes::io;
using namespace kybernetes::sensor;

// Constructor for the object
GarminGPS::GarminGPS(std::string port, unsigned int baudrate = B9600) :
m_port(port), m_baudrate(baudrate)
{
    // Do some initialization
    m_ready = false;
    
    // Start the processing thread
    this->start();
}

GarminGPS::~GarminGPS()
{
    // Stop the processing thread
    this->stop();
}

// Thread control
void GarminGPS::start()
{
    // Start the processing thread
    m_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&GarminGPS::do_parsing, this)));
}

void GarminGPS::stop()
{
    // Interrupt the thread
    m_thread->interrupt();
    
    // Join the thread
    m_thread->join();
}

// Thread which parses the data from the IMU controller
void GarminGPS::do_parsing()
{
    // Attempt to open a connection to the device
    try
    {
        // Try to open device
        m_device = new SerialDevice(m_port, m_baudrate);
    } catch (SerialDeviceException &e)
    {
        // Alert of error
        std::cerr << "[GarminGPS:" << m_port << "] Could not open port: " << e.message << std::endl;
        
        // Stop thread
        return;
    }
    
    // Flush the input buffer
    m_device->flush(BUFFER_INPUT);
    
    // Process data from the IMU controller
    try
    {
        // Locals to store currently downloading data
        GPS::state       state;
        std::string      sentence;
        
        // Download the new data (always)
        while(1)
        {
            // Provide an interruption point
            boost::this_thread::interruption_point();
            
            // --- Get a sentence ---
            // align with the beginning of a sentence
            char b = 0;
            while(b != '@') m_device->read(&b, 1);
            
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
            
            // Alert if the GPS has transistioned to ready
            if(!m_ready)
            {
                // Flag ready
                m_ready = true;
                
                // Perform ready callbacks
                for(std::list<GPS::callback *>::iterator it = m_callbacks.begin(); it != m_callbacks.end(); ++it)
                    (*it)->gps_event_ready();
            }
            
            // Process the time component
            std::string component_time = sentence.substr(0, 12);
            
            // Process the latitude
            double latitude, longitude, multipler = (sentence[12] == 'N') ? 1.0 : -1.0;
            std::string component_latitude_deg = sentence.substr(13, 2);
            latitude = atof(component_latitude_deg.c_str());
            std::string component_latitude_min = sentence.substr(15, 2) + "." + sentence.substr(17,3);
            latitude += atof(component_latitude_min.c_str()) / 60.0f;
            latitude *= multipler;
            state.location.latitude = latitude;
            
            // Process the longitude
            multipler = (sentence[20] == 'E') ? 1.0 : -1.0;
            std::string component_longitude_deg = sentence.substr(21, 3);
            longitude = atof(component_longitude_deg.c_str());
            std::string component_longitude_min = sentence.substr(24, 2) + "." + sentence.substr(26,3);
            longitude += atof(component_longitude_min.c_str()) / 60.0f;
            longitude *= multipler;
            state.location.longitude = longitude;
            
            // Check if our location is valid
            if(sentence[29] == '_' || sentence[29] == 'S')
                state.valid = false;
            else
                state.valid = true;
            
            // Check our error
            std::string component_eph = sentence.substr(30, 3);
            state.error = atof(component_eph.c_str());
            
            // Check our altitude
            multipler = (sentence[33] == '+') ? 1.0 : -1.0;
            std::string component_altitude = sentence.substr(34, 5);
            state.altitude = atof(component_altitude.c_str());
            
            // Swap with shared copy
            boost::mutex::scoped_lock lock(m_mutex);
            m_state = state;
            lock.unlock();
            
            // Perform update callbacks
            for(std::list<GPS::callback *>::iterator it = m_callbacks.begin(); it != m_callbacks.end(); ++it)
                (*it)->gps_event_update(state);
        }
        
    } catch (boost::thread_interrupted)
    {
        
    }
    
    // Alert to shut down
    std::cerr << "[GarminGPS:" << m_port << "] Stopped updating GPS data" << std::endl;
    m_ready = false;
    
    // Perform shutdown callbacks
    for(std::list<GPS::callback *>::iterator it = m_callbacks.begin(); it != m_callbacks.end(); ++it)
        (*it)->gps_event_stopped();
    
    // Close the link to the device
    delete m_device;
}

// Return the state of the GPS
GPS::state GarminGPS::fetchState()
{
    boost::mutex::scoped_lock lock(m_mutex);
    return m_state;
}

// Return if the GPS is ready
bool GarminGPS::isReady()
{
    return m_ready;
}

// Callback registration
void GarminGPS::registerCallback(GPS::callback *c)
{
    // Add the callback to our callback list
    m_callbacks.push_back(c);
}

void GarminGPS::unregisterCallback(GPS::callback *c)
{
    // Remove the callback from our callback list
    m_callbacks.remove(c);
}

