/*
 *  sensor_controller.cpp
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

#include <kybernetes/controller/sensor_controller.hpp>

using namespace kybernetes::io;
using namespace kybernetes::controller;

// Constructor for the object
SensorController::SensorController(std::string port, unsigned int baudrate) :
    m_port(port), m_baudrate(baudrate)
{
    // Do some initialization
    m_ready = false;
    
    // Start the processing thread
    this->start();
}

SensorController::~SensorController()
{
    // Stop the processing thread
    this->stop();
}

// Thread control
void SensorController::start()
{
    // Start the processing thread
    m_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&SensorController::do_parsing, this)));
}

void SensorController::stop()
{
    // Interrupt the thread
    m_thread->interrupt();
    
    // Join the thread
    m_thread->join();
}

// Thread which parses the data from the sensor controller
void SensorController::do_parsing()
{
    // Attempt to open a connection to the device
    try
    {
        // Try to open device
        m_device = new SerialDevice(m_port, m_baudrate);
    } catch (SerialDeviceException &e)
    {
        // Alert of error
        std::cerr << "[SensorController:" << m_port << "] Could not open port: " << e.message << std::endl;
        
        // Stop thread
        return;
    }
    
    // Attempt to synchronize with the controller
    std::cout << "[SensorController:" << m_port << "] Waiting for device reset" << std::endl;
    boost::this_thread::sleep(boost::posix_time::seconds(3));
    
    // Flush the input buffer
    m_device->flush(BUFFER_INPUT);
    
    // Request the synchronization token
    std::cout << "[SensorController:" << m_port << "] Attempting synchronization" << std::endl;
    std::string command = "#a";
    m_device->write((char *) command.data(), 2);
    
    // Attempt to synchronize
    bool         synced = false;
    unsigned int count  = 0;
    while(!synced && count++ < 10000)
    {
        // Check if we have received the token yet
        synced = m_device->readToken("#SYNCH\r\n", 8);
        
        // Stall a bit to allow the board to respond
        boost::this_thread::sleep(boost::posix_time::microseconds(100));
    }
    
    // Check if the process was successful
    if(synced)
    {
        std::cout << "[SensorController:" << m_port << "] Synchronized" << std::endl;
    } else
    {
        // If we failed to synchronize, fail out
        std::cerr << "[SensorController:" << m_port << "] Failed to synchronize" << std::endl;
        delete m_device;
        return;
    }
    
    // Flag that the IMU is ready
    m_ready = true;
    
    // Execute queued callbacks for the "sensor controller becomes ready" event
    for(std::list<SensorController::callback *>::iterator it = m_callbacks.begin(); it != m_callbacks.end(); ++it)
        (*it)->sensors_event_ready();
    
    // Process data from the sensor controller
    try
    {
        // Locals to store currently downloading data
        SensorController::state state;
        
        // Download the new data (always)
        while(1)
        {
            // Retrieve the bumper value
            boost::this_thread::interruption_point();
            if(m_device->read((char *) &state.bumpers, 1) != 1)
            {
                std::cerr << "[SensorController:" << m_port << "] Disconnected upon read error" << std::endl;
                m_thread->interrupt();
            }
            
            // Retrieve the sonar values
            boost::this_thread::interruption_point();
            if(m_device->read((char *) state.sonars, 10) != 10)
            {
                std::cerr << "[SensorController:" << m_port << "] Disconnected upon read error" << std::endl;
                m_thread->interrupt();
            }
            
            // Swap with shared copy
            boost::mutex::scoped_lock lock(m_mutex);
            m_state = state;
            lock.unlock();
            
            // Execute queued callbacks for the "sensor controller updates" event
            for(std::list<SensorController::callback *>::iterator it = m_callbacks.begin(); it != m_callbacks.end(); ++it)
                (*it)->sensors_event_update(state);
        }
        
    } catch (boost::thread_interrupted)
    {
        
    }
    
    // Alert
    std::cerr << "[SensorController:" << m_port << "] Stopped updating sensor data" << std::endl;
    
    // Execute queued callbacks for the "sensor controller goes down" event
    m_ready = false;
    for(std::list<SensorController::callback *>::iterator it = m_callbacks.begin(); it != m_callbacks.end(); ++it)
        (*it)->sensors_event_stopped();
    
    // Close the link to the device
    delete m_device;
}

SensorController::state SensorController::fetchState()
{
    boost::mutex::scoped_lock lock(m_mutex);
    return m_state;
}

// Store a callback object in our callbacks list
void SensorController::registerCallback(SensorController::callback *c)
{
    m_callbacks.push_back(c);
}

// Remove a stored callback object in our callbacks list
void SensorController::unregisterCallback(SensorController::callback *c)
{
    // Iterate through the callback list and look for an element equivalent to the one to remove
    m_callbacks.remove(c);
}

// Is the sensor controller ready
bool SensorController::isReady()
{
    return m_ready;
}