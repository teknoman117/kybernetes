/*
 *  razorimu.cpp
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

#include <kybernetes/sensor/razorimu.hpp>

using namespace kybernetes::io;
using namespace kybernetes::sensor;

// Constructor for the object
RazorIMU::RazorIMU(std::string port, unsigned int baudrate) :
    m_port(port), m_baudrate(baudrate)
{
    // Do some initialization
    m_state.roll  = 0;
    m_state.pitch = 0;
    m_state.yaw   = 0;
    
    // Start the processing thread
    this->start();
}

RazorIMU::~RazorIMU()
{
    // Stop the processing thread
    this->stop();
}

// Thread control
void RazorIMU::start()
{
    // Start the processing thread
    m_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&RazorIMU::do_parsing, this)));
}

void RazorIMU::stop()
{
    // Interrupt the thread
    m_thread->interrupt();
    
    // Join the thread
    m_thread->join();
}

// Thread which parses the data from the IMU controller
void RazorIMU::do_parsing()
{
    // Attempt to open a connection to the device
    try
    {
        // Try to open device
        m_device = new SerialDevice(m_port, m_baudrate);
    } catch (SerialDeviceException &e)
    {
        // Alert of error if we encountered an exception
        std::cerr << "[RazorIMU:" << m_port << "] Could not open port: " << e.message << std::endl;
        
        // Stop thread
        return;
    }
    
    // Attempt to synchronize with the controller
    std::cout << "[RazorIMU:" << m_port << "] Waiting for device reset" << std::endl;
    boost::this_thread::sleep(boost::posix_time::seconds(3));
    
    // Request the synchronization token
    std::cout << "[RazorIMU:" << m_port << "] Attempting synchronization" << std::endl;
    std::string command = "";
    
    // Flush the input buffer
    //m_device->flush(BUFFER_INPUT);
    
    // Send the initialization commands
    command = "#ob";  // binary output
    m_device->write((char *) command.data(), 3);
    command = "#o1";  // streaming mode on
    m_device->write((char *) command.data(), 3);
    command = "#oe0"; // i don't know
    m_device->write((char *) command.data(), 4);
    
    // Request the synchronization token
    command = "#s00";
    m_device->write((char *) command.data(), 4);
    
    // Attempt to synchronize
    bool         synced = false;
    unsigned int count  = 0;
    while(!synced && count++ < 10000)
    {
        // Check if we have received the token yet
        synced = m_device->readToken("#SYNCH00\r\n", 10);
        
        // Stall a bit to allow the board to respond
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
    
    // Check if the process was successful
    if(synced)
    {
        std::cout << "[RazorIMU:" << m_port << "] Synchronized" << std::endl;
    } else
    {
        // If we failed to synchronize, fail out
        std::cerr << "[RazorIMU:" << m_port << "] Failed to synchronize" << std::endl;
        delete m_device;
        return;
    }
    
    // Flag that the IMU is ready
    m_ready = true;
            
    // Execute queued callbacks for the "imu becomes ready" event
    for(std::list<IMU::callback *>::iterator it = m_callbacks.begin(); it != m_callbacks.end(); ++it)
        (*it)->imu_event_ready();
    
    // Process data from the IMU
    try
    {
        // Locals to store currently downloading data
        IMU::state state;
        
        // Download the new data (always)
        while(1)
        {
            // Retrieve the IMU enabled value
            boost::this_thread::interruption_point();
            if(m_device->read((char *) &state.yaw, 4) != 4)
            {
                std::cerr << "[RazorIMU:" << m_port << "] Disconnected upon read error" << std::endl;
                m_thread->interrupt();
            }
            
            // Retrieve the on target value
            boost::this_thread::interruption_point();
            if(m_device->read((char *) &state.pitch, 4) != 4)
            {
                std::cerr << "[RazorIMU:" << m_port << "] Disconnected upon read error" << std::endl;
                m_thread->interrupt();
            }
            
            // Retrieve the odometer value
            boost::this_thread::interruption_point();
            if(m_device->read((char *) &state.roll, 4) != 4)
            {
                std::cerr << "[RazorIMU:" << m_port << "] Disconnected upon read error" << std::endl;
                m_thread->interrupt();
            }
            
            // Swap with shared copy
            boost::mutex::scoped_lock lock(m_mutex);
            m_state = state;
            lock.unlock();
            
            // Execute queued callbacks for the "imu updated" event
            for(std::list<IMU::callback *>::iterator it = m_callbacks.begin(); it != m_callbacks.end(); ++it)
                (*it)->imu_event_update(state);
        }
    } catch (boost::thread_interrupted)
    {
        // Well, something right?
    }
    
    // Alert that something has caused the IMU to shut down
    std::cerr << "[RazorIMU:" << m_port << "] Stopped updating IMU data" << std::endl;
    m_ready = false;
            
    // Execute queued callbacks for the "imu goes down" event
    for(std::list<IMU::callback *>::iterator it = m_callbacks.begin(); it != m_callbacks.end(); ++it)
        (*it)->imu_event_stopped();
    
    // Close the link to the device
    delete m_device;
}

// Store a callback object in our callbacks list
void RazorIMU::registerCallback(IMU::callback *c)
{
    m_callbacks.push_back(c);
}

// Remove a stored callback object in our callbacks list
void RazorIMU::unregisterCallback(IMU::callback *c)
{
    // Iterate through the callback list and look for an element equivalent to the one to remove
    m_callbacks.remove(c);
}

// Obtaining data
IMU::state RazorIMU::fetchState()
{
    boost::mutex::scoped_lock lock(m_mutex);
    return m_state;
}

// Is the IMU ready
bool RazorIMU::isReady()
{
    return m_ready;
}

