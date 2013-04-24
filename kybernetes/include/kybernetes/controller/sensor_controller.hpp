/*
 *  sensor_controller.hpp
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

#ifndef _kybernetes_controller_sensor_h_
#define _kybernetes_controller_sensor_h_

// Pull in some boost utilities
#include <boost/thread/thread.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// Language dependencies
#include <sys/time.h>
#include <string>
#include <list>

// Other kybernetes dependencies
#include <kybernetes/io/serial.hpp>

// Kybernetes namespace
namespace kybernetes
{
    // controller namespace
    namespace controller
    {
        // class that manages incoming sensor traffic
        class SensorController
        {
            // Types created for the sensor controller
        public:
            // State object for the sensor controller
            typedef struct _sensorcontroller_state
            {
                unsigned char           bumpers;
                unsigned short          sonars[5];
            } state;
            
            // Sensor controller callback class
            class callback
            {
            public:
                // Called to perform the callback
                virtual void sensors_event_update(state s) {}
                
                // Called when the sensor controller becomes ready
                virtual void sensors_event_ready() {}
                
                // Called when the sensor controller shuts down
                virtual void sensors_event_stopped() {}
                
                // Called when the sensor controller encounters an error
                virtual void sensors_event_error(int code, std::string description)
                {
                    std::cerr << "[SensorController] Unhandled error (" << code << "): " << description << std::endl;
                }
            };
        private:
            // Internal thread control
            boost::shared_ptr<boost::thread>             m_thread;
            boost::mutex                                 m_mutex; // Lock sensor data while its being updated
            
            // The thread function
            void do_parsing();
            void start();
            void stop();
            
            // Sensor controller interface
            kybernetes::io::SerialDevice                *m_device;
            std::string                                  m_port;
            unsigned int                                 m_baudrate;
            SensorController::state                      m_state;
            bool                                         m_ready;
            
            // Callback objects
            std::list<SensorController::callback *>      m_callbacks;
            
        public:
            // Constructor for the object
            SensorController(std::string port, unsigned int baudrate);
            ~SensorController();
            
            // Obtaining data
            SensorController::state fetchState();
            
            // Callback registration
            void registerCallback(SensorController::callback *c);
            void unregisterCallback(SensorController::callback *c);
            
            // Check if the sensor controller is ready
            bool isReady();
        };
    }
}

#endif