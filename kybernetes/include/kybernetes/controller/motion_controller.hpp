/*
 *  motion_controller.hpp
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

#ifndef _kybernetes_controller_motion_h_
#define _kybernetes_controller_motion_h_

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
        // class that manages motion control
        class MotionController
        {
            // Types for the motion controller
        public:
            // Contains the current state of the motion controller
            typedef struct _motion_controller_state
            {
                unsigned char           enabled;
                unsigned char           ontarget;
                float                   odometer;
                unsigned short          throttle;
                unsigned short          drift;
            } state;
            
            // Sensor controller callback class
            class callback
            {
            public:
                // Called to perform the callback
                virtual void motors_event_update(state s) {}
                
                // Called when the sensor controller becomes ready
                virtual void motors_event_ready() {}
                
                // Called when the sensor controller shuts down
                virtual void motors_event_stopped() {}
                
                // Called when the sensor controller encounters an error
                virtual void motors_event_error(int code, std::string description)
                {
                    std::cerr << "[MotionController] Unhandled error (" << code << "): " << description << std::endl;
                }
            };
        private:
            // Internal thread control
            boost::shared_ptr<boost::thread>        m_thread;
            boost::mutex                            m_mutex; // Lock telemetry data while its being updated
            
            // The thread function
            void do_parsing();
            void start();
            void stop();
            
            // Motion controller interface
            kybernetes::io::SerialDevice           *m_device;
            std::string                             m_port;
            unsigned int                            m_baudrate;
            MotionController::state                 m_state;
            bool                                    m_ready;
            
            // Updated callback
            std::list<MotionController::callback *> m_callbacks;
            
        public:
            // Constructor for the object
            MotionController(std::string port, unsigned int baudrate);
            ~MotionController();
            
            // Obtaining data
            MotionController::state fetchState();
            
            // Setting data
            void setTarget(int distance, unsigned short maxThrottle);
            void setThrottle(unsigned short throttle);
            void setDrift(unsigned short drift);
            
            // Callback registration
            void registerCallback(MotionController::callback *c);
            void unregisterCallback(MotionController::callback *c);
            
            // Check if the motion controller is ready
            bool isReady();
        };
    }
}

#endif
