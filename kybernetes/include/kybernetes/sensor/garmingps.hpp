/*
 *  garmingps.hpp
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
 *
 *  Reads a Garmin GPSMAP 60csx in Text Out mode.  Gets 1 Hz precision
 *  see - http://www8.garmin.com/support/text_out.html
 */

#ifndef _kybernetes_sensor_garmingps_h_
#define _kybernetes_sensor_garmingps_h_

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
#include <kybernetes/sensor/gps.hpp>

// Kybernetes namespace
namespace kybernetes
{
    // controller namespace
    namespace sensor
    {
        // class that manages the imu
        class GarminGPS : public kybernetes::sensor::GPS
        {
            // Internal thread control
            boost::shared_ptr<boost::thread> m_thread;
            boost::mutex                     m_mutex; // Lock telemetry data while its being updated
            
            // The thread function
            void do_parsing();
            void start();
            void stop();
            
            // device interface
            kybernetes::io::SerialDevice    *m_device;
            std::string                      m_port;
            unsigned int                     m_baudrate;
            GarminGPS::state                 m_state;
            bool                             m_ready;
            
            // Updated callback
            std::list<GPS::callback *>       m_callbacks;
            
        public:
            // Constructor for the object
            GarminGPS(std::string port, unsigned int baudrate);
            ~GarminGPS();
            
            // Obtaining data
            GPS::state       fetchState();
            bool             isReady();
            
            // Callback registration
            void registerCallback(GPS::callback *c);
            void unregisterCallback(GPS::callback *c);
        };
    }
}

#endif