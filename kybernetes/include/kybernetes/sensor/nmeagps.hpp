/*
 *  nmeagps.hpp
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

#ifndef _kybernetes_sensor_nmeagps_h_
#define _kybernetes_sensor_nmeagps_h_

// Pull in some boost utilities
#include <boost/thread/thread.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/function.hpp>

// Language dependencies
#include <sys/time.h>
#include <string>
#include <vector>

// Other kybernetes dependencies
#include <kybernetes/io/serial.hpp>
#include <kybernetes/math/gps_common.hpp>

// Kybernetes namespace
namespace kybernetes
{
    // controller namespace
    namespace sensor
    {
        // class that manages the imu
        class NMEAGPS
        {
            // Typedefs for useful types
        public:
            // structure that contains the state of the gps
            typedef struct _nmeagps_state
            {
                kybernetes::math::GeoCoordinate location;
                time_t                          fix_time;
                double                          velocity;
                double                          heading;
                double                          variation;
            } state;
            
            typedef boost::function<void (NMEAGPS::state &)> callback;
            
        private:
            // Internal thread control
            boost::shared_ptr<boost::thread> m_thread;
            boost::mutex                     m_mutex; // Lock telemetry data while its being updated
            
            // The thread function
            void do_parsing();
            void write_nmea(std::string nmea);
            void start();
            void stop();
            
            // device interface
            kybernetes::io::SerialDevice    *m_device;
            std::string                      m_port;
            unsigned int                     m_baudrate;
            NMEAGPS::state                   m_state;
            
            // Updated callback
            NMEAGPS::callback                m_callback;
            
        public:
            // Constructor for the object
            NMEAGPS(std::string port, unsigned int baudrate);
            ~NMEAGPS();
            
            // Obtaining data
            
            
            // Callback registration
            void  registerCallback(NMEAGPS::callback c);
        };
    }
}

#endif