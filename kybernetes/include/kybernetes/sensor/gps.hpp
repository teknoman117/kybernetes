/*
 *  gps.hpp
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

#ifndef _kybernetes_sensor_gps_h_
#define _kybernetes_sensor_gps_h_

// GPS coordinate math
#include <kybernetes/math/gps_common.hpp>

// Kybernetes namespace
namespace kybernetes
{
    // sensor namespace
    namespace sensor
    {
        // class that manages the imu
        class GPS
        {
        public:
            // structure that contains the state of the gps
            typedef struct _gps_state
            {
                // GPS coordinate of the location of the GPS
                kybernetes::math::GeoCoordinate location;
                
                // Altitude of the GPS
                double                          altitude;
                
                // Mean error of the GPS
                double                          error;
                
                // Is this GPS packet valid
                bool                            valid;
            } state;
            
            // callback type for the imu being updated.  Callback objects have
            // to extend this class (IMU::callback)
            class callback
            {
            public:
                // Called to perform the callback
                virtual void gps_event_update(state s) {}
                
                // Called when the GPS becomes ready
                virtual void gps_event_ready() {}
                
                // Called when the GPS shuts down
                virtual void gps_event_stopped() {}
                
                // Called when the GPS encounters an error
                virtual void gps_event_error(int code, std::string description)
                {
                    std::cerr << "[GPS] Unhandled error (" << code << "): " << description << std::endl;
                }
            };
            
            // Obtaining data
            virtual state fetchState() = 0;
            
            // Callback registration
            virtual void registerCallback(callback *c) = 0;
            virtual void unregisterCallback(callback *c) = 0;
        };
    }
}

#endif
