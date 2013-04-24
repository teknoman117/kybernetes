/*
 *  imu.hpp
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

#ifndef _kybernetes_sensor_imu_h_
#define _kybernetes_sensor_imu_h_

// Kybernetes namespace
namespace kybernetes
{
    // sensor namespace
    namespace sensor
    {
        // class that manages the imu
        class IMU
        {
            // Typedefs for IMU
        public:
            // structure that contains the state of the imu 
            typedef struct _imu_state
            {
                // Roll of the IMU in degrees
                float roll;
                
                // Pitch of the IMY in degrees
                float pitch;
                
                // Yaw (heading) of the IMU in degrees
                float yaw;
            } state;
            
            // callback type for the imu being updated.  Callback objects have
            // to extend this class (IMU::callback)
            class callback
            {
            public:
                // Called to perform the callback
                virtual void imu_event_update(state s) {}
                
                // Called when the IMU becomes ready
                virtual void imu_event_ready() {}
                
                // Called when the IMU shuts down
                virtual void imu_event_stopped() {}
                
                // Called when the IMU encounters an error
                virtual void imu_event_error(int code, std::string description) 
                {
                    std::cerr << "[IMU] Unhandled error (" << code << "): " << description << std::endl;
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
