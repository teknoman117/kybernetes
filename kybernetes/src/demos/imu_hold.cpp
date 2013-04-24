/*
 *  imu_hold.cpp
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

// Language deps
#include <iostream>
#include <signal.h>
#include <cmath>

// Kybernetes deps
#include <kybernetes/controller/sensor_controller.hpp>
#include <kybernetes/controller/motion_controller.hpp>
#include <kybernetes/sensor/razorgyro.hpp>

// Globals
volatile bool                           __kill = false;

// Catch the kill signal
void handle_sigint (int sig)
{
    std::cout << "Terminating" << std::endl;
    __kill = true;
}

// Object which represents the demo.  The main method will declare an instance of this and
// repeatedly call 
class imu_hold_demo : public kybernetes::sensor::IMU::callback, public kybernetes::controller::MotionController::callback, kybernetes::controller::SensorController::callback
{
    // IMU sensor control object
    kybernetes::sensor::IMU                  *imu;
    
    // Motion controller object
    kybernetes::controller::MotionController *motion_controller;
    
    // Sensor controller object
    kybernetes::controller::SensorController *sensor_controller;
    
    // Internal data
    float                                     goal;
    short                                     center;
    bool                                      steering;
public:
    // Initialize hardware required for the demo
    imu_hold_demo() : goal(0.0), center(0), steering(false)
    {
        // Start the razor imu and register ourself as a callback
        imu = new kybernetes::sensor::RazorGyro("/dev/kybernetes/imu", B57600);
        imu->registerCallback(this);

        // Start the motion controller
        motion_controller = new kybernetes::controller::MotionController("/dev/kybernetes/motion_controller", B57600);
        motion_controller->registerCallback(this);
        
        // Start the sensor controller
        sensor_controller = new kybernetes::controller::SensorController("/dev/kybernetes/sensor_controller", B57600);
        sensor_controller->registerCallback(this);
    }
    
    // Called to deconstruct the demo
    ~imu_hold_demo()
    {
        // Remove this from the imu's callbacks and stop the imu
        imu->unregisterCallback(this);
        delete imu;
        
        // Stop the motion controller
        motion_controller->unregisterCallback(this);
        delete motion_controller;
        
        // Stop the sensor controller
        sensor_controller->unregisterCallback(this);
        delete sensor_controller;
    }
    
    // Called to perform the imu callback
    void imu_event_update(kybernetes::sensor::IMU::state state)
    {
        // If the robot is steering, update the imu heading
        if (steering)
        {
            // Update the goal to the current IMU valud
            goal = state.yaw;
        } else
        {
            // Otherwise hold the heading
            // Calculate error to goal
            float error = goal - state.yaw;
            if(error >= 180) error -= 360;
            if(error <= -180) error += 360;
            
            // Calculate the steering value
            short drift = error * 30;
            if(drift > 500) drift = 500;
            if(drift < -500) drift = -500;

            // Set the steering value of the robot
            motion_controller->setDrift(-drift);
        }
    }
    
    // Called to perform the sensor controller callback
    void sensors_event_update(kybernetes::controller::SensorController::state state)
    {
        
    }
    
    // Called to perform the sensor controller callback
    void motors_event_update(kybernetes::controller::MotionController::state state)
    {
        // Instruct the motion controller to move forward a bit
        motion_controller->setThrottle(40);
        
        // Check if the center of the radio has been ascertained yet
        if(center == 0)
        {
            // store the current value for relations later
            center = state.drift;
            std::cout << "Center of steering channel = " << center << std::endl;
            return;
        }
        
        // Get the adjusted drift from the robot
        float delta = state.drift - center;
        
        // If the robot is in steering mode (more than 50 us derivation)
        if(fabs(delta) >= 25.0)
        {
            // Manually move the wheels
            steering = true;
            motion_controller->setDrift(delta);
            std::cout << "Drift = " << delta << std::endl;
        } else
            steering = false;
    }
    
    // 'Main' method of the demo
    void run()
    {
        
        // Loop while the interruption signal hasn't been fired
        while(!__kill)
        {
            // Just don't do anything
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        }
    
        // Stop the motors
        motion_controller->setThrottle(0);
    }
};

int main (int argc, char** argv)
{
    // Add a handler for Control-C
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = handle_sigint;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // Ignore sigpipe
    signal(SIGPIPE, SIG_IGN);
    
    // Start this demo
    imu_hold_demo demo;

    // Run the demo
    demo.run();

    // Return success
    return 0;
}


// code which calculates error between two headings
/*      
 
 // Calculate the heading (correct for initial position in relation to north)
 float heading = state.yaw;
 heading = heading - 90.0f;
 if(heading < 0.0f) heading = 360.0f + heading;
 
 // Calculate error to goal
 float error = goal - heading;
 if(error >= 180) error -= 360;
 if(error <= -180) error += 360;
 
 // Calculate the steering value
 short drift = error * 20;
 if(drift > 500) drift = 500;
 if(drift < -500) drift = -500;
 
 */
