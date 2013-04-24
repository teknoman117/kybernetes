/*
 *  avoid.cpp
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
#include <fstream>
#include <iomanip>
#include <vector>

// Kybernetes deps
#include <kybernetes/controller/sensor_controller.hpp>
#include <kybernetes/controller/motion_controller.hpp>

// Flags
volatile bool __kill = false;

// Catch the kill signal
void handle_sigint (int sig)
{
    std::cout << "Terminating" << std::endl;
    __kill = true;
}

// Demo object
class avoid_demo : public kybernetes::controller::SensorController::callback, public kybernetes::controller::MotionController::callback
{
    // Motion controller interface object
    kybernetes::controller::MotionController *motion_controller;
    
    // Sensor controller interface object
    kybernetes::controller::SensorController *sensor_controller;
public:
    // Construct the avoidance demo
    avoid_demo()
    {
        // Start the motion controller
        motion_controller = new kybernetes::controller::MotionController("/dev/kybernetes/motion_controller", B57600);
        motion_controller->registerCallback(this);
        
        // Start the sensor controller
        sensor_controller = new kybernetes::controller::SensorController("/dev/kybernetes/sensor_controller", B57600);
        sensor_controller->registerCallback(this);
    }
    
    // Stop the avoidance demo
    ~avoid_demo()
    {
        // Unregister the motion controller callback, and close the motion controller
        motion_controller->unregisterCallback(this);
        delete motion_controller;
        
        // Unregister the sensor controller callback, and close the sensor controller
        sensor_controller->unregisterCallback(this);
        delete sensor_controller;
    }
    
    // Sensors updated callback
    void sensors_event_update(kybernetes::controller::SensorController::state state)
    {
        // If we see something straight ahead
        if(state.sonars[2] < 90)
        {
            // We need to react fast, but react based on if the obstacle more to
            // the left than the right.
            if(state.sonars[1] < state.sonars[3])
            {
                motion_controller->setDrift(-500);
            } else
            {
                motion_controller->setDrift(500);
            }
        }
        
        // Is an obstacle visible to our left or right of center
        else if(state.sonars[1] < 90)
        {
            motion_controller->setDrift(-500);
        } else if(state.sonars[3] < 90)
        {
            motion_controller->setDrift(500);
        }
        
        // Is an obstacle slightly to our left or right
        else if(state.sonars[0] < 90)
        {
            motion_controller->setDrift(-350);
        } else if(state.sonars[4] < 90)
        {
            motion_controller->setDrift(350);
        }
        
        // If there is no obstacle of any threat
        else
            motion_controller->setDrift(0);
        
        // Make sure the robot is traversing
        motion_controller->setThrottle(20);
        
        // Output sonar readings
        std::cout << "Soanrs = ";
        for(int i = 0; i < 5; i++)
        {
            std::cout << state.sonars[i] << "\t";
        }
        std::cout << std::endl;
    }
    
    // Motion controller updated callback
    void motors_event_update(kybernetes::controller::MotionController::state state)
    {
        // Alert the world of the current odometer value
        //std::cout << "Odometer = " << state.odometer << std::endl;
    }
    
    // Demo main method
    void run()
    {
        // Loop while no one hits Control-C
        while(!__kill)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        }
        
        // Stop the robot before terminating the demo
        motion_controller->setThrottle(0);
        
        // Alert
        std::cerr << "Demo has stopped" << std::endl;
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
    
    // Create the object representing the demo
    avoid_demo demo;
    
    // Run the demo
    demo.run();
    
    // Return success
    return 0;
}

