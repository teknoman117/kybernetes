/*
 *  gps_navigate.cpp
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
#include <list>

// Kybernetes deps
#include <kybernetes/controller/sensor_controller.hpp>
#include <kybernetes/controller/motion_controller.hpp>
#include <kybernetes/sensor/razorgyro.hpp>
#include <kybernetes/sensor/garmingps.hpp>

// Flags
volatile bool __kill = false;

// Catch the kill signal
void handle_sigint (int sig)
{
    std::cout << "Terminating" << std::endl;
    __kill = true;
}

// Demo program
class gps_navigate_demo
    : public kybernetes::controller::MotionController::callback, public kybernetes::controller::SensorController::callback,
      public kybernetes::sensor::IMU::callback, public kybernetes::sensor::GPS::callback
{
    // Hardware interface objects
    kybernetes::controller::MotionController   *motion_controller;
    kybernetes::controller::SensorController   *sensor_controller;
    kybernetes::sensor::IMU                    *imu;
    kybernetes::sensor::GPS                    *gps;
    
    // Information about our path
    std::list<kybernetes::math::GeoCoordinate> &m_path;
    float                                       m_goal;
    
public:
    // Constructor for GPS navigation demo
    gps_navigate_demo(std::list<kybernetes::math::GeoCoordinate>& path)
        : m_path(path), m_goal(0.0)
    {
        // Start the motion controller
        motion_controller = new kybernetes::controller::MotionController("/dev/kybernetes/motion_controller", B57600);
        motion_controller->registerCallback(this);
        
        // Start the sensor controller
        sensor_controller = new kybernetes::controller::SensorController("/dev/kybernetes/sensor_controller", B57600);
        sensor_controller->registerCallback(this);
        
        // Start the razor imu
        imu = new kybernetes::sensor::RazorGyro("/dev/kybernetes/imu", B57600);
        imu->registerCallback(this);
        
        // Start the gps (Garmin 60csx)
        gps = new kybernetes::sensor::GarminGPS("/dev/kybernetes/gps", B9600);
        gps->registerCallback(this);
    }
    
    // Deconstructor for the GPS navigation demo
    ~gps_navigate_demo()
    {
        // Unregister all of the callbacks
        motion_controller->unregisterCallback(this);
        sensor_controller->unregisterCallback(this);
        imu->unregisterCallback(this);
        gps->unregisterCallback(this);
        
        // Close all of the hardware
        delete motion_controller;
        delete sensor_controller;
        delete imu;
        delete gps;
    }
    
    // The IMU updated
    void imu_event_update(kybernetes::sensor::IMU::state state)
    {
        // Calculate our directional error
        float error = m_goal - state.yaw;
        if(error >= 180) error -= 360;
        if(error <= -180) error += 360;
        
        // Calculate steering goal (essentially P-control; weight * error)
        short drift = error * 30;
        if(drift > 500) drift = 500;
        if(drift < -500) drift = -500;
        
        // Log the decision
        std::cout << "Current Heading = " << state.yaw << ", Target Error = " << error << ", wheel position = " << drift << std::endl;
        
        // Set the drift angle in the steering servos
        motion_controller->setDrift(-drift);
    }
    
    // GPS Updated callback
    void gps_event_update(kybernetes::sensor::GarminGPS::state state)
    {
        // If the GPS packet is valid and we have more to our path
        if(state.valid && m_path.begin() != m_path.end())
        {
            // Get the distance and heading to target
            double heading = state.location.headingTo(*(m_path.begin()));
            double distance = state.location.distanceTo(*(m_path.begin()));
            
            // Calculate the goal angle
            if(heading < 0.0f) heading = 360.0f + heading;
            m_goal = heading;
            
            // If we are far away, go fast
            if(distance > 20.0)
                motion_controller->setThrottle(80);
            
            // If we are approaching, go slower
            else if(distance > 4.0)
                motion_controller->setThrottle(50);
            
            // If we are on target, remove this path element and continue to the next
            else
                m_path.pop_front();
        }
        
        // If the GPS 
        else
            motion_controller->setThrottle(0);
    }
    
    // The main method of the demo
    void run()
    {
        // While no one kills the program, loop forever
        while(!__kill)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        }
    }
};

int main (int argc, char** argv)
{
    // Check that we have enough params
    if(argc < 2)
    {
        // Return docs
        std::cerr << "Fatal: Too few arguments" << std::endl;
        std::cerr << "Usage: " << argv[0] << " <coordinate list>" << std::endl;
        std::cout << std::endl;
        
        // Return fail
        return 1;
    }
    
    // Add a handler for Control-C
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = handle_sigint;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    signal(SIGPIPE, SIG_IGN);
    
    // Load the coordinates from the manifest file
    std::list<kybernetes::math::GeoCoordinate> path;
    std::ifstream                              manifest(argv[1]);
    while(!manifest.eof())
    {
        // Load the coordinate from the file
        kybernetes::math::GeoCoordinate coord;
        manifest >> coord.latitude;
        manifest >> coord.longitude;
        
        // Check if the coordinate is valid
        if(manifest.eof()) break;
        
        // Otherwise store the coordinate
        std::cout << std::setprecision(10) << "Coordinate = " << coord.latitude << "," << coord.longitude << std::endl;
        path.push_back(coord);
    }
    manifest.close();
    
    // Create the demo
    gps_navigate_demo demo(path);

    // Run the demo
    demo.run();
    
    // Return success
    return 0;
}


