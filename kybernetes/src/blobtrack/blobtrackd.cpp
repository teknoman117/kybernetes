/*
 *  blobtrackd.h
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

// Kybernetes dependencies
#include <kybernetes/sensor/uvccamera.hpp>
#include <kybernetes/network/serversocket.hpp>
#include <kybernetes/cv/cv.hpp>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// cvbloblib
#include <cvblobs/BlobResult.h>

// Boost
#include <boost/thread/thread.hpp>

// Text formats
#include <jsoncpp/json/json.h>

// Language deps
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <cassert>
#include <csignal>

// Camera information
kybernetes::sensor::UVCCamera     *camera;
int                                width;            
int                                height;           
int                                port;
int                                pt_response;
bool                               should_track;

// Resources for imaging
struct v4l2_buffer                 image_buffer;
void*                              image_data = NULL;
size_t                             image_size;

// Image tracking resultant                 
void*                              image_resultant = NULL;

// Color components
uint8_t                            y_max = 0;
uint8_t                            y_min = 0;
uint8_t                            u_max = 0;
uint8_t                            u_min = 0;
uint8_t                            v_max = 0;
uint8_t                            v_min = 0;

// Blob
CvRect                             boundingBox;

// Synchronization for resources
boost::shared_mutex                 image_data_mutex;
boost::condition_variable           image_data_condition;

boost::shared_mutex                 image_resultant_mutex;
boost::condition_variable           image_resultant_condition;

boost::shared_mutex                 blob_mutex;
boost::condition_variable           blob_condition;

boost::shared_mutex                 range_mutex;

// Flags
bool                              __kill = false;

// Catch the kill signal
void handle_sigint (int sig)
{
    __kill = true;
}

// Thread to handle image capture and processing
void image_process_thread(const char* device)
{
    // Local copy of image data
    struct v4l2_buffer buffer;
    void*              data;
    size_t             size;
    void*              resultant;
    void*              erosion;

    // Start the camera
    camera = new kybernetes::sensor::UVCCamera(device, width, height, V4L2_PIX_FMT_YUYV);
    std::cout << " >> Using camera on: " << device << std::endl;
    
    // Wait for camera to initialize by requesting a frame
    camera->capture_buffer(&buffer, &data, &size);
    camera->release_buffer(&buffer);
    
    // Reset the pan/tilt turret of the camera
    camera->ptz_reset();
    
    // Create the structuring element for image erosion
    int erosion_size = 2;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*erosion_size + 1, 2*erosion_size + 1), cv::Point(erosion_size, erosion_size));

    // Loop while not killed
    while(!__kill)
    {
        // Capture a new image buffer, release the old, and wake threads waiting for a new image
        {
            // Capture an image and get the system buffer
            camera->capture_buffer(&buffer, &data, &size);
            
            // Get a unique lock to the image data
            boost::unique_lock<boost::shared_mutex> uniqueLock(image_data_mutex);
            
            // Check if we have an old image buffer, if so release it
            if(image_data) 
                camera->release_buffer(&image_buffer);
            
            // Update the shared data with the new image buffer
            image_buffer = buffer;
            image_data = data;
            image_size = size;
            
            // Alert potential other threads that we now have an image
            image_data_condition.notify_all();
        }
        
        // Perform morphology to get the new image resultant for labeling
        {
            // Allocate a new buffer for image operations result
            resultant = malloc (width * height);
            
            // Create wrappers for OpenCV
            cv::Mat threshold_result(cv::Size(width, height), CV_8UC1);
            cv::Mat erosion_result(cv::Size(width, height), CV_8UC1, resultant);
            
            // Perform the bithreshold operation
            boost::shared_lock<boost::shared_mutex> rangeLock(range_mutex);
            kybernetes::cv::yuv422_bithreshold(data, threshold_result.ptr(), width, height, y_min, u_min, v_min, y_max, u_max, v_max);
            rangeLock.unlock();
            
            // Perform erosion operation
            boost::unique_lock<boost::shared_mutex> resultantLock(image_resultant_mutex);
            cv::erode(threshold_result, erosion_result, element);
            
            // Wait threads waiting on the image resultant
            image_resultant_condition.notify_all();
        }
        
        // Perform the blob tracking operation
        {
            
        }
    }

    // Close the camera
    delete camera;
    std::cout << " << Camera closed" << std::endl;
}

// Network access thread
void client_handle_thread(kybernetes::network::Socket *client)
{
    // Message buffer
    Json::FastWriter jsonWriter;
    Json::Reader     jsonReader;
    std::string      message;
    std::string      response;
    bool             valid = true;
    
    // Alert that a client has connected
    std::cout << " >> Client Connected" << std::endl;
    
    // Loop while the program has not been requested to be killed
    while(!__kill && valid)
    {
        // Get a message from the client
        message.clear();
        while(message.back() != '\n' && !__kill)
        {
            // Attempt to read from the socket
            char c = 0;
            if(!client->read((void *) &c, 1))
            {
                // Something went wrong, exit
                valid = false;
                break;
            }
            message.push_back(c);
        }
        
        // Check if we are still valid
        if(!valid || __kill) break;
        
        // Process the message (decompose into JSON)
        Json::Value root;
        if(!jsonReader.parse(message, root))
        {
            // The JSON message was malformed
            continue;
        }
        
        // Now we have a valid Json blob, so process the command
        response.clear();
        if(root["command"].asString() == "echo")
        {
            // Command was to echo whatever was send (just return the received message)
            Json::Value r;
            r["data"] = root;
            r["response"] = "ack";
            r["command"] = "echo";
            
            // Encode the JSON
            response = jsonWriter.write(r);
        } else if(root["command"].asString() == "set_parameters")
        {
            // Lock the range settings
            boost::shared_lock<boost::shared_mutex> lock(range_mutex);
            
            // Get the parameters from the message
            y_max = root["data"].get("y_max", 0).asUInt();
            y_min = root["data"].get("y_min", 0).asUInt();
            u_max = root["data"].get("u_max", 0).asUInt();
            u_min = root["data"].get("u_min", 0).asUInt();
            v_max = root["data"].get("v_max", 0).asUInt();
            v_min = root["data"].get("v_min", 0).asUInt();
            
            // Return an acknowledgement
            Json::Value r;
            r["response"] = "ack";
            r["command"] = "set_parameters";
            
            // Encode JSON
            response = jsonWriter.write(r);
        } else if(root["command"].asString() == "get_blob")
        {
            // Lock the blob information
            
        }
        
        else
        {
            // Command was not recognized, return a NACK (not ack)
            Json::Value r;
            r["response"] = "nack";
            r["command"] = root["command"];
            response = jsonWriter.write(r);
        }
        
        // Write request to the client
        if(!client->write((void *) response.data(), response.size()))
        {
            // Problem writing, close connection
            valid = false;
            break;
        }
    }

    // Close connection to client
    std::cout << " << Client disconnected" << std::endl;
    delete client;
}

// Main thread
int main(int argc, char **argv)
{
    // Check that we have the proper number of parameters
    if(argc < 6)
    {
        std::cerr << "Error: Too Few Arguments" << std::endl;
        std::cerr << "Usage: " << argv[0] << " <video device file> <width> <height> <port> <pt response per pixel error>" << std::endl;
        std::cerr << "   (e.g. " << argv[0] << " /dev/video0 640 480 8080 4)" << std::endl;
        return 1;
    }
    
    // Add a handler for Control-C
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = handle_sigint;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    
    // Ignore sigpipe
    signal(SIGPIPE, SIG_IGN);

    // Get the parameters (set for now)
    width            = atoi(argv[2]);
    height           = atoi(argv[3]);
    port             = atoi(argv[4]);
    pt_response      = atoi(argv[5]);
    should_track     = false;
    
    // Start the image processing thread
    boost::thread processing_thread(image_process_thread, argv[1]);
    
    // Start the server
    kybernetes::network::ServerSocket server;
    if(!server.startListening(port))
    {
        // Could not start server
        std::cerr << " << Could not start server on port " << port << std::endl;
        
        // Flag to kill
        __kill = true;
    }
    
    // Loop while we are not to be killed
    while(!__kill)
    {
        // Create a new socket object
        kybernetes::network::Socket *client = new kybernetes::network::Socket();
        
        // Get a connection from a client
        while(!server.acceptConnection(*client) && !__kill) 
            continue;
            
        // If we are supposed to kill the 
        if(__kill) break;
        
        // Create a thread to handle the client
        boost::thread handle(client_handle_thread, client);
    }
    
    // Stop the server
    server.stopListening();
    std::cout << " << Server Down" << std::endl;
    processing_thread.join();
    
    // Return success
    return 0;   
}
