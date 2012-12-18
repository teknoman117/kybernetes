/**
 * This node pulls images from a UVC camera and publishes either a YUYV, RGB24, or JPEG image
 * It also provides an interface to the Logitech Quickcam Orbit AF's pan/tilt system
 */

#include <iostream>
#include "framegrabber_uvc.h"

#include <xmlrpc-c/base.hpp>
#include <xmlrpc-c/registry.hpp>
#include <xmlrpc-c/server_abyss.hpp>

#include <boost/thread.hpp>

// Camera instance
uvc_camera::FrameGrabber_UVC *camera; 

// XML RPC camera move method
class movePTZMethod : public xmlrpc_c::method {
    // Framegrabber instance
    uvc_camera::FrameGrabber_UVC *_camera;

    // State storage
    int _pan;
    int _tilt;
    int _zoom;

public:
    // Method constructor
    movePTZMethod(uvc_camera::FrameGrabber_UVC *camera) 
        : _camera(camera), _pan(0), _tilt(0), _zoom(0)
    {
        // signature and help strings are documentation -- the client
        // can query this information with a system.methodSignature and
        // system.methodHelp RPC.
        this->_signature = "i:iii";

        // method's result and two arguments are integers
        this->_help = "This method moves the pan/tilt/zoom unit on Kybernetes' camera";
    }

    void
    execute(xmlrpc_c::paramList const& paramList,
            xmlrpc_c::value *   const  retvalP) {

        // Get the parameters
        int const dP(paramList.getInt(0));
        int const dT(paramList.getInt(1));
        int const dZ(paramList.getInt(2));

        // Verify the end of the command
        paramList.verifyEnd(3);

        // Perform the operation
        std::cout << "Derp = " << dP << "," << dT << "," << dZ << std::endl;
        camera->movePTZ(dP, dT, dZ);

        // Store the cumulative
        _pan += dP;
        _tilt += dT;
        _zoom += dZ;

        // Generate a return value
        std::map<std::string, xmlrpc_c::value> returnData;
        returnData.insert(std::pair<std::string, xmlrpc_c::value>("pan", xmlrpc_c::value_int(_pan)));
        returnData.insert(std::pair<std::string, xmlrpc_c::value>("tilt", xmlrpc_c::value_int(_tilt)));
        returnData.insert(std::pair<std::string, xmlrpc_c::value>("zoom", xmlrpc_c::value_int(_zoom)));
        *retvalP = xmlrpc_c::value_struct(returnData);
    }
};

// Thread to handle the camera stuff
void camera_thread()
{
    int c = 0;
    camera->resetPTZ();
    while (1)
    {
        // Pubish CompressedImages or Images
        camera->grab_dummy();
        //camera->grab_compressed(image_data);

        // Alert
        std::cout << "Frame: " << c++ << std::endl;
    }

    delete camera;
}

// Main thread
int main(int argc, char **argv)
{
    // Init the ROS node system
    std::vector<unsigned char> image_data;

    // Get the parameters (set for now)
    std::string videodev = "/dev/kybernetes/camera";
    std::string mode = "jpeg";
    int width = 640;
    int height = 480;

    // Set up the camera
    camera = new uvc_camera::FrameGrabber_UVC(videodev, width, height, mode);
    camera->grab_compressed(image_data);
    std::cout << "Configured" << std::endl;

    // Start the camera thread
    boost::thread cam_thread(camera_thread);

    // Start the XML-RPC server
    xmlrpc_c::registry myRegistry;
    xmlrpc_c::methodPtr const moveCameraMethod(new movePTZMethod(camera));
    myRegistry.addMethod("camera.move_relative", moveCameraMethod);

    xmlrpc_c::serverAbyss myAbyssServer(
        xmlrpc_c::serverAbyss::constrOpt()
            .registryP(&myRegistry)
            .portNumber(8080));

    myAbyssServer.run();

    // main loop
    return 0;
}
