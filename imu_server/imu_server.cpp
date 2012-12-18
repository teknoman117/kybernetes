#include <iostream>
#include <pthread.h>
#include <signal.h>

#include "serversocket.hpp"
#include "serialstream.h"

// Volatile shared members
volatile float pitch = 0.0;
volatile float roll = 0.0;
volatile float yaw = 0.0;

// Kill flag
volatile bool __kill = false;

using namespace mts::networking;

// Catch the kill signal
void handle_sigint (int sig)
{
    std::cout << "Terminating" << std::endl;
    __kill = true;
}

// token reader
bool readToken(SerialDevice& d, const char* token, size_t length)
{
    // If there are not enough values to represent the token, well obviously its not there
    if(d.available() < length)
        return false;

    // Check every element if its part of the token
    char b = 0;
    for(size_t i = 0; i < length; i ++)
    {
        // Read a byte from the imu device's buffer
        d.read(&b, 1);

        // If the byte is not equivalent to that of the token, return false
        if(b != token[i])
        {
            return false;
        }
    }

    // return success, we synchronized with the imu data stream
    return true;
}

// Thread which fetches IMU data
void *imu_thread (void *o)
{
    // Initialize the serial port
    std::string imu_dev = "/dev/kybernetes/imu";
    SerialDevice *imu;

    // Try to open the IMU's serial port
    try 
    {
        imu = new SerialDevice(imu_dev, B57600);
    } catch (SerialDeviceException &e)
    {
        std::cerr << "Error encountered with serial port: " << e.message << std::endl;
        exit(1);
    }

    // Wait for IMU to reset
    std::cout << "Waiting for Razor to complete reset" << std::endl;
    sleep(3);

    // Set up the IMU
    std::cout << "Syncing with IMU" << std::endl;
    bool synced = false;
    std::string command = "";

    // Send the initialization commands
    command = "#ob";  // binary output
    imu->write((char *) command.data(), 3);
    command = "#o1";  // streaming mode on
    imu->write((char *) command.data(), 3);
    command = "#oe0"; // IDK
    imu->write((char *) command.data(), 4);

    // Request the synchronization token
    command = "#s00";
    imu->write((char *) command.data(), 4);

    // Flush the input buffer
    imu->flush(BUFFER_INPUT);

    // Attempt to synchronize
    unsigned int count = 0;
    while(!synced && !__kill && count++ < 10000)
    {
        // Check if we have received the token yet
        synced = readToken(*imu, "#SYNCH00\r\n", 10);

        // Stall a bit to allow the board to respond
        usleep(1000);
    }

    // If we synchronized with the IMU versus failing out
    if(synced)
    {
        // Alert the log that we synchronized with the IMU
        std::cout << "Synchronized" << std::endl;
    } else
        // Otherwise, fail out
	goto not_synced;

    // Loop while we are not requested to die
    while(!__kill)
    {
        // Retrieve the yaw value
        if(imu->read((char *) &yaw, 4) != 4)
        {
            std::cerr << "Disconnected upon read error" << std::endl;
            break;
        }

        // Retrieve the pitch value
        if(imu->read((char *) &pitch, 4) != 4)
        {
            std::cerr << "Disconnected upon read error" << std::endl;
            break;
        }

        // Retrieve the roll value
        if(imu->read((char *) &roll, 4) != 4)
        {
            std::cerr << "Disconnected upon read error" << std::endl;
            break;
        }

        // Present
        //std::cout << "Y: " << yaw;
        //std::cout << "; P: " << pitch;
        //std::cout << "; R: " << roll << std::endl;
    }

// Fail to point when the device does synchronize or is requested to die
not_synced:

    // Stop the imu serial port
    delete imu;

    // Return null
    return NULL;
}

int main (int argc, char** argv)
{
    // Some defaults
    int         port    = 5000;

    // Add a handler for Control-C
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = handle_sigint;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // Ignore sigpipe
    signal(SIGPIPE, SIG_IGN);

    // Start the imu reader thread
    pthread_t imu_thread_t;
    pthread_create(&imu_thread_t, NULL, imu_thread, NULL);

    // Open the server
    ServerSocket server;
    server.startListening(port);

    // Loop until kill sent
    Socket client;
    while(!__kill)
    {
        // Listen for a connection
        if(!server.acceptConnection(client)) continue;

        // Handle connection while we are not requested to kill
        std::cout << "Client connected" << std::endl;
        while(!__kill)
        {
            // Write the Yaw to the stream
            if(!client.write((void *) &yaw, 4))
            {
                std::cerr << "Disconnected upon write error to client" << std::endl;
                break;
            }

            // Write the pitch to the stream
            if(!client.write((void *) &pitch, 4))
            {
                std::cerr << "Disconnected upon write error to client" << std::endl;
                break;
            }

            // Write the roll to the stream
            if(!client.write((void *) &roll, 4))
            {
                std::cerr << "Disconnected upon write error to client" << std::endl;
                break;
            }

            // Delay for a bit
            usleep(1000);
        }

        // Disconnect the client
        client.disconnect();
    }

    // Returning nicely
    pthread_join(imu_thread_t, NULL);
    std::cout << "IMU shut down" << std::endl;

    // Return success
    return 0;
}
