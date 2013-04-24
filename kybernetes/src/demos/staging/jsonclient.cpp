// system deps
#include <iostream>
#include <sstream>

// kybernetes deps
#include <kybernetes/network/socket.hpp>
#include <jsoncpp/json/json.h>

// Boost
#include <boost/thread/thread.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// main
int main (int argc, char** argv)
{
    // Utilities
    Json::FastWriter jsonWriter;
    
    // Open a server socket
    kybernetes::network::Socket socket;
    socket.connectTo("localhost", 8080);
    
    // Create a blob of json data to send
    Json::Value root;
    root["command"] = "echo";
    std::string message = jsonWriter.write(root);
    
    // Write request to the server
    socket.write((void *) message.data(), message.size());
    
    // Read response
    message.clear();
    while(message.back() != '\n')
    {
        // Attempt to read from the socket
        char c = 0;
        if(!socket.read((void *) &c, 1))
        {
            // Something went wrong, exit
            break;
        }
        message.push_back(c);
    }
    
    // Herp derp
    std::cout << "Received: " << message << std::endl;

    // finished
    return 0;
}
