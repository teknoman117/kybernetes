// system deps
#include <iostream>

// kybernetes deps
#include <kybernetes/network/serversocket.hpp>
#include <jsoncpp/json/json.h>

// Boost
#include <boost/thread/thread.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// Test connection acceptor thread
void connection_thread(kybernetes::network::Socket *client)
{
    // Message buffer
    std::string message;
    bool valid = true;
    
    // Utilities
    Json::FastWriter jsonWriter;
    
    // Loop while valid
    while(valid)
    {
        // Loop while we don't have the complete message
        message.clear();
        while(message.back() != '\n')
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
        if(!valid) break;
        
        // 
    }
    
    // Close connection to client
    delete client;
}

// main
int main (int argc, char** argv)
{
    // Open a server socket
    kybernetes::network::ServerSocket server;
    server.startListening(8080, 5);
    
    // Main loop
    kybernetes::network::Socket *socket;
    while(1)
    {
        // Accept a connection and create a handler thread
        socket = new kybernetes::network::Socket();
        while(!server.acceptConnection(*socket)) continue;
        boost::thread t(connection_thread, socket);
    }

    // finished
    return 0;
}
