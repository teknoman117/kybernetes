// Implementation of the buffered server socket

#include "serversocket.hpp"
#include <iostream>

using namespace mts::networking;

ServerSocket::ServerSocket()
{
    // Initialize internal state
    m_port = -1;
    m_fd = -1;
    memset((char *)&m_address, 0, sizeof(m_address));
}

ServerSocket::~ServerSocket()
{
    stopListening();
}

// Start listening on a port
bool ServerSocket::startListening(int port, int backlog)
{
    // Check if we are connected
    if(m_fd >= 0) return false;

    // Create the socket we are going to use to listen
    if((m_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        std::cerr << "[ERROR] Can't open socket" << std::endl;
        m_fd = -1;
        return false;
    }

    // Bind the socket to this server's settings
    memset((char *)&m_address, 0, sizeof(m_address));
    m_address.sin_family = AF_INET;
    m_address.sin_addr.s_addr = INADDR_ANY;
    m_address.sin_port = htons((m_port = port));
    if(bind(m_fd, (struct sockaddr *)&m_address, sizeof(m_address)) < 0)
    {
        std::cerr << "[ERROR] Can't bind to socket" << std::endl;
        close(m_fd);
        m_fd = -1;
        m_port = -1;
        return false;
    }

    // Set the receive timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // timeout is set to 1/10 sec (100,000 microseconds)
    setsockopt(m_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));

    // Start listening
    if(listen(m_fd, backlog) < 0)
    {
        std::cerr << "[ERROR] Can't start listening" << std::endl;
        close(m_fd);
        m_fd = -1;
        m_port = -1;
        return false;
    }

    // Return success
    return true;
}

// Stop listening
void ServerSocket::stopListening()
{
    close(m_fd);
    m_fd = -1;
    m_port = -1;
}

// Accept a connection
bool ServerSocket::acceptConnection(Socket& sock)
{
    // Check if we are connected
    if(m_fd < 0) return false;

    // Accept the connection
    socklen_t clilen = sizeof(sock.m_address);
    if((sock.m_fd = accept(m_fd, (struct sockaddr *)&sock.m_address, &clilen)) < 0)
    {
        //std::cerr << "[ERROR] Can't establish connection" << std::endl;
        sock.m_fd = -1;
        return false;
    }

    // Copy the port handler and start the buffer
    sock.m_port = m_port;

    // Set the timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // timeout is set to 1/10 sec (100,000 microseconds)
    setsockopt(sock.m_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));

    // Return success
    return true;
}

// Return if we are successfully listening
bool ServerSocket::isListening()
{
    return (m_fd >= 0);
}
