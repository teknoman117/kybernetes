/*
 *  socket.h
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

#include <kybernetes/network/socket.hpp>

#include <iostream>
#include <cstring>

using namespace kybernetes::network;

// Empty constructor, waiting for a connection to a server
Socket::Socket()
{
    // Initialize the connection to an invalid state
    m_server = NULL;
    m_fd = -1;
    m_port = -1;
}

// Copy constructor
Socket::Socket(const Socket& socket)
{
    // Initialize the connection to an invalid state
    m_server = NULL;
    m_fd = -1;
    m_port = -1;

    // Swap with the other socket
    swap(socket);
}

// Standard deconstructor.  Will close connection if its open
Socket::~Socket()
{
    // Make sure we are disconnected
    disconnect();
}

// Connection command
bool Socket::connectTo(std::string host, int port)
{
    // Verify that we are not currently open
    if(m_fd >= 0) return false;

    // Create a UNIX socket
    if((m_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        //std::cerr << "[ERROR] Can't open socket" << std::endl;
        m_fd = -1;
        return false;
    }
    
    // Lookup server by the IP or hostname
    if(!(m_server = gethostbyname(host.c_str())))
    {
        //std::cerr << "[ERROR] Can't lookup host" << std::endl;
        close(m_fd);
        m_server = NULL;
        return false;
    }
   
    // Build the address
    memset((char *) &m_address, 0, sizeof(m_address));
    m_address.sin_family = AF_INET;
    memcpy((char *) &m_address.sin_addr.s_addr, (char *)m_server->h_addr, m_server->h_length);
    m_address.sin_port = htons((m_port = port));
    
    // Connect to the server
    if(connect(m_fd,(struct sockaddr *) &m_address, sizeof(m_address)) < 0) 
    {
        //std::cerr << "[ERROR] Can't connect to server" << std::endl;
        close(m_fd);
        m_fd = -1;
        m_server = NULL;
        return false;
    }

    // Set a timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // timeout is set to 1/10 sec (100,000 microseconds)
    setsockopt(m_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));

    // Since we are now connected, return true
    return true;
}

// Invalidate command
void Socket::invalidate()
{
    // Clean up
    m_server = NULL;
    m_fd = -1;
    m_port = -1;
}

// Disconnect command
void Socket::disconnect()
{
    // If the port is not open, do nothing
    if(m_fd < 0) return;

    // Close the connection
    close(m_fd);

    // Invalidate socket
    m_server = NULL;
    m_fd = -1;
    m_port = -1;
}

void Socket::swap(Socket& socket)
{
    // Make sure we aren't connected to anything
    disconnect();

    // If we want to copy the socket, we only want one instance of the socket running.  Therefore we shutdown the other socket after copying
    m_server = socket.m_server;
    m_fd = socket.m_fd;
    m_port = socket.m_port;
    m_address = socket.m_address;

    // Invalidate the other socket
    socket.invalidate();
}

void Socket::swap(const Socket& socket)
{
    // Make sure we aren't connected to anything
    disconnect();

    // If we want to copy the socket, we only want one instance of the socket running.  Therefore we shutdown the other socket after copying
    m_server = socket.m_server;
    m_fd = socket.m_fd;
    m_port = socket.m_port;
    m_address = socket.m_address;
}

Socket& Socket::operator =(const Socket& rhs)
{
    swap(rhs);
    return *this;
}

// Get current string in the buffer
bool Socket::read(void* message, size_t size, bool blocking)
{
	// Check if the socket is open (did it get shut down for error reasons)
	if(m_fd < 0) return false;

    // Perform the recv operation, handling any sort of major error conditions
    int ret = 0;
    while((ret = recv(m_fd, message, size, MSG_WAITALL)) != 0)
    {
        // If we are non blocking and the connection failed to receive data, break out
        if(ret < 0 && !blocking)
            return false;

        // If we received a valid message, return true
        if(ret == (int) size)
            return true;

        // Interruption point
        //boost::this_thread::interruption_point();
    }

    // If the return code was 0, close the socket down
    if(ret == 0)
    {
        //std::cout << "Disconnecting upon peer disconnect" << std::endl;
        disconnect();
    }

    // Return an error
    return false;
}

// Dump a certain amount of data
bool Socket::dump(size_t size)
{
    // Check if the socket is open (did it get shut down for error reasons)
    if(m_fd < 0) return false;

    // Create a bogus buffer
    std::vector<unsigned char> m_dump(size);
    return read(m_dump.data(), size);
}

// Write a string into the write buffer
bool Socket::write(void* message, size_t size)
{
    // Check if the socket is open (did it get shut down for error reasons)
    if(m_fd < 0) return false;

    // Just a simple write command, although if it fails, send an error response
    if(send(m_fd, message, size, 0) < 0)
    {
        // Check the here *here*
        return false;
    }

    // Return success
    return true;
}

// Check if the socket is connected
bool Socket::isConnected()
{
    return (m_fd >= 0);
}
