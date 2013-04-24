/*
 *  socket.hpp
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
 
#ifndef _NETWORKING_SOCKET_HPP_
#define _NETWORKING_SOCKET_HPP_

// Unix includes
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#include <cstring>
#include <string>
#include <vector>

// Kybernetes namespace
namespace kybernetes
{
    // Networking namespace
    namespace network
    {
        // A buffered socket
        class Socket
        {
            friend class ServerSocket;
        private:
            // Unix socket information
            struct sockaddr_in               m_address;
            struct hostent                  *m_server;
            int                              m_fd;
            int                              m_port;

        public:
            // Empty constructor, basically a connect can be called later to attach to a server
            Socket();

            // Copy constructor - swaps the other socket to this one
            Socket(const Socket& socket);

            // Standard de-constructor.  Will disconnection from server if connection is open
            ~Socket();

            // Utilities to change the ownership of sockets
            void        swap(Socket& socket);
            void        swap(const Socket& socket);
            Socket&     operator=(const Socket& rhs);


            // Connection control commands
            bool        connectTo(std::string host, int port);
            void        invalidate(); // invalidate this socket instance
            void        disconnect();

            // Buffer access controls
            bool        read(void* message, size_t size, bool block = true);
            bool        dump(size_t size);
            bool        write(void* message, size_t size);

            // State access
            bool        isConnected();
        };
    }
}

#endif
