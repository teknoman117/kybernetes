/* 
 * This object represent a network socket, but with two distinctly awesome features
 *   1) Its buffered in the background - a thread handles it
 *   2) The buffer is made up of strings, so its awesome for
 *      passing things like JSON
 */
 
#ifndef _STRING_SOCKET_HPP_
#define _STRING_SOCKET_HPP_

// Unix includes
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#include <cstring>
#include <string>
#include <vector>

// ERF namespace
namespace mts
{
    // Networking namespace
    namespace networking
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
            Socket(Socket& socket);

            // Standard de-constructor.  Will disconnection from server if connection is open
            ~Socket();

            // Utilities to change the ownership of sockets
            void        swap(Socket& socket);
            Socket&     operator=(Socket& rhs);


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
