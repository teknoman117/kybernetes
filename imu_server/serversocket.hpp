/* This object represents a server socket.  It makes encapsulates an entity which listens for connections
 * a port on the server process.  It then returns a BufferedSocket object to represent the transfer
 */

#ifndef _ERF_SERVER_SOCKET_HPP_
#define _ERF_SERVER_SOCKET_HPP_

// Unix includes
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

// Boost

// Internal
#include "socket.hpp"

// MTS namespace
namespace mts
{
    // Networking namespace
    namespace networking
    {
        // Server socket object
        class ServerSocket
        {
        private:
            // Server socket descriptor
            int                m_fd;
            int                m_port;
            struct sockaddr_in m_address;

        public:
            // Simple constructor, just initializes the default state
            ServerSocket();

            // Deconstructor
            ~ServerSocket();

            // Listening functions
            bool startListening(int port, int backlog = 3);   // attempts to listen on a particular port
            void stopListening();                             // stops listening

            // Accept a connection
            bool acceptConnection(mts::networking::Socket &sock);

            // State monitoring
            bool isListening();
        };
    }
}

#endif
