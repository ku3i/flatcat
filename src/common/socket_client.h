/*---------------------------------+
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | July 2017                       |
 +---------------------------------*/

#ifndef SOCKET_CLIENT_H
#define SOCKET_CLIENT_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <common/globalflag.h>
#include <common/log_messages.h>
#include <common/lock.h>

namespace network {

namespace constants {
    const unsigned seconds_us   = 1000*1000;
    const unsigned default_port = 7777;
    const unsigned msglen       = 8192;
}

std::string hostname_to_ip(const char* hostname);

class Socket_Client
{
    Socket_Client( const Socket_Client& other ) = delete;      // non construction-copyable
    Socket_Client& operator=( const Socket_Client& ) = delete; // non copyable

public:
    Socket_Client()
    : srv_addr()
    , server()
    , sockfd(-1)
    , connection_established(false)
    , msgbuf()
    , mtx()
    { sts_msg("Creating client socket."); }

    ~Socket_Client(void) { sts_msg("Destroying client socket."); }

    bool open_connection(const char* server_addr, const unsigned short port);
    void close_connection(void);

    std::string recv(unsigned int time_out_us);
    void send(const char* format, ...); /* sends independent messages immediately */

    void append(const char* format, ...);
    void flush();

private:
    struct sockaddr_in srv_addr;
    struct hostent *server;
    int sockfd;
    bool connection_established;

    std::string msgbuf;
    common::mutex_t mtx;
};

} /* namespace network */

#endif /* SOCKET_CLIENT_H */
