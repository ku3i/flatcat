#ifndef SOCKET_CLIENT_H
#define SOCKET_CLIENT_H

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "./basic.h"
#include "./globalflag.h"
#include "./log_messages.h"

#define DEFAULTPORT 7777
#define MSGLEN 8192

namespace{
    const unsigned int seconds_us = 1000000;
}

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
    { sts_msg("Creating client socket."); }

    ~Socket_Client(void) { sts_msg("Destroying client socket."); }

    bool open_connection(const unsigned short port);
    void close_connection(void);

    std::string recv(unsigned int time_out_us);
    void send(const char* format, ...) const;

private:
    struct sockaddr_in srv_addr;
    struct hostent *server;
    int sockfd;
    bool connection_established;
};


#endif /* SOCKET_CLIENT_H */
