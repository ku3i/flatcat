#ifndef SOCKET_SERVER_H
#define SOCKET_SERVER_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

namespace network {

    namespace constants {
        const unsigned max_connections = 1;
        const unsigned max_tcp_msg_len = 8192;
        const unsigned timeout_ms = 500;
    }

class Socket_Server
{
    int sockfd;
    int connectfd = -1;
    uint16_t port;
    struct sockaddr_in serv_addr = {};
    std::string recv_stream = "";
    std::string current_client_addr = "";

public:
    Socket_Server(const uint16_t port)
    : sockfd(socket(AF_INET, SOCK_STREAM, 0))
    , port(port)
    {
        if (sockfd < 0)
            err_msg(__FILE__,__LINE__,"Cannot create TCP socket listening to port %u.\n%s", port, strerror(errno));

        memset(&serv_addr, 0, sizeof(serv_addr));          /* clear struct                                     */
        serv_addr.sin_family = AF_INET;                    /* socket address type                              */
        serv_addr.sin_port = htons(port);                  /* set port, convert unsigned to network byte order */
        serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);     /* set IP of own host (INADDR_ANY)                  */
        int flag = 1;                                      /* set TCP_NODELAY flag                             */
        setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));

        /* set timeout */
        struct timeval read_timeout = timeval{};
        read_timeout.tv_usec = constants::timeout_ms * 1000;
        if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout)) < 0)
            err_msg(__FILE__,__LINE__,"Cannot set socket options for read timeout.\n%s", strerror(errno));

        /* bind socket to address and port */
        if (-1 == bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)))
        {
            close(sockfd);
            err_msg(__FILE__,__LINE__,"Failed binding the socket.\nPlease wait until released or try another port.\n");
        }

        if (-1 == listen(sockfd, (int) constants::max_connections))
        {
            close(sockfd);
            err_msg(__FILE__,__LINE__,"Failed listening to socket.");
        }
    }

    ~Socket_Server() {
        close_connection();
        close(sockfd);
    }

    bool open_connection(void)
    {
        /* wait for client connection and accept, if any. */
        struct sockaddr_storage addr;
        socklen_t len = sizeof addr;


        connectfd = accept(sockfd, (struct sockaddr*) &addr, &len);
        if (connectfd < 0) {
            return false;
        }

        char client_addr[INET6_ADDRSTRLEN];
        unsigned port;
        if (addr.ss_family == AF_INET) { /* IPv4 */
            struct sockaddr_in *s = (struct sockaddr_in*) &addr;
            port = ntohs(s->sin_port);
            inet_ntop(AF_INET, &s->sin_addr, client_addr, sizeof client_addr);
        } else { // AF_INET6
            struct sockaddr_in6 *s = (struct sockaddr_in6*) &addr;
            port = ntohs(s->sin6_port);
            inet_ntop(AF_INET6, &s->sin6_addr, client_addr, sizeof client_addr);
        }

        sts_msg("Connection opened to client: %s:%u", client_addr, port);
        current_client_addr = client_addr;
        return true;
    }

    void close_connection(void)
    {
        if (connectfd < 0) return; /* already closed, nothing to do */

        if (shutdown(connectfd, SHUT_RDWR) < 0)
            wrn_msg("Cannot shutdown TCP connection.\n%s", strerror(errno));
        else
            sts_msg("Connection successfully shut down.");

        close(connectfd);
        sts_msg("Connection closed.");
    }

    bool send_message(std::string const& msg) const
    {
        if (write(connectfd, msg.c_str(), msg.length()) < 0) {
            wrn_msg("Failed writing to TCP socket.\n%s", strerror(errno));
            return false;
        }
        return true;
    }

    std::string get_next_msg(void) const
    {
        /* create and clear buffer */
        char buffer[constants::max_tcp_msg_len];
        memset(buffer, 0, constants::max_tcp_msg_len);

        /* read from socket (blocking) */
        if (read(connectfd, buffer, constants::max_tcp_msg_len) < 0)
        {
            if (errno != EAGAIN)
                wrn_msg("Failed reading from socket.\n%s", strerror(errno));
            return "";
        }

        /*if (0 == n)
        {
            printf("Reading no more bytes from socket. Exiting.\n");
            return std::string("EXIT\n");
        }*/

        return std::string(buffer);
    }

    /* return part of the stream until next newline character */
    std::string get_next_line(void)
    {
        std::string::size_type pos;

        while((pos = recv_stream.find("\n", 0)) == std::string::npos)
            recv_stream += get_next_msg();

        std::string retstr = recv_stream.substr(0, pos);
        recv_stream.erase(0, pos+1);

        return retstr;
    }

    std::string const& get_current_client_address(void) const { return current_client_addr; }

};

} /* namespace network */

#endif /* SOCKET_SERVER_H */
