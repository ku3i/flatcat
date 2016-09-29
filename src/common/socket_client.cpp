/* implements a simple socket client for simloid communication */

#include "./socket_client.h"

extern GlobalFlag do_quit;

bool
Socket_Client::open_connection(const unsigned short port)
{
    if (connection_established)
    {
        wrn_msg("Already connected to server.");
        return false;
    }

    /* create socket */
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        wrn_msg("Cannot create socket");
        return false;
    }

    srv_addr.sin_family = AF_INET;
    srv_addr.sin_port = htons(port);
    int result = inet_pton(AF_INET, "127.0.0.1", &srv_addr.sin_addr);

    if (0 > result)
    {
        wrn_msg("First parameter is not a valid address family.");
        close(sockfd);
        return false;
    }
    else if (0 == result)
    {
        wrn_msg("Char string (second parameter does not contain valid IP address)");
        close(sockfd);
        return false;
    }

    if (-1 == connect(sockfd, (struct sockaddr *)&srv_addr, sizeof(srv_addr)))
    {
        wrn_msg("Connect failed");
        close(sockfd);
        return false;
    }

    /* make the socket non-blockable */
    int x = fcntl(sockfd ,F_GETFL, 0);
    fcntl(sockfd, F_SETFL, x | O_NONBLOCK);

    connection_established = true;
    sts_msg("Socket opened.");
    return true;
}

void
Socket_Client::close_connection(void)
{
    if (connection_established)
    {
        shutdown(sockfd, SHUT_RDWR);
        close(sockfd);
        connection_established = false;
        sts_msg("Socket closed.");
    }
    else
        wrn_msg("No connection left to be closed.");
}

void
Socket_Client::send(const char* format, ...) const
{
    static char buffer[MSGLEN];
    bzero(buffer, MSGLEN);

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, MSGLEN, format, args);
    va_end(args);
    if ((unsigned int) write(sockfd, buffer, strlen(buffer)) != strlen(buffer))
        err_msg(__FILE__, __LINE__, "Send incomplete.");
}


std::string
Socket_Client::recv(unsigned int timeout_us = 0)
{
    char buffer[MSGLEN];
    unsigned int time_spent_us = 0;
    unsigned int interval_us = 1;

    int len = read(sockfd, buffer, MSGLEN);

    while ((-1 == len) && (time_spent_us < timeout_us) && !do_quit.status())
    {
        usleep(interval_us);
        time_spent_us += interval_us; // sum up time spent

        if (interval_us < 4096)       // double the interval for next sleep
            interval_us *= 2;

        len = read(sockfd, buffer, MSGLEN);
    }

    if (0 == len) return "";
    else if (0 > len) {
        if (do_quit.status()) wrn_msg("Received signal to exit during read. Cancel reception of data.");
        else  wrn_msg("Connection timed out.");
        return "";
    }

    return std::string(buffer);
}


