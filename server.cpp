#include "server.h"
using namespace std;

bool Server::connect_server(int port, string ip_str)
{
    // 创建socket
    this->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (this->sockfd == -1)
    {
        return false;
        puts("Failed to create socket");
    }
    memset(&this->addr, 0, sizeof(this->addr));
    this->addr.sin_family = AF_INET;   // Use IPV4
    this->addr.sin_port = htons(port); //
    this->addr.sin_addr.s_addr = inet_addr(ip_str);
    return true;
}
bool Server::send_msg(string message)
{
    int len = strlen(message);
    sendto(this->sockfd, message, len, 0, (struct sockaddr *)&this->addr, sizeof(this->addr));
    std::cout << "send" << std::endl;
}
bool Server::close_server()
{
    close(this->sockfd);
}