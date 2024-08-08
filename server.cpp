#include "server.h"
using namespace std;

bool Server::connect_server()
{
    // 创建socket
    this->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (this->sockfd == -1)
    {
        return false;
        puts("Failed to create socket");
    }
    memset(&this->addr, 0, sizeof(this->addr));
    this->addr.sin_family = AF_INET;         // Use IPV4
    this->addr.sin_port = htons(this->port); //
    this->addr.sin_addr.s_addr = inet_addr(this->ip_str);
    return true;
}
void Server::send_msg(char *message,int len)
{
    //int len = strlen(message);
    cout<<len<<" "<<this->addr.sin_port<<" "<<this->addr.sin_addr.s_addr<<" "<<this->port<<" "<<this->ip_str<<endl;
    ssize_t bytes_sent = sendto(this->sockfd, message, len, 0, (struct sockaddr *)&this->addr, sizeof(this->addr));
    if (bytes_sent == -1)
    {
        perror("Failed to send message");
    }
    else
    {
        cout << "Sent " << bytes_sent << " bytes" << endl;
    }
}
void Server::close_server()
{
    close(this->sockfd);
    cout << "Server closed" << endl;
}