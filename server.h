#pragma once    
#include <sys/select.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
using namespace std;
class Server {
public:
    Server(int port,const string ip_str):port(port),ip_str(ip_str){};
    Server(){};
    bool connect_server(int port,string ip_str);
    bool send_msg(string massage);
    bool close_server();
    ~Server(){};

private:
    int port;
    int sockfd;
    string ip_str;
    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);
    
};
