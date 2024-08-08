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
#pragma pack(1)
typedef struct tagNET_MSG_HEADER{//报文头10
	unsigned short wSerialNo;	
	unsigned short wMsgID; //报文标识	
	unsigned short wMsgLen;	
	unsigned intiSendTime;//距当日0时的秒数	
	unsigned intiSendIP;// 发送IP	
	unsigned int iDstIP;//目的IP	
}NET_MSG_HEADER,*LPNET_MSG_HEADER;
#pragma pack(1)
struct tagOFC_Track_Receive{
NET_MSG_HEADER header;

int	iBoat;	//艇ID;	     1
unsigned char bTargetOK; //1
unsigned short wBatchNum;//目标批号0为无效//是否稳定跟踪目标。1为跟踪好，0为未跟踪好0X78 I
unsigned char bDistanceOK;	//距离有否有效 1有效0无效   1	
unsigned short sFWAngle;	//东北天方位角0-360(0.01度)	
unsigned short wLaserDis;	//激光目标距离0-8000(1米)	
int wX_F_n;	//X方向距离1米 // 20221007 unsigned int变成int	
int wY_F_n;	//Y方向距离1米	
int wZ_F_n;	//Z方向距离1米	
int wVX_F_n;	//X方向速度0.1米每秒	
int wVY_F_n;	//Y方向速度0.1米每秒	
int wVZ_F_n;	//Z方向速度0.1米每秒	
int	uiLon;	//目标经度分辨率10E-6东经(E)为正，西经(W)为负	必须有 1 
int	uiLat;	//目标纬度分辨率10E-6北纬(N)为正，南纬(S)为负	必须有 1
unsigned char bTypeOK;	//判断识别是否有效1:有效0:无效	

unsigned short usTargetKind;//目标种类:规定0x10对应浮标0x11对应艇

unsigned char bNumberFeatureOK;//数字特征是否有效0无效1有效
unsigned char bSymbolFeatureOK;//图形特征是否有效0无效1有效	
unsigned char bColorFeatureOK; //颜色特征是否有效0无效1有效/20221006 wjs	
unsigned char cNumberFeature;//0-9数字0-9	
unsigned char cSymbolFeature;//1正方形2三角形3五角星4圆形	
unsigned char cColorFeature;//0不清晰1红色2绿色3蓝色	
unsigned long long dwTime;	//1970年至今的毫秒数 1	

};

class Server {
public:
    Server(int port,char* ip_str):port(port),ip_str(ip_str){};
    Server(){};
    bool connect_server();
    void send_msg(char* massage,int len);
    void close_server();
    ~Server(){};

private:
    int port;
    int sockfd;
    char* ip_str;
    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);
    
};
