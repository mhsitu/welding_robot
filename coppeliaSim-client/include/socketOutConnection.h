#pragma once

#include <vector>
#include <string>

#ifdef _WIN32
    #include <Windows.h>
    #include <process.h>
    #ifndef QT_COMPIL
        #pragma message("Adding library: Winmm.lib")
        #pragma comment(lib,"Winmm.lib")
        #pragma message("Adding library: Ws2_32.lib")
        #pragma comment(lib,"Ws2_32.lib")
    #endif
#elif defined (__linux) || defined (__APPLE__)
    #include <sys/time.h>
    #include <unistd.h>
    #include <sys/socket.h>
    #include <arpa/inet.h>
    #include <netdb.h>
    #define SOCKET int
    #define INVALID_SOCKET (-1)
#endif

class CSocketOutConnection  
{
public:
    CSocketOutConnection(const char* theConnectionAddress,int theConnectionPort,unsigned short maxPacketSize=250,char headerID1=59,char headerID2=57);
    virtual ~CSocketOutConnection();

    int connectToServer();
    bool sendData(const char* data,int dataSize);
    char* receiveReplyData(int& dataSize);

protected:
    bool _sendSimplePacket(const char* packet,int packetLength,unsigned short packetsLeft);
    int _receiveSimplePacket(std::vector<char>& packet);

    int _getTimeInMs();
    int _getTimeDiffInMs(int lastTime);

    std::string         _socketConnectionAddress;
    int                 _socketConnectionPort;

#ifdef _WIN32
    WSADATA _socketWsaData;
#else

#endif
    SOCKET _socketConn;
    struct sockaddr_in _socketServer;

    char _headerByte1;
    char _headerByte2;
    unsigned short _maxPacketSize;
};
