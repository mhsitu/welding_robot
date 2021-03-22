#pragma once

#include <vector>
#include <string>

#ifdef _WIN32
    #include <winsock2.h>
    #include <Windows.h>
    #include <process.h>
    #ifndef QT_COMPIL
        #pragma message("Adding library: Winmm.lib")
        #pragma comment(lib,"Winmm.lib")
        #pragma message("Adding library: Ws2_32.lib")
        #pragma comment(lib,"Ws2_32.lib")
    #endif
    typedef timeval             _timeval;
    typedef int                 _socklen;
#elif defined (__linux) || defined (__APPLE__)
    #include <string.h>
    #include <sys/time.h>
    #include <unistd.h>
    #include <sys/socket.h>
    #include <arpa/inet.h>
    #include <netdb.h>
    #define SOCKET int
    #define INVALID_SOCKET (-1)
    typedef socklen_t _socklen;
#endif


class CSocketInConnection
{
public:
    CSocketInConnection(int theConnectionPort,unsigned short maxPacketSize=250,char headerID1=59,char headerID2=57);
    virtual ~CSocketInConnection();

    bool connectToClient();
    char* receiveData(int& dataSize);
    bool replyToReceivedData(char* data,int dataSize);

    std::string getConnectedMachineIP();

protected:
    bool _sendSimplePacket(char* packet,int packetLength,unsigned short packetsLeft);
    int _receiveSimplePacket(std::vector<char>& packet);

    unsigned int _getTimeInMs();
    unsigned int _getTimeDiffInMs(unsigned int lastTime);


    SOCKET _socketServer;
    SOCKET _socketClient;
    struct sockaddr_in _socketLocal;

    fd_set _socketTheSet;
#ifdef _WIN32
    WSADATA _socketWsaData;
#endif /* _WIN32 */

    int _socketConnectionPort;
    bool _socketConnectWasOk;
    std::string _socketConnectedMachineIP;

    char _headerByte1;
    char _headerByte2;
    unsigned short _maxPacketSize;
};
