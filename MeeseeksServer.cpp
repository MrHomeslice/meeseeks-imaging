#include <sys/socket.h>
#include <sys/unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <arpa/inet.h>
#include <signal.h>

#include <stdio.h>
#include <string.h>
#include <chrono>
#include <thread>
#include <sstream>
#include <atomic>
#include <fcntl.h>

#include "MeeseeksProperties.h"
#include "MeeseeksServer.h"

extern unsigned int   g_totalByteCount;
extern unsigned int   g_serverFrameCount ;
extern unsigned int   g_sendFrameSize;
extern unsigned int   g_header;
extern double         g_serverFrameStartTime;
extern unsigned char  g_sendFrame;
extern bool           g_bStopThreads;

extern MeeseeksProperties g_mp;

void MeeseeksServer::IncrementTotalByteCount(unsigned int count)
{
  g_totalByteCount += count; 
}

int MeeseeksServer::SendData(int conn, const void *pData, int sizeofData)
{
  IncrementTotalByteCount(sizeofData);

  int writeStatus = write(conn, pData, sizeofData);
}

int MeeseeksServer::GetData(int conn, void *pData, int sizeofData)
{
  IncrementTotalByteCount(sizeofData);

  int totalRead = 0;
  int remaining = sizeofData;

  unsigned char *pReadData = (unsigned char *) pData;

  while (remaining > 0) {
    int readSize = read(conn, pReadData, remaining);

    if (readSize <= 0)
      return -1;  

    totalRead += readSize;
    pReadData += readSize;
    remaining -= readSize;    
  }

  return totalRead;
}

bool MeeseeksServer::GetString(int conn, std::string &str, int maxStringLength)
{
  bool bOK = false;

  if (SendData(conn, &g_header, sizeof(g_header)) != -1) {
    unsigned int stringLength = 0;
    if (GetData(conn, &stringLength, sizeof(stringLength)) != -1) {
      if (stringLength < maxStringLength) {
        char *pStr = new char[stringLength + 1];
        pStr[stringLength] = 0;
        if (GetData(conn, pStr, stringLength) != -1) {
          str = pStr;
          bOK = true;
        }
        delete []pStr;
      }
    }
  }
  return bOK;
}

bool MeeseeksServer::SendString(int conn, std::string &str)
{
  bool bOK = false;

  unsigned int stringLength = str.length();

  if (SendData(conn, &g_header, sizeof(g_header)) != -1) {    
    if (SendData(conn, &stringLength, sizeof(stringLength)) != -1)
      bOK = (SendData(conn, str.c_str(), stringLength) != -1);
  }

  return bOK;
}

void MeeseeksServer::ProcessConnection(int conn)
{
}

void MeeseeksServer::ServerThread(int portNumber)
{
  signal(SIGPIPE, SIG_IGN);

  int sock = socket(AF_INET, SOCK_STREAM, 0);

  if (sock == -1) {
    printf("Error creating socket\n");  
    return;
  }

  int reuseAddr = 1;

  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char *) &reuseAddr, sizeof(reuseAddr)) == -1) {
    printf("Error setting socket options\n");
    return;
  }

  int flags = fcntl(sock, F_GETFL);
  fcntl(sock, F_SETFL, flags | O_NONBLOCK);

  sockaddr_in address, clientAddress;

  memset(&address, 0, sizeof(address));
  
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons(portNumber);

  printf("%s istening on port %d\n", m_name.c_str(), portNumber);

  if (bind(sock, (struct sockaddr*) &address, sizeof(address)) == -1) {
    printf("Cannot bind to socket\n");
    return;
  }

  if (listen(sock, 10) == -1) {
    printf("Cannot listen on socket\n");
    return;
  }

  while (!g_bStopThreads) {
    socklen_t clientAddressLen = sizeof(clientAddress);

    int connection = accept(sock, (struct sockaddr*) &clientAddress, &clientAddressLen);
    
    if (connection == -1) {
      if (errno == EWOULDBLOCK)
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));            
      else
        printf("%s error when accepting connection", m_name.c_str());      
    } 
    else {    
      printf("%s connected on port %d\n", m_name.c_str(), portNumber);

      ProcessConnection(connection);  

      printf("%s disconnected\n", m_name.c_str());

      Close(connection);
    }
  }

  Close(sock);

  printf("%s terminated\n", m_name.c_str());
}

void MeeseeksServer::Close(int conn)
{
  close(conn);  
}