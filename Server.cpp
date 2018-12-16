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

#define HEADER_VALUE 0x0BADF00D

typedef struct
{
   unsigned int header;
   unsigned int cmd;
} ClientCommand;

extern MeeseeksProperties g_mp;
extern int                g_targetX, g_targetY;
extern unsigned int       g_frameCount;
extern double             g_startTime;
extern pthread_mutex_t    g_lock;
extern unsigned char      g_frameBuffer[];
extern unsigned int       g_frameBufferSize;
extern std::atomic<bool>  g_bNewFrame, g_bStopThreads;

double GetTimeStamp();

unsigned int   g_totalByteCount       = 0;
unsigned int   g_serverFrameCount     = 0;
unsigned int   g_sendFrameSize        = 0;
unsigned int   g_header               = HEADER_VALUE;
double         g_serverFrameStartTime = 0.0;
unsigned char  g_sendFrame[640 * 360 * 3];

void IncrementTotalByteCount(unsigned int count)
{
  g_totalByteCount += count; 
}

int SendData(int conn, const void *pData, int sizeofData)
{
  IncrementTotalByteCount(sizeofData);

  int writeStatus = write(conn, pData, sizeofData);
}

int GetData(int conn, void *pData, int sizeofData)
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

bool GetVideoFrame(int conn)
{
  bool bOK = false;

  if (SendData(conn, (const void *) &g_header, sizeof(g_header)) != -1) {
    //printf("Sent header\n"); 
    for (int retry = 0; retry<1000; retry++) {
      if (!g_bNewFrame)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));      
      else {
        //printf("Copying %d bytes from 0x%08X to 0x%08X\n", g_frameBufferSize, g_frameBuffer, g_sendFrame);
        pthread_mutex_lock(&g_lock);
        memcpy(g_sendFrame, g_frameBuffer, g_frameBufferSize);
        g_sendFrameSize = g_frameBufferSize;
        pthread_mutex_unlock(&g_lock);              
        g_bNewFrame = false;
        if (SendData(conn, &g_sendFrameSize, sizeof(g_sendFrameSize)) != -1) {
          //printf("Sent frame buffer size %d\n", g_sendFrameSize);
          if (SendData(conn, g_sendFrame, g_sendFrameSize) != -1) {
            //printf("Sent %d bytes\n", g_sendFrameSize);
            g_serverFrameCount++;
            bOK = true; 
          }
        }
        break;
      }
    }
  }
  return bOK;
}

bool GetString(int conn, std::string &str, int maxStringLength = 1024)
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

bool SendString(int conn, std::string &str)
{
  bool bOK = false;

  unsigned int stringLength = str.length();

  if (SendData(conn, &g_header, sizeof(g_header)) != -1) {    
    if (SendData(conn, &stringLength, sizeof(stringLength)) != -1)
      bOK = (SendData(conn, str.c_str(), stringLength) != -1);
  }

  return bOK;
}

bool GetFileName(int conn, std::string &fileName)
{
  return (GetString(conn, fileName) != -1);
}

bool ReadParamsFromFile(int conn)
{
  std::string fileName;

  if (!GetFileName(conn, fileName)) 
    return false;

  std::string pathAndFile;

  pathAndFile = "params/" + fileName;
  
  g_mp.LoadFromFile(pathAndFile.c_str());

  return true;
}

bool SaveParamsToFile(int conn)
{
  std::string fileName;

  if (!GetFileName(conn, fileName)) 
    return false;

  std::string pathAndFile;

  pathAndFile = "params/" + fileName;    
  
  g_mp.SaveToFile(pathAndFile.c_str());  

  return true;
}

bool SetParams(int conn)
{
  std::string params;
  
  if (!GetString(conn, params, 8192))
    return false;

  g_mp.LoadFromString(params);
  return true;
}

bool SetProperty(int conn)
{
  std::string nameValue;
  
  if (!GetString(conn, nameValue, 8192))
    return false;

  g_mp.SetProperty(nameValue);
  
  return true;
}

double GetCameraFPS()
{
  double timeNow   = GetTimeStamp();
	double deltaTime = timeNow - g_startTime;
	double fps       = ((double) g_frameCount) / deltaTime;

  g_frameCount = 0;
  g_startTime = timeNow;

  return fps;
}

void GetVideoFPSAndBandwidth(double &fps, double &bandwidth)
{
  double timeNow   = GetTimeStamp();
	double deltaTime = timeNow - g_serverFrameStartTime;

	fps       = ((double) g_serverFrameCount) / deltaTime;
  bandwidth = g_totalByteCount / deltaTime;
  bandwidth *= 8;
  bandwidth /= 1000000.0f;

  g_serverFrameCount     = 0;
  g_totalByteCount       = 0;
  g_serverFrameStartTime = timeNow;
}

void StatsToString(std::string &statString)
{
  double videoFPS, videoBandwidth;
  GetVideoFPSAndBandwidth(videoFPS, videoBandwidth);

  std::stringstream str;

  str << "Camera Frame Rate : " << GetCameraFPS() << "\n";
  str << "Video Frame Rate  : " << videoFPS       << "\n";
  str << "Video Bandwidth   : " << videoBandwidth << "\n";
  str << "Target (X, Y)     : " << g_targetX << ", " << g_targetY << "\n";

  int offsetX = (320 - g_targetX);
  int offsetY = (180 - g_targetY);

  str << "Offset (X, Y)     : " << offsetX << ", " << offsetY << "\n";
  
  statString = str.str();
}

bool GetStats(int conn)
{
  std::string stats;

  StatsToString(stats);

  return SendString(conn, stats);
}

bool GetParamFiles(int conn)
{
  bool bOK = false;

  int len;
  struct dirent *pDirent;
  DIR *pDir = opendir("params");
  
  if (pDir == NULL) {
    printf ("Cannot open directory params");
    return true;
  }

  std::string files;

  while ((pDirent = readdir(pDir)) != NULL) {
    if ((strcmp(pDirent->d_name, ".") != 0) && (strcmp(pDirent->d_name, "..") != 0)) {
      if (files.length() != 0)
        files += "\n";
      files += pDirent->d_name;
    }
  }
  
  closedir(pDir);

  return SendString(conn, files);
}

bool GetParams(int conn)
{
  std::string params;
  
  g_mp.SaveToString(params);
  
  return SendString(conn, params);
}

void Close(int sock)
{
   close(sock);
}

void ProcessConnection(int conn)
{
  g_serverFrameCount = 0;
  g_serverFrameStartTime = GetTimeStamp();

  bool bOK = true;

  while (bOK) {
    bOK = false;
    ClientCommand cc;

    if (GetData(conn, &cc, sizeof(cc)) != -1) {  
      if (cc.header == HEADER_VALUE) { 
          switch (cc.cmd) {
            case 0: bOK = GetVideoFrame(conn);        break;
            case 1: bOK = ReadParamsFromFile(conn);   break;
            case 2: bOK = SaveParamsToFile(conn);     break;
            case 3: bOK = SetParams(conn);            break;
            case 4: bOK = GetParams(conn);            break;
            case 5: bOK = GetParamFiles(conn);        break;
            case 6: bOK = GetStats(conn);             break;
            case 7: bOK = SetProperty(conn);          break;
          }
      }
    }  
    if (g_bStopThreads)
      break;
  }
}

void ServerThread()
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
  address.sin_port = htons(g_mp.serverPort);

  printf("Listening on port %d\n", g_mp.serverPort);

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
        printf("Error when accepting connection");      
    } 
    else {    
      printf("Connected\n");

      ProcessConnection(connection);  

      printf("Closing connection, camera FPS %.6f\n", GetCameraFPS());     
      Close(connection);
    }
  }
  Close(sock);

  printf("Socket thread terminated\n");
}