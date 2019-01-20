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
#include "ControlServer.h"

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

ControlServer::ControlServer()
{
  m_name = "Control Server";  
}

bool ControlServer::GetVideoFrame(int conn)
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

bool ControlServer::GetFileName(int conn, std::string &fileName)
{
  return (GetString(conn, fileName) != -1);
}

bool ControlServer::ReadParamsFromFile(int conn)
{
  std::string fileName;

  if (!GetFileName(conn, fileName)) 
    return false;

  std::string pathAndFile;

  pathAndFile = "params/" + fileName;
  
  g_mp.LoadFromFile(pathAndFile.c_str());

  return true;
}

bool ControlServer::SaveParamsToFile(int conn)
{
  std::string fileName;

  if (!GetFileName(conn, fileName)) 
    return false;

  std::string pathAndFile;

  pathAndFile = "params/" + fileName;    
  
  g_mp.SaveToFile(pathAndFile.c_str());  

  return true;
}

bool ControlServer::SetParams(int conn)
{
  std::string params;
  
  if (!GetString(conn, params, 8192))
    return false;

  g_mp.LoadFromString(params);
  return true;
}

bool ControlServer::SetProperty(int conn)
{
  std::string nameValue;
  
  if (!GetString(conn, nameValue, 8192))
    return false;

  g_mp.SetProperty(nameValue);
  
  return true;
}

double ControlServer::GetCameraFPS()
{
  double timeNow   = GetTimeStamp();
  double deltaTime = timeNow - g_startTime;
  double fps       = ((double) g_frameCount) / deltaTime;

  g_frameCount = 0;
  g_startTime = timeNow;

  return fps;
}

void ControlServer::GetVideoFPSAndBandwidth(double &fps, double &bandwidth)
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

void ControlServer::StatsToString(std::string &statString)
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

bool ControlServer::GetStats(int conn)
{
  std::string stats;

  StatsToString(stats);

  return SendString(conn, stats);
}

bool ControlServer::GetParamFiles(int conn)
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

bool ControlServer::GetParams(int conn)
{
  std::string params;
  
  g_mp.SaveToString(params);
  
  return SendString(conn, params);
}

void ControlServer::ProcessConnection(int conn)
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

  printf("Closing connection, camera FPS %.6f\n", GetCameraFPS());     
}