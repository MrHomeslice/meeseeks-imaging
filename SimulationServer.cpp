#include "SimulationServer.h"
#include <atomic>
#include <string.h>

extern bool              g_bStopThreads;
extern unsigned int      g_header;
extern std::atomic<bool> g_bSimActive;
extern pthread_mutex_t   g_simFrameLock;

std::atomic<bool> g_bSimFrameReady;
unsigned int      g_simFrameSize = 0;

unsigned char *g_pSimFrame = NULL;

void SetSimFrame(unsigned char *pFrame, int length)
{
  pthread_mutex_lock(&g_simFrameLock);

  if (length > g_simFrameSize) {
    if (g_pSimFrame != NULL) 
      delete []g_pSimFrame;
    g_pSimFrame = new unsigned char[length];
  }  
  memcpy(g_pSimFrame, pFrame, length);
  g_simFrameSize = length;
  g_bSimFrameReady = true;
  
  pthread_mutex_unlock(&g_simFrameLock);  
}

bool GetSimFrame(char *pSimFrame, int &length)
{
  bool bStatus = false;

  pthread_mutex_lock(&g_simFrameLock);

  if (g_pSimFrame) {
    memcpy(pSimFrame, g_pSimFrame, g_simFrameSize);
    length = g_simFrameSize;
    bStatus = true;
  }  
  else
    length = 0;
  
  pthread_mutex_unlock(&g_simFrameLock);  

  return bStatus;
}

SimulationServer::SimulationServer()
{
  g_bSimFrameReady = false;
  m_simFrameSize = 0;
  m_pSimFrame = NULL;
  m_name = "Simulation Server";
}

bool SimulationServer::SetVideoFrame(int conn)
{ 
  int length;

  if (SendData(conn, (const void *) &g_header, sizeof(g_header)) != -1) {
    if (GetData(conn, &length, sizeof(int)) != -1) {
      if (length > m_simFrameSize) {
         if (m_pSimFrame != NULL)
            delete []m_pSimFrame;        
        m_simFrameSize = length;          
        m_pSimFrame = new unsigned char[m_simFrameSize];
      }
      if (GetData(conn, m_pSimFrame, length) == length) {
        SetSimFrame(m_pSimFrame, length);
        m_simFrameCount++;
        return true;
      }      
    }
  }

  return false;
}

void SimulationServer::ProcessConnection(int conn)
{
  m_simFrameCount = 0;

  bool bOK = true;

  while (bOK) {
    bOK = false;
    ClientCommand cc;

    if (GetData(conn, &cc, sizeof(cc)) == -1)
      break;

    if (cc.header == HEADER_VALUE) { 
      switch (cc.cmd) {
        case 0: bOK = SetVideoFrame(conn); break;
      }    
    } 

    if (g_bStopThreads)
      break;
  }
}