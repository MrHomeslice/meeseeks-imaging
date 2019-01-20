#ifndef _SIMULATION_SERVER_INCLUDED
#define _SIMULATION_SERVER_INCLUDED

#include "MeeseeksServer.h"

class SimulationServer : public MeeseeksServer
{
  public    : SimulationServer();
  protected :
              void ProcessConnection(int conn);  
              bool SetVideoFrame(int conn);

              int m_simFrameCount;
              int            m_simFrameSize;
              unsigned char *m_pSimFrame;
};

#endif