#ifndef _CONTROL_SERVER_INCLUDED
#define _CONTROL_SERVER_INCLUDED

#include "MeeseeksServer.h"

class ControlServer : public MeeseeksServer
{
  public    : ControlServer();
  protected : 
              void ProcessConnection(int conn);  

              bool GetVideoFrame(int conn);
              bool GetFileName(int conn, std::string &fileName);
              bool ReadParamsFromFile(int conn);
              bool SaveParamsToFile(int conn);
              bool SetParams(int conn);
              bool SetProperty(int conn);
              double GetCameraFPS();
              void GetVideoFPSAndBandwidth(double &fps, double &bandwidth);
              void StatsToString(std::string &statString);
              bool GetStats(int conn);
              bool GetParamFiles(int conn);
              bool GetParams(int conn);
};

#endif