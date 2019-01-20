#ifndef SRC_MEESEEKS_SERVER_H_
#define SRC_MEESEEKS_SERVER_H_

#include <string>

#define HEADER_VALUE 0x0BADF00D

typedef struct
{
   unsigned int header;
   unsigned int cmd;
} ClientCommand;

class MeeseeksServer 
{
  public    : MeeseeksServer()
              {              
              }

              void ServerThread(int portNumber);
  protected :
              virtual void ProcessConnection(int conn);

              void IncrementTotalByteCount(unsigned int count);
              int  SendData(int conn, const void *pData, int sizeofData);
              int  GetData(int conn, void *pData, int sizeofData);
              bool GetString(int conn, std::string &str, int maxStringLength = 1024);
              bool SendString(int conn, std::string &str);
              void Close(int sock);

              std::string m_name;
};

#endif






