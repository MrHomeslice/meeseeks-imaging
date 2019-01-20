#include <sys/ioctl.h>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <memory>
#include <string>
#include <atomic>

#include "MeeseeksProperties.h"
#include "networktables/NetworkTable.h"
#include "ControlServer.h"
#include "SimulationServer.h"

MeeseeksProperties g_mp;
pthread_mutex_t    g_lock, g_simFrameLock;
std::atomic<bool>  g_bStopThreads, g_bSimActive;

std::shared_ptr<NetworkTable> g_targetTable; 

void ServerThread();
void CameraThread();

int _kbhit()
{
	static const int STDIN = 0;
	static bool initialized = false;

	if (!initialized) {
		termios term;
		tcgetattr(STDIN, &term);
		term.c_lflag &= ~ICANON;
		tcsetattr(STDIN, TCSANOW, &term);
		setbuf(stdin, NULL);
		initialized = true;
	}

	int bytesWaiting;
	ioctl(STDIN, FIONREAD, &bytesWaiting);
	return bytesWaiting;
}

int main()
{
  g_mp.Initialize();

  pthread_mutex_init(&g_lock, NULL);
  pthread_mutex_init(&g_simFrameLock, NULL);

  g_bStopThreads = false;  
  g_bSimActive   = g_mp.bSimActive;

  NetworkTable::SetClientMode();
  NetworkTable::SetDSClientEnabled(false);

  NetworkTable::SetIPAddress(llvm::StringRef(g_mp.networkTableAddress.c_str()));

  NetworkTable::Initialize();

  g_targetTable = NetworkTable::GetTable("Target");

  //ControlServer controlServer;

  std::thread cameraThread(CameraThread);
  std::thread serverThread(&ControlServer::ServerThread, ControlServer(), g_mp.serverPort);

  std::thread simulationThread;
  if (g_bSimActive)  
    simulationThread = std::thread(&SimulationServer::ServerThread, SimulationServer(), g_mp.simulationPort);  

  do {
    if (_kbhit())
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(250));      
  } while (true);

  g_bStopThreads = true;
    
  cameraThread.join();
  serverThread.join();
  
  if (g_bSimActive)
    simulationThread.join();

  printf("Done\n");
 }