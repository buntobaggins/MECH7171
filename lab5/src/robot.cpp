#include <string>
#include <vector>
#include <windows.h>
#include "robot.h"
#include "constants.h"
#pragma warning (disable : 4290)

using namespace std;
using namespace openutils;

extern "C"
{
   CRobot robot;
   int sendRobotCommand(const char *strMessage)
   {
      return robot.Send(strMessage);
   }

   void closeRobot()
   {
      robot.Close();
   }

   int initializeRobot()
   {
      return robot.Initialize();
   }

   SCARA_STATE getRobotState()
   {
      return robot.getSCARAState();
   }
}


void CWinSock::Initialize()
{
   WORD ver = MAKEWORD(1, 1);
   WSADATA wsadata;
   WSAStartup(ver, &wsadata);
}


void CWinSock::Finalize()
{
   WSACleanup();
}

CServerSocket::CServerSocket()
{
   m_nPort = 80;
   m_nQueue = 10;
   Init();
}

CServerSocket::CServerSocket(int port)
{
   m_nPort = port;
   m_nQueue = 10;
   Init();
}

CServerSocket::CServerSocket(int port, int queue)
{
   m_nPort = port;
   m_nQueue = queue;
   Init();
}

/**
* Binds the server to the given address.
*/
void CServerSocket::Bind(CSocketAddress *sock_addr)
{
   m_bBound = false;
   Close();
   m_sockAddr = sock_addr;
   Accept();
}

/**
* Listens and accepts a client.Returns the accepted connection.
*/
CRobot *CServerSocket::Accept() throw (CSocketException)
{
   if(m_sockAddr != NULL)
      m_sockAddrIn = m_sockAddr->GetSockAddrIn();
   if(!m_bBound)
   {
      m_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
      int nret = bind(m_socket, (LPSOCKADDR)&m_sockAddrIn, sizeof(struct sockaddr));
      if(nret == SOCKET_ERROR)
      {
         nret = WSAGetLastError();
         throw CSocketException(nret, "Failed to bind: Accept()");
      }
      m_bBound = true;
   }
   int nret = listen(m_socket, m_nQueue);
   if(nret == SOCKET_ERROR)
   {
      nret = WSAGetLastError();
      throw CSocketException(nret, "Failed to listen: Accept()");
   }
   SOCKET theClient;
   SOCKADDR_IN clientAddr;
   int ssz = sizeof(struct sockaddr);
   theClient = accept(m_socket, (LPSOCKADDR)&clientAddr, &ssz);
   //theClient = accept(m_socket,NULL,NULL);
   if(theClient == INVALID_SOCKET)
   {
      int nret2 = WSAGetLastError();
      throw CSocketException(nret2, "Invalid client socket: Accept()");
   }
   CRobot *sockClient = new CRobot();
   sockClient->SetSocket(theClient);
   sockClient->SetClientAddr(clientAddr);
   return sockClient;
}

void CServerSocket::Close()
{
   closesocket(m_socket);
   m_sockAddr = NULL;
   m_bBound = false;
   m_bListening = false;
}

void CServerSocket::Init()
{
   m_sockAddrIn.sin_family = AF_INET;
   m_sockAddrIn.sin_addr.s_addr = INADDR_ANY;
   m_sockAddrIn.sin_port = htons((u_short)m_nPort);

   m_sockAddr = NULL; // bind the same machine
   m_bBound = false;
   m_bListening = true;
}

CServerSocket::~CServerSocket()
{
   Close();
}

void CServerSocket::SetPort(int port)
{
   m_nPort = port;
   Init();
}

/**
* Sets the queue size
* @param port Value of port
*/
void CServerSocket::SetQueue(int q)
{
   m_nQueue = q;
}

int CServerSocket::GetPort()
{
   return m_nPort;
}

int CServerSocket::GetQueue()
{
   return m_nQueue;
}

CSocketAddress *CServerSocket::GetSocketAddress()
{
   return m_sockAddr;
}

bool CServerSocket::IsListening()
{
   return m_bListening;
}

// class CRobot

CRobot::CRobot()
{
   m_clientAddr = NULL;

   // initialize the state to the bootup state of the robot
   m_state.jointAngles.theta1Deg = m_state.jointAngles.theta2Deg = 0.0;
   m_state.motorSpeed = MOTOR_SPEED_MEDIUM;
   m_state.penColor = {255, 0 ,0}; // red
   m_state.penPos = PEN_DOWN;

   m_bCyclePenColors = false;
   m_cyclePenColorIndex = 0;
   m_cyclePenColor[0] = RGB(255, 0, 0);
   m_cyclePenColor[1] = RGB(0, 255, 0);
   m_cyclePenColor[2] = RGB(0, 0, 255);
   m_cyclePenColor[3] = RGB(255, 255, 0);
   m_cyclePenColor[4] = RGB(255, 0, 255);
   m_cyclePenColor[5] = RGB(0, 255, 255);
   m_cyclePenColor[6] = RGB(128, 0, 0);
   m_cyclePenColor[7] = RGB(0, 128, 0);
   m_cyclePenColor[8] = RGB(0, 0, 128);
   m_cyclePenColor[9] = RGB(128, 128, 0);
   m_cyclePenColor[10] = RGB(128, 0, 128);
   m_cyclePenColor[11] = RGB(0, 128, 128);
   m_cyclePenColor[12] = RGB(0, 0, 0);
}

void CRobot::SetSocket(SOCKET sock)
{
   m_socket = sock;
}

/**
* Sets address details
* @param addr SOCKADDR_IN
*/
void CRobot::SetClientAddr(SOCKADDR_IN addr)
{
   if(m_clientAddr != NULL) delete m_clientAddr;
   m_clientAddr = new CSocketAddress(addr);
}

int CRobot::Connect()
{
   if(m_clientAddr == NULL)
   {
      printf("Cannot connect to NULL host");
   }
   Connect(m_clientAddr->GetName(), m_clientAddr->GetPort());
   return 1;
}

/**
* Connects to a server
* @param host_name Server name
* @param port Port to connect
*/
int CRobot::Connect(const char *host_name, int port)
{
   int nret;
   LPHOSTENT hostEntry;
   hostEntry = gethostbyname(host_name);
   if(!hostEntry)
   {
      nret = WSAGetLastError();
      printf("Failed to resolve host");
      return 0;
   }

   m_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
   if(m_socket == INVALID_SOCKET)
   {
      nret = WSAGetLastError();
      printf("Failed to create client socket");
      return 0;
   }

   SOCKADDR_IN serverInfo;
   serverInfo.sin_family = AF_INET;
   serverInfo.sin_addr = *((LPIN_ADDR)*hostEntry->h_addr_list);
   serverInfo.sin_port = htons((u_short)port);
   nret = connect(m_socket, (LPSOCKADDR)&serverInfo, sizeof(struct sockaddr));
   if(nret == SOCKET_ERROR)
   {
      nret = WSAGetLastError();
      printf("Connect failed.");
      return 0;
   }

   return 1;
}

/**
* Writes data to the socket. Returns number of bytes written
* @param data data to write
*/
int CRobot::Send(const char *data) throw (CSocketException)
{
   int len, nret = 0, nSent, nTotalSent = 0;

   len = (int)strlen(data);

   while(nTotalSent < len)
   {
      nSent = send(m_socket, data + nTotalSent, len - nTotalSent, 0);
      if(nSent == SOCKET_ERROR)
      {
         nret = WSAGetLastError();
         throw CSocketException(nret, "Network failure: Send()");
      }
      else
      {
         nTotalSent += nSent;
      }
   }
   Sleep(200);

   setSCARAState(data);

   return nret;
}

/*
* Reads data from the socket.Returns number of bytes actually read.
* @param buffer Data buffer
* @param len Number of bytes to read
*/
int CRobot::Read(char *buffer, int len) throw (CSocketException)
{
   int nret = 0;
   nret = recv(m_socket, buffer, len, 0);
   if(nret == SOCKET_ERROR)
   {
      nret = WSAGetLastError();
      throw CSocketException(nret, "Network failure: Read()");
   }
   buffer[nret] = '\0';
   return nret;
}

void CRobot::Close()
{
   closesocket(m_socket);
   if(m_clientAddr != NULL) delete m_clientAddr;
   CWinSock::Finalize();
}

int CRobot::Initialize()
{
   int nret;                  // for integer return values
   system("cls");
   printf("Connecting to %s through port %d...\n", IPV4_STRING, PORT);

   // initializes winsock
   CWinSock::Initialize();
   nret = Connect(IPV4_STRING, PORT);
   if(nret == 0)
   {
      printf("\n\nSimulator must be started and placed in\n");
      printf("remote mode before running this program.\n\n");
      printf("Press ENTER to close program...");
      (void)getchar();
      return FALSE;
   }
   system("cls");
   return TRUE;
}

SCARA_STATE CRobot::getSCARAState()
{
   return m_state;
}

void CRobot::setSCARAState(const char *strCommandString)
{
   size_t i;
   size_t size = strlen(strCommandString);

   char *strTmp = (char *)malloc(size * sizeof(char));
   if(strTmp == NULL)
   {
      printf("strTmp == NULL (CRobot::setSCARAState)\n");
      return;
   }
   for(i = 0; i < strlen(strCommandString); i++)
   {
      strTmp[i] = (char)toupper(strCommandString[i]);
      if(strTmp[i] == '\n') strTmp[i] = '\0';
   }

   if(strcmp(strTmp,"PEN_UP") == 0)
   {
      m_state.penPos = PEN_UP;
   }
   else if(strcmp(strTmp, "PEN_DOWN") == 0)
   {
      m_state.penPos = PEN_DOWN;
   }
   else if(strstr(strTmp, "PEN_COLOR") != NULL && strstr(strTmp, "CYCLE") == NULL)
   {
      const char *seps = "\n\r\t ";
      char *tok, *nextTok = NULL;
      int cc[3] = {0};  // cc = color components

      tok = strtok_s(strTmp, seps, &nextTok); // get rid of "PEN_COLOR"
      for(i = 0; i < 3; i++)  // get components
      {
         tok = strtok_s(NULL, seps, &nextTok);
         if(tok == NULL)
         {
            free(strTmp);
            return;  // color component missing
         }
         cc[i] = atoi(tok);
      }
      m_state.penColor.r = cc[0];
      m_state.penColor.g = cc[1];
      m_state.penColor.b = cc[2];
   }
   else if(strstr(strTmp, "MOTOR_SPEED") != NULL)
   {
      const char *seps = "\n\r\t ";
      char *tok, *nextTok = NULL;

      tok = strtok_s(strTmp, seps, &nextTok); // get rid of "MOTOR_SPEED"
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok == NULL)
      {
         free(strTmp);
         return;  // motor speed value missing
      }
      if(strcmp(tok, "HIGH") == 0)
         m_state.motorSpeed = MOTOR_SPEED_HIGH;
      else if(strcmp(tok, "MEDIUM") == 0)
         m_state.motorSpeed = MOTOR_SPEED_MEDIUM;
      else if(strcmp(tok, "LOW") == 0)
         m_state.motorSpeed = MOTOR_SPEED_LOW;
   }
   else if(strstr(strTmp, "ROTATE_JOINT") != NULL)
   {
      const char *seps = "\n\r\t ";
      char *tok[4], *nextTok = NULL;

      tok[0] = strtok_s(strTmp, seps, &nextTok); // get rid of "ROTATE_JOINT"
      for(i = 0; i < 4; i++)
      {
         tok[i] = strtok_s(NULL, seps, &nextTok);

         if(tok[i] == NULL)
         {
            free(strTmp);
            return;  // missing parameter
         }
      }

      if(strcmp(tok[0], "ANG1") != 0 || strcmp(tok[2], "ANG2") != 0)
      {
         free(strTmp);
         return;  // invalid parameter
      }

      m_state.jointAngles.theta1Deg = atof(tok[1]);
      m_state.jointAngles.theta2Deg = atof(tok[3]);

      if(m_bCyclePenColors && m_state.penPos == PEN_DOWN)
      {
         m_cyclePenColorIndex = (++m_cyclePenColorIndex) % NUM_PEN_COLORS;
         m_state.penColor.r = GetRValue(m_cyclePenColor[m_cyclePenColorIndex]);
         m_state.penColor.g = GetGValue(m_cyclePenColor[m_cyclePenColorIndex]);
         m_state.penColor.b = GetBValue(m_cyclePenColor[m_cyclePenColorIndex]);
      }
   }
   else if(strstr(strTmp, "CYCLE_PEN_COLORS") != NULL)
   {
      const char *seps = "\n\r\t ";
      char *tok, *nextTok = NULL;

      tok = strtok_s(strTmp, seps, &nextTok); // get rid of "CYCLE_PEN_COLORS"
      tok = strtok_s(NULL, seps, &nextTok);
      if(tok == NULL)
      {
         free(strTmp);
         return;  // motor speed value missing
      }
      if(strcmp(tok, "ON") == 0)
      {
         m_bCyclePenColors = true;
      }
      else if(strcmp(tok, "OFF") == 0)
      {
         m_bCyclePenColors = false;
      }
   }
   else if(strcmp(strTmp, "HOME") == 0)
   {
      m_state.jointAngles.theta1Deg = 0.0;
      m_state.jointAngles.theta2Deg = 0.0;
   }

   free(strTmp);
}

CRobot::~CRobot()
{
   Close();
}

// CSocketAddress

CSocketAddress::CSocketAddress(const char *host, int port)
{
   m_sockAddrIn.sin_family = AF_INET;
   m_sockAddrIn.sin_addr.s_addr = INADDR_ANY; // initialized only in GetSockAddrIn()
   m_sockAddrIn.sin_port = htons((u_short)port);
   m_strHostName = host;
   m_nPort = port;
}

CSocketAddress::CSocketAddress(SOCKADDR_IN sockAddr)
{
   m_sockAddrIn.sin_family = sockAddr.sin_family;
   m_sockAddrIn.sin_addr.s_addr = sockAddr.sin_addr.s_addr;
   m_sockAddrIn.sin_port = sockAddr.sin_port;
   m_strHostName = inet_ntoa(m_sockAddrIn.sin_addr);
   m_nPort = sockAddr.sin_port;;
}

const char *CSocketAddress::GetIP()
{
   return (const char *)inet_ntoa(m_sockAddrIn.sin_addr);
}

const char *CSocketAddress::GetName()
{
   HOSTENT *lpHost = gethostbyname(GetIP());
   if(lpHost == NULL) return NULL;
   return lpHost->h_name;
}

void CSocketAddress::GetAliases(vector<string> *ret)
{
   HOSTENT *lpHost = gethostbyname(GetIP());
   if(lpHost == NULL) return;
   char **tmp = (char **)lpHost->h_aliases;
   if(tmp == NULL)
      return;
   else
   {
      int i = 0;
      while(true)
      {
         if(tmp[i] == NULL) break;
         else ret->push_back(tmp[i]);
      }
   }
}

/**
* Returns the sockaddr_in. tries to bind with the server.
* throws CSocketException on failure.
*/
SOCKADDR_IN CSocketAddress::GetSockAddrIn() throw (CSocketException)
{
   m_lpHostEnt = gethostbyname(m_strHostName.c_str());
   if(!m_lpHostEnt)
   {
      int nret = WSAGetLastError();
      throw CSocketException(nret, "Failed to resolve host:gethostbyname()");
   }
   m_sockAddrIn.sin_addr = *((LPIN_ADDR)*m_lpHostEnt->h_addr_list);
   return m_sockAddrIn;
}

void CSocketAddress::operator = (CSocketAddress addr)
{
   m_sockAddrIn = addr.m_sockAddrIn;
   m_strHostName = addr.m_strHostName;
   m_nPort = addr.m_nPort;
   m_lpHostEnt = addr.m_lpHostEnt;
}

CSocketAddress::~CSocketAddress()
{
}
