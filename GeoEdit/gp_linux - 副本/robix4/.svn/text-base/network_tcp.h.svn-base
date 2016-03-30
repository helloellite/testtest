#ifndef _NETWORK_TCP_H_
#define _NETWORK_TCP_H_

#include <string>
using namespace std;

class TCPConn
{
	public:
		TCPConn();
		~TCPConn();
		int connect(string ip,int port);	
		int close();
		int retryConnect(string ip,int port,int retry_time);
		int write(const void *buf,size_t len);
		int read(void *buf,size_t len);
		bool isConnected();

		bool isListening();
		int startListen(int port);
		int acceptNextConn(TCPConn*);
	private:
		int socketfd;
		int listen_socketfd;
};

#endif
