#ifndef _NETWORK_UDP_H_
#define _NETWORK_UDP_H_
#include <unistd.h>
#include <stdint.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>

#include "rbx4net.h"

#define MAX_BUF_SIZE	(1024)

using namespace std;

class NetworkUDP
{
	public:
		NetworkUDP();
		~NetworkUDP();
		int init_recv(char* ip, uint16_t port);
		int init_send(char* ip, uint16_t port);
		int recv_data(void* (*func)(void *));
		int send_data(void * packet, int size);

	protected:
	private:
		int sock_recv;
		int sock_send;
		struct sockaddr_in 	addr_recv;
		struct sockaddr_in 	addr_send;
		char recv_buf[MAX_BUF_SIZE];
};

#endif 
