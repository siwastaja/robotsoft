/*
	PULUROBOT RN1-HOST Computer-on-RobotBoard main software

	(c) 2017-2018 Pulu Robotics and other contributors
	Maintainer: Antti Alhonen <antti.alhonen@iki.fi>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.



	Module for working around complex POSIX socket API. Builds a simple TCP connection.


*/

#define _BSD_SOURCE // for usleep
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <termios.h>
#include <string.h>
#include <fcntl.h>
#include <stropts.h>
#include <sys/select.h>
#include <sys/ioctl.h>

#include "tcp_parser.h"

int tcp_listener_sock;
int tcp_client_sock = -1; // One client at the time is allowed.

int build_socket(uint16_t port)
{
	int sock;
	struct sockaddr_in name;

	sock = socket(PF_INET, SOCK_STREAM, 0);
	if (sock < 0)
	{
		perror ("socket");
		exit(EXIT_FAILURE);
	}


	int reuse = 1;
	if(setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) < 0)
	{
		perror("setsockopt(SO_REUSEADDR) failed");
		exit(EXIT_FAILURE);
	}

	#ifdef SO_REUSEPORT
	if(setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, (const char*)&reuse, sizeof(reuse)) < 0) 
	{
		perror("setsockopt(SO_REUSEPORT) failed");
		exit(EXIT_FAILURE);
	}
	#endif

	name.sin_family = AF_INET;
	name.sin_port = htons (port);
	name.sin_addr.s_addr = htonl (INADDR_ANY);
	if (bind(sock, (struct sockaddr *) &name, sizeof (name)) < 0)
	{
		perror("bind");
		exit(EXIT_FAILURE);
	}

	return sock;
}

int init_tcp_comm()
{
	/* Create the socket and set it up to accept connections. */
	tcp_listener_sock = build_socket(22222);
	if(listen(tcp_listener_sock, 1) < 0)
	{
		perror ("listen");
		exit(EXIT_FAILURE);
	}

	return 0;
}

// Call this when you have data (connection request) in tcp_listener_sock input buffer, for example, after using select()
int handle_tcp_listener()
{
	int new_fd;
	struct sockaddr_in clientname;
	size_t size = sizeof(clientname);
	new_fd = accept(tcp_listener_sock, (struct sockaddr *) &clientname, &size);
	if(new_fd < 0)
	{
		fprintf(stderr, "ERROR: accepting tcp connection request failed.\n");
	}

	if(tcp_client_sock >= 0)
	{
		// We already have a client; disconnect it.
		close(tcp_client_sock);
	}

	tcp_client_sock = new_fd;
	printf("INFO: connection accepted, client %08x, port %u.\n", clientname.sin_addr.s_addr,
	        ntohs(clientname.sin_port));

	return 0;
}

// Call this when you have data in tcp_client_sock input buffer, for example, after using select()
int handle_tcp_client()
{
	int ret = tcp_parser(tcp_client_sock);
	if(ret == -10 || ret == -11)
	{
		printf("Info: closing TCP connection.\n");
		close(tcp_client_sock);
		tcp_client_sock = -1;
	}
	return ret;
}

void tcp_comm_close()
{
	close(tcp_client_sock);
	tcp_client_sock = -1;
}

// write() on tcp sockets has been broken with len over about 65536 on linux - so we do smaller writes to be sure.
#define MAX_WRITE_AT_ONCE 50000

int tcp_send(uint32_t msgid, int32_t paylen, uint8_t* buf)
{
	int timeout = 100;

	static uint8_t headbuf[8];
	headbuf[0] = (msgid&0xff000000)>>24;
	headbuf[1] = (msgid&0x00ff0000)>>16;
	headbuf[2] = (msgid&0x0000ff00)>>8;
	headbuf[3] = (msgid&0x000000ff)>>0;
	headbuf[4] = (paylen&0xff000000)>>24;
	headbuf[5] = (paylen&0x00ff0000)>>16;
	headbuf[6] = (paylen&0x0000ff00)>>8;
	headbuf[7] = (paylen&0x000000ff)>>0;


	int header_cnt = 8;
	uint8_t* p_head = headbuf;
	while(header_cnt)
	{
		int ret = write(tcp_client_sock, p_head, header_cnt); 
		if(ret < 0)
		{
			fprintf(stderr, "ERROR: tcp_send(): socket write error %d (%s), writing header. Closing TCP connection.\n", errno, strerror(errno));
			tcp_comm_close();
			return -1;
		}
		header_cnt -= ret;
		p_head += ret;
		if(header_cnt)
		{
			if(timeout%10 == 0)
				printf("INFO: tcp_send(): write() didn't write everything, writing header, written=%d, left=%d\n", ret, header_cnt);
			usleep(200);
			timeout--;

			if(timeout == 0)
			{
				fprintf(stderr, "ERROR: tcp_send() timeouted because write() doesn't seem to do anything useful, writing header. Closing TCP connection.\n");
				tcp_comm_close();
				return -1;
			}
		}
	}


	uint8_t* p_buf = buf;
	while(paylen)
	{
		int ret = write(tcp_client_sock, p_buf, (paylen>MAX_WRITE_AT_ONCE)?MAX_WRITE_AT_ONCE:paylen); 
		if(ret < 0)
		{
			fprintf(stderr, "ERROR: tcp_send(): socket write error %d (%s). Closing TCP connection.\n", errno, strerror(errno));
			tcp_comm_close();
			return -1;
		}
		paylen -= ret;
		p_buf += ret;
		if(paylen)
		{
			if(timeout%10 == 0)
				printf("INFO: tcp_send(): write() didn't write everything, written=%d, left=%d\n", ret, paylen);
			usleep(200);
			timeout--;

			if(timeout == 0)
			{
				fprintf(stderr, "ERROR: tcp_send() timeouted because write() doesn't seem to do anything useful. Closing TCP connection.\n");
				tcp_comm_close();
				return -1;
			}
		}
	}

	return 0;
}
