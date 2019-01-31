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
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>

#include "tcp_parser.h"

int tcp_listener_sock;
volatile int tcp_client_sock = -1; // One client at the time is allowed. Volatile because accessed in SIGPIPE signal handler.
#define MAX_MSG_LEN (1024*1024)
#define MAX_CRITICAL_MSG_LEN (8*1024)
#define MAX_PAYLEN_PER_MESSAGE ((MAX_MSG_LEN)-5)
#define TCP_WRITE_BUF_LEN ((MAX_PAYLEN_PER_MESSAGE+5)*3)

static uint8_t txfifo[TCP_WRITE_BUF_LEN];
static volatile int tx_wr, tx_rd;
pthread_mutex_t txfifo_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t newdata_cond = PTHREAD_COND_INITIALIZER;

pthread_t send_thread;
void* tcp_send_thread();


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

void sigpipe_handler(int signum)
{
	printf("Info: Broken pipe, closing TCP connection.\n");
	close(tcp_client_sock);
	tcp_client_sock = -1;
}

int init_tcp_comm()
{
	sigaction(SIGPIPE, &(struct sigaction){{sigpipe_handler}}, NULL);

	/* Create the socket and set it up to accept connections. */
	tcp_listener_sock = build_socket(22222);
	if(listen(tcp_listener_sock, 1) < 0)
	{
		perror ("listen");
		exit(EXIT_FAILURE);
	}

	return 0;
}


void tcp_comm_close()
{
	close(tcp_client_sock);
	int ret;
	if( (ret = pthread_cancel(send_thread)) )
	{
		printf("ERROR: tcp_comm_close pthread_cancel failed, ret = %d\n", ret);
	}
	tcp_client_sock = -1;
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
		tcp_comm_close();
	}

	tcp_client_sock = new_fd;
	printf("INFO: connection accepted, client %08x, port %u.\n", clientname.sin_addr.s_addr,
	        ntohs(clientname.sin_port));

	// Empty the FIFO.
	tx_wr = 0;
	tx_rd = 0;

	int ret;
	if( (ret = pthread_create(&send_thread, NULL, tcp_send_thread, NULL)) )
	{
		printf("ERROR: tcp_send_thread creation, ret = %d\n", ret);
		return -1;
	}

	if( (ret = pthread_detach(send_thread)) )
	{
		printf("ERROR: tcp_send_thread detaching, ret = %d\n", ret);
		return -1;
	}


	extern void tcp_send_robot_info();
	tcp_send_robot_info();
	tcp_send_statevect();

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


/*
	It WOULD be nice to be able to query the remaining send buffer in linux - but it's impossible.
	We implement our own buffer. This way, we can:
	* Know beforehand if a certain message would not fit. This means a completely stuck connection, and probably means
	  falling back to autonomous connectionless state is the right thing to do. Connection is reset.
	* Know beforehand that the buffer is starting to fill up - WAY before the above condition happens. This allows us to
	  dynamically adjust data generation, dropping all unnecessary communication, still accounting all vital state changes,
	  acknowledgements etc.
*/


// Don't try to increment by more than TCP_WRITE_BUF_LEN
#define INCR(what_, amount_)  do{what_+=(amount_); if(what_ >= TCP_WRITE_BUF_LEN) what_ -= TCP_WRITE_BUF_LEN;}while(0)

int tcp_query_sendbuf_space()
{
	// Will only block for a tiny amount of time
	// The consumer thread only locks when it accesses the indeces tx_wr, tx_rd
	pthread_mutex_lock(&txfifo_mutex);

	int space_in_buffer = tx_rd - tx_wr;
	if(space_in_buffer <= 0) space_in_buffer += TCP_WRITE_BUF_LEN;

	// I have a strange feeling that I want to keep one byte of clearance...
	space_in_buffer--;

	pthread_mutex_unlock(&txfifo_mutex);

	return space_in_buffer;
}

int tcp_is_space_for_noncritical_message()
{
	if(tcp_query_sendbuf_space() > MAX_MSG_LEN + 10*MAX_CRITICAL_MSG_LEN)
		return 1;
	return 0;
}

// Called from other threads:
int tcp_send(uint16_t msgid, uint32_t paylen, uint8_t* buf)
{
	int total_len = paylen+5;

	// Will only block for a tiny amount of time
	// The consumer thread only locks when it accesses the indeces tx_wr, tx_rd
	pthread_mutex_lock(&txfifo_mutex);

	int wr = tx_wr;

	// Ex.1: buf=1000, wr = 0,   rd = 0 -> space = 1000
	// Ex.2: buf=1000, wr = 10,  rd = 0 -> space = 990
	// Ex.3: buf=1000, wr = 990, rd = 0 -> space = 10
	// Ex.4: buf=1000, wr = 0,   rd = 10 -> space = 10
	

	int space_in_buffer = tx_rd - wr;
	if(space_in_buffer <= 0) space_in_buffer += TCP_WRITE_BUF_LEN;

	// I have a strange feeling that I want to keep one byte of clearance...
	space_in_buffer--;


//	printf("tcp_send %d bytes, free %d: wr=%d, rd=%d\n", total_len, space_in_buffer, wr, tx_rd);

	#ifdef TCP_DBGPR
	printf("tcp_send %d bytes, free %d: wr=%d, rd=%d\n", total_len, space_in_buffer, wr, tx_rd);
	#endif

	if(total_len > space_in_buffer)
	{
		printf("WARN: Trying to send larger tcp packet than there is free buffer for - msgid=%d paylen=%d - DISCONNECTING CLIENT\n", msgid, paylen);
		pthread_mutex_unlock(&txfifo_mutex);
		tcp_comm_close();
		return -1;
	}

	if(paylen > MAX_PAYLEN_PER_MESSAGE)
	{
		fprintf(stderr, "ERROR: tcp_send(): Illegal payload len %u\n", paylen);
		pthread_mutex_unlock(&txfifo_mutex);
		tcp_comm_close();
		return -1;
	}

	txfifo[wr] = (msgid&0xff00)>>8;
	INCR(wr, 1);
	txfifo[wr] = (msgid&0x00ff)>>0;
	INCR(wr, 1);
	txfifo[wr] = (paylen&0x00ff0000)>>16;
	INCR(wr, 1);
	txfifo[wr] = (paylen&0x0000ff00)>>8;
	INCR(wr, 1);
	txfifo[wr] = (paylen&0x000000ff)>>0;
	INCR(wr, 1);

	int last_wr_idx = wr + paylen - 1; // example: wr = 100, paylen = 10, last_wr_idx = 109

	if(last_wr_idx >= TCP_WRITE_BUF_LEN)
	{
		// Would go beyond the ringbuf: copy in two pieces
		// Example:
		// MAX_PAYLEN_PER_MESSAGE = 1000
		// wr = 900
		// paylen = 110
		// First write 1000-90 = 100 bytes, from idx 900 to 999
		// Then write paylen-first_write = 110-100 = 10 bytes, from idx 0 to 9
		// Set wr to second_write = 10

		int first_write = TCP_WRITE_BUF_LEN-wr;
		int second_write = paylen-first_write;

		#ifdef TCP_DBGPR
		printf("Two writes: first %d len %d to %d, second %d len %d to %d\n", wr, first_write, wr+first_write-1, 0, paylen-first_write, paylen-first_write-1);
		#endif

		memcpy(&txfifo[wr], buf, first_write);
		memcpy(&txfifo[0], buf, second_write);
		wr = second_write;
	}
	else
	{
		#ifdef TCP_DBGPR
		printf("One write: %d len %d to %d\n", wr, paylen, wr+paylen-1);
		#endif
		memcpy(&txfifo[wr], buf, paylen);
		wr += paylen;
	}

	tx_wr = wr;

	pthread_cond_signal(&newdata_cond);
	pthread_mutex_unlock(&txfifo_mutex);


	return 0;
}

// write() on tcp sockets has been broken with len over about 65536 on linux - so we do smaller writes to be sure.
#define MAX_WRITE_AT_ONCE 50000


/*
In this case, write() is the only cancellation point of the thread.
Thread cancellation is specified so that, cancellation will happen even during write() if it would block.
So, cancellation guarantees by design that write() will not block.


*/
void* tcp_send_thread()
{

	while(1)
	{
		// We want a simple timeout on condition variable.
		// This is not possible: only absolute time is supported.
		// To make things worse, it's poorly specified WHAT
		// "absolute time" means. Finally,
		// https://sector7.xray.aps.anl.gov/~dohnarms/programming/glibc/html/Condition-Variables.html
		// seems to be the only document which specified this:
		// "with the same origin as time and gettimeofday"
		// So, clock_gettime, which would work with the same timespec struct,
		// is illogically not supported.


		struct timespec timeout;
		struct timeval now;

		gettimeofday(&now,NULL);

		int64_t sec = now.tv_sec;
		int64_t nsec = (int64_t)now.tv_usec*1000;

		// Timeout every 10ms to check the condition, in case
		// there is some weird lockup edge condition
		// No more than 999 ms
		nsec += 10*1000000;

		if(nsec >= 1000000000)
		{
			nsec -= 1000000000;
			sec++;
		}

		timeout.tv_sec = sec;
		timeout.tv_nsec = nsec;

		int ret;
		pthread_mutex_lock(&txfifo_mutex);
		// cond_wait releases the mutex right away
		ret = pthread_cond_timedwait(&newdata_cond, &txfifo_mutex, &timeout);
		// Don't unlock, would lock right next:

		while(1) // call write() until the FIFO is completely emptied
		{
			// Fetch the shared variables:
			// Already locked
			int wr = tx_wr;
			int rd = tx_rd;
			pthread_mutex_unlock(&txfifo_mutex);


			int amount_to_send = wr - rd;
			if(amount_to_send < 0)
				amount_to_send += TCP_WRITE_BUF_LEN;

			if(amount_to_send == 0)
			{
				break;
			}

			if(ret != 0)
			{
				printf("NOTE: tcp_send_thread wakeup by timeout! This is not serious, just interesting.\n");
			}

			// Ex.: buflen=1000, rd = 900, amount_to_send = 100: over = 0
			// Ex.: buflen=1000, rd = 900, amount_to_send = 110: over = 10
			int over = rd + amount_to_send - TCP_WRITE_BUF_LEN; 

			int amount_to_write = amount_to_send;
			if(over > 0)
			{
				// Only write partially
				amount_to_write -= over;
			}

			if(amount_to_write > MAX_WRITE_AT_ONCE)
				amount_to_write = MAX_WRITE_AT_ONCE;


//			printf("write %d...\n", amount_to_write);

			#ifdef TCP_DBGPR
			printf("write: wr=%d, rd=%d, amount_to_send=%d, over=%d, amount_to_write=%d...\n", wr, rd, amount_to_send, over, amount_to_write);
			#endif
			// Write should block if there is no space in the buffer - let it do that
			int ret = write(tcp_client_sock, &txfifo[rd], amount_to_write);
			if(ret < 0)
			{
				fprintf(stderr, "ERROR: tcp_send_thread(): socket write error %d (%s). Closing TCP connection.\n", errno, strerror(errno));
				tcp_comm_close();
				return NULL;
			}
			if(ret == 0)
			{
				fprintf(stderr, "ERROR: blocking write() returned 0 unexpectedly. Closing TCP connection.\n");
				tcp_comm_close();
				return NULL;
			}
//			printf("... done write\n");
			#ifdef TCP_DBGPR
			printf("... done write\n");
			#endif
			rd += ret;
			if(rd >= TCP_WRITE_BUF_LEN)
				rd -= TCP_WRITE_BUF_LEN;


			// Update the shared variable:
			pthread_mutex_lock(&txfifo_mutex);
			tx_rd = rd;
			// Don't unlock, will use again right away
			
		} // end while send everything
	}
}
