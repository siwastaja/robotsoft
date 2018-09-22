#include <stdint.h>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <signal.h>
#include "spi.h"

int retval;

volatile sig_atomic_t quit = 0;
void handle_sigint(int sig)
{
	quit = 1;
}

void* main_thread()
{
	printf("Hello!\n");

	while(!quit)
	{
		uint8_t* p_data = spi_rx_pop();
		if(p_data != NULL)
			printf("Got something!\n");
	}
	spi_comm_thread_quit();
	return NULL;
}

int main(int argc, char** argv)
{
	pthread_t thread_main, thread_spi;

	int ret;

	signal(SIGINT, handle_sigint);

	if( (ret = pthread_create(&thread_main, NULL, main_thread, NULL)) )
	{
		printf("ERROR: main thread creation, ret = %d\n", ret);
		return -1;
	}

	if( (ret = pthread_create(&thread_spi, NULL, spi_comm_thread, NULL)) )
	{
		printf("ERROR: tof3d access thread creation, ret = %d\n", ret);
		return -1;
	}

	pthread_join(thread_main, NULL);
	pthread_join(thread_spi, NULL);

	return retval;
}
