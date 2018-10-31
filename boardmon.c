#define _BSD_SOURCE  // glibc backwards incompatibility workaround to bring usleep back.
#include <stdint.h>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

#include "spi.h"

#include "api_board_to_soft.h"
#include "api_soft_to_board.h"

int retval;

volatile sig_atomic_t quit = 0;
void handle_sigint(int sig)
{
	quit = 1;
}

uint64_t subs_test1[4] = {0b0100,0,0,0};
uint64_t subs_test2[4] = {0b0100,0,0,0};
uint64_t subs_test3[4] = {0b0100,0,0,0};
uint64_t subs_test4[4] = {0b0100,0,0,0};

void* main_thread()
{
	printf("Hello!\n");

	int cnt = 0;
	while(!quit)
	{
		usleep(1000);

		uint8_t* p_data = spi_rx_pop();
		if(p_data != NULL)
		{
			printf("Got something! Messages:\n");

			int offs = sizeof(b2s_header_t);
			for(int i=0; i<B2S_SUBS_U64_ITEMS; i++)
			{
				uint64_t t = ((b2s_header_t*)p_data)->subs[i];
				for(int s=i*64; s<(i+1)*64; s++)
				{
					if(t & 1)
					{
						// id #s is enabled
						printf("msgid=%u  name=%s  comment=%s\n", s, b2s_meta[s].name, b2s_meta[s].comment);
						//b2s_meta[s].p_print(&p_data[offs]);
						offs += b2s_msgs[s].size;
					}
					t >>= 1;
				}
			}

			cnt++;

			if(is_error())
			{
				printf("Communication error reported -- clearing it.\n");
				ack_error();
				continue;
			}


			if(cnt == 3)
			{
				subscribe_to(subs_test1);
				continue;
			}

//			if(cnt==5)
//				simulate_crc_err_on_rx();


			if(cnt==7)
			{

				if(spi_init_cmd_queue() >= 0)
				{
					s2b_test1_t *p_msg = spi_init_cmd(CMD_TEST1);

					if(p_msg != NULL)
					{

						uint8_t c = 0x11;
						for(int i=0; i < sizeof(p_msg->buf); i++)
							p_msg->buf[i] = c++;

						spi_send_queue();
					}
					else
						printf("WARN: p_msg == NULL\n");
				}
				else
					printf("WARN: spi_init_cmd_queue failed\n");

				continue;


			}

/*			else if(cnt == 20)
				subscribe_to(subs_test2);
			else if(cnt == 30)
				subscribe_to(subs_test3);
			else if(cnt == 40)
				subscribe_to(subs_test4);
			else if(cnt == 50)
				cnt = 0;*/
		}
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
