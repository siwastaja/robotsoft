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

volatile uint64_t subs[B2S_SUBS_U64_ITEMS];

int read_file(char* fname)
{
	FILE* fil = fopen(fname, "rb");
	if(!fil)
	{
		printf("Error opening file %s for read\n", fname);
	}

	static uint8_t buf[B2S_MAX_LEN];

	int n_bytes_read = fread(buf, 1, B2S_MAX_LEN, fil);

	fclose(fil);

	printf("Read %d bytes\n", n_bytes_read);
	if(n_bytes_read < B2S_TOTAL_OVERHEAD_WITHOUT_CRC)
	{
		printf("ERROR: File is too short.\n");
		return -2;
	}

	int expected_size = ((b2s_header_t*)buf)->payload_len + B2S_TOTAL_OVERHEAD_WITHOUT_CRC;

	if(n_bytes_read != expected_size)
	{
		printf("ERROR: File size vs. header information mismatch, bytes_read=%d, expected_size=%d\n", n_bytes_read, expected_size);
		return -3;
	}


	uint8_t* p_data = buf;

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
				printf("msgid=%u  name=%s  comment=%s\n", s, b2s_msgs[s].name, b2s_msgs[s].comment);
				if(b2s_msgs[s].p_print)
					b2s_msgs[s].p_print(&p_data[offs]);
				offs += b2s_msgs[s].size;
			}
			t >>= 1;
		}
	}

	return 0;
	
}

void* main_thread()
{
	printf("Hello!\n");

	subscribe_to((uint64_t*)subs);

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
						printf("msgid=%u  name=%s  comment=%s\n", s, b2s_msgs[s].name, b2s_msgs[s].comment);
						if(b2s_msgs[s].p_print)
							b2s_msgs[s].p_print(&p_data[offs]);
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


//			if(cnt == 3)
//			{
//				subscribe_to(subs_test1);
//				continue;
//			}

//			if(cnt==5)
//				simulate_crc_err_on_rx();

/*
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
*/
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
	if(argc <= 1)
	{
		printf("Usage: boardmon <msgid1> [msgid2] [msgid3] ...\n");
		printf("Following subscriptions are available:\n");
		printf(" ID  length  name  (comment)\n");
		for(int i=0; i<B2S_MAX_MSGIDS; i++)
		{
			if(b2s_msgs[i].size != 0)
			{
				printf("%3u  %5u  %s   (%s)\n", i, b2s_msgs[i].size, b2s_msgs[i].name, b2s_msgs[i].comment);
			}
		}

		printf("Alternative usage: boardmon -f <filename>: reads one SPI data packet from a file, prints it normally.\n");

		return -1;
	}

	if(argv[1][0] == '-' && argv[1][0] == 'f')
	{
		if(argc != 3)
		{
			printf("Error: expecting filename after -f\n");
			return -1;
		}
		return read_file(argv[2]);
	}
	else
	{
		printf("Subscribing to...\n");
		for(int i=1; i<argc; i++)
		{
			int msgid = atoi(argv[i]);

			if(msgid < 1 || msgid >= B2S_MAX_MSGIDS || b2s_msgs[msgid].name == NULL)
			{
				printf("Invalid msgid %s  (atoi)-> %u\n", argv[i], msgid); 
			}
			else
			{
				printf("%3u %s %s ", msgid, b2s_msgs[msgid].name, b2s_msgs[msgid].comment);
				subs[msgid/64] |= 1ULL<<(msgid - msgid/64);
			}
		}

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
			printf("ERROR: spi access thread creation, ret = %d\n", ret);
			return -1;
		}

		pthread_join(thread_main, NULL);
		pthread_join(thread_spi, NULL);

		return retval;
	}
}
