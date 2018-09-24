/*
	PULUROBOT ROBOTSOFT Computer-on-RobotBoard main software

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



	Driver for SPI communication with RobotBoard2

	For Raspberry Pi 3, make sure that:
	dtparam=spi=on     is in /boot/config.txt uncommented
	/dev/spidev0.0 should exist

	Also:
	/boot/cmdline.txt:  spidev.bufsiz=65536
	This often defaults to 4096, which is ridiculously too small.
	(65535 is hardware maximum for STM32 DMA transfer, so bigger transfers
	couldn't be easily utilized, anyway.)
*/

#define _BSD_SOURCE  // glibc backwards incompatibility workaround to bring usleep back.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <unistd.h>

#include "spi.h"
#include "config.h"

#define DEFINE_API_VARIABLES
#include "api_board_to_soft.h"
#undef DEFINE_API_VARIABLES



extern volatile int verbose_mode;

static int spi_fd;
static volatile int running = 1;

static const unsigned char spi_mode = SPI_MODE_0;
static const unsigned char spi_bits_per_word = 8;
static const unsigned int spi_speed = SPI_SPEED_MHZ*1000000; // Hz

static int init_spi()
{
	spi_fd = open(SPI_DEV, O_RDWR);

	if(spi_fd < 0)
	{
		printf("ERROR: Opening SPI device %s failed: %d (%s).\n", SPI_DEV, errno, strerror(errno));
		return -1;
	}

	/*
		SPI_MODE_0 CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
		SPI_MODE_1 CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
		SPI_MODE_2 CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
		SPI_MODE_3 CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge
	*/

	/*
		Several code examples, (for example, http://www.raspberry-projects.com/pi/programming-in-c/spi/using-the-spi-interface),
		have totally misunderstood what SPI_IOC_WR_* and SPI_IOC_RD_* mean. They are not different settings for SPI RX/TX respectively
		(that wouldn't make any sense: SPI by definition has synchronous RX and TX so clearly the settings are always the same). Instead,
		SPI_IOC_WR_* and SPI_IOC_RD_* write and read the driver settings, respectively, as explained in spidev documentation:
		https://www.kernel.org/doc/Documentation/spi/spidev

		Here, we just set what we need.
	*/

	if(ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode) < 0)
	{
		printf("ERROR: Opening SPI devide: ioctl SPI_IOC_WR_MODE failed: %d (%s).\n", errno, strerror(errno));
		return -2;
	}

	if(ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word) < 0)
	{
		printf("ERROR: Opening SPI devide: ioctl SPI_IOC_WR_BITS_PER_WORD failed: %d (%s).\n", errno, strerror(errno));
		return -2;
	}

	if(ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0)
	{
		printf("ERROR: Opening SPI devide: ioctl SPI_IOC_WR_MAX_SPEED_HZ failed: %d (%s).\n", errno, strerror(errno));
		return -2;
	}

	return 0;
}


static int deinit_spi()
{
	if(close(spi_fd) < 0)
	{
		printf("WARNING: Closing SPI devide failed: %d (%s).\n", errno, strerror(errno));
		return -1;
	}

	return 0;
}

/*
	As you should know, SPI is a "show yours, I'll show mine" protocol. As a master, we'll only send something, and get something back.
	Unfortunately, Raspberry Pi can only be a master, in SPI terms.
	However, the RobotBoard is kind of a "real" master, because it's free-running all the data generation, and will do it regardless of what we are
	doing on the Raspberry - the data is anyway needed for low-level autonomous obstacle avoidance, completely independent of the Raspi.

	The firmware implements TX and RX FIFOs.

	poll_availability commands a small transfer, which will only return the first status bytes. The RobotBoard firmware does not increment it's FIFO
	counters with transfer sizes less than 16 bytes, to implement this "peek" mode.

	If we are happy with the status (i.e., there is data available), we can go on and make another transfer, with len >= 16 bytes. 


	When there is no data available (RobotBoard's TX fifo empty), the board sends out the following pattern:
	Byte 0: do not care (random value)
	Rest: 0x00
*/

// Depending on the spidev driver version and platform, this datatype might be uint32_t or uint64_t or something else.
// Correct type removes compiler warnings of the cast.
typedef uint64_t spidev_buf_t;

/*
 < 0 error
   0 no data available
   1 data is available (*p_paylen set to reflect number of payload bytes available)
*/
static int is_available(int* p_paylen)
{
	struct spi_ioc_transfer xfer;
	b2s_header_t response;

	memset(&xfer, 0, sizeof(xfer)); // unused fields need to be initialized zero.
	//xfer.tx_buf left at NULL - documented spidev feature to send out zeroes - we don't have anything to send, just want to get what the sensor wants to send us!
	xfer.tx_buf = (spidev_buf_t)NULL;
	xfer.rx_buf = (spidev_buf_t)&response;
	xfer.len = sizeof response;

	if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
	{
		printf("ERROR: spi ioctl transfer operation failed: %d (%s)\n", errno, strerror(errno));
		return -1;
	}

//	printf("avail response: header=0x%04x, res1=0x%04x, res2=0x%08x\n", response.header, response.reserved1, response.reserved2);

	if((response.magic & 0xff00) == 0)
	{
		return 0; // no data avail
	}
	else if(response.magic != 0xabcd)
	{
		printf("ERROR: Illegal magic number in the SPI message header: 0x%04x\n", response.magic);
		return -1;
	}
	else
	{
		*p_paylen = response.payload_len;
		return 1;
	}
}

#define MAX_SPI_FRAME_LEN 65536  // Keep sanely aligned for best cache performance

#define SPI_RX_FIFO_DEPTH 32
//#define SPI_TX_FIFO_DEPTH 8

static uint8_t spi_rx_fifo[SPI_RX_FIFO_DEPTH][MAX_SPI_FRAME_LEN];

static int rx_fifo_wr;
static int rx_fifo_rd;

//static uint8_t spi_tx_fifo[SPI_TX_FIFO_DEPTH][MAX_SPI_FRAME_LEN];

//static int tx_fifo_wr;
//static int tx_fifo_rd;


static int cur_tx_frame_offs;
static uint8_t spi_tx_frame[MAX_SPI_FRAME_LEN];

/*
	Allocates space from the tx frame, giving a pointer to the area reserved for the
	command-specific struct.
*/
void* spi_init_cmd(uint8_t msgid)
{
	if(!p_p_s2b_msgs[msgid])
	{
		printf("ERROR: spi_init_cmd tries to initialize a non-existing message id (msgid=%u)\n", msgid);
		return NULL;
	}
	int size = s2b_msg_sizes[msgid];

	int next_start = cur_tx_frame_offs + sizeof(s2b_cmdheader_t) + size;

	if(next_start > RX_MAX_LEN)
	{
		printf("ERROR: spi_init_cmd ran out of max RX buffer length (msgid=%u, size=%u, cur offset=%u, would end at=%u.\n", msgid, size, cur_tx_frame_offs, next_start);
		return NULL;
	}

	return &spi_tx_frame[cur_tx_frame_offs+sizeof(s2b_cmdheader_t)];
}

int spi_queue_cmd()
{

}


uint8_t* p_footer;

static void update_expected_subs(uint64_t *subs_vector)
{
	for(int i=0; i<256; i++)
	{
		if(p_p_tx_msgs[i])
			*p_p_tx_msgs[i] = 0;
	}

	int offs = SUBS_START_OFFSET;

	for(int i=0; i<4; i++)
	{
		subs[i] = 0;
		uint64_t t = subs_vector[i];
		for(int s=i*64; s<(i+1)*64; s++)
		{
			if(t & 1)
			{
				// id #s is enabled
				if(offs + tx_msg_sizes[s] > TX_MAX_LEN-TX_FOOTER_LEN)
				{
					// requested subscription doesn't fit: stop adding subscriptions.
					break;
				}

				subs[i] |= 1ULL<<(s-i*64);
				*p_p_tx_msgs[s] = tx_fifo[tx_fifo_cpu] + offs;
				offs += tx_msg_sizes[s];
			}
			t >>= 1;
		}
	}

	p_footer = &tx_fifo[i][offs];
	spi_rx_frame_len = offs + FOOTER_LEN;
	printf("INFO: Updated expected subscriptions, spi_rx_frame_len=%d\n", spi_rx_frame_len);
}


uint8_t* spi_rx_pop()
{
	if(rx_fifo_wr == rx_fifo_rd)
		return NULL;

	uint8_t* ret = spi_rx_fifo[rx_fifo_rd];
	rx_fifo_rd++; if(rx_fifo_rd >= SPI_RX_FIFO_DEPTH) rx_fifo_rd = 0;
	return ret;
}

/*
< 0 error
  0 transaction ok, no more data on the FIFO
  1 transaction ok, more data on the FIFO that can be read right away, p_paylen is set to reflect the number of payload bytes of the next transaction
*/
static int transact(int len)
{
	struct spi_ioc_transfer xfer;

	memset(&xfer, 0, sizeof(xfer)); // unused fields need to be initialized zero.
	xfer.tx_buf = (spidev_buf_t)spi_tx_frame;
	xfer.rx_buf = (spidev_buf_t)spi_rx_fifo[rx_fifo_wr];
	xfer.len = len;

	if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
	{
		printf("ERROR: spi ioctl transfer operation failed: %d (%s)\n", errno, strerror(errno));
		return -1;
	}

/*
	printf("\n");
	for(int i=0; i<32; i++)
	{
		printf("%02x ", spi_rx_fifo[rx_fifo_wr][i]);
		if(i%8 == 7) printf(" ");
	}
	printf("\n");
*/

	b2s_header_t p_header = (b2s_header_t*)&spi_rx_fifo[rx_fifo_wr][0];

	if((p_header->magic & 0xff00) == 0)
	{
		printf("ERROR: RobotBoard SPI comms returned \"no data available\" message, when data was supposedly available\n");
		return -1;
	}
	else if(p_header->magic != 0xabcd)
	{
		printf("ERROR: Illegal header in the SPI message: magic=0x%04x\n", p_header->magic);
		return -1;
	}
	else
	{
		rx_fifo_wr++; if(rx_fifo_wr >= SPI_RX_FIFO_DEPTH) rx_fifo_wr = 0;

		if(p_header->fifo_status&1) // Next read WILL give the next FIFO item - no polling necessary.
			return 1;

		return 0;
	}
}


void spi_comm_thread_quit()
{
	running = 0;
}

void* spi_comm_thread()
{
	init_spi();
	while(running)
	{
		int next_rx_fifo_wr = rx_fifo_wr+1; if(next_rx_fifo_wr >= SPI_RX_FIFO_DEPTH) next_rx_fifo_wr = 0;
		if(next_rx_fifo_wr == rx_fifo_rd)
		{
			printf("WARNING: SPI RX FIFO is going to overflow, ignoring data & sleeping...\n");
			usleep(500000);
			continue;
		}

		int paylen = -1;
		int avail = is_available(&paylen);
		if(avail > 0)
		{
			// Polling told us we will get this much data by calling transact():
			int rxlen = paylen+B2S_TOTAL_OVERHEAD;

			int txlen = 0;
			// Do we have something to send as well? Let's do it on the same pass.
			if(tx_fifo_wr != tx_fifo_rd)
			{
			
				txlen = spi_tx_frame_len+S2B_TOTAL_OVERHEAD;
			}



			// Transfer needs to be long enough to account for tx and rx sides, whichever is longer:
			int len = (txlen>rxlen)?txlen:rxlen;

			// Read as many packets as the RobotBoard wants to give.
			// These no-polling-inbetween multipackets are guaranteed to be of same length.
			int timeout = 16;
			while(transact(len) == 1)
			{
				if(timeout-- == 0)
				{
					printf("ERROR: RobotBoard keeps reporting more readable content on FIFO over and over again, which is impossible.\n");
					sleep(1);
					break;
				}
			}
			usleep(10000);
		}
		else if(avail < 0)
		{
			// Error condition, let's hope it gets resolved by waiting.
			sleep(1);
		}
		else
		{
			usleep(10000);
		}
	}

	deinit_spi();

	return NULL;
}

