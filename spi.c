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

static int is_available()
{
	struct spi_ioc_transfer xfer;
	struct response { uint16_t header; uint16_t reserved1; uint32_t reserved2;} __attribute__((packed)) response;

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

	if((response.header & 0xff00) == 0)
	{
		if(response.reserved1 != 0 || response.reserved2 != 0)
		{
			printf("WARN: Corrupted \"no data available\" message, header=0x%04x, res1=0x%04x, res2=0x%08x\n", response.header, response.reserved1, response.reserved2);
		}
		return 0;
	}
	else if(response.header != 0xabcd)
	{
		printf("ERROR: Illegal header in the SPI message: header=0x%04x\n", response.header);
		return -1;
	}
	else
	{
		return 1;
	}
}

#define MAX_SPI_FRAME_LEN 65536 // Keep sanely aligned for best cache performance

#define SPI_RX_FIFO_DEPTH 32

static uint8_t spi_rx_fifo[SPI_RX_FIFO_DEPTH][MAX_SPI_FRAME_LEN];

static int rx_fifo_wr;
static int rx_fifo_rd;

static int spi_rx_frame_len = 1000; // depends on subscriptions

static uint8_t spi_tx_frame[MAX_SPI_FRAME_LEN];

static int spi_tx_frame_len = 1000;


#define SUBS_START_OFFSET 16
#define FOOTER_LEN 4

// Enabled subscriptions, 1 bit per message ID, [0] LSb = id0, [0] MSb = id63, [1] LSb = id64, and so on:
uint64_t subs[4];

uint8_t* p_footer;

static void update_subs(uint64_t *subs_vector)
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
	printf("INFO: Updated subscriptions, spi_rx_frame_len=%d\n", spi_rx_frame_len);
}



uint8_t* spi_rx_pop()
{
	if(rx_fifo_wr == rx_fifo_rd)
		return NULL;

	uint8_t* ret = spi_rx_fifo[rx_fifo_rd];
	rx_fifo_rd++; if(rx_fifo_rd >= SPI_RX_FIFO_DEPTH) rx_fifo_rd = 0;
	return ret;
}

static int transact()
{
	struct spi_ioc_transfer xfer;

	memset(&xfer, 0, sizeof(xfer)); // unused fields need to be initialized zero.
	xfer.tx_buf = (spidev_buf_t)spi_tx_frame;
	xfer.rx_buf = (spidev_buf_t)spi_rx_fifo[rx_fifo_wr];
	xfer.len = (spi_rx_frame_len>spi_tx_frame_len)?spi_rx_frame_len:spi_tx_frame_len;

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
	uint16_t header = *(uint16_t*)&spi_rx_fifo[rx_fifo_wr][0];
	uint8_t  fifo_flags = *(uint8_t*)&spi_rx_fifo[rx_fifo_wr][2];

//	printf("header=%04x fifo_flags=%02x\n", header, fifo_flags);

	if((header & 0xff00) == 0)
	{
		printf("ERROR: RobotBoard SPI comms returned \"no data available\" message, when data was supposedly available\n");
		return -1;
	}
	else if(header != 0xabcd)
	{
		printf("ERROR: Illegal header in the SPI message: header=0x%04x\n", header);
		return -1;
	}
	else
	{
		rx_fifo_wr++; if(rx_fifo_wr >= SPI_RX_FIFO_DEPTH) rx_fifo_wr = 0;

		if(fifo_flags&1) // Next read WILL give next FIFO item - no polling necessary.
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

		int avail = is_available();
		if(avail > 0)
		{
			int timeout = 16;
			while(transact() == 1)
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
			// Error condition
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

