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
#include "api_soft_to_board.h"
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
	b2s_poll_t response;

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

//	printf("avail response: magic=0x%04x, fifo_status=0x%02x, err_flags=0x%02x, payload_len=%u\n", response.magic, response.fifo_status, response.err_flags, response.payload_len);

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

#define BIGGER_BUF_LEN B2S_MAX_LEN
static uint8_t spi_rx_fifo[SPI_RX_FIFO_DEPTH][BIGGER_BUF_LEN];

static int rx_fifo_wr;
static int rx_fifo_rd;

static uint8_t spi_tx_frame[BIGGER_BUF_LEN];

//static uint8_t spi_tx_fifo[SPI_TX_FIFO_DEPTH][MAX_SPI_FRAME_LEN];

//static int tx_fifo_wr;
//static int tx_fifo_rd;


static int cur_tx_frame_offs;

volatile int send_pending;


int subscribe_to(uint64_t* subs_vector)
{
	if(spi_init_cmd_queue() < 0)
		return -1;

	s2b_subscribe_t *p_msg = spi_init_cmd(CMD_SUBSCRIBE);

	if(!p_msg)
		return -1;

	memcpy(p_msg->subs_vector, subs_vector, sizeof(uint64_t)*B2S_SUBS_U64_ITEMS);
	spi_send_queue();

	return 0;
}

static int error_state;
int is_error()
{
	return error_state;
}

int ack_error()
{
	if(spi_init_cmd_queue() < 0)
		return -1;

	s2b_ack_error_t *p_msg = spi_init_cmd(CMD_ACK_ERROR);

	if(!p_msg)
		return -1;

	memset(p_msg, 0, sizeof(*p_msg));

	spi_send_queue();
	error_state = 0;
	return 0;
}

int spi_init_cmd_queue()
{
	if(send_pending)
	{
		printf("WARN: spi_init_cmd: send is already pending.\n");
		return -1;
	}

	((s2b_header_t*)spi_tx_frame)->magic = 0x2345;
	((s2b_header_t*)spi_tx_frame)->n_cmds = 0;
	cur_tx_frame_offs = S2B_HEADER_LEN;
	return 0;
}


/*
	Allocates space from the tx frame, giving a pointer to the area reserved for the
	command-specific struct.
*/
void* spi_init_cmd(uint8_t msgid)
{
	if(send_pending)
	{
		printf("WARN: spi_init_cmd: send is already pending.\n");
		return NULL;
	}

	if (s2b_msgs[msgid].size == 0) {
	   printf("ERROR: spi_init_cmd tries to initialize a message id (msgid=%u) with zero size\n", msgid);
	   return NULL;
	} // if
	int size = s2b_msgs[msgid].size;

	int next_start = cur_tx_frame_offs + sizeof(s2b_cmdheader_t) + size;

	if(next_start > S2B_MAX_LEN)
	{
		printf("ERROR: spi_init_cmd ran out of max RX buffer length (msgid=%u, size=%u, cur offset=%u, would end at=%u.\n", msgid, size, cur_tx_frame_offs, next_start);
		return NULL;
	}

	void* ret = &spi_tx_frame[cur_tx_frame_offs+sizeof(s2b_cmdheader_t)];

	printf("spi_init_cmd offset before=%d ", cur_tx_frame_offs);

	((s2b_cmdheader_t*)&spi_tx_frame[cur_tx_frame_offs])->msgid=msgid;
	((s2b_cmdheader_t*)&spi_tx_frame[cur_tx_frame_offs])->paylen=size;

	cur_tx_frame_offs = next_start;
	((s2b_header_t*)spi_tx_frame)->n_cmds++;


	printf(" after=%d\n", cur_tx_frame_offs);

	return ret;
}

int spi_send_queue()
{
	printf("spi_send_queue()\n");
	for(int i=0; i<16; i++)
		printf("%02x ", spi_tx_frame[i]);
	printf("\n");
	send_pending = 1;
	return 0;
}


uint8_t* spi_rx_pop()
{
	if(rx_fifo_wr == rx_fifo_rd)
		return NULL;

	uint8_t* ret = spi_rx_fifo[rx_fifo_rd];
	rx_fifo_rd++; if(rx_fifo_rd >= SPI_RX_FIFO_DEPTH) rx_fifo_rd = 0;
	return ret;
}


#define CRC_INITIAL_REMAINDER 0x00
#define CRC_POLYNOMIAL 0x07

#define CALC_CRC(remainder) \
	for(int crc__bit = 8; crc__bit > 0; --crc__bit) \
	{ \
		if((remainder) & 0b10000000) \
		{ \
			(remainder) = ((remainder) << 1) ^ CRC_POLYNOMIAL; \
		} \
		else \
		{ \
			(remainder) = ((remainder) << 1); \
		} \
	}



/*
	rxlen = # bytes we expect from the RobotBoard
	txlen = # bytes we have valid data in our send buffer

	The actual transaction length will be the bigger of the two guys.

< 0 error
  0 transaction ok, no more data on the FIFO
  1 transaction ok, more data on the FIFO that can be read right away, p_paylen is set to reflect the number of payload bytes of the next transaction
*/

int rx_crc_err_simu;
void simulate_crc_err_on_rx()
{
	rx_crc_err_simu = 1;
}

int tx_crc_err_simu;
void simulate_crc_err_on_tx()
{
	tx_crc_err_simu = 1;
}

static int transact(int rxlen, int txlen)
{
	int bigger_len = (txlen>rxlen)?txlen:rxlen;

	if(txlen)
	{
		// RX is _typically_ longer than tx, but other way around is possible as well.
		// The actual SPI transaction, however, must always be symmetric.
		// "Excess" data in either buffer is guaranteed to be zeroes. CRC calculation
		// works based on the total transaction length, which is the bigger.
		// Optimized CRC algorithm can be used for big buffers of repeated zeroes.

		// An example:
		// rxlen = 10000, txlen=1000, bigger_len=10000, excess_in_txbuf = 10000-1000 = 9000

		int excess_in_txbuf = bigger_len - txlen;
		memset(&spi_tx_frame[txlen], 0, excess_in_txbuf);

		// Now calculate CRC8 for the tx packet.
		// Optimization trick:
		// Typically, the tx length is around 30-3000 bytes, whereas rx tends to be around 30000-40000 bytes.
		// Hence, tx is padded with a lot of zeroes; crc needs to be calculated over the whole packet, it's
		// the only way we can utilize the HW CRC calculation inside the microcontroller SPI peripheral.
		// When xorring in only zeroes, the CRC8 wraps around every 127 bytes.

		uint8_t crc = CRC_INITIAL_REMAINDER;
		for(int i=0; i<txlen; i++)
		{
			crc ^= spi_tx_frame[i];
			CALC_CRC(crc);
		}
		int wrapped_length_of_zeroes = excess_in_txbuf%127;
		for(int i=0; i<wrapped_length_of_zeroes; i++)
		{
			// only zeroes, no need to xor anything in.
			CALC_CRC(crc);
		}

		spi_tx_frame[bigger_len] = crc+tx_crc_err_simu;
		tx_crc_err_simu=0;

		printf("Transact() with send! - CRC: %02x\n", crc);
	}

	struct spi_ioc_transfer xfer;

	memset(&xfer, 0, sizeof(xfer)); // unused fields need to be initialized zero.
	xfer.tx_buf = txlen?((spidev_buf_t)spi_tx_frame):0; // Send out zeroes if nothing to send.
	xfer.rx_buf = (spidev_buf_t)spi_rx_fifo[rx_fifo_wr];
	xfer.len = bigger_len+1; // 1 byte for CRC8

	if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
	{
		printf("ERROR: spi ioctl transfer operation failed: %d (%s)\n", errno, strerror(errno));
		return -1;
	}



	#if 1
		int plen = bigger_len+1;
		{
			printf("\n");
			int printlen = plen; if(printlen > 128) printlen=64;
			for(int i=0; i<printlen; i++)
			{
				printf("%02x ", spi_rx_fifo[rx_fifo_wr][i]);
				if(i%4 == 3) printf(" ");
				if(i%16 == 15) printf("\n");
			}
			printf("\n");
		}
		if(plen > 128)
		{
			printf("\n(truncated %u bytes)\n", plen-128);

			for(int i=plen-64; i<plen; i++)
			{
				printf("%02x ", spi_rx_fifo[rx_fifo_wr][i]);
				if(i%4 == 3) printf(" ");
				if(i%16 == 15) printf("\n");
			}
			printf("\n");
		}

	#endif

	b2s_header_t *p_header = (b2s_header_t*)&spi_rx_fifo[rx_fifo_wr][0];

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
		uint8_t crc = CRC_INITIAL_REMAINDER;
		for(int i=0; i<bigger_len; i++)
		{
			crc ^= spi_rx_fifo[rx_fifo_wr][i];
			CALC_CRC(crc);
		}

		if(spi_rx_fifo[rx_fifo_wr][bigger_len] != crc+rx_crc_err_simu)
		{
			rx_crc_err_simu = 0;
			printf("ERROR: CRC mismatch (expected %02x, got %02x) - ignoring the packet.\n", crc, spi_rx_fifo[rx_fifo_wr][bigger_len]);
			return -1;
		}
		rx_crc_err_simu = 0;

		if(p_header->err_flags&1)
		{
			printf("WARN: Error flag from RobotBoard: communicating the previous command failed!\n");
			error_state = 1;
		}


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
			usleep(50000);
			continue;
		}

		int paylen = -1;
		int avail = is_available(&paylen);
		if(avail > 0)
		{
			// Polling told us we will get this much data by calling transact():
			int rxlen = paylen+B2S_TOTAL_OVERHEAD_WITHOUT_CRC;

			int txlen = 0;
			// Do we have something to send as well? Let's do it on the same pass.
			if(send_pending)
			{
				txlen = cur_tx_frame_offs;
			}

			// Read as many packets as the RobotBoard wants to give.
			// These no-polling-inbetween multipackets are guaranteed to be of same length.
			int timeout = 16;
			//usleep(10000);

			while(transact(rxlen, txlen) == 1)
			{
			//	usleep(10000);
				txlen = 0; // We already sent our content, rest is zero.
				if(timeout-- == 0)
				{
					printf("ERROR: RobotBoard keeps reporting more readable content on FIFO over and over again, which is impossible.\n");
					sleep(1);
					break;
				}
			}
			send_pending = 0;
			usleep(40000);
		}
		else if(avail < 0)
		{
			// Error condition, let's hope it gets resolved by waiting.
			sleep(1);
		}
		else
		{
//			usleep(10000);
			usleep(10000);
		}
	}

	deinit_spi();

	return NULL;
}

