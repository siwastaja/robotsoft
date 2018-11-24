/*
	PULUROBOT SPIPROG

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



	Tool for reflashing RobotBoard Rev.2 (firmware updates or calibration data)


	NOTE: Requires little-endian system


*/

#define _BSD_SOURCE // for usleep
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <unistd.h>


#define SPI_DEVICE "/dev/spidev0.0"


static int spi_fd;

static const unsigned char spi_mode = SPI_MODE_0;
static const unsigned char spi_bits_per_word = 8;
static unsigned int spi_speed = 15000000; // Hz

static int init_spi()
{
	spi_fd = open(SPI_DEVICE, O_RDWR);

	if(spi_fd < 0)
	{
		printf("ERROR: Opening SPI device %s failed: %d (%s).\n", SPI_DEVICE, errno, strerror(errno));
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
		printf("WARNING: Closing SPI device failed: %d (%s).\n", errno, strerror(errno));
		return -1;
	}

	return 0;
}

#define CRC_INITIAL_REMAINDER 0x00
#define CRC_POLYNOMIAL 0x07 // As per CRC-8-CCITT

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

int verbose = 0;

uint8_t fw[128*1024+4]; // initialized to zero

int prog_verify_sector(int size, int bank, int sector)
{
	if((bank==1 && sector==0) && (size < 2048 || size >= 128*1024))
	{
		printf("Error: illegal fw file size (%d) - writing to bank0,sector0, so this must be a valid firmware.\n", size);
		deinit_spi();
		return -1;
	}
	else if(size <0 || size > 128*1024)
	{
		printf("Error: illegal fw file size (%d)\n", size);
		deinit_spi();
		return -1;
	}

	uint8_t chk = CRC_INITIAL_REMAINDER;

	for(int i = 0; i < 128*1024; i++)
	{
		chk ^= fw[i];
		CALC_CRC(chk);
	}


	if(verbose) printf("Sending the write command...\n");
	{
		struct __attribute__((packed)) write_cmd { uint32_t magic; uint8_t bank; uint8_t sector; uint8_t crc;} write_cmd;
		write_cmd.magic = 0xabba1337;
		write_cmd.bank = bank;
		write_cmd.sector = sector;
		write_cmd.crc = chk;

		struct spi_ioc_transfer xfer;
		memset(&xfer, 0, sizeof(xfer));
		xfer.tx_buf = &write_cmd;
		xfer.rx_buf = NULL;
		xfer.len = 7;
		xfer.cs_change = 0;

		if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
		{
			printf("ERROR: spi ioctl transfer operation failed: %d (%s)\n", errno, strerror(errno));
			deinit_spi();
			return -1;
		}
	}

	int bytes_sent = 0, chunk_count = 0;
	if(!verbose) printf("Sending the data...\n");
	while(bytes_sent < 128*1024)
	{
		if(verbose) printf("Sending the data, chunk %u...\n", chunk_count);
		int amount_to_send = 128*1024 - bytes_sent; if(amount_to_send > 65532) amount_to_send = 65532;
		struct spi_ioc_transfer xfer;
		memset(&xfer, 0, sizeof(xfer)); // unused fields need to be initialized zero.
		xfer.tx_buf = &fw[bytes_sent];
		xfer.rx_buf = NULL; // not interested in what we get back
		xfer.len = amount_to_send;
		xfer.cs_change = 0;

		if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
		{
			printf("ERROR: spi ioctl transfer operation failed: %d (%s)\n", errno, strerror(errno));
			deinit_spi();
			return -1;
		}
		bytes_sent += amount_to_send;
		chunk_count++;
	}
	if(verbose) printf("Polling for status...\n");

	int cnt = 0;
	while(1)
	{
		usleep(50000);

		struct __attribute__((packed)) reply { uint32_t magic; uint8_t result;} reply;

		uint8_t dummy[5] = {0xa1, 0xa2, 0xa3, 0xa4, 0xa5};

		struct spi_ioc_transfer xfer;
		memset(&xfer, 0, sizeof(xfer)); // unused fields need to be initialized zero.
		xfer.tx_buf = dummy; //NULL; // tx data content doesn't matter
		xfer.rx_buf = &reply;
		xfer.len = 5;
		xfer.cs_change = 0;

		if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
		{
			printf("ERROR: spi ioctl transfer operation failed: %d (%s)\n", errno, strerror(errno));
			deinit_spi();
			return -1;
		}

		if(verbose) printf("Reply: magic=%08x ret=%d\n", reply.magic, reply.result);
		if(reply.magic == 0xacdc3579)
		{
			if(reply.result != 123)
			{
				printf("FLASHING ERROR: flasher returned error code %d. Nothing was written. Old firmware should be intact.\n", reply.result);
				return -5;
			}
			else
				break;
		}


		cnt++;

		if(cnt > 10000000/50000)
		{
			printf("Flashing timed out. Most likely, the old firmware is intact and this is just a communication error. Please reset the system manually.\n");
			deinit_spi();
			return -2;
		}

	}

	if(verbose) printf("Flashing OK. Reading back to verify...\n");

	uint8_t rx_fw[256*1024];
	int bytes_read = 0, read_chunk_count = 0;

	while(bytes_read < 128*1024)
	{
		if(verbose) printf("Verifying chunk %u...\n", read_chunk_count);

		{
			struct __attribute__((packed)) read_cmd { uint32_t magic; uint32_t offset;} read_cmd;
			read_cmd.magic = 0xbeef1234;
			read_cmd.offset = ((bank-1)*8+sector)*128*1024 + bytes_read;

			struct spi_ioc_transfer xfer;
			memset(&xfer, 0, sizeof(xfer));
			xfer.tx_buf = &read_cmd;
			xfer.rx_buf = NULL;
			xfer.len = 8;
			xfer.cs_change = 0;

			if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
			{
				printf("ERROR: spi ioctl transfer operation failed: %d (%s)\n", errno, strerror(errno));
				deinit_spi();
				return -1;
			}
		}

		int amount_to_read = 128*1024-bytes_read; if(amount_to_read > 65532) amount_to_read = 65532;
		{
			struct spi_ioc_transfer xfer;
			memset(&xfer, 0, sizeof(xfer));
			xfer.tx_buf = NULL;
			xfer.rx_buf = &rx_fw[bytes_read];
			xfer.len = amount_to_read;
			xfer.cs_change = 0;

			if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
			{
				printf("ERROR: spi ioctl transfer operation failed: %d (%s)\n", errno, strerror(errno));
				deinit_spi();
				return -1;
			}
		}
		bytes_read += amount_to_read;
		read_chunk_count++;
	}

	int errors = 0;
	for(int i=0; i<128*1024; i++)
	{
		if(rx_fw[i] != fw[i])
		{
			errors++;
			if(errors == 10) printf("(Suppressing further error messages.)\n");
			else if(errors < 10) printf("ERROR: Mismatch at %d: read back %02x, should be %02x.\n", i, rx_fw[i], fw[i]);
		}
	}
	if(errors)
		printf("ERROR: Flashing & verification: %d errors in %d bytes!\n", errors, 128*1024);
	else
		printf("OK: Flashing & verification done (%d bytes)\n", 128*1024);

	return 0;
}

int main(int argc, char** argv)
{

	if(!(argc == 2 || argc == 4 || argc == 5 || argc >= 11))
	{
		printf("Usage: spiprog <reset_type> [bank&sector] [firmware_filename] [v]\n");
		printf("Reset types: r = s = soft reset.  h = hard reset.  l = stuck in infinite loop.  d = don't reset, stay in programmer.  R = reset if already in programmer\n");
		printf("Bank&sector string: FOR FIRMWARE: 10 = bank1, sector0.  Valid banks:1,2. Valid sectors:0..7\n");
		printf("If bank, sector and firmware filename is supplied, it's flashed and verified.\n");
		printf("If additional v is given, verbose mode is enabled.\n");
		printf("\n");
		printf("Ex.1:   ./spiprog r        -- Resets the device\n");
		printf("Ex.2:   ./spiprog d        -- Goes in the programmer (turning motors, sensors, etc. off), stays there (nice for hotswaps)\n");
		printf("Ex.3:   ./spiprog R        -- Resets the device after Ex.2\n");
		printf("Ex.4:   ./spiprog r 10 firmware.bin     -- reprograms the firmware\n");
		printf("Ex.5:   ./spiprog r 10 firmware.bin v   -- reprograms the firmware, being verbose about the process\n");
		printf("Ex.6:   ./spiprog r 25 calib.bin        -- reprograms bank2,sector5 with calib.bin\n");

		printf("Totally different mode - reprogram all the TOF calibration: 10 arguments (or 11 with v (verbose)):\n");
		printf("./spiprog <file for sensor 0> <file for sensor1> ... <file for sensor 9>\n");
		printf("Use 0 instead of filename to generate blank data (0xff) (no sensor)\n");

		return -10;
	}


	int reset_type = -1;
	if(argv[1][0] == 'r' || argv[1][0] == 's') reset_type = 1;
	if(argv[1][0] == 'h') reset_type = 2;
	if(argv[1][0] == 'l') reset_type = 55;
	if(argv[1][0] == 'd') reset_type = 999;
	if(argv[1][0] == 'R') reset_type = 9999;

	if(argc==11 || argc==12) 
	{
		reset_type = 1;
	}
	else
	{
		if(reset_type == -1 || argv[1][1] != 0)
		{
			printf("Error: invalid <reset_type> parameter. See usage.\n");
			return -10;
		}
	}

	if((argc == 5 && argv[4][0] == 'v') || (argc==12 && argv[11][0] == 'v'))
	{
		verbose = 1;
		printf("Verbose mode.\n");
	}

	int bank=-1;
	int sector=-1;

	if(argc >= 4 && argc < 11)
	{
		bank=argv[2][0]-'0';
		sector=argv[2][1]-'0';

		if(bank < 1 || bank > 2)
		{
			printf("Error: invalid bank number (must be 1 or 2)\n");
			return -10;
		}

		if(sector < 0 || sector > 7)
		{
			printf("Error: invalid sector number (must be 0 to 7)\n");
			return -10;
		}

	}

	uint8_t* tofcals[10];
	#define TOFCAL_SIZE 157284
	#define SECT_SIZE (128*1024)
	if(argc>=11)
	{
		for(int i=0; i<10; i++)
		{
			tofcals[i] = malloc(TOFCAL_SIZE);
			if(!tofcals[i])
			{
				printf("Out of memory!\n");
				return -1;
			}

			// Clear any excess with 0xff anyway:
			memset(tofcals[i], 0xff, TOFCAL_SIZE);

			if(!(argv[i+1][0] == '0' && argv[i+1][1] == 0))
			{
				printf("Reading TOF calibration input file %s...\n", argv[i+1]);
				FILE *toff = fopen(argv[i+1], "rb");
				if(!toff)
				{
					printf("Error opening %s.\n", argv[i+1]);
					return -1;
				}
				int len = fread(tofcals[i], 1, TOFCAL_SIZE, toff);
				if(len < 10000)
				{
					printf("The file length doesn't seem right! (%d bytes?)\n", len);
				}
				fclose(toff);
			}
			
		}
	}

	init_spi();


	if(reset_type != 9999)
	{
		if(verbose) printf("Sending magic flasher entering sequence...\n");

		{
			uint32_t msg = 0x9876fedb;
			struct spi_ioc_transfer xfer;
			memset(&xfer, 0, sizeof(xfer)); // unused fields need to be initialized zero.
			xfer.tx_buf = &msg;
			xfer.rx_buf = NULL;
			xfer.len = 4;
			xfer.cs_change = 0;

			if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
			{
				printf("ERROR: spi ioctl transfer operation failed: %d (%s)\n", errno, strerror(errno));
				deinit_spi();
				return -1;
			}
		}
	}

	usleep(500000);

	if(argc>=11)
	{
		// sector size 131072
		// calib  size 157284
		// 5 sensors fit 6 sectors
		// was tired. calculated in excel.

		for(int b=0; b<2; b++)
		{
			memcpy(&fw[0],      &tofcals[5*b+0][0]     , 131072);
			prog_verify_sector(SECT_SIZE, b+1, 2);
			memcpy(&fw[0],      &tofcals[5*b+0][131072], 26212);
			memcpy(&fw[26212],  &tofcals[5*b+1][0]     , 104860);
			prog_verify_sector(SECT_SIZE, b+1, 3);
			memcpy(&fw[0],      &tofcals[5*b+1][104860], 52424);
			memcpy(&fw[52424],  &tofcals[5*b+2][0]     , 78648);
			prog_verify_sector(SECT_SIZE, b+1, 4);
			memcpy(&fw[0],      &tofcals[5*b+2][78648] , 78636);
			memcpy(&fw[78636],  &tofcals[5*b+3][0]     , 52436);
			prog_verify_sector(SECT_SIZE, b+1, 5);
			memcpy(&fw[0],      &tofcals[5*b+3][52436] , 104848);
			memcpy(&fw[104848], &tofcals[5*b+4][0]     , 26224);
			prog_verify_sector(SECT_SIZE, b+1, 6);
			memcpy(&fw[0],      &tofcals[5*b+4][26224] , 131060);
			prog_verify_sector(SECT_SIZE, b+1, 7);
		}
	}
	else if(argc >= 4)
	{
		if(verbose) printf("Reading the firmware file...\n");
		FILE* bin = fopen(argv[3], "rb");
		if(!bin)
		{
			printf("Error opening %s for read.\n", argv[3]);
			deinit_spi();
			return -1;
		}

		int size = fread(fw, 1, 128*1024, bin);

		prog_verify_sector(size, bank, sector);
	}

	// Do the reset:

	usleep(100000);

	if(reset_type != 999)
	{
		printf("Resetting the device...\n");

		if(reset_type == 9999) reset_type = 1;

		{
			struct __attribute__((packed)) reset_cmd { uint32_t magic; uint8_t type;} reset_cmd;
			reset_cmd.magic = 0xdead5678;
			reset_cmd.type = reset_type;

			struct spi_ioc_transfer xfer;
			memset(&xfer, 0, sizeof(xfer));
			xfer.tx_buf = &reset_cmd;
			xfer.rx_buf = NULL;
			xfer.len = 5;
			xfer.cs_change = 0;

			if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
			{
				printf("ERROR: spi ioctl transfer operation failed: %d (%s)\n", errno, strerror(errno));
				deinit_spi();
				return -1;
			}
		}
	}

	if(verbose) printf("Everything's done, quit.\n");

	deinit_spi();
	return 0;
}

