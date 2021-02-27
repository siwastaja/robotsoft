#pragma once

#define SPI_DEV  "/dev/spidev0.0"
#define SPI_SPEED_MHZ 20

#define UART_DEV "/dev/serial0"


#define ROBOTBOARD_REV2B

#ifdef DISABLE_SPI
#define MAP_DIR "/home/hrst/pulu/robotsoft/maps/"
#else
#define MAP_DIR "/home/pulu/maps/"
#endif
