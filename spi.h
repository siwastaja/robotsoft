#pragma once
#include <stdint.h>

void spi_comm_thread_quit();
void* spi_comm_thread();
uint8_t* spi_rx_pop();

