#pragma once
#include <stdint.h>

void spi_comm_thread_quit();
void* spi_comm_thread();
uint8_t* spi_rx_pop();
int subscribe_to(uint64_t* subs_vector);

int spi_init_cmd_queue();
void* spi_init_cmd(uint8_t msgid);
int spi_send_queue();

int is_error();
int ack_error();


void simulate_crc_err_on_rx();
void simulate_crc_err_on_tx();


#define ADD_SUB(vect, msgid) do{vect[msgid/64] |= 1ULL<<(msgid - msgid/64);}while(0)
#define REM_SUB(vect, msgid) do{vect[msgid/64] &= ~(1ULL<<(msgid - msgid/64));}while(0)
