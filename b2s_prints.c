#include <stdint.h>
#include <stdio.h>
#include "api_board_to_soft.h"

void print_test_msg1(void* m)
{
	test_msg1_t* mm = m;
	printf("a =%u  b =%u  c=%u  d=%u\n", mm->a, mm->b, mm->c, mm->d);
}

void print_test_msg2(void* m)
{
	test_msg2_t* mm = m;

	int i;
	for(i=0; i < sizeof(mm->buf); i++)
	{
		printf("%02x", mm->buf[i]);
		if(i%4 == 3) printf(" ");
		if(i%16 == 15) printf(" ");
		if(i%32 == 31) printf("\n");
	}
	printf("\n");
}

void print_test_msg3(void* m)
{
	test_msg3_t* mm = m;

	int i;
	for(i=0; i < 256; i++)
	{
		printf("%02x", mm->buf[i]);
		if(i%4 == 3) printf(" ");
		if(i%16 == 15) printf(" ");
		if(i%32 == 31) printf("\n");
	}
	printf("\n...\n");
	for(i=sizeof(mm->buf)-256; i < sizeof(mm->buf); i++)
	{
		printf("%02x", mm->buf[i]);
		if(i%4 == 3) printf(" ");
		if(i%16 == 15) printf(" ");
		if(i%32 == 31) printf("\n");
	}
	printf("\n");

}

void print_pwr_status(void* m)
{
	pwr_status_t* mm = m;

	printf("Battery %.3f V (%u %%), charger input %.2f V, charging current phase A: %.2f A, phase B: %.2f, total %.2f\n", 
		(float)mm->bat_mv/1000.0, mm->bat_percent, (float)mm->charger_input_mv/1000.0,
		(float)mm->pha_charging_current_ma/1000.0, (float)mm->phb_charging_current_ma/1000.0,
		(float)mm->pha_charging_current_ma/1000.0+(float)mm->phb_charging_current_ma/1000.0);
}

void print_tof_raw_dist(void* m)
{
	tof_raw_dist_t* mm = m;

	printf("Raw distances sensor %u. Midpoint: %u\n", mm->sensor_idx, mm->dist[30*160+80]);
}

void print_tof_raw_ampl8(void* m)
{
	tof_raw_ampl8_t* mm = m;

	printf("Raw amplitudes sensor %u. Midpoint: %u\n", mm->sensor_idx, mm->ampl[30*160+80]);
}

