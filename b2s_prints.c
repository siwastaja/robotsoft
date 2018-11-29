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

void print_tof_raw_ambient8(void* m)
{

}

void print_tof_diagnostics(void* m)
{
	tof_diagnostics_t* mm = m;

	printf("TOF diagnostics: sensor_idx=%u, temperature=%.1f C. Timing data:\n",
		mm->sensor_idx, mm->temperature/10.0);

	for(int i=0; i<32; i++)
	{
		printf("%d:%.1f ", i, (float)mm->timestamps[i]/10.0);
	}
	printf("\n");
	printf("Time deltas to:\n");
	for(int i=1; i<32; i++)
	{
		printf(">%d:%.1f ", i, (float)(mm->timestamps[i]-mm->timestamps[i-1])/10.0);
	}
	printf("\n");
	printf("dbg_i32:\n");
	for(int i=0; i<8; i++)
	{
		printf("[%d] %11d  ", i, mm->dbg_i32[i]);
	}
	printf("\n");
}

void print_tof_raw_img(void* m)
{
}

void print_hw_pose(void* m)
{
	hw_pose_t *mm = m;

	printf("HW pose  ang=%5.1f deg  x=%8d mm  y=%8d mm\n", ANG32TOFDEG(mm->ang), mm->x, mm->y);
}

void print_drive_diag(void* m)
{
	drive_diag_t *mm = m;

	printf("Drive diagnostics  ang_err=%5.2f deg  x=%d  y=%d\n", ANG_I32TOFDEG(mm->ang_err), mm->x, mm->y);
}


