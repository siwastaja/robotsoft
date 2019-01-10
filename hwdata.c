/*
	PULUROBOT RN1-HOST Computer-on-RobotBoard main software

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



	Hardware (embedded, i.e., rn-1brain) access layer.
	This module is one level up from uart.c
	Provides abstract functions for giving commands to the microcontroller
	Parses data coming from the microcontroller.

*/


#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h> // for mutexing current coords.
#include <string.h>

#include "mapping.h"
#include "datatypes.h"
#include "hwdata.h"


#define SONAR_RING_BUF_LEN 128

extern double subsec_timestamp();

extern int verbose_mode;

int lidar_wr = 0;
int lidar_rd = 0;
int significant_lidar_wr = 0;
int significant_lidar_rd = 0;
int sonar_wr = 0;
int sonar_rd = 0;

lidar_scan_t* latest_lidar;
lidar_scan_t lidars[LIDAR_RING_BUF_LEN];
lidar_scan_t significant_lidars[SIGNIFICANT_LIDAR_RING_BUF_LEN];
sonar_point_t sonars[SONAR_RING_BUF_LEN];
xymove_t cur_xymove;

int32_t cur_pos_invalid_for_3dtof = 0;

int32_t hwdbg[10];

lidar_scan_t* get_basic_lidar()
{
	return 0;
}

lidar_scan_t* get_significant_lidar()
{
	return 0;
}


sonar_point_t* get_sonar()
{
	return 0;
}

extern int32_t cur_ang, cur_x, cur_y;
extern double robot_pos_timestamp;
pthread_mutex_t cur_pos_mutex = PTHREAD_MUTEX_INITIALIZER;
int update_robot_pos(int32_t ang, int32_t x, int32_t y)
{
	return 0;
}

#define sq(x) ((x)*(x))

#define I16FROMBUFLE(b_, s_)  ( ((uint16_t)b_[(s_)+1]<<8) | ((uint16_t)b_[(s_)+0]<<0) )
#define I32FROMBUFLE(b_, s_)  ( ((uint32_t)b_[(s_)+3]<<24) | ((uint32_t)b_[(s_)+2]<<16) | ((uint32_t)b_[(s_)+1]<<8) | ((uint32_t)b_[(s_)+0]<<0) )

void send_keepalive()
{
}

void release_motors()
{
}

void move_to(int32_t x, int32_t y, int8_t backmode, int id, int speedlimit, int accurate_turn)
{
	printf("Move(%d,%d),back=%d,id=%d, speedlim=%d\n", x,y,backmode,id, speedlimit);
}

void turn_and_go_abs_rel(int32_t ang_abs, int fwd_rel, int speedlimit, int accurate_turn)
{
	printf("Turn & go abs %d, rel %d\n", ang_abs/ANG_1_DEG, fwd_rel);
}

void turn_and_go_rel_rel(int32_t ang_rel, int fwd_rel, int speedlimit, int accurate_turn)
{
	printf("Turn & go rel %d, rel %d\n", ang_rel/ANG_1_DEG, fwd_rel);
}

void limit_speed(int speedlimit)
{
}

void stop_movement()
{
	printf("stop_movement()\n");
}

void send_motcon_pid(uint8_t i_max, uint8_t feedfwd, uint8_t p, uint8_t i, uint8_t d)
{
	printf("INFO: send_motcon_pid: i_max=%3d  feedfwd=%3d  p=%3d  i=%3d  d=%3d\n", i_max, feedfwd, p, i, d);
}


void correct_robot_pos(int32_t da, int32_t dx, int32_t dy, int id)
{
	dx<<=2;
	dy<<=2;

	if(dx < -32767 || dx > 32767 || dy < -32767 || dy > 32767 || id < 0 || id > 127)
	{
		printf("ERROR: out of range coords or id in correct_robot_pos()\n");
		return;
	}

	da *= -1; // Robot angles are opposite to those of trigonometric funtions.
}

void set_robot_pos(int32_t na, int32_t nx, int32_t ny)
{
	printf("Setting robot pos to ang=%d, x=%d, y=%d\n", na>>16, nx, ny);
}


void set_hw_obstacle_avoidance_margin(int mm)
{
}

void do_compass_round()
{
}

void hw_find_charger()
{
}

// Last resort when routefinding is stuck; robot randomly goes wherever it can.
void daiju_mode(int on)
{
}
