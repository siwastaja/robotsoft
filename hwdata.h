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



*/

#ifndef HWDATA_H
#define HWDATA_H

#include <stdint.h>
#include "datatypes.h"

int parse_uart_msg(uint8_t* buf, int msgid, int len);
lidar_scan_t* get_basic_lidar();
lidar_scan_t* get_significant_lidar();
extern lidar_scan_t* latest_lidar;


sonar_point_t* get_sonar();

void send_keepalive();
void release_motors();

void move_to(int32_t x, int32_t y, int8_t backmode, int id, int speedlimit, int accurate_turn);

void correct_robot_pos(int32_t da, int32_t dx, int32_t dy, int id);
void set_hw_obstacle_avoidance_margin(int mm);
void set_robot_pos(int32_t na, int32_t nx, int32_t ny);
void do_compass_round();
void hw_find_charger();
void daiju_mode(int on);
void turn_and_go_abs_rel(int32_t ang_abs, int fwd_rel, int speedlimit, int accurate_turn);
void turn_and_go_rel_rel(int32_t ang_rel, int fwd_rel, int speedlimit, int accurate_turn);
void stop_movement();
void limit_speed(int speedlimit);
void prevent_3dtoffing();

void send_motcon_pid(uint8_t i_max, uint8_t feedfwd, uint8_t p, uint8_t i, uint8_t d);

// These four lines are exposed here now because of a ugly temporary "bugfix", in do_map_lidars_new_quick in mapping.c.
#define SIGNIFICANT_LIDAR_RING_BUF_LEN 32
#define LIDAR_RING_BUF_LEN 32
extern lidar_scan_t lidars[LIDAR_RING_BUF_LEN];
extern lidar_scan_t significant_lidars[SIGNIFICANT_LIDAR_RING_BUF_LEN];


#endif
