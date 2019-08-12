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

#ifndef ROUTING_H
#define ROUTING_H

typedef struct
{
	int x;
	int y;
} route_xy_t;

typedef struct route_unit_t route_unit_t;

struct route_unit_t
{
	route_xy_t loc;
	int backmode;
	route_unit_t* prev;
	route_unit_t* next;
};



void clear_route(route_unit_t **route);
int search_route(route_unit_t **route, float start_ang, int start_x_mm, int start_y_mm, int end_x_mm, int end_y_mm, int reverse);

#define MINIMAP_SIZE 768
#define MINIMAP_MIDDLE 384
extern uint32_t minimap[MINIMAP_SIZE][MINIMAP_SIZE/32 + 1];


int minimap_find_mapping_dir(float ang_now, int32_t* x, int32_t* y, int32_t desired_x, int32_t desired_y, int* back);

int check_direct_route(int32_t start_ang, int start_x, int start_y, int end_x, int end_y);
int check_direct_route_non_turning(int start_x, int start_y, int end_x, int end_y);
int check_direct_route_mm(int32_t start_ang, int start_x, int start_y, int end_x, int end_y);
int check_direct_route_non_turning_mm(int start_x, int start_y, int end_x, int end_y);
int check_turn_mm(int32_t start_ang, int start_x, int start_y, int end_x, int end_y);
int test_robot_turn_mm(int start_x, int start_y, float start_ang_rad, float end_ang_rad);
int check_direct_route_non_turning_hitcnt_mm(int start_x, int start_y, int end_x, int end_y);
int check_direct_route_hitcnt_mm(int32_t start_ang, int start_x, int start_y, int end_x, int end_y);

#include "voxmap.h"
void voxmap_to_routing_pages(voxmap_t* vm);

void load_routing_pages();
void manage_routing_page_saves();

void routing_unit_coords(int mm_x, int mm_y, int* unit_x, int* unit_y);
void mm_from_routing_unit_coords(int unit_x, int unit_y, int* mm_x, int* mm_y);

#endif
