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


	Routefinding/routeplanning module

	A modified version of Theta Star algorithm is implemented. This algorithm
	takes the robot shape into account: any shape is programmable in the discrete
	lookup table! Shape lookup generation code is supplied for any rectangular shape.

	This algorithm understand how the robot needs to turn around its origin fairly
	well. So, in addition to line-of-sight checks, turning check is performed as well.

	Backing off mid-route is not supported, but it's a very rare case anyway (mostly
	only seen in puzzles, not real world). Backing off in the beginning of the route
	works well. This also works mid route in case the robot gets stuck, as
	the routing will be redone.

	A 60x speed increase has already been optimized in compared to the first dumb
	implementation,	but I'm sure there's still a lot to do.

	One "routing unit" is 40x40mm.

*/

#define ROUTING_3D_FORGIVENESS 0
#define AVOID_3D_THINGS

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <inttypes.h>

#include "mapping.h"
#include "routing.h"
#include "uthash.h"
#include "utlist.h"

#ifndef M_PI
#define M_PI 3.141592653589793238
#endif

#ifdef PULU1
float main_robot_xs = 300.0;
float main_robot_ys = 330.0;
float main_robot_middle_to_lidar = -90.0;

#elif defined(DELIVERY_BOY)
float main_robot_xs = 650.0 + 20.0;
float main_robot_ys = 480.0 + 20.0;
float main_robot_middle_to_lidar = -180.0;

#else
float main_robot_xs = 524.0;
float main_robot_ys = 480.0;
float main_robot_middle_to_lidar = -183.0;
#endif

static void wide_search_mode();
static void normal_search_mode();
static void tight_search_mode();

world_t* routing_world;

typedef struct search_unit_t search_unit_t;
struct search_unit_t
{
	route_xy_t loc;
	float g;
	float f;
	int direction;

	search_unit_t* parent;

	UT_hash_handle hh;
};

#define sq(x) ((x)*(x))
#define MAX_F 99999999999999999.9

float robot_shape_x_len;
#define ROBOT_SHAPE_WINDOW 32
uint32_t robot_shapes[32][ROBOT_SHAPE_WINDOW];


static int check_hit(int x, int y, int direction)
{
//	printf("check_hit(%d, %d, %d)\n", x, y, direction);
	for(int chk_x=0; chk_x<ROBOT_SHAPE_WINDOW; chk_x++)
	{
		int pageidx_x, pageidx_y, pageoffs_x, pageoffs_y;
		page_coords_from_unit_coords(x-ROBOT_SHAPE_WINDOW/2+chk_x, y-ROBOT_SHAPE_WINDOW/2, &pageidx_x, &pageidx_y, &pageoffs_x, &pageoffs_y);

		int yoffs = pageoffs_y/32;
		int yoffs_remain = pageoffs_y - yoffs*32;

		if(!routing_world->rpages[pageidx_x][pageidx_y]) // out of bounds (not allocated) - give up instantly
		{
			printf("rpages[%d][%d] not allocated\n", pageidx_x, pageidx_y);
			printf("x = %d  y = %d  direction = %d\n", x, y, direction);
			//exit(1);
			return 1;
		}

		// Now the quick comparison, which could be even faster, but we don't want to mess up the compatibility between
		// big endian / little endian systems.

		uint64_t shape = (uint64_t)robot_shapes[direction][chk_x] << (32-yoffs_remain);

		if((((uint64_t)routing_world->rpages[pageidx_x][pageidx_y]->obst_u32[pageoffs_x][yoffs]<<32) |
		   (uint64_t)routing_world->rpages[pageidx_x][pageidx_y]->obst_u32[pageoffs_x][yoffs+1])
		      & shape)
		{
			return 1;
		}
	}

	return 0;
}

static int check_hit_hitcnt(int x, int y, int direction)
{
//	printf("check_hit(%d, %d, %d)\n", x, y, direction);

	int hit_cnt = 0;

	for(int chk_x=0; chk_x<ROBOT_SHAPE_WINDOW; chk_x++)
	{
		int pageidx_x, pageidx_y, pageoffs_x, pageoffs_y;
		page_coords_from_unit_coords(x-ROBOT_SHAPE_WINDOW/2+chk_x, y-ROBOT_SHAPE_WINDOW/2, &pageidx_x, &pageidx_y, &pageoffs_x, &pageoffs_y);

		int yoffs = pageoffs_y/32;
		int yoffs_remain = pageoffs_y - yoffs*32;

		if(!routing_world->rpages[pageidx_x][pageidx_y]) // out of bounds (not allocated) - give up instantly
		{
			printf("rpages[%d][%d] not allocated\n", pageidx_x, pageidx_y);
			printf("x = %d  y = %d  direction = %d\n", x, y, direction);
			//exit(1);
			return 999;
		}

		// Now the quick comparison, which could be even faster, but we don't want to mess up the compatibility between
		// big endian / little endian systems.

		uint64_t shape = (uint64_t)robot_shapes[direction][chk_x] << (32-yoffs_remain);

		if((((uint64_t)routing_world->rpages[pageidx_x][pageidx_y]->obst_u32[pageoffs_x][yoffs]<<32) |
		   (uint64_t)routing_world->rpages[pageidx_x][pageidx_y]->obst_u32[pageoffs_x][yoffs+1])
		      & shape)
		{
			hit_cnt++;
		}
	}

	return hit_cnt;
}


static int test_robot_turn(int x, int y, float start, float end)
{
	int cw = 0;

	while(start >= 2.0*M_PI) start -= 2.0*M_PI;
	while(start < 0.0) start += 2.0*M_PI;

	while(end >= 2.0*M_PI) end -= 2.0*M_PI;
	while(end < 0.0) end += 2.0*M_PI;

	// Calc for CCW (positive angle):
	float da = end - start;
	while(da >= 2.0*M_PI) da -= 2.0*M_PI;
	while(da < 0.0) da += 2.0*M_PI;

	if(da > M_PI)
	{
		// CCW wasn't fine, turn CW
		cw = 1;
		da = start - end;
		while(da >= 2.0*M_PI) da -= 2.0*M_PI;
		while(da < 0.0) da += 2.0*M_PI;
	}

	int dir_cur = (start/(2.0*M_PI) * 32.0);
	int dir_end = (end/(2.0*M_PI) * 32.0);

	if(dir_cur < 0) dir_cur = 0; else if(dir_cur > 31) dir_cur = 31;
	if(dir_end < 0) dir_end = 0; else if(dir_end > 31) dir_end = 31;

	while(dir_cur != dir_end)
	{
		if(check_hit(x, y, dir_cur))
			return 0;

		if(cw) dir_cur--; else dir_cur++;

		if(dir_cur < 0) dir_cur = 31;
		else if(dir_cur > 31) dir_cur = 0;

	}

	return 1;
}

static int test_robot_turn_hitcnt(int x, int y, float start, float end)
{
	int cw = 0;

	while(start >= 2.0*M_PI) start -= 2.0*M_PI;
	while(start < 0.0) start += 2.0*M_PI;

	while(end >= 2.0*M_PI) end -= 2.0*M_PI;
	while(end < 0.0) end += 2.0*M_PI;

	// Calc for CCW (positive angle):
	float da = end - start;
	while(da >= 2.0*M_PI) da -= 2.0*M_PI;
	while(da < 0.0) da += 2.0*M_PI;

	if(da > M_PI)
	{
		// CCW wasn't fine, turn CW
		cw = 1;
		da = start - end;
		while(da >= 2.0*M_PI) da -= 2.0*M_PI;
		while(da < 0.0) da += 2.0*M_PI;
	}

	int dir_cur = (start/(2.0*M_PI) * 32.0);
	int dir_end = (end/(2.0*M_PI) * 32.0);

	if(dir_cur < 0) dir_cur = 0; else if(dir_cur > 31) dir_cur = 31;
	if(dir_end < 0) dir_end = 0; else if(dir_end > 31) dir_end = 31;

	int hitcnt = 0;

	while(dir_cur != dir_end)
	{
		hitcnt += check_hit_hitcnt(x, y, dir_cur);

		if(cw) dir_cur--; else dir_cur++;

		if(dir_cur < 0) dir_cur = 31;
		else if(dir_cur > 31) dir_cur = 0;

	}

	return hitcnt;
}

static int line_of_sight(route_xy_t p1, route_xy_t p2)
{
	int dx = p2.x - p1.x;
	int dy = p2.y - p1.y;

	float step = ((robot_shape_x_len-10.0)/MAP_UNIT_W);

	float len = sqrt(sq(dx) + sq(dy));

	int terminate = 0;

	float pos = step/2.0;
	if(pos > len) {pos = len; terminate = 1;}

	float ang = atan2(dy, dx);
	if(ang < 0.0) ang += 2.0*M_PI;
	int dir = (ang/(2.0*M_PI) * 32.0)+0.5;
	if(dir < 0) dir = 0; else if(dir > 31) dir = 31;

//	printf("ang = %.4f  dir = %d \n", ang, dir);

	while(1)
	{
		int x = (cos(ang)*pos + (float)p1.x)+0.5;
		int y = (sin(ang)*pos + (float)p1.y)+0.5;

//		printf("check_hit(%d, %d, %d) = ", x, y, dir);

		if(check_hit(x, y, dir))
		{
//			printf("1 !\n");
			return 0;
		}
//		printf("0\n");

		if(terminate) break;
		pos += step;
		if(pos > len)
		{
			pos = len;
			terminate = 1;
		}
	}

	return 1;
}

static int line_of_sight_hitcnt(route_xy_t p1, route_xy_t p2)
{
	int dx = p2.x - p1.x;
	int dy = p2.y - p1.y;

	float step = ((robot_shape_x_len-10.0)/MAP_UNIT_W);

	float len = sqrt(sq(dx) + sq(dy));

	int terminate = 0;

	float pos = step/2.0;
	if(pos > len) {pos = len; terminate = 1;}

	float ang = atan2(dy, dx);
	if(ang < 0.0) ang += 2.0*M_PI;
	int dir = (ang/(2.0*M_PI) * 32.0)+0.5;
	if(dir < 0) dir = 0; else if(dir > 31) dir = 31;

	int hit_cnt = 0;

//	printf("ang = %.4f  dir = %d \n", ang, dir);

	while(1)
	{
		int x = (cos(ang)*pos + (float)p1.x)+0.5;
		int y = (sin(ang)*pos + (float)p1.y)+0.5;

//		printf("check_hit(%d, %d, %d) = ", x, y, dir);

		hit_cnt += check_hit_hitcnt(x, y, dir);
//		printf("0\n");

		if(terminate) break;
		pos += step;
		if(pos > len)
		{
			pos = len;
			terminate = 1;
		}
	}

	return hit_cnt;
}


uint32_t minimap[MINIMAP_SIZE][MINIMAP_SIZE/32 + 1];

void dbg_save_minimap()
{
	FILE* f = fopen("minimap.data", "w");
	for(int yy=0; yy<MINIMAP_SIZE; yy++)
	{
		for(int xx=0; xx<MINIMAP_SIZE; xx++)
		{
			int yidx = yy/32;
			int yoffs = yy - yidx*32;

			int val = (minimap[xx][yidx]&(1<<(32-yoffs)))?255:0;

			fputc(val, f);
			fputc(val, f);
			fputc((xx==MINIMAP_MIDDLE&&yy==MINIMAP_MIDDLE)?255:0, f);
		}
	}
	fclose(f);
}

static int minimap_check_hit(int x, int y, int direction)
{
//	printf("minimap_check_hit(%d, %d, %d)\n", x, y, direction);
	for(int chk_x=0; chk_x<ROBOT_SHAPE_WINDOW; chk_x++)
	{
		int xx = x-ROBOT_SHAPE_WINDOW/2+chk_x + MINIMAP_MIDDLE;
		int yy = y-ROBOT_SHAPE_WINDOW/2 + MINIMAP_MIDDLE;

		int yoffs = yy/32;
		int yoffs_remain = yy - yoffs*32;

		if( xx < 0 || xx > 767 || yoffs < 0 || yoffs > 768/32)
		{
			printf("WARN: minimap_check_hit(): illegal coords xx=%d, yoffs=%d, yoffs_remain=%d\n", xx, yoffs, yoffs_remain);
			return 1;
		}

		// Now the quick comparison, which could be even faster, but we don't want to mess up the compatibility between
		// big endian / little endian systems.

		uint64_t shape = (uint64_t)robot_shapes[direction][chk_x] << (32-yoffs_remain);

		if((((uint64_t)minimap[xx][yoffs]<<32) | (uint64_t)minimap[xx][yoffs+1])  & shape)
		{
//			printf("DBG: minimap_check_hit() HIT xx=%d, yoffs=%d, yoffs_remain=%d, shape=%016" PRIx64 " minimap=%016" PRIx64 "\n", xx, yoffs, yoffs_remain, shape, (((uint64_t)minimap[xx][yoffs]<<32) | (uint64_t)minimap[xx][yoffs+1]));
			return 1;
		}
//		printf("DBG: minimap_check_hit() nonhit xx=%d, yoffs=%d, yoffs_remain=%d\n", xx, yoffs, yoffs_remain);
	}

	return 0;
}


static int minimap_test_robot_turn(int x, int y, float start, float end)
{
	int cw = 0;

	while(start >= 2.0*M_PI) start -= 2.0*M_PI;
	while(start < 0.0) start += 2.0*M_PI;

	while(end >= 2.0*M_PI) end -= 2.0*M_PI;
	while(end < 0.0) end += 2.0*M_PI;

	// Calc for CCW (positive angle):
	float da = end - start;
	while(da >= 2.0*M_PI) da -= 2.0*M_PI;
	while(da < 0.0) da += 2.0*M_PI;

	if(da > M_PI)
	{
		// CCW wasn't fine, turn CW
		cw = 1;
		da = start - end;
		while(da >= 2.0*M_PI) da -= 2.0*M_PI;
		while(da < 0.0) da += 2.0*M_PI;
	}

	int dir_cur = (start/(2.0*M_PI) * 32.0);
	int dir_end = (end/(2.0*M_PI) * 32.0);

	if(dir_cur < 0) dir_cur = 0; else if(dir_cur > 31) dir_cur = 31;
	if(dir_end < 0) dir_end = 0; else if(dir_end > 31) dir_end = 31;

	while(dir_cur != dir_end)
	{
		if(minimap_check_hit(x, y, dir_cur))
			return 0;

		if(cw) dir_cur--; else dir_cur++;

		if(dir_cur < 0) dir_cur = 31;
		else if(dir_cur > 31) dir_cur = 0;
	}

	return 1;
}

static int minimap_line_of_sight(route_xy_t p1, route_xy_t p2, int reverse)
{
	int dx = p2.x - p1.x;
	int dy = p2.y - p1.y;

	float step = ((robot_shape_x_len-10.0)/MAP_UNIT_W);

	float len = sqrt(sq(dx) + sq(dy));

	float pos = 0.0;
	int terminate = 0;

	float ang = atan2(dy, dx);
	if(ang < 0.0) ang += 2.0*M_PI;
	if(ang > 2.0*M_PI) ang -= 2.0*M_PI;

	int dir = (ang/(2.0*M_PI) * 32.0)+0.5;
	if(reverse) dir += 16;
	if(dir > 31) dir -= 32;
	if(dir < 0) dir = 0; else if(dir > 31) dir = 31;

	while(1)
	{
		int x = (cos(ang)*pos + (float)p1.x)+0.5;
		int y = (sin(ang)*pos + (float)p1.y)+0.5;

		printf("DBG: minimap_line_of_sight(): x=%d, y=%d, dir=%d:", x, y, dir);

		if(minimap_check_hit(x, y, dir))
		{
			printf(" HIT\n");
			return 0;
		}
		printf(" ok\n");
		if(terminate) break;
		pos += step;
		if(pos > len)
		{
			pos = len;
			terminate = 1;
		}
	}

	return 1;
}

static int minimap_test_endpoint(route_xy_t p1, route_xy_t p2, int reverse)
{
	int dx = p2.x - p1.x;
	int dy = p2.y - p1.y;

	float len = sqrt(sq(dx) + sq(dy));
	float pos = len;

	float ang = atan2(dy, dx);
	if(ang < 0.0) ang += 2.0*M_PI;
	if(ang > 2.0*M_PI) ang -= 2.0*M_PI;

	int dir = (ang/(2.0*M_PI) * 32.0)+0.5;
	if(reverse) dir += 16;
	if(dir > 31) dir -= 32;
	if(dir < 0) dir = 0; else if(dir > 31) dir = 31;

	int x = (cos(ang)*pos + (float)p1.x)+0.5;
	int y = (sin(ang)*pos + (float)p1.y)+0.5;

	if(minimap_check_hit(x, y, dir))
	{
		return 0;
	}

	return 1;
}


#define MAX_CANGOS 500

int minimap_find_mapping_dir(world_t *w, float ang_now, int32_t* x, int32_t* y, int32_t desired_x, int32_t desired_y, int* back)
{
	extern int32_t cur_ang;
	extern int cur_x, cur_y;

	routing_world = w;

	wide_search_mode();

	#define NUM_FWDS 10
	const float fwds[NUM_FWDS] = {1000.0, 750.0, 500.0, 400.0, -500.0, -300.0, 200.0, -150.0, 100.0, -100.0};

	int num_cango_places = 0;
	route_xy_t cango_places[MAX_CANGOS];
	route_xy_t internal_cango_places[MAX_CANGOS];
	int backs[MAX_CANGOS];
	int disagrees = 0;

	gen_all_routing_pages(w, 0);

	int in_tight_spot = 0;

	route_xy_t start = {0, 0};

	dbg_save_minimap();
	for(int tries=0; tries < 3; tries++)
	{
		for(float ang_to = 0; ang_to < DEGTORAD(359.9); ang_to += (tries==0)?(DEGTORAD(10.0)):(DEGTORAD(5.0)))
		{
			if(minimap_test_robot_turn(0, 0, ang_now, ang_to))
			{
				printf("Minimap: can turn %.1f deg -> %.1f deg\n", RADTODEG(ang_now), RADTODEG(ang_to));
				for(int f=0; f < NUM_FWDS; f++)
				{
					float fwd_len = fwds[f];
					route_xy_t end = {(int)(cos(ang_to)*fwd_len/(float)MAP_UNIT_W),
							  (int)(sin(ang_to)*fwd_len/(float)MAP_UNIT_W)};

					if(minimap_line_of_sight(start, end, fwd_len<0.0))
					{
						int dest_x = cos(ang_to)*fwd_len;
						int dest_y = sin(ang_to)*fwd_len;

						printf("Minimap: can go to (%d, %d), check actual map...", dest_x, dest_y);
						if(check_direct_route(cur_ang, MM_TO_UNIT(cur_x), MM_TO_UNIT(cur_y), 
							MM_TO_UNIT(dest_x+cur_x), MM_TO_UNIT(dest_y+cur_y)))
						{
							printf(" Agreed.\n");
							internal_cango_places[num_cango_places] = end;
							cango_places[num_cango_places].x = dest_x; cango_places[num_cango_places].y = dest_y;
							if(fwd_len < 0.0) backs[num_cango_places] = 1; else backs[num_cango_places] = 0;
							num_cango_places++;
							if(num_cango_places > MAX_CANGOS-1)
								goto PLACE_LIST_FULL;
						}
						else
						{
							disagrees++;
							printf(" Disagreed.\n");
						}

					}
					else
					{
						printf("minimap_find_mapping_dir: robot cannot go %.1f mm to %.1f deg\n", fwd_len, RADTODEG(ang_to));
					}
				}
			}
			else
			{
				printf("minimap_find_mapping_dir: robot cannot turn %.1f deg -> %.1f deg\n", RADTODEG(ang_now), RADTODEG(ang_to));
			}
		}

		if(num_cango_places < 5)
		{
			in_tight_spot = 1;
			if(tries == 0)
			{
				printf("minimap_find_mapping_dir goes to tighter (normal) search mode to find more possibilities (%d so far).\n", num_cango_places);
				normal_search_mode();
			}
			else if(tries == 1)
			{
				printf("minimap_find_mapping_dir goes to the tightest (tight) search mode to find more possibilities (%d so far).\n", num_cango_places);
				tight_search_mode();
			}
		}
		else
			goto PLACE_LIST_DONE;
	}

	PLACE_LIST_DONE: ;
	PLACE_LIST_FULL: ;

	if(num_cango_places == 0)
	{
		return 0;
	}

	int64_t nearest;
	int nearest_i;

	if(in_tight_spot)
	{
		wide_search_mode();
		nearest = INT64_MAX;
		nearest_i = 0;
		int good_candidates = 0;
		for(int i=0; i < num_cango_places; i++)	
		{
			if(minimap_test_endpoint(start, internal_cango_places[i], backs[i]))
			{
				int64_t dist_sq = sq(cango_places[i].x - desired_x) + sq(cango_places[i].y - desired_y);
				if(dist_sq < nearest)
				{
					nearest = dist_sq;
					nearest_i = i;
					good_candidates++;
				}
			}
		}

		if(nearest == INT64_MAX)
		{
			printf("In a tight spot; tried to choose route leading to wider environment (from %d routes); couldn't find one, just choosing the closest to desired coords:\n", num_cango_places);
		}
		else
		{
			printf("In a tight spot; of (%d found from lidar only; %d agreed with map) possibilities, %d lead to wider environment, of which (%d, %d) is nearest the desired (%d, %d)\n",
				num_cango_places+disagrees, num_cango_places, good_candidates, cango_places[nearest_i].x, cango_places[nearest_i].y, desired_x, desired_y);
			*x = cango_places[nearest_i].x ; *y = cango_places[nearest_i].y; *back = backs[nearest_i];
			return 1 | ((in_tight_spot)?2:0);
		}
	}

	nearest = INT64_MAX;
	nearest_i = 0;
	for(int i=0; i < num_cango_places; i++)
	{	
		int64_t dist_sq = sq(cango_places[i].x - desired_x) + sq(cango_places[i].y - desired_y);
		if(backs[i]) dist_sq *= 2; // Make backing off less appealing
		if(dist_sq < nearest)
		{
			nearest = dist_sq;
			nearest_i = i;
		}
	}

	printf("(%d, %d) is nearest the desired (%d, %d) (%d found from lidar only; %d agreed with map)\n",
		cango_places[nearest_i].x, cango_places[nearest_i].y, desired_x, desired_y, num_cango_places+disagrees, num_cango_places);
	*x = cango_places[nearest_i].x ; *y = cango_places[nearest_i].y; *back = backs[nearest_i];

	return 1 | ((in_tight_spot)?2:0);
}



#define SHAPE_PIXEL(shape, x, y) { robot_shapes[shape][(x)] |= 1UL<<(31-y);}
int limits_x[ROBOT_SHAPE_WINDOW][2];
#define ABS(x) ((x >= 0) ? x : -x)
static void triangle_scanline(int x1, int y1, int x2, int y2)
{
	int sx, sy, dx1, dy1, dx2, dy2, x, y, m, n, k, cnt;

	sx = x2 - x1;
	sy = y2 - y1;

	if (sx > 0) dx1 = 1;
	else if (sx < 0) dx1 = -1;
	else dx1 = 0;

	if (sy > 0) dy1 = 1;
	else if (sy < 0) dy1 = -1;
	else dy1 = 0;

	m = ABS(sx);
	n = ABS(sy);
	dx2 = dx1;
	dy2 = 0;

	if (m < n)
	{
		m = ABS(sy);
		n = ABS(sx);
		dx2 = 0;
		dy2 = dy1;
	}

	x = x1; y = y1;
	cnt = m + 1;
	k = n / 2;

	while (cnt--)
	{
		if ((y >= 0) && (y < ROBOT_SHAPE_WINDOW))
		{
			if (x < limits_x[y][0]) limits_x[y][0] = x;
			if (x > limits_x[y][1]) limits_x[y][1] = x;
		}

		k += n;
		if (k < m)
		{
			x += dx2;
			y += dy2;
		}
		else
		{
			k -= m;
			x += dx1;
			y += dy1;
		}
	}
}

static void draw_triangle(int a_idx, int x0, int y0, int x1, int y1, int x2, int y2)
{
	int y;

	for (y = 0; y < ROBOT_SHAPE_WINDOW; y++)
	{
		limits_x[y][0] = 999999; // min X
		limits_x[y][1] = -999999; // max X
	}

	triangle_scanline(x0, y0, x1, y1);
	triangle_scanline(x1, y1, x2, y2);
	triangle_scanline(x2, y2, x0, y0);

	for (y = 0; y < ROBOT_SHAPE_WINDOW; y++)
	{
		if (limits_x[y][1] >= limits_x[y][0])
		{
			int x = limits_x[y][0];
			int len = 1 + limits_x[y][1] - limits_x[y][0];

			// Horizontal line.
			while (len--)
			{
				SHAPE_PIXEL(a_idx, x, y);
				x++;
			}
		}
	}
}

#define TODEG(x) ((360.0*x)/(2.0*M_PI))

int tight_shapes = 0;

static void draw_robot_shape(int a_idx, float ang)
{
	float o_x = (ROBOT_SHAPE_WINDOW/2.0)*(float)MAP_UNIT_W;
	float o_y = (ROBOT_SHAPE_WINDOW/2.0)*(float)MAP_UNIT_W;

	float robot_xs, robot_ys;

	float middle_xoffs; // from o_x, o_y to the robot middle point.
	float middle_yoffs = -0.0;
	middle_xoffs = main_robot_middle_to_lidar;

	#ifdef PULU1
	const float extra_x = 40.0, extra_y = 40.0;
	#else
	const float extra_x = 0.0, extra_y = 0.0;
	#endif

	if(tight_shapes == 2)
	{
		robot_xs = (main_robot_xs - 20.0 + extra_x);
		robot_ys = (main_robot_ys - 20.0 + extra_y);
	}
	else if(tight_shapes == 1)
	{
		robot_xs = (main_robot_xs + 20.0 + extra_x);
		robot_ys = (main_robot_ys + 20.0 + extra_y);
	}
	else if(tight_shapes == 0)
	{
		robot_xs = (main_robot_xs + 100.0 + extra_x);
		robot_ys = (main_robot_ys + 100.0 + extra_y);
	}
	else // wide
	{
		robot_xs = (main_robot_xs + 200.0 + extra_x);
		robot_ys = (main_robot_ys + 200.0 + extra_y);
	}

	robot_shape_x_len = robot_xs;


/*

	Y                         / positive angle
	^                       /
	|                     /
	|                   /
	+----> X            -------------> zero angle

	Corner numbers:

	1-----------------------2
	|                       |
	|                       |
	|                       |
	|                 O     |  --->
	|                       |
	|                       |
	|                       |
	4-----------------------3

	Vector lengths O->1, O->2, O->3, O->4 are called a,b,c,d, respectively.
	Angles of those vectors relative to O are called ang1, ang2, ang3, ang4.

*/	
	// Basic Pythagoras thingie
	float a = sqrt(sq(0.5*robot_xs - middle_xoffs) + sq(0.5*robot_ys + middle_yoffs));
	float b = sqrt(sq(0.5*robot_xs + middle_xoffs) + sq(0.5*robot_ys + middle_yoffs));
	float c = sqrt(sq(0.5*robot_xs + middle_xoffs) + sq(0.5*robot_ys - middle_yoffs));
	float d = sqrt(sq(0.5*robot_xs - middle_xoffs) + sq(0.5*robot_ys - middle_yoffs));

//	printf("a=%.1f b=%.1f c=%.1f d=%.1f\n", a,b,c,d);

	// The angles:
	float ang1 = M_PI - asin((0.5*robot_ys)/a);
	float ang2 = asin((0.5*robot_ys)/b);
	float ang3 = -1*asin((0.5*robot_ys)/c);
	float ang4 = -1*(M_PI - asin((0.5*robot_ys)/d));

//	printf("ang1=%.2f ang2=%.2f ang3=%.2f ang4=%.2f\n", ang1,ang2,ang3,ang4);
//	printf("(ang1=%.2f ang2=%.2f ang3=%.2f ang4=%.2f)\n", TODEG(ang1),TODEG(ang2),TODEG(ang3),TODEG(ang4));

	// Turn the whole robot:
	ang1 += ang;
	ang2 += ang;
	ang3 += ang;
	ang4 += ang;

	float x1 = cos(ang1)*a + o_x;
	float y1 = sin(ang1)*a + o_y;
	float x2 = cos(ang2)*b + o_x;
	float y2 = sin(ang2)*b + o_y;
	float x3 = cos(ang3)*c + o_x;
	float y3 = sin(ang3)*c + o_y;
	float x4 = cos(ang4)*d + o_x;
	float y4 = sin(ang4)*d + o_y;

	// "Draw" the robot in two triangles:

	// First, triangle 1-2-3
	draw_triangle(a_idx, x1/(float)MAP_UNIT_W, y1/(float)MAP_UNIT_W, x2/(float)MAP_UNIT_W, y2/(float)MAP_UNIT_W, x3/(float)MAP_UNIT_W, y3/(float)MAP_UNIT_W);
	// Second, triangle 1-3-4
	draw_triangle(a_idx, x1/(float)MAP_UNIT_W, y1/(float)MAP_UNIT_W, x3/(float)MAP_UNIT_W, y3/(float)MAP_UNIT_W, x4/(float)MAP_UNIT_W, y4/(float)MAP_UNIT_W);
}


static void gen_robot_shapes()
{
	// Generate lookup tables showing the shape of robot in mapping unit matrices in different orientations.
	// These are used in mapping to test whether the (x,y) coords result in some part of a robot hitting a wall.

        memset(robot_shapes, 0, sizeof(robot_shapes));

	for(int a=0; a<32; a++)
	{
		draw_robot_shape(a, ((float)a*2.0*M_PI)/32.0);

/*		printf("a = %d\n", a);

		for(int y = 0; y < ROBOT_SHAPE_WINDOW; y++)
		{
			for(int x = 0; x < ROBOT_SHAPE_WINDOW; x++)
			{
				printf(robot_shapes[a][x][y]?"# ":"  ");
			}
			printf("\n");
		}
*/
	}

}

static void wide_search_mode()
{
	tight_shapes = -1;
	gen_robot_shapes();	
}


static void normal_search_mode()
{
	tight_shapes = 0;
	gen_robot_shapes();	
}

static void tight_search_mode()
{
	tight_shapes = 1;
	gen_robot_shapes();	
}

static void extra_tight_search_mode()
{
	tight_shapes = 2;
	gen_robot_shapes();	
}

void clear_route(route_unit_t **route)
{
	route_unit_t *elt, *tmp;
	DL_FOREACH_SAFE(*route,elt,tmp)
	{
		DL_DELETE(*route,elt);
		free(elt);
	}
	*route = NULL;
}

static int search(route_unit_t **route, float start_ang, int start_x_mm, int start_y_mm, int end_x_mm, int end_y_mm)
{
	search_unit_t* closed_set = NULL;
	search_unit_t* open_set = NULL;

	clear_route(route);

	int s_x, s_y, e_x, e_y;
	unit_coords(start_x_mm, start_y_mm, &s_x, &s_y);
	unit_coords(end_x_mm, end_y_mm, &e_x, &e_y);

	while(start_ang >= 2.0*M_PI) start_ang -= 2.0*M_PI;
	while(start_ang < 0.0) start_ang += 2.0*M_PI;
	int start_dir = (start_ang/(2.0*M_PI) * 32.0)+0.5;

//	printf("Start %d,%d,  end %d,%d  start_ang=%f  start_dir=%d\n", s_x, s_y, e_x, e_y, start_ang, start_dir);


	search_unit_t* p_start = (search_unit_t*) malloc(sizeof(search_unit_t));
	memset(p_start, 0, sizeof(search_unit_t));

	p_start->loc.x = s_x;
	p_start->loc.y = s_y;
	p_start->direction = start_dir;
	p_start->parent = NULL;
	// g = 0
	p_start->f = sqrt((float)(sq(e_x-s_x) + sq(e_y-s_y)));

	HASH_ADD(hh, open_set, loc,sizeof(route_xy_t), p_start);

	int cnt = 0;

	while(HASH_CNT(hh, open_set) > 0)
	{
		cnt++;

		if(cnt > 50000)
		{
			printf("Giving up at cnt = %d\n", cnt);
			return 3;
		}

		// Find the lowest f score from open_set.
		search_unit_t* p_cur = NULL;
		float lowest_f = 2.0*MAX_F;
		for(search_unit_t* p_iter = open_set; p_iter != NULL; p_iter=p_iter->hh.next)
		{
			if(p_iter->f < lowest_f)
			{
				lowest_f = p_iter->f;
				p_cur = p_iter;
			}
		}

		if(p_cur == NULL)
		{
			printf("search() error: open set empty while finding lowest f score.\n");
			return 55;
		}

		if(p_cur->loc.x == e_x && p_cur->loc.y == e_y)
		{

			//printf("Solution found, cnt = %d\n", cnt);

			// solution found.

			// Reconstruct the path

			search_unit_t* p_recon = p_cur;
			while( (p_recon = p_recon->parent) )
			{
				route_unit_t* point = malloc(sizeof(route_unit_t));
				point->loc.x = p_recon->loc.x; point->loc.y = p_recon->loc.y;
				point->backmode = 0;
				DL_PREPEND(*route, point);
			}

			route_unit_t *rt = *route;
			while(rt->next && rt->next->next)
			{
				if(line_of_sight(rt->loc, rt->next->next->loc))
				{
//					printf("Deleting.\n");
					route_unit_t *tmp = rt->next;
					DL_DELETE(*route, tmp);
					free(tmp);
				}
				else
					rt = rt->next;
			}

			// Remove the first, because it's the starting point.
			route_unit_t *tm = *route;
			DL_DELETE(*route, tm);
			free(tm);

			// Free all memory.
			search_unit_t *p_del, *p_tmp;
			HASH_ITER(hh, closed_set, p_del, p_tmp)
			{
				HASH_DEL(closed_set, p_del);
				free(p_del);
			}
			HASH_ITER(hh, open_set, p_del, p_tmp)
			{
				HASH_DEL(open_set, p_del);
				free(p_del);
			}

			return 0;
		}

		// move from open to closed:
		HASH_DELETE(hh, open_set, p_cur);
		HASH_ADD(hh, closed_set, loc,sizeof(route_xy_t), p_cur);

		// For each neighbor
		for(int xx=-1; xx<=1; xx++)
		{
			for(int yy=-1; yy<=1; yy++)
			{
				search_unit_t* found;
				float new_g;
				float new_g_from_parent;
				if(xx == 0 && yy == 0) continue;

				search_unit_t* p_neigh;
				route_xy_t neigh_loc = {p_cur->loc.x + xx, p_cur->loc.y + yy};

				// Check if it's out-of-allowed area here:


				HASH_FIND(hh, closed_set, &neigh_loc,sizeof(route_xy_t), found);
				if(found)
					continue; // ignore neighbor that's in closed_set.


				// gscore for the neigbor: distance from the start to neighbor
				// is current unit's g score plus distance to the neighbor.
				new_g = p_cur->g + sqrt((float)(sq(p_cur->loc.x-neigh_loc.x) + sq(p_cur->loc.y-neigh_loc.y)));

				// for theta*:
				if(p_cur->parent)
					new_g_from_parent = p_cur->parent->g + sqrt((float)(sq(p_cur->parent->loc.x-neigh_loc.x) + sq(p_cur->parent->loc.y-neigh_loc.y)));


				HASH_FIND(hh, open_set, &neigh_loc,sizeof(route_xy_t), p_neigh);


				int direction_from_cur_parent = -1;
				int direction_from_neigh_parent = -1;

				if(p_cur->parent)  // Use this if possible
				{
					int dir_dx = neigh_loc.x - p_cur->parent->loc.x;
					int dir_dy = neigh_loc.y - p_cur->parent->loc.y;
					float ang = atan2(dir_dy, dir_dx);
					if(ang < 0.0) ang += 2.0*M_PI;
					int dir_parent = ang/(2.0*M_PI) * 32.0;
					if(dir_parent < 0) dir_parent = 0; else if(dir_parent > 31) dir_parent = 31;
					direction_from_cur_parent = dir_parent;
				}

				if(p_neigh && p_neigh->parent)  // secondary
				{
					int dir_dx = p_neigh->parent->loc.x - neigh_loc.x;
					int dir_dy = p_neigh->parent->loc.y - neigh_loc.y;
					float ang = atan2(dir_dy, dir_dx);
					if(ang < 0.0) ang += 2.0*M_PI;
					int dir_parent = ang/(2.0*M_PI) * 32.0;
					if(dir_parent < 0) dir_parent = 0; else if(dir_parent > 31) dir_parent = 31;
					direction_from_neigh_parent = dir_parent;
				}

				int direction = 0;
				if(direction_from_cur_parent < 0 && direction_from_neigh_parent < 0)  // if the previous two don't work out.
				{
					if(xx==1 && yy==1)        direction = 1*4; 
					else if(xx==0 && yy==1)   direction = 2*4; 
					else if(xx==-1 && yy==1)  direction = 3*4; 
					else if(xx==-1 && yy==0)  direction = 4*4; 
					else if(xx==-1 && yy==-1) direction = 5*4; 
					else if(xx==0 && yy==-1)  direction = 6*4; 
					else if(xx==1 && yy==-1)  direction = 7*4; 
				}
				else
				{
					if(direction_from_cur_parent >= 0)
						direction = direction_from_cur_parent;
					else
						direction = direction_from_neigh_parent;
				}

				// If this is the first neighbor search, test if the robot can turn:
				if(cnt == 1)
				{
					if(!test_robot_turn(p_cur->loc.x, p_cur->loc.y, start_ang, ((float)direction/32.0)*2.0*M_PI*direction))
					{
//						printf("Robot cannot turn to direction %d\n", direction);
						continue;
					}
				}

				if(check_hit(neigh_loc.x, neigh_loc.y, direction))
				{

					if(direction_from_neigh_parent != -1)
					{
						// try another thing before giving up:
						direction = direction_from_neigh_parent;

						if(check_hit(neigh_loc.x, neigh_loc.y, direction))
							continue;
					}
					else
						continue;

				}

				if(!p_neigh)
				{
					p_neigh = (search_unit_t*) malloc(sizeof(search_unit_t));
					memset(p_neigh, 0, sizeof(search_unit_t));
					p_neigh->loc.x = neigh_loc.x; p_neigh->loc.y = neigh_loc.y;
					HASH_ADD(hh, open_set, loc,sizeof(route_xy_t), p_neigh);

					p_neigh->direction = direction;
					p_neigh->parent = p_cur;
					p_neigh->g = new_g;
					p_neigh->f = new_g + sqrt((float)(sq(e_x-neigh_loc.x) + sq(e_y-neigh_loc.y)));

				}
				else
				{
					if(p_cur->parent && line_of_sight(p_cur->parent->loc, p_neigh->loc)) // Theta* style near-optimum (probably shortest) path
					{
						if(new_g_from_parent < p_neigh->g)
						{
							p_neigh->direction = direction;
							p_neigh->parent = p_cur->parent;
							p_neigh->g = new_g_from_parent;
							p_neigh->f = new_g_from_parent + sqrt((float)(sq(e_x-neigh_loc.x) + sq(e_y-neigh_loc.y)));
						}
					}
					else if(new_g < p_neigh->g)  // A* style path shorter than before.
					{
						p_neigh->direction = direction;
						p_neigh->parent = p_cur;
						p_neigh->g = new_g;
						p_neigh->f = new_g + sqrt((float)(sq(e_x-neigh_loc.x) + sq(e_y-neigh_loc.y)));
					}
				}
			}


		}		
	}

	search_unit_t *p_del, *p_tmp;
	HASH_ITER(hh, closed_set, p_del, p_tmp)
	{
		HASH_DELETE(hh, closed_set, p_del);
		free(p_del);
	}
	HASH_ITER(hh, open_set, p_del, p_tmp)
	{
		HASH_DELETE(hh, open_set, p_del);
		free(p_del);
	}
	
	//printf("Solution not found, cnt = %d\n", cnt);

	if(cnt < 200)
	{
		return 1;
	}

	return 2;
	// Failure.

}

/*

search2():

Return values:

0 success
1 fails near the beginning
2 backing off helped search to succeed in the beginning, but it still fails later
3 search succeeds in the beginning, but fails later

Searches for the route; if it fails almost right away, different back-offs are tried.
Note that now "backoffs" also include trying to go forward first, as this could provide better
results than the search algorithm can provide, in some rare cases.

*/


int search2(route_unit_t **route, float start_ang, int start_x_mm, int start_y_mm, int end_x_mm, int end_y_mm)
{

#define SRCH_NUM_A 23
	static const int a_s[SRCH_NUM_A] =   // back-off angles in degs
	{	0,	-4,	4,	-8,	8,	-12,	12,	-18,	18,	-24,	24,	-36,	36,	-48,	48,	-60,	60,	-72,	72, 	-84,	84,	-96,	96	};


#define SRCH_NUM_BACK 18
	static const int b_s[SRCH_NUM_BACK] = // back-offs in mm - positive backoffs are actually forward :-).
	{	80,	120,	160,	240,	280,	320,	400,	480,	-320,	-400,	-280,	-240,	-200,	-480,	-160,	-120,	-80,	-560	};


	// If going forward doesn't work out from the beginning, try backing off slightly.

	int ret = search(route, start_ang, start_x_mm, start_y_mm, end_x_mm, end_y_mm);

	if(ret == 0)
		return 0;

	if(ret == 1)
	{
		//printf("Search fails in the start - trying to back off.\n");

		for(int a_idx = 0; a_idx < SRCH_NUM_A; a_idx++)
		{
			for(int back_idx = 0; back_idx < SRCH_NUM_BACK; back_idx++)
			{
				float new_ang = start_ang + (2.0*M_PI*(float)a_s[a_idx]/360.0);
				while(new_ang >= 2.0*M_PI) new_ang -= 2.0*M_PI;
				while(new_ang < 0.0) new_ang += 2.0*M_PI;
				int new_x = start_x_mm + cos(new_ang)*b_s[back_idx];
				int new_y = start_y_mm + sin(new_ang)*b_s[back_idx];

				int dir = (new_ang/(2.0*M_PI) * 32.0)+0.5;
				if(dir < 0) dir = 0; else if(dir > 31) dir = 31;

				int new_x_units, new_y_units;
				unit_coords(new_x, new_y, &new_x_units, &new_y_units);

		//		printf("Back off ang=%.2f deg, mm = %d  -> new start = (%d, %d) --> ", TODEG(new_ang), b_s[back_idx], new_x_units, new_y_units);

				if(check_hit(new_x_units, new_y_units, dir))
				{
		//			printf("backing off hits the wall.\n");
				}
				else
				{
					int ret = search(route, new_ang, new_x, new_y, end_x_mm, end_y_mm);
					if(ret == 0)
					{
						//printf("Search succeeded (back off ang=%.1fdeg, mm = %d), stopping back-off search.\n", TODEG(new_ang), b_s[back_idx]);

						route_unit_t* point = malloc(sizeof(route_unit_t));
						point->loc.x = new_x_units; point->loc.y = new_y_units;
						point->backmode = (b_s[back_idx]<0)?1:0;
						DL_PREPEND(*route, point);

						return 0;
					}
					else if(ret > 1)
					{
						//printf("Search failed later than in the beginning, stopping back-off search.\n");
						return 2;
					}
				}

			}
		}
		return 1;
	}

	return 3;

}


void gen_routing_page(world_t *w, int xpage, int ypage, int forgiveness)
{
	if(!w->pages[xpage][ypage])
	{
		return;
	}
	if(!w->rpages[xpage][ypage])
	{
		w->rpages[xpage][ypage] = malloc(sizeof(routing_page_t));
	}

	forgiveness = ROUTING_3D_FORGIVENESS;
	if(forgiveness == 0)
	{
		for(int xx=0; xx < MAP_PAGE_W; xx++)
		{
			for(int yy=0; yy < MAP_PAGE_W/32; yy++)
			{
				uint32_t tmp = 0;
				for(int i = 0; i < 32; i++)
				{
					tmp<<=1;
					uint8_t res  = w->pages[xpage][ypage]->units[xx][yy*32+i].result;
					uint8_t cons = w->pages[xpage][ypage]->units[xx][yy*32+i].constraints;
#ifdef AVOID_3D_THINGS
					tmp |= (res & UNIT_FREE) || (res & UNIT_WALL) || (res & UNIT_INVISIBLE_WALL) || (res & UNIT_3D_WALL) || (res & UNIT_ITEM) || (res & UNIT_DROP) || (cons & CONSTRAINT_FORBIDDEN);
#else
					tmp |= (res & UNIT_FREE) || (res & UNIT_WALL) || (res & UNIT_INVISIBLE_WALL) || (cons & CONSTRAINT_FORBIDDEN);
#endif
				}
				w->rpages[xpage][ypage]->obst_u32[xx][yy] = tmp;
			}
			if(w->pages[xpage][ypage+1])
			{
				uint32_t tmp = 0;
				for(int i = 0; i < 32; i++)
				{
					tmp<<=1;
					uint8_t res  = w->pages[xpage][ypage+1]->units[xx][0*32+i].result;
					uint8_t cons = w->pages[xpage][ypage+1]->units[xx][0*32+i].constraints;
#ifdef AVOID_3D_THINGS
					tmp |= (res & UNIT_FREE) || (res & UNIT_WALL) || (res & UNIT_INVISIBLE_WALL) || (res & UNIT_3D_WALL) || (res & UNIT_ITEM) || (res & UNIT_DROP) || (cons & CONSTRAINT_FORBIDDEN);
#else
					tmp |= (res & UNIT_FREE) || (res & UNIT_WALL) || (res & UNIT_INVISIBLE_WALL) || (cons & CONSTRAINT_FORBIDDEN);
#endif				
				}
				w->rpages[xpage][ypage]->obst_u32[xx][MAP_PAGE_W/32] = tmp;
			}
			else
			{
				w->rpages[xpage][ypage]->obst_u32[xx][MAP_PAGE_W/32] = 0xffffffff;
			}
		}
	}
	else
	{
		for(int xx=0; xx < MAP_PAGE_W; xx++)
		{
			for(int yy=0; yy < MAP_PAGE_W/32; yy++)
			{
				uint32_t tmp = 0;
				for(int i = 0; i < 32; i++)
				{
					tmp<<=1;
					uint8_t res =  w->pages[xpage][ypage]->units[xx][yy*32+i].result;
					uint8_t cons = w->pages[xpage][ypage]->units[xx][yy*32+i].constraints;
#ifdef AVOID_3D_THINGS
					tmp |= (res & UNIT_FREE) || (res & UNIT_WALL) || (res & UNIT_INVISIBLE_WALL) || (w->pages[xpage][ypage]->units[xx][yy*32+i].num_3d_obstacles > forgiveness) || (cons & CONSTRAINT_FORBIDDEN);
#else
					tmp |= (res & UNIT_FREE) || (res & UNIT_WALL) || (res & UNIT_INVISIBLE_WALL) || (cons & CONSTRAINT_FORBIDDEN);
#endif
				}
				w->rpages[xpage][ypage]->obst_u32[xx][yy] = tmp;
			}
			if(w->pages[xpage][ypage+1])
			{
				uint32_t tmp = 0;
				for(int i = 0; i < 32; i++)
				{
					tmp<<=1;
					uint8_t res  = w->pages[xpage][ypage+1]->units[xx][0*32+i].result;
					uint8_t cons = w->pages[xpage][ypage+1]->units[xx][0*32+i].constraints;
#ifdef AVOID_3D_THINGS
					tmp |= (res & UNIT_FREE) || (res & UNIT_WALL) || (res & UNIT_INVISIBLE_WALL) || (w->pages[xpage][ypage+1]->units[xx][0*32+i].num_3d_obstacles > forgiveness) || (cons & CONSTRAINT_FORBIDDEN);
#else
					tmp |= (res & UNIT_FREE) || (res & UNIT_WALL) || (res & UNIT_INVISIBLE_WALL) || (cons & CONSTRAINT_FORBIDDEN);
#endif
				}
				w->rpages[xpage][ypage]->obst_u32[xx][MAP_PAGE_W/32] = tmp;
			}
			else
			{
				w->rpages[xpage][ypage]->obst_u32[xx][MAP_PAGE_W/32] = 0xffffffff;
			}
		}
	}
	
}

void gen_all_routing_pages(world_t *w, int forgiveness)
{
	for(int xpage = 0; xpage < MAP_W; xpage++)
	{
		for(int ypage = 0; ypage < MAP_W; ypage++)
		{
			gen_routing_page(w, xpage, ypage, forgiveness);
		}
	}
}

int search_route(world_t *w, route_unit_t **route, float start_ang, int start_x_mm, int start_y_mm, int end_x_mm, int end_y_mm, int no_tight)
{
	routing_world = w;

	//printf("Searching for route...\n");

//	printf("Generating routing pages... "); fflush(stdout);
	gen_all_routing_pages(w, 0);
//	printf("done.\n");

	wide_search_mode();
	if(search2(route, start_ang, start_x_mm, start_y_mm, end_x_mm, end_y_mm))
	{
		normal_search_mode();
	//	printf("Searching with normal limits...\n");

		int ret;
		if( (ret = search2(route, start_ang, start_x_mm, start_y_mm, end_x_mm, end_y_mm)) )
		{
			if(no_tight)
			{
				printf("There is no route - didn't try tight search because no_tight was set.\n");
				return ret;
			}
			else
			{
		//		printf("Search failed - retrying with tighter limits.\n");
				tight_search_mode();
				if( (ret = search2(route, start_ang, start_x_mm, start_y_mm, end_x_mm, end_y_mm)) ) 
				{
					printf("There is no route.\n");
					return ret;
				}
				else
					printf("Found route with TIGHT limits\n");
			}
		}
		else
			printf("Found route with normal limits\n");
	}
	else
		printf("Found route with WIDE limits\n");


	// tight_search_mode is put into action so that collision avoidance / step skipping/rounding can use it.
	// TODO: fix this state horror. 
	tight_search_mode();
	return 0;
}

int32_t temp_lidar_map_mid_x, temp_lidar_map_mid_y;
uint8_t temp_lidar_map[256][256];

void clear_lidar_map(uint8_t *p_map)
{
	memset(p_map, 0, 256*256*sizeof(uint8_t));
}

int lidar_to_map(uint8_t *p_map, int32_t *mid_x, int32_t *mid_y, lidar_scan_t* p_lid)
{
	int mx = p_lid->robot_pos.x;
	int my = p_lid->robot_pos.y;
	*mid_x = mx; *mid_y = my;
	for(int i = 0; i < 360; i++)
	{
		if(!p_lid->scan[i].valid) continue;
		int x = (p_lid->scan[i].x - mx) / MAP_UNIT_W;
		int y = (p_lid->scan[i].y - my) / MAP_UNIT_W;
		x += 128; y+= 128;

		if(x < 0 || y < 0 || x > 255 || y > 255)
		{
			printf("WARNING: lidar_to_map(), ignoring out-of-range coords (%d, %d)\n", x, y);
			continue;
		}

		temp_lidar_map[x][y] = 1;
	}
	return 0;
}

void routing_set_world(world_t *w)
{
	routing_world = w;
}

int check_direct_route(int32_t start_ang, int start_x, int start_y, int end_x, int end_y)
{
	int dx = end_x - start_x;
	int dy = end_y - start_y;

	float end_ang = atan2(dy, dx);
	if(end_ang < 0.0) end_ang += 2.0*M_PI;

	//printf("check_direct_route(%d, %d, %d, %d, %d)\n", start_ang, start_x, start_y, end_x, end_y);

	if(test_robot_turn(start_x, start_y, ANG32TORAD(start_ang), end_ang))
	{
		//printf("check_direct_route(): robot can turn...\n");
		route_xy_t start = {start_x, start_y};
		route_xy_t end = {end_x, end_y};
		//printf(" start = (%d, %d)  end = (%d, %d)\n", start_x, start_y, end_x, end_y);
		if(line_of_sight(start, end))
		{
			//printf("check_direct_route(): there is line of sight\n");
			return 1;
		}
	}
	return 0;
}

int check_direct_route_hitcnt(int32_t start_ang, int start_x, int start_y, int end_x, int end_y)
{
	int dx = end_x - start_x;
	int dy = end_y - start_y;

	float end_ang = atan2(dy, dx);
	if(end_ang < 0.0) end_ang += 2.0*M_PI;

	int hitcnt = 0;

	hitcnt += test_robot_turn_hitcnt(start_x, start_y, ANG32TORAD(start_ang), end_ang);

	route_xy_t start = {start_x, start_y};
	route_xy_t end = {end_x, end_y};
	hitcnt += line_of_sight_hitcnt(start, end);

	return hitcnt;
}


int check_direct_route_non_turning(int start_x, int start_y, int end_x, int end_y)
{
	route_xy_t start = {start_x, start_y};
	route_xy_t end = {end_x, end_y};
	//printf(" start = (%d, %d)  end = (%d, %d)\n", start_x, start_y, end_x, end_y);
	if(line_of_sight(start, end))
	{
		//printf("check_direct_route(): there is line of sight\n");
		return 1;
	}
	return 0;
}

int check_direct_route_non_turning_hitcnt(int start_x, int start_y, int end_x, int end_y)
{
	route_xy_t start = {start_x, start_y};
	route_xy_t end = {end_x, end_y};
	return line_of_sight_hitcnt(start, end);
}


int check_turn(int32_t start_ang, int start_x, int start_y, int end_x, int end_y)
{
	int dx = end_x - start_x;
	int dy = end_y - start_y;

	float end_ang = atan2(dy, dx);
	if(end_ang < 0.0) end_ang += 2.0*M_PI;

	return test_robot_turn(start_x, start_y, ANG32TORAD(start_ang), end_ang);
}

int test_robot_turn_mm(int start_x, int start_y, float start_ang_rad, float end_ang_rad)
{
	return test_robot_turn(MM_TO_UNIT(start_x), MM_TO_UNIT(start_y), start_ang_rad, end_ang_rad);
}


int check_direct_route_mm(int32_t start_ang, int start_x, int start_y, int end_x, int end_y)
{
	return check_direct_route(start_ang, MM_TO_UNIT(start_x), MM_TO_UNIT(start_y), MM_TO_UNIT(end_x), MM_TO_UNIT(end_y));
}

int check_direct_route_non_turning_mm(int start_x, int start_y, int end_x, int end_y)
{
	return check_direct_route_non_turning(MM_TO_UNIT(start_x), MM_TO_UNIT(start_y), MM_TO_UNIT(end_x), MM_TO_UNIT(end_y));
}

int check_direct_route_hitcnt_mm(int32_t start_ang, int start_x, int start_y, int end_x, int end_y)
{
	return check_direct_route_hitcnt(start_ang, MM_TO_UNIT(start_x), MM_TO_UNIT(start_y), MM_TO_UNIT(end_x), MM_TO_UNIT(end_y));
}


int check_direct_route_non_turning_hitcnt_mm(int start_x, int start_y, int end_x, int end_y)
{
	return check_direct_route_non_turning_hitcnt(MM_TO_UNIT(start_x), MM_TO_UNIT(start_y), MM_TO_UNIT(end_x), MM_TO_UNIT(end_y));
}

int check_turn_mm(int32_t start_ang, int start_x, int start_y, int end_x, int end_y)
{
//	printf("check_turn_mm(%d, %d, %d, %d, %d)\n", start_ang, start_x, start_y, end_x, end_y);
	return check_turn(start_ang, MM_TO_UNIT(start_x), MM_TO_UNIT(start_y), MM_TO_UNIT(end_x), MM_TO_UNIT(end_y));
}
