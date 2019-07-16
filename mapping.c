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


	SLAM module:
	- 2D LIDAR scan prefiltering
	- 2D LIDAR scan matching algorithm
	- Sonar, mechanical obstacle, 3DTOF mapping (no localization based on these)
	- Exploration (autonomous mapping) - consider refactoring to its own module

*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>


#ifndef M_PI
#define M_PI 3.14159265358979323
#endif

#include "datatypes.h"
#include "map_memdisk.h"
#include "mapping.h"
#include "routing.h"

#include "tcp_comm.h"   // to send dbgpoint.
#include "tcp_parser.h" // to send dbgpoint.

#include "api_board_to_soft.h"
#include "misc.h"

extern void send_info(info_state_t state);


world_t world;

void page_coords(int mm_x, int mm_y, int* pageidx_x, int* pageidx_y, int* pageoffs_x, int* pageoffs_y)
{
	int unit_x = mm_x / MAP_UNIT_W;
	int unit_y = mm_y / MAP_UNIT_W;
	unit_x += MAP_MIDDLE_UNIT;
	unit_y += MAP_MIDDLE_UNIT;
	int page_x = unit_x / MAP_PAGE_W;
	int page_y = unit_y / MAP_PAGE_W;
	int offs_x = unit_x - page_x*MAP_PAGE_W;
	int offs_y = unit_y - page_y*MAP_PAGE_W;

	*pageidx_x = page_x;
	*pageidx_y = page_y;
	*pageoffs_x = offs_x;
	*pageoffs_y = offs_y;
}

void unit_coords(int mm_x, int mm_y, int* unit_x, int* unit_y)
{
	int unit_x_t = mm_x / MAP_UNIT_W;
	int unit_y_t = mm_y / MAP_UNIT_W;
	unit_x_t += MAP_MIDDLE_UNIT;
	unit_y_t += MAP_MIDDLE_UNIT;

	*unit_x = unit_x_t;
	*unit_y = unit_y_t;
}

void mm_from_unit_coords(int unit_x, int unit_y, int* mm_x, int* mm_y)
{
	unit_x -= MAP_MIDDLE_UNIT;
	unit_y -= MAP_MIDDLE_UNIT;

	*mm_x = unit_x * MAP_UNIT_W;
	*mm_y = unit_y * MAP_UNIT_W;
}

void page_coords_from_unit_coords(int unit_x, int unit_y, int* pageidx_x, int* pageidx_y, int* pageoffs_x, int* pageoffs_y)
{
	int page_x = unit_x / MAP_PAGE_W;
	int page_y = unit_y / MAP_PAGE_W;
	int offs_x = unit_x - page_x*MAP_PAGE_W;
	int offs_y = unit_y - page_y*MAP_PAGE_W;

	*pageidx_x = page_x;
	*pageidx_y = page_y;
	*pageoffs_x = offs_x;
	*pageoffs_y = offs_y;
}


// Shift page,offset coords directly by shift_x, shift_y units.
void shift_coords(int* px, int* py, int* ox, int* oy, int shift_x, int shift_y)
{
	*ox += shift_x;
	while(*ox > MAP_PAGE_W)
	{
		*ox -= MAP_PAGE_W;
		(*px)--;
	}
	while(*ox < 0)
	{
		*ox += MAP_PAGE_W;
		(*px)++;
	}

	*oy += shift_x;
	while(*oy > MAP_PAGE_W)
	{
		*oy -= MAP_PAGE_W;
		(*py)--;
	}
	while(*oy < 0)
	{
		*oy += MAP_PAGE_W;
		(*py)++;
	}
}


#define sq(x) ((x)*(x))


/* Rotation:
x2 = x*cos(a) + y*sin(a)
y2 = -1*x*sin(a) + y*cos(a)
*/

/*
	// Output 768x768x24bit raw image for debug.
	FILE* dbg_f = fopen("dbg_scoremap.data", "w");

	for(int iy = 0; iy < TEMP_MAP_W; iy++)
	{
		for(int ix = 0; ix < TEMP_MAP_W; ix++)
		{
			int r = 0, g = 0;
			if(scoremap[iy*TEMP_MAP_W+ix] > 0)
				g = scoremap[iy*TEMP_MAP_W+ix]*4;
			else
				r = scoremap[iy*TEMP_MAP_W+ix]*-4;

			if(r > 255) r = 255; if(g > 255) g = 255;
			fputc(r, dbg_f); // R
			fputc(g, dbg_f); // G
			fputc(0, dbg_f); // B
		}
	}

	fclose(dbg_f);
*/

extern double subsec_timestamp();





#define N_SENSORS 10


typedef struct
{
	int32_t mount_mode;             // mount position 1,2,3 or 4
	int32_t x_rel_robot;          // zero = robot origin. Positive = robot front (forward)
	int32_t y_rel_robot;          // zero = robot origin. Positive = to the right of the robot
	uint16_t ang_rel_robot;        // zero = robot forward direction. positive = ccw
	uint16_t vert_ang_rel_ground;  // zero = looks directly forward. positive = looks up. negative = looks down
	int32_t z_rel_ground;         // sensor height from the ground	
} sensor_mount_t;


/*
	Sensor mount position 1:
	 _ _
	| | |
	| |L|
	|O|L|
	| |L|
	|_|_|  (front view)

	Sensor mount position 2:
	 _ _
	| | |
	|L| |
	|L|O|
	|L| |
	|_|_|  (front view)

	Sensor mount position 3:

	-------------
	|  L  L  L  |
	-------------
	|     O     |
	-------------

	Sensor mount position 4:

	-------------
	|     O     |
	-------------
	|  L  L  L  |
	-------------
*/

#define DEGTOANG16(x)  ((uint16_t)((float)(x)/(360.0)*65536.0))


// const
sensor_mount_t sensor_mounts[N_SENSORS] =
{          //      mountmode    x     y       hor ang           ver ang      height    
 /*0:                */ { 0,     0,     0, DEGTOANG16(       0), DEGTOANG16( 2),         300 },

 /*1:                */ { 1,   130,   103, DEGTOANG16(    24.4), DEGTOANG16( 4.4),       310  }, // -1
 /*2:                */ { 2,  -235,   215, DEGTOANG16(    66.4), DEGTOANG16( 1.4),       310  }, // -1
 /*3:                */ { 2,  -415,   215, DEGTOANG16(    93.5), DEGTOANG16( 1.9),       310  }, // -1
 /*4:                */ { 2,  -522,   103, DEGTOANG16(   157.4), DEGTOANG16( 3.9),       280  }, // -1
 /*5:                */ { 2,  -522,   -35, DEGTOANG16(   176.0), DEGTOANG16( 4.9),       290  }, // -1
 /*6:                */ { 1,  -522,  -103, DEGTOANG16(   206.0), DEGTOANG16( 4.4),       290  }, // -1
 /*7:                */ { 1,  -415,  -215, DEGTOANG16(   271.5), DEGTOANG16( 2.4),       280  }, // -1
 /*8:                */ { 1,  -235,  -215, DEGTOANG16(   294.9), DEGTOANG16( 4.4),       300  }, // -1
 /*9:                */ { 2,   130,  -103, DEGTOANG16(   334.9), DEGTOANG16( -0.9),      320  }  // 0
};



// Modifies val_ as a side effect.
#define CALC_ONES(val_, bits_, result_) do{ result_=0; for(int i_=0; i_<(bits_); i_++) { if(val_&1){result_++;}   val_>>=1;  }   } while(0)
#define CALC_ZEROES(val_, bits_, result_) do{ result_=0; for(int i_=0; i_<(bits_); i_++) { if(!(val_&1)){result_++;}   val_>>=1;  }   } while(0)


#define CMP_MASK 0b1111111111110000



#define PAGE(mm) ( ((mm)/MAP_UNIT_W + MAP_MIDDLE_UNIT)/MAP_PAGE_W )
#define OFFS(mm) ( ((mm)/MAP_UNIT_W + MAP_MIDDLE_UNIT) - PAGE((mm))*MAP_PAGE_W )

#define PAGE_UN(un) ( ((un))/MAP_PAGE_W )
#define OFFS_UN(un) ( ((un)) - PAGE_UN((un))*MAP_PAGE_W )

static inline void shift_page_offs(int* page, int* offs, int delta) __attribute__((always_inline));
static inline void shift_page_offs(int* page, int* offs, int delta)
{
	int absol;
	absol = *page * MAP_PAGE_W + *offs + delta;
//	printf("page=%d, offs=%d, delta=%d, absol=%d\n", *page, *offs, delta, absol);
	*page = PAGE_UN(absol);
	*offs = OFFS_UN(absol);
//	printf("--> page=%d, offs=%d\n", *page, *offs);
}



extern float main_robot_xs;
extern float main_robot_ys;

#define STOP_REASON_JERK 1
#define STOP_REASON_OBSTACLE_FRONT_RIGHT 2
#define STOP_REASON_OBSTACLE_FRONT_LEFT 3
#define STOP_REASON_OBSTACLE_BACK_LEFT 5
#define STOP_REASON_OBSTACLE_BACK_RIGHT 6


// for vacuum app demo
#define ORIGIN_TO_ROBOT_FRONT 300
#define ASSUMED_ITEM_POS_FROM_MIDDLE_START 100
#define ASSUMED_ITEM_STEP_SIZE (MAP_UNIT_W)
#define ASSUMED_ITEM_NUM_STEPS 5
#define ARSE_OBSTACLE_BACK_LOCATION 300

void clear_visited(world_t* w, int now_x, int now_y)
{
	int idx_x, idx_y, offs_x, offs_y;

	page_coords(now_x, now_y, &idx_x, &idx_y, &offs_x, &offs_y);

	load_25pages(&world, idx_x, idx_y);

	for(int py=idx_y-2; py <= idx_y+2; py++)
	{
		for(int px=idx_x-2; px < idx_x+2; px++)
		{
			if(world.pages[px][py])
			{
				for(int i=0; i<(MAP_PAGE_W/2)*(MAP_PAGE_W/2); i++)
				{
					world.pages[px][py]->meta[i].num_visited = 0;
				}
				world.changed[px][py] = 1;
			}
		}
	}
}

void mark_current_as_visited(world_t* w, uint32_t now_ang, int now_x, int now_y)
{
	int idx_x, idx_y, offs_x, offs_y;

#if 1 // Vacuum application, mark the full nozzle area

	#define VACUUM_NOZZLE_WIDTH 500.0
	#define ITEM_STEP_SIZE 40.0
	#define VACUUM_NUM_STEPS ((int)(VACUUM_NOZZLE_WIDTH/ITEM_STEP_SIZE))


	for(int deep=0; deep<1; deep++)
	{
		float x = (float)now_x + cos(ANG32TORAD(now_ang))*(float)(ORIGIN_TO_ROBOT_FRONT-50.0 + deep*50.0);
		float y = (float)now_y + sin(ANG32TORAD(now_ang))*(float)(ORIGIN_TO_ROBOT_FRONT-50.0 + deep*50.0);

		int32_t angle = now_ang + (uint32_t)(-90*ANG_1_DEG);
		int32_t opposite_angle = now_ang + (uint32_t)(90*ANG_1_DEG);

		x += cos(ANG32TORAD(opposite_angle))*VACUUM_NOZZLE_WIDTH/2.0;
		y += sin(ANG32TORAD(opposite_angle))*VACUUM_NOZZLE_WIDTH/2.0;

		page_coords(x,y, &idx_x, &idx_y, &offs_x, &offs_y);
		load_9pages(&world, idx_x, idx_y);

		int prev_ox = -999, prev_oy = -999;
		for(int i = 0; i < VACUUM_NUM_STEPS; i++)
		{
			x += cos(ANG32TORAD(angle))*(float)ITEM_STEP_SIZE;
			y += sin(ANG32TORAD(angle))*(float)ITEM_STEP_SIZE;

			page_coords(x,y, &idx_x, &idx_y, &offs_x, &offs_y);

			if(offs_x == prev_ox && offs_y == prev_oy)
				continue;

			prev_ox = offs_x;
			prev_oy = offs_y;
			PLUS_SAT_255(world.pages[idx_x][idx_y]->meta[(offs_y/2)*(MAP_PAGE_W/2)+(offs_x/2)].num_visited);
			w->changed[idx_x][idx_y] = 1;
		}
	}

#else

	page_coords(now_x,now_y, &idx_x, &idx_y, &offs_x, &offs_y);
	load_page(&world, idx_x, idx_y);
	PLUS_SAT_255(world.pages[idx_x][idx_y]->meta[(offs_y/2)*(MAP_PAGE_W/2)+(offs_x/2)].num_visited);
	w->changed[idx_x][idx_y] = 1;

#endif
}

void map_collision_obstacle(world_t* w, int32_t now_ang, int now_x, int now_y, int stop_reason, int vect_valid, float vect_ang_rad)
{
#if 0
	int idx_x, idx_y, offs_x, offs_y;
	if(stop_reason == STOP_REASON_OBSTACLE_FRONT_LEFT || stop_reason == STOP_REASON_OBSTACLE_FRONT_RIGHT)
	{
		printf("Mapping FRONT obstacle due to wheel slip.\n");
		for(int deep=0; deep<3; deep++)
		{
			float x = (float)now_x + cos(ANG32TORAD(now_ang))*(float)(ORIGIN_TO_ROBOT_FRONT+20.0 + deep*40.0);
			float y = (float)now_y + sin(ANG32TORAD(now_ang))*(float)(ORIGIN_TO_ROBOT_FRONT+20.0 + deep*40.0);

			// Shift the result to the right or left:
			int32_t angle = now_ang + (uint32_t)( ((stop_reason==STOP_REASON_OBSTACLE_FRONT_RIGHT)?90:-90) *ANG_1_DEG);

			x += cos(ANG32TORAD(angle))*(float)ASSUMED_ITEM_POS_FROM_MIDDLE_START;
			y += sin(ANG32TORAD(angle))*(float)ASSUMED_ITEM_POS_FROM_MIDDLE_START;

			for(int i = 0; i < ASSUMED_ITEM_NUM_STEPS; i++)
			{
				x += cos(ANG32TORAD(angle))*(float)ASSUMED_ITEM_STEP_SIZE;
				y += sin(ANG32TORAD(angle))*(float)ASSUMED_ITEM_STEP_SIZE;

				page_coords(x,y, &idx_x, &idx_y, &offs_x, &offs_y);
				load_9pages(&world, idx_x, idx_y);
				world.pages[idx_x][idx_y]->units[offs_x][offs_y].result |= UNIT_INVISIBLE_WALL;
				world.pages[idx_x][idx_y]->units[offs_x][offs_y].latest |= UNIT_INVISIBLE_WALL;
				w->changed[idx_x][idx_y] = 1;
			}
		}
	}

	else if(stop_reason == STOP_REASON_OBSTACLE_BACK_LEFT || stop_reason == STOP_REASON_OBSTACLE_BACK_RIGHT)
	{
		printf("Mapping BACK (ARSE) obstacle due to wheel slip.\n");
		float x = (float)now_x + cos(ANG32TORAD(now_ang))*(float)-(float)ARSE_OBSTACLE_BACK_LOCATION;
		float y = (float)now_y + sin(ANG32TORAD(now_ang))*(float)-(float)ARSE_OBSTACLE_BACK_LOCATION;

		// Shift the result to the right or left:
		int32_t angle = now_ang + (uint32_t)( ((stop_reason==STOP_REASON_OBSTACLE_BACK_RIGHT)?90:-90) *ANG_1_DEG);

		x += cos(ANG32TORAD(angle))*(float)main_robot_ys/2.0;
		y += sin(ANG32TORAD(angle))*(float)main_robot_ys/2.0;

		for(int i = 0; i < 3; i++)
		{
			x += cos(ANG32TORAD(angle))*(float)ASSUMED_ITEM_STEP_SIZE;
			y += sin(ANG32TORAD(angle))*(float)ASSUMED_ITEM_STEP_SIZE;

			page_coords(x,y, &idx_x, &idx_y, &offs_x, &offs_y);
			load_9pages(&world, idx_x, idx_y);
			world.pages[idx_x][idx_y]->units[offs_x][offs_y].result |= UNIT_INVISIBLE_WALL;
			world.pages[idx_x][idx_y]->units[offs_x][offs_y].latest |= UNIT_INVISIBLE_WALL;
			w->changed[idx_x][idx_y] = 1;
		}
	}

	else if(stop_reason == STOP_REASON_JERK)
	{
/*		printf("Mapping obstacle due to acceleration (ang = %.0f).\n", RADTODEG(vect_ang_rad));
		for(int i=-2; i<=2; i++)
		{
			int idx = 32.0*vect_ang_rad/(2.0*M_PI);
			idx += i;
			while(idx < 0) idx+=32;
			while(idx > 31) idx-=32;
			for(int o = 0; o < 2; o++)
			{
				int dist_to_outline = robot_outline[idx] + 30 + o*40;
				float x = (float)now_x + cos( ANG32TORAD(now_ang) + vect_ang_rad+((float)i*2.0*M_PI/32.0) )*(float)dist_to_outline;
				float y = (float)now_y + sin( ANG32TORAD(now_ang) + vect_ang_rad+((float)i*2.0*M_PI/32.0) )*(float)dist_to_outline;

				page_coords(x,y, &idx_x, &idx_y, &offs_x, &offs_y);
				load_9pages(&world, idx_x, idx_y);
				world.pages[idx_x][idx_y]->units[offs_x][offs_y].result |= UNIT_ITEM | UNIT_WALL | UNIT_DO_NOT_REMOVE_BY_LIDAR;
				world.pages[idx_x][idx_y]->units[offs_x][offs_y].latest |= UNIT_ITEM | UNIT_WALL | UNIT_DO_NOT_REMOVE_BY_LIDAR;
				PLUS_SAT_255(world.pages[idx_x][idx_y]->units[offs_x][offs_y].num_obstacles);
				PLUS_SAT_255(world.pages[idx_x][idx_y]->units[offs_x][offs_y].num_obstacles);
				w->changed[idx_x][idx_y] = 1;
			}
		} */
	}
	else
	{
		printf("WARN: Unrecognized stop reason %d\n", stop_reason);
	}
	
#endif
}

void clear_within_robot(world_t* w, int32_t now_ang, int32_t now_x, int32_t now_y)
{
	int idx_x, idx_y, offs_x, offs_y;
	int robot_xs = main_robot_xs;
	int robot_ys = main_robot_ys;

//	printf("robot_xs=%d, robot_ys=%d, stripes = %d, i_end = %d\n", robot_xs, robot_ys, robot_xs/20-1, robot_ys/20-1);

	for(int stripe = -3; stripe < robot_xs/20 + 3; stripe++)
	{
		float x = (float)now_x + cos(ANG32TORAD(now_ang))*(float)(ORIGIN_TO_ROBOT_FRONT-stripe*20);
		float y = (float)now_y + sin(ANG32TORAD(now_ang))*(float)(ORIGIN_TO_ROBOT_FRONT-stripe*20);

		// Shift the result in robot_y direction
		int32_t angle = now_ang + (uint32_t)(90*ANG_1_DEG);

		x += cos(ANG32TORAD(angle))*((float)robot_ys/-2.0-100);
		y += sin(ANG32TORAD(angle))*((float)robot_ys/-2.0-100);

		for(int i = 0; i < robot_ys/20 + 6; i++)
		{
			x += cos(ANG32TORAD(angle))*(float)20.0;
			y += sin(ANG32TORAD(angle))*(float)20.0;

			page_coords(x,y, &idx_x, &idx_y, &offs_x, &offs_y);
			load_1page(&world, idx_x, idx_y);
			world.pages[idx_x][idx_y]->voxmap[MAPIDX(offs_x, offs_y)] &= 0b111;
			world.changed[idx_x][idx_y] = 1;
		}
	}
}



void map_sonars(world_t* w, int n_sonars, sonar_point_t* p_sonars)
{
#if 0
	int idx_x, idx_y, offs_x, offs_y;

	for(int i=0; i<n_sonars; i++)
	{
		if(p_sonars[i].c >= 2 && p_sonars[i].z > 40 && p_sonars[i].z < 1600)
		{
			//printf("Mapping a sonar item at (%d, %d) z=%d c=%d\n", p_sonars[i].x, p_sonars[i].y, p_sonars[i].z, p_sonars[i].c);
			page_coords(p_sonars[i].x,p_sonars[i].y, &idx_x, &idx_y, &offs_x, &offs_y);
			world.pages[idx_x][idx_y]->units[offs_x][offs_y].result |= UNIT_ITEM;
		}
	}

//	printf("Mapping an item\n");


	// Erase old items, but only if all three sonars show a ping from farther away.
/*
	if(p_son->scan[0].valid && p_son->scan[1].valid && p_son->scan[2].valid)
	{
		float nearest = 2000.0;
		for(int i = 0; i < 3; i++)
		{
			int dx = p_son->scan[i].x - p_son->robot_pos.x;
			int dy = p_son->scan[i].y - p_son->robot_pos.y;

			float cur_len = sqrt(sq(dx) + sq(dy));

			if(cur_len < nearest) nearest = cur_len;
		}

		if(nearest > 500.0)
		{
			const float step = 3*MAP_UNIT_W;

			float pos = 300.0;
			int terminate = 0;

			int dx = p_son->scan[1].x - p_son->robot_pos.x;
			int dy = p_son->scan[1].y - p_son->robot_pos.y;
			float ang = atan2(dy, dx) + M_PI;
			if(ang < 0.0) ang += 2.0*M_PI;
			else if(ang > 2.0*M_PI) ang -= 2.0*M_PI;

		//	printf("Clearing items start (%d, %d) ang = %.1f deg, len = %.1f\n", p_son->scan[1].x, p_son->scan[1].y, RADTODEG(ang), nearest);
		//	printf("ang = %.4f  dir = %d \n", ang, dir);

			while(1)
			{
				int x = (cos(ang)*pos + (float)p_son->scan[1].x);
				int y = (sin(ang)*pos + (float)p_son->scan[1].y);

				for(int ix=-5*MAP_UNIT_W; ix<=5*MAP_UNIT_W; ix+=MAP_UNIT_W)
				{
					for(int iy=-5*MAP_UNIT_W; iy<=5*MAP_UNIT_W; iy+=MAP_UNIT_W)
					{	
						page_coords(x+ix,y+iy, &idx_x, &idx_y, &offs_x, &offs_y);
						load_9pages(&world, idx_x, idx_y);
						world.pages[idx_x][idx_y]->units[offs_x][offs_y].result &= ~(UNIT_ITEM);
					}
				}

				if(terminate) break;
				pos += step;
				if(pos > nearest-150)
				{
					pos = nearest;
					terminate = 1;
				}
			}
		}
	}

	for(int i=0; i<3; i++)
	{
		if(!p_son->scan[i].valid) continue;

		for(int s=0; s<25; s++)
		{
			int x = p_son->scan[i].x+search_order[s][0]*MAP_UNIT_W;
			int y = p_son->scan[i].y+search_order[s][1]*MAP_UNIT_W;
			page_coords(x,y, &idx_x, &idx_y, &offs_x, &offs_y);
			load_9pages(&world, idx_x, idx_y);

			if(world.pages[idx_x][idx_y]->units[offs_x][offs_y].result & UNIT_ITEM)
			{
//				printf("Item already mapped\n");
				goto ALREADY_MAPPED_ITEM;
			}
		}

		int dx = p_son->scan[i].x - p_son->robot_pos.x;
		int dy = p_son->scan[i].y - p_son->robot_pos.y;

		int sqdist = sq(dx) + sq(dy);

		if(sqdist < sq(1500))
		{
			page_coords(p_son->scan[i].x,p_son->scan[i].y, &idx_x, &idx_y, &offs_x, &offs_y);
			world.pages[idx_x][idx_y]->units[offs_x][offs_y].result |= UNIT_ITEM;
//			printf("Mapping an item\n");
			//world.changed[idx_x][idx_y] = 1;
		}

		ALREADY_MAPPED_ITEM: ;
	}

	*/

#endif
}



/*
	Autonomous Exploration code starts here
*/

int unfamiliarity_score(world_t* w, int x, int y)
{
	int n_walls = 0;
	int n_seen = 0;
	int n_visited = 1; // to avoid div per zero
	for(int xx = x - 300; xx <= x + 300; xx += 50)
	{
		for(int yy = y - 300; yy <= y + 300; yy += 50)
		{
			int px, py, ox, oy;
			page_coords(xx,yy, &px, &py, &ox, &oy);

			if(w->pages[px][py])
			{
				if(w->pages[px][py]->voxmap[MAPIDX(ox, oy)] & 0b1111111111100000) n_walls++;

				if(w->pages[px][py]->voxmap[MAPIDX(ox, oy)] & 0b0000000000011111) n_seen++;

				n_visited += w->pages[px][py]->meta[(oy/2)*(MAP_PAGE_W/2)+(ox/2)].num_visited;
			}

		}
	}

	if(n_walls > 10)
		return 0;

	if(n_seen < 5)
		return 0;

	return (1000000-90000*n_walls)/n_visited;
}

typedef struct
{
	int enabled;
	int x;
	int y;
} cant_goto_place_t;

// This list is populated when going towards a (desired_x,desired_y) destination leads to stop, or the dest is not reached within sensible time.
// Places near the ones in this list are ignored when finding unfamiliar places.
// The list is purposedly overwritten so that we try again older places, in case the conditions have changed.
// TODO: make this work per every world, now only one world supported.

#define CANT_GOTO_PLACE_LIST_LEN 64
cant_goto_place_t cant_goto_places[CANT_GOTO_PLACE_LIST_LEN];
int cant_goto_place_wr_idx;

void add_cant_goto_place(int x, int y)
{
//	printf("Adding cant_goto_place idx=%d, abs (%d, %d) mm\n", cant_goto_place_wr_idx, x, y);
	cant_goto_places[cant_goto_place_wr_idx].enabled = 1;
	cant_goto_places[cant_goto_place_wr_idx].x = x;
	cant_goto_places[cant_goto_place_wr_idx].y = y;
	cant_goto_place_wr_idx++;
	if(cant_goto_place_wr_idx >= CANT_GOTO_PLACE_LIST_LEN)
	{
		printf("cant_goto_place_list wraps around, old out-of-limits areas will be tried again.\n");
		cant_goto_place_wr_idx = 0;

	}
}

int find_unfamiliar_direction(world_t* w, int *x_out, int *y_out)
{
	int biggest = 0;
	int biggest_x, biggest_y;
	for(int dx = -4800; dx <= 4800; dx += 200)
	{
		for(int dy = -4800; dy <= 4800; dy += 200)
		{
			extern int32_t cur_x, cur_y;
			int score = unfamiliarity_score(w, cur_x+dx, cur_y+dy);
			if(score > biggest)
			{
				for(int i = 0; i < CANT_GOTO_PLACE_LIST_LEN; i++)
				{
					if(cant_goto_places[i].enabled && (sq(cant_goto_places[i].x-(cur_x+dx))+sq(cant_goto_places[i].y-(cur_y+dy))) < sq(500) )
					{
//						printf("ignoring potential biggest score place, cant_goto_idx=%d, abs (%d, %d) mm.\n", i, cur_x+dx, cur_y+dy);
						goto CONTINUE_UNFAM_LOOP;
					}
				}

				biggest = score;
				biggest_x = cur_x+dx;
				biggest_y = cur_y+dy;

				CONTINUE_UNFAM_LOOP:;
			}
		}
	}

	if(biggest)
	{
		*x_out = biggest_x;
		*y_out = biggest_y;
	}

	return biggest;
}

int find_unfamiliar_direction_randomly(world_t* w, int *x_out, int *y_out)
{
	int biggest = 0;
	int biggest_x, biggest_y;
	for(int try=0; try<2000; try++)
	{
		extern int32_t cur_x, cur_y;
		float rand1 = ((float)rand() / (float)RAND_MAX)*5000.0+300.0;
		float rand2 = ((float)rand() / (float)RAND_MAX)*5000.0+300.0;

		if(rand()&1)
			rand1 *= -1.0;
		if(rand()&1)
			rand2 *= -1.0;

		int potential_x = cur_x+rand1;
		int potential_y = cur_y+rand2;

		int score = unfamiliarity_score(w, potential_x, potential_y);
//		printf("unfam try=%d x=%d y=%d score=%d\n", try, potential_x, potential_y, score);
		if(score > biggest)
		{
			for(int i = 0; i < CANT_GOTO_PLACE_LIST_LEN; i++)
			{
				if(cant_goto_places[i].enabled && (sq(cant_goto_places[i].x-potential_x)+sq(cant_goto_places[i].y-potential_y)) < sq(500) )
				{
					goto CONTINUE_UNFAM_LOOP;
				}
			}

			biggest = score;
			biggest_x = potential_x;
			biggest_y = potential_y;

			CONTINUE_UNFAM_LOOP:;
		}

		if(biggest >= 1000000)
			break;
	}

	if(biggest)
	{
		*x_out = biggest_x;
		*y_out = biggest_y;
	}	

	return biggest;
}

const char* const AUTOSTATE_NAMES[] =
{
	"IDLE",
	"START",
	"COMPASS",
	"WAIT_COMPASS_START",
	"WAIT_COMPASS_END",
	"WAIT_COMPASS_MEASURED",
	"SYNC_TO_COMPASS",
	"GEN_DESIRED_DIR",
	"FIND_DIR",
	"WAIT_MOVEMENT",
	"DAIJUING",
	"GEN_ROUTING",
	"S_WAIT_ROUTING_FINISHED",
	"WAIT_ROUTE_DRIVE",
	"res",
	"res"
};

typedef enum
{
	S_IDLE   		= 0,
	S_START 		= 1,
	S_COMPASS		= 2,
	S_WAIT_COMPASS_START	= 3,
	S_WAIT_COMPASS_END	= 4,
	S_WAIT_COMPASS_MEASURED	= 5,
	S_SYNC_TO_COMPASS	= 6,
	S_GEN_DESIRED_DIR	= 7,
	S_FIND_DIR		= 8,
	S_WAIT_MOVEMENT  	= 9,
	S_DAIJUING		= 10,
	S_GEN_ROUTING		= 11,
	S_WAIT_ROUTING_FINISHED = 12,
	S_WAIT_ROUTE_DRIVE	= 13
} autostate_t;

autostate_t cur_autostate;

int doing_autonomous_things()
{
	return (int)cur_autostate;
}

void start_automapping_from_compass()
{
	cur_autostate = S_START;
}

void start_automapping_skip_compass()
{
	cur_autostate = S_GEN_DESIRED_DIR;
}

void stop_automapping()
{
	map_significance_mode = MAP_SIGNIFICANT_IMGS;
	cur_autostate = S_IDLE;
	send_info(INFO_STATE_IDLE);
}

int automap_only_compass;

void start_automap_only_compass()
{
	cur_autostate = S_START;
	automap_only_compass = 1;
}

extern double subsec_timestamp();

extern int run_search(int32_t dest_x, int32_t dest_y, int dont_map_lidars);
extern int poll_search_status(int act_as_well);


extern int max_speedlim;
void autofsm()
{
	static int movement_id = 0;
	extern int32_t cur_compass_ang;
	extern int compass_round_active;
	extern int32_t cur_x, cur_y;

	int prev_autostate = cur_autostate;

	static int desired_x;
	static int desired_y;
	static int same_dir_cnt;
	static int same_dir_len;
	static double daijuing_timestamp;
	static int num_stops;

	static int map_lidars_when_searched = 0;

	static int increase_speedlim = 0;

	switch(cur_autostate)
	{
		case S_IDLE: {

		} break;

		case S_START: {
			daiju_mode(0);
			state_vect.v.mapping_collisions = state_vect.v.mapping_3d = state_vect.v.mapping_2d = state_vect.v.loca_3d = state_vect.v.loca_2d = 0;
			cur_autostate++;
		} break;

		case S_COMPASS: {
			increase_speedlim = 0;
			max_speedlim = 15;
			send_info(INFO_STATE_THINK);

//			printf("Started compass round\n");
			do_compass_round();
			cur_autostate++;
		} break;

		case S_WAIT_COMPASS_START: {
			if(compass_round_active)
				cur_autostate++;
		} break;

		case S_WAIT_COMPASS_END: {
			if(!compass_round_active)
			{
				compass_round_active = 1; // to force one more compass reading, with robot being still.
				cur_autostate++;
			}
		} break;

		case S_WAIT_COMPASS_MEASURED: {
			if(!compass_round_active)
			{
				compass_round_active = 1; // to force one more compass reading, with robot being still.
				cur_autostate++;
			}
		} break;

		case S_SYNC_TO_COMPASS: {
			if(!compass_round_active)
			{
//				printf("Syncing robot angle to compass, zeroing coords, turning mapping on, requesting massive search area.\n");
				int32_t ang = cur_compass_ang-90*ANG_1_DEG;
				printf("DBG: cur_compass_ang=%d (%.1fdeg), ang=%d (%.1fdeg)\n", cur_compass_ang, ANG32TOFDEG(cur_compass_ang), ang, ANG32TOFDEG(ang));
				set_robot_pos(ang,0,0);
				state_vect.v.mapping_collisions = state_vect.v.mapping_3d = state_vect.v.mapping_2d = state_vect.v.loca_3d = state_vect.v.loca_2d = 1;
				state_vect.v.localize_with_big_search_area = 1;
				if(automap_only_compass)
					cur_autostate = S_IDLE;
				else
					cur_autostate++;
			}
		} break;

		case S_GEN_DESIRED_DIR: {
			send_info(INFO_STATE_THINK);

			daiju_mode(0);

			int unfam_score = find_unfamiliar_direction_randomly(&world, &desired_x, &desired_y);

			if(unfam_score)
			{
				same_dir_len = 10;
				printf("Generated new desired vector abs (%d, %d) mm based on unfamiliarity score %d, time to follow = %d\n", desired_x, desired_y, unfam_score, same_dir_len);
				if(tcp_client_sock >= 0) tcp_send_dbgpoint(desired_x, desired_y, 0, 255, 40, 0);
			}
			else
			{
				float rand1 = ((float)rand() / (float)RAND_MAX)*6000.0+300.0;
				float rand2 = ((float)rand() / (float)RAND_MAX)*6000.0+300.0;

				if(rand()&1)
					rand1 *= -1.0;
				if(rand()&1)
					rand2 *= -1.0;

				desired_x = cur_x+rand1;
				desired_y = cur_y+rand2;
				same_dir_len = 10; //((float)rand() / (float)RAND_MAX)*7.0;
				printf("No unfamiliarity scores generated (area unmapped?): generated new random desired vector abs (%d, %d) mm, time to follow = %d\n", desired_x, desired_y, same_dir_len);
				if(tcp_client_sock >= 0) tcp_send_dbgpoint(desired_x, desired_y, 128, 150, 40, 0);
			}
			same_dir_cnt = 0;
			//cur_autostate++;
			cur_autostate = S_GEN_ROUTING;
		} break;

		case S_FIND_DIR: {
			send_info(INFO_STATE_THINK);

			map_significance_mode = MAP_SIGNIFICANT_IMGS | MAP_SEMISIGNIFICANT_IMGS;
			state_vect.v.mapping_collisions = state_vect.v.mapping_3d = state_vect.v.mapping_2d = state_vect.v.loca_3d = state_vect.v.loca_2d = 1;

			#define NUM_LATEST_LIDARS_FOR_ROUTING_START 7
			//extern lidar_scan_t* lidars_to_map_at_routing_start[NUM_LATEST_LIDARS_FOR_ROUTING_START];
// jooooooo			map_lidars_to_minimap(NUM_LATEST_LIDARS_FOR_ROUTING_START, lidars_to_map_at_routing_start);

//			map_lidar_to_minimap(latest_lidar);

			int32_t dx, dy;
			int need_to_back = 0;
			extern int32_t cur_ang;
			if(num_stops > 8)
			{
				num_stops = 0;
				printf("Too many stops without success, daijuing for a while.\n");
				send_info(INFO_STATE_DAIJUING);
				daiju_mode(1);
				cur_autostate = S_DAIJUING;
				daijuing_timestamp = subsec_timestamp();
			}
			else
			{
				int ret;
				if( (ret = minimap_find_mapping_dir(&world, ANG32TORAD(cur_ang), &dx, &dy, desired_x-cur_x, desired_y-cur_y, &need_to_back)) )
				{
					printf("Found direction\n");
					if(movement_id == cur_xymove.id)
					{
						printf("id issue, incrementing by 5\n");
						movement_id+=5;
						if(movement_id > 100) movement_id = 0;
					}
					set_hw_obstacle_avoidance_margin((ret&2)?0:120);

					send_info(need_to_back?INFO_STATE_REV:INFO_STATE_FWD);

					move_to(cur_x+dx, cur_y+dy, need_to_back, movement_id, 30, 0);
					cur_autostate++;
				}
				else
				{
					printf("Automapping: can't go anywhere; daijuing for a while.\n");
					send_info(INFO_STATE_DAIJUING);
					daiju_mode(1);
					cur_autostate = S_DAIJUING;
					daijuing_timestamp = subsec_timestamp();
				}
			}

		} break;

		case S_WAIT_MOVEMENT: {

			if(cur_xymove.id == movement_id && cur_xymove.remaining < 80)
			{
				num_stops = 0;
				printf("Automapping: movement id=%d finished, ", movement_id);

				movement_id++; if(movement_id > 100) movement_id = 0;
				same_dir_cnt++;
				if(same_dir_cnt > same_dir_len || (sq(cur_x-desired_x) + sq(cur_y-desired_y)) < sq(400))
				{
					printf("generate new desired direction.\n");
					if(same_dir_cnt > same_dir_len)
					{
						add_cant_goto_place(desired_x, desired_y);
						if(tcp_client_sock >= 0) tcp_send_dbgpoint(desired_x, desired_y, 255, 30, 50, 1);
					}
					cur_autostate = S_GEN_DESIRED_DIR;
				}
				else
				{
					printf("continue with the old desired direction.\n");
					cur_autostate = S_FIND_DIR;
				}
			}
			else if(cur_xymove.id == movement_id && (cur_xymove.micronavi_stop_flags || cur_xymove.feedback_stop_flags))
			{
				printf("Automapping: movement id=%d stopped, generating new desired direction\n", movement_id);
				map_lidars_when_searched = 1;
				add_cant_goto_place(desired_x, desired_y);
				if(tcp_client_sock >= 0) tcp_send_dbgpoint(desired_x, desired_y, 255, 30, 50, 1);
				movement_id++; if(movement_id > 100) movement_id = 0;
				num_stops++;
				cur_autostate = S_GEN_DESIRED_DIR;
			}

		} break;

		case S_DAIJUING: {
			if(subsec_timestamp() > daijuing_timestamp+3.0)
			{
				cur_autostate = S_GEN_DESIRED_DIR;
				daiju_mode(0);
			}
		} break;

		case S_GEN_ROUTING: {

			if(increase_speedlim==1)
			{
				increase_speedlim = 2;
				max_speedlim = 50;
			}

			int ret = run_search(desired_x, desired_y, !map_lidars_when_searched);
			cur_autostate = S_WAIT_ROUTING_FINISHED;
			map_lidars_when_searched = 0;
			
		} break;

		case S_WAIT_ROUTING_FINISHED: {

			int ret = poll_search_status(0);

			if(ret == 12345)
			{
				// Keep waiting...
			}
			else if(ret == 1)
			{
				printf("Automapping: run_search() fails in the start due to close obstacles (nonroutable), daijuing for a while.\n");
				send_info(INFO_STATE_DAIJUING);
				daiju_mode(1);
				cur_autostate = S_DAIJUING;
				daijuing_timestamp = subsec_timestamp();				
			}
			else if(ret == 0 || ret == -999)
			{
				cur_autostate = S_WAIT_ROUTE_DRIVE;
			}
			else
			{
//				printf("Automapping: run_search() fails later than in the start: destination is unreachable. Generating a new direction.\n");
				add_cant_goto_place(desired_x, desired_y);
				cur_autostate = S_GEN_DESIRED_DIR;
			}

		} break;

		case S_WAIT_ROUTE_DRIVE: {
			extern int route_finished_or_notfound;

			if(route_finished_or_notfound)
			{
//				printf("Automapping: Following route finished.\n");
				cur_autostate = S_GEN_DESIRED_DIR;
				if(increase_speedlim==0) increase_speedlim=1;
			}

		} break;

		default: break;
	}

	if(cur_autostate != prev_autostate)
	{
		printf("autostate change %s --> %s\n", AUTOSTATE_NAMES[prev_autostate], AUTOSTATE_NAMES[cur_autostate]);
	}
}


void add_map_constraint(world_t* w, int32_t x, int32_t y)
{
	int px, py, ox, oy;
	page_coords(x, y, &px, &py, &ox, &oy);
	load_1page(w, px, py);
	w->pages[px][py]->meta[(oy/2)*(MAP_PAGE_W/2)+(ox/2)].constraints |= CONSTRAINT_FORBIDDEN;
	w->changed[px][py] = 1;
}

void remove_map_constraint(world_t* w, int32_t x, int32_t y)
{
	int px, py, ox, oy;
	page_coords(x, y, &px, &py, &ox, &oy);
	load_1page(w, px, py);
	w->pages[px][py]->meta[(oy/2)*(MAP_PAGE_W/2)+(ox/2)].constraints &= ~(CONSTRAINT_FORBIDDEN);
	w->changed[px][py] = 1;
}
