/*
	PULUROBOT RN1-HOST Computer-on-RobotBoard main software

	(c) 2017-2019 Pulu Robotics and other contributors
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

#ifndef MAPPING_H
#define MAPPING_H

#include <stdint.h>
#include "datatypes.h"
#include "api_board_to_soft.h"

#define UNIT_FREE	0
#define UNIT_ITEM           (1<<0)	// Small obstacle, detected by sonars or bumping into it
#define UNIT_WALL           (1<<1)	// Obstacle seen by the lidar.
#define UNIT_INVISIBLE_WALL (1<<2)  // Only found out by collision
#define UNIT_3D_WALL        (1<<3)  // Wall seen by 3DTOF, can't be removed by lidar.
#define UNIT_DROP           (1<<4)
#define UNIT_DBG            (1<<6)
#define UNIT_MAPPED         (1<<7)  // We have seen this area.

#define CONSTRAINT_FORBIDDEN 	(1<<0)	// "Don't go here" unit

#define PLUS_SAT_255(x) {if((x)<255) (x)++;}
#define MINUS_SAT_0(x) {if((x)>0) (x)--;}


#define MM_TO_UNIT(x) ((x)/MAP_UNIT_W + MAP_MIDDLE_UNIT)


typedef struct __attribute__ ((packed))
{
	uint8_t timestamp;	// Latest time scanned
	uint8_t num_visited;    // Incremented when mapped with this robot coord. Saturated at 255.
	uint8_t constraints;
	uint8_t reserved;
} map_unit_meta_t;


/*
Map page is a fixed 256*256 * 50mm*50mm = 12.8m*12.8m area.

At all times, the following principles apply:
- There are 25 map pages (5*5) in the memory
- The robot is always located somewhere in the middle page.
To satisfy these conditions, map pages are stored/retrieved to/from the disk every time the robot crosses the map border.

-> always a guaranteed minimum of 25.6 m of pre-loaded map range in any direction
-> Mapping algorithm doesn't need to check and load pages based on measurements, as long as maximum measured distance is guaranteed below 25.6m.

Memory footprint is around 15Mbytes for the 25 pages

*/

#define MAP_UNIT_W 50  // in mm
#define MAP_PAGE_W 256 // in map_units
#define MAP_PAGE_W_MM (MAP_UNIT_W * MAP_PAGE_W)  // convenience define


typedef struct  __attribute__ ((packed))
{
	uint32_t magic; // 0xAA1337AA
	uint32_t file_version; // Currently 0x420

	int32_t xy_step_mm;
	int32_t z_step_mm;

	/*
		Voxel map:
		
		Each voxel is 2-bit value: another is stored in vox_occu, another in vox_free

		Occu	Free	Description
		0	0	Not seen (initial state)
		0	1	Seen as free space
		1	0	Seen as occupied space
		1	1	Uncertain occupance. Exact nature will reveal in the future; the needs are not clear yet.



	*/

	uint64_t vox_occu[MAP_PAGE_W*MAP_PAGE_W];
	uint64_t vox_free[MAP_PAGE_W*MAP_PAGE_W];




	int32_t  base_z_mm; // Reference Z level, bit0 spans  [base_z_mm...base_z_mm+z_step[
	uint32_t voxmap[MAP_PAGE_W*MAP_PAGE_W];
	map_unit_meta_t meta[(MAP_PAGE_W/2)*(MAP_PAGE_W/2)]; // half*half resolution

/*
	Routing pages (for optimization purposes only) use single bits to denote forbidden areas, so
	that 32-bit wide robot shapes can be compared against hits efficiently. For the same reason,
	one extra uint32 block is included on the bottom (positive) end.
*/

	uint8_t  routing_valid;
	uint32_t routing[MAP_PAGE_W][MAP_PAGE_W/32 + 1];

} map_page_t;


// Voxel Coordinate macro: index vox_occu[VC(x,y)] and vox_free[VC(x,y)]
#define VC(x_,y_) ((y_)*MAP_PAGE_W+(x_))



/*
world_t is one continuously mappable entity. There can be several worlds, but the worlds cannot overlap;
in case they would, they should be combined.

When the software cannot decide which world it's in, we create a new, empty world. If we figure out it matches
an earlier world, we will combine them.

To make things simple, we statically allocate for the map page pointers. Map pages themselves are dynamically allocated.

256*256 map pages will limit the maximum world size to 2.62 km * 2.62 km. On a 64-bit system, storing pointers to map pages
and qmap pages, 1Mbyte is required for the pointers. NULL pointer means the data is not loaded in memory.

*/

#define MAP_W 256
#define MAP_MIDDLE_PAGE (MAP_W/2)

#define MAP_MIDDLE_UNIT (MAP_PAGE_W * MAP_MIDDLE_PAGE)

typedef struct
{
	uint32_t id;

	map_page_t*  pages[MAP_W][MAP_W];
	uint8_t changed[MAP_W][MAP_W];
} world_t;

void page_coords(int mm_x, int mm_y, int* pageidx_x, int* pageidx_y, int* pageoffs_x, int* pageoffs_y);
void unit_coords(int mm_x, int mm_y, int* unit_x, int* unit_y);
void mm_from_unit_coords(int unit_x, int unit_y, int* mm_x, int* mm_y);
void page_coords_from_unit_coords(int unit_x, int unit_y, int* pageidx_x, int* pageidx_y, int* pageoffs_x, int* pageoffs_y);


int map_lidars(world_t* w, int n_lidars, lidar_scan_t** lidar_list, int* da, int* dx, int* dy);
void map_next_with_larger_search_area();

void map_sonars(world_t* w, int n_sonars, sonar_point_t* p_sonars);
void map_collision_obstacle(world_t* w, int32_t cur_ang, int cur_x, int cur_y, int stop_reason, int vect_valid, float vect_ang_rad);
void mark_current_as_visited(world_t* w, uint32_t now_ang, int now_x, int now_y);
void clear_visited(world_t* w, int now_x, int now_y);



void start_automapping_from_compass();
void start_automapping_skip_compass();
void stop_automapping();
void start_automap_only_compass();
void autofsm();
void dbg_test();
void clear_within_robot(world_t* w, int32_t ang, int32_t x, int32_t y);
void massive_search_area();
int doing_autonomous_things();

void add_map_constraint(world_t* w, int32_t x, int32_t y);
void remove_map_constraint(world_t* w, int32_t x, int32_t y);

void provide_mcu_voxmap(world_t* w, mcu_multi_voxel_map_t* mcuvox, int32_t* acorr, int32_t* xcorr, int32_t* ycorr);

#endif
