/*
	PULUROBOT Robotsoft Computer-on-RobotBoard main software

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

	One "routing unit" is one resolevel1 unit: currently 64x64mm.

*/

//#define SEARCH_DBGPRINTS

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <inttypes.h>
#include <errno.h>
#include <dirent.h>

#include "routing.h"
#include "uthash.h"
#include "utlist.h"
#include "misc.h"

#include "voxmap.h"
#include "voxmap_memdisk.h"

#ifndef M_PI
#define M_PI 3.141592653589793238
#endif
/*
#ifdef PULU1
float routing_robot_xs = 300.0;
float routing_robot_ys = 330.0;
float routing_robot_middle_to_origin = -90.0;

#elif defined(DELIVERY_BOY)
float routing_robot_xs = 650.0 + 20.0;
float routing_robot_ys = 480.0 + 20.0;
float routing_robot_middle_to_origin = -180.0;

#else
float routing_robot_xs = 524.0;
float routing_robot_ys = 480.0;
float routing_robot_middle_to_origin = -183.0;
#endif
*/

#define MARGIN (50.0)

//static float routing_robot_xs = 850.0+(MARGIN*2.0);
//static float routing_robot_ys = 600.0+(MARGIN*2.0);
static float routing_robot_xs = 1100.0+(MARGIN*2.0);
static float routing_robot_ys = 700.0+(MARGIN*2.0);

static float routing_robot_middle_to_origin = 0; // -100.0;

static void wide_search_mode();
static void normal_search_mode();
static void tight_search_mode();


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

/*
	Routing pages are very small, because they are:
	* 2D,
	* 1 bit per pixel,
	* on resolevel 1 (64mm grid, currently)

	We can fit all of them in memory for most practical worlds.
	Limiting to 65534 pages max, 159MB of memory worst case.
	65534 pages would be enough for a continuous, gapless world of 
	400*150 pages (3.2 * 1.2 km), for example.

	16-bit index table would only take 0.5MB of memory (assuming MAX_PAGES_X
	= MAX_PAGES_Y = 512), but using pointers directly is easier, and just takes
	1.0 or 2.0MB.

	For now, we just load pages in, with no means to kick out pages. This would work
	up to about 3 km^2 worlds on Raspi3 with 1GB of memory.
*/
#define ROUTING_RL 1
#define ROUTING_UNIT (VOX_UNITS[ROUTING_RL])
#define ROUTING_MAP_PAGE_W 128

#define MM_TO_UNIT(x) ((x)/ROUTING_UNIT + VOX_XS[ROUTING_RL]*(MAX_PAGES_X/2))


#define ROUTING_PAGE_FLAG_SAVED (1<<1)
typedef struct __attribute__((packed))
{
	uint32_t dummy1;
	uint16_t dummy2;
	uint8_t dummy3;
	uint8_t flags;
	uint32_t routing[ROUTING_MAP_PAGE_W][ROUTING_MAP_PAGE_W/32 + 1];
} routing_page_t;

routing_page_t* routing_pages[MAX_PAGES_X][MAX_PAGES_Y]; // size from voxmap_memdisk.h

// Obstacle ranges in mm
#define ROUTING_Z_MIN 192
#define ROUTING_Z_MAX 1600


void clear_routing_pages()
{
	for(int xx=0; xx<MAX_PAGES_X; xx++)
	{
		for(int yy=0; yy<MAX_PAGES_Y; yy++)
		{
			if(routing_pages[xx][yy])
				free(routing_pages[xx][yy]);
		}
	}
}

static void save_routing_page(int px, int py)
{
	assert(routing_pages[px][py]);
	// Add the flag first - that way, it's in the file, and when the file is loaded, it's already marked as saved,
	// and won't trigger storing.
	routing_pages[px][py]->flags |= ROUTING_PAGE_FLAG_SAVED;

	char fname[4096];
	snprintf(fname, 4096, "./current_routing/routing_%u_%u.routing", px, py);
	FILE* f = fopen(fname, "w");
	if(!f)
	{
		printf("ERROR: Opening routing page %s for write failed: %s. Routing will only survive in memory.\n", fname, strerror(errno));
		// Keep the stored flag - trying over and over again won't help anybody. At least it will work while in memory.
		return;
	}

	// Don't bother compressing - the files are very small.
	// Future TODO: use larger units, and compress them, to really reduce disk IO.
	if(fwrite(routing_pages[px][py], sizeof(routing_page_t), 1, f) != 1 || ferror(f))
	{
		printf("ERROR: Writing to file %s failed.\n", fname);
	}

	fclose(f);
	return;
}

static void load_routing_page(int px, int py)
{
	// Only used at init: check that the page isn't allocated.
	assert(!routing_pages[px][py]);

	printf("INFO: load_routing_page: Allocating and reading new page %d,%d\n", px, py);
	routing_pages[px][py] = malloc(sizeof(routing_page_t));
	assert(routing_pages[px][py]);

	char fname[4096];
	snprintf(fname, 4096, "./current_routing/routing_%u_%u.routing", px, py);
	FILE* f = fopen(fname, "r");
	if(!f)
	{
		printf("ERROR: Opening routing page %s for read failed: %s. No routing available.\n", fname, strerror(errno));
		free(routing_pages[px][py]);
		routing_pages[px][py] = NULL;
		return;
	}

	if(fread(routing_pages[px][py], sizeof(routing_page_t), 1, f) != 1 || ferror(f))
	{
		printf("ERROR: Reading file %s failed.\n", fname);
		free(routing_pages[px][py]);
		routing_pages[px][py] = NULL;
	}

	fclose(f);
	return;
}

/*
	Scan for any existing routing page, and load them all.

	Only use at the init.
*/
void load_routing_pages()
{
	DIR *d;
	struct dirent *dir;
	d = opendir("./current_routing");

	if(d)
	{
		while( (dir = readdir(d)) != 0)
		{
			uint32_t px, py;

			if(sscanf(dir->d_name, "routing_%u_%u.routing", &px, &py) == 2)
			{
				load_routing_page(px, py);
			}

		}

		closedir(d);
	}
	else
	{
		printf("ERROR: directory ./current_routing doesn't exists! Please create it.\n");
		abort();
	}
}

void delete_routing_pages()
{
	DIR *d;
	struct dirent *dir;
	d = opendir("./current_routing");

	if(d)
	{
		while( (dir = readdir(d)) != 0)
		{
			uint32_t px, py;

			if(sscanf(dir->d_name, "routing_%u_%u.routing", &px, &py) == 2)
			{
				int rc;
				char joo[4096];
				snprintf(joo, 4095, "./current_routing/%s", dir->d_name);
				joo[4095] = '\0';
				if( (rc = unlink(joo)) < 0)
				{
					printf("WARNING: cannot delete routing file %s: %s\n", joo, strerror(errno));
				}
			}
		}

		closedir(d);
	}
	else
	{
		printf("ERROR: directory ./current_routing doesn't exists! Please create it.\n");
		abort();
	}

}

/*
	Call whenever you have time. Saves non-saved routing pages, but not too many at once, preventing long blocking.
*/
void manage_routing_page_saves()
{
	static int xx, yy;

	// Saving stops when either 1 page is saved, or when 1/5th of the page range is exhausted.
	int rounds = MAX_PAGES_X*MAX_PAGES_Y/5;

	while(rounds--)
	{
		if(routing_pages[xx][yy] && !(routing_pages[xx][yy]->flags & ROUTING_PAGE_FLAG_SAVED))
		{
			save_routing_page(xx, yy);
			break; // One actual save is enough!
		}

		yy++;

		if(yy >= MAX_PAGES_Y)
		{
			yy = 0;
			xx++;

			if(xx >= MAX_PAGES_X)
			{
				xx = 0;
			}
		}
	}
}

void save_routing_pages()
{
	for(int xx = 0; xx < MAX_PAGES_X; xx++)
	{
		for(int yy=0; yy < MAX_PAGES_Y; yy++)
		{
			if(routing_pages[xx][yy])
				save_routing_page(xx, yy);
		}
	}
		
}

/*
	Convert a resolevel1 voxmap into a routing page.
	Outputs a full routing page, plus a 32-bit wide stride into the next lower y index routing page,
	allocating two routing pages (hence name _pages in plural)
*/

void voxmap_to_routing_pages(voxmap_t* vm)
{
	// Just ignore wrong resolevel voxmaps:
	if(vm->header.xy_step_mm != VOX_UNITS[ROUTING_RL])
	{
		//printf("INFO: voxmap_to_routing_pages: Ignoring wrong resolevel voxmap, xy_step = %d mm\n", vm->header.xy_step_mm);
		return;
	}

	po_coords_t po = po_coords(vm->header.ref_x_mm, vm->header.ref_y_mm, vm->header.ref_z_mm, ROUTING_RL);

	int z_min_p, z_min_o, z_max_p, z_max_o;
	po_coords_t z_min = po_coords(0,0, ROUTING_Z_MIN, ROUTING_RL);
	po_coords_t z_max = po_coords(0,0, ROUTING_Z_MAX, ROUTING_RL);

	z_min_p = z_min.pz;
	z_min_o = z_min.oz;
	z_max_p = z_max.pz;
	z_max_o = z_max.oz;

	assert(z_min_o >= 0 && z_min_o < VOX_ZS[ROUTING_RL]);
	assert(z_max_o >= 0 && z_max_o < VOX_ZS[ROUTING_RL]);

	// Ignore completely out-of-interest pages:
	if(   po.pz < z_min_p
	   || po.pz > z_max_p)
	{
		//printf("INFO: voxmap_to_routing_pages: Ignoring out of z range voxmap, pz = %d, interest [%d..%d]\n", po.pz, z_min_p, z_max_p);
		return;
	}

	if(z_min_p != z_max_p)
	{
		// z_min and z_max are on different pages - adjust the offset ranges to only loop through the current page
		// Note that the other page of interest will get to this function later.

		assert(z_max_p == z_min_p+1); // Must be neighbors: True for any sane values (map page height more than robot height)

		if(z_min_p != po.pz)
		{
			assert(z_max_p == po.pz);

			// z_max_o is correct already, but z_min_o must start from 0.
			z_min_o = 0;
		}
		else
		{
			assert(z_min_p == po.pz);

			// z_min_o is correct already, but z_max_o must end at the maximum possible value.
			z_max_o = VOX_ZS[ROUTING_RL];
		}
	}
	else
	{
		// z_min and z_max are on the same page
		assert(z_min_o < z_max_o);
	}



	assert(vm->header.xy_step_mm == VOX_UNITS[ROUTING_RL] && vm->header.xs == ROUTING_MAP_PAGE_W && vm->header.ys == ROUTING_MAP_PAGE_W);
	assert(vm->header.xs == VOX_XS[ROUTING_RL] && vm->header.ys == VOX_YS[ROUTING_RL] && vm->header.zs == VOX_ZS[ROUTING_RL]);


	if(!routing_pages[po.px][po.py])
	{
		//printf("INFO: voxmap_to_routing_pages: Allocating new page %d,%d\n", po.px, po.py);
		routing_pages[po.px][po.py] = calloc(1, sizeof(routing_page_t));
		assert(routing_pages[po.px][po.py]);
	}
	else
	{
		//printf("INFO: voxmap_to_routing_pages: clearing existing page %d,%d\n", po.px, po.py);
		memset(routing_pages[po.px][po.py], 0, sizeof(routing_page_t));
	}

	if(po.py > 0)
	{
		if(!routing_pages[po.px][po.py-1])
		{
			//printf("INFO: voxmap_to_routing_pages: Allocating new page %d,%d\n", po.px, po.py-1);
			routing_pages[po.px][po.py-1] = calloc(1, sizeof(routing_page_t));
			assert(routing_pages[po.px][po.py-1]);
		}
		else
		{
			//printf("INFO: voxmap_to_routing_pages: clearing existing page slice %d,%d\n", po.px, po.py-1);
			for(int xx=0; xx<VOX_XS[ROUTING_RL]; xx++)
			{
				routing_pages[po.px][po.py-1]->routing[xx][ROUTING_MAP_PAGE_W/32] = 0;
			}
		}
	}


	//printf("INFO: voxmap_to_routing_pages: converting %d,%d,%d\n", po.px, po.py, po.pz);
	for(int yy=0; yy<VOX_YS[ROUTING_RL]; yy++)
	{
		for(int xx=0; xx<VOX_XS[ROUTING_RL]; xx++)
		{
			for(int zz=z_min_o; zz<z_max_o; zz++)
			{
				if(VOXEL_FASTEST(*vm, xx, yy, zz, VOX_XS[ROUTING_RL], VOX_YS[ROUTING_RL], VOX_ZS[ROUTING_RL]) & 0x0f)
				{
					int y = yy/32;
					int y_remain = yy%32;
					routing_pages[po.px][po.py]->routing[xx][y] |= 1<<y_remain;

					if(y == 0)
					{
						// Duplicated Y stride on the lower y-index routing page:
						routing_pages[po.px][po.py-1]->routing[xx][ROUTING_MAP_PAGE_W/32] |= 1<<y_remain;
					}
					break; // out of z loop; one obstacle is enough to make it fully blocked, continue on next x.
				}

			}
		}
	}
}

#include "slam_cloud.h"
void cloud_to_routing_pages(cloud_t const * const cloud)
{

	for(int i=0; i<cloud->m.n_points; i++)
	{
		int x_mm =(cloud->m.ref_x+(int)cloud->points[i].x)*CLOUD_MM;
		int y_mm = (cloud->m.ref_y+(int)cloud->points[i].y)*CLOUD_MM;
		int z_mm = (cloud->m.ref_z+(int)cloud->points[i].z)*CLOUD_MM;


		if(z_mm >= ROUTING_Z_MIN && z_mm <= ROUTING_Z_MAX)
		{
			po_coords_t po = po_coords(x_mm, y_mm, z_mm, ROUTING_RL);

			if(!routing_pages[po.px][po.py])
			{
				routing_pages[po.px][po.py] = calloc(1, sizeof(routing_page_t));
				assert(routing_pages[po.px][po.py]);
			}

			// Routing algorithm needs one extra y stride per routing page for efficiency; this is just a copy
			// of the stride on the "actual" page. Say page (10,10) has something at offset (123, 0). Copy of this
			// needs to be in page (10, 9) offset (123, 256/32 (last+1))
			if(!routing_pages[po.px][po.py-1])
			{
				routing_pages[po.px][po.py-1] = calloc(1, sizeof(routing_page_t));
				assert(routing_pages[po.px][po.py-1]);
			}

			
			int y_whole = po.oy/32;
			int y_remain = po.oy%32;

			assert(po.ox >= 0 && po.ox < ROUTING_MAP_PAGE_W);
			assert(y_whole >= 0 && y_whole < ROUTING_MAP_PAGE_W/32 + 1);

			routing_pages[po.px][po.py]->routing[po.ox][y_whole] |= 1<<y_remain;

			if(y_whole == 0)
			{
				// Duplicated Y stride on the lower y-index routing page:
				routing_pages[po.px][po.py-1]->routing[po.ox][ROUTING_MAP_PAGE_W/32] |= 1<<y_remain;
			}
		}
	}

}


#define MAX_F 99999999999999999.9

float robot_shape_x_len;
#define ROBOT_SHAPE_WINDOW 32
uint32_t robot_shapes[32][ROBOT_SHAPE_WINDOW];

//static int kakka = 0;
ALWAYS_INLINE int check_hit(int x, int y, int direction)
{

//	kakka++;
//	if(kakka > 1) abort();

	#ifdef SEARCH_DBGPRINTS
		int retval = 0;
		printf("check_hit(%d, %d, %d): \n", x, y, direction);
	#endif
	for(int chk_x=0; chk_x<ROBOT_SHAPE_WINDOW; chk_x++)
	{
		po_coords_t po = po_unit_coords(x-ROBOT_SHAPE_WINDOW/2+chk_x, y-ROBOT_SHAPE_WINDOW/2, 0, ROUTING_RL);

		int yoffs = po.oy/32;
		int yoffs_remain = po.oy - yoffs*32;

		#ifdef SEARCH_DBGPRINTS
//			printf("chk_x=%d, p(%d,%d), xoffs=%d, yoffs=%d, remain=%d: ", chk_x, po.px, po.py, po.ox, yoffs, yoffs_remain);
		#endif

		if(!routing_pages[po.px][po.py]) // out of bounds (not allocated) - assume free space
		{
//			printf("pages[%d][%d] not allocated\n", po.px, po.py);
//			printf("x = %d  y = %d  direction = %d\n", x, y, direction);
			//exit(1);
			#ifdef SEARCH_DBGPRINTS
				printf("no_alloc\n");
			#endif
			continue;
		}

		// Now the quick comparison, which could be even faster, but we don't want to mess up the compatibility between
		// big endian / little endian systems.

		uint64_t shape = (uint64_t)robot_shapes[direction][chk_x] << (yoffs_remain);
		uint64_t map = (((uint64_t)routing_pages[po.px][po.py]->routing[po.ox][yoffs+1]<<32) |
		   (uint64_t)routing_pages[po.px][po.py]->routing[po.ox][yoffs]);

		#ifdef SEARCH_DBGPRINTS
//			printf("\nshape: ");
//			for(int i = 64; i>=0; i--){ if(shape&(1ULL<<i)) putchar('#'); else putchar('-');}
			for(int i = 64; i>=0; i--)
			{
				if((map&(1ULL<<i)) && (shape&(1ULL<<i)))
					putchar('X'); 
				else if((map&(1ULL<<i)) && !(shape&(1ULL<<i)))
					putchar('+');
				else if(!(map&(1ULL<<i)) && (shape&(1ULL<<i)))
					putchar('O');
				else
					putchar(' ');

			}
	
		#endif

		if(map & shape)
		{
			#ifdef SEARCH_DBGPRINTS
				printf(" --> HITS!\n");
				retval = 1;
			#else
				return 1;
			#endif
		}
		#ifdef SEARCH_DBGPRINTS
		else
			printf("\n");
		#endif
	}

	#ifdef SEARCH_DBGPRINTS
		if(!retval)
			printf(" --> no hit!\n");

		return retval;
	#else
		return 0;
	#endif
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

#ifdef SEARCH_DBGPRINTS
	printf("test_robot_turn start=%.1f end=%.1f, da=%.1f, dir_cur=%d, dir_end=%d\n", RADTODEG(start), RADTODEG(end), RADTODEG(da), dir_cur, dir_end);
#endif
	while(dir_cur != dir_end)
	{
#ifdef SEARCH_DBGPRINTS
		printf("dir %d", dir_cur);
#endif
		if(check_hit(x, y, dir_cur))
		{
			return 0;
		}

#ifdef SEARCH_DBGPRINTS
		printf("\n");
#endif
		if(cw) dir_cur--; else dir_cur++;

		if(dir_cur < 0) dir_cur = 31;
		else if(dir_cur > 31) dir_cur = 0;

	}

	return 1;
}


static int line_of_sight(route_xy_t p1, route_xy_t p2)
{
	int dx = p2.x - p1.x;
	int dy = p2.y - p1.y;

	float step = ((robot_shape_x_len-10.0)/ROUTING_UNIT);

	float len = sqrt(sq(dx) + sq(dy));

	int terminate = 0;

	float pos = step/2.0;
	if(pos > len) {pos = len; terminate = 1;}

	float ang = atan2(dy, dx);
	if(ang < 0.0) ang += 2.0*M_PI;
	int dir = (ang/(2.0*M_PI) * 32.0)+0.5;
	if(dir < 0) dir = 0; else if(dir > 31) dir = 31;

#ifdef SEARCH_DBGPRINTS
	printf("line_of_sight (%d,%d)->(%d,%d), dx=%d, dy=%d, ang = %.1f  dir = %d, step=%.1f, len=%.1f\n", p1.x, p1.y, p2.x, p2.y, dx, dy, RADTODEG(ang), dir, step, len);
#endif
	while(1)
	{
		int x = (cos(ang)*pos + (float)p1.x)+0.5;
		int y = (sin(ang)*pos + (float)p1.y)+0.5;

//		printf("check_hit(%d, %d, %d) = ", x, y, dir);

#ifdef SEARCH_DBGPRINTS
		printf("LOS pos=%.2f: ", pos);
#endif		
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

	float step = ((robot_shape_x_len-10.0)/ROUTING_UNIT);

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

int minimap_find_mapping_dir(float ang_now, int32_t* x, int32_t* y, int32_t desired_x, int32_t desired_y, int* back)
{
	extern int32_t cur_ang;
	extern int cur_x, cur_y;


	wide_search_mode();

	#define NUM_FWDS 10
	const float fwds[NUM_FWDS] = {1000.0, 750.0, 500.0, 400.0, -500.0, -300.0, 200.0, -150.0, 100.0, -100.0};

	int num_cango_places = 0;
	route_xy_t cango_places[MAX_CANGOS];
	route_xy_t internal_cango_places[MAX_CANGOS];
	int backs[MAX_CANGOS];
	int disagrees = 0;

	int in_tight_spot = 0;

	route_xy_t start = {0, 0};

	//dbg_save_minimap();
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
					route_xy_t end = {(int)(cos(ang_to)*fwd_len/(float)ROUTING_UNIT),
							  (int)(sin(ang_to)*fwd_len/(float)ROUTING_UNIT)};

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



#define SHAPE_PIXEL(shape, x, y) { robot_shapes[shape][(x)] |= 1UL<<(y);}
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

int reverse_shapes = 0;
int tight_shapes = 0;

static void draw_robot_shape(int a_idx, float ang)
{
//	if(reverse_shapes)
//		ang += M_PI;

	float o_x = (ROBOT_SHAPE_WINDOW/2.0)*(float)ROUTING_UNIT;
	float o_y = (ROBOT_SHAPE_WINDOW/2.0)*(float)ROUTING_UNIT;

	float robot_xs, robot_ys;

	float middle_xoffs; // from o_x, o_y to the robot middle point.
	float middle_yoffs = -0.0;
	middle_xoffs = routing_robot_middle_to_origin;

	const float extra_x = 0.0, extra_y = 0.0;

	if(tight_shapes == 2)
	{
		robot_xs = (routing_robot_xs - 20.0 + extra_x);
		robot_ys = (routing_robot_ys - 20.0 + extra_y);
	}
	else if(tight_shapes == 1)
	{
		robot_xs = (routing_robot_xs + 85.0 + extra_x);
		robot_ys = (routing_robot_ys + 50.0 + extra_y);
	}
	else if(tight_shapes == 0)
	{
		robot_xs = (routing_robot_xs + 100.0 + extra_x);
		robot_ys = (routing_robot_ys + 100.0 + extra_y);
	}
	else // wide
	{
		robot_xs = (routing_robot_xs + 160.0 + extra_x);
		robot_ys = (routing_robot_ys + 160.0 + extra_y);
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
	draw_triangle(a_idx, x1/(float)ROUTING_UNIT, y1/(float)ROUTING_UNIT, x2/(float)ROUTING_UNIT, y2/(float)ROUTING_UNIT, x3/(float)ROUTING_UNIT, y3/(float)ROUTING_UNIT);
	// Second, triangle 1-3-4
	draw_triangle(a_idx, x1/(float)ROUTING_UNIT, y1/(float)ROUTING_UNIT, x3/(float)ROUTING_UNIT, y3/(float)ROUTING_UNIT, x4/(float)ROUTING_UNIT, y4/(float)ROUTING_UNIT);
}


static void gen_robot_shapes()
{
	// Generate lookup tables showing the shape of robot in mapping unit matrices in different orientations.
	// These are used in mapping to test whether the (x,y) coords result in some part of a robot hitting a wall.

        memset(robot_shapes, 0, sizeof(robot_shapes));

//	FILE* f_dbg_shapes = fopen("dbg_shapes.txt", "a");

	for(int a=0; a<32; a++)
	{
		draw_robot_shape(a, ((float)a*2.0*M_PI)/32.0);

/*
		if(f_dbg_shapes)
		{
			fprintf(f_dbg_shapes, "a = %d, tight_shapes = %d\n", a, tight_shapes);

			for(int y = 0; y < ROBOT_SHAPE_WINDOW; y++)
			{
				for(int x = 0; x < ROBOT_SHAPE_WINDOW; x++)
				{
					if(y==ROBOT_SHAPE_WINDOW/2 && x==ROBOT_SHAPE_WINDOW/2)
						fprintf(f_dbg_shapes, "X ");
					else	
						fprintf(f_dbg_shapes, (robot_shapes[a][x]&(1UL<<(31-y)))?"# ":"  ");
				}
				fprintf(f_dbg_shapes, "\n");
			}
		}
*/
	}

//	if(f_dbg_shapes) fclose(f_dbg_shapes);
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
	if(*route == NULL)
		return;

	route_unit_t *elt, *tmp;
	DL_FOREACH_SAFE(*route,elt,tmp)
	{
		DL_DELETE(*route,elt);
		free(elt);
	}
	*route = NULL;
}

void routing_unit_coords(int mm_x, int mm_y, int* unit_x, int* unit_y)
{
	int unit_x_t = mm_x / ROUTING_UNIT;
	int unit_y_t = mm_y / ROUTING_UNIT;
	unit_x_t += VOX_XS[ROUTING_RL]*(MAX_PAGES_X/2);
	unit_y_t += VOX_YS[ROUTING_RL]*(MAX_PAGES_Y/2);

	*unit_x = unit_x_t;
	*unit_y = unit_y_t;
}

void mm_from_routing_unit_coords(int unit_x, int unit_y, int* mm_x, int* mm_y)
{
	unit_x -= VOX_XS[ROUTING_RL]*(MAX_PAGES_X/2);
	unit_y -= VOX_XS[ROUTING_RL]*(MAX_PAGES_X/2);

	*mm_x = unit_x * ROUTING_UNIT;
	*mm_y = unit_y * ROUTING_UNIT;
}


static int search(route_unit_t **route, float start_ang, int start_x_mm, int start_y_mm, int end_x_mm, int end_y_mm, int change_to_normal, int accept_dist_blocks)
{
	search_unit_t* closed_set = NULL;
	search_unit_t* open_set = NULL;

	clear_route(route);

	int s_x, s_y, e_x, e_y;
	routing_unit_coords(start_x_mm, start_y_mm, &s_x, &s_y);
	routing_unit_coords(end_x_mm, end_y_mm, &e_x, &e_y);

	while(start_ang >= 2.0*M_PI) start_ang -= 2.0*M_PI;
	while(start_ang < 0.0) start_ang += 2.0*M_PI;
	int start_dir = (start_ang/(2.0*M_PI) * 32.0)+0.5;

	//printf("Start %d,%d,  end %d,%d  start_ang=%f  start_dir=%d\n", s_x, s_y, e_x, e_y, start_ang, start_dir);


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

		if(cnt > 25000)
		{
			printf("Giving up at cnt = %d\n", cnt);
			return 3;
		}

		if(change_to_normal && cnt == 201)
		{
			normal_search_mode();
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

		// See if we are close enough:
		int64_t goal_dx = p_cur->loc.x - e_x;
		int64_t goal_dy = p_cur->loc.y - e_y;

		int64_t distsq = sq(goal_dx) + sq(goal_dy);
		int64_t acceptsq = sq(accept_dist_blocks);

//		if(p_cur->loc.x == e_x && p_cur->loc.y == e_y)
		if(distsq <= acceptsq)
		{

			printf("Solution found, cnt = %d\n", cnt);

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
			while(rt && rt->next && rt->next->next)
			{
				if(line_of_sight(rt->loc, rt->next->next->loc))
				{
					//printf("Deleting.\n");
					route_unit_t *tmp = rt->next;
					DL_DELETE(*route, tmp); // crash
					free(tmp);
				}
				else
					rt = rt->next;
			}

			// Remove the first, because it's the starting point.
			route_unit_t *tm = *route;
			if(tm)
			{
				DL_DELETE(*route, tm);
				free(tm);
			}

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
					if(reverse_shapes) ang+=M_PI;
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
					if(reverse_shapes) ang+=M_PI;
					if(ang < 0.0) ang += 2.0*M_PI;
					int dir_parent = ang/(2.0*M_PI) * 32.0;
					if(dir_parent < 0) dir_parent = 0; else if(dir_parent > 31) dir_parent = 31;
					direction_from_neigh_parent = dir_parent;
				}

				int direction = 0;
				if(direction_from_cur_parent < 0 && direction_from_neigh_parent < 0)  // if the previous two don't work out.
				{
					if(!reverse_shapes)
					{
						if(xx==1 && yy==0)        direction = 0*4; 
						else if(xx==1 && yy==1)   direction = 1*4; 
						else if(xx==0 && yy==1)   direction = 2*4; 
						else if(xx==-1 && yy==1)  direction = 3*4; 
						else if(xx==-1 && yy==0)  direction = 4*4; 
						else if(xx==-1 && yy==-1) direction = 5*4; 
						else if(xx==0 && yy==-1)  direction = 6*4; 
						else if(xx==1 && yy==-1)  direction = 7*4; 
					}
					else
					{
						if(xx==1 && yy==0)        direction = 4*4; 
						else if(xx==1 && yy==1)   direction = 5*4; 
						else if(xx==0 && yy==1)   direction = 6*4; 
						else if(xx==-1 && yy==1)  direction = 7*4; 
						else if(xx==-1 && yy==0)  direction = 0*4; 
						else if(xx==-1 && yy==-1) direction = 1*4; 
						else if(xx==0 && yy==-1)  direction = 2*4; 
						else if(xx==1 && yy==-1)  direction = 3*4; 
					}
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
					if(!test_robot_turn(p_cur->loc.x, p_cur->loc.y, start_ang, ((float)direction/32.0)*2.0*M_PI))
					{
						//printf("Robot cannot turn to direction %d\n", direction);
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
					HASH_ADD(hh, open_set, loc,sizeof(route_xy_t), p_neigh); // crash

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


change_to_normal: enable to call normal_search_mode() after some distance; with tight mode initially, has better
chances of getting out of tight spot, but then switches to normal limits.

*/


int search2(route_unit_t **route, float start_ang, int start_x_mm, int start_y_mm, int end_x_mm, int end_y_mm, int change_to_normal, int accept_dist_blocks)
{

	printf("search2() start_ang=%.1f deg  start (%d,%d), end (%d, %d), accept_dist_blocks=%d\n", RADTODEG(start_ang), start_x_mm, start_y_mm, end_x_mm, end_y_mm, accept_dist_blocks);
#define SRCH_NUM_A 23
	static const int a_s[SRCH_NUM_A] =   // back-off angles in degs
	{	0,	-4,	4,	-8,	8,	-12,	12,	-18,	18,	-24,	24,	-36,	36,	-48,	48,	-60,	60,	-72,	72, 	-84,	84,	-96,	96	};


#define SRCH_NUM_BACK 18
	static const int b_s[SRCH_NUM_BACK] = // back-offs in mm - positive backoffs are actually forward :-).
	{	80,	120,	160,	240,	280,	320,	400,	480,	-320,	-400,	-280,	-240,	-200,	-480,	-160,	-120,	-80,	-560	};


	// If going forward doesn't work out from the beginning, try backing off slightly.

	int ret = search(route, start_ang, start_x_mm, start_y_mm, end_x_mm, end_y_mm, change_to_normal, accept_dist_blocks);

	if(ret == 0)
	{
/*		printf("search2(): search() returned 0 (success). Route:\n");

		route_unit_t *rt;
		DL_FOREACH(*route, rt)
		{
			if(rt->backmode)
				printf(" REVERSE ");
			else
				printf("         ");

			printf("to %d,%d\n", rt->loc.x, rt->loc.y);
		}
*/

		return 0;
	}

	if(ret == 1)
	{
		printf("Search fails in the start - trying to back off.\n");

		for(int a_idx = 0; a_idx < SRCH_NUM_A; a_idx++)
		{
			for(int back_idx = 0; back_idx < SRCH_NUM_BACK; back_idx++)
			{
				float new_ang = start_ang + ((reverse_shapes?0:0)+2.0*M_PI*(float)a_s[a_idx]/360.0);
				while(new_ang >= 2.0*M_PI) new_ang -= 2.0*M_PI;
				while(new_ang < 0.0) new_ang += 2.0*M_PI;
				int new_x = start_x_mm + cos(new_ang)*(reverse_shapes?-1:1)*b_s[back_idx];
				int new_y = start_y_mm + sin(new_ang)*(reverse_shapes?-1:1)*b_s[back_idx];

				int dir = (new_ang/(2.0*M_PI) * 32.0)+0.5;
				if(dir < 0) dir = 0; else if(dir > 31) dir = 31;

				int new_x_units, new_y_units;
				routing_unit_coords(new_x, new_y, &new_x_units, &new_y_units);

				//printf("Back off ang=%.2f deg, mm = %d  -> new start = (%d, %d) --> ", TODEG(new_ang), b_s[back_idx], new_x_units, new_y_units);

				if(check_hit(new_x_units, new_y_units, dir))
				{
				//	printf("backing off hits the wall.\n");
				}
				else
				{
					int ret = search(route, new_ang, new_x, new_y, end_x_mm, end_y_mm, change_to_normal, accept_dist_blocks);
					if(ret == 0)
					{
						printf("Search succeeded (back off ang=%.1fdeg, mm = %d), stopping back-off search.\n", TODEG(new_ang), (reverse_shapes?-1:1)*b_s[back_idx]);

						route_unit_t* point = malloc(sizeof(route_unit_t));
						point->loc.x = new_x_units; point->loc.y = new_y_units;
						point->backmode = (b_s[back_idx]<0)?1:0;
						DL_PREPEND(*route, point);

						return 0;
					}
					else if(ret > 1)
					{
						printf("Search failed later than in the beginning, stopping back-off search.\n");
						return 2;
					}
				}

			}
		}
		return 1;
	}

	printf("search2(): search() returned %d, search2() fails with rc=3\n", ret);

	return 3;

}

// Reverse: search running in opposite direction, i.e., "initial back-off" goes forward, otherwise we go backwards.
// Note: You need to manually invert the resulting backmode bytes.
int search_route(route_unit_t **route, float start_ang, int start_x_mm, int start_y_mm, int end_x_mm, int end_y_mm, int reverse)
{
	reverse_shapes = reverse;

	int retval = 0;


	printf("Searching with wide limits...\n");

	wide_search_mode();
	//normal_search_mode();
	if(search2(route, start_ang, start_x_mm, start_y_mm, end_x_mm, end_y_mm, 0, 0))
	{
		normal_search_mode();
		printf("Searching with normal limits...\n");

		int ret;
		if( (ret = search2(route, start_ang, start_x_mm, start_y_mm, end_x_mm, end_y_mm, 0, 0)) )
		{
			printf("Search failed - retrying with tighter limits.\n");
			tight_search_mode();
			if( (ret = search2(route, start_ang, start_x_mm, start_y_mm, end_x_mm, end_y_mm, 1, 0)) ) 
			{
				printf("Tight search failed - trying to get halfway\n");
				double direct_len = sqrt(sq(end_x_mm-start_x_mm)+sq(end_y_mm-start_y_mm));
				direct_len = direct_len*2.0/3.0;
				int acceptance = direct_len/ROUTING_UNIT;

				if( (ret = search2(route, start_ang, start_x_mm, start_y_mm, end_x_mm, end_y_mm, 1, acceptance)) ) 
				{
					printf("Halfway failed - gave up.\n");	
					return ret;
				}
				else
				{
					printf("Found route HALFWAY.\n");
					retval = -999;
				}
			}
			else
				printf("Found route with TIGHT limits\n");
		}
		else
			printf("Found route with normal limits\n");
	}
	else
		printf("Found route with WIDE limits\n");


	// tight_search_mode is put into action so that collision avoidance / step skipping/rounding can use it.
	// TODO: fix this state horror. 
	tight_search_mode();
	return retval;
}

/*

	1>>2  1>>2
	^  v  ^  v
	^  v  ^  .
	^  v  ^  .
	0  v  ^  .
	   v  ^
	   v  ^
	   v  ^
	   3>>0

	* = start_x, start_y, start_ang required to check if the robot can turn initially
	advance_ang = denoted by >
	first_ang = denoted by ^
	back_ang  = denoted by v

*/
int vacuum_route2(route_unit_t **route, float start_ang, int start_x_mm, int start_y_mm, float advance_ang, int reverse, int* end_x_mm, int* end_y_mm, float* end_ang)
{
	int s_x, s_y;
	routing_unit_coords(start_x_mm, start_y_mm, &s_x, &s_y);

	float first_ang = advance_ang + M_PI/2.0;

	if(first_ang > 2.0*M_PI) first_ang -= 2.0*M_PI;

	float back_ang = advance_ang - M_PI/2.0;
	if(back_ang < 0.0) back_ang += 2.0*M_PI;


	normal_search_mode();

	route_xy_t now = {s_x, s_y};

	int advance_len = 7;
	int advance_beneficial_len = advance_len - 5;
	if(advance_beneficial_len < 0) advance_beneficial_len = 0;

	if(!test_robot_turn(s_x, s_y, start_ang, first_ang))
	{
		printf("vacuum_route: robot cannot initially turn\n");
		if(end_x_mm != NULL) *end_x_mm = start_x_mm;
		if(end_y_mm != NULL) *end_y_mm = start_y_mm;
		if(end_ang != NULL) *end_ang = start_ang;
		return -1;
	}

	int retval = 0;

	int do_partial = 0;
	float ang1, ang2;
	for(int stride = 0; stride < 100; stride++)
	{
		// binary search

		int max_len = 2000; // that doesn't work
		int min_len = 0;    // that works

		while(1)
		{
			int now_len = (min_len + max_len)/2;

			//printf("now_len = %d ", now_len);

			ang1 = (stride&1)?back_ang:first_ang;
			ang2 = (stride&1)?first_ang:back_ang;

			route_xy_t p0 = now;
			route_xy_t p1 = {p0.x + cos(ang1)*now_len, p0.y + sin(ang1)*now_len};
			route_xy_t p2 = {p1.x + cos(advance_ang)*advance_len, p1.y + sin(advance_ang)*advance_len};

			if(now_len <= min_len) // can't be improved further
			{
				//printf("can't be improved further\n");

				if(now_len >= 3)
				{
					{
						route_unit_t* point = malloc(sizeof(route_unit_t));
						point->loc = p1;
						point->backmode = 0; // done by the original caller later
						DL_APPEND(*route, point);
						retval += now_len;
					}

					if(!do_partial)
					{
						route_unit_t* point = malloc(sizeof(route_unit_t));
						point->loc = p2;
						point->backmode = 0; // done by the original caller later
						DL_APPEND(*route, point);

						retval += advance_beneficial_len;
					}
					else
						goto STOP_ROUTE;

					now = p2;
					break;
				}
				else
				{
					if(!do_partial)
					{
						// Try the same again, without going further in advance_ang
						do_partial = 1;
						stride--; // keep the same stride number
						break;
					}
					else
					{
						goto STOP_ROUTE;
					}
				}
			}

			//printf("(%d,%d)->(%d,%d)->(%d,%d)\n", p0.x, p0.y, p1.x, p1.y, p2.x, p2.y);

			if(
				line_of_sight(p0, p1) &&
				(do_partial || 
				(test_robot_turn(p1.x, p1.y, ang1, advance_ang) &&
				line_of_sight(p1, p2) &&
				test_robot_turn(p2.x, p2.y, advance_ang, ang2)))
			  )
			{
				// now_len works out
				//printf("works\n");
				min_len = now_len;
			}
			else
			{
				// now_len doesn't work out
				//printf("doesn't work\n");
				max_len = now_len;
			}
		}

	}

	STOP_ROUTE:;

	if(end_x_mm != NULL && end_y_mm != NULL)
		mm_from_routing_unit_coords(now.x, now.y, end_x_mm, end_y_mm);

	if(end_ang != NULL)
		*end_ang = do_partial?ang1:ang2;

	tight_search_mode();

	return retval;
}

int vacuum_route_combo(route_unit_t **route, float start_ang, int start_x_mm, int start_y_mm, float advance_ang, int reverse)
{
	route_unit_t* route1 = NULL;
	route_unit_t* route2 = NULL;
	route_unit_t* route3 = NULL;
	route_unit_t* route4 = NULL;

	int vac_end_x, vac_end_y;
	float vac_end_ang;

	// vacuum path from start_*_mm to wherever it takes
	printf("generate vacuum route1 from %d,%d in advance_ang=%.1f deg\n", start_x_mm, start_y_mm, RADTODEG(advance_ang));
	int len1 = vacuum_route2(&route1, start_ang, start_x_mm, start_y_mm, advance_ang, reverse, &vac_end_x, &vac_end_y, &vac_end_ang);
	printf("len1=%d\n", len1);

	// route finding from the end of the vacuum path back to start_*_mm
	printf("find route2 back to start\n");
	if(search2(&route2, vac_end_ang, vac_end_x, vac_end_y, start_x_mm, start_y_mm, 0, 0))
	{
		printf("can't find route2 back to start\n");
		return -1;		
	}

	DL_CONCAT(*route, route1);
	DL_CONCAT(*route, route2);


	// vacuum path from start_*_mm to wherever it takes - in opposite direction
	advance_ang += M_PI;
	if(advance_ang > 2.0*M_PI)
		advance_ang -= 2.0*M_PI;

	printf("generate vacuum route3 from %d,%d in advance_ang=%.1f deg\n", start_x_mm, start_y_mm, RADTODEG(advance_ang));
	int len3 = vacuum_route2(&route3, start_ang, start_x_mm, start_y_mm, advance_ang, reverse, &vac_end_x, &vac_end_y, &vac_end_ang);
	printf("len3=%d\n", len3);

	// find route back to the start
//	printf("find route2 back to start\n");
//	if(search2(&route4, vac_end_ang, vac_end_x, vac_end_y, start_x_mm, start_y_mm, 0, 0))
//	{
//		printf("can't find route4 back to start\n");
//		return -1;		
//	}

	DL_CONCAT(*route, route3);
//	DL_CONCAT(*route, route4);

	int total_len = len1+len3;
	printf("total len = %d\n", total_len);

	return total_len;

}

int vacuum_route(route_unit_t **route, float start_ang, int start_x_mm, int start_y_mm,int reverse)
{
	int s_x, s_y;
	int best_len = -1;
	float advance_ang = 0.0, best_ang = 0.0;

	int backoff_start_x_mm = start_x_mm, backoff_start_y_mm = start_y_mm;

	for(int backoff = 1000; backoff >= 200; backoff -= 50)
	{
		double test_backoff = backoff + 600;
		routing_unit_coords(start_x_mm, start_y_mm, &s_x, &s_y);
		route_xy_t start = {s_x, s_y};
		routing_unit_coords(start_x_mm + cos(start_ang)*test_backoff, start_y_mm + sin(start_ang)*test_backoff, &s_x, &s_y);
		route_xy_t end = {s_x, s_y};

		if(line_of_sight(start, end))
		{
			printf("vacuum_route(): can backoff %.0f mm, will back off %d mm\n", test_backoff, backoff);
			backoff_start_x_mm = start_x_mm + cos(start_ang)*test_backoff;
			backoff_start_y_mm = start_y_mm + sin(start_ang)*test_backoff;
			break;
		}
		else
		{
			printf("vacuum_route(): can't backoff %.0f mm\n", test_backoff);
		}
	}


	for(int i=0; i<32; i++)
	{
		advance_ang += 2.0*M_PI/32.0;
		if(advance_ang > 2.0*M_PI)
			advance_ang -= 2.0*M_PI;

		clear_route(route);
		int cur_len = vacuum_route_combo(route, start_ang, backoff_start_x_mm, backoff_start_y_mm, advance_ang, reverse);
		if(cur_len > best_len)
		{
			best_len = cur_len;
			best_ang = advance_ang;
		}
	}

	printf("best ang = %.1f deg with len=%d, doing that.\n", best_ang, best_len);

	clear_route(route);

	if(backoff_start_x_mm != start_x_mm || backoff_start_y_mm != start_y_mm)
	{
		route_unit_t* point = malloc(sizeof(route_unit_t));
		routing_unit_coords(backoff_start_x_mm, backoff_start_y_mm, &s_x, &s_y);
		route_xy_t p1 = {s_x, s_y};
		point->loc = p1;
		point->backmode = 1; 
		DL_APPEND(*route, point);
	}

	vacuum_route_combo(route, start_ang, backoff_start_x_mm, backoff_start_y_mm, best_ang, reverse);

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


int check_turn_mm(int32_t start_ang, int start_x, int start_y, int end_x, int end_y)
{
//	printf("check_turn_mm(%d, %d, %d, %d, %d)\n", start_ang, start_x, start_y, end_x, end_y);
	return check_turn(start_ang, MM_TO_UNIT(start_x), MM_TO_UNIT(start_y), MM_TO_UNIT(end_x), MM_TO_UNIT(end_y));
}
