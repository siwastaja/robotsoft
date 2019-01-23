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
#include "hwdata.h"
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


/*
	The RobotBoard generates the voxel map in 12 pieces:
	Pieces 0,1,2,3 near the robot have 50mm resolution and are 5mx5m each (total 10mx10m hires area)
	Pieces 4,5,6,7,8,9,10,11 have 100mm resolution and are 10x10m each.

	Each piece is an identical 100x100 unit thing.

	For mapping, we merge them to one buffer with unified 50mm resolution, i.e., the lores blocks are
	nearest neighbor upsampled.

	      |     |   
	   7  |  6  |  5
	      |     |   
	 -----|-----|-----
	      |1 | 0|
	   8  |-----|  4
              |2 | 3|
	 -----|-----|-----
	      |     |   
           9  | 10  | 11
	      |     |   
*/

#include "mcu_voxmap_defs.h"

#define TMPVOX_XS (6*VOX_SEG_XS)
#define TMPVOX_YS (6*VOX_SEG_YS)
#define TMPVOX_XMID (TMPVOX_XS/2)
#define TMPVOX_YMID (TMPVOX_XS/2)

int32_t tmpvox_ref_x, tmpvox_ref_y;
static uint16_t tmpvox[TMPVOX_XS*TMPVOX_YS];
static uint16_t tmpvox_lores[TMPVOX_XS*TMPVOX_YS/4];

// Out must not point to the same area as in
void rotate_tmpvox(uint16_t* restrict out, uint16_t* restrict in, double ang_rad)
{
	static uint16_t tmp_skewed[TMPVOX_XS*TMPVOX_YS];

	memset(tmp_skewed, 0, sizeof tmp_skewed);

	if(ang_rad > DEGTORAD(15) || ang_rad < DEGTORAD(-15))
	{
		printf("Error: out-of-range ang_rad in rotate_tmpvox\n");
		return;
	}
	double step = tan(ang_rad);

	// Shift everything X direction first, creating a skewed copy

	for(int yy = 0; yy < TMPVOX_YS; yy++)
	{
		double fshift = ((double)(TMPVOX_YMID - yy)*step); // Bottom line (yy=0) shifts right with positive ang.
		int shift = (fshift>=0.0)?(fshift+0.5):(fshift-0.5);

		if(shift >= 0)
		{
			for(int xx=0; xx<TMPVOX_XS-shift; xx++)
			{
				tmp_skewed[yy*TMPVOX_XS+xx+shift] = in[yy*TMPVOX_XS+xx];
			}
		}
		else
		{
			for(int xx=-1*shift; xx<TMPVOX_XS; xx++)
			{
				tmp_skewed[yy*TMPVOX_XS+xx+shift] = in[yy*TMPVOX_XS+xx];
			}
		}
	}

	// Shift in Y direction

	for(int xx = 0; xx < TMPVOX_XS; xx++)
	{
		double fshift = ((double)(xx - TMPVOX_XMID)*step); // Left column (xx=0) shifts down with positive ang.
		int shift = (fshift>=0.0)?(fshift+0.5):(fshift-0.5);

		if(shift >= 0)
		{
			for(int yy=0; yy<TMPVOX_YS-shift; yy++)
			{
				out[(yy+shift)*TMPVOX_XS+xx] = tmp_skewed[yy*TMPVOX_XS+xx];
			}
		}
		else
		{
			for(int yy=-1*shift; yy<TMPVOX_YS; yy++)
			{
				out[(yy+shift)*TMPVOX_XS+xx] = tmp_skewed[yy*TMPVOX_XS+xx];
			}
		}

	}
}

// Out must not point to the same area as in
void rotate_tmpvox_lores(uint16_t* restrict out, uint16_t* restrict in, double ang_rad)
{
	static uint16_t tmp_skewed[TMPVOX_XS*TMPVOX_YS/4];

	memset(tmp_skewed, 0, sizeof tmp_skewed);

	if(ang_rad > DEGTORAD(15) || ang_rad < DEGTORAD(-15))
	{
		printf("Error: out-of-range ang_rad in rotate_tmpvox\n");
		return;
	}
	double step = tan(ang_rad);

	// Shift everything X direction first, creating a skewed copy

	for(int yy = 0; yy < TMPVOX_YS/2; yy++)
	{
		double fshift = ((double)((TMPVOX_YMID/2) - yy)*step); // Bottom line (yy=0) shifts right with positive ang.
		int shift = (fshift>=0.0)?(fshift+0.5):(fshift-0.5);

		if(shift >= 0)
		{
			for(int xx=0; xx<(TMPVOX_XS/2)-shift; xx++)
			{
				tmp_skewed[yy*(TMPVOX_XS/2)+xx+shift] = in[yy*(TMPVOX_XS/2)+xx];
			}
		}
		else
		{
			for(int xx=-1*shift; xx<TMPVOX_XS/2; xx++)
			{
				tmp_skewed[yy*(TMPVOX_XS/2)+xx+shift] = in[yy*(TMPVOX_XS/2)+xx];
			}
		}
	}

	// Shift in Y direction

	for(int xx = 0; xx < (TMPVOX_XS/2); xx++)
	{
		double fshift = ((double)(xx - (TMPVOX_XMID/2))*step); // Left column (xx=0) shifts down with positive ang.
		int shift = (fshift>=0.0)?(fshift+0.5):(fshift-0.5);

		if(shift >= 0)
		{
			for(int yy=0; yy<(TMPVOX_YS/2)-shift; yy++)
			{
				out[(yy+shift)*(TMPVOX_XS/2)+xx] = tmp_skewed[yy*(TMPVOX_XS/2)+xx];
			}
		}
		else
		{
			for(int yy=-1*shift; yy<(TMPVOX_YS/2); yy++)
			{
				out[(yy+shift)*(TMPVOX_XS/2)+xx] = tmp_skewed[yy*(TMPVOX_XS/2)+xx];
			}
		}

	}
}


void clear_voxel_map()
{
	memset(tmpvox, 0, sizeof tmpvox);
}

// Modifies val_ as a side effect.
#define CALC_ONES(val_, bits_, result_) do{ result_=0; for(int i_=0; i_<(bits_); i_++) { if(val_&1){result_++;}   val_>>=1;  }   } while(0)
#define CALC_ZEROES(val_, bits_, result_) do{ result_=0; for(int i_=0; i_<(bits_); i_++) { if(!(val_&1)){result_++;}   val_>>=1;  }   } while(0)


#define CMP_MASK 0b1111111111110000



void insert_mcu_voxel_map(uint16_t* restrict extvox, mcu_multi_voxel_map_t* restrict mcuvox)
{
	for(int i = 0; i<3; i++)
	{
		int b = mcuvox->first_block_id+i;

		if(b<0 || b>12)
		{
			printf("ERROR: Invalid block id %d\n", b);
			return;
		}
		// Non-scaled copy of blocks 0,1,2,3
		if(b<4)
		{
			for(int yy=0; yy<VOX_SEG_YS; yy++)
			{
				for(int xx=0; xx<VOX_SEG_XS; xx++)
				{
					int ex = seg_lims[b].xmin/MAP_UNIT_W + TMPVOX_XMID + xx;
					int ey = seg_lims[b].ymin/MAP_UNIT_W + TMPVOX_YMID + yy;
					uint16_t val = mcuvox->maps[i][yy*VOX_SEG_XS+xx];
					extvox[(ey)*TMPVOX_XS + (ex)] = val;
				}
			}
		}
		else
		{
			for(int yy=0; yy<VOX_SEG_YS; yy++)
			{
				for(int xx=0; xx<VOX_SEG_XS; xx++)
				{
					int ex = seg_lims[b].xmin/MAP_UNIT_W + TMPVOX_XMID + xx*2;
					int ey = seg_lims[b].ymin/MAP_UNIT_W + TMPVOX_YMID + yy*2;
					uint16_t val = mcuvox->maps[i][yy*VOX_SEG_XS+xx];
					extvox[(ey+0)*TMPVOX_XS + (ex+0)] = val;
					extvox[(ey+0)*TMPVOX_XS + (ex+1)] = val;
					extvox[(ey+1)*TMPVOX_XS + (ex+0)] = val;
					extvox[(ey+1)*TMPVOX_XS + (ex+1)] = val;
				}
			}

		}
	}	
}

void insert_mcu_voxel_map_lores(uint16_t* restrict extvox, mcu_multi_voxel_map_t* restrict mcuvox)
{
	for(int i = 0; i<3; i++)
	{
		int b = mcuvox->first_block_id+i;

		if(b<0 || b>12)
		{
			printf("ERROR: Invalid block id %d\n", b);
			return;
		}
		// Hires block is downscaled by 2 (each big site is union of four small sites)
		if(b<4)
		{
			for(int yy=0; yy<VOX_SEG_YS/2; yy++)
			{
				for(int xx=0; xx<VOX_SEG_XS/2; xx++)
				{
					int ex = (seg_lims[b].xmin/MAP_UNIT_W + TMPVOX_XMID)/2 + xx;
					int ey = (seg_lims[b].ymin/MAP_UNIT_W + TMPVOX_YMID)/2 + yy;
					uint16_t val0 = mcuvox->maps[i][(yy*2+0)*VOX_SEG_XS+(xx*2+0)];
					uint16_t val1 = mcuvox->maps[i][(yy*2+0)*VOX_SEG_XS+(xx*2+1)];
					uint16_t val2 = mcuvox->maps[i][(yy*2+1)*VOX_SEG_XS+(xx*2+0)];
					uint16_t val3 = mcuvox->maps[i][(yy*2+1)*VOX_SEG_XS+(xx*2+1)];
					extvox[(ey)*(TMPVOX_XS/2) + (ex)] = val0 | val1 | val2 | val3;
				}
			}
		}
		else
		{	// Lores block goes in directly
			for(int yy=0; yy<VOX_SEG_YS; yy++)
			{
				for(int xx=0; xx<VOX_SEG_XS; xx++)
				{
					int ex = (seg_lims[b].xmin/MAP_UNIT_W + TMPVOX_XMID)/2 + xx;
					int ey = (seg_lims[b].ymin/MAP_UNIT_W + TMPVOX_YMID)/2 + yy;
					uint16_t val = mcuvox->maps[i][yy*VOX_SEG_XS+xx];
					extvox[(ey)*(TMPVOX_XS/2) + (ex)] = val;
				}
			}

		}
	}	
}


typedef struct
{
	int8_t x_units;
	int8_t y_units;
	int32_t score;
} speculation_t;


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

static uint16_t refmap_hires[TMPVOX_XS*TMPVOX_YS];
static uint16_t refmap_lores[(TMPVOX_XS/2)*(TMPVOX_YS/2)];

void gen_refmap_hires(world_t* w, int32_t ref_x, int32_t ref_y)
{
	int ref_px = PAGE(ref_x);
	int ref_ox = OFFS(ref_x);
	int ref_py = PAGE(ref_y);
	int ref_oy = OFFS(ref_y);	

	for(int yy = 0; yy < TMPVOX_YS; yy++)
	{
		for(int xx = 0; xx < TMPVOX_XS; xx++)
		{
			int map_px = ref_px;
			int map_ox = ref_ox;
			int map_py = ref_py;
			int map_oy = ref_oy;
			shift_page_offs(&map_px, &map_ox, xx-TMPVOX_XMID);			
			shift_page_offs(&map_py, &map_oy, yy-TMPVOX_YMID);	
		
			uint32_t map_says = w->pages[map_px][map_py]->voxmap[MAPIDX(map_ox, map_oy)];

			uint32_t map_blurred = (map_says<<1) | (map_says) | (map_says>>1);

			uint16_t map_cmp = map_blurred&CMP_MASK;

			refmap_hires[yy*TMPVOX_XS+xx] = map_cmp;
		}
	}
}

void gen_refmap_lores(world_t* w, int32_t ref_x, int32_t ref_y)
{
	int ref_px = PAGE(ref_x);
	int ref_ox = OFFS(ref_x);
	int ref_py = PAGE(ref_y);
	int ref_oy = OFFS(ref_y);	

	for(int yy = 0; yy < TMPVOX_YS/2; yy++)
	{
		for(int xx = 0; xx < TMPVOX_XS/2; xx++)
		{
			uint16_t map_cmp = 0;
			for(int suby = 0; suby < 2; suby++)
			{
				for(int subx = 0; subx < 2; subx++)
				{
					int map_px = ref_px;
					int map_ox = ref_ox;
					int map_py = ref_py;
					int map_oy = ref_oy;
					shift_page_offs(&map_px, &map_ox, xx*2-TMPVOX_XMID+subx);			
					shift_page_offs(&map_py, &map_oy, yy*2-TMPVOX_YMID+suby);	
				
					uint32_t map_says = w->pages[map_px][map_py]->voxmap[MAPIDX(map_ox, map_oy)];

					uint32_t map_blurred = (map_says<<1) | (map_says) | (map_says>>1);

					map_cmp |= map_blurred;
				}
			}
			map_cmp &= CMP_MASK;

			refmap_lores[yy*(TMPVOX_XS/2)+xx] = map_cmp;

		}
	}
}

#define PERFORMANCE_CROP_LORES 32
#define PERFORMANCE_CROP 32

// Extra weight given to the XY zero-correction (dx=0, dy=0) case:
// Setting 8 means zero gets a multiplier of 8, other cases 8-1=7
#define EXTRA_WEIGHT_FOR_ZERO 14
#define EXTRA_WEIGHT_FOR_ZERO_LORES 10


static int score_voxmap_vs_refmap_hires(uint16_t* restrict ref, uint16_t* restrict vox,
	int n_dx, int dx_start, int dx_step, int n_dy, int dy_start, int dy_step, int* best_dx, int* best_dy, int ena_weigh)
{
	int min_x = dx_start;
	int max_x = dx_start + dx_step*(n_dx-1);
	int min_y = dy_start;
	int max_y = dy_start + dy_step*(n_dy-1);

	if(n_dx < 1 || n_dx > 32 || n_dy < 1 || n_dy > 32)
	{
		printf("ERROR: illegal n_dx=%d or n_dy=%d \n", n_dx, n_dy);
		return -1;
	}

	int32_t scores[32][32] = {{0}};


	int x_start = -1*min_x; if(x_start < PERFORMANCE_CROP) x_start = PERFORMANCE_CROP;
	int x_end = TMPVOX_XS-1-max_x; if(x_end > TMPVOX_XS-1-PERFORMANCE_CROP) x_end = TMPVOX_XS-1-PERFORMANCE_CROP;

	int y_start = -1*min_y; if(y_start < PERFORMANCE_CROP) y_start = PERFORMANCE_CROP;
	int y_end = TMPVOX_YS-1-max_y; if(y_end > TMPVOX_YS-1-PERFORMANCE_CROP) y_end = TMPVOX_YS-1-PERFORMANCE_CROP;


	for(int ix = 0; ix < n_dx; ix++)
	{
		for(int iy = 0; iy < n_dy; iy++)
		{
			for(int yy = y_start; yy <= y_end; yy+=1)
			{
				for(int xx = x_start; xx <= x_end; xx+=1)
				{
					uint16_t map_cmp = ref[yy*TMPVOX_XS+xx];

					int voxx = xx + dx_start + dx_step*ix;
					int voxy = yy + dy_start + dy_step*iy;
					uint16_t voxval = vox[voxy*TMPVOX_XS+voxx];

					voxval &= CMP_MASK;

					uint16_t compared = voxval & map_cmp;
					int numbits;
					CALC_ONES(compared, 16, numbits);

					scores[ix][iy] += numbits;
				}
			}
		}
	}

//	printf("scores: ");
	int best_score = -99999999, best_ix = 0, best_iy = 0;
	int weigh_mid_idx = n_dx/2;
	int weigh_mid_idy = n_dy/2;
	for(int ix = 0; ix < n_dx; ix++)
	{
		for(int iy = 0; iy < n_dy; iy++)
		{
			int sco;
			if(ena_weigh)
			{
				int weigh_dx = 2*n_dx - abs(weigh_mid_idx - ix);
				int weigh_dy = 2*n_dy - abs(weigh_mid_idy - iy);
				int extra_weight = (weigh_mid_idx==ix && weigh_mid_idy==iy)?EXTRA_WEIGHT_FOR_ZERO:(EXTRA_WEIGHT_FOR_ZERO-1);
				int extra_offset = (weigh_mid_idx==ix && weigh_mid_idy==iy)?1:0;
				sco = scores[ix][iy]*weigh_dx*weigh_dy*extra_weight+extra_offset;
			}
			else if(weigh_mid_idx==ix && weigh_mid_idy==iy)
			{	// Even without weighing, give a very tiny (+1) weight for the no-corr case,
				// so it maps properly to empty map
				sco = 2*n_dx*2*n_dy*EXTRA_WEIGHT_FOR_ZERO*scores[ix][iy]+1;
			}
			else
			{
				sco = 2*n_dx*2*n_dy*EXTRA_WEIGHT_FOR_ZERO*scores[ix][iy];
			}

//			printf(" (%2d,%2d): %6d ", ix, iy, score[ix][iy]);
			if(sco > best_score)
			{
				best_score = sco;
				best_ix = ix;
				best_iy = iy;
			}
		}
	}

//	printf("\n");
	*best_dx = dx_start+dx_step*best_ix;
	*best_dy = dy_start+dy_step*best_iy;

	return best_score;
}


int score_voxmap_vs_refmap_lores(uint16_t* restrict ref, uint16_t* restrict vox,
	int n_dx, int dx_start, int dx_step, int n_dy, int dy_start, int dy_step, int* best_dx, int* best_dy, int ena_weigh)
{
	int min_x = dx_start;
	int max_x = dx_start + dx_step*(n_dx-1);
	int min_y = dy_start;
	int max_y = dy_start + dy_step*(n_dy-1);

	if(n_dx < 1 || n_dx > 32 || n_dy < 1 || n_dy > 32)
	{
		printf("ERROR: illegal n_dx=%d or n_dy=%d \n", n_dx, n_dy);
		return -1;
	}

	int32_t scores[32][32] = {{0}};

//	printf("INFO: score_voxmap, n_speculations=%d, min_x=%d, max_x=%d, min_y=%d, max_y=%d\n", n_speculations, min_x, max_x, min_y, max_y);

	int x_start = -1*min_x; if(x_start < PERFORMANCE_CROP_LORES) x_start = PERFORMANCE_CROP_LORES;
	int x_end = (TMPVOX_XS/2)-1-max_x; if(x_end > (TMPVOX_XS/2)-1-PERFORMANCE_CROP_LORES) x_end = (TMPVOX_XS/2)-1-PERFORMANCE_CROP_LORES;

	int y_start = -1*min_y; if(y_start < PERFORMANCE_CROP_LORES) y_start = PERFORMANCE_CROP_LORES;
	int y_end = (TMPVOX_YS/2)-1-max_y; if(y_end > (TMPVOX_YS/2)-1-PERFORMANCE_CROP_LORES) y_end = (TMPVOX_YS/2)-1-PERFORMANCE_CROP_LORES;


	for(int ix = 0; ix < n_dx; ix++)
	{
		for(int iy = 0; iy < n_dy; iy++)
		{
			for(int yy = y_start; yy <= y_end; yy+=1)
			{
				for(int xx = x_start; xx <= x_end; xx+=1)
				{
					uint16_t map_cmp = ref[yy*(TMPVOX_XS/2)+xx];

					int voxx = xx + dx_start + dx_step*ix;
					int voxy = yy + dy_start + dy_step*iy;
					uint16_t voxval = vox[voxy*(TMPVOX_XS/2)+voxx];

					voxval &= CMP_MASK;

					uint16_t compared = voxval & map_cmp;
					int numbits;
					CALC_ONES(compared, 16, numbits);

					scores[ix][iy] += numbits;
				}
			}
		}
	}

//	printf("scores: ");
	int best_score = -99999999, best_ix = 0, best_iy = 0;
	int weigh_mid_idx = n_dx/2;
	int weigh_mid_idy = n_dy/2;

	for(int ix = 0; ix < n_dx; ix++)
	{
		for(int iy = 0; iy < n_dy; iy++)
		{
			int sco;
			if(ena_weigh)
			{
				int weigh_dx = 2*n_dx - abs(weigh_mid_idx - ix);
				int weigh_dy = 2*n_dy - abs(weigh_mid_idy - iy);
				int extra_weight = (weigh_mid_idx==ix && weigh_mid_idy==iy)?EXTRA_WEIGHT_FOR_ZERO_LORES:(EXTRA_WEIGHT_FOR_ZERO_LORES-1);
				int extra_offset = (weigh_mid_idx==ix && weigh_mid_idy==iy)?1:0;
				sco = scores[ix][iy]*weigh_dx*weigh_dy*extra_weight+extra_offset;
			}
			else if(weigh_mid_idx==ix && weigh_mid_idy==iy)
			{	// Even without weighing, give a very tiny (+1) weight for the no-corr case,
				// so it maps properly to empty map
				sco = 2*n_dx*2*n_dy*EXTRA_WEIGHT_FOR_ZERO*scores[ix][iy]+1;
			}
			else
			{
				sco = 2*n_dx*2*n_dy*EXTRA_WEIGHT_FOR_ZERO*scores[ix][iy];
			}

//			printf(" (%2d,%2d): %6d ", ix, iy, score[ix][iy]);
			if(sco > best_score)
			{
				best_score = sco;
				best_ix = ix;
				best_iy = iy;
			}
		}
	}

//	printf("\n");
	*best_dx = dx_start+dx_step*best_ix;
	*best_dy = dy_start+dy_step*best_iy;

	return best_score;
}


int score_voxmap(world_t* w, uint16_t* restrict vox, int32_t ref_x, int32_t ref_y,
	int n_speculations, speculation_t* restrict speculations)
{
	int8_t min_x = 127;
	int8_t max_x = -128;
	int8_t min_y = 127;
	int8_t max_y = -128;

	for(int s=0; s<n_speculations; s++)
	{
		speculations[s].score = 0;
		SAVE_MAX(speculations[s].x_units, max_x);
		SAVE_MIN(speculations[s].x_units, min_x);
		SAVE_MAX(speculations[s].y_units, max_y);
		SAVE_MIN(speculations[s].y_units, min_y);
	}

	if(n_speculations == 0 || n_speculations > 64)
	{
		printf("ERROR: illegal n_speculations = %d\n", n_speculations);
		return -1;
	}

	int32_t scores[32] = {0};

//	printf("INFO: score_voxmap, n_speculations=%d, min_x=%d, max_x=%d, min_y=%d, max_y=%d\n", n_speculations, min_x, max_x, min_y, max_y);

	int x_start = -1*min_x; if(x_start < PERFORMANCE_CROP) x_start = PERFORMANCE_CROP;
	int x_end = TMPVOX_XS-1-max_x; if(x_end > TMPVOX_XS-1-PERFORMANCE_CROP) x_end = TMPVOX_XS-1-PERFORMANCE_CROP;

	int y_start = -1*min_y; if(y_start < PERFORMANCE_CROP) y_start = PERFORMANCE_CROP;
	int y_end = TMPVOX_YS-1-max_y; if(y_end > TMPVOX_YS-1-PERFORMANCE_CROP) y_end = TMPVOX_YS-1-PERFORMANCE_CROP;


/*	int ref_x_unit = ref_x/MAP_UNIT_W + MAP_MIDDLE_UNIT;
	int ref_px = ref_x_unit / MAP_PAGE_W;
	int ref_ox = ref_x_unit - ref_px*MAP_PAGE_W;

	int ref_y_unit = ref_y/MAP_UNIT_W + MAP_MIDDLE_UNIT;
	int ref_py = ref_y_unit / MAP_PAGE_W;
	int ref_oy = ref_y_unit - ref_yx*MAP_PAGE_W;
*/

	int ref_px = PAGE(ref_x);
	int ref_ox = OFFS(ref_x);
	int ref_py = PAGE(ref_y);
	int ref_oy = OFFS(ref_y);	

	for(int yy = y_start; yy <= y_end; yy++)
	{
		for(int xx = x_start; xx <= x_end; xx++)
		{
			// The map is looked at the same site for each speculation, since it's slower
			// to jump between page boundaries, whereas the voxmap is contiguous.
			int map_px = ref_px;
			int map_ox = ref_ox;
			int map_py = ref_py;
			int map_oy = ref_oy;
			shift_page_offs(&map_px, &map_ox, xx-TMPVOX_XMID);			
			shift_page_offs(&map_py, &map_oy, yy-TMPVOX_YMID);	
		
			uint32_t map_says = w->pages[map_px][map_py]->voxmap[MAPIDX(map_ox, map_oy)];

//			uint16_t map_cmp_lo = ((map_says&CMP_MASK)>>1);
			uint16_t map_cmp_neutral = map_says&CMP_MASK;
//			uint16_t map_cmp_hi = ((map_says&CMP_MASK)<<1);

//			printf("map: x=%d,%d, y=%d,%d, says %x\n", map_px, map_ox, map_py, map_oy, (uint32_t)map_says);

			for(int s=0; s<n_speculations; s++)
			{
				int voxx = xx + (int)speculations[s].x_units;
				int voxy = yy + (int)speculations[s].y_units;
				uint16_t voxval = vox[voxy*TMPVOX_XS+voxx];

				voxval &= CMP_MASK;

//				uint16_t lo = voxval & map_cmp_lo;
				uint16_t neut = voxval & map_cmp_neutral;
//				uint16_t hi = voxval & map_cmp_hi;
				
//				int numbits_lo, numbits_neut, numbits_hi;
				int numbits_neut;
//				CALC_ONES(lo, 16, numbits_lo);
				CALC_ONES(neut, 16, numbits_neut);
//				CALC_ONES(hi, 16, numbits_hi);

				int best = numbits_neut;
//				if(numbits_lo > best) best = numbits_lo;
//				if(numbits_hi > best) best = numbits_hi;

				scores[s] += best;
//				printf("specul %d: voxx=%d, voxy=%d, val %x,  num_zeroes: lo %d, neut %d, hi %d, best %d\n", s, voxx, voxy, voxval, numbits_lo, numbits_neut, numbits_hi, best);

			}
		}
	}

	for(int i=0; i<n_speculations; i++)
		speculations[i].score = scores[i];


	return 1;

}

#if 1
int map_voxmap(world_t* w, uint16_t* restrict vox, int32_t ref_x, int32_t ref_y, int da, int dx_blocks, int dy_blocks, double *ang_corr_out)
{
	int ref_px = PAGE(ref_x);
	int ref_ox = OFFS(ref_x);
	int ref_py = PAGE(ref_y);
	int ref_oy = OFFS(ref_y);

	static map_page_t copies[5][5];
	int copy_start_pagex = ref_px-2;
	int copy_start_pagey = ref_py-2;	

	for(int i = 0; i<5; i++)
	{
		for(int o=0; o<5; o++)
		{
			memcpy(&copies[i][o], w->pages[copy_start_pagex+i][copy_start_pagey+o], sizeof(map_page_t));
		}
	}

//	int dx_blocks = dx/MAP_UNIT_W;
//	int dy_blocks = dy/MAP_UNIT_W;

//	int cnt = 0;

	double avg_ang_corr_accum = 0.0;
	int avg_ang_corr_n = 0;

	for(int yy = 0; yy < TMPVOX_YS; yy++)
	{
		for(int xx = 0; xx < TMPVOX_XS; xx++)
		{
			int map_px = ref_px;
			int map_ox = ref_ox;
			int map_py = ref_py;
			int map_oy = ref_oy;
			shift_page_offs(&map_px, &map_ox, xx-TMPVOX_XMID+dx_blocks);			
			shift_page_offs(&map_py, &map_oy, yy-TMPVOX_YMID+dy_blocks);

			int copy_px = map_px-copy_start_pagex;
			int copy_py = map_py-copy_start_pagey;

			uint32_t map_says = copies[copy_px][copy_py].voxmap[MAPIDX(map_ox, map_oy)];
			uint16_t map_cmp_neutral = map_says&CMP_MASK;

			if(vox[yy*TMPVOX_XS+xx] != 0)
			{
				w->pages[map_px][map_py]->voxmap[MAPIDX(map_ox, map_oy)] = vox[yy*TMPVOX_XS+xx];

				w->changed[map_px][map_py] = 1;

			}
		}
	}

	*ang_corr_out = 0.0;

	return 0;
}

#endif


#if 0
int map_voxmap(world_t* w, uint16_t* restrict vox, int32_t ref_x, int32_t ref_y, int da, int dx_blocks, int dy_blocks, double *ang_corr_out)
{
	int ref_px = PAGE(ref_x);
	int ref_ox = OFFS(ref_x);
	int ref_py = PAGE(ref_y);
	int ref_oy = OFFS(ref_y);

	static map_page_t copies[5][5];
	int copy_start_pagex = ref_px-2;
	int copy_start_pagey = ref_py-2;	

	for(int i = 0; i<5; i++)
	{
		for(int o=0; o<5; o++)
		{
			memcpy(&copies[i][o], w->pages[copy_start_pagex+i][copy_start_pagey+o], sizeof(map_page_t));
		}
	}

//	int dx_blocks = dx/MAP_UNIT_W;
//	int dy_blocks = dy/MAP_UNIT_W;

//	int cnt = 0;

	double avg_ang_corr_accum = 0.0;
	int avg_ang_corr_n = 0;

	int yy = 2, ystate = 0;
//	for(int yy = 2; yy < TMPVOX_YS-2; yy++)
	while(1)
	{
//		for(int xx = 2; xx < TMPVOX_XS-2; xx++)
		int xx = 2, xstate = 0;
		while(1)
		{
			int map_px = ref_px;
			int map_ox = ref_ox;
			int map_py = ref_py;
			int map_oy = ref_oy;
			shift_page_offs(&map_px, &map_ox, xx-TMPVOX_XMID+dx_blocks);			
			shift_page_offs(&map_py, &map_oy, yy-TMPVOX_YMID+dy_blocks);

			int copy_px = map_px-copy_start_pagex;
			int copy_py = map_py-copy_start_pagey;

//			if((xx == TMPVOX_XMID && yy == TMPVOX_YMID) || (xx == TMPVOX_XMID+10 && yy == TMPVOX_YMID+10))
//			{
//				printf("tmpvox(%d, %d) goes to map x: %d,%d, y:%d,%d\n", xx, yy, map_px, map_ox, map_py, map_oy);
//			}

		//	if(vox[yy*TMPVOX_XS+xx] != 0) cnt++;


			uint32_t map_says = copies[copy_px][copy_py].voxmap[MAPIDX(map_ox, map_oy)];
			uint16_t map_cmp_neutral = map_says&CMP_MASK;

			static const int extra_score[5][5] =
			{
				{0,1,1,1,0},
				{1,2,3,2,1},
				{1,3,5,3,1},
				{1,2,3,2,1},
				{0,1,1,1,0},
			};

/*
			static const int extra_score[5][5] =
			{
				{0,0,0,0,0},
				{0,0,0,0,0},
				{0,0,0,0,0},
				{0,0,0,0,0},
				{0,0,0,0,0},
			};
*/
			int best_at_dx = 0, best_at_dy = 0;
			int best = -9999;
			for(int dy = -2; dy <= 2; dy++)
			{
				for(int dx = -2; dx <= 2; dx++)
				{
					int voxx = xx+dx;
					int voxy = yy+dy;
					uint16_t voxval = vox[voxy*TMPVOX_XS+voxx];
					voxval &= CMP_MASK;

					uint16_t neut = voxval & map_cmp_neutral;
					int score;
					CALC_ONES(neut, 16, score);
					score *= 5;
					// Give a bit advantage to the zero corr case, to avoid shift on empty map
					score += extra_score[dy+2][dx+2];

					if(score > best)
					{
						best = score;
						best_at_dy = dy;
						best_at_dx = dx;
					}
				}
			}
			
			int use_voxyy = yy + best_at_dy;
			int use_voxxx = xx + best_at_dx;

//			if(xx > TMPVOX_XMID+75 && xx < TMPVOX_XMID+85 && yy > TMPVOX_YMID+45 && yy < TMPVOX_YMID+55)
//			{
//				printf("xx=%d yy=%d  best_score=%d at dx=%d dy=%d, use_voxyy=%d, use_voxxx=%d\n", xx, yy, best, best_at_dx, best_at_dy, use_voxyy, use_voxxx);
//			}

			if(vox[use_voxyy*TMPVOX_XS+use_voxxx] != 0)
			{
				w->pages[map_px][map_py]->voxmap[MAPIDX(map_ox, map_oy)] = vox[use_voxyy*TMPVOX_XS+use_voxxx];
				vox[use_voxyy*TMPVOX_XS+use_voxxx] = 0;

				w->changed[map_px][map_py] = 1;

				#if 0
					int uncorrected_y_units = yy - TMPVOX_YMID;
					int uncorrected_x_units = xx - TMPVOX_XMID;

					if(uncorrected_y_units < -10 || uncorrected_y_units > 10 || uncorrected_x_units < -10 || uncorrected_x_units > 10)
					{
						int bestfit_y_units = use_voxyy - TMPVOX_YMID;
						int bestfit_x_units = use_voxxx - TMPVOX_XMID;

						double supposed_angle = atan2(uncorrected_y_units, uncorrected_x_units);
						double bestfit_angle  = atan2(bestfit_y_units, bestfit_x_units);

						double ang_corr = supposed_angle - bestfit_angle;
						if(ang_corr < -M_PI/2.0) ang_corr += 2*M_PI;

						if(ang_corr > DEGTORAD(-15) && ang_corr < DEGTORAD(15))
						{
							avg_ang_corr_accum += ang_corr;
							avg_ang_corr_n++;
						}
					}
				#endif
			}

			xx += 3;
			if(xx >= TMPVOX_XS-2)
			{
				if(xstate == 0)
				{
					xx=2+1;
					xstate = 1;
				}
				else if(xstate == 1)
				{
					xx=2+2;
					xstate = 2;
				}
				else
					break;
			}


		}

		yy += 3;
		if(yy >= TMPVOX_YS-2)
		{
			if(ystate == 0)
			{
				yy=2+1;
				ystate = 1;
			}
			else if(ystate == 1)
			{
				yy=2+2;
				ystate = 2;
			}
			else
				break;
		}
	}

	double avg_ang_corr = 0.0;
	if(avg_ang_corr_n > 0)
	{
		avg_ang_corr = avg_ang_corr_accum / avg_ang_corr_n;

		if(avg_ang_corr_n > 100)
			*ang_corr_out = avg_ang_corr;
		else if(avg_ang_corr_n > 60)
			*ang_corr_out = avg_ang_corr/1.5;
		else if(avg_ang_corr_n > 30)
			*ang_corr_out = avg_ang_corr/2.5;
		else
			*ang_corr_out = 0.0;
	}
	else
	{
		*ang_corr_out = 0.0;
	}

	printf("Voxmap insertion to map done - average yaw correction during insertion %.5f deg (%d samples) -> %.5f\n", RADTODEG(avg_ang_corr), avg_ang_corr_n, RADTODEG(*ang_corr_out));

	//printf("MAP_VOXMAP DONE YEEEEES non-zeroes %d\n", cnt);

	return 0;

}

#endif



/*
	ref_x, ref_y: actual millimeter coordinates of the TMPVOX_XMID, TMPVOX_YMID. This isn't necessarily the robot's
		      position within the voxmap, but originates from the microcontroller. Because the microcontroller
	              already joins images together while moving, the reference position is kept during such operations,
	              and updated every time there is a risk of the robot running too far from the reference, which would
	              limit the available area ahead.

	rota_mid_x, rota_mid_y: actual millimeter coordinates to whatever point the rotational correction is going to be
	            made around. This is always a compromise, since the images are combined already on the MCU, but as
	            a compromise, the average between ref_x,ref_y and the latest robot position should be fairly good.

*/

extern double subsec_timestamp();
#define MULTITHREADED_SLAM 

#ifdef MULTITHREADED_SLAM

typedef struct
{
	// Inputs:
	uint16_t* voxmap;
	uint16_t* refmap;
	int n_angles;
	double angles[16];
	int weight_offs[16];
	double weight_mults[16];
	int x_start;
	int x_step;
	int x_nsteps;
	int y_start;
	int y_step;
	int y_nsteps;

	// Outputs:
	int* x_winner;
	int* y_winner;
	double* ang_winner;
	int* winner_score;
} thread_args_t;


void* pass1_thread(void* args_in)
{
	uint16_t rotated_vox_lores[TMPVOX_XS*TMPVOX_YS/4];

	thread_args_t* args = args_in;

	double ang_winner = 0.0;
	int x_winner = 0;
	int y_winner = 0;
	int max_score = -9999999;
	for(int a=0; a<args->n_angles; a++)
	{
		double rota_ang = args->angles[a];

		int score;
		int best_dx = 0, best_dy = 0;
		if(rota_ang > DEGTORAD(-0.1) && rota_ang < DEGTORAD(0.1))
		{
			// No rotation
			score = score_voxmap_vs_refmap_lores(args->refmap, args->voxmap,
				args->x_nsteps, args->x_start, args->x_step, args->y_nsteps, args->y_start, args->y_step,
				&best_dx, &best_dy, 1);
		}
		else
		{
			memset(rotated_vox_lores, 0, sizeof(rotated_vox_lores));
			rotate_tmpvox_lores(rotated_vox_lores, args->voxmap, rota_ang);
			score = score_voxmap_vs_refmap_lores(refmap_lores, rotated_vox_lores,
				args->x_nsteps, args->x_start, args->x_step, args->y_nsteps, args->y_start, args->y_step,
				&best_dx, &best_dy, 1);
		}

		// convert to actual map units right away:
		best_dx *= 2;
		best_dy *= 2;

//		printf("Winner for angle %.2f deg is x=%d, y=%d, score=%d\n",  RADTODEG(rota_ang), best_dx, best_dy, score);

		int weighed_score = (int)(args->weight_mults[a]*(double)score) + args->weight_offs[a];

		if(weighed_score > max_score)
		{
			max_score = weighed_score;
			ang_winner = rota_ang;
			x_winner = best_dx;
			y_winner = best_dy;
		}
	}

	*(args->x_winner) = x_winner;
	*(args->y_winner) = y_winner;
	*(args->ang_winner) = ang_winner;
	*(args->winner_score) = max_score;

	return NULL;
}


void* pass2_thread(void* args_in)
{
	uint16_t rotated_vox[TMPVOX_XS*TMPVOX_YS];

	thread_args_t* args = args_in;

	double ang_winner = 0.0;
	int x_winner = 0;
	int y_winner = 0;
	int max_score = -9999999;
	for(int a=0; a<args->n_angles; a++)
	{
		double rota_ang = args->angles[a];

		int score;
		int best_dx = 0, best_dy = 0;
		if(rota_ang > DEGTORAD(-0.1) && rota_ang < DEGTORAD(0.1))
		{
			// No rotation
			score = score_voxmap_vs_refmap_hires(args->refmap, args->voxmap,
				args->x_nsteps, args->x_start, args->x_step, args->y_nsteps, args->y_start, args->y_step,
				&best_dx, &best_dy, 1);
		}
		else
		{
			memset(rotated_vox, 0, sizeof(rotated_vox));
			rotate_tmpvox(rotated_vox, args->voxmap, rota_ang);
			score = score_voxmap_vs_refmap_hires(args->refmap, rotated_vox,
				args->x_nsteps, args->x_start, args->x_step, args->y_nsteps, args->y_start, args->y_step,
				&best_dx, &best_dy, 1);
		}

//		printf("Winner for angle %.2f deg is x=%d, y=%d, score=%d\n",  RADTODEG(rota_ang), best_dx, best_dy, score);

		int weighed_score = (int)(args->weight_mults[a]*(double)score) + args->weight_offs[a];

		if(weighed_score > max_score)
		{
			max_score = weighed_score;
			ang_winner = rota_ang;
			x_winner = best_dx;
			y_winner = best_dy;
		}
	}

	*(args->x_winner) = x_winner;
	*(args->y_winner) = y_winner;
	*(args->ang_winner) = ang_winner;
	*(args->winner_score) = max_score;

	return NULL;
}

#define N_SLAM_THREADS 3

int slam_voxmap(world_t* w, uint16_t* vox, uint16_t* vox_lores, int32_t ref_x, int32_t ref_y, int32_t rota_mid_x, int32_t rota_mid_y, 
	int32_t *corr_a, int32_t *corr_x, int32_t *corr_y)
{
	int ret;

	double time;

	static int test_first = 1;

	static uint16_t rotated_vox[TMPVOX_XS*TMPVOX_YS];

//	printf("%d-thread localization & mapping on voxmap, ref_x=%d, ref_y=%d, rota_mid_x=%d, rota_mid_y=%d\n", N_SLAM_THREADS, ref_x, ref_y, rota_mid_x, rota_mid_y);

	time = subsec_timestamp();

	load_25pages(w, PAGE(ref_x), PAGE(ref_y));

	double time_pageload = subsec_timestamp() - time;

	time = subsec_timestamp();

	gen_refmap_lores(w, ref_x, ref_y);

	double time_gen_lores = subsec_timestamp() - time;

	time = subsec_timestamp();

	gen_refmap_hires(w, ref_x, ref_y);

	double time_gen_hires = subsec_timestamp() - time;


	volatile int x_winners[N_SLAM_THREADS];
	volatile int y_winners[N_SLAM_THREADS];
	volatile double ang_winners[N_SLAM_THREADS];
	volatile int winner_scores[N_SLAM_THREADS];


	thread_args_t pass1_args[N_SLAM_THREADS];
	thread_args_t pass2_args[N_SLAM_THREADS];
	for(int i=0; i<N_SLAM_THREADS; i++)
	{
		pass1_args[i].voxmap = vox_lores;
		pass1_args[i].refmap = refmap_lores;
		pass1_args[i].x_winner = &x_winners[i];
		pass1_args[i].y_winner = &y_winners[i];
		pass1_args[i].ang_winner = &ang_winners[i];
		pass1_args[i].winner_score = &winner_scores[i];

		// Pass1: 1 unit is 100mm
		pass1_args[i].x_start  = -4;   //    -400mm
		pass1_args[i].x_step   =  1;   //     100mm
		pass1_args[i].x_nsteps =  9;   // to +400mm

		pass1_args[i].y_start  = -4;   //    -400mm
		pass1_args[i].y_step   =  1;   //     100mm
		pass1_args[i].y_nsteps =  9;   // to +400mm
	}

	pass1_args[0].n_angles = 3;
	pass1_args[0].angles[0] = DEGTORAD(-6.0);
	pass1_args[0].weight_offs[0] = 0;
	pass1_args[0].weight_mults[0] = 0.80;
	pass1_args[0].angles[1] = DEGTORAD(-4.5);
	pass1_args[0].weight_offs[1] = 0;
	pass1_args[0].weight_mults[1] = 0.85;
	pass1_args[0].angles[2] = DEGTORAD(-3.0);
	pass1_args[0].weight_offs[2] = 0;
	pass1_args[0].weight_mults[2] = 0.90;

	pass1_args[1].n_angles = 3;
	pass1_args[1].angles[0] = DEGTORAD(-1.5);
	pass1_args[1].weight_offs[0] = 1;
	pass1_args[1].weight_mults[0] = 0.95;
	pass1_args[1].angles[1] = DEGTORAD(0.0);
	pass1_args[1].weight_offs[1] = 3;
	pass1_args[1].weight_mults[1] = 1.0;
	pass1_args[1].angles[2] = DEGTORAD(+1.5);
	pass1_args[1].weight_offs[2] = 1;
	pass1_args[1].weight_mults[2] = 0.95;

	pass1_args[2].n_angles = 3;
	pass1_args[2].angles[0] = DEGTORAD(+3.0);
	pass1_args[2].weight_offs[0] = 0;
	pass1_args[2].weight_mults[0] = 0.90;
	pass1_args[2].angles[1] = DEGTORAD(+4.5);
	pass1_args[2].weight_offs[1] = 0;
	pass1_args[2].weight_mults[1] = 0.85;
	pass1_args[2].angles[2] = DEGTORAD(+6.0);
	pass1_args[2].weight_offs[2] = 0;
	pass1_args[2].weight_mults[2] = 0.80;


//	printf("Pass 1\n");

	time = subsec_timestamp();

	pthread_t threads[N_SLAM_THREADS];
	pthread_t threads_pass2[N_SLAM_THREADS];

	for(int th=0; th<N_SLAM_THREADS; th++)
	{
		if( (ret = pthread_create(&threads[th], NULL, pass1_thread, (void*)&pass1_args[th])) )
		{
			printf("ERROR: pass1 processing thread #%d creation, ret = %d\n", th, ret);
			return -1;
		}
	}

	for(int th=N_SLAM_THREADS-1; th>=0; th--)
	{
		pthread_join(threads[th], NULL);
	}

	double time_pass1 = subsec_timestamp() - time;

	int pass1_best_score = -99999999;
	int pass1_x_winner = 0;
	int pass1_y_winner = 0;
	double pass1_ang_winner = 0;
	for(int th=0; th<N_SLAM_THREADS; th++)
	{
		//printf("Pass1 thread #%d winner ang=%.2f x=%d y=%d score=%d\n", th, RADTODEG(ang_winners[th]), x_winners[th], y_winners[th], winner_scores[th]);

		if(winner_scores[th] > pass1_best_score)
		{
			pass1_best_score = winner_scores[th];
			pass1_x_winner = x_winners[th];
			pass1_y_winner = y_winners[th];
			pass1_ang_winner = ang_winners[th];
		}
	}

//	printf("Winner for pass1 is %.2f deg, x=%d, y=%d, score=%d\n", RADTODEG(pass1_ang_winner), pass1_x_winner, pass1_y_winner, pass1_best_score);

	// Reuse x_winners, y_winners, ang_winners_, winner_scores variables for pass2

	for(int i=0; i<N_SLAM_THREADS; i++)
	{

		pass2_args[i].voxmap = vox;
		pass2_args[i].refmap = refmap_hires;
		pass2_args[i].x_winner = &x_winners[i];
		pass2_args[i].y_winner = &y_winners[i];
		pass2_args[i].ang_winner = &ang_winners[i];
		pass2_args[i].winner_score = &winner_scores[i];

		// Pass2: 1 unit is 50mm
		pass2_args[i].x_start  = -2 + pass1_x_winner;   //    -100mm
		pass2_args[i].x_step   =  1;   //      50mm
		pass2_args[i].x_nsteps =  5;   // to +100mm

		pass2_args[i].y_start  = -2 + pass1_y_winner;   //    -100mm
		pass2_args[i].y_step   =  1;   //      50mm
		pass2_args[i].y_nsteps =  5;   // to +100mm
	}

	pass2_args[0].n_angles = 2;
	pass2_args[0].angles[0] = pass1_ang_winner + DEGTORAD(-1.0);
	pass2_args[0].weight_offs[0] = 0;
	pass2_args[0].weight_mults[0] = 1.0;
	pass2_args[0].angles[1] = pass1_ang_winner + DEGTORAD(-0.5);
	pass2_args[0].weight_offs[1] = 0;
	pass2_args[0].weight_mults[1] = 1.0;

	pass2_args[1].n_angles = 2;
	pass2_args[1].angles[0] = pass1_ang_winner + DEGTORAD(0.0);
	pass2_args[1].weight_offs[0] = 1; // to map as zero to empty maps
	pass2_args[1].weight_mults[0] = 1.0;
	pass2_args[1].angles[1] = pass1_ang_winner + DEGTORAD(0.5);
	pass2_args[1].weight_offs[1] = 0;
	pass2_args[1].weight_mults[1] = 1.0;

	pass2_args[2].n_angles = 1;
	pass2_args[2].angles[0] = pass1_ang_winner + DEGTORAD(1.0);
	pass2_args[2].weight_offs[0] = 0;
	pass2_args[2].weight_mults[0] = 1.0;



	time = subsec_timestamp();



	for(int th=0; th<N_SLAM_THREADS; th++)
	{
		if( (ret = pthread_create(&threads_pass2[th], NULL, pass2_thread, (void*)&pass2_args[th])) )
		{
			printf("ERROR: pass2 processing thread #%d creation, ret = %d\n", th, ret);
			return -1;
		}
	}


	for(int th=N_SLAM_THREADS-1; th>=0; th--)
	{
		pthread_join(threads_pass2[th], NULL);
	}

	double time_pass2 = subsec_timestamp() - time;

	int pass2_best_score = -99999999;
	int pass2_x_winner = 0;
	int pass2_y_winner = 0;
	double pass2_ang_winner = 0;
	for(int th=0; th<N_SLAM_THREADS; th++)
	{
		//printf("Pass2 thread #%d winner ang=%.2f x=%d y=%d score=%d\n", th, RADTODEG(ang_winners[th]), x_winners[th], y_winners[th], winner_scores[th]);

		if(winner_scores[th] > pass2_best_score)
		{
			pass2_best_score = winner_scores[th];
			pass2_x_winner = x_winners[th];
			pass2_y_winner = y_winners[th];
			pass2_ang_winner = ang_winners[th];
		}
	}

//	printf("Winner for pass2 is %.2f deg, x=%d, y=%d, score=%d\n", RADTODEG(pass2_ang_winner), pass2_x_winner, pass2_y_winner, pass2_best_score);




	int x_corr = -1*pass2_x_winner;
	int y_corr = -1*pass2_y_winner;

/*
	if(test_first)
	{
		pass1_ang_winner = DEGTORAD(5.0);
		x_corr = 6;
		y_corr = 6;
	}
*/
	// Regenerating the best rotated voxmap is probably more efficient than storing it from when it's generated for matching.

	double insertion_ang_corr = 0.0;


	time = subsec_timestamp();


	if(state_vect.v.mapping_3d)
	{
		if(pass2_ang_winner > DEGTORAD(-0.1) && pass2_ang_winner < DEGTORAD(0.1))
		{
			// no rotation
			map_voxmap(w, vox, ref_x, ref_y, 0, x_corr, y_corr, &insertion_ang_corr);
		}
		else
		{
			memset(rotated_vox, 0, sizeof(rotated_vox));
			rotate_tmpvox(rotated_vox, vox, pass2_ang_winner);
			map_voxmap(w, rotated_vox, ref_x, ref_y, 0, x_corr, y_corr, &insertion_ang_corr);
		}
		printf("  ->>>>  Mapping with correction ang=%.2f deg, x=%d, y=%d units\n", RADTODEG(pass2_ang_winner), x_corr, y_corr);

	}
	else
	{
		printf(" Mapping turned off ----- not mapping.\n");
	}

	double time_insertion = subsec_timestamp() - time;

	double a_corr = pass2_ang_winner;

	*corr_a = RADTOANG32(a_corr);
	*corr_x = x_corr*MAP_UNIT_W;
	*corr_y = y_corr*MAP_UNIT_W;

//	printf("Performance: page load %.1fms  gen_lores %.1fms  gen_hires %.1fms  pass1 %.1fms  pass2 %.1fms  insertion %.1fms\n",
//		time_pageload*1000.0, time_gen_lores*1000.0, time_gen_hires*1000.0, time_pass1*1000.0, time_pass2*1000.0, time_insertion*1000.0);

	test_first = 0;

	return 0;
}

#else // implement single-threaded slam

int slam_voxmap(world_t* w, uint16_t* vox, uint16_t* vox_lores, int32_t ref_x, int32_t ref_y, int32_t rota_mid_x, int32_t rota_mid_y, 
	int32_t *corr_a, int32_t *corr_x, int32_t *corr_y)
{
	double time;

	static int test_first = 1;

	static uint16_t rotated_vox[TMPVOX_XS*TMPVOX_YS];
	static uint16_t rotated_vox_lores[TMPVOX_XS*TMPVOX_YS/4];

//	printf("Localization & mapping on voxmap, ref_x=%d, ref_y=%d, rota_mid_x=%d, rota_mid_y=%d\n", ref_x, ref_y, rota_mid_x, rota_mid_y);

	time = subsec_timestamp();

	load_25pages(w, PAGE(ref_x), PAGE(ref_y));

	double time_pageload = subsec_timestamp() - time;

	time = subsec_timestamp();

	gen_refmap_lores(w, ref_x, ref_y);

	double time_gen_lores = subsec_timestamp() - time;

	time = subsec_timestamp();

	gen_refmap_hires(w, ref_x, ref_y);

	double time_gen_hires = subsec_timestamp() - time;


#define NUM_ANGLES_PASS1 9
	static const double angles_pass1[NUM_ANGLES_PASS1] =
	{
		DEGTORAD(-6.0),
		DEGTORAD(-4.5),
		DEGTORAD(-3.0),
		DEGTORAD(-1.5),
		DEGTORAD(0.0),
		DEGTORAD(1.5),
		DEGTORAD(3.0),
		DEGTORAD(4.5),
		DEGTORAD(6.0)
	};

	static const int angles_additional_score_pass1[NUM_ANGLES_PASS1] =
	{
		0,
		0,
		0,
		1,
		3,
		1,
		0,
		0,
		0
	};

	// Pass1 units correspond to TWO actual units
	int xy_start_pass1 = -4;   // -400mm
	int xy_step_pass1 = 1;     // 100mm
	int xy_nsteps_pass1 = 9;  // to +400mm

#define NUM_ANGLES_PASS2 5
	static const double angles_pass2[NUM_ANGLES_PASS2] =
	{
		DEGTORAD(-1.0),
		DEGTORAD(-0.5),
		DEGTORAD(0.0),
		DEGTORAD(0.5),
		DEGTORAD(1.0)
	};

	static const int angles_additional_score_pass2[NUM_ANGLES_PASS2] =
	{
		0,
		0,
		1,
		0,
		0
	};

	// Pass2 units correspond to actual units directly
	int xy_start_pass2 = -2; // -100mm
	int xy_step_pass2 = 1;   // 50mm
	int xy_nsteps_pass2 = 5;  // to +100mm


	//printf("Pass 1\n");

	time = subsec_timestamp();

	double pass1_ang_winner = 0.0;
	int pass1_x_winner = 0;
	int pass1_y_winner = 0;
	int pass1_max_score = -9999999;
	for(int a=0; a<NUM_ANGLES_PASS1; a++)
	{
		double rota_ang = angles_pass1[a];

		int score;
		int best_dx = 0, best_dy = 0;
		if(rota_ang > DEGTORAD(-0.1) && rota_ang < DEGTORAD(0.1))
		{
			// No rotation
			score = score_voxmap_vs_refmap_lores(refmap_lores, vox_lores,
				xy_nsteps_pass1, xy_start_pass1, xy_step_pass1, xy_nsteps_pass1, xy_start_pass1, xy_step_pass1,
				&best_dx, &best_dy, 1);
		}
		else
		{
			memset(rotated_vox_lores, 0, sizeof(rotated_vox_lores));
			rotate_tmpvox_lores(rotated_vox_lores, vox_lores, rota_ang);
			score = score_voxmap_vs_refmap_lores(refmap_lores, rotated_vox_lores,
				xy_nsteps_pass1, xy_start_pass1, xy_step_pass1, xy_nsteps_pass1, xy_start_pass1, xy_step_pass1,
				&best_dx, &best_dy, 1);
		}

		// convert to actual map units right away:
		best_dx *= 2;
		best_dy *= 2;

		printf("Winner for angle %.2f deg is x=%d, y=%d, score=%d\n",  RADTODEG(rota_ang), best_dx, best_dy, score);

		int weighed_score = score + angles_additional_score_pass1[a];

		if(weighed_score > pass1_max_score)
		{
			pass1_max_score = weighed_score;
			pass1_ang_winner = rota_ang;
			pass1_x_winner = best_dx;
			pass1_y_winner = best_dy;
		}
	}

	double time_pass1 = subsec_timestamp() - time;


//	printf("Winner for pass1 is %.2f deg, x=%d, y=%d, score=%d\n", RADTODEG(pass1_ang_winner), pass1_x_winner, pass1_y_winner, pass1_max_score);

	int x_start_pass2 = xy_start_pass2 + pass1_x_winner;
	int y_start_pass2 = xy_start_pass2 + pass1_y_winner;

	time = subsec_timestamp();


	double pass2_ang_winner = 0.0;
	int pass2_x_winner = 0;
	int pass2_y_winner = 0;
	int pass2_max_score = -9999999;
	for(int a=0; a<NUM_ANGLES_PASS2; a++)
	{
		double rota_ang = pass1_ang_winner + angles_pass2[a];

		int score;
		int best_dx = 0, best_dy = 0;
		if(rota_ang > DEGTORAD(-0.1) && rota_ang < DEGTORAD(0.1))
		{
			// No rotation
			score = score_voxmap_vs_refmap_hires(refmap_hires, vox,
				xy_nsteps_pass2, x_start_pass2, xy_step_pass2, xy_nsteps_pass2, y_start_pass2, xy_step_pass2,
				&best_dx, &best_dy, 1);
		}
		else
		{
			memset(rotated_vox, 0, sizeof(rotated_vox));
			rotate_tmpvox(rotated_vox, vox, rota_ang);
			score = score_voxmap_vs_refmap_hires(refmap_hires, rotated_vox,
				xy_nsteps_pass2, x_start_pass2, xy_step_pass2, xy_nsteps_pass2, y_start_pass2, xy_step_pass2,
				&best_dx, &best_dy, 1);
		}

		printf("Winner for angle %.2f deg is x=%d, y=%d, score=%d\n",  RADTODEG(rota_ang), best_dx, best_dy, score);

		int weighed_score = score + angles_additional_score_pass2[a];

		if(weighed_score > pass2_max_score)
		{
			pass2_max_score = weighed_score;
			pass2_ang_winner = rota_ang;
			pass2_x_winner = best_dx;
			pass2_y_winner = best_dy;
		}
	}

	double time_pass2 = subsec_timestamp() - time;

	printf("Winner for pass2 is %.2f deg, x=%d, y=%d, score=%d\n", RADTODEG(pass2_ang_winner), pass2_x_winner, pass2_y_winner, pass2_max_score);





	int x_corr = -1*pass2_x_winner;
	int y_corr = -1*pass2_y_winner;

/*
	if(test_first)
	{
		pass1_ang_winner = DEGTORAD(5.0);
		x_corr = 6;
		y_corr = 6;
	}
*/
	// Regenerating the best rotated voxmap is probably more efficient than storing it from when it's generated for matching.

	printf("          ->>>>>>>>>>>>>>>    Mapping with correction ang=%.2f deg, x=%d units, y=%d units\n", RADTODEG(pass2_ang_winner), x_corr, y_corr);
	double insertion_ang_corr = 0.0;


	time = subsec_timestamp();


	if(state_vect.v.mapping_3d)
	{
		if(pass2_ang_winner > DEGTORAD(-0.1) && pass2_ang_winner < DEGTORAD(0.1))
		{
			// no rotation
			map_voxmap(w, vox, ref_x, ref_y, 0, x_corr, y_corr, &insertion_ang_corr);
		}
		else
		{
			memset(rotated_vox, 0, sizeof(rotated_vox));
			rotate_tmpvox(rotated_vox, vox, pass2_ang_winner);
			map_voxmap(w, rotated_vox, ref_x, ref_y, 0, x_corr, y_corr, &insertion_ang_corr);
		}
	}


	double time_insertion = subsec_timestamp() - time;

	double a_corr = pass2_ang_winner;

	*corr_a = RADTOANG32(a_corr);
	*corr_x = x_corr*MAP_UNIT_W;
	*corr_y = y_corr*MAP_UNIT_W;

	printf("Performance: page load %.1fms  gen_lores %.1fms  gen_hires %.1fms  pass1 %.1fms  pass2 %.1fms  insertion %.1fms\n",
		time_pageload*1000.0, time_gen_lores*1000.0, time_gen_hires*1000.0, time_pass1*1000.0, time_pass2*1000.0, time_insertion*1000.0);

	test_first = 0;

	return 0;
}

#endif // single-threaded slam implementation


void provide_mcu_voxmap(world_t* w, mcu_multi_voxel_map_t* mcuvox, int32_t* acorr, int32_t* xcorr, int32_t* ycorr)
{
	int b = mcuvox->first_block_id;
//	printf("provide_mcu_voxmap, got %d\n", b);

	insert_mcu_voxel_map(tmpvox, mcuvox);
	insert_mcu_voxel_map_lores(tmpvox_lores, mcuvox);

	if(b==9)
	{
		tmpvox_ref_x = mcuvox->ref_x;
		tmpvox_ref_y = mcuvox->ref_y;
//		printf("Got full mcu voxmap\n");
		
		int32_t corr_a = 0, corr_x = 0, corr_y = 0;
		slam_voxmap(w, tmpvox, tmpvox_lores, mcuvox->ref_x, mcuvox->ref_y, mcuvox->ref_x, mcuvox->ref_y, 
			&corr_a, &corr_x, &corr_y);

		*xcorr = corr_x;
		*ycorr = corr_y;
		*acorr = corr_a;

		clear_voxel_map();
	}

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
	"WAIT_ROUTE",
	"res",
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
	S_WAIT_ROUTE		= 12
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
			extern lidar_scan_t* lidars_to_map_at_routing_start[NUM_LATEST_LIDARS_FOR_ROUTING_START];
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


			map_lidars_when_searched = 0;

			
			if(ret == 1)
			{
				printf("Automapping: run_search() fails in the start due to close obstacles (nonroutable), daijuing for a while.\n");

				send_info(INFO_STATE_DAIJUING);
				daiju_mode(1);
				cur_autostate = S_DAIJUING;
				daijuing_timestamp = subsec_timestamp();				
			}
			else if(ret == 0 || ret == -999)
			{
				cur_autostate = S_WAIT_ROUTE;
			}
			else
			{
//				printf("Automapping: run_search() fails later than in the start: destination is unreachable. Generating a new direction.\n");
				add_cant_goto_place(desired_x, desired_y);
				cur_autostate = S_GEN_DESIRED_DIR;
			}
		} break;

		case S_WAIT_ROUTE: {
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
