/*
	PULUROBOT robotsoft

	Memory / disk management on top of the voxmap.h data type

	voxmap module handles single voxel maps with limited size.

	This module provides a simple way to handle large worlds, divided
	into map pages.

	This module fixes the map resolution and block size into a compile
	time constant values.

	One map page can contain several voxmaps at different resolutions,
	but they always have the same absolute dimensions (number of voxels
	vary).

	For a simplistic example:
	Voxel size = 100 mm
	Cube size = 100 voxels = 10 meters
	(500mm, 500mm, 500mm) has page address of 0,0,0, and offset address of 50, 50, 50
	(-1500mm, 500mm, 500mm) has page address of -2,0,0, and offset address of 50, 50, 50



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


#pragma once


/*
	Only a few dozen of pages can live in the memory at once.
	You can address this array directly with your page address to see:

		- if the page exists on disk
		  (exists: bit0(LSb) high = highest resolution exists, bit1 = next decimated resolution, bit2 = even lower, and so on...)

		- if they are loaded in the memory
		  (loaded: similar bit mappings)

		- indeces to access the pointer table, to access the data.
		  The index given gets you to the highest resolution pointer.
		  Index+1 gives the next lower quality decimated, and so on.

	We could store the pointers directly, but for a 1024*1024*16 page world with 8 decimation levels and 8-byte pointers,
	that would waste 1024MB (compare to 64MB here).
*/


// Highest resolution block in millimeters
#define MIN_UNIT 32
#define MAP_PAGE_XY_W_MM 8192
#define MAP_PAGE_Z_H_MM 4096


#if ((MAP_PAGE_XY_W_MM % (MIN_UNIT*32)) != 0)
#error MAP_PAGE_XY_W_MM must be evenly divisible by the highest decimation level block size
#endif

#if ((MAP_PAGE_Z_H_MM % (MIN_UNIT*32)) != 0)
#error MAP_PAGE_Z_H_MM must be evenly divisible by the highest decimation level block size
#endif


#define MAX_RESOLEVELS 6

#if (MAX_RESOLEVELS < 1 || MAX_RESOLEVELS > 8)
#error Invalid MAX_RESOLEVELS
#endif

static const int VOX_UNITS[MAX_RESOLEVELS] =
{MIN_UNIT, MIN_UNIT*2, MIN_UNIT*4, MIN_UNIT*8, MIN_UNIT*16, MIN_UNIT*32};

static const int VOX_XS[MAX_RESOLEVELS] =
{MIN_UNIT, MIN_UNIT*2, MIN_UNIT*4, MIN_UNIT*8, MIN_UNIT*16, MIN_UNIT*32};


#define PAGE_META_FLAG_CHANGED (1<<12)
#define PAGE_META_MEM_IDX_MASK (0xfff)

typedef struct __attribute__ ((packed))
{
//	uint8_t exists;
	uint8_t loaded;

	uint16_t flags_mem_idx; // 4 bits for flags, 12 bits for mem idx
	
} page_meta_t;


#define MAX_PAGES_X 512
#define MAX_PAGES_Y 512
#define MAX_PAGES_Z 8

// Macros to get between the 0-referenced page indeces and always-positive array indeces:
#define PIDX(x_) ((x_)+MAX_PAGES_X/2)
#define PIDY(y_) ((y_)+MAX_PAGES_Y/2)
#define PIDZ(z_) ((z_)+MAX_PAGES_Z/2)

#define PX(x_) ((x_)-MAX_PAGES_X/2)
#define PY(y_) ((y_)-MAX_PAGES_Y/2)
#define PZ(z_) ((z_)-MAX_PAGES_Z/2)

#define MAX_LOADED_PAGES (7*7*5+20)

#if (MAX_LOADED_PAGES>4096)
#error MAX_LOADED_PAGES too big, mem_idx wont fit into 12 bits
#endif

extern page_meta_t page_metas[MAX_PAGES_X][MAX_PAGES_Y][MAX_PAGES_Z];

typedef struct __attribute__ ((packed))
{
//	uint8_t dummy;
//	uint8_t loaded;

	uint64_t access_timestamp;
	voxmap_t* p_voxmap[8]; // [0] = highest resolevel

	// reference back to the meta
	uint16_t px_idx;
	uint16_t py_idx;
	uint16_t pz_idx;

} page_pointer_t;


extern page_pointer_t* page_pointers[MAX_LOADED_PAGES];


