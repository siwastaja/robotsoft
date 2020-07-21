/*
	PULUROBOT robotsoft

	Memory / disk management on top of the voxmap.h data type

	voxmap module handles single voxel maps with limited size.

	This module provides a simple way to handle large worlds, divided
	into map pages.

	This module fixes the map resolution and block size to compile time
	constant values.

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


#define MAX_RESOLEVELS 4  // The highest enabled resolevel index. By design, max is 8.
#define RESOLEVELS 0b1111 // Enabled resolevels: LSb = rl0


#if (MAX_RESOLEVELS < 1 || MAX_RESOLEVELS > 8)
#error Invalid MAX_RESOLEVELS
#endif

static const int VOX_RELATIONS[MAX_RESOLEVELS] =
{1, 
 2, 
 4, 
 8
};


static const int VOX_UNITS[MAX_RESOLEVELS] =
{MIN_UNIT, 
 MIN_UNIT*2, 
 MIN_UNIT*4, 
 MIN_UNIT*8
};

static const int VOX_XS[MAX_RESOLEVELS] =
{MAP_PAGE_XY_W_MM/MIN_UNIT,
 MAP_PAGE_XY_W_MM/(MIN_UNIT*2),
 MAP_PAGE_XY_W_MM/(MIN_UNIT*4),
 MAP_PAGE_XY_W_MM/(MIN_UNIT*8)
};

static const int VOX_YS[MAX_RESOLEVELS] =
{MAP_PAGE_XY_W_MM/MIN_UNIT,
 MAP_PAGE_XY_W_MM/(MIN_UNIT*2),
 MAP_PAGE_XY_W_MM/(MIN_UNIT*4),
 MAP_PAGE_XY_W_MM/(MIN_UNIT*8)
};

static const int VOX_ZS[MAX_RESOLEVELS] =
{MAP_PAGE_Z_H_MM/MIN_UNIT,
 MAP_PAGE_Z_H_MM/(MIN_UNIT*2),
 MAP_PAGE_Z_H_MM/(MIN_UNIT*4),
 MAP_PAGE_Z_H_MM/(MIN_UNIT*8)
};


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

#define PAGE_META_FLAG_CHANGED (1<<12) // Page is in memory, and has been changed. Storing is needed.
#define PAGE_META_FLAG_STORED  (1<<13) // Page isn't in memory (anymore), but was changed, and thus was stored. You may want to read it to see changes.
#define PAGE_META_MEM_IDX_MASK (0xfff)

typedef struct __attribute__ ((packed))
{
	uint8_t loaded;

	uint16_t flags_mem_idx; // 4 bits for flags, 12 bits for mem idx
	
} page_meta_t;


#define MAX_PAGES_X 512
#define MAX_PAGES_Y 512
#define MAX_PAGES_Z 32

// approx. 10MB per page. We can afford. approx 500-600MB on Raspi3 (1GB)
// Note:
// The most optimized usage loads multiple pages at once, around the theoretical maximum range of usage (for example, sensor range),
// thus avoiding separate "is loaded?" checks for every point. If MAX_LOADED_PAGES is too small to fit all the pages around
// this range in memory at once, load_pages() call will fail, and the program aborts. To avoid problems:
// * call load_pages() with static ranges so that this failure is not hidden; then make sure to use MAX_LOADED_PAGES large enough;
// * don't use mass load_pages(), but load single pages for every point. Will be slower, but is guaranteed to work as long as
//   MAX_LOADED_PAGES is at least 1.

// Note that even if you call load_pages_quick for every single point, it's a fairly quick operation only if the pages tend to fit
// memory, at least mostly. Pages are shuffled on and off disk multiple times if the value is too small.
#ifdef SLAM_STANDALONE
#define MAX_LOADED_PAGES (256)
#else
#define MAX_LOADED_PAGES (56)
#endif

// Valid ranges of page indeces, inclusive
#define PX_MIN (0)
#define PY_MIN (0)
#define PZ_MIN (0)

#define PX_MAX (MAX_PAGES_X-1)
#define PY_MAX (MAX_PAGES_Y-1)
#define PZ_MAX (MAX_PAGES_Z-1)



#if (MAX_LOADED_PAGES>4095)
#error MAX_LOADED_PAGES too big, mem_idx wont fit into 12 bits
#endif

extern page_meta_t page_metas[MAX_PAGES_X][MAX_PAGES_Y][MAX_PAGES_Z];

typedef struct __attribute__ ((packed))
{
//	uint8_t dummy;
//	uint8_t loaded;

	uint64_t access_timestamp;
	voxmap_t* p_voxmap[MAX_RESOLEVELS]; // [0] = highest resolevel

	// reference back to the meta
	uint16_t px;
	uint16_t py;
	uint16_t pz;

} page_pointer_t;


extern page_pointer_t page_pointers[MAX_LOADED_PAGES];

#include <assert.h>

static inline voxmap_t* get_p_voxmap(int px, int py, int pz, int rl)  __attribute__((always_inline));
static inline voxmap_t* get_p_voxmap(int px, int py, int pz, int rl) 
{
	assert(px >= PX_MIN && px <= PX_MAX && py >= PY_MIN && py <= PY_MAX && pz >= PZ_MIN && pz <= PZ_MAX);
	assert(page_metas[px][py][pz].loaded & (1<<rl));
	return page_pointers[(page_metas[px][py][pz].flags_mem_idx&PAGE_META_MEM_IDX_MASK)].p_voxmap[rl];
}

typedef struct 
{
	int px;
	int ox;
	int py;
	int oy;
	int pz;
	int oz;
} po_coords_t;

// Input: millimeters. Output: page indeces and page offsets.
static inline __attribute__((always_inline)) po_coords_t po_coords(int x, int y, int z, int rl)
{
	x += VOX_UNITS[rl]*VOX_XS[rl]*(MAX_PAGES_X/2);
	y += VOX_UNITS[rl]*VOX_YS[rl]*(MAX_PAGES_Y/2);
	z += VOX_UNITS[rl]*VOX_ZS[rl]*(MAX_PAGES_Z/2);

	x /= VOX_UNITS[rl];
	y /= VOX_UNITS[rl];
	z /= VOX_UNITS[rl];

	po_coords_t ret;
	ret.px = x/VOX_XS[rl];
	ret.py = y/VOX_YS[rl];
	ret.pz = z/VOX_ZS[rl];
	ret.ox = x%VOX_XS[rl];
	ret.oy = y%VOX_YS[rl];
	ret.oz = z%VOX_ZS[rl];
	return ret;
}

// Input: Middle-biased (always positive) block units in whatever resolevel. Output: page indeces and page offsets in that same resolevel
static inline __attribute__((always_inline)) po_coords_t po_unit_coords(int x, int y, int z, int rl)
{
	po_coords_t ret;
	ret.px = x/VOX_XS[rl];
	ret.py = y/VOX_YS[rl];
	ret.pz = z/VOX_ZS[rl];
	ret.ox = x%VOX_XS[rl];
	ret.oy = y%VOX_YS[rl];
	ret.oz = z%VOX_ZS[rl];
	return ret;
}


static inline __attribute__((always_inline)) uint8_t* get_p_voxel(po_coords_t c, int rl)
{
	if(!(c.px >= PX_MIN && c.px <= PX_MAX && c.py >= PY_MIN && c.py <= PY_MAX && c.pz >= PZ_MIN && c.pz <= PZ_MAX) ||
	   !(c.ox >= 0 && c.oy <= VOX_XS[rl]-1 &&  c.oy >= 0 && c.oy <= VOX_YS[rl]-1 && c.oz >= 0 && c.oz <= VOX_ZS[rl]-1))
	{
		printf("OOR coords, page (%d,%d,%d) offs (%d,%d,%d), rl%d\n", c.px, c.py, c.pz, c.ox, c.oy, c.oz, rl);
		abort();
	}
	if(!(page_metas[c.px][c.py][c.pz].loaded & (1<<rl)))
	{
		printf("Page (%d,%d,%d,rl%d) not loaded\n", c.px, c.py, c.pz, rl);
		abort();
	}

	return &page_pointers[(page_metas[c.px][c.py][c.pz].flags_mem_idx&PAGE_META_MEM_IDX_MASK)].p_voxmap[rl]->
		voxels[c.oy*VOX_XS[rl]*VOX_ZS[rl] + c.ox*VOX_ZS[rl] + c.oz];
}

static inline void mark_page_changed(int px, int py, int pz) __attribute__((always_inline));
static inline void mark_page_changed(int px, int py, int pz)
{
	assert(px >= PX_MIN && px <= PX_MAX && py >= PY_MIN && py <= PY_MAX && pz >= PZ_MIN && pz <= PZ_MAX);

	page_metas[px][py][pz].flags_mem_idx |= PAGE_META_FLAG_CHANGED;
}

void mark_page_accessed(int px, int py, int pz);
void mem_manage_pages(int time_threshold);
void load_pages(uint8_t open_files, uint8_t create_emptys, int px_start, int px_end, int py_start, int py_end, int pz_start, int pz_end);
void load_page_quick(int xx, int yy, int zz);
void load_page_quick_and_mark_changed(int xx, int yy, int zz);
void load_page_quick_nonew(int xx, int yy, int zz);

void free_all_pages();
void store_all_pages();
char* gen_fname(char* dir, int px, int py, int pz, int resolevel, char* buf);

// Iterates through all memory-loaded pages; if the page is changed, iterates through all loaded resolevels,
// and calls the supplied function pointer.
void do_something_to_changed_pages(void (*doer)(voxmap_t*));

// Iterates through the whole world range, looking for pages tagged as not-in-memory-but-stored. Loads
// them from disk to memory, to call the supplied function, immediately releasing the memory.
// Removes the stored flag after the operation.
void do_something_to_stored_pages(void (*doer)(voxmap_t*));


