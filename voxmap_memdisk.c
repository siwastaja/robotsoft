/*
	PULUROBOT robotsoft

	Memory / disk management on top of the voxmap.h data type

	See voxmap_memdisk.h for full comments.

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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "voxmap.h"
#include "voxmap_memdisk.h"

static uint64_t cur_timestamp;

page_meta_t page_metas[MAX_PAGES_X][MAX_PAGES_Y][MAX_PAGES_Z];

page_pointer_t* page_pointers[MAX_LOADED_PAGES];

static char* gen_fname(int px, int py, int pz, int resolevel)
{
	static char fname[2048];
	int snprintf_ret = snprintf(fname, 2048, "voxmap_x%d_y%d_z%d_r%d.pluuvox", px, py, pz, resolevel);
	assert(snprintf_ret < 2048);
	return fname;
}

// -1 if not found, else the slot idx
static int find_free_slot()
{
	for(int i = 0; i < MAX_LOADED_PAGES; i++)
	{
		int px_idx = page_pointers[i].px_idx;
		int py_idx = page_pointers[i].py_idx;
		int pz_idx = page_pointers[i].pz_idx;

		if(!page_pointers[i]->loaded)
			return i;
	}
	return -1;
}

// Naming convention: px, py, pz run in the user-exposed range (both sides around 0). px_idx, py_idx, pz_idx run from 0 to MAX_PAGES_*-1

static void store_page(int idx)
{
	assert(idx >= 0 && idx < MAX_LOADED_PAGES);

	int px_idx = page_pointers[idx].px_idx;
	int py_idx = page_pointers[idx].py_idx;
	int pz_idx = page_pointers[idx].pz_idx;

	for(int rl=0; rl < MAX_RESOLEVELS; rl++)
	{
		if(page_metas[px_idx][py_idx][pz_idx]->loaded & (1<<rl))
		{
			printf("INFO: Storing page (%d,%d,%d,rl%d)\n", PX(px_idx), PY(py_idx), PZ(pz_idx), rl);
			assert(page_pointers[idx].p_voxmap[rl] != NULL);
			write_uncompressed_voxmap(page_pointers[idx].p_voxmap[rl], gen_fname(PX(px_idx), PY(py_idx), PZ(pz_idx), rl));
		}
	}
}

static int alloc_read_page_single_rl(int idx, int rl)
{
	assert(idx >= 0 && idx < MAX_LOADED_PAGES);
	assert(rl >= 0 && rl < MAX_RESOLEVELS);

	int px_idx = page_pointers[idx].px_idx;
	int py_idx = page_pointers[idx].py_idx;
	int pz_idx = page_pointers[idx].pz_idx;

	assert(px_idx >= 0 && px_idx < MAX_PAGES_X && py_idx >= 0 && py_idx < MAX_PAGES_X && pz_idx >= 0 && pz_idx < MAX_PAGES_X);

	assert(!(page_metas[px_idx][py_idx][pz_idx].loaded & (1<<rl)));

	page_pointers[idx].p_voxmap[rl] = malloc(sizeof(voxmap_t));

	if(!page_pointers[idx].p_voxmap[rl])
	{
		printf("ERROR: out of memory\n");
		abort();
	}

	int ret = read_uncompressed_voxmap(page_pointers[idx].p_voxmap[rl], gen_fname(PX(px_idx), PY(py_idx), PZ(pz_idx), rl));

	if(ret >= 0)
	{
		page_metas[px_idx][py_idx][pz_idx].loaded |= 1<<rl;
		page_pointers[idx].access_timestamp = cur_timestamp;
	}

	return ret;
}

// Frees a page completely (all resolevels)
static void free_page(int idx)
{
	assert(idx >= 0 && idx < MAX_LOADED_PAGES);

	for(int rl=0; rl < MAX_RESOLEVELS; rl++)
	{
		if(page_metas[page_pointers[idx].px_idx][page_pointers[idx].py_idx][page_pointers[idx].pz_idx].loaded & (1<<rl))
		{
			assert(page_pointers[idx].p_voxmap[rl] != NULL);
			deinit_voxmap(page_pointers[i].p_voxmap);
			free(page_pointers[i].p_voxmap);
			page_pointers[idx].p_voxmap[rl] = NULL;
		}
	}

	page_metas[page_pointers[idx].px_idx][page_pointers[idx].py_idx][page_pointers[idx].pz_idx].loaded = 0;

}

// Finds a free slot id; if not available, garbage collects one spot free, and returns the id for that.
// Doensn't allocate memory. Doesn't update the index table. So safe to use just to "test".
static int create_memslot_for_page()
{
	int idx = find_free_slot();

	if(idx < 0) // Need to arrange some space - kick the oldest out of memory
	{
		uint64_t smallest_stamp = UINT64_MAX;
		int oldest_i = 0;
		for(int i = 0; i < MAX_LOADED_PAGES; i += 8)
		{
			if(page_pointers[i].access_timestamp < smallest_stamp)
			{
				smallest_stamp = page_pointers[i].access_timestamp;
				oldest_i = i;
			}
		}

		store_page(i);
		free_page(i);
	}

	return idx;
}

int alloc_empty_page_single_rl(int idx, int resolevel)
{
	assert(resolevel >= 0 && resolevel < MAX_RESOLEVELS);

	assert(!(page_metas[pxa][pya][pza].loaded&(1<<resolevel)));

	int px_idx = page_pointers[idx].px_idx;
	int py_idx = page_pointers[idx].py_idx;
	int pz_idx = page_pointers[idx].pz_idx;

	assert(px_idx >= 0 && px_idx < MAX_PAGES_X && py_idx >= 0 && py_idx < MAX_PAGES_X && pz_idx >= 0 && pz_idx < MAX_PAGES_X);

	page_pointers[idx].p_voxmap = malloc(sizeof(voxmap_t));
	int ret = init_empty_voxmap(page_pointers[idx].p_voxmap,
		px*VOX_UNITS[resolevel], py*VOX_UNITS[resolevel], pz*VOX_UNITS[resolevel],
		VOX_XS, VOX_YS, VOX_ZS, VOX_UNITS[resolevel], VOX_UNITS[resolevel]);
	assert(ret >= 0);

	page_pointers[idx].access_timestamp = cur_timestamp;
	page_metas[px_idx][py_idx][pz_idx].loaded |= 1<<resolevel;

	return 0;
}


/*
	load_pages: call this whenever you are going to access the map buffers


	open_files: set bits to '1' to search for and open files of the related resolevels.
	If false, only empty maps are created, and potential files overwritten later.

	create_emptys: set bits to '1' to create empty maps, but only when no file is found.

	Both start and end indeces are inclusive.

	Safe and efficient to call even when pages in the range are already loaded.
*/
void load_pages(uint8_t open_files, uint8_t create_emptys, int px_start, int px_end, int py_start, int py_end, int pz_start, int py_end)
{
	assert(!open_files && !create_emptys); // Doesn't make any sense to do, essentially a no-operation.

	int n_x = px_end - px_start + 1;
	int n_y = py_end - py_start + 1;
	int n_z = pz_end - pz_start + 1;

	assert(n_x*n_y*n_z < MAX_LOADED_PAGES);

	for(int xx=px_start; xx<=px_end; xx++)
	{
		for(int yy=py_start; yy<=py_end; y++)
		{
			for(int zz=pz_start; zz<=pz_end; zz++)
			{
				for(int rl=0; rl<MAX_RESOLEVELS; rl++)
				{
					if(!(open_files & (1<<rl)) && !(create_emptys & (1<<rl)))
						continue; // Don't want to do anything on this rl

					if(page_metas[PIDX(xx)][PIDY(yy)][PIDZ(zz)].loaded & (1<<rl))
					{
						printf("INFO: load_pages(): page (%d,%d,%d,rl%d) already loaded\n", xx, yy, zz, rl);
						continue;
					}

					int idx = create_memslot_for_page();
					page_pointers[idx].px_idx = PIDX(xx);
					page_pointers[idx].py_idx = PIDY(yy);
					page_pointers[idx].pz_idx = PIDZ(zz);
					assert((idx&PAGE_META_MEM_IDX_MASK) == idx); // idx must fit into its bitmask.
					page_metas[PIDX(xx)][PIDY(yy)][PIDZ(zz)].flags_mem_idx = idx; // Zero the flags at the same time.

					int do_empty = 0;
					if(open_files & (1<<rl))
					{
						//page_meta_t* m = &page_metas[page_pointers[idx].px_idx][page_pointers[idx].py_idx][page_pointers[idx].pz_idx];
						
						int ret = read_page_single_rl(idx, rl);
						if(ret == ERR_MAPFILE_NOT_FOUND)
							do_empty = 1;
						if(ret < 0 && ret != ERR_MAPFILE_NOT_FOUND)
						{
							printf("WARNING: reading voxmap failed. Starting an empty one. TODO: This may be unwanted behavior.\n");
							do_empty = 1;
						}
					}

					if(do_empty && create_emptys & (1<<rl))
					{
						printf("INFO: load_pages(): allocating a new empty page (%d,%d,%d,rl%d)\n", xx, yy, zz, rl);
						alloc_empty_page_single_rl(idx, rl);
					}

					// As a result, we must have a loaded page, and a valid pointer, leading to a valid voxmap struct. Test everything:
					assert(page_metas[PIDX(xx)][PIDY(yy)][PIDZ(zz)].loaded & (1<<rl));
					assert(page_pointers[page_metas[PIDX(xx)][PIDY(yy)][PIDZ(zz)].flags_mem_idx&PAGE_META_MEM_IDX_MASK].p_voxmap->magic == 0xaa13);
				}
				
			}

		}

	}
	
}

