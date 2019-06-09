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

#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "voxmap.h"
#include "voxmap_memdisk.h"

static uint64_t cur_timestamp;

page_meta_t page_metas[MAX_PAGES_X][MAX_PAGES_Y][MAX_PAGES_Z];

page_pointer_t page_pointers[MAX_LOADED_PAGES];

char* gen_fname(char* dir, int px, int py, int pz, int resolevel, char* buf)
{
	int snprintf_ret = snprintf(buf, 2048, "%s/voxmap_x%d_y%d_z%d_r%d.pluuvox", dir, px, py, pz, resolevel);
	assert(snprintf_ret < 2048);
	return buf;
}

// -1 if not found, else the slot idx
static int find_free_slot()
{
	for(int i = 0; i < MAX_LOADED_PAGES; i++)
	{
		int px = page_pointers[i].px;
		int py = page_pointers[i].py;
		int pz = page_pointers[i].pz;

		if(!(page_metas[px][py][pz].loaded)) // All resolevels must be free
			return i;
	}
	return -1;
}

static char fnamebuf[2048];

static void store_page(int idx)
{
	assert(idx >= 0 && idx < MAX_LOADED_PAGES);

	int px = page_pointers[idx].px;
	int py = page_pointers[idx].py;
	int pz = page_pointers[idx].pz;

	if(!(page_metas[px][py][pz].flags_mem_idx & PAGE_META_FLAG_CHANGED))
	{
		//printf("INFO: page (%d,%d,%d) unchanged, not storing.\n", px, py, pz);
		return;
	}

	for(int rl=0; rl < MAX_RESOLEVELS; rl++)
	{
		if(page_metas[px][py][pz].loaded & (1<<rl))
		{
			assert(page_pointers[idx].p_voxmap[rl] != NULL);
			//printf("INFO: Storing page (%d,%d,%d,rl%d)\n", px, py, pz, rl);
			write_uncompressed_voxmap(page_pointers[idx].p_voxmap[rl], gen_fname("../robotui/current_maps", px, py, pz, rl, fnamebuf));			
		}
	}
}

static int alloc_read_page_single_rl(int idx, int rl)
{
	assert(idx >= 0 && idx < MAX_LOADED_PAGES);
	assert(rl >= 0 && rl < MAX_RESOLEVELS);

	int px = page_pointers[idx].px;
	int py = page_pointers[idx].py;
	int pz = page_pointers[idx].pz;

	assert(px >= 0 && px < MAX_PAGES_X && py >= 0 && py < MAX_PAGES_X && pz >= 0 && pz < MAX_PAGES_X);

	assert(!(page_metas[px][py][pz].loaded & (1<<rl)));

	page_pointers[idx].p_voxmap[rl] = malloc(sizeof(voxmap_t));

	if(!page_pointers[idx].p_voxmap[rl])
	{
		printf("ERROR: out of memory\n");
		abort();
	}

	int ret = read_uncompressed_voxmap(page_pointers[idx].p_voxmap[rl], gen_fname("../robotui/current_maps", px, py, pz, rl, fnamebuf));

	if(ret >= 0)
	{
//		printf("INFO: Succesfully loaded file from the disk (%d,%d,%d,rl%d), memslot %d\n", px, py, pz, rl, idx);
		page_metas[px][py][pz].loaded |= 1<<rl;
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
		//printf("rl = %d\n", rl);
		if(page_metas[page_pointers[idx].px][page_pointers[idx].py][page_pointers[idx].pz].loaded & (1<<rl))
		{
			assert(page_pointers[idx].p_voxmap[rl] != NULL);
			//printf("page_pointers[idx].p_voxmap[rl] = %lx\n", (uint64_t)page_pointers[idx].p_voxmap[rl]);
			deinit_voxmap(page_pointers[idx].p_voxmap[rl]);
			free(page_pointers[idx].p_voxmap[rl]);
			page_pointers[idx].p_voxmap[rl] = NULL;
		}
	}

	page_metas[page_pointers[idx].px][page_pointers[idx].py][page_pointers[idx].pz].loaded = 0;

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
		for(int i = 0; i < MAX_LOADED_PAGES; i++)
		{
			if(page_pointers[i].access_timestamp < smallest_stamp)
			{
				smallest_stamp = page_pointers[i].access_timestamp;
				oldest_i = i;
			}
		}

		//printf("INFO: Kicking out oldest page idx=%d (%d,%d,%d), timestamp %" PRIu64 "\n", oldest_i,
		//	page_pointers[oldest_i].px, page_pointers[oldest_i].py, page_pointers[oldest_i].pz,
		//	smallest_stamp);

		store_page(oldest_i);
		free_page(oldest_i);

		idx = oldest_i;
	}

	return idx;
}

static int find_memslot_for_page(int px, int py, int pz)
{
	if(page_metas[px][py][pz].loaded) // If any resolevel is loaded, this is the page...
		return page_metas[px][py][pz].flags_mem_idx&PAGE_META_MEM_IDX_MASK;
	else
		return -1;
}

static int alloc_empty_page_single_rl(int idx, int rl)
{
	assert(rl >= 0 && rl < MAX_RESOLEVELS);

	int px = page_pointers[idx].px;
	int py = page_pointers[idx].py;
	int pz = page_pointers[idx].pz;

	assert(!(page_metas[px][py][pz].loaded&(1<<rl)));

	assert(px >= 0 && px < MAX_PAGES_X && py >= 0 && py < MAX_PAGES_X && pz >= 0 && pz < MAX_PAGES_X);

	page_pointers[idx].p_voxmap[rl] = malloc(sizeof(voxmap_t));
	int ret = init_empty_voxmap(page_pointers[idx].p_voxmap[rl],
		px*VOX_UNITS[rl], py*VOX_UNITS[rl], pz*VOX_UNITS[rl],
		VOX_XS[rl], VOX_YS[rl], VOX_ZS[rl], VOX_UNITS[rl], VOX_UNITS[rl]);
	assert(ret >= 0);

	page_pointers[idx].access_timestamp = cur_timestamp;
	page_metas[px][py][pz].loaded |= 1<<rl;

	return 0;
}


/*
	load_pages: call this whenever you are going to access the map buffers


	open_files: set bits to '1' to search for and open files of the related resolevels.
	If false, only empty maps are created, and potential files overwritten later.

	create_emptys: set bits to '1' to create empty maps, but only when no file is found.

	Both start and end indeces are inclusive.

	Safe and efficient to call even when pages in the range are already loaded.

	Also safe to call even if ranges are partially outside of valid range (fully out of range does nothing)
*/
void load_pages(uint8_t open_files, uint8_t create_emptys, int px_start, int px_end, int py_start, int py_end, int pz_start, int pz_end)
{
	assert(open_files || create_emptys); // Doesn't make any sense to do nothing

	// Clip out-of-range operations:

	if(px_start < PX_MIN) px_start = PX_MIN;
	else if(px_start > PX_MAX) px_start = PX_MAX;

	if(py_start < PY_MIN) py_start = PY_MIN;
	else if(py_start > PY_MAX) py_start = PY_MAX;

	if(pz_start < PZ_MIN) pz_start = PZ_MIN;
	else if(pz_start > PZ_MAX) pz_start = PZ_MAX;

	if(px_end < PX_MIN) px_end = PX_MIN;
	else if(px_end > PX_MAX) px_end = PX_MAX;

	if(py_end < PY_MIN) py_end = PY_MIN;
	else if(py_end > PY_MAX) py_end = PY_MAX;

	if(pz_end < PZ_MIN) pz_end = PZ_MIN;
	else if(pz_end > PZ_MAX) pz_end = PZ_MAX;


	int n_x = px_end - px_start + 1;
	int n_y = py_end - py_start + 1;
	int n_z = pz_end - pz_start + 1;

	// We use an assumption that all pages to be loaded at once will fit.
	// Worst case, we kick out everything earlier, but we assume we don't kick
	// the pages we are currently loading, during the loading.
	assert(n_x*n_y*n_z < MAX_LOADED_PAGES);

	cur_timestamp++;

	//printf("load_pages: n_x=%d, n_y=%d, n_z=%d, n_total = %d, cur_timestamp = %"PRIu64"\n", n_x, n_y, n_z, n_x*n_y*n_z, cur_timestamp);

	//printf("Allocation round:\n");
	for(int xx=px_start; xx<=px_end; xx++)
	{
		for(int yy=py_start; yy<=py_end; yy++)
		{
			for(int zz=pz_start; zz<=pz_end; zz++)
			{
				for(int rl=0; rl<MAX_RESOLEVELS; rl++)
				{
					if(!(open_files & (1<<rl)) && !(create_emptys & (1<<rl)))
						continue; // Don't want to do anything on this rl

					//printf("xx=%d, yy=%d, zz=%d, rl=%d\n", xx, yy, zz, rl);

					if(page_metas[xx][yy][zz].loaded & (1<<rl))
					{
						//printf("INFO: load_pages(): page (%d,%d,%d,rl%d) already loaded\n", xx, yy, zz, rl);
						// Need to update the timestamp: otherwise this page we need could be kicked out.
						page_pointers[page_metas[xx][yy][zz].flags_mem_idx&PAGE_META_MEM_IDX_MASK].access_timestamp = cur_timestamp;
						continue;
					}

					int idx = find_memslot_for_page(xx, yy, zz);
					if(idx < 0)
						idx = create_memslot_for_page();
					page_pointers[idx].px = xx;
					page_pointers[idx].py = yy;
					page_pointers[idx].pz = zz;
					assert((idx&PAGE_META_MEM_IDX_MASK) == idx); // idx must fit into its bitmask.
					page_metas[xx][yy][zz].flags_mem_idx = idx; // Zero the flags at the same time.

					int do_empty = 0;
					if(open_files & (1<<rl))
					{
						//page_meta_t* m = &page_metas[page_pointers[idx].px][page_pointers[idx].py][page_pointers[idx].pz];
						
						int ret = alloc_read_page_single_rl(idx, rl);
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
						//printf("INFO: load_pages(): allocating a new empty page (%d,%d,%d,rl%d), memslot %d\n", xx, yy, zz, rl, idx);
						alloc_empty_page_single_rl(idx, rl);
					}

					// As a result, we must have a loaded page, and a valid pointer, leading to a valid voxmap struct. Test everything:
					assert(page_metas[xx][yy][zz].loaded & (1<<rl));
					int check_idx = page_metas[xx][yy][zz].flags_mem_idx&PAGE_META_MEM_IDX_MASK;
					//printf("PIDXES (%d, %d, %d), check_idx=%d\n", xx, yy, zz, check_idx);

					assert(page_pointers[check_idx].p_voxmap[rl]);
					assert(page_pointers[check_idx].p_voxmap[rl]->header.magic == 0xaa13);
				}
				
			}

		}

	}

	//printf("verification round:\n");
	
	// Verification after the operations:
	for(int xx=px_start; xx<=px_end; xx++)
	{
		for(int yy=py_start; yy<=py_end; yy++)
		{
			for(int zz=pz_start; zz<=pz_end; zz++)
			{
				for(int rl=0; rl<MAX_RESOLEVELS; rl++)
				{
					if(!(open_files & (1<<rl)) && !(create_emptys & (1<<rl)))
						continue; // Don't want to do anything on this rl

					//printf("xx=%d, yy=%d, zz=%d, rl=%d\n", xx, yy, zz, rl);


					int check_idx = page_metas[xx][yy][zz].flags_mem_idx&PAGE_META_MEM_IDX_MASK;
					//printf("PIDXES (%d, %d, %d), check_idx=%d\n", xx, yy, zz, check_idx);

					assert(page_metas[xx][yy][zz].loaded & (1<<rl));
					assert(page_pointers[check_idx].p_voxmap[rl]);
					assert(page_pointers[check_idx].p_voxmap[rl]->header.magic == 0xaa13);
				}
			}
		}
	}
	//printf("Verification OK\n");
}


void mark_page_accessed(int px, int py, int pz)
{
//	printf("Mark page accessed (%d,%d,%d)\n", px, py, pz);

	assert(px >= PX_MIN && px <= PX_MAX && py >= PY_MIN && py <= PY_MAX && pz >= PZ_MIN && pz <= PZ_MAX);

	unsigned int idx = page_metas[px][py][pz].flags_mem_idx&PAGE_META_MEM_IDX_MASK;
	assert(idx < MAX_LOADED_PAGES);

	page_pointers[idx].access_timestamp = cur_timestamp;
}

void mem_manage_pages(int time_threshold)
{
	cur_timestamp++;
	//printf("mem_manage_pages, cur_timestamp=%"PRIu64"\n", cur_timestamp);

	uint64_t thresh = (uint64_t)time_threshold;
	if(thresh > cur_timestamp) thresh = cur_timestamp;

	uint64_t kickout_timestamp = cur_timestamp - thresh;

	for(int idx=0; idx < MAX_LOADED_PAGES; idx++)
	{
		if(page_metas[page_pointers[idx].px][page_pointers[idx].py][page_pointers[idx].pz].loaded &&
		   page_pointers[idx].access_timestamp < kickout_timestamp)
		{
			//printf("INFO: mem_manage_pages: kicking out old page (%d,%d,%d), access timestamp = %"PRIu64", cur_timestamp = %"PRIu64"\n",
			//	page_pointers[idx].px, page_pointers[idx].py, page_pointers[idx].pz,
			//	page_pointers[idx].access_timestamp, cur_timestamp);

			store_page(idx);
			free_page(idx);
		}
	} 
}

void free_all_pages()
{
	for(int idx=0; idx < MAX_LOADED_PAGES; idx++)
	{
		if(page_metas[page_pointers[idx].px][page_pointers[idx].py][page_pointers[idx].pz].loaded)
		{
//			printf("free_all_pages: idx=%d, (%d,%d,%d)\n", idx, 
//				page_pointers[idx].px, page_pointers[idx].py, page_pointers[idx].pz);
			store_page(idx);
			free_page(idx);
		}
	}

	
}

void store_all_pages()
{
	for(int idx=0; idx < MAX_LOADED_PAGES; idx++)
	{
		if(page_metas[page_pointers[idx].px][page_pointers[idx].py][page_pointers[idx].pz].loaded)
		{
//			printf("free_all_pages: idx=%d, (%d,%d,%d)\n", idx, 
//				page_pointers[idx].px, page_pointers[idx].py, page_pointers[idx].pz);
			store_page(idx);
		}
	}

	
}

