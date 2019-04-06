/*
	PULUROBOT robotsoft API

	Voxel map API

	See voxmap.h for full comments

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
#include <errno.h>
#include <assert.h>

#include "voxmap.h"

int init_empty_voxmap(voxmap_t* vm, int ref_x, int ref_y, int ref_z, int xs, int ys, int zs, int xy_step, int z_step)
{
	int n_voxels = xs*ys*zs;

	if(xs > VOXMAP_MAX_XS || ys > VOXMAP_MAX_YS || zs > VOXMAP_MAX_ZS || n_voxels > VOXMAP_MAX_VOXELS)
	{
		printf("ERROR: Trying to initialize oversized voxel map (%d x %d x %d)\n", xs, ys, zs);
		return -1;
	}

	if(! (vm->voxels = calloc(xs*ys*zs, sizeof(uint8_t))))
	{
		printf("ERROR: Out of memory trying to allocate voxel map\n");
		return -2;

	}

	vm->header.magic = 0xaa13;
	vm->header.api_version = 0x0420;
	vm->header.ref_x_mm = ref_x;
	vm->header.ref_y_mm = ref_y;
	vm->header.ref_z_mm = ref_z;
	vm->header.xs = xs;
	vm->header.ys = ys;
	vm->header.zs = zs;
	vm->header.xy_step_mm = xy_step;
	vm->header.z_step_mm = z_step;

	return 0;
}

void deinit_voxmap(voxmap_t* vm)
{
	assert(vm);
	assert(vm->header.magic == 0xaa13);

	free(vm->voxels);
}

// Suggested file extension: .pluuvox
int write_uncompressed_voxmap(voxmap_t* vm, char* fname)
{
	// Recheck to avoid ridiculous file sizes, in case someone messed up after init, or forgot to init.
	int n_bytes = vm->header.xs*vm->header.ys*vm->header.zs;
	if(n_bytes < 0 || n_bytes > VOXMAP_MAX_VOXELS)
	{
		printf("ERROR: invalid number of voxels (%d x %d x %d)\n", vm->header.xs, vm->header.ys, vm->header.zs);
		return -1;
	}

	FILE* f = fopen(fname, "wb");
	if(!f)
	{
		printf("Error opening %s for write: %s\n", fname, strerror(errno));
		return -2;
	}

	if(fwrite(&vm->header, sizeof(voxmap_header_t), 1, f) != 1)
	{
		printf("ERROR: fwrite failed\n");
		fclose(f);
		return -2;
	}

	if(fwrite(vm->voxels, n_bytes, 1, f) != 1)
	{
		printf("ERROR: fwrite failed\n");
		fclose(f);
		return -2;
	}

	fclose(f);

	return 0;
}

/*
Suggested file extension: .pluuvox

Usage:
	voxmap_t vm;
	read_uncompressed_voxmap(&vm, "file.pluuvox"); // Does all initialization and memory allocation internally
	...
	deinit_voxmap(&vm); // Free memory
	// in this example, vm was directly in stack. If it was malloc'd, free it manually.
*/
int read_uncompressed_voxmap(voxmap_t* vm, char* fname)
{
	FILE* f = fopen(fname, "rb");
	if(!f)
	{
		printf("Error opening %s for read: %s\n", fname, strerror(errno));
		return ERR_MAPFILE_NOT_FOUND;
	}

	if(fread(&vm->header, 4, 1, f) != 1)
	{
		printf("ERROR: fread failed for first four bytes of header, corrupted pluuvox file?\n");
		fclose(f);
		return -2;
	}

	if(vm->header.magic != 0xaa13)
	{
		printf("ERROR: wrong magic value; file is not a proper pluuvox file\n");
		fclose(f);
		return -2;
	}

	if(vm->header.api_version != 0x0420)
	{
		printf("ERROR: wrong API version %04x - expecting %04x\n", vm->header.api_version, 0x0420);
		fclose(f);
		return -2;
	}

	if(fread((uint8_t*)(&vm->header)+4, sizeof(voxmap_header_t)-4, 1, f) != 1)
	{
		printf("ERROR: fread failed for header, corrupted pluuvox file?\n");
		fclose(f);
		return -2;
	}


	// Recheck to avoid ridiculous file sizes, in case someone messed up after init, or forgot to init.
	int n_bytes = vm->header.xs*vm->header.ys*vm->header.zs;
	if(vm->header.xs > VOXMAP_MAX_XS || vm->header.ys > VOXMAP_MAX_YS || vm->header.zs > VOXMAP_MAX_ZS || n_bytes > VOXMAP_MAX_VOXELS)
	{
		printf("ERROR: Invalid voxel map size (%d x %d x %d)\n", vm->header.xs, vm->header.ys, vm->header.zs);
		fclose(f);
		return -2;
	}

	vm->voxels = malloc(n_bytes);
	if(vm->voxels == NULL)
	{
		printf("ERROR: out of memory.\n");
		abort();
	}

	if(fread(vm->voxels, n_bytes, 1, f) != 1)
	{
		printf("ERROR: fread failed - partial or corrupt file?\n");
		free(vm->voxels);
		vm->voxels = NULL;
		fclose(f);
		return -2;
	}

	fclose(f);

	return 0;
}






