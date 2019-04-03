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



typedef struct  __attribute__ ((packed))
{
	uint16_t magic; // 0xaa13
	uint16_t api_version; // Currently 0x0420

	// Location of the first [0,0,0] voxel in the world coordinates:
	int32_t ref_x_mm;
	int32_t ref_y_mm;
	int32_t ref_z_mm;

	// Voxel unit size:
	uint16_t xy_step_mm;
	uint16_t z_step_mm;

	// 
	// Number of voxels in the file is xs*ys*zs
	uint16_t xs;
	uint16_t ys;
	uint16_t zs;


} voxmap_header_t;

typedef struct  __attribute__ ((packed))
{
	voxmap_header_t header;
	uint8_t* voxmap;
} voxmap_t;



int init_empty_voxmap(voxmap_t* vm, int ref_x, int ref_y, int ref_z, int xs, int ys, int zs, int xy_step, int z_step)
{
	int n_voxels = xs*ys*zs;

	if(xs > VOXMAP_MAX_XS || ys > VOXMAP_MAX_YS || zs > VOXMAP_MAX_ZS || n_bytes > VOXMAP_MAX_VOXELS)
	{
		printf("ERROR: Trying to initialize oversized voxel map (%d x %d x %d)\n", xs, ys, zs);
		return -1;
	}

	if(! (vm.voxmap = calloc(xs*ys*zs*sizeof(uint8_t))))
	{
		printf("ERROR: Out of memory trying to allocate voxel map\n");
		return -2;

	}

	vm->magic = 0xaa13;
	vm->api_version = 0x0420;
	vm->ref_x_mm = ref_x;
	vm->ref_y_mm = ref_y;
	vm->ref_z_mm = ref_z;
	vm->xs = xs;
	vm->ys = ys;
	vm->zs = zs;
	vm->xy_step_mm = xy_step;
	vm->z_step_mm = z_step;

	return 0;
}

// Suggested file extension: .pluuvox
int write_uncompressed_voxmap(voxmap_t* vm, char* fname)
{
	// Recheck to avoid ridiculous file sizes, in case someone messed up after init, or forgot to init.
	int n_bytes = vm->xs*vm->ys*vm->zs;
	if(n_bytes < 0 || n_bytes > VOXMAP_MAX_VOXELS)
	{
		printf("ERROR: invalid number of voxels (%d x %d x %d)\n", vm->xs, vm->ys, vm->zs);
		return -1;
	}

	FILE* f = fopen(fname, "wb");
	if(!f)
	{
		printf("Error opening %s for write: %s\n", fname, strerror(errno));
		return -2;
	}

	if(fwrite(&vm->header, sizeof(voxmap_header_t), 1, f) != 1))
	{
		printf("ERROR: fwrite failed\n");
		fclose(f);
		return -2;
	}

	if(fwrite(vm->voxmap, n_bytes, 1, f) != 1))
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
*/
int read_uncompressed_voxmap(voxmap_t* vm, char* fname)
{
	FILE* f = fopen(fname, "rb");
	if(!f)
	{
		printf("Error opening %s for read: %s\n", fname, strerror(errno));
		return -2;
	}

	if(fread(&vm->header, 4, 1, f) != 1))
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

	if(fread((uint8_t*)(&vm->header)+4, sizeof(voxmap_header_t)-4, 1, f) != 1))
	{
		printf("ERROR: fread failed for header, corrupted pluuvox file?\n");
		fclose(f);
		return -2;
	}


	// Recheck to avoid ridiculous file sizes, in case someone messed up after init, or forgot to init.
	int n_bytes = vm->xs*vm->ys*vm->zs;
	if(vm->xs > VOXMAP_MAX_XS || vm->ys > VOXMAP_MAX_YS || vm->zs > VOXMAP_MAX_ZS || n_bytes > VOXMAP_MAX_VOXELS)
	{
		printf("ERROR: Invalid voxel map size (%d x %d x %d)\n", vm->xs, vm->ys, vm->zs);
		fclose(f);
		return -2;
	}


	if(fread(vm->voxmap, n_bytes, 1, f) != 1))
	{
		printf("ERROR: fread failed - partial or corrupt file?\n");
		fclose(f);
		return -2;
	}

	fclose(f);

	return 0;
}



