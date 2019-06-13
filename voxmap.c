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

#include <zlib.h>

#define ZLIB_CHUNK (256*1024)
#define ZLIB_LEVEL 2  // from 1 to 9, 9 = slowest but best compression

/*
	Compression level effect on a typical 50-submap dataset (79 pages at 4 resolutions = 316 files).
	On Lenovo W530 laptop with SSD.
	(Time includes loading uncompressed submap pointclouds and transforming them, which is probably around
	10-20 sec offset.)


	Uncompressed	757.2MB  77 sec	
	Level 1 	 11.2MB  51 sec
	Level 3 	 10.2MB  55 sec
	Level 6 	  6.5MB 122 sec
	
	
*/

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

//	printf("deinit_voxmap: freeing %lx\n", (uint64_t)vm->voxels);
	free(vm->voxels);
}

// Suggested file extension: .pluuvox
int write_voxmap(voxmap_t* vm, char* fname, int compress)
{
	// Recheck to avoid ridiculous file sizes, in case someone messed up after init, or forgot to init.
	int n_bytes = vm->header.xs*vm->header.ys*vm->header.zs;
	if(n_bytes < 0 || n_bytes > VOXMAP_MAX_VOXELS)
	{
		printf("ERROR: invalid number of voxels (%d x %d x %d)\n", vm->header.xs, vm->header.ys, vm->header.zs);
		return -1;
	}

	vm->header.compression = compress?1:0;

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

	if(compress)
	{
		uint8_t outbuf[ZLIB_CHUNK];
		z_stream strm;
		strm.zalloc = Z_NULL;
		strm.zfree = Z_NULL;
		strm.opaque = Z_NULL;
		if(deflateInit(&strm, ZLIB_LEVEL) != Z_OK)
		{
			printf("ERROR: ZLIB initialization failed\n");
			return -1;
		}
		strm.avail_in = n_bytes;
		strm.next_in = (uint8_t*)vm->voxels;

		do
		{
			strm.avail_out = ZLIB_CHUNK;
			strm.next_out = outbuf;

			int ret = deflate(&strm, Z_FINISH);
			assert(ret != Z_STREAM_ERROR);

			int produced = ZLIB_CHUNK - strm.avail_out;

			if(fwrite(outbuf, produced, 1, f) != 1 || ferror(f))
			{
				printf("ERROR: fwrite failed\n");
				deflateEnd(&strm);
				fclose(f);
				return -2;
			}
		} while(strm.avail_out == 0);

		assert(strm.avail_in == 0);

		deflateEnd(&strm);
	}
	else
	{
		if(fwrite(vm->voxels, n_bytes, 1, f) != 1)
		{
			printf("ERROR: fwrite failed\n");
			fclose(f);
			return -2;
		}
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
int read_voxmap(voxmap_t* vm, char* fname)
{
	FILE* f = fopen(fname, "rb");
	if(!f)
	{
		if(errno == ENOENT)
			return ERR_MAPFILE_NOT_FOUND;
		else
		{		
			printf("ERROR: Error opening %s for read: %s\n", fname, strerror(errno));
			return -2;
		}
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

	if(vm->header.compression == 0)
	{
		if(fread(vm->voxels, n_bytes, 1, f) != 1)
		{
			printf("ERROR: fread failed - partial or corrupt file?\n");
			free(vm->voxels);
			vm->voxels = NULL;
			fclose(f);
			return -2;
		}
	}
	else if(vm->header.compression == 1) // ZLIB
	{
		uint8_t inbuf[ZLIB_CHUNK];
		z_stream strm;
		strm.zalloc = Z_NULL;
		strm.zfree = Z_NULL;
		strm.opaque = Z_NULL;
		strm.avail_in = 0;
		strm.next_in = Z_NULL;

		if(inflateInit(&strm) != Z_OK)
		{
			printf("ERROR: ZLIB initialization failed\n");
			free(vm->voxels);
			vm->voxels = NULL;
			fclose(f);
			return -1;
		}

		int got_bytes = 0;
		int bytes_left = n_bytes;
		int ret = 0;
		do
		{
			strm.avail_in = fread(inbuf, 1, ZLIB_CHUNK, f);
			if(ferror(f))
			{
				printf("ERROR reading voxmap input file\n");
				inflateEnd(&strm);
				free(vm->voxels);
				vm->voxels = NULL;
				fclose(f);
				return -1;
			}
			if(strm.avail_in == 0)
				break;

			strm.next_in = inbuf;

			do
			{
				strm.avail_out = bytes_left;
				strm.next_out = (uint8_t*)vm->voxels + got_bytes;

				ret = inflate(&strm, Z_FINISH);
				assert(ret != Z_STREAM_ERROR);

				switch(ret)
				{
					case Z_NEED_DICT:
					case Z_DATA_ERROR:
					case Z_MEM_ERROR:
					{
						printf("ERROR: voxmap input file decompression error, inflate() returned %d\n", ret);
						inflateEnd(&strm);
						free(vm->voxels);
						vm->voxels = NULL;
						fclose(f);
						return -1;
					}
					default: break;
				}

				got_bytes += bytes_left - strm.avail_out;

			} while(strm.avail_out == 0);
		} while(ret != Z_STREAM_END);

		inflateEnd(&strm);

	}
	else
	{
		printf("ERROR: illegal compression field in pluvox header, corrupted file or new, unsupported compression?\n");
			free(vm->voxels);
			vm->voxels = NULL;
		fclose(f);
		return -2;
	}

	fclose(f);

	return 0;
}


