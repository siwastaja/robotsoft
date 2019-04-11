/*
	PULUROBOT robotsoft API

	Voxel map API

	Voxel map is one (currently the first) map format that Pulurobot robotsoft can output.

	The file format has a fixed length header, and a variable length voxel map. The whole
	file is typically compressed with zlib, either "on-wire" for network transfer, or to
	be written to a file. The byte stream is the same in both cases, and is handled by the API functions
	here.

	The voxmap is not meant to represent large worlds, but to work as a map page format.



	voxmap_t object contains (in memory):

		one voxmap_header_t object, with all the meta information for voxmap access,
		uint8_t* voxmap, pointer to the voxmap, which can be easily accessed with the provided macros

	The byte stream, or file generated contains:

		voxmap_header_t
		variable-length voxel data (xs*ys*zs bytes)


	The API functions handle the file access, compression/decompression, etc.



	Voxel (volumetric pixel) represents a discrete cube within the world: it can be:
	* Unseen
	* Free space
	* Occupied space

	This data format provides rudimentary counters for statistical purposes. The 8-bit
	voxel structure:

	b7	b6	b5	b4	b3	b2	b1	b0
	-----Free cnt-------  latest   ----Occupied cnt----   latest
	                       free                            occu

	See the macros for easy access.


	Map page size and resolution may vary, and multiple map pages can be saved with different resolutions.


	Coordinate system:
	Positive Z = up against gravity
	Positive X = towards East (if synchronized to compass); preferably towards right on screen
	Positive Y = towards North (if syncronized to compass); preferably towards top of the screen


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

#include <stdint.h>
#include "datatypes.h"
#include "api_board_to_soft.h"

#define VOXMAP_MAX_XS 1024
#define VOXMAP_MAX_YS 1024
#define VOXMAP_MAX_ZS 1024

#define VOXMAP_MAX_VOXELS (512*512*128)


typedef struct  __attribute__ ((packed))
{
	uint16_t magic; // 0xAA13
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
	uint8_t* voxels;
} voxmap_t;

/*
	High-performance voxel access: prove your own ranges (x = 0...xs-1, y = 0...ys-1, z = 0...zs-1)

	Usage:
	voxmap_t my_voxmap;
	// voxmap loaded here...

	if(GET_VOX_OCCU_CNT( VOXEL(my_voxmap, 123, 50, 10) ) > 3 )
		printf("Voxel at 123,50,10 has occupance count of over 3");


	Note that when you pass voxmap as a pointer, you dereference it for VOXEL macro:
	VOXEL(*voxmap_in, ...)
	
*/
#define VOXEL(vm_, x_, y_, z_) ((vm_).voxels[((y_)*(vm_).header.xs*(vm_).header.zs) + ((x_)*(vm_).header.zs)  + (z_)])


#define ERR_MAPFILE_NOT_FOUND (-10)


int init_empty_voxmap(voxmap_t* vm, int ref_x, int ref_y, int ref_z, int xs, int ys, int zs, int xy_step, int z_step);
void deinit_voxmap(voxmap_t* vm);
int write_uncompressed_voxmap(voxmap_t* vm, char* fname);
int read_uncompressed_voxmap(voxmap_t* vm, char* fname);

