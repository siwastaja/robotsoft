/*
	PULUROBOT robotsoft API

	Small point cloud API

	Pulurobot robotsoft can output, among other formats, point clouds. This file defines
	a small, compressed point cloud type, with reduced resolution and range.

	The file format has a fixed length header, and a variable length point cloud. The point cloud
	is typically compressed with zlib, either "on-wire" for network transfer, or to
	be written to a file. The byte stream is the same in both cases, and is handled by the API functions
	here.

	The small point cloud is not meant to represent large worlds, but to work as a "map page" or "current view"
	format.

	Coordinate system:
	Positive Z = up against gravity
	Positive X = towards East (if synchronized to compass); preferably towards right on screen
	Positive Y = towards North (if syncronized to compass); preferably towards top of the screen

		nBits   start	res_mm	Range (+/- m)
	Flag	  1	  63
	Source
	   X	  9	  54	  32	8.1m
	   Y	  9	  45	  32	8.1m
	   Z	  7	  38	  32	2.0m
	Point
	   X	 13	  25	  16	65.5m
	   Y	 13	  12	  16	65.5m
	   Z	 12	   0	  16	32.7m

	Resolution can be compile-time changed by changing the #defines;
	numbers of bits are fixed.

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

#include <inttypes.h>
#include "misc.h"

// Resolutions in mm
#define SMALL_CLOUD_SRC_RESO_X 32
#define SMALL_CLOUD_SRC_RESO_Y 32
#define SMALL_CLOUD_SRC_RESO_Z 32

#define SMALL_CLOUD_POINT_RESO_X 16
#define SMALL_CLOUD_POINT_RESO_Y 16
#define SMALL_CLOUD_POINT_RESO_Z 16

typedef struct __attribute__((packed))
{
	uint16_t magic; // 0xAA14
	uint16_t api_version; // Currently 0x0420
	uint8_t  compression; // 0 = uncompressed; 1 = data is zlib compressed (header is uncompressed)
	uint8_t  dummy;

	// Location of the reference zero
	int32_t ref_x_mm;
	int32_t ref_y_mm;
	int32_t ref_z_mm;

	int32_t n_points;
} small_cloud_header_t;

typedef uint64_t small_cloud_t;

#define GET_SMALL_CLOUD_FLAG(cl) !!((cl)&(1ULL<<63))
#define SET_SMALL_CLOUD_FLAG(cl) do{(cl) |= (1ULL<<63)}while(0)
#define UNSET_SMALL_CLOUD_FLAG(cl) do{(cl) &= ~(1ULL<<63)}while(0)

typedef struct
{
	int x;
	int y;
	int z;
} tmp_point_t;

/*
	Bitfields in C are broken beyond imagination because of random reordering, with no attributes to control
	on GCC. However, bitfields of 1 consequtive variable do work, and can be used to specify sign-extend operation
	reliably ( http://graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend )
*/

ALWAYS_INLINE tmp_point_t get_small_cloud_point_native_units(small_cloud_t p)
{
	tmp_point_t ret;

	struct {signed int a:13;} x;
	struct {signed int a:13;} y;
	struct {signed int a:12;} z;
	ret.x = x.a = (p>>25)&0x1fff;
	ret.y = y.a = (p>>12)&0x1fff;
	ret.z = z.a = (p>> 0)&0x0fff;
	return ret;
}

ALWAYS_INLINE tmp_point_t get_small_cloud_point(small_cloud_t p)
{
	tmp_point_t ret;

	struct {signed int a:13;} x;
	struct {signed int a:13;} y;
	struct {signed int a:12;} z;
	ret.x = x.a = (p>>25)&0x1fff;
	ret.y = y.a = (p>>12)&0x1fff;
	ret.z = z.a = (p>> 0)&0x0fff;

	ret.x *= SMALL_CLOUD_POINT_RESO_X;
	ret.y *= SMALL_CLOUD_POINT_RESO_Y;
	ret.z *= SMALL_CLOUD_POINT_RESO_Z;
	return ret;
}

ALWAYS_INLINE tmp_point_t get_small_cloud_source_native_units(small_cloud_t p)
{
	tmp_point_t ret;

	struct {signed int a:9;} x;
	struct {signed int a:9;} y;
	struct {signed int a:7;} z;
	ret.x = x.a = (p>>54)&0x1ff;
	ret.y = y.a = (p>>45)&0x1ff;
	ret.z = z.a = (p>>38)&0x07f;

	return ret;
}


ALWAYS_INLINE tmp_point_t get_small_cloud_source(small_cloud_t p)
{
	tmp_point_t ret;

	struct {signed int a:9;} x;
	struct {signed int a:9;} y;
	struct {signed int a:7;} z;
	ret.x = x.a = (p>>54)&0x1ff;
	ret.y = y.a = (p>>45)&0x1ff;
	ret.z = z.a = (p>>38)&0x07f;

	ret.x *= SMALL_CLOUD_SRC_RESO_X;
	ret.y *= SMALL_CLOUD_SRC_RESO_Y;
	ret.z *= SMALL_CLOUD_SRC_RESO_Z;
	return ret;
}

// For speed, nothing's being done on out-of-range points; they will alias in wrong places.
// For correct results, make sure all data is withing range.
ALWAYS_INLINE small_cloud_t set_small_cloud(int flag, int32_t sx, int32_t sy, int32_t sz, int32_t x, int32_t y, int32_t z)
{
	small_cloud_t ret;

	x /= SMALL_CLOUD_POINT_RESO_X;
	y /= SMALL_CLOUD_POINT_RESO_Y;
	z /= SMALL_CLOUD_POINT_RESO_Z;

	sx /= SMALL_CLOUD_SRC_RESO_X;
	sy /= SMALL_CLOUD_SRC_RESO_Y;
	sz /= SMALL_CLOUD_SRC_RESO_Z;

	ret =	(((uint64_t)flag&1)<<63) |
		(((uint64_t)sx&0x1ff)<<54) |
		(((uint64_t)sy&0x1ff)<<45) |
		(((uint64_t)sx&0x07f)<<38) |
		(((uint64_t)x&0x1fff)<<25) |
		(((uint64_t)y&0x1fff)<<12) |
		(((uint64_t)z&0x0fff)<<0);

	return ret;
}


