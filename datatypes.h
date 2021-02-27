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



*/

#pragma once

#include <math.h>
#ifndef M_PI
#define M_PI 3.141592653589793238
#endif

#include <stdint.h>


#define ANG_360_DEG_ULL 4294967296ULL
#define ANG_360_DEG_LL  4294967296LL

#define UANG_180_DEG 2147483648UL
#define UANG_90_DEG  1073741824UL
#define UANG_2_5_DEG   29826162UL
#define UANG_1_DEG     11930465UL
#define UANG_0_5_DEG    5965232UL
#define UANG_0_25_DEG   2982616UL
#define UANG_0_125_DEG  1491308UL
#define UANG_0_1_DEG    1193047UL
#define UANG_0_05_DEG    596523UL
#define UANG_0_01_DEG    119305UL
#define UANG_0_001_DEG    11930UL
#define UANG_1PER16_DEG  745654UL  // cumulated full circle rounding error: 0.000006%


#define ANG_180_DEG 2147483648UL
#define ANG_90_DEG  1073741824
#define ANG_2_5_DEG   29826162
#define ANG_1_DEG     11930465
#define ANG_0_5_DEG    5965232
#define ANG_0_25_DEG   2982616
#define ANG_0_125_DEG  1491308
#define ANG_0_1_DEG    1193047
#define ANG_0_05_DEG    596523
#define ANG_0_01_DEG    119305
#define ANG_0_001_DEG    11930
#define ANG_1PER16_DEG  745654  // cumulated full circle rounding error: 0.000006%


#define ANG_I32TORAD(x) ( ((double)((int32_t)(x)))/683565275.576432)
#define ANG32TOFDEG(x) ( ((double)((uint32_t)(x)))/11930464.7111111)
#define ANG_I32TOFDEG(x) ( ((double)((int32_t)(x)))/11930464.7111111)
#define RADTODEG(x) ((x)*(360.0/(2.0*M_PI)))
#define DEGTORAD(x) ((x)*((2.0*M_PI)/360.0))
#define RADTOANG32(x) ( (int32_t)((((double)(x)) / (2.0*M_PI)) * 4294967296.0))

#define DEGTOANG16(x)  ((uint16_t)((float)(x)/(360.0)*65536.0))
#define DEGTOANG32(x)  ((uint16_t)((float)(x)/(360.0)*4294967296.0))


#define ANG32TORAD(x) ( ((double)((uint32_t)(x)))/683565275.576432)
#define ANG32TOFDEG(x) ( ((double)((uint32_t)(x)))/11930464.7111111)
#define ANGI32TORAD(x) ( ((double)((int32_t)(x)))/683565275.576432)
#define ANGI32TOFDEG(x) ( ((double)((int32_t)(x)))/11930464.7111111)


#define RADTODEG(x) ((x)*(360.0/(2.0*M_PI)))
#define DEGTORAD(x) ((x)*((2.0*M_PI)/360.0))
#define RADTOANGU32(x) ( (uint32_t)((((double)(x)) / (2.0*M_PI)) * 4294967296.0))
#define RADTOANGI32(x) ( ( int32_t)((((double)(x)) / (2.0*M_PI)) * 4294967296.0))

#define RADTOANGU16(x) ( (uint16_t)((((double)(x)) / (2.0*M_PI)) * 65536.0))
#define RADTOANGI16(x) ( ( int16_t)((((double)(x)) / (2.0*M_PI)) * 65536.0))

typedef struct __attribute__((packed))
{
	uint32_t ang; // uint32_t range --> 0..360 deg; let it overflow freely. 1 unit = 83.81903171539 ndeg
	int32_t x;   // in mm
	int32_t y;
} pos_t;

typedef struct __attribute__((packed))
{
	int valid;
	int32_t x;   // in mm
	int32_t y;
} point_t;

typedef struct
{
	int32_t x;
	int32_t y;
	int16_t z;
	int8_t c;
} sonar_point_t;


typedef struct __attribute__((packed))
{
	int32_t x;
	int32_t y;
	int32_t z;
} xyz_t;


extern int32_t hwdbg[10];

typedef struct
{
	int status;
	int id;
	int remaining;
	uint32_t micronavi_stop_flags;
	uint32_t micronavi_action_flags;
	uint32_t feedback_stop_flags;
	int stop_xcel_vector_valid;
	float stop_xcel_vector_ang_rad;
} xymove_t;

extern xymove_t cur_xymove;


#define MAP_SIGNIFICANT_IMGS     1
#define MAP_SEMISIGNIFICANT_IMGS 2
extern int map_significance_mode;

typedef enum 
{
	INFO_STATE_UNDEF = -1,
	INFO_STATE_IDLE = 0,
	INFO_STATE_THINK = 1,
	INFO_STATE_FWD = 2,
	INFO_STATE_REV = 3,
	INFO_STATE_LEFT = 4,
	INFO_STATE_RIGHT = 5,
	INFO_STATE_CHARGING = 6,
	INFO_STATE_DAIJUING = 7
} info_state_t;


