#pragma once

#include <inttypes.h>
#include "api_board_to_soft.h"
#include "misc.h"
#include "slam_config.h"

// Ray sources are always near the middle of the submap. Consider typical submap robot moving range of 5000mm (+/- 2500mm), so
// 16-bit range of +/-32787mm is plentiful.
// For Z, 16-bit range is more than enough as well -> struct is 16 bytes nicely.

typedef struct __attribute__((packed))
{
	int16_t sx;
	int16_t sy;
	int16_t sz;

	int32_t px;
	int32_t py;
	int16_t pz;
} cloud_point_t;

#define TRANSLATE_CLOUD_POINT(c_, x_, y_, z_) (cloud_point_t){c_.sx + (x_) , c_.sy + (y_) , c_.sz + (z_) , c_.px + (x_) , c_.py + (y_) , c_.pz + (z_)}

#define MAX_POINTS (9600*10*64)
typedef struct
{
	int n_points;
	cloud_point_t points[MAX_POINTS];
} cloud_t;


typedef struct __attribute__((packed))
{
	int16_t x;
	int16_t y;
	int16_t z;
} small_cloud_point_t;



// 24MB of memory with these, coverage 4.1x4.1x2.0 m
// Outside of the filter, all points go through as they are.
// When robot is fully stationary, max two sensors can see the same spot per full scan.
// When moving, three sensors can see the same spot per full scan.
// It's quite unlikely that 10 references would run out for 6 full scans. If it happens, no big harm done, data goes through without filtration
// Tested different numbers of MAX_RAY_SOURCES for the Vuokravarasto Kellari dataset:
// max  4: 390810 skipped points
// max  7: 1544 skipped points
// max  8: 107 skipped points
// max  9: 6 skipped points
// max 10: 0 skipped points
// 10 is good: sizeof(voxfilter_point_t) = 24 bytes (alignable by 4 and 8)
// It doesn't have a lot of extra margin, in some cases a few points could slip through the voxfilter,
// but this is not catastrophic.

#define VOXFILTER_MAX_RAY_SOURCES 10
#define VOXFILTER_XS 128
#define VOXFILTER_YS 128
#define VOXFILTER_ZS 64
#define VOXFILTER_STEP 32


/*
When the voxfilter accumulates (averages) close points from different sources, we must store the information of all
sources involved, so that the free space can be traced from all correct sources. 

In order to filter better, we allow another source point to be used instead, if it's close enough to
the correct source. This happens, for example, if the robot is stationary (creating exact same source coordinates again)
or crawling very slowly (nearly the same). Or, in some cases, when the robot happens to run forward at a certain lucky
speed, so that another sensor gets an image close to the point where the another sensor picked an image earlier.

Upside in increasing this: point clouds gets smaller. Normally, each different source needs to generate a new point even if
the target point is combined by the voxfilter, but if the sources are combined, one point per one combined target suffices.

The only downside to increasing this: free space very close to the robot may not be traced perfectly - some
small stripes and spots *might* remain unknown (not free) even when they are actually free. Note that if you have
a lot of moving people around the robot while mapping, this might prevent the later free space filter from removing
some artefacts.
*/
#define VOXFILTER_SOURCE_COMBINE_THRESHOLD 50 // mm

typedef struct __attribute__((packed))
{
	uint8_t src_idxs[VOXFILTER_MAX_RAY_SOURCES]; // Zero-terminated list of indeces to voxfilter.ray_sources[]  (zero termination not required, if max length used)
	uint16_t cnt;
	int32_t x;
	int32_t y;
	int32_t z;
} voxfilter_point_t;

typedef struct
{
	int x;
	int y;
	int z;
} voxfilter_ray_source_t;

// Remember to zero this struct out first.
typedef struct
{
	int n_ray_sources;
	voxfilter_ray_source_t ray_sources[VOXFILTER_N_SCANS*N_SENSORS+1]; // Start collecting at index 1
	voxfilter_point_t points[VOXFILTER_XS][VOXFILTER_YS][VOXFILTER_ZS];
} voxfilter_t;




int save_cloud(cloud_t* cloud, int idx);
int load_cloud(cloud_t* cloud, int idx);
void rotate_cloud(cloud_t* cloud, double yaw);
void rotate_cloud_copy(cloud_t* cloud, cloud_t* out, double yaw);
void filter_cloud(cloud_t* cloud, cloud_t* out, int32_t transl_x, int32_t transl_y, int32_t transl_z);
void cloud_to_voxmap(cloud_t* cloud, int ref_x, int ref_y, int ref_z);

void tof_to_voxfilter_and_cloud(int is_narrow, uint16_t* ampldist, hw_pose_t pose, int sidx, int32_t ref_x, int32_t ref_y, int32_t ref_z,
	 voxfilter_t* voxfilter, int32_t voxfilter_ref_x, int32_t voxfilter_ref_y, int32_t voxfilter_ref_z, cloud_t* cloud, int voxfilter_threshold, int dist_ignore_threshold);

void voxfilter_to_cloud(voxfilter_t* voxfilter, cloud_t* cloud);

ALWAYS_INLINE void cloud_insert_point(cloud_t* cloud, int16_t sx, int16_t sy, int16_t sz, int32_t px, int32_t py, int32_t pz)
{
	if(cloud->n_points >= MAX_POINTS)
	{
		printf("WARNING: Ignoring point, cloud full.\n");
		return;
	}
	assert(cloud->n_points >= 0);

	//assert(sx != 0 || sy != 0 || sz != 0); // Would be an alarming coincidence. Remove this assert later as it's a real possibility given enough data.

	cloud->points[cloud->n_points].sx = sx;
	cloud->points[cloud->n_points].sy = sy;
	cloud->points[cloud->n_points].sz = sz;

	cloud->points[cloud->n_points].px = px;
	cloud->points[cloud->n_points].py = py;
	cloud->points[cloud->n_points].pz = pz;

	cloud->n_points++;
}

#include "../robotboard2-fw/tof_process.h" // for sensor_mount_t
typedef struct __attribute__((packed))
{
	uint32_t magic;
	uint32_t chip_id;
	uint32_t calib_timestamp;
	uint32_t calib_info;
	uint32_t reserved;

	sensor_mount_t mount;

	uint16_t hor_angs[TOF_XS*TOF_YS];
	uint16_t ver_angs[TOF_XS*TOF_YS];
	
} sensor_softcal_t;

void load_sensor_softcals();

// Temporary legacy shit glued here:
void restart_voxmap(int32_t ref_x, int32_t ref_y);

typedef struct __attribute__((packed))
{
	uint16_t segs[12][VOX_SEG_XS*VOX_SEG_YS];
} full_voxel_map_t;

extern full_voxel_map_t voxmap;
extern int32_t vox_ref_x;
extern int32_t vox_ref_y;



