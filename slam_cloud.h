#pragma once

#include <inttypes.h>
#include "api_board_to_soft.h"
#include "misc.h"
#include "slam_config.h"


// Cloud resolution
#define CLOUD_MM       16
#define CLOUD_MM_SHIFT  4

typedef struct __attribute__((packed))
{
	uint16_t src_idx;
	int16_t x;
	int16_t y;
	int16_t z;
} cloud_point_t;

// Hard maximum number of points, for failure detection (early abort instead of freezing or ENOMEM)
// Each full scan produces max 9600*10 points. Approx 1 scan per second.
// Support 5 minutes of scans
#define MAX_POINTS (9600*10*60*5)

// Maximize performance vs. memory usage, having to call realloc() every now and then is just fine,
// but having to call it multiple times for every point cloud is only a performance hindrance.
// Linux handles reallocs very well because large chunks use mmap internally which can move data
// around without actually moving any data.
// So make this a bit smaller than a "typical" pointcloud.
// Say half of the points are valid, and we collect for 15 seconds. This would be 9600/2*10*15 = 720000
// Since a point is 8 bytes, this is 5.5Mbytes. So let's just do 4MBytes = 524288 points
// Have another option to initialize a cloud for smaller usage, 0.5MBytes = 65536 points
#define INITIAL_POINTS_ALLOC_LARGE (524288)
#define INITIAL_POINTS_ALLOC_SMALL (65536)

// Hard maximum number of sources (for failure detection as well)
// Each full scan produces 10 sources. Approx 1 scan per second
// Support 5 minutes of scans
// For initial allocations, similar reasoning (see above)
#define MAX_SOURCES (10*60*5)
#define INITIAL_SRCS_ALLOC_LARGE (256)
#define INITIAL_SRCS_ALLOC_SMALL (64)

typedef struct
{
	int n_srcs;
	int alloc_srcs;
	int n_points;
	int alloc_points;
	cloud_point_t* srcs;  // when used as sources, cloud_point_t.src_idx is dummy.
	cloud_point_t* points;
} cloud_t;

// cloud_t instances are small, can be stored in stack, arrays, etc. efficiently.
// remember to init_cloud and free_cloud, though.

#define CLOUD_INIT_SMALL (1<<0)

// New cloud must be created with this function. All-zeroes is no proper initialization.
void init_cloud(cloud_t* cloud, int flags)
{
	cloud->n_srcs = 0;
	cloud->n_points = 0;
	
	if(flags & CLOUD_INIT_SMALL)
	{
		cloud->alloc_srcs = INITIAL_SRCS_ALLOC_SMALL;
		cloud->alloc_points = INITIAL_POINTS_ALLOC_SMALL;
	}
	else
	{
		cloud->alloc_srcs = INITIAL_SRCS_ALLOC_LARGE;
		cloud->alloc_points = INITIAL_POINTS_ALLOC_LARGE;
	}

	cloud->srcs = malloc(cloud->alloc_srcs * sizeof (cloud_point_t));
	assert(cloud->srcs);
	cloud->points = malloc(cloud->alloc_points * sizeof (cloud_point_t));
	assert(cloud->points);
}

void free_cloud(cloud_t* cloud)
{
	free(cloud->srcs);
	free(cloud->points);
}

ALWAYS_INLINE int cloud_add_source(cloud_t* cloud, cloud_point_t p)
{
	if(cloud->n_srcs >= cloud->alloc_srcs)
	{
		cloud->alloc_srcs <<= 1;
		cloud->srcs = realloc(cloud->srcs, cloud->alloc_srcs * sizeof(cloud_point_t));
		assert(cloud->srcs);
	}

	cloud->srcs[cloud->n_srcs] = p;

	return cloud->n_srcs++;
}

// Find existing (close enough) source, use it. If suitable source is not found, a new source is added. Returns the source idx.
// For now, linear search is used because the number of sources is fairly low, and there are usually thousands of points per source.
// Never fails.
ALWAYS_INLINE int cloud_find_source(cloud_t* cloud, cloud_point_t p)
{
	int_fast32_t closest = INT_FAST32_MAX;
	for(int i=0; i<cloud->n_srcs; i++)
	{
		int_fast32_t sqdist = sq(p.x - cloud->srcs[i].x) + sq(p.y - cloud->srcs[i].y) + sq(p.z - cloud->srcs[i].z);
		if(sqdist < closest)
		{
			closest = sqdist;
			closest_i = i;
		}
	}

	if(closest <= sq(SOURCE_COMBINE_THRESHOLD))
		return i;

	return cloud_add_source(cloud, p);
}

ALWAYS_INLINE void cloud_add_point(cloud_t* cloud, cloud_point_t p)
{
	if(cloud->n_points >= cloud->alloc_points)
	{
		cloud->alloc_points <<= 1;
		cloud->points = realloc(cloud->points, cloud->alloc_points * sizeof(cloud_point_t));
		assert(cloud->points);
	}

	cloud->points[cloud->n_points++] = p;
}


// 128x128x64, step 32: 16MB of memory with these dimensions, coverage 4.1x4.1x2.0 m
// Outside of the filter, all points go through as they are.
// When robot is fully stationary, max two sensors can see the same spot per full scan.
// When moving, three sensors can see the same spot per full scan.
// It's quite unlikely that 10 references would run out for 6 full scans (VOXFILTER_N_SCANS was 6 when writing this).
// If it happens, no big harm done, some data goes through without filtration.
// Tested different numbers of MAX_RAY_SOURCES for the Vuokravarasto Kellari dataset:
// max  4: 390810 skipped points
// max  7: 1544 skipped points
// max  8: 107 skipped points
// max  9: 6 skipped points
// max 10: 0 skipped points
// 6 is good: sizeof(voxfilter_point_t) = 16 bytes (alignable by 4 and 8)
// It doesn't have a lot of extra margin, in some cases a few points could slip through the voxfilter,
// but this is not catastrophic.

#define VOXFILTER_MAX_RAY_SOURCES 6
#define VOXFILTER_XS 128
#define VOXFILTER_YS 128
#define VOXFILTER_ZS 64
//#define VOXFILTER_STEP (CLOUD_MM*2)


/*
Voxfilter: Because sensors output points at constant angular resolution, nearby surfaces produce
a huge clumps of points, as opposed to faraway surfaces. Voxfilter is a simple, first-order filter
that reduces the amount of data before it is accumulated in memory. Instead of storing points directly
to the pointcloud, it is stored using the voxfilter_insert_point interface. If the point fits inside the
(fairly small) voxfilter area of voxels, i.e., it's in the dense near field sensing area, it is averaged within
other points hitting the same voxel. If the point is outside the voxelized area, point is added to the pointcloud
directly instead. After enough scans are processed (e.g., the robot has moved enough that the small voxfilter
range is running out, or at latest, when we want to output the point cloud), the points contained in the voxfilter
are added to the point cloud at once. At this stage, only one output point is produced per voxel. This point
isn't the quantized voxel coordinates; instead, it's the true average of the points within the voxel. The result
is more equalized amount of points near and far, and increased resolution (thanks to averaging) near-field.

When the voxfilter accumulates (averages) close points from different sources, we must store the information of all
sources involved, so that the free space can be traced from all correct sources.

VOXFILTER_STEP is now designed as a fixed CLOUD_MM*2, so that on each voxel, there are 8 different possible positions for
the point. Now, utilizing the points[][][] table indeces, the point position within the voxel can be described by 1+1+1 bits
for x,y,z respectively. So we can accumulate the average by just counting the cases when LSb = 1. If this count is over
cnt_of_points/2, then the resulting point has LSb=1 as well.

*/

typedef struct __attribute__((packed))
{
	uint16_t src_idxs[VOXFILTER_MAX_RAY_SOURCES]; // Zero-terminated list of indeces to cloud ray sources  (zero termination not required, if max length used)

	uint8_t cnt;

	// point LSb accumulation: increased whenever the point has LSb='1'
	// Obviously, max 256 points per voxel.
	uint8_t x;
	uint8_t y;
	uint8_t z;
} voxfilter_point_t;

// Remember to zero this struct out first.
typedef struct
{
	// Extra translation, so that the voxfilter can live in its own limited space instead of the larger submap span.
	int_fast32_t ref_x;
	int_fast32_t ref_y;
	int_fast32_t ref_z;
	voxfilter_point_t points[VOXFILTER_XS][VOXFILTER_YS][VOXFILTER_ZS];
} voxfilter_t;


#include "small_cloud.h"
#define MAX_REALTIME_N_POINTS (10*9600)
typedef struct
{
	int32_t n_points;
	small_cloud_t points[MAX_REALTIME_N_POINTS];
} realtime_cloud_t;


int save_cloud(cloud_t* cloud, int idx);
int load_cloud(cloud_t* cloud, int idx);
void rotate_cloud(cloud_t* cloud, double yaw);
void rotate_cloud_copy(cloud_t* cloud, cloud_t* out, double yaw);
void filter_cloud(cloud_t* cloud, cloud_t* out, int32_t transl_x, int32_t transl_y, int32_t transl_z);
void cloud_to_voxmap(cloud_t* cloud, int ref_x, int ref_y, int ref_z);

void tof_to_voxfilter_and_cloud(int is_narrow, uint16_t* ampldist, hw_pose_t pose, int sidx, int32_t ref_x, int32_t ref_y, int32_t ref_z,
	 voxfilter_t* voxfilter, int32_t voxfilter_ref_x, int32_t voxfilter_ref_y, int32_t voxfilter_ref_z, cloud_t* cloud, int voxfilter_threshold, int dist_ignore_threshold,
	 realtime_cloud_t* realtime, int rt_flag);

void voxfilter_to_cloud(voxfilter_t* voxfilter, cloud_t* cloud);

ALWAYS_INLINE void cloud_insert_point(cloud_t* cloud, int16_t sx, int16_t sy, int16_t sz, int32_t px, int32_t py, int32_t pz)
{
	if(cloud->n_points >= MAX_POINTS)
	{
		printf("WARNING: Ignoring point, cloud full.\n");
		return;
	}
	//assert(cloud->n_points >= 0);

	cloud->points[cloud->n_points].sx = sx;
	cloud->points[cloud->n_points].sy = sy;
	cloud->points[cloud->n_points].sz = sz;

	cloud->points[cloud->n_points].px = px;
	cloud->points[cloud->n_points].py = py;
	cloud->points[cloud->n_points].pz = pz;

	cloud->n_points++;
}

//#include "../robotboard2-fw/tof_process.h" // for sensor_mount_t
#if 1
typedef struct __attribute__((packed))
{
	int16_t mount_mode;             // mount position 1,2,3 or 4
	int16_t x_rel_robot;          // zero = robot origin. Positive = robot front (forward)
	int16_t y_rel_robot;          // zero = robot origin. Positive = to the right of the robot
	uint16_t ang_rel_robot;        // zero = robot forward direction. positive = ccw
	uint16_t vert_ang_rel_ground;  // zero = looks directly forward. positive = looks up. negative = looks down
	int16_t z_rel_ground;         // sensor height from the ground	
} sensor_mount_t;
#endif

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


void transform_cloud(cloud_t* cloud, int32_t transl_x, int32_t transl_y, int32_t transl_z, double yaw);
void transform_cloud_copy(cloud_t* cloud_in, cloud_t* cloud_out, int32_t transl_x, int32_t transl_y, int32_t transl_z, double yaw);

