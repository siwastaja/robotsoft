#pragma once

#include <inttypes.h>
#include "api_board_to_soft.h"
#include "misc.h"
#include "slam_config.h"


// Cloud resolution
#define CLOUD_MM       16
#define CLOUD_MM_SHIFT  4

/*
	Always little-endian.

	cloud_point_t usage options:

	src_idx, x, y, z
	src_idx, coords[3]
	raw_u16[4]
	raw_i16[4]
	raw_u64
*/


typedef union
{
	struct __attribute__((packed))
	{
		uint16_t src_idx;
		union
		{
			struct __attribute__((packed))
			{
				int16_t x;
				int16_t y;
				int16_t z;
			};

			int16_t coords[3];
		};
	};

	uint16_t raw_u16[4];

	int16_t raw_i16[4];

	uint64_t raw_u64;

} cloud_point_t;

#define CL_P(x_,y_,z_) ((cloud_point_t){{0,{{(x_),(y_),(z_)}}}})
#define CL_SRCP(s_,x_,y_,z_) ((cloud_point_t){{(s_),{{(x_),(y_),(z_)}}}})

#define CL_P_MM(x_,y_,z_) ((cloud_point_t){{0,{{(x_)>>CLOUD_MM_SHIFT,(y_)>>CLOUD_MM_SHIFT,(z_)>>CLOUD_MM_SHIFT}}}})
#define CL_SRCP_MM(s_,x_,y_,z_) ((cloud_point_t){{(s_),{{(x_)>>CLOUD_MM_SHIFT,(y_)>>CLOUD_MM_SHIFT,(z_)>>CLOUD_MM_SHIFT}}}})


ALWAYS_INLINE cloud_point_t cloud_point_minus(cloud_point_t a, cloud_point_t b)
{
	return CL_P(a.x-b.x, a.y-b.y, a.z-b.z);
}

ALWAYS_INLINE cloud_point_t cloud_point_plus(cloud_point_t a, cloud_point_t b)
{
	return CL_P(a.x+b.x, a.y+b.y, a.z+b.z);
}

ALWAYS_INLINE int_fast64_t cloud_point_dot(cloud_point_t a, cloud_point_t b)
{
	return (int_fast64_t)a.x*(int_fast64_t)b.x + (int_fast64_t)a.y*(int_fast64_t)b.y + (int_fast64_t)a.z*(int_fast64_t)b.z;
}

ALWAYS_INLINE cloud_point_t cloud_point_cross(cloud_point_t a, cloud_point_t b)
{
	return CL_P(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);
}

ALWAYS_INLINE int_fast64_t cloud_point_sqlen(cloud_point_t a)
{
	return sq(a.x)+sq(a.y)+sq(a.z);
}

ALWAYS_INLINE float cloud_point_len(cloud_point_t a)
{
	return sqrtf(cloud_point_sqlen(a));
}


ALWAYS_INLINE cloud_point_t transform_point(cloud_point_t p, int tx, int ty, int tz, float yaw)
{
	float cosa = cosf(yaw);
	float sina = sinf(yaw);

	cloud_point_t ret;

	ret.src_idx = p.src_idx;

	ret.x = ((float)p.x*cosa) - ((float)p.y*sina) + tx;
	ret.y = ((float)p.x*sina) + ((float)p.y*cosa) + ty;
	ret.z = p.z + tz;

	return ret;
}

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

// For initial allocations, similar reasoning (see above)
#define INITIAL_SRCS_ALLOC_LARGE (256)
#define INITIAL_SRCS_ALLOC_SMALL (64)

#define INITIAL_POSES_ALLOC_LARGE (64)
#define INITIAL_POSES_ALLOC_SMALL (16)


typedef struct __attribute__((packed))
{
	// points have limited numerical range; ref coords give an extra translation to the whole pointcloud
	int64_t ref_x;
	int64_t ref_y;
	int64_t ref_z;

	// Cloud can be rotated by just modifying the header
	uint32_t yaw;

	int32_t n_srcs;
	int32_t n_points;
	int32_t n_poses;
} cloud_meta_t;

#define MAX_SOURCES 4096 // 12 bits leaves 4 bits for flags

#define METASOURCE_FLAG 0x8000
//#define METASOURCE_LAST  0x4000

//#define U64_METASOURCE_START   0x8000000000000000ULL
//#define U64_METASOURCE_ANYLAST 0x4000400040004000ULL // if raw_u64 matches this, the metasource ends

typedef struct
{
	// This part can be stored to a file etc. directly:
	cloud_meta_t m;


	// The rest is for storing in memory. To write to a file or socket,
	// ignore the alloc_ counts and loop over the tables using the actual length parameters (n_*)
	int alloc_srcs;
	int alloc_points;
	int alloc_poses;

	// The sources.
	// Sources are the sensor locations; a line can be traced from source to the point.
	// This is handy, because such line was knowingly empty of obstacles, so can be used for filtration.
	// Storing source coordinates for each point would be huge waste of space and also processing, because
	// one sensor (located at one point at a time) sees thousands of points at once. So, all possible sources
	// are stored in one table, then indeces to this source table are stored in the points.
	// When multiple points seen from different sources are combined into one, the need to have single-point-
	// with-multiple-sources datatype emerges.
	//
	// Through a trick, a single point can have multiple sources.
	// The sources use the same cloud_point_t datatype as the points. Normally, with only one source needed,
	// the src_idx field is excess, and set to zero. But, when you need a special new multi-source source,
	// you can create a "meta-source" that refers to other sources.
	// These meta-sources have the first field (src_idx) flag space (4 highest bits) set non-zero;
	// When a meta-source starts, raw_u16[0] has METASOURCE_START (S in the example below) flag set
	// With the last index, raw_u16[i] has METASOURCE_LAST (L in the example below) flag set

	// Multisources are not allowed to refer to other multisources to avoid long chains of references

/*
	Example:

	table_i	srcid	x	y	z	comment
		raw[0]	raw[1]	raw[2]	raw[3]	

	1	0	123	123	123	Normal source 1
	2	0	1234	567	-42	Normal source 2
	...
	57	S|5	6	L|8	any	Metasource with 3 sources: 5,6,8
	...
	123	S|10	11	12	13	Metasource with 8 sources: 10,11,12,13,14,15,16,17
	124	14	15	16	L|17	Invalid item: using this index alone should never happen

	
*/


	cloud_point_t* srcs;

	// The points.
	cloud_point_t* points;
	hw_pose_t* poses; // robot trajectory

} cloud_t;

// cloud_t instances are small, can be stored in stack, arrays, etc. efficiently.
// remember to init_cloud and free_cloud, though.

#define CLOUD_INIT_SMALL (1<<0)

/*
	init_cloud: the basic initialization
	New cloud must be created with this function. All-zeroes is no proper initialization.
	Flags:
		CLOUD_INIT_SMALL - initially reserve less memory.

	Example:

	cloud_t my_cloud;
	init_cloud(&my_cloud, 0);
	cloud_add_point(&my_cloud, CL_P_MM(123, 456, 789));
	free_cloud(&my_cloud);
*/
void init_cloud(cloud_t* cloud, int flags);

/*
	alloc_cloud: alternative initialization
	You supply a pointer to an otherwise-uninitialized cloud, but you have
	set the m.n_* fields. This function does the rest of the init: allocates for
	this exact amount of space.

	Example:

	cloud_t my_cloud;
	my_cloud.m.n_srcs = read_from_file_header();
	my_cloud.m.n_points = read_from_file_header();
	my_cloud.m.n_poses = read_from_file_header();
	alloc_cloud(&my_cloud);
	// read from file directly, write to memory pointed by my_cloud.points, .srcs, .poses	
*/

void alloc_cloud(cloud_t* cloud);


void free_cloud(cloud_t* cloud);

ALWAYS_INLINE int cloud_add_source(cloud_t* cloud, cloud_point_t p)
{
	if(cloud->m.n_srcs >= cloud->alloc_srcs)
	{
		cloud->alloc_srcs <<= 1;
		cloud->srcs = realloc(cloud->srcs, cloud->alloc_srcs * sizeof (cloud_point_t));
		assert(cloud->srcs);
	}

	cloud->srcs[cloud->m.n_srcs] = p;

	return cloud->m.n_srcs++;
}

// Find existing (close enough) source, use it. If suitable source is not found, a new source is added. Returns the source idx.
// For now, linear search is used because the number of sources is fairly low, and there are usually thousands of points per source.
// Never fails.
ALWAYS_INLINE int cloud_find_source(cloud_t* cloud, cloud_point_t p)
{
	int_fast32_t closest = INT_FAST32_MAX;
	int closest_i;
	for(int i=0; i<cloud->m.n_srcs; i++)
	{
/*
		if(cloud->srcs[i].raw_u64 & U64_METASOURCE_START)
		{
			while(!(cloud->srcs[i].raw_u64 & U64_METASOURCE_ANYLAST)) // Metasource does not end here
			{
				i++; // Skip source indeces until it ends.
				assert(i < cloud->m.n_srcs);
			}
		}
*/
		if(cloud->srcs[i].src_idx & METASOURCE_FLAG)
			continue;

		int_fast32_t sqdist = sq(p.x - cloud->srcs[i].x) + sq(p.y - cloud->srcs[i].y) + sq(p.z - cloud->srcs[i].z);
		if(sqdist < closest)
		{
			closest = sqdist;
			closest_i = i;
		}
	}

	if(closest <= sq(SOURCE_COMBINE_THRESHOLD))
		return closest_i;

	return cloud_add_source(cloud, p);
}

ALWAYS_INLINE void cloud_add_point(cloud_t* cloud, cloud_point_t p)
{
	if(UNLIKELY(cloud->m.n_points >= cloud->alloc_points))
	{
		cloud->alloc_points <<= 1;
		cloud->points = realloc(cloud->points, cloud->alloc_points * sizeof (cloud_point_t));
		assert(cloud->points);
	}

	cloud->points[cloud->m.n_points++] = p;
}

// Allocates for multiple points at once. After calling this, you are free to call
// cloud_fast_add_point for maximum number of cnt times.
ALWAYS_INLINE void cloud_prepare_fast_add_points(cloud_t* cloud, int cnt)
{
	int change = 0;
	while(cloud->m.n_points + cnt >= cloud->alloc_points)
	{
		cloud->alloc_points <<= 1;
		change = 1;
	}

	if(change)
	{
		cloud->points = realloc(cloud->points, cloud->alloc_points * sizeof (cloud_point_t));
		assert(cloud->points);
	}
}

// No range checking, no allocation expansion:
// Call cloud_prepare_fast_add_points() first with the maximum possible number of points you are going to add with this function.
ALWAYS_INLINE void cloud_fast_add_point(cloud_t* cloud, cloud_point_t p)
{
	cloud->points[cloud->m.n_points++] = p;
}


ALWAYS_INLINE void cloud_add_pose(cloud_t* cloud, hw_pose_t p)
{
	if(cloud->m.n_poses >= cloud->alloc_poses)
	{
		cloud->alloc_poses <<= 1;
		cloud->poses = realloc(cloud->poses, cloud->alloc_poses * sizeof (hw_pose_t));
		assert(cloud->poses);
	}

	cloud->poses[cloud->m.n_poses++] = p;
}

#if 0
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
#define VOXFILTER_STEP 2

/*
Voxfilter: Because sensors output points at constant angular resolution, nearby surfaces produce
a huge clumps of points, as opposed to faraway surfaces. Voxfilter is a simple, first-order filter
that reduces the amount of data before it is accumulated in memory. Instead of storing points directly
to the pointcloud, it is stored using the voxfilter_insert_point interface. If the point fits inside the
(fairly small) voxfilter area of voxels, i.e., it's in the dense near field sensing area, it is averaged with
the other points hitting the same voxel. If the point is outside the voxelized area, point is added to the pointcloud
directly instead. After enough scans are processed (e.g., the robot has moved enough that the small voxfilter
range is running out, or at latest, when we want to output the point cloud), the points contained in the voxfilter
are added to the point cloud at once. At this stage, only one output point is produced per voxel. This point
isn't the quantized voxel coordinates; instead, it's the true average of the points within the voxel. This results in
more equalized number of points near and far, and increased resolution (thanks to averaging) near-field.

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
#endif

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

void tof_to_cloud(int is_narrow, int setnum, tof_slam_set_t* tss, int32_t ref_x, int32_t ref_y, int32_t ref_z,
	 int do_filter, cloud_t* cloud, int dist_ignore_threshold);


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

// translates cloud in and its sources to the reference of out.
// Tries to find matching sources in in, creates new sources if necessary, changes the source indeces.
void cat_cloud(cloud_t * const restrict out, const cloud_t * const restrict in);

// ... but keep the allocations and meta fields
void cloud_remove_points(cloud_t* cloud);


