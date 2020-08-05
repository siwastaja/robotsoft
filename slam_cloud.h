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

	cloud_point_t usage options, pick any of these ways to access:

	flags, src_idx, x, y, z     example: p.y
	flags, src_idx, coords[3]   example: p.coords[1]
	raw_u8[8]
	raw_u16[4]
	raw_i16[4]
	raw_u64
*/


// If set, src_idx refers to multisrcs[] of the cloud instead of srcs[].
#define CLOUD_FLAG_MULTISRC   (1<<0)
#define CLOUD_FLAG_REMOVED    (1<<1)


typedef union
{
	struct __attribute__((packed))
	{
		uint8_t  flags;
		uint8_t  src_idx;
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

	uint16_t raw_u8[8];
	uint16_t raw_u16[4];
	int16_t raw_i16[4];
	uint64_t raw_u64;
} cloud_point_t;


#define CLOUD_GENERAL_SRC 255       // Source index meaning: "no source available"

#define CLOUD_MAX_SRCS 255
#define CLOUD_MAX_MULTISRCS 255
#define CLOUD_MAX_SRCS_IN_MULTISRC 15

typedef struct __attribute__((packed))
{
	uint8_t n; // below 2 is illegal, such multisources wouldn't make sense. above CLOUD_MAX_SRCS_IN_MULTISRC is illegal.
	uint8_t idxs[CLOUD_MAX_SRCS_IN_MULTISRC]; // is kept sorted in increasing order
} multisource_t;


#define CL_P(x_,y_,z_) ((cloud_point_t){{0,0,{{(x_),(y_),(z_)}}}})
#define CL_SRCP(s_,x_,y_,z_) ((cloud_point_t){{0,(s_),{{(x_),(y_),(z_)}}}})
#define CL_FSRCP(f_, s_,x_,y_,z_) ((cloud_point_t){{(f_),(s_),{{(x_),(y_),(z_)}}}})

#define CL_P_MM(x_,y_,z_) ((cloud_point_t){{0,0,{{(x_)>>CLOUD_MM_SHIFT,(y_)>>CLOUD_MM_SHIFT,(z_)>>CLOUD_MM_SHIFT}}}})
#define CL_SRCP_MM(s_,x_,y_,z_) ((cloud_point_t){{0,(s_),{{(x_)>>CLOUD_MM_SHIFT,(y_)>>CLOUD_MM_SHIFT,(z_)>>CLOUD_MM_SHIFT}}}})
#define CL_FSRCP_MM(s_,x_,y_,z_) ((cloud_point_t){{(f_),(s_),{{(x_)>>CLOUD_MM_SHIFT,(y_)>>CLOUD_MM_SHIFT,(z_)>>CLOUD_MM_SHIFT}}}})


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
	cloud_point_t ret;

	ret.src_idx = p.src_idx;

	// Often we just translate using this function, but don't want stupid rounding errors when we can avoid them.
	// Translation is lossless. Rotation is lossy.
	if(yaw > -DEGTORAD(0.1) && yaw < DEGTORAD(0.1))
	{
		ret.x = p.x + tx;
		ret.y = p.y + ty;
		ret.z = p.z + tz;
	}
	else
	{
		float cosa = cosf(yaw);
		float sina = sinf(yaw);

		ret.x = ((float)p.x*cosa) - ((float)p.y*sina) + tx;
		ret.y = ((float)p.x*sina) + ((float)p.y*cosa) + ty;
		ret.z = p.z + tz;
	}

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
	int32_t n_multisrcs;
	int32_t n_points;
	int32_t n_poses;
} cloud_meta_t;



typedef struct
{
	// This part can be stored to a file etc. directly:
	cloud_meta_t m;


	// The rest is for storing in memory. To write to a file or socket,
	// ignore the alloc_ counts and loop over the tables using the actual length parameters (n_*)
	//int alloc_srcs;
	int alloc_points;
	int alloc_poses;

	// The sources.
	// Sources are the sensor locations; a line can be traced from source to the point.
	// This is handy, because such line was knowingly empty of obstacles, so can be used for filtration.
	// Storing source coordinates for each point would be huge waste of space and also processing, because
	// one sensor (located at one point at a time) sees thousands of points at once. So, all possible sources
	// are stored in one table, then indeces to this source table are stored in the points.

	// When multiple points seen from different sources are combined into one, the need to have single-point-
	// with-multiple-sources datatype emerges. Such points use CLOUD_FLAG_MULTISRC, and the src_idx refers to
	// multisrcs[] instead. multisrcs[] does not hold coordinates, but refers to srcs[] instead; this is because
	// almost always when having multisources, all of those sources exist anyway for some single points, as well,
	// so only storing one byte to refer to the srcs[] saves space.
	// Because we want to refer to the sources by one byte (so that multisources are efficient), we limit the number
	// of sources to 256. Because multisrcs have their own index space, we have another 256 of them.
	// Static allocation is OK for now, it's such low number compared to the typical number of points.
	// Is 256 srcs running out?
	//   * Combine closeby sources. Having gazillion sources is slow anyway (tracing the empty space)
	//   * Do not construct massive point clouds using this cloud datatype, it's not meant for that. Even the coordinates are limited range.

	cloud_point_t srcs[CLOUD_MAX_SRCS];
	multisource_t multisrcs[CLOUD_MAX_MULTISRCS];

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
	/*
	if(cloud->m.n_srcs >= cloud->alloc_srcs)
	{
		cloud->alloc_srcs <<= 1;
		cloud->srcs = realloc(cloud->srcs, cloud->alloc_srcs * sizeof (cloud_point_t));
		assert(cloud->srcs);
	}
	*/

	assert(cloud->m.n_srcs < CLOUD_MAX_SRCS);

	cloud->srcs[cloud->m.n_srcs] = p;

	return cloud->m.n_srcs++;
}

// Find existing (close enough) source. If suitable source is not found, a new source is added. Returns the source idx of the found or added source.
// For now, linear search is used because the number of sources is fairly low, and there are usually thousands of points per source for which this is called once.
// Never fails.
ALWAYS_INLINE int cloud_find_source(cloud_t* cloud, cloud_point_t p)
{
	int_fast32_t closest = INT_FAST32_MAX;
	int closest_i;
	for(int i=0; i<cloud->m.n_srcs; i++)
	{
		int_fast32_t sqdist = sq(p.x - cloud->srcs[i].x) + sq(p.y - cloud->srcs[i].y) + sq(p.z - cloud->srcs[i].z);
		if(sqdist < closest)
		{
			closest = sqdist;
			closest_i = i;
		}
	}

	if(closest <= sq(SOURCE_COMBINE_THRESHOLD))
		return closest_i;

	if(cloud->m.n_srcs == CLOUD_MAX_SRCS)
	{
		printf("WARNING: cloud_find_source(): cannot create new source, sources full, returning general no-source index\n");
		return CLOUD_GENERAL_SRC;
	}
	else
		return cloud_add_source(cloud, p);
}


ALWAYS_INLINE int cloud_add_multisource(cloud_t* cloud, multisource_t ms)
{
	assert(cloud->m.n_multisrcs < CLOUD_MAX_MULTISRCS);

	cloud->multisrcs[cloud->m.n_multisrcs] = ms;

	return cloud->m.n_multisrcs++;
}

ALWAYS_INLINE int cloud_find_multisource(cloud_t* cloud, multisource_t ms)
{
	assert(ms.n > 1 && ms.n < CLOUD_MAX_SRCS_IN_MULTISRC);
	//printf("cloud_find_multisource(): ms.n=%d\n", ms.n);
	for(int i=0; i<cloud->m.n_multisrcs; i++)
	{
		//printf("i%d, n=%d\n", i, cloud->multisrcs[i].n);
		if(ms.n == cloud->multisrcs[i].n)
		{
			for(int o=0; o<ms.n; o++)
			{
				//printf("o%d, idxs: %d vs %d\n", o, cloud->multisrcs[i].idxs[o], ms.idxs[o]);

				if(o>0) assert(ms.idxs[o] > ms.idxs[o-1]);
				if(o>0) assert(cloud->multisrcs[i].idxs[o] > cloud->multisrcs[i].idxs[o-1]);

/*
				if(o>0 && cloud->multisrcs[i].idxs[o] <= cloud->multisrcs[i].idxs[o-1]) 
				{
					printf("WRONG ORDER\n");
					exit(1);
				}
*/
				if(cloud->multisrcs[i].idxs[o] != ms.idxs[o]) // no match
					goto BREAK_NO_MATCH_SO_FAR;
			}
			// match!
			return i;
			BREAK_NO_MATCH_SO_FAR:;
		}
	}

	// no match
	if(cloud->m.n_multisrcs == CLOUD_MAX_MULTISRCS)
	{
		printf("WARNING: cloud_find_multisource(): cannot create new multisource, multisources full, returning general no-source index\n");
		return CLOUD_GENERAL_SRC;
	}
	else
		return cloud_add_multisource(cloud, ms);

}

ALWAYS_INLINE void cloud_sort_multisource(multisource_t* ms)
{
	for(int i=1; i<ms->n; i++)
	{
		uint8_t t = ms->idxs[i];
		int j;
		for(j=i; j > 0 && ms->idxs[j-1] > t; j--)
			ms->idxs[j] = ms->idxs[j-1];
		ms->idxs[j] = t;
	}	
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
void cloud_remove_points_keep_sources(cloud_t* cloud);


int cloud_is_init(cloud_t* cloud);
void cloud_print_sources(cloud_t* cloud, int analyze);
