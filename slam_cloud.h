#pragma once

#include <inttypes.h>
#include <assert.h>
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



#define CLOUD_GENERAL_SRC 255       // Source index meaning: "no source available"

#define CLOUD_MAX_SRCS 254
#define POINT_MAX_SRCS 9 // resulting 16-byte point. Abs. max 15, number of sources has to fit in 4 bits


typedef union
{
	struct __attribute__((packed))
	{
		uint8_t  removed  : 1;
		uint8_t  reserved1 : 1;
		uint8_t  reserved2 : 2;
		uint8_t  n_srcs : 4;
		uint8_t  srcs[POINT_MAX_SRCS];
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

	uint8_t raw_u8[16];
	uint16_t raw_u16[8];
	int16_t raw_i16[8];
	uint64_t raw_u64[2];
} cloud_point_t;


typedef struct __attribute__((packed))
{
	uint8_t reserved;
	uint8_t src;
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
} freevect_point_t;



#define CL_P(x_,y_,z_) ((cloud_point_t){{0,0,0,0,{0},{{(x_),(y_),(z_)}}}})
#define CL_SRCP(s_,x_,y_,z_) ((cloud_point_t){{0,0,0,1,{(s_),0,0,0,0,0,0,0},{{(x_),(y_),(z_)}}}})

#define CL_P_MM(x_,y_,z_) ((cloud_point_t){{0,0,0,0,{0},{{(x_)>>CLOUD_MM_SHIFT,(y_)>>CLOUD_MM_SHIFT,(z_)>>CLOUD_MM_SHIFT}}}})
#define CL_SRCP_MM(s_,x_,y_,z_) ((cloud_point_t){{0,0,0,1,{(s_),0,0,0,0,0,0,0},{{(x_)>>CLOUD_MM_SHIFT,(y_)>>CLOUD_MM_SHIFT,(z_)>>CLOUD_MM_SHIFT}}}})


#define CL_FREEVECT_MM(s_, x_,y_,z_) ((freevect_point_t){0,(s_),{{(x_)>>CLOUD_MM_SHIFT,(y_)>>CLOUD_MM_SHIFT,(z_)>>CLOUD_MM_SHIFT}}})


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

// I trust the compiler can optimize the cosf(),sinf() out of the loop if this always_inline function is "called" inside a loop.
ALWAYS_INLINE cloud_point_t transform_point(cloud_point_t p, int tx, int ty, int tz, float yaw)
{
	cloud_point_t ret;

	ret = p; // copy source indeces and flags

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


ALWAYS_INLINE freevect_point_t transform_freevect(freevect_point_t p, int tx, int ty, int tz, float yaw)
{
	freevect_point_t ret;

	ret = p; // copy src index, reserved field

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

	// Cloud can be rotated by just modifying the header. Well, not yet, but maybe some time...
	uint32_t yaw;

	int32_t n_srcs;
	int32_t n_points;
	int32_t n_poses;
	int32_t n_freevects;
} cloud_meta_t;



typedef struct
{
	// This part can be stored to a file, copied to socket, etc. directly:
	cloud_meta_t m;


	// The rest is for storing in memory. To write to a file or socket,
	// ignore the alloc_ counts and loop over the tables using the actual length parameters (m.n_*)
	//int alloc_srcs;
	int alloc_points;
	int alloc_poses;
	int alloc_freevects;

	// The sources.
	// Sources are the sensor locations; a line can be traced from source to the point.
	// This is handy, because such line was knowingly empty of obstacles, so can be used for filtration.
	// Storing source coordinates for each point would be huge waste of space and also processing, because
	// one sensor (located at one point at a time) sees thousands of points at once. So, all possible sources
	// are stored in one table, then indeces to this source table are stored in the points.
	// Room for optimization: use another datatype for srcs[], they are x,y,z only.

	// Is 256 srcs running out?
	//   * Combine closeby sources. Having gazillion sources is slow anyway (tracing the empty space)
	//   * Do not construct massive point clouds using this cloud datatype, it's not meant for that. Even the coordinates are limited range.

	cloud_point_t srcs[CLOUD_MAX_SRCS];

	// The points.
	cloud_point_t* points;
	hw_pose_t* poses; // robot trajectory
	freevect_point_t* freevects;

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

// Add source(s) of in to the source list of out, not adding existing indeces
ALWAYS_INLINE void point_add_src_idxs(cloud_point_t *out, cloud_point_t *in)
{
	assert(in->n_srcs <= POINT_MAX_SRCS);
	assert(out->n_srcs <= POINT_MAX_SRCS);
	for(int i=0; i<in->n_srcs; i++)
	{
		if(out->n_srcs >= POINT_MAX_SRCS)
		{
			// Source list is full, can't add
			break;
		}

		// Find existing to see if adding is unnecessary
		int o;
		for(o=0; o<out->n_srcs; o++)
		{
			if(in->srcs[i] == out->srcs[o])
				goto IDX_EXISTS;
		}
		assert(o == out->n_srcs);

		// Add.
		out->srcs[out->n_srcs++] = in->srcs[i];

		IDX_EXISTS:;
	}

	assert(out->n_srcs <= POINT_MAX_SRCS);

}

#if 0
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
#endif

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

ALWAYS_INLINE void cloud_add_freevect(cloud_t* cloud, freevect_point_t p)
{
//	printf("add_freevect n=%d, alloc=%d\n", cloud->m.n_freevects, cloud->alloc_freevects);
	if(UNLIKELY(cloud->m.n_freevects >= cloud->alloc_freevects))
	{
		cloud->alloc_freevects <<= 1;
		cloud->freevects = realloc(cloud->freevects, cloud->alloc_freevects * sizeof (freevect_point_t));
		assert(cloud->freevects);
	}

	cloud->freevects[cloud->m.n_freevects++] = p;
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
void cloud_to_voxmap(cloud_t* cloud, int ref_x, int ref_y, int ref_z);

void tof_to_cloud(int is_narrow, int setnum, tof_slam_set_t* tss, int32_t ref_x, int32_t ref_y, int32_t ref_z,
	 int do_filter, cloud_t* cloud, int dist_ignore_threshold);


void cloud_copy_all_but_points(cloud_t* restrict out, cloud_t* restrict cloud);
void freespace_filter_cloud(cloud_t* out, cloud_t* cloud);



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

// Translates all cloud points, sources and freevects.
// Note that the cloud operates on limited numerical range. Use this function to re-reference the cloud, or similar.
// If you just want to move the cloud around, think about adjusting the cloud.m.ref_* fields.
void translate_cloud(cloud_t * cloud, int tx, int ty, int tz);

// translates cloud in and its sources to the reference of out.
// Tries to find matching sources in in, creates new sources if necessary, changes the source indeces.
void cat_cloud(cloud_t * const restrict out, const cloud_t * const restrict in);

// ... but keep the allocations and meta fields
void cloud_remove_points(cloud_t* cloud);
void cloud_remove_points_keep_sources(cloud_t* cloud);


int cloud_is_init(cloud_t* cloud);
void cloud_print_sources(cloud_t* cloud, int analyze);
