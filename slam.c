#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#define DEFINE_API_VARIABLES
#include "api_board_to_soft.h"
#undef DEFINE_API_VARIABLES

#include "voxmap.h"
#include "voxmap_memdisk.h"


#include "sin_lut.c"
#include "geotables.h"
#include "b2s_prints.c"

#define N_SENSORS 10

#define RADTODEG(x) ((x)*(360.0/(2.0*M_PI)))
#define DEGTORAD(x) ((x)*((2.0*M_PI)/360.0))
#define DEGTOANG16(x)  ((uint16_t)((float)(x)/(360.0)*65536.0))

#define DIST_UNDEREXP 0

typedef struct
{
	int32_t mount_mode;             // mount position 1,2,3 or 4
	int32_t x_rel_robot;          // zero = robot origin. Positive = robot front (forward)
	int32_t y_rel_robot;          // zero = robot origin. Positive = to the right of the robot
	uint16_t ang_rel_robot;        // zero = robot forward direction. positive = ccw
	uint16_t vert_ang_rel_ground;  // zero = looks directly forward. positive = looks up. negative = looks down
	int32_t z_rel_ground;         // sensor height from the ground	
} sensor_mount_t;


/*
	Sensor mount position 1:
	 _ _
	| | |
	| |L|
	|O|L|
	| |L|
	|_|_|  (front view)

	Sensor mount position 2:
	 _ _
	| | |
	|L| |
	|L|O|
	|L| |
	|_|_|  (front view)

	Sensor mount position 3:

	-------------
	|  L  L  L  |
	-------------
	|     O     |
	-------------

	Sensor mount position 4:

	-------------
	|     O     |
	-------------
	|  L  L  L  |
	-------------
*/



// const
sensor_mount_t sensor_mounts[N_SENSORS] =
{          //      mountmode    x     y       hor ang           ver ang      height    
 /*0:                */ { 0,     0,     0, DEGTOANG16(       0), DEGTOANG16( 2),         300 },

 /*1:                */ { 1,   130,   103, DEGTOANG16(    24.4), DEGTOANG16( 4.4),       310  }, // -1
 /*2:                */ { 2,  -235,   215, DEGTOANG16(    66.4), DEGTOANG16( 1.4),       310  }, // -1
 /*3:                */ { 2,  -415,   215, DEGTOANG16(    93.5), DEGTOANG16( 1.9),       310  }, // -1
 /*4:                */ { 2,  -522,   103, DEGTOANG16(   157.4), DEGTOANG16( 3.9),       280  }, // -1
 /*5:                */ { 2,  -522,   -35, DEGTOANG16(   176.0), DEGTOANG16( 4.9),       290  }, // -1
 /*6:                */ { 1,  -522,  -103, DEGTOANG16(   206.0), DEGTOANG16( 4.4),       290  }, // -1
 /*7:                */ { 1,  -415,  -215, DEGTOANG16(   271.5), DEGTOANG16( 2.4),       280  }, // -1
 /*8:                */ { 1,  -235,  -215, DEGTOANG16(   294.9), DEGTOANG16( 4.4),       300  }, // -1
 /*9:                */ { 2,   130,  -103, DEGTOANG16(   334.9), DEGTOANG16( -0.9),      320  }  // 0
};

#undef Z_STEP
#undef BASE_Z
#undef MAX_Z


#define XS 128
#define YS 128
#define ZS 128
#define XY_STEP 64
#define Z_STEP 64


#define BASE_Z -400
#define MAX_Z ((Z_STEP*ZS + BASE_Z)-1)

#define sq(x) ((x)*(x))
#define abso(x) (((x)<0)?(-(x)):(x))

typedef struct
{
	int start_idx;
	int end_idx;
	int32_t avg_x;
	int32_t avg_y;
	int32_t avg_z;
} subsubmap_meta_t;

#define MAX_SUBSUBMAPS 16

typedef struct
{
	int start_idx;
	int end_idx;

	int32_t min_x;
	int32_t avg_x;
	int32_t max_x;

	int32_t min_y;
	int32_t avg_y;
	int32_t max_y;

	int32_t min_z;
	int32_t avg_z;
	int32_t max_z;

	int n_subsubmaps;
	subsubmap_meta_t subsubmaps[MAX_SUBSUBMAPS];

} submap_meta_t;


#define VOXFILTER_N_SCANS 6

#define MAX_SUBMAPS 100 //16384

int process_file(char* fname, tof_slam_set_t** tss);

#define FIRST_SIDX 1

static double translation_vect_len(hw_pose_t pose1, hw_pose_t pose2)
{
	int dx = pose1.x - pose2.x;
	int dy = pose1.y - pose2.y;
	int dz = pose1.z - pose2.z;

	return sqrt(sq(dx)+sq(dy)+sq(dz));
}

static int32_t max_rotation(hw_pose_t pose1, hw_pose_t pose2)
{
	int32_t dyaw = abso(pose1.ang - pose2.ang);
	int32_t dpitch = abso(pose1.pitch - pose2.pitch);
	int32_t droll = abso(pose1.roll - pose2.roll);

	int32_t biggest = dyaw;
	if(dpitch > biggest) biggest = dpitch;
	if(droll > biggest) biggest = droll;

	return biggest;
}

int start_i = 500;
int end_i = 12000;

#define MAX_FIRSTSIDX_POSES 65536
typedef struct __attribute__((packed))
{
	uint32_t idx;
	hw_pose_t pose;
} firstsidx_pose_t;

int extract_firstsidx_poses(firstsidx_pose_t* out, int* n_out)
{
	printf(__func__);
	printf("\n");
	int n = 0;
	for(int i=start_i; i<end_i; i++)
	{
		char fname[1024];
		sprintf(fname, "/home/hrst/robotsoft/tsellari/trace%08d.rb2", i);
		tof_slam_set_t* tss;
//		printf("&tss = %lx\n", (uint64_t)&tss);
		if(process_file(fname, &tss) == 0) // tof_slam_set record succesfully extracted
		{
//			printf("OK, sidx=%d\n", tss->sidx);
			if(tss->sidx == FIRST_SIDX)
			{
				//printf("OK!\n");
				out[n].pose = tss->sets[0].pose;
				out[n].idx = i;
				n++;
				if(n >= MAX_FIRSTSIDX_POSES)
					break;
			}
		}
	}

	*n_out = n;
	return 0;
}


#define TRACK_MIN(invar_, minvar_) do{if((invar_) < (minvar_)) (minvar_) = (invar_);}while(0)
#define TRACK_MAX(invar_, maxvar_) do{if((invar_) > (maxvar_)) (maxvar_) = (invar_);}while(0)

void group_submaps(submap_meta_t* smm_out, int* n_smm_out, firstsidx_pose_t* fsp, int n_fsp)
{
	if(n_fsp < 1)
	{
		printf("Error: n_fsp < 1\n");

	}
	printf(__func__);
	printf(" n_fsp=%d\n", n_fsp);

	// Count limit of full sensor rotations:
	int count_limit = 10*VOXFILTER_N_SCANS;
	// When max_x-min_x exceeds the limit, new submap group is initiated (at the next FIRST_SIDX edge)
	int dx_limit = 4000;
	int dy_limit = 4000;
	int dz_limit = 2000;

	int overlap_dx_limit = 2500;
	int overlap_dy_limit = 2500;
	int overlap_dz_limit = 1200;

	int start_fsp = 0;

	int n_smm = 0;
	while(1)
	{
		// Current group starts at start_fsp
		// Find the end for this group, and potentially a start for the overlapped next group:
		int cur_fsp = start_fsp+1;
		int next_start_fsp = -1;

		//printf("n_smm=%5d start_fsp=%d (seq id=%d)\n", n_smm, start_fsp, fsp[start_fsp].idx);

		smm_out[n_smm].start_idx = fsp[start_fsp].idx;

		int32_t min_x = INT32_MAX;
		int32_t min_y = INT32_MAX;
		int32_t min_z = INT32_MAX;
		int32_t max_x = INT32_MIN;
		int32_t max_y = INT32_MIN;
		int32_t max_z = INT32_MIN;
		int64_t avg_x = 0;
		int64_t avg_y = 0;
		int64_t avg_z = 0;
		int avg_n = 0;

		int64_t subsub_avg_x = 0;
		int64_t subsub_avg_y = 0;
		int64_t subsub_avg_z = 0;
		int subsub_avg_n = 0;

		smm_out[n_smm].n_subsubmaps = 0;
		smm_out[n_smm].subsubmaps[0].start_idx = fsp[start_fsp].idx;

		while(1)
		{
			assert(cur_fsp < n_fsp);

			TRACK_MIN(fsp[cur_fsp].pose.x, min_x);
			TRACK_MIN(fsp[cur_fsp].pose.y, min_y);
			TRACK_MIN(fsp[cur_fsp].pose.z, min_z);

			TRACK_MAX(fsp[cur_fsp].pose.x, max_x);
			TRACK_MAX(fsp[cur_fsp].pose.y, max_y);
			TRACK_MAX(fsp[cur_fsp].pose.z, max_z);

			avg_x += fsp[cur_fsp].pose.x;
			avg_y += fsp[cur_fsp].pose.y;
			avg_z += fsp[cur_fsp].pose.z;
			avg_n++;

			subsub_avg_x += fsp[cur_fsp].pose.x;
			subsub_avg_y += fsp[cur_fsp].pose.y;
			subsub_avg_z += fsp[cur_fsp].pose.z;
			subsub_avg_n++;

			if(cur_fsp == n_fsp-1)
			{
				// Input data ended, work done.
				//printf("   input data end at n_smm=%d, cur_fsp=%d (seq id=%d)\n", n_smm, cur_fsp, fsp[cur_fsp].idx);
				smm_out[n_smm].end_idx = fsp[cur_fsp].idx;
				smm_out[n_smm].min_x = min_x;
				smm_out[n_smm].min_y = min_y;
				smm_out[n_smm].min_z = min_z;
				smm_out[n_smm].max_x = max_x;
				smm_out[n_smm].max_y = max_y;
				smm_out[n_smm].max_z = max_z;
				smm_out[n_smm].avg_x = avg_x/avg_n;
				smm_out[n_smm].avg_y = avg_y/avg_n;
				smm_out[n_smm].avg_z = avg_z/avg_n;

				smm_out[n_smm].subsubmaps[smm_out[n_smm].n_subsubmaps].end_idx = fsp[cur_fsp].idx;
				smm_out[n_smm].subsubmaps[smm_out[n_smm].n_subsubmaps].avg_x = subsub_avg_x / subsub_avg_n;
				smm_out[n_smm].subsubmaps[smm_out[n_smm].n_subsubmaps].avg_y = subsub_avg_y / subsub_avg_n;
				smm_out[n_smm].subsubmaps[smm_out[n_smm].n_subsubmaps].avg_z = subsub_avg_z / subsub_avg_n;
				smm_out[n_smm].n_subsubmaps++;

				n_smm++;
				break;
			}

//			int dx = fsp[cur_fsp].pose.x - fsp[start_fsp].pose.x;
//			int dy = fsp[cur_fsp].pose.y - fsp[start_fsp].pose.y;
//			int dz = fsp[cur_fsp].pose.z - fsp[start_fsp].pose.z;

			int dx = max_x - min_x;
			int dy = max_y - min_y;
			int dz = max_z - min_z;

			if(next_start_fsp == -1 && (abso(dx) > overlap_dx_limit || abso(dy) > overlap_dy_limit || abso(dz) > overlap_dz_limit))
			{
				next_start_fsp = cur_fsp;
			}

			if((cur_fsp-start_fsp)%VOXFILTER_N_SCANS == 0) // Also includes the case when the count limit is reached
			{
				smm_out[n_smm].subsubmaps[smm_out[n_smm].n_subsubmaps].end_idx = fsp[cur_fsp].idx;
				assert(subsub_avg_n == VOXFILTER_N_SCANS);

				smm_out[n_smm].subsubmaps[smm_out[n_smm].n_subsubmaps].avg_x = subsub_avg_x / subsub_avg_n;
				smm_out[n_smm].subsubmaps[smm_out[n_smm].n_subsubmaps].avg_y = subsub_avg_y / subsub_avg_n;
				smm_out[n_smm].subsubmaps[smm_out[n_smm].n_subsubmaps].avg_z = subsub_avg_z / subsub_avg_n;

				smm_out[n_smm].n_subsubmaps++;
				smm_out[n_smm].subsubmaps[smm_out[n_smm].n_subsubmaps].start_idx = fsp[cur_fsp].idx; // There will be one "excess" start_idx, doesn't matter.
				subsub_avg_x = 0;
				subsub_avg_y = 0;
				subsub_avg_z = 0;
				subsub_avg_n = 0;
			}

			// Let the dx,dy, or dz overshoot a bit, instead ensure that the submap is evenly divisible into subsubmaps
			// count limit can be compared as-is, because it's divisible by VOXFILTER_N_SCANS
			if(((cur_fsp-start_fsp)%VOXFILTER_N_SCANS == 0 && (abso(dx) > dx_limit || abso(dy) > dy_limit || abso(dz) > dz_limit))
			    || (cur_fsp >= start_fsp + count_limit))
			{
				smm_out[n_smm].end_idx = fsp[cur_fsp].idx;

				smm_out[n_smm].min_x = min_x;
				smm_out[n_smm].min_y = min_y;
				smm_out[n_smm].min_z = min_z;
				smm_out[n_smm].max_x = max_x;
				smm_out[n_smm].max_y = max_y;
				smm_out[n_smm].max_z = max_z;
				smm_out[n_smm].avg_x = avg_x/avg_n;
				smm_out[n_smm].avg_y = avg_y/avg_n;
				smm_out[n_smm].avg_z = avg_z/avg_n;

				n_smm++;
				//printf("   limit reached\n");
				break;
			}

			cur_fsp++;
		}

		if(next_start_fsp == -1)
		{
			start_fsp = cur_fsp;
		}
		else // Want to overlap: jump back
		{
			start_fsp = next_start_fsp;
		}

		if(start_fsp >= n_fsp-1)
		{
			// Input data ended, work done.
			break;
		}

		if(n_smm >= MAX_SUBMAPS)
		{
			printf("Warning: submap limit exceeded\n");
			break;
		}

	}

	*n_smm_out = n_smm;

}


typedef struct
{
	int32_t src_seq_id; // Sequence ID gives us both pose and sensor idx, for exact source x,y,z for raytracing empty space. 0 is an invalid id

	int32_t cnt;
	int32_t x;
	int32_t y;
	int32_t z;
} voxfilter_point_t;

// 20MB of memory with these, coverage 4.1x4.1x2.0 m
// Outside of the filter, all points go through as they are.
// When robot is fully stationary, max two sensors can see the same spot per full scan.
// When moving, three sensors can see the same spot per full scan.
// It's very unlikely that 16 references would run out for 6 full scans.

#define MAX_SRC_POSE_REFS 16
#define VOXFILTER_XS 128
#define VOXFILTER_YS 128
#define VOXFILTER_ZS 64
#define VOXFILTER_STEP 32

// Remember to zero this struct out first.
typedef struct
{
	voxfilter_point_t points[VOXFILTER_XS][VOXFILTER_YS][VOXFILTER_ZS][MAX_SRC_POSE_REFS];
} voxfilter_t;

typedef struct
{
	int32_t src_seq_id; // Sequence ID gives us both pose and sensor idx, for exact source x,y,z for raytracing empty space. 0 is an invalid id

	int32_t x;
	int32_t y;
	int32_t z;
} tmp_cloud_point_t;

/*
#define MAX_POINTS_PER_INDEX_LIST
#define INDEX_LIST_XS 160
#define INDEX_LIST_YS 160
#define INDEX_LIST_ZS 64
#define INDEX_LIST_STEP 256
tmp_cloud_list_indeces[
*/

#define MAX_POINTS (8000*10*64)
typedef struct
{
	int n_points;
	tmp_cloud_point_t points[MAX_POINTS];
} tmp_cloud_t;

#define RESOLEVELS 0b1111

static void cloud_to_voxmap(tmp_cloud_t* cloud, int ref_x, int ref_y, int ref_z)
{
	for(int rl=0; rl<8; rl++)
	{
		if(!(RESOLEVELS & (1<<rl)))
			continue;

		for(int i=0; i<cloud->n_points; i++)
		{
			po_coords_t po = po_coords(cloud->points[i].x+ref_x, cloud->points[i].y+ref_y, cloud->points[i].z+ref_z, rl);
			uint8_t* p_vox = get_p_voxel(po, rl);
			*p_vox = 0x0f;
			mark_page_changed(po.px, po.py, po.pz);
		}
	}
}

static inline void cloud_insert_point(tmp_cloud_t* cloud, int seq_id, int32_t x, int32_t y, int32_t z)
{
	if(cloud->n_points >= MAX_POINTS)
	{
		printf("WARNING: Ignoring point seq=%d, (%d,%d,%d), cloud full.\n", seq_id, x, y, z);
		return;
	}
	assert(cloud->n_points >= 0);

	cloud->points[cloud->n_points].src_seq_id = seq_id;
	cloud->points[cloud->n_points].x = x;
	cloud->points[cloud->n_points].y = y;
	cloud->points[cloud->n_points].z = z;

	cloud->n_points++;
}

static void voxfilter_to_cloud(voxfilter_t* voxfilter, tmp_cloud_t* cloud)
{
	int insert_cnt = 0;
	for(int yy=0; yy<VOXFILTER_YS; yy++)
	{
		for(int xx=0; xx<VOXFILTER_XS; xx++)
		{
			for(int zz=0; zz<VOXFILTER_ZS; zz++)
			{
				for(int i=0; i<MAX_SRC_POSE_REFS; i++)
				{
					voxfilter_point_t* p = &(voxfilter->points[xx][yy][zz][i]);
					if(p->src_seq_id == 0)
					{
						break;
					}

					assert(p->cnt > 0); // if seq_id is valid, there must be at least one point.

					int32_t x = p->x / p->cnt;
					int32_t y = p->y / p->cnt;
					int32_t z = p->z / p->cnt;

					cloud_insert_point(cloud, p->src_seq_id, x, y, z);
					insert_cnt++;
				}
			}
		}
	}

	printf("INFO: voxfilter_to_cloud inserted %d points\n", insert_cnt);
}

static inline void voxfilter_insert_point(tmp_cloud_t* cloud, voxfilter_t* voxfilter, int seq_id, int32_t x, int32_t y, int32_t z)
{
	int vox_x = x/VOXFILTER_STEP + VOXFILTER_XS/2;
	int vox_y = y/VOXFILTER_STEP + VOXFILTER_YS/2;
	int vox_z = z/VOXFILTER_STEP + VOXFILTER_ZS/2;

	if(vox_x < 0 || vox_x > VOXFILTER_XS-1 || vox_y < 0 || vox_y > VOXFILTER_YS-1 || vox_z < 0 || vox_z > VOXFILTER_ZS-1)
	{
//		printf("INFO: voxfilter: skipping OOR point %d, %d, %d\n", vox_x, vox_y, vox_z);
		cloud_insert_point(cloud, seq_id, x, y, z);
		return;
	}

	
	int found = 0;
	for(int i=0; i < MAX_SRC_POSE_REFS; i++)
	{
		if(voxfilter->points[vox_x][vox_y][vox_z][i].src_seq_id == seq_id)
		{
			// Existing point with the same seq_id - average on the top of this.
			voxfilter->points[vox_x][vox_y][vox_z][i].cnt++;
			voxfilter->points[vox_x][vox_y][vox_z][i].x += x;
			voxfilter->points[vox_x][vox_y][vox_z][i].y += y;
			voxfilter->points[vox_x][vox_y][vox_z][i].z += z;
			found = 1;
			break;
		}
	}

	if(!found)
	{
		int empty_spot_found = 0;
		for(int i=0; i < MAX_SRC_POSE_REFS; i++)
		{
			if(voxfilter->points[vox_x][vox_y][vox_z][i].src_seq_id == 0)
			{
				// Take this new, empty spot
				voxfilter->points[vox_x][vox_y][vox_z][i].src_seq_id = seq_id;
				voxfilter->points[vox_x][vox_y][vox_z][i].cnt = 1;
				voxfilter->points[vox_x][vox_y][vox_z][i].x = x;
				voxfilter->points[vox_x][vox_y][vox_z][i].y = y;
				voxfilter->points[vox_x][vox_y][vox_z][i].z = z;
				empty_spot_found = 1;
				break;
			}
		}

		if(!empty_spot_found)
		{
			printf("INFO: voxfilter_insert_point had to ignore a point due to reference indeces running out.\n");
		}
	}
	
}

void tof_to_voxfilter_and_cloud(int is_narrow, uint16_t* ampldist, hw_pose_t* pose, int seq_id, int sidx, int32_t ref_x, int32_t ref_y, int32_t ref_z, voxfilter_t* voxfilter, tmp_cloud_t* cloud, int voxfilter_threshold)
{
	if(sidx < 0 || sidx >= N_SENSORS)
	{
		printf("Invalid sidx\n");
		return;
	}

	int32_t robot_x = pose->x;
	int32_t robot_y = pose->y;
	int32_t robot_z = pose->z;

	uint16_t robot_ang = pose->ang>>16;

	// Rotation: xr = x*cos(a) + y*sin(a)
	//           yr = -x*sin(a) + y*cos(a)
	// It seems to me this widely touted formula has inverted y axis, don't understand why, so it should be:
	// Rotation: xr = x*cos(a) - y*sin(a)
	//           yr = x*sin(a) + y*cos(a)


	uint16_t global_sensor_hor_ang = sensor_mounts[sidx].ang_rel_robot + robot_ang;
//	uint16_t global_sensor_ver_ang = sensor_mounts[sidx].vert_ang_rel_ground;

	int16_t pitch_ang = pose->pitch>>16;
	int16_t roll_ang = pose->roll>>16;

	uint16_t global_sensor_ver_ang = 
		(int32_t)((int16_t)sensor_mounts[sidx].vert_ang_rel_ground) +
		((lut_cos_from_u16(sensor_mounts[sidx].ang_rel_robot)*pitch_ang)>>SIN_LUT_RESULT_SHIFT) +
		((lut_sin_from_u16(sensor_mounts[sidx].ang_rel_robot)*roll_ang)>>SIN_LUT_RESULT_SHIFT);


	uint16_t local_sensor_hor_ang = sensor_mounts[sidx].ang_rel_robot;
	uint16_t local_sensor_ver_ang = sensor_mounts[sidx].vert_ang_rel_ground;

	int32_t  global_sensor_x = robot_x - ref_x +
			((lut_cos_from_u16(robot_ang)*sensor_mounts[sidx].x_rel_robot)>>SIN_LUT_RESULT_SHIFT) +
			((lut_sin_from_u16(robot_ang)*-1*sensor_mounts[sidx].y_rel_robot)>>SIN_LUT_RESULT_SHIFT);

	int32_t  global_sensor_y = robot_y - ref_y + 
			((lut_sin_from_u16(robot_ang)*sensor_mounts[sidx].x_rel_robot)>>SIN_LUT_RESULT_SHIFT) +
			((lut_cos_from_u16(robot_ang)*sensor_mounts[sidx].y_rel_robot)>>SIN_LUT_RESULT_SHIFT);

	int32_t  global_sensor_z = robot_z - ref_z + sensor_mounts[sidx].z_rel_ground;


	int32_t  local_sensor_x = sensor_mounts[sidx].x_rel_robot;
	int32_t  local_sensor_y = sensor_mounts[sidx].y_rel_robot;
	int32_t  local_sensor_z = sensor_mounts[sidx].z_rel_ground;


	for(int py=1; py<TOF_YS-1; py++)
//	for(int py=29; py<32; py++)
	{
		for(int px=1; px<TOF_XS-1; px++)
//		for(int px=75; px<85; px++)
//		for(int px=79; px<82; px++)
		{
			int32_t avg = 0;
			int n_conform = 0;
			if(is_narrow)
			{
				int npy, npx;
				npx=px-TOF_NARROW_X_START;
				npy=py-TOF_NARROW_Y_START;
				if(npx < 1 || npy < 1 || npx >= TOF_XS_NARROW-1 || npx >= TOF_YS_NARROW-1)
					continue;

				int32_t refdist = ampldist[(npy+0)*TOF_XS_NARROW+(npx+0)]&DIST_MASK;
				if(refdist == DIST_UNDEREXP)
					continue;

				for(int iy=-1; iy<=1; iy++)
				{
					for(int ix=-1; ix<=1; ix++)
					{
						int32_t dist = ampldist[(npy+iy)*TOF_XS_NARROW+(npx+ix)]&DIST_MASK;
						if(dist != DIST_UNDEREXP && dist > refdist-100 && dist < refdist+100)
						{
							avg+=dist;
							n_conform++;
						}
					
					}
				}

			}
			else
			{
				int32_t refdist = ampldist[(py+0)*TOF_XS+(px+0)]&DIST_MASK;
				if(refdist == DIST_UNDEREXP)
					continue;

				for(int iy=-1; iy<=1; iy++)
				{
					for(int ix=-1; ix<=1; ix++)
					{
						int32_t dist = ampldist[(py+iy)*TOF_XS+(px+ix)]&DIST_MASK;
						if(dist != DIST_UNDEREXP && dist > refdist-100 && dist < refdist+100)
						{
							avg+=dist;
							n_conform++;
						}
					
					}
				}
			}

			avg <<= DIST_SHIFT;
			avg /= n_conform;

			if(n_conform >= 7)
			{
				int32_t d = avg;

				uint16_t hor_ang, ver_ang;


				// TODO: This optimizes out once we have sensor-by-sensor geometric tables;
				// they can be pre-built to the actual mount_mode.
				switch(sensor_mounts[sidx].mount_mode)
				{
					case 1: 
					hor_ang = -1*geocoords[py*TOF_XS+px].yang;
					ver_ang = geocoords[py*TOF_XS+px].xang;
					break;

					case 2: 
					hor_ang = geocoords[py*TOF_XS+px].yang;
					ver_ang = -1*geocoords[py*TOF_XS+px].xang;
					break;

					case 3:
					hor_ang = -1*geocoords[py*TOF_XS+px].xang;
					ver_ang = geocoords[py*TOF_XS+px].yang;
					break;

					case 4:
					hor_ang = geocoords[py*TOF_XS+px].xang;
					ver_ang = -1*geocoords[py*TOF_XS+px].yang;
					break;

					default: return;
				}

				uint16_t comb_hor_ang = hor_ang + global_sensor_hor_ang;
				uint16_t comb_ver_ang = ver_ang + global_sensor_ver_ang;

				int32_t x = (((int64_t)d * (int64_t)lut_cos_from_u16(comb_ver_ang) * (int64_t)lut_cos_from_u16(comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + global_sensor_x;
				int32_t y = (((int64_t)d * (int64_t)lut_cos_from_u16(comb_ver_ang) * (int64_t)lut_sin_from_u16(comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + global_sensor_y;
				int32_t z = (((int64_t)d * (int64_t)lut_sin_from_u16(comb_ver_ang))>>SIN_LUT_RESULT_SHIFT) + global_sensor_z;

				uint16_t local_comb_hor_ang = hor_ang + local_sensor_hor_ang;
				uint16_t local_comb_ver_ang = ver_ang + local_sensor_ver_ang;


				int32_t local_x = (((int64_t)d * (int64_t)lut_cos_from_u16(local_comb_ver_ang) * (int64_t)lut_cos_from_u16(local_comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + local_sensor_x;
				int32_t local_y = (((int64_t)d * (int64_t)lut_cos_from_u16(local_comb_ver_ang) * (int64_t)lut_sin_from_u16(local_comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + local_sensor_y;
				int32_t local_z = (((int64_t)d * (int64_t)lut_sin_from_u16(local_comb_ver_ang))>>SIN_LUT_RESULT_SHIFT) + local_sensor_z;

				// VACUUM APP: Ignore the nozzle
				#define NOZZLE_WIDTH 760
				if(local_z < 200 && local_x < 520 && local_x > 120 && local_y > -(NOZZLE_WIDTH/2) && local_y < (NOZZLE_WIDTH/2))
					continue;

				// Completely ignore nozzle area obstacles for mapping, but give the floor if visible!
				if(local_z > 100 && local_x < 520 && local_x > 120 && local_y > -(NOZZLE_WIDTH/2) && local_y < (NOZZLE_WIDTH/2))
					continue;


				if(voxfilter && d < voxfilter_threshold)
					voxfilter_insert_point(cloud, voxfilter, seq_id, x, y, z);
				else
					cloud_insert_point(cloud, seq_id, x, y, z);

			}
		}
	}

}

// Not thread safe, supplies pointer to internal static buf
static uint8_t buf[B2S_MAX_LEN];
int process_file(char* fname, tof_slam_set_t** tss)
{
	*tss = NULL;
	FILE* fil = fopen(fname, "rb");
	if(!fil)
	{
		printf("Error opening file %s for read\n", fname);
		return -2;
	}


	int n_bytes_read = fread(buf, 1, B2S_MAX_LEN, fil);

	fclose(fil);

//	printf("Read %d bytes\n", n_bytes_read);
	if(n_bytes_read < B2S_TOTAL_OVERHEAD_WITHOUT_CRC)
	{
		printf("ERROR: File is too short.\n");
		return -2;
	}

	int expected_size = ((b2s_header_t*)buf)->payload_len + B2S_TOTAL_OVERHEAD_WITHOUT_CRC;

	if(n_bytes_read != expected_size)
	{
		printf("ERROR: File size vs. header information mismatch, bytes_read=%d, expected_size=%d\n", n_bytes_read, expected_size);
		return -3;
	}


	uint8_t* p_data = buf;

//	printf("Got something! Messages:\n");


	int offs = sizeof(b2s_header_t);
	for(int i=0; i<B2S_SUBS_U64_ITEMS; i++)
	{
		uint64_t t = ((b2s_header_t*)p_data)->subs[i];
		for(int s=i*64; s<(i+1)*64; s++)
		{
			if(t & 1)
			{
				// id #s is enabled
//				printf("msgid=%u  name=%s  comment=%s\n", s, b2s_msgs[s].name, b2s_msgs[s].comment);
//				if(b2s_msgs[s].p_print)
//					b2s_msgs[s].p_print(&p_data[offs]);

				if(s==14)
				{
					*tss = (tof_slam_set_t*)&p_data[offs];
					//printf("Set tss to %lx\n", (uint64_t)(*tss));
					return 0;

//					handle_tof_slam_set(tss);

//					tof_to_voxmap(0, tss->sets[0].ampldist, &tss->sets[0].pose, tss->sidx, base_x, base_y);

//					if(tss->flags & TOF_SLAM_SET_FLAG_SET1_WIDE)
//						tof_to_voxmap(0, tss->sets[1].ampldist, &tss->sets[1].pose, tss->sidx, base_x, base_y);
//					if(tss->flags & TOF_SLAM_SET_FLAG_SET1_NARROW)
//						tof_to_voxmap(1, tss->sets[1].ampldist, &tss->sets[1].pose, tss->sidx, base_x, base_y);


				}

				offs += b2s_msgs[s].size;

			}
			t >>= 1;
		}
	}

	return 1;

}


int main()
{
	printf("Super Slammings 2.0 2000\n");
	static firstsidx_pose_t firstsidx_poses[MAX_FIRSTSIDX_POSES];
	static submap_meta_t submap_metas[MAX_SUBMAPS];

	int n_firstsidx_poses;

	extract_firstsidx_poses(firstsidx_poses, &n_firstsidx_poses);

	int n_submaps;

	group_submaps(submap_metas, &n_submaps, firstsidx_poses, n_firstsidx_poses);

	for(int sm=0; sm<n_submaps; sm++)
	{
		printf("Submap %3d: idx %8d .. %8d  (len %4d)", sm, submap_metas[sm].start_idx, submap_metas[sm].end_idx, submap_metas[sm].end_idx-submap_metas[sm].start_idx);
		if(sm < n_submaps-1)
		{
			printf(" (overlaps the next by %4d)  ", -1*(submap_metas[sm+1].start_idx - submap_metas[sm].end_idx));
		}
		else
			printf("                              ");

		printf("dx=%4d  dy=%4d  dz=%4d   avg (%+6d %+6d %+6d)\n", submap_metas[sm].max_x-submap_metas[sm].min_x, submap_metas[sm].max_y-submap_metas[sm].min_y,
			submap_metas[sm].max_z-submap_metas[sm].min_z, submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z);

		for(int ssm=0; ssm<submap_metas[sm].n_subsubmaps; ssm++)
		{
			printf("         Subsubmap %2d: idx %8d .. %8d (len %2d), avg (%+6d %+6d %+6d)\n", 
				ssm, submap_metas[sm].subsubmaps[ssm].start_idx, submap_metas[sm].subsubmaps[ssm].end_idx, 
				submap_metas[sm].subsubmaps[ssm].end_idx-submap_metas[sm].subsubmaps[ssm].start_idx,
				submap_metas[sm].subsubmaps[ssm].avg_x, submap_metas[sm].subsubmaps[ssm].avg_y, submap_metas[sm].subsubmaps[ssm].avg_z);
		}
	}	

	return 0;

	for(int sm=0; sm<n_submaps; sm+=1)
	{
		printf("Optimizing pointcloud for submap %d: idx %8d .. %8d  (len %4d)", sm, submap_metas[sm].start_idx, submap_metas[sm].end_idx, submap_metas[sm].end_idx-submap_metas[sm].start_idx);
		printf("dx=%4d  dy=%4d  dz=%4d   avg (%+6d %+6d %+6d)\n", submap_metas[sm].max_x-submap_metas[sm].min_x, submap_metas[sm].max_y-submap_metas[sm].min_y,
			submap_metas[sm].max_z-submap_metas[sm].min_z, submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z);

		static tmp_cloud_t tmp_cloud;
		tmp_cloud.n_points = 0;
		static voxfilter_t tmp_voxfilter;

		memset(&tmp_voxfilter, 0, sizeof(voxfilter_t));

		for(int idx=submap_metas[sm].start_idx; idx<submap_metas[sm].end_idx; idx+=VOXFILTER_N_SCANS)
		{
			for(int vi=0; vi<VOXFILTER_N_SCANS; vi++)
			{
				int refidx = idx+vi;

				char fname[1024];
				sprintf(fname, "/home/hrst/robotsoft/tsellari/trace%08d.rb2", refidx);
				tof_slam_set_t* tss;
				if(process_file(fname, &tss) == 0) // tof_slam_set record succesfully extracted
				{
					tof_to_voxfilter_and_cloud(0, 
						tss->sets[0].ampldist, &tss->sets[0].pose,
						refidx, tss->sidx, 
						submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z,
						&tmp_voxfilter, &tmp_cloud, 1500);
				}
				else
				{
					printf("WARNING: Did not find tof_slam_set record from file %s\n", fname);
				}
			}
		}

		printf("---> n_points = %d\n", tmp_cloud.n_points);

		po_coords_t po;
		po = po_coords(submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z, 0);
		load_pages(RESOLEVELS, RESOLEVELS, po.px-2, po.px+2, po.py-2, po.py+2, po.pz-2, po.pz+2);


		static tmp_cloud_t tmp_cloud2;
		tmp_cloud2.n_points = 0;

		voxfilter_to_cloud(&tmp_voxfilter, &tmp_cloud2);

		cloud_to_voxmap(&tmp_cloud2, submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z);


	}


	for(int rl = 0; rl < 4; rl++)
	{
		po_coords_t po = po_coords(0, 0, 10000, rl);
		load_pages(RESOLEVELS, RESOLEVELS, po.px, po.px, po.py, po.py, po.pz, po.pz);
		uint8_t* p_vox = get_p_voxel(po, rl);
		*p_vox = 0x0f;
		mark_page_changed(po.px, po.py, po.pz);
	}

	for(int rl = 0; rl < 4; rl++)
	{
		po_coords_t po = po_coords(2500, 4500, 10000, rl);
		load_pages(RESOLEVELS, RESOLEVELS, po.px, po.px, po.py, po.py, po.pz, po.pz);
		uint8_t* p_vox = get_p_voxel(po, rl);
		*p_vox = 0x0f;
		mark_page_changed(po.px, po.py, po.pz);
	}




	free_all_pages();

	return 0;
} 
