#include <stdio.h>
#include <math.h>
#include <inttypes.h>
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

#define DEGTOANG16(x)  ((uint16_t)((float)(x)/(360.0)*65536.0))

/*
#define RADTODEG(x) ((x)*(360.0/(2.0*M_PI)))
#define DEGTORAD(x) ((x)*((2.0*M_PI)/360.0))
#define ANG32TORAD(x) ( ((double)((uint32_t)(x)))/683565275.576432)
#define ANG32TOFDEG(x) ( ((double)((uint32_t)(x)))/11930464.7111111)
*/

#define DIST_UNDEREXP 0
#define DIST_OVEREXP 1

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

#define MAX_SUBMAPS 16384

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
int end_i = 25000; //12800;

#define MAX_FIRSTSIDX_POSES 65536
#define MAX_REFIDXS 65536

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
		sprintf(fname, "/home/hrst/robodev/robotsoft/tsellari/trace%08d.rb2", i);
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

hw_pose_t add_poses(hw_pose_t a, hw_pose_t b)
{
	hw_pose_t ret;
	ret.x = a.x + b.x;
	ret.y = a.y + b.y;
	ret.z = a.z + b.z;
	ret.ang = a.ang + b.ang;
	ret.pitch = a.pitch + b.pitch;
	ret.roll = a.roll + b.roll;

	return ret;
}

typedef struct
{
	double x;
	double y;
	double z;
	double ang;
	double pitch;
	double roll;
} fpose_t;

#define WRAP_RAD_2PI(x_) do{while(x_ >= 2.0*M_PI) x_ -= 2.0*M_PI; }while(0)
#define WRAP_RAD_0(x_) do{while(x_ < 0.0) x_ += 2.0*M_PI; }while(0)
#define WRAP_RAD(x_) do{WRAP_RAD_2PI(x_); WRAP_RAD_0(x_);}while(0)

#define WRAP_RAD_PI(x_) do{while(x_ >= M_PI) x_ -= 2.0*M_PI; }while(0)
#define WRAP_RAD_NEGPI(x_) do{while(x_ < -M_PI) x_ += 2.0*M_PI; }while(0)
#define WRAP_RAD_BIPO(x_) do{WRAP_RAD_PI(x_); WRAP_RAD_NEGPI(x_);}while(0)


typedef struct
{
	int fsp_idx;
	fpose_t corr;
} corr_point_t;

#define MAX_CORR_POINTS 1000
int n_corr_points;
corr_point_t corr_points[MAX_CORR_POINTS];

void add_corr_point(int fsp_idx, fpose_t corr)
{
	corr_points[n_corr_points].fsp_idx = fsp_idx;
	corr_points[n_corr_points].corr = corr;

	n_corr_points++;
}

double yaw_corr = 0.00030; // 0.000001;

uint8_t enable_submaps[MAX_SUBMAPS] = {
/*  0*/ 1,1,1,1,1, 1,1,1,1,1,
/* 10*/ 1,1,1,1,1, 1,1,1,1,1,
/* 20*/ 1,1,1,1,1, 1,1,1,1,1,
/* 30*/ 1,1,1,1,1, 1,1,1,1,1,
/* 40*/ 1,1,1,1,1, 1,1,1,1,1,
/* 50*/ 1,1,1,1,1, 1,1,1,1,1,
/* 60*/ 1,1,1,1,1, 1,1,1,1,1,
/* 70*/ 1,1,1,1,1, 1,1,1,1,1,
/* 80*/ 1,0,0,0,0, 0,0,0,0,0,
/* 90*/ 0,0,0,0,0, 0,0,0,0,0,
/*100*/ 0,0,0,0,0, 0,0,0,0,0,
/*110*/ 0,0,0,0,0, 0,0,0,0,0,
/*120*/ 0,0,0,0,0, 0,0,0,0,0,
/*130*/ 0,0,0,0,0, 0,0,1,1,1,
/*140*/ 1,1,1,1,1, 1,1,1,1,1,
/*150*/ 1,1,1,1,1, 1,1,1,1,1};

void init_corr_points()
{

	for(int i=0; i<MAX_SUBMAPS; i++)
		enable_submaps[i] = 1;

/*
	add_corr_point(136, (fpose_t){0,0,0, DEGTORAD(-1.5), 0, 0});

	add_corr_point(197, (fpose_t){0,0,0, DEGTORAD(+1.5), 0, 0});


	add_corr_point(334, (fpose_t){0,-90,0, DEGTORAD(+3.0), 0, 0});

	add_corr_point(521, (fpose_t){0,0,0, DEGTORAD(+1.5), 0, 0});

	add_corr_point(1031, (fpose_t){0,0,0, DEGTORAD(-0.75), 0, 0});

	add_corr_point(1195, (fpose_t){0,0,0, DEGTORAD(-0.75), 0, 0});

	add_corr_point(1332, (fpose_t){0,0,0, DEGTORAD(+1.5), 0, 0});


	add_corr_point(2300, (fpose_t){0,0,0, DEGTORAD(-5.0), 0, 0});

	add_corr_point(2355, (fpose_t){-2300,+1000,0, DEGTORAD(0), 0, 0});
*/

}

//double pos_gyro_corr = -0.0135;
//double neg_gyro_corr = -0.0345;

double pos_gyro_corr = -0.00650;
double neg_gyro_corr = -0.02700;

void gen_fposes(firstsidx_pose_t* hwposes, int n_hwposes, fpose_t* out)
{
	out[0].x = hwposes[0].pose.x;
	out[0].y = hwposes[0].pose.y;
	out[0].z = hwposes[0].pose.z;
	out[0].ang   = ANG32TORAD(hwposes[0].pose.ang);
	out[0].pitch = ANG32TORAD(hwposes[0].pose.pitch);
	out[0].roll  = ANG32TORAD(hwposes[0].pose.roll);

	double running_yaw_corr = 0.0;
	double running_pitch_corr = 0.0;
	double running_roll_corr = 0.0;
	for(int i=1; i<n_hwposes; i++)
	{
/*
		if(i==1400)
			yaw_corr = 0.00090;
		if(i==1900)
			yaw_corr = 0.00030;

		if(i==2300)
			yaw_corr = 0.00020;
*/

//		running_yaw_corr += yaw_corr;
		double prev_x = hwposes[i-1].pose.x;
		double prev_y = hwposes[i-1].pose.y;
		double prev_z = hwposes[i-1].pose.z;
		double prev_ang   = ANG32TORAD(hwposes[i-1].pose.ang);
		double prev_pitch = ANG32TORAD(hwposes[i-1].pose.pitch);
		double prev_roll  = ANG32TORAD(hwposes[i-1].pose.roll);

		double now_x = hwposes[i].pose.x;
		double now_y = hwposes[i].pose.y;
		double now_z = hwposes[i].pose.z;
		double now_ang   = ANG32TORAD(hwposes[i].pose.ang);
		double now_pitch = ANG32TORAD(hwposes[i].pose.pitch);
		double now_roll  = ANG32TORAD(hwposes[i].pose.roll);

		double dx = now_x - prev_x;
		double dy = now_y - prev_y;
		double dz = now_z - prev_z;

		double dyaw = now_ang - prev_ang;
		double dpitch = now_pitch - prev_pitch;
		double droll = now_roll - prev_roll;

		WRAP_RAD_BIPO(dyaw);
		WRAP_RAD_BIPO(dpitch);
		WRAP_RAD_BIPO(droll);

		double yaw_corr;
		double pitch_corr = 0.0;
		double roll_corr = 0.0;

		if(dyaw > 0.0)
			yaw_corr = pos_gyro_corr * dyaw;
		else
			yaw_corr = neg_gyro_corr * dyaw;

		running_yaw_corr += yaw_corr;
		running_pitch_corr += pitch_corr;
		running_roll_corr += roll_corr;

		WRAP_RAD(running_yaw_corr);
		WRAP_RAD(running_pitch_corr);
		WRAP_RAD(running_roll_corr);

		// Search for relevant corrpoint:
/*
		for(int cpi=0; cpi<n_corr_points; cpi++)
		{
			if(corr_points[cpi].fsp_idx == i)
			{
				running_yaw_corr += corr_points[cpi].corr.ang;
				dx += corr_points[cpi].corr.x;
				dy += corr_points[cpi].corr.y;
				dz += corr_points[cpi].corr.z;
			}
		}
*/
		out[i].x = out[i-1].x + dx * cos(running_yaw_corr) - dy * sin(running_yaw_corr);
		out[i].y = out[i-1].y + dx * sin(running_yaw_corr) + dy * cos(running_yaw_corr);
		out[i].z = out[i-1].z + dz;

		out[i].ang = out[i-1].ang + dyaw + yaw_corr;
		out[i].pitch = out[i-1].pitch + dpitch + pitch_corr;
		out[i].roll  = out[i-1].roll + droll + roll_corr;

		WRAP_RAD(out[i].ang);
		WRAP_RAD(out[i].pitch);
		WRAP_RAD(out[i].roll);

/*
		now_ang += running_yaw_corr; 

		WRAP_RAD(now_ang);
		WRAP_RAD(now_pitch);
		WRAP_RAD(now_roll);

		out[i].ang = now_ang;
		out[i].pitch = now_pitch;
		out[i].roll = now_roll;
*/
	}
}

void gen_pose_corrs(firstsidx_pose_t* hwposes, firstsidx_pose_t* out, int n_hwposes, fpose_t* fposes)
{
	for(int i=0; i<n_hwposes; i++)
	{
		out[i].idx = hwposes[i].idx;
		out[i].pose.x = (int32_t)(fposes[i].x - (double)hwposes[i].pose.x);
		out[i].pose.y = (int32_t)(fposes[i].y - (double)hwposes[i].pose.y);
		out[i].pose.z = (int32_t)(fposes[i].z - (double)hwposes[i].pose.z);
		double da = fposes[i].ang - ANG32TORAD(hwposes[i].pose.ang);
		double dp = fposes[i].pitch - ANG32TORAD(hwposes[i].pose.pitch);
		double dr = fposes[i].roll - ANG32TORAD(hwposes[i].pose.roll);
		WRAP_RAD(da);
		WRAP_RAD(dp);
		WRAP_RAD(dr);

		out[i].pose.ang = RADTOANGU32(da);
		out[i].pose.pitch = RADTOANGU32(dp);
		out[i].pose.roll = RADTOANGU32(dr);
	}
}



#define TRACK_MIN(invar_, minvar_) do{if((invar_) < (minvar_)) (minvar_) = (invar_);}while(0)
#define TRACK_MAX(invar_, maxvar_) do{if((invar_) > (maxvar_)) (maxvar_) = (invar_);}while(0)

void group_submaps(submap_meta_t* smm_out, int* n_smm_out, firstsidx_pose_t* fsp, int n_fsp, firstsidx_pose_t* pose_corrs)
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

			hw_pose_t cur_pose = add_poses(fsp[cur_fsp].pose, pose_corrs[cur_fsp].pose);

			TRACK_MIN(cur_pose.x, min_x);
			TRACK_MIN(cur_pose.y, min_y);
			TRACK_MIN(cur_pose.z, min_z);

			TRACK_MAX(cur_pose.x, max_x);
			TRACK_MAX(cur_pose.y, max_y);
			TRACK_MAX(cur_pose.z, max_z);

			avg_x += cur_pose.x;
			avg_y += cur_pose.y;
			avg_z += cur_pose.z;
			avg_n++;

			subsub_avg_x += cur_pose.x;
			subsub_avg_y += cur_pose.y;
			subsub_avg_z += cur_pose.z;
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

typedef struct
{
	int x;
	int y;
	int z;
} ray_source_t;

static ray_source_t ray_sources[MAX_REFIDXS];


#define MAX_POINTS (8000*10*64)
typedef struct
{
	int n_points;
	tmp_cloud_point_t points[MAX_POINTS];
} tmp_cloud_t;

/*
#define CLOUDFLT_XS 1280
#define CLOUDFLT_YS 1280
#define CLOUDFLT_ZS 384
#define CLOUDFLT_UNIT 32 //mm
*/

#define CLOUDFLT_XS 800
#define CLOUDFLT_YS 800
#define CLOUDFLT_ZS 256
#define CLOUDFLT_UNIT 64 //mm

static uint8_t free_cnt[CLOUDFLT_XS][CLOUDFLT_YS][CLOUDFLT_ZS];



#define UNCERT_Z 2
#define UNCERT 2
#define LINE_CONE_SLOPE_XY 15
#define LINE_CONE_SLOPE_Z  20

static inline void output_voxel(int x, int y, int z)
{
	assert(x >= 1 && x < CLOUDFLT_XS-1 && y >= 1 && y < CLOUDFLT_YS-1 && z > 1 && z <= CLOUDFLT_ZS-1);

//	if(free_cnt[x][y][z] < 255)
//		free_cnt[x][y][z]++;
	free_cnt[x][y][z] = 1;
}

static inline uint8_t get_voxel(int x, int y, int z)
{
	assert(x >= 0 && x < CLOUDFLT_XS && y >= 0 && y < CLOUDFLT_YS && z > 0 && z <= CLOUDFLT_ZS);

	return free_cnt[x][y][z];
}

static void bresenham3d(int x1, int y1, int z1, int x2, int y2, int z2)
{	
	int dx = x2 - x1;
	int dy = y2 - y1;
	int dz = z2 - z1;
	int x_incr = (dx < 0) ? -1 : 1;
	int y_incr = (dy < 0) ? -1 : 1;
	int z_incr = (dz < 0) ? -1 : 1;

	int l = abs(dx);
	int m = abs(dy);
	int n = abs(dz);
	int dx2 = l << 1;
	int dy2 = m << 1;
	int dz2 = n << 1;
	
	// Current output voxel:
	int px = x1;
	int py = y1;
	int pz = z1;

	if ((l >= m) && (l >= n)) // Move full voxels in x direction
	{
		int err_1 = dy2 - l;
		int err_2 = dz2 - l;
		for(int i = 0; i < l-UNCERT; i++)
		{
			if(abs(px-x2) <= UNCERT || abs(py-y2) <= UNCERT || abs(pz-z2) <= UNCERT_Z)
				return;

			int rangexy = i/LINE_CONE_SLOPE_XY;
			int rangez = i/LINE_CONE_SLOPE_Z;
			for(int iy=-rangexy; iy<=rangexy; iy++)
			{
				int ix=0;
				for(int iz=-rangez; iz<=rangez; iz++)
				{
					output_voxel(px+ix, py+iy, pz+iz);
				}
			}

			if(err_1 > 0)
			{
				py += y_incr;
				err_1 -= dx2;
			}
			if(err_2 > 0)
			{
				pz += z_incr;
				err_2 -= dx2;
			}
			err_1 += dy2;
			err_2 += dz2;
			px += x_incr;
		}
	} 
	else if ((m >= l) && (m >= n)) // Move full voxels in y direction
	{
		int err_1 = dx2 - m;
		int err_2 = dz2 - m;
		for(int i = 0; i < m-UNCERT; i++)
		{
			if(abs(px-x2) <= UNCERT || abs(py-y2) <= UNCERT || abs(pz-z2) <= UNCERT)
				return;

			int rangexy = i/LINE_CONE_SLOPE_XY;
			int rangez = i/LINE_CONE_SLOPE_Z;

			int iy=0;
			for(int ix=-rangexy; ix<=rangexy; ix++)
			{
				for(int iz=-rangez; iz<=rangez; iz++)
				{
					output_voxel(px+ix, py+iy, pz+iz);
				}
			}


			if (err_1 > 0)
			{
				px += x_incr;
				err_1 -= dy2;
			}
			if (err_2 > 0)
			{
				pz += z_incr;
				err_2 -= dy2;
			}
			err_1 += dx2;
			err_2 += dz2;
			py += y_incr;
		}
	}
	else // Move full voxels in z direction
	{
		int err_1 = dy2 - n;
		int err_2 = dx2 - n;
		for(int i = 0; i < n-UNCERT; i++)
		{
			if(abs(px-x2) <= UNCERT || abs(py-y2) <= UNCERT || abs(pz-z2) <= UNCERT)
				return;

			int rangexy = i/LINE_CONE_SLOPE_XY;
			int rangez = i/LINE_CONE_SLOPE_Z;
			for(int iy=-rangexy; iy<=rangexy; iy++)
			{
				for(int ix=-rangexy; ix<=rangexy; ix++)
				{
					int iz=0;
					output_voxel(px+ix, py+iy, pz+iz);
				}
			}

			if (err_1 > 0)
			{
				py += y_incr;
				err_1 -= dz2;
			}
			if (err_2 > 0)
			{
				px += x_incr;
				err_2 -= dz2;
			}
			err_1 += dy2;
			err_2 += dx2;
			pz += z_incr;
		}
	}
}

void filter_cloud(tmp_cloud_t* cloud, tmp_cloud_t* out, int ref_x, int ref_y, int ref_z)
{
//	printf("filter_cloud ref (%d, %d, %d)\n", ref_x, ref_y, ref_z);
	memset(free_cnt, 0, sizeof(free_cnt));

	// Trace empty space:
	for(int p=0; p<cloud->n_points; p++)
	{
		int seq_id = cloud->points[p].src_seq_id;

		int sx = (ray_sources[seq_id].x)/CLOUDFLT_UNIT + CLOUDFLT_XS/2;
		int sy = (ray_sources[seq_id].y)/CLOUDFLT_UNIT + CLOUDFLT_YS/2;
		int sz = (ray_sources[seq_id].z)/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

		int px = (cloud->points[p].x)/CLOUDFLT_UNIT + CLOUDFLT_XS/2;
		int py = (cloud->points[p].y)/CLOUDFLT_UNIT + CLOUDFLT_YS/2;
		int pz = (cloud->points[p].z)/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

/*
		printf("p=%d, seq_id=%d, src=(%d,%d,%d), point=(%d,%d,%d), (%d,%d,%d)->(%d,%d,%d)\n", p, seq_id, 
			ray_sources[seq_id].x, ray_sources[seq_id].y, ray_sources[seq_id].z,
			cloud->points[p].x, cloud->points[p].y, cloud->points[p].z,
			sx,sy,sz, px,py,pz);
*/
		bresenham3d(sx, sy, sz, px, py, pz);
	}

#if 0
	{
		int sz = (ray_sources[507].z)/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

		printf("SLICE AT Z=%d:\n", sz);
		for(int yy=400+80; yy>=400-80; yy-=1)
		{
			for(int xx=400-160; xx<=400+160; xx+=1)
			{
				//printf("%3d ", free_cnt[xx][yy][sz]);
				if(free_cnt[xx][yy][sz] > 2)
					putchar('#');
				else if(free_cnt[xx][yy][sz] > 1)
					putchar('-');
				else
					putchar(' ');
			}
			printf("\n");
		}
		printf("\n");
	//	abort();
	}
#endif	

	out->n_points = 0;
	int n_p = 0;
	// Remove points at empty space
	for(int p=0; p<cloud->n_points; p++)
	{
		int px = (cloud->points[p].x)/CLOUDFLT_UNIT + CLOUDFLT_XS/2;
		int py = (cloud->points[p].y)/CLOUDFLT_UNIT + CLOUDFLT_YS/2;
		int pz = (cloud->points[p].z)/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

		int val = get_voxel(px, py, pz);
		if(val < 1)
		{
			out->points[n_p] = cloud->points[p];
			n_p++;
		}
			
	}
	out->n_points = n_p;

}


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
			if(((*p_vox) & 0x0f) < 15)
				(*p_vox)++;
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

	//printf("INFO: voxfilter_to_cloud inserted %d points\n", insert_cnt);
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





void tof_to_voxfilter_and_cloud(int is_narrow, uint16_t* ampldist, hw_pose_t pose, int seq_id, int sidx, int32_t ref_x, int32_t ref_y, int32_t ref_z,
	 voxfilter_t* voxfilter, tmp_cloud_t* cloud, int voxfilter_threshold, int dist_ignore_threshold)
{
	if(sidx < 0 || sidx >= N_SENSORS)
	{
		printf("Invalid sidx\n");
		return;
	}

//	if(sidx != 8)
//		return;

	int32_t robot_x = pose.x;
	int32_t robot_y = pose.y;
	int32_t robot_z = pose.z;

	uint16_t robot_ang = pose.ang>>16;

	// Rotation: xr = x*cos(a) + y*sin(a)
	//           yr = -x*sin(a) + y*cos(a)
	// It seems to me this widely touted formula has inverted y axis, don't understand why, so it should be:
	// Rotation: xr = x*cos(a) - y*sin(a)
	//           yr = x*sin(a) + y*cos(a)


	uint16_t global_sensor_hor_ang = sensor_mounts[sidx].ang_rel_robot + robot_ang;
//	uint16_t global_sensor_ver_ang = sensor_mounts[sidx].vert_ang_rel_ground;

	int16_t pitch_ang = pose.pitch>>16;
	int16_t roll_ang = pose.roll>>16;

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


	ray_sources[seq_id].x = global_sensor_x;
	ray_sources[seq_id].y = global_sensor_y;
	ray_sources[seq_id].z = global_sensor_z;


	int y_ignore=1;
	int x_ignore=1;

	// Horrible temporary kludge
	if(!is_narrow)
	{

		int32_t nearfield_avg_dist = 0;
		int nearfield_avg_n = 0;
		for(int py=10; py<TOF_YS-10; py++)
		{
			for(int px=50; px<TOF_XS-50; px++)
			{
				int32_t dist = ampldist[(py)*TOF_XS+(px)]&DIST_MASK;

//				printf("py=%d, px=%d, dist=%d\n", py, px, dist);

				if(dist == DIST_UNDEREXP) dist = 2000>>DIST_SHIFT;

				nearfield_avg_dist += dist; // -O3 segfaults here for no apparent reason
				nearfield_avg_n++;
			}
		}	

		nearfield_avg_dist /= nearfield_avg_n;
		nearfield_avg_dist <<= DIST_SHIFT;


		// x_ignore: 0 at 1000mm, 66.6 at 0mm
		// y_ignore: 0 at 500mm, 25 at 0mm
	//	x_ignore = (1000-nearfield_avg_dist)/15;
	//	y_ignore = (500-nearfield_avg_dist)/20;

		// x_ignore: 0 at 1400mm, 87.5 at 0mm
		// y_ignore: 0 at 500mm, 25 at 0mm
	//	x_ignore = (1400-nearfield_avg_dist)/16;
	//	y_ignore = (500-nearfield_avg_dist)/20;

		// x_ignore: 0 at 1600mm, 94 at 0mm
		// y_ignore: 0 at 800mm, 32 at 0mm
		x_ignore = (1600-nearfield_avg_dist)/13; // /17
		y_ignore = (800-nearfield_avg_dist)/22; // /25

		if(x_ignore < 1) x_ignore = 1;
		if(x_ignore > 72) x_ignore = 72;

		if(y_ignore < 1) y_ignore = 1;
		if(y_ignore > 26) y_ignore = 26;

	//	printf("nearfield_avg_dist = %d, x_ignore=%d, y_ignore=%d\n", nearfield_avg_dist, x_ignore, y_ignore);
	}

	// end kludge

	for(int py=y_ignore; py<TOF_YS-y_ignore; py++)
//	for(int py=29; py<32; py++)
	{
		for(int px=x_ignore; px<TOF_XS-x_ignore; px++)
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
						if(dist != DIST_UNDEREXP && dist != DIST_OVEREXP && dist > refdist-(120>>DIST_SHIFT) && dist < refdist+(120>>DIST_SHIFT))
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
						if(dist != DIST_UNDEREXP && dist != DIST_OVEREXP && dist > refdist-(120>>DIST_SHIFT) && dist < refdist+(120>>DIST_SHIFT))
						{
							avg+=dist;
							n_conform++;
						}
					
					}
				}
			}

			if(n_conform >= 7)
			{
				avg <<= DIST_SHIFT;
				avg /= n_conform;

				int32_t d = avg;

				if(d < dist_ignore_threshold)
					continue;

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

//				if(z > 2200)
//					continue;

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


int main(int argc, char** argv)
{

	if(argc == 3)
	{
		sscanf(argv[1], "%lf", &pos_gyro_corr);
		sscanf(argv[2], "%lf", &neg_gyro_corr);
	}

	printf("Super Slammings 2.0 2000\n");
	printf("pos_gyro_corr = %lf, neg_gyro_corr = %lf\n", pos_gyro_corr, neg_gyro_corr);

	init_corr_points();

	static firstsidx_pose_t firstsidx_poses[MAX_FIRSTSIDX_POSES];
	static firstsidx_pose_t pose_corrs[MAX_FIRSTSIDX_POSES];
	static submap_meta_t submap_metas[MAX_SUBMAPS];

	int n_firstsidx_poses;

	extract_firstsidx_poses(firstsidx_poses, &n_firstsidx_poses);

	static fpose_t fposes[MAX_FIRSTSIDX_POSES];
	gen_fposes(firstsidx_poses, n_firstsidx_poses, fposes);

#if 0
	FILE* kakkafile = fopen("kakka.kak", "rb");
	static fpose_t cmp_fposes[MAX_FIRSTSIDX_POSES];
	fread(cmp_fposes, sizeof(cmp_fposes), 1, kakkafile);
	fclose(kakkafile);

	double win_gpos = 0.0;
	double win_gneg = 0.0;
	double smallest = 9999999999999999999999999999999999999999999999999999999.9;
	for(double gpos=-0.10; gpos<0.10; gpos += 0.0005)
	{
		for(double gneg=-0.10; gneg<0.10; gneg += 0.0005)
		{
			memset(fposes, 0, sizeof(fposes));

			pos_gyro_corr = gpos;
			neg_gyro_corr = gneg;

			gen_fposes(firstsidx_poses, n_firstsidx_poses, fposes);

			double score_acc = 0.0;
			for(int i=0; i<n_firstsidx_poses; i++)
			{
				double dx = cmp_fposes[i].x - fposes[i].x;
				double dy = cmp_fposes[i].y - fposes[i].y;
				double dang = cmp_fposes[i].ang - fposes[i].ang;
				WRAP_RAD_BIPO(dang);

				// 180 degrees off (3.14 rad) is scored equally to a 31.4m error:
//				double score = sq(dx) + sq(dy) + sq(dang*10000.0);
				// 180 degrees off (3.14 rad) is scored equally to a 6*31.4m error:
				double score = sq(dx) + sq(dy) + sq(dang*60000.0);
				score_acc += score;
			}

			if(score_acc < smallest)
			{
				smallest = score_acc;
				win_gpos = gpos;
				win_gneg = gneg;
			}

			printf("gpos=%+8.5f gneg=%+8.5f score=%15.0lf winner: %+8.5f  %+8.5f with score=%15.0lf               \r",
				gpos, gneg, score_acc, win_gpos, win_gneg, smallest);
			fflush(stdout);

		}

	}

	printf("\n");
	return 0;
#endif

	gen_pose_corrs(firstsidx_poses, pose_corrs, n_firstsidx_poses, fposes);



	// Copy the last correction, will be accessed later.
	pose_corrs[n_firstsidx_poses] = pose_corrs[n_firstsidx_poses-1];
	int n_pose_corrs = n_firstsidx_poses + 1;



	for(int i=0; i < n_firstsidx_poses; i++)
	{
		printf("%6d (%8d): (%+6d %+6d %+6d  %5.1f) -> (%+6.0f %+6.0f %+6.0f  %5.1f), corr (%+6d %+6d %+6d  %5.1f)\n",
			i, firstsidx_poses[i].idx,
			firstsidx_poses[i].pose.x, firstsidx_poses[i].pose.y, firstsidx_poses[i].pose.z, ANG32TOFDEG(firstsidx_poses[i].pose.ang),
			fposes[i].x, fposes[i].y, fposes[i].z, RADTODEG(fposes[i].ang),
			pose_corrs[i].pose.x, pose_corrs[i].pose.y, pose_corrs[i].pose.z, ANG32TOFDEG(pose_corrs[i].pose.ang));
	}

//	FILE* kakkafile = fopen("kakka.kak", "wb");
//	fwrite(fposes, sizeof(fposes), 1, kakkafile);
//	fclose(kakkafile);
//	return 0;


	

	int n_submaps;

	group_submaps(submap_metas, &n_submaps, firstsidx_poses, n_firstsidx_poses, pose_corrs);

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

	int64_t total_points = 0;
	int64_t total_after_filtering = 0;


	for(int sm=0; sm<n_submaps; sm+=2)
	{
		if(!enable_submaps[sm])
			continue;

		printf("Optimizing pointcloud for submap %d: idx %8d .. %8d  (len %4d)", sm, submap_metas[sm].start_idx, submap_metas[sm].end_idx, submap_metas[sm].end_idx-submap_metas[sm].start_idx);
		printf("dx=%4d  dy=%4d  dz=%4d   avg (%+6d %+6d %+6d)\n", submap_metas[sm].max_x-submap_metas[sm].min_x, submap_metas[sm].max_y-submap_metas[sm].min_y,
			submap_metas[sm].max_z-submap_metas[sm].min_z, submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z);


		// One point cloud is created for every submap
		// One voxel filter is used for every subsubmap.
		// Subsubmaps are contiguous and span the whole submap - if you loop through all subsubmap indices,
		// you have looped through the submap as well.

		// Create the cloud here:
		static tmp_cloud_t tmp_cloud;
		tmp_cloud.n_points = 0;

		for(int ssm=0; ssm<submap_metas[sm].n_subsubmaps; ssm++)
		{
			// And the voxfilter here:
			static voxfilter_t tmp_voxfilter;
			memset(&tmp_voxfilter, 0, sizeof(voxfilter_t));
//			for(int idx=submap_metas[sm].subsubmaps[ssm].start_idx; idx < submap_metas[sm].subsubmaps[ssm].start_idx+9; idx++)
			for(int idx=submap_metas[sm].subsubmaps[ssm].start_idx; idx < submap_metas[sm].subsubmaps[ssm].end_idx; idx++)
			{
				// Brute-force (todo: more efficiently) search the matching table index for pose correction
				int cidx = -1;
				for(int i=0; i<n_pose_corrs-1; i++)
				{
					if(idx >= pose_corrs[i].idx && idx <= pose_corrs[i+1].idx)
					{
						cidx = i;
					//	printf("idx=%d; cidx=%d: pose_corrs[%d].idx = %d\n", idx, cidx, i, pose_corrs[i].idx);
						break;
					}
				}
				assert(cidx>=0);

				char fname[1024];
				sprintf(fname, "/home/hrst/robodev/robotsoft/tsellari/trace%08d.rb2", idx);
				tof_slam_set_t* tss;
				if(process_file(fname, &tss) == 0) // tof_slam_set record succesfully extracted
				{
					hw_pose_t pose0 = add_poses(tss->sets[0].pose, pose_corrs[cidx].pose);
					hw_pose_t pose1 = add_poses(tss->sets[1].pose, pose_corrs[cidx].pose);
					tof_to_voxfilter_and_cloud(0, 
						tss->sets[0].ampldist, pose0,
						idx, tss->sidx, 
						submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z,
						&tmp_voxfilter, &tmp_cloud, 1800, 300);

					if(tss->flags & TOF_SLAM_SET_FLAG_SET1_NARROW)
						tof_to_voxfilter_and_cloud(1, 
							tss->sets[1].ampldist, pose1,
							idx, tss->sidx, 
							submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z,
							NULL, &tmp_cloud, 1800, 3000);

					if(tss->flags & TOF_SLAM_SET_FLAG_SET1_WIDE)
						tof_to_voxfilter_and_cloud(0, 
							tss->sets[1].ampldist, pose1,
							idx, tss->sidx, 
							submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z,
							NULL, &tmp_cloud, 1800, 3000);
				}
				else
				{
					printf("WARNING: Did not find tof_slam_set record from file %s\n", fname);
				}
			}

			// Voxfilter was used for a small subsubmap - insert it to the larger submap cloud
			voxfilter_to_cloud(&tmp_voxfilter, &tmp_cloud);
		}

		printf("---> n_points = %d ", tmp_cloud.n_points);
		total_points += tmp_cloud.n_points;

		static tmp_cloud_t tmp_cloud_filtered;

//		filter_cloud(&tmp_cloud, &tmp_cloud_filtered, submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z);

		printf("--> after filtering = %d\n", tmp_cloud_filtered.n_points);
		total_after_filtering += tmp_cloud_filtered.n_points;

		po_coords_t po;
		//printf("Load %d,%d,%d\n", submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z);
		po = po_coords(submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z, 0);
		load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);


//		cloud_to_voxmap(&tmp_cloud_filtered, submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z);
		cloud_to_voxmap(&tmp_cloud, submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z);

	}


/*
Example of voxfilter data reduction: 78836630/119061330 = 0.66215 (threshold 1500)
                                     77905961/119061330 = 0.65433 (threshold 1800)
*/

	printf("Total points: %"PRIi64", after filtering %"PRIi64"\n", total_points, total_after_filtering);

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
