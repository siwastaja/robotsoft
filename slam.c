#include <stdio.h>
#include <math.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <time.h>
#include <sys/time.h>

#define DEFINE_API_VARIABLES
#include "api_board_to_soft.h"
#undef DEFINE_API_VARIABLES

#include "voxmap.h"
#include "voxmap_memdisk.h"


#include "sin_lut.c"
#include "geotables.h"
#include "b2s_prints.c"

#define N_SENSORS 10

#define SWAP(t_, a_, b_) do{t_ tmp_ = a_; a_ = b_; b_ = tmp_;}while(0)


#define DEGTOANG16(x)  ((uint16_t)((float)(x)/(360.0)*65536.0))

/*
#define RADTODEG(x) ((x)*(360.0/(2.0*M_PI)))
#define DEGTORAD(x) ((x)*((2.0*M_PI)/360.0))
#define ANG32TORAD(x) ( ((double)((uint32_t)(x)))/683565275.576432)
#define ANG32TOFDEG(x) ( ((double)((uint32_t)(x)))/11930464.7111111)
*/

#define DIST_UNDEREXP 0
#define DIST_OVEREXP 1

#define HEAP_PARENT(i) (((i) - 1) >> 1)
#define HEAP_LEFT(i)   (((i) << 1) + 1)
#define HEAP_RIGHT(i) (((i) << 1) + 2)

double subsec_timestamp()
{
	struct timespec spec;
	clock_gettime(CLOCK_MONOTONIC, &spec);

	return (double)spec.tv_sec + (double)spec.tv_nsec/1.0e9;
}


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

typedef struct __attribute__((packed))
{
	int32_t start_idx;
	int32_t end_idx;
	int32_t avg_x;
	int32_t avg_y;
	int32_t avg_z;
} subsubmap_meta_t;

#define MAX_SUBSUBMAPS 16

typedef struct  __attribute__((packed))
{
	int32_t start_idx;
	int32_t end_idx;

	int32_t min_x;
	int32_t avg_x;
	int32_t max_x;

	int32_t min_y;
	int32_t avg_y;
	int32_t max_y;

	int32_t min_z;
	int32_t avg_z;
	int32_t max_z;

	int32_t n_subsubmaps;
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

int start_i = 0;
int end_i = 27294; //12800;

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


// Pitch and roll are assumed to have no accumulating error (based on gravity vector always available)
// Accumulating yaw error is eventually limited by compass, but compass will be inaccurate and subject
// to local magnetic fields.

#define XY_ACCUM_UNCERT_BY_SUBMAP 100 // mm
#define Z_ACCUM_UNCERT_BY_SUBMAP 50   // mm
#define YAW_ACCUM_UNCERT_BY_SUBMAP (DEGTORAD(1))
#define YAW_ACCUM_UNCERT_SATURATION (DEGTORAD(45))

#define TRY_CLOSURE_MARGIN_XY 5000
#define TRY_CLOSURE_MARGIN_Z  1000

void match_submaps(int i_sma, int i_smb, double xy_range, double z_range, double yaw_range, int dx, int dy, int dz);

void find_possible_closures(submap_meta_t* sms, int n_sms, int cur_sms)
{
	int cur_x = sms[cur_sms].avg_x;
	int cur_y = sms[cur_sms].avg_y;
	int cur_z = sms[cur_sms].avg_z;

//	for(int i=0; i<n_sms; i++)
	for(int i=0; i<cur_sms; i++)
	{
		// Distance of submap sequence numbers:
		int sm_dist = abs(cur_sms - i);

		// Never try to find closures between neighbors (which are trivial matches)
		if(sm_dist < 5)
		{
			continue;
		}

		double xy_uncert = sm_dist * XY_ACCUM_UNCERT_BY_SUBMAP + 512.0;
		double z_uncert = sm_dist * Z_ACCUM_UNCERT_BY_SUBMAP + 512.0;
		double yaw_uncert = sm_dist * YAW_ACCUM_UNCERT_BY_SUBMAP + DEGTORAD(1.5);

		if(yaw_uncert > YAW_ACCUM_UNCERT_SATURATION)
			yaw_uncert = YAW_ACCUM_UNCERT_SATURATION;


		int smi_x = sms[i].avg_x;
		int smi_y = sms[i].avg_y;
		int smi_z = sms[i].avg_z;

		if(abs(smi_x - cur_x) > xy_uncert + TRY_CLOSURE_MARGIN_XY)
			continue;

		if(abs(smi_y - cur_y) > xy_uncert + TRY_CLOSURE_MARGIN_XY)
			continue;

		if(abs(smi_z - cur_z) > z_uncert + TRY_CLOSURE_MARGIN_Z)
			continue;

		// Submaps themselves have data around zero - (0,0,0) corresponding world coordinates are its avg_x,avg_y,avg_z.
		// For the matcher, submap a (ref) is given as is (around 0 coordinates),
		// submap b (cmp) is translated during matching, and difference between the avg_ coords is added on top of any
		// running matching correction.
		int dx = cur_x - smi_x;
		int dy = cur_y - smi_y;
		int dz = cur_z - smi_z;

		printf("   LOOP    Cur submap %d (%d,%d,%d), possible loop closure with submap %d (%d,%d,%d), xy_uncert=%.0f, z_uncert=%.0f, dx=%d, dy=%d, dz=%d\n",
			cur_sms, cur_x, cur_y, cur_z, i, smi_x, smi_y, smi_z, xy_uncert, z_uncert, dx, dy, dz);

		match_submaps(cur_sms, i, xy_uncert, z_uncert, yaw_uncert, dx, dy, dz);
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
	int dx_limit = 3500;
	int dy_limit = 3500;
	int dz_limit = 1500;

	int overlap_dx_limit = 2500;
	int overlap_dy_limit = 2500;
	int overlap_dz_limit = 1000;

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

			// Group subsubmaps:
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

			// Check if we need to finish the current submap:
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

typedef struct __attribute__((packed))
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


#define MAX_POINTS (9600*10*64)
typedef struct
{
	int n_points;
	tmp_cloud_point_t points[MAX_POINTS];
} tmp_cloud_t;

// Point cloud files contain source sequence indeces, meaning they are only valid with the exact same input data set.
int save_tmp_cloud(tmp_cloud_t* cloud, int idx)
{
	char fname[1024];
	snprintf(fname, 1024, "submap%06u", idx);
	FILE* f = fopen(fname, "wb");
	assert(f);

	uint32_t n_points = cloud->n_points;
	assert(fwrite(&n_points, sizeof(uint32_t), 1, f) == 1);
	assert(fwrite(cloud->points, sizeof(tmp_cloud_point_t), n_points, f) == n_points);

	fclose(f);
	return 0;
}

int load_tmp_cloud(tmp_cloud_t* cloud, int idx)
{
	char fname[1024];
	snprintf(fname, 1024, "submap%06u", idx);
	FILE* f = fopen(fname, "rb");
	assert(f);

	uint32_t n_points = 0;
	assert(fread(&n_points, sizeof(uint32_t), 1, f) == 1);
	cloud->n_points = n_points;
	assert(fread(cloud->points, sizeof(tmp_cloud_point_t), n_points, f) == n_points);

	fclose(f);
	return 0;
}

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

static inline void output_voxel_cloudflt(int x, int y, int z)
{
	assert(x >= 1 && x < CLOUDFLT_XS-1 && y >= 1 && y < CLOUDFLT_YS-1 && z > 1 && z <= CLOUDFLT_ZS-1);

//	if(free_cnt[x][y][z] < 255)
//		free_cnt[x][y][z]++;
	free_cnt[x][y][z] = 1;
}

static inline uint8_t get_voxel_cloudflt(int x, int y, int z)
{
	assert(x >= 0 && x < CLOUDFLT_XS && y >= 0 && y < CLOUDFLT_YS && z > 0 && z <= CLOUDFLT_ZS);

	return free_cnt[x][y][z];
}

static void bresenham3d_cloudflt(int x1, int y1, int z1, int x2, int y2, int z2)
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
					output_voxel_cloudflt(px+ix, py+iy, pz+iz);
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
					output_voxel_cloudflt(px+ix, py+iy, pz+iz);
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
					output_voxel_cloudflt(px+ix, py+iy, pz+iz);
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
		bresenham3d_cloudflt(sx, sy, sz, px, py, pz);
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

		int val = get_voxel_cloudflt(px, py, pz);
		if(val < 1)
		{
			out->points[n_p] = cloud->points[p];
			n_p++;
		}
			
	}
	out->n_points = n_p;

}

typedef struct
{
	uint64_t occu;
	uint64_t free;
} ref_matchmap_unit_t;

typedef struct
{
	uint64_t occu;
	uint64_t free;
} ref_fine_matchmap_unit_t;


#define MATCHMAP_XS 256
#define MATCHMAP_YS 256
#define MATCHMAP_ZS 64

#define FINE_MATCHMAP_XS 384
#define FINE_MATCHMAP_YS 384
#define FINE_MATCHMAP_ZS 64

/*
	ZS is fixed at 64 (single bits in uint64)

	256x256x64 units:

	At 256mm resolution: 65.536 m x 65.536 m x 16.384 m
	Assuming centered submap range of 45m x 45m x 10m,
	wiggle room of +/- 12.5m, 12.5m, 3m
*/

typedef struct
{
	ref_matchmap_unit_t units[MATCHMAP_YS][MATCHMAP_XS];
} ref_matchmap_t;

typedef struct
{
	ref_fine_matchmap_unit_t units[FINE_MATCHMAP_YS][FINE_MATCHMAP_XS];
} ref_fine_matchmap_t;

typedef struct
{
	uint64_t occu;
} cmp_matchmap_unit_t;

typedef struct
{
	uint64_t occu;
} cmp_fine_matchmap_unit_t;

typedef struct
{
	cmp_matchmap_unit_t units[MATCHMAP_YS][MATCHMAP_XS];
} cmp_matchmap_t;

typedef struct
{
	cmp_fine_matchmap_unit_t units[FINE_MATCHMAP_YS][FINE_MATCHMAP_XS];
} cmp_fine_matchmap_t;


ref_matchmap_t ref_matchmap __attribute__((aligned(64)));
cmp_matchmap_t cmp_matchmap __attribute__((aligned(64)));

ref_fine_matchmap_t ref_fine_matchmap __attribute__((aligned(64)));
cmp_fine_matchmap_t cmp_fine_matchmap __attribute__((aligned(64)));

static inline int count_ones_u64(uint64_t in)
{
	int cnt;
#if 0
	cnt = 0;
	for(int i=0; i<64; i++)
	{
		if(in & 1)
			cnt++;
		in >>= 1;
	}
#endif

#if 1
	// Massively faster

	// https://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel

	in = in - ((in >> 1) & 0x5555555555555555ULL);
	in = (in & 0x3333333333333333ULL) + ((in >> 2) & 0x3333333333333333ULL);
	cnt = (((in + (in >> 4)) & 0x0F0F0F0F0F0F0F0FULL) * 0x0101010101010101ULL) >> 56;

#endif

	return cnt;

}


/*
	Raspi3 has 16kB of L1 data cache per core; 512 kB shared L2 cache.
	Odroid XU4 has 32kB of L1 data cacge per core; 2MB shared L2 cache for fast cores, 512kB shared for slow cores

	Cache line is 64 bytes for L1 and L2

	For L1, 256 lines * 64 bytes = 16kB

	x_range, y_range = +/-16, inmost loops use half of the L1 cache (32*32*8bytes), in 32 * 4 = 128 lines
	prefetch should work fairly well with inmost loop being in x direction
*/
/*
	Range example:
	x_range = 16 (biggest allowed)
	X shifts are looped from -16 to +15
	Matchmaps are compared from 16 to MATCHMAP_XS-16
*/

#define ODROID_XU4
//#define RASPI3

// MATCHMAP_*_RANGE includes lower bound, doesn't include upper bound, i.e., RANGE=12 scans from -12 to +11
// FINE_MATCHMAP_*_RANGE, on the other hand, scans for example, from -8 to +8, given RANGE=8.
// L1 cache usage = (2*xy_range)^2 * 16  (8 bytes for voxels; 8 bytes for score counters)
// 2*XY_RANGE*8 should be multiple of 64 (cache line length)
// xy_range is usually limited by architecture (to fit in L1 cache), but may be limited by comparison range
//   (should be much smaller than MATCHMAP_XS/2, otherwise there will be clipping of comparison space)
// z_range is not limited by architecture, just to avoid clipping (so should be much smaller than MATCHMAP_ZS/2, i.e. 64/2)
#ifdef RASPI3 
#define MATCHMAP_XY_RANGE 12 // 9216 bytes out of 16384 L1 cache
#define MATCHMAP_Z_RANGE  12
#endif

#ifdef ODROID_XU4
#define MATCHMAP_XY_RANGE 20 // 25600 bytes out of 32768 L1 cache
#define MATCHMAP_Z_RANGE  12
#endif

#define FINE_MATCHMAP_XY_RANGE 8 // 1024mm
#define FINE_MATCHMAP_Z_RANGE  2

#define MATCHMAP_XYZ_N_RESULTS 64

// 100% match with no occu_on_free will be score=10000 (OCCU_MATCH_COEFF*REL_SCORE_MULT)
#define OCCU_MATCH_COEFF 1 // 1 is good for efficiency
#define OCCU_ON_FREE_COEFF -4
#define REL_SCORE_MULT 10000

#define FINE_REL_SCORE_MULT 10000


typedef struct __attribute__((packed))
{
	int8_t dummy;
	int8_t x_shift;
	int8_t y_shift;
	int8_t z_shift;

	int32_t score;

} match_xyz_result_t;


//#define PREFILTER_COMBINE_XYZ_RESULTS

static void match_matchmaps_xyz(int z_start, int z_end, match_xyz_result_t* p_results)
{
	printf("match_matchmaps_xyz(%d, %d)... ", z_start, z_end); fflush(stdout);
	double start_time = subsec_timestamp();

	assert(z_start >= -MATCHMAP_Z_RANGE && z_end <= MATCHMAP_Z_RANGE);

	// Bit counts: max 256*256*64 = 4194304 voxels (this is the maximum count for each alignment)
	// Fits int32 with massive margin

	// Scoring: order:[z][y][x], L2 cache or RAM acceptable
	// fixed-z [y][x], L1 cache preferred
	// the two score counters are next to each other, for cache coherency and 64-bit SIMD addition
	int32_t scores[2*MATCHMAP_Z_RANGE][2*MATCHMAP_XY_RANGE][2*MATCHMAP_XY_RANGE] __attribute__((aligned(64))); 

	memset(scores, 0, sizeof scores);

	for(int iz = z_start; iz < z_end; iz++)
	{
		for(int yy = MATCHMAP_XY_RANGE; yy < MATCHMAP_YS-MATCHMAP_XY_RANGE; yy++)
		{
			for(int xx = MATCHMAP_XY_RANGE; xx < MATCHMAP_XS-MATCHMAP_XY_RANGE; xx++)
			{
				uint64_t ref_occu = (iz < 0) ? 
					(ref_matchmap.units[yy][xx].occu >> (-1*iz)) :
					(ref_matchmap.units[yy][xx].occu << iz) ;

				uint64_t ref_free = (iz < 0) ? 
					(ref_matchmap.units[yy][xx].free >> (-1*iz)) :
					(ref_matchmap.units[yy][xx].free << iz) ;

				for(int iy = -MATCHMAP_XY_RANGE; iy < MATCHMAP_XY_RANGE; iy++)
				{
					for(int ix = -MATCHMAP_XY_RANGE; ix < MATCHMAP_XY_RANGE; ix++)
					{
						uint64_t cmp_occu = cmp_matchmap.units[yy+iy][xx+ix].occu;

						uint64_t occu_matches = cmp_occu & ref_occu;
						uint64_t occu_on_free = cmp_occu & ref_free;

						int32_t cur_n_occu_matches = count_ones_u64(occu_matches);
						int32_t cur_n_occu_on_free = count_ones_u64(occu_on_free);
						int32_t cur_score = OCCU_MATCH_COEFF*cur_n_occu_matches + OCCU_ON_FREE_COEFF*cur_n_occu_on_free;

						scores[iz+MATCHMAP_Z_RANGE][iy+MATCHMAP_XY_RANGE][ix+MATCHMAP_XY_RANGE] += cur_score;
					}
				}
			}

		}
	}


//		memset(p_results, 0, sizeof(match_xyz_result_t)*MATCHMAP_XYZ_N_RESULTS);

	// result array works as binary min heap

	int heap_size = 0;

	for(int iz = z_start; iz < z_end; iz++)
	{
		for(int iy = -MATCHMAP_XY_RANGE; iy < MATCHMAP_XY_RANGE; iy++)
		{
			for(int ix = -MATCHMAP_XY_RANGE; ix < MATCHMAP_XY_RANGE; ix++)
			{
				// occu_on_free *= -1, so that larger is better
				int32_t cur_score = scores[iz+MATCHMAP_Z_RANGE][iy+MATCHMAP_XY_RANGE][ix+MATCHMAP_XY_RANGE];

				#ifdef PREFILTER_COMBINE_XYZ_RESULTS

					/*
						If any of the neighbor result is better, ignore this result.
						This reduces number of results by priorizing local maxima.
					*/
					if(iz > z_start && iz < z_end-1 && iy > -MATCHMAP_XY_RANGE && iy < MATCHMAP_XY_RANGE-1 && ix > -MATCHMAP_XY_RANGE && ix < MATCHMAP_XY_RANGE-1)
					{
						int max_neigh = -99999;
						for(int iiz = -1; iiz <= 1; iiz++)
						{
							for(int iiy = -1; iiy <= 1; iiy++)
							{
								for(int iix = -1; iix <= 1; iix++)
								{
									int32_t neigh_score = scores[iz+iiz+MATCHMAP_Z_RANGE][iy+iiy+MATCHMAP_XY_RANGE][ix+iix+MATCHMAP_XY_RANGE];
									if(neigh_score > max_neigh)
										max_neigh = neigh_score;
								}
							}

						}

						if(100*max_neigh > 102*cur_score)
							continue;
					}
				#endif

				if(heap_size < MATCHMAP_XYZ_N_RESULTS || cur_score > p_results[0].score)
				{

					if(heap_size == MATCHMAP_XYZ_N_RESULTS) // full: remove the lowest score
					{
						// Pop element 0

						heap_size--;

						SWAP(match_xyz_result_t, p_results[0], p_results[heap_size]);

						int i = 0;
						while(1)
						{
							int l = HEAP_LEFT(i);
							int r = HEAP_RIGHT(i);
							int smallest_idx = ( (l < heap_size) && (p_results[l].score < p_results[i].score) ) ? l : i;
							if( (r < heap_size) && (p_results[r].score < p_results[smallest_idx].score) )
								smallest_idx = r;

							if(smallest_idx == i)
								break;

							SWAP(match_xyz_result_t, p_results[i], p_results[smallest_idx]);
							i = smallest_idx;
						}
					}

					// Push
					p_results[heap_size] = (match_xyz_result_t){0, ix, iy, iz, cur_score};
					heap_size++;
				}

			}
		}

	}

	double end_time = subsec_timestamp();

	printf("took %.2f ms\n", (end_time-start_time)*1000.0);
}




static void match_fine_matchmaps_xyz(match_xyz_result_t* p_results)
{
	printf("match_fine_matchmaps_xyz... "); fflush(stdout);
	double start_time = subsec_timestamp();

	int32_t scores[2*FINE_MATCHMAP_Z_RANGE+1][2*FINE_MATCHMAP_XY_RANGE+1][2*FINE_MATCHMAP_XY_RANGE+1] __attribute__((aligned(64))); 

	memset(scores, 0, sizeof scores);

	for(int iz = -FINE_MATCHMAP_Z_RANGE; iz <= FINE_MATCHMAP_Z_RANGE; iz++)
	{
		for(int yy = FINE_MATCHMAP_XY_RANGE; yy < FINE_MATCHMAP_YS-FINE_MATCHMAP_XY_RANGE-1; yy++)
		{
			for(int xx = FINE_MATCHMAP_XY_RANGE; xx < FINE_MATCHMAP_XS-FINE_MATCHMAP_XY_RANGE-1; xx++) // xx=8, xx < 375 (374 last)
			{
				uint64_t ref_occu = (iz < 0) ? 
					(ref_fine_matchmap.units[yy][xx].occu >> (-1*iz)) :
					(ref_fine_matchmap.units[yy][xx].occu << iz) ;

				uint64_t ref_free = (iz < 0) ? 
					(ref_fine_matchmap.units[yy][xx].free >> (-1*iz)) :
					(ref_fine_matchmap.units[yy][xx].free << iz) ;

				for(int iy = -FINE_MATCHMAP_XY_RANGE; iy <= FINE_MATCHMAP_XY_RANGE; iy++)
				{
					for(int ix = -FINE_MATCHMAP_XY_RANGE; ix <= FINE_MATCHMAP_XY_RANGE; ix++) // ix=-8, ix<=+8 (+8 last)
					{
						uint64_t cmp_occu = cmp_fine_matchmap.units[yy+iy][xx+ix].occu; // xx+ix: 0..16;  366..382

						uint64_t occu_matches = cmp_occu & ref_occu;
						uint64_t occu_on_free = cmp_occu & ref_free;

						int32_t cur_n_occu_matches = count_ones_u64(occu_matches);
						int32_t cur_n_occu_on_free = count_ones_u64(occu_on_free);
						int32_t cur_score = OCCU_MATCH_COEFF*cur_n_occu_matches + OCCU_ON_FREE_COEFF*cur_n_occu_on_free;

						scores[iz+FINE_MATCHMAP_Z_RANGE][iy+FINE_MATCHMAP_XY_RANGE][ix+FINE_MATCHMAP_XY_RANGE] += cur_score;
					}
				}
			}

		}
	}

	// result array works as binary min heap

//	static const int32_t xy_weights[FINE_MATCHMAP_XY_RANGE*2] = {42,43,44,45,46,47,48,49,50,49,48,46,44,42,40,38,36};
//	static const int32_t z_weights[FINE_MATCHMAP_Z_RANGE*2] = {8, 9, 10, 9, 8};

	int heap_size = 0;

	for(int iz = -FINE_MATCHMAP_Z_RANGE; iz <= FINE_MATCHMAP_Z_RANGE; iz++)
	{
		for(int iy = -FINE_MATCHMAP_XY_RANGE; iy <= FINE_MATCHMAP_XY_RANGE; iy++)
		{
			for(int ix = -FINE_MATCHMAP_XY_RANGE; ix <= FINE_MATCHMAP_XY_RANGE; ix++)
			{
				// occu_on_free *= -1, so that larger is better
				int32_t cur_score = scores[iz+FINE_MATCHMAP_Z_RANGE][iy+FINE_MATCHMAP_XY_RANGE][ix+FINE_MATCHMAP_XY_RANGE];

				//cur_score *= xy_weights[iy+FINE_MATCHMAP_XY_RANGE] * xy_weights[ix+FINE_MATCHMAP_XY_RANGE] * z_weights[iz+FINE_MATCHMAP_Z_RANGE];

				#ifdef PREFILTER_FINE_COMBINE_XYZ_RESULTS

					/*
						If any of the neighbor result is better, ignore this result.
						This reduces number of results by priorizing local maxima.
					*/
					if(iz > -FINE_MATCHMAP_Z_RANGE && iz < FINE_MATCHMAP_Z_RANGE-1 && iy > -FINE_MATCHMAP_XY_RANGE && iy < FINE_MATCHMAP_XY_RANGE-1 && ix > -FINE_MATCHMAP_XY_RANGE && ix < FINE_MATCHMAP_XY_RANGE-1)
					{
						int max_neigh = -99999;
						for(int iiz = -1; iiz <= 1; iiz++)
						{
							for(int iiy = -1; iiy <= 1; iiy++)
							{
								for(int iix = -1; iix <= 1; iix++)
								{
									int32_t neigh_score = scores[iz+iiz+FINE_MATCHMAP_Z_RANGE][iy+iiy+FINE_MATCHMAP_XY_RANGE][ix+iix+FINE_MATCHMAP_XY_RANGE];
									if(neigh_score > max_neigh)
										max_neigh = neigh_score;
								}
							}

						}

						if(100*max_neigh > 102*cur_score)
							continue;
					}
				#endif

				if(heap_size < MATCHMAP_XYZ_N_RESULTS || cur_score > p_results[0].score)
				{

					if(heap_size == MATCHMAP_XYZ_N_RESULTS) // full: remove the lowest score
					{
						// Pop element 0

						heap_size--;

						SWAP(match_xyz_result_t, p_results[0], p_results[heap_size]);

						int i = 0;
						while(1)
						{
							int l = HEAP_LEFT(i);
							int r = HEAP_RIGHT(i);
							int smallest_idx = ( (l < heap_size) && (p_results[l].score < p_results[i].score) ) ? l : i;
							if( (r < heap_size) && (p_results[r].score < p_results[smallest_idx].score) )
								smallest_idx = r;

							if(smallest_idx == i)
								break;

							SWAP(match_xyz_result_t, p_results[i], p_results[smallest_idx]);
							i = smallest_idx;
						}
					}

					// Push
					p_results[heap_size] = (match_xyz_result_t){0, ix, iy, iz, cur_score};
					heap_size++;
				}

			}
		}

	}

	double end_time = subsec_timestamp();

	printf("took %.2f ms\n", (end_time-start_time)*1000.0);
}




static inline void output_voxel_ref_matchmap(int x, int y, int z)
{
//	assert(x >= 0 && x < MATCHMAP_XS-0 && y >= 0 && y < MATCHMAP_YS-0 && z > 0 && z < MATCHMAP_ZS-0);
	if(x >= 0 && x < MATCHMAP_XS-0 && y >= 0 && y < MATCHMAP_YS-0 && z > 0 && z < MATCHMAP_ZS-0)
		ref_matchmap.units[y][x].free |= 1UL<<z;
}

static inline void output_voxel_ref_fine_matchmap(int x, int y, int z)
{
//	assert(x >= 0 && x < FINE_MATCHMAP_XS-0 && y >= 0 && y < FINE_MATCHMAP_YS-0 && z > 0 && z < FINE_MATCHMAP_ZS-0);

	if(x >= 0 && x < FINE_MATCHMAP_XS-0 && y >= 0 && y < FINE_MATCHMAP_YS-0 && z > 0 && z < FINE_MATCHMAP_ZS-0)
		ref_fine_matchmap.units[y][x].free |= 1UL<<z;
}



#define MATCHMAP_UNIT 256 //mm

#define FINE_MATCHMAP_UNIT 128 //mm


#define MATCHMAP_UNCERT_Z 1
#define MATCHMAP_UNCERT 1
#define MATCHMAP_LINE_CONE_SLOPE_XY 60
#define MATCHMAP_LINE_CONE_SLOPE_Z  80

#define FINE_MATCHMAP_UNCERT_Z 1
#define FINE_MATCHMAP_UNCERT 1
#define FINE_MATCHMAP_LINE_CONE_SLOPE_XY 60
#define FINE_MATCHMAP_LINE_CONE_SLOPE_Z  80

static void bresenham3d_ref_matchmap(int x1, int y1, int z1, int x2, int y2, int z2)
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
		for(int i = 0; i < l-MATCHMAP_UNCERT; i++)
		{
			if(abs(px-x2) <= MATCHMAP_UNCERT || abs(py-y2) <= MATCHMAP_UNCERT || abs(pz-z2) <= MATCHMAP_UNCERT_Z)
				return;

			int rangexy = i/MATCHMAP_LINE_CONE_SLOPE_XY;
			int rangez = i/MATCHMAP_LINE_CONE_SLOPE_Z;
			for(int iy=-rangexy; iy<=rangexy; iy++)
			{
				int ix=0;
				for(int iz=-rangez; iz<=rangez; iz++)
				{
					output_voxel_ref_matchmap(px+ix, py+iy, pz+iz);
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
		for(int i = 0; i < m-MATCHMAP_UNCERT; i++)
		{
			if(abs(px-x2) <= MATCHMAP_UNCERT || abs(py-y2) <= MATCHMAP_UNCERT || abs(pz-z2) <= MATCHMAP_UNCERT)
				return;

			int rangexy = i/MATCHMAP_LINE_CONE_SLOPE_XY;
			int rangez = i/MATCHMAP_LINE_CONE_SLOPE_Z;

			int iy=0;
			for(int ix=-rangexy; ix<=rangexy; ix++)
			{
				for(int iz=-rangez; iz<=rangez; iz++)
				{
					output_voxel_ref_matchmap(px+ix, py+iy, pz+iz);
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
		for(int i = 0; i < n-MATCHMAP_UNCERT; i++)
		{
			if(abs(px-x2) <= MATCHMAP_UNCERT || abs(py-y2) <= MATCHMAP_UNCERT || abs(pz-z2) <= MATCHMAP_UNCERT)
				return;

			int rangexy = i/MATCHMAP_LINE_CONE_SLOPE_XY;
			int rangez = i/MATCHMAP_LINE_CONE_SLOPE_Z;
			for(int iy=-rangexy; iy<=rangexy; iy++)
			{
				for(int ix=-rangexy; ix<=rangexy; ix++)
				{
					int iz=0;
					output_voxel_ref_matchmap(px+ix, py+iy, pz+iz);
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



static void bresenham3d_ref_fine_matchmap(int x1, int y1, int z1, int x2, int y2, int z2)
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
		for(int i = 0; i < l-FINE_MATCHMAP_UNCERT; i++)
		{
			if(abs(px-x2) <= FINE_MATCHMAP_UNCERT || abs(py-y2) <= FINE_MATCHMAP_UNCERT || abs(pz-z2) <= FINE_MATCHMAP_UNCERT_Z)
				return;

			int rangexy = i/FINE_MATCHMAP_LINE_CONE_SLOPE_XY;
			int rangez = i/FINE_MATCHMAP_LINE_CONE_SLOPE_Z;
			for(int iy=-rangexy; iy<=rangexy; iy++)
			{
				int ix=0;
				for(int iz=-rangez; iz<=rangez; iz++)
				{
					output_voxel_ref_fine_matchmap(px+ix, py+iy, pz+iz);
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
		for(int i = 0; i < m-FINE_MATCHMAP_UNCERT; i++)
		{
			if(abs(px-x2) <= FINE_MATCHMAP_UNCERT || abs(py-y2) <= FINE_MATCHMAP_UNCERT || abs(pz-z2) <= FINE_MATCHMAP_UNCERT)
				return;

			int rangexy = i/FINE_MATCHMAP_LINE_CONE_SLOPE_XY;
			int rangez = i/FINE_MATCHMAP_LINE_CONE_SLOPE_Z;

			int iy=0;
			for(int ix=-rangexy; ix<=rangexy; ix++)
			{
				for(int iz=-rangez; iz<=rangez; iz++)
				{
					output_voxel_ref_fine_matchmap(px+ix, py+iy, pz+iz);
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
		for(int i = 0; i < n-FINE_MATCHMAP_UNCERT; i++)
		{
			if(abs(px-x2) <= FINE_MATCHMAP_UNCERT || abs(py-y2) <= FINE_MATCHMAP_UNCERT || abs(pz-z2) <= FINE_MATCHMAP_UNCERT)
				return;

			int rangexy = i/FINE_MATCHMAP_LINE_CONE_SLOPE_XY;
			int rangez = i/FINE_MATCHMAP_LINE_CONE_SLOPE_Z;
			for(int iy=-rangexy; iy<=rangexy; iy++)
			{
				for(int ix=-rangexy; ix<=rangexy; ix++)
				{
					int iz=0;
					output_voxel_ref_fine_matchmap(px+ix, py+iy, pz+iz);
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



// remember that tmp_clouds are centered on zero (so that the ref_x,y,z from the submap meta corresponds to 0,0,0)
// This zero center is good for rotational center as well.

/*
	For every point, it's supposed surface normal is approximated,
	simply as the vector from sensor to the point.

	You can choose whether to include all points, or only fairly vertical surfaces (walls, etc.),
	or only fairly horizontal surfaces (floors and ceilings)

	Empty space tracing always works for all points, which is always beneficial for matching.
*/
#define NORM_FLT_MODE_ALL 0
#define NORM_FLT_MODE_WALLS_ONLY 1
#define NORM_FLT_MODE_FLOOR_ONLY 2


// Returns number of occupied voxels, for later calculation of relative score (percentage of matches)
static int cloud_to_cmp_matchmap(tmp_cloud_t* cloud, cmp_matchmap_t* matchmap, double x_corr, double y_corr, double z_corr, double yaw_corr)
{
	printf("cloud_to_cmp_matchmap, adding %d points\n", cloud->n_points);

	double cosa = cos(yaw_corr);
	double sina = sin(yaw_corr);

	int vox_cnt = 0;
	for(int p=0; p<cloud->n_points; p++)
	{
		double x_in = cloud->points[p].x;
		double y_in = cloud->points[p].y;
		double z_in = cloud->points[p].z;

		double x = x_in*cosa - y_in*sina + x_corr;
		double y = x_in*sina + y_in*cosa + y_corr;
		double z = z_in + z_corr;

		int px = x/MATCHMAP_UNIT + MATCHMAP_XS/2;
		int py = y/MATCHMAP_UNIT + MATCHMAP_YS/2;
		int pz = z/MATCHMAP_UNIT + MATCHMAP_ZS/2;

		if(px < 1 || px > MATCHMAP_XS-2 || py < 1 || py > MATCHMAP_YS-2 || pz < 1 || pz > MATCHMAP_ZS-2)
		{
			printf("cloud_to_cmp_matchmap: WARN, OOR point %d,%d,%d\n", px, py, pz);
			continue;
		}

		if(!(matchmap->units[py][px].occu & (1ULL<<pz)))
		{
			// A new point
			vox_cnt++;
		}

		matchmap->units[py][px].occu |= 1ULL<<pz;
	}
	return vox_cnt;
}

static int cloud_to_cmp_fine_matchmap(tmp_cloud_t* cloud, cmp_fine_matchmap_t* matchmap, double x_corr, double y_corr, double z_corr, double yaw_corr)
{
	printf("cloud_to_cmp_fine_matchmap, adding %d points\n", cloud->n_points);

	double cosa = cos(yaw_corr);
	double sina = sin(yaw_corr);

	int vox_cnt = 0;
	int oor = 0;
	for(int p=0; p<cloud->n_points; p++)
	{
		double x_in = cloud->points[p].x;
		double y_in = cloud->points[p].y;
		double z_in = cloud->points[p].z;

		double x = x_in*cosa - y_in*sina + x_corr;
		double y = x_in*sina + y_in*cosa + y_corr;
		double z = z_in + z_corr;

		int px = x/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_XS/2;
		int py = y/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_YS/2;
		int pz = z/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_ZS/2;

		if(px < 1 || px > FINE_MATCHMAP_XS-2 || py < 1 || py > FINE_MATCHMAP_YS-2 || pz < 1 || pz > FINE_MATCHMAP_ZS-2)
		{
//			printf("cloud_to_cmp_fine_matchmap: WARN, OOR point %d,%d,%d\n", px, py, pz);
			oor++;
			continue;
		}

		if(!(matchmap->units[py][px].occu & (1ULL<<pz)))
		{
			// A new point
			vox_cnt++;
		}

		matchmap->units[py][px].occu |= 1ULL<<pz;
	}

	if(oor > cloud->n_points/5)
	{
		printf("cloud_to_cmp_fine_matchmap: WARN: over 20%% OOR points\n");
	}
	return vox_cnt;
}


#define REF_MATCHMAP_OCCUPY_3X3X3

static void cloud_to_ref_matchmap(tmp_cloud_t* cloud, ref_matchmap_t* matchmap)
{
	printf("cloud_to_ref_matchmap, adding %d points\n", cloud->n_points);
	// Trace empty space:
	for(int p=0; p<cloud->n_points; p++)
	{
		int seq_id = cloud->points[p].src_seq_id;

		int sx = (ray_sources[seq_id].x)/MATCHMAP_UNIT + MATCHMAP_XS/2;
		int sy = (ray_sources[seq_id].y)/MATCHMAP_UNIT + MATCHMAP_YS/2;
		int sz = (ray_sources[seq_id].z)/MATCHMAP_UNIT + MATCHMAP_ZS/2;

		int px = (cloud->points[p].x)/MATCHMAP_UNIT + MATCHMAP_XS/2;
		int py = (cloud->points[p].y)/MATCHMAP_UNIT + MATCHMAP_YS/2;
		int pz = (cloud->points[p].z)/MATCHMAP_UNIT + MATCHMAP_ZS/2;

		bresenham3d_ref_matchmap(sx, sy, sz, px, py, pz);
	}

	// Add points, and remove empty space around them.

	for(int p=0; p<cloud->n_points; p++)
	{
		int px = (cloud->points[p].x)/MATCHMAP_UNIT + MATCHMAP_XS/2;
		int py = (cloud->points[p].y)/MATCHMAP_UNIT + MATCHMAP_YS/2;
		int pz = (cloud->points[p].z)/MATCHMAP_UNIT + MATCHMAP_ZS/2;

		if(px < 1 || px > MATCHMAP_XS-2 || py < 1 || py > MATCHMAP_YS-2 || pz < 1 || pz > MATCHMAP_ZS-2)
		{
			printf("cloud_to_ref_matchmap: WARN, OOR point %d,%d,%d\n", px, py, pz);
			continue;
		}

		#ifdef REF_MATCHMAP_OCCUPY_3X3X3
			for(int ix=-1; ix<=1; ix++)
			{
				for(int iy=-1; iy<=1; iy++)
				{
					matchmap->units[py+iy][px+ix].occu |= 0b111ULL << (pz-1);
				}
			}
		#else
			matchmap->units[py][px].occu |= (1ULL << pz);
		#endif

		// Remove empty space:
		for(int ix=-1; ix<=1; ix++)
		{
			for(int iy=-1; iy<=1; iy++)
			{
				matchmap->units[py+iy][px+ix].free &= ~(0b111ULL << (pz-1));
			}
		}

	}
}

static void cloud_to_ref_fine_matchmap(tmp_cloud_t* cloud, ref_fine_matchmap_t* matchmap)
{
	printf("cloud_to_ref_fine_matchmap, adding %d points\n", cloud->n_points);
	// Trace empty space:
	for(int p=0; p<cloud->n_points; p++)
	{
		int seq_id = cloud->points[p].src_seq_id;

		int sx = (ray_sources[seq_id].x)/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_XS/2;
		int sy = (ray_sources[seq_id].y)/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_YS/2;
		int sz = (ray_sources[seq_id].z)/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_ZS/2;

		int px = (cloud->points[p].x)/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_XS/2;
		int py = (cloud->points[p].y)/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_YS/2;
		int pz = (cloud->points[p].z)/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_ZS/2;

		bresenham3d_ref_fine_matchmap(sx, sy, sz, px, py, pz);
	}

	// Add points, and remove empty space around them.
	int oor = 0;
	for(int p=0; p<cloud->n_points; p++)
	{
		int px = (cloud->points[p].x)/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_XS/2;
		int py = (cloud->points[p].y)/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_YS/2;
		int pz = (cloud->points[p].z)/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_ZS/2;

		if(px < 1 || px > FINE_MATCHMAP_XS-2 || py < 1 || py > FINE_MATCHMAP_YS-2 || pz < 1 || pz > FINE_MATCHMAP_ZS-2)
		{
			//printf("cloud_to_ref_fine_matchmap: WARN, OOR point %d,%d,%d\n", px, py, pz);
			oor++;
			continue;
		}

		#ifdef REF_MATCHMAP_OCCUPY_3X3X3
			for(int ix=-1; ix<=1; ix++)
			{
				for(int iy=-1; iy<=1; iy++)
				{
					matchmap->units[py+iy][px+ix].occu |= 0b111ULL << (pz-1);
				}
			}
		#else
			matchmap->units[py][px].occu |= (1ULL << pz);
		#endif

		// Remove empty space:
		for(int ix=-1; ix<=1; ix++)
		{
			for(int iy=-1; iy<=1; iy++)
			{
				matchmap->units[py+iy][px+ix].free &= ~(0b111ULL << (pz-1));
			}
		}

	}

	if(oor > cloud->n_points/5)
	{
		printf("cloud_to_ref_fine_matchmap: WARN: over 20%% OOR points\n");
	}

}


typedef struct
{
	int32_t x;
	int32_t y;
	int32_t z;
	float yaw;

	int32_t score;
} result_t;

static int compar_scores(const void* a, const void* b)
{
	if(((result_t*)a)->score > ((result_t*)b)->score)
		return -1;
	else if(((result_t*)a)->score < ((result_t*)b)->score)
		return 1;

	return 0;
}

void match_submaps(int i_sma, int i_smb,  // Ref and cmp submap indeces
                   double xy_range, double z_range, double yaw_range, // Matching correction ranges (mm, rad)
                   int dx, int dy, int dz) // World coordinate midpoint differences between ref and cmp
{
	static tmp_cloud_t sma, smb;

	// TODO: try removing memsets, shouldn't matter.
	memset(&sma, 0, sizeof(tmp_cloud_t));
	memset(&smb, 0, sizeof(tmp_cloud_t));

	load_tmp_cloud(&sma, i_sma);
	load_tmp_cloud(&smb, i_smb);

	// SMA is ref_matchmap
	// SMB is cmp_matchmap

	// ref_matchmap includes free space, so is slower to calculate
	// cmp_matchmap only includes occupied space, and is compared against ref_matchmap quickly

	memset(&ref_matchmap, 0, sizeof(ref_matchmap));
	cloud_to_ref_matchmap(&sma, &ref_matchmap);


	int xy_batches = ceil(xy_range/((double)MATCHMAP_XY_RANGE*MATCHMAP_UNIT));
	int z_batches  = ceil(z_range/((double)MATCHMAP_Z_RANGE*MATCHMAP_UNIT));

	assert(xy_batches > 0 && z_batches > 0);

	int total_xy_steps = xy_batches * MATCHMAP_XY_RANGE;
	int total_z_steps  = z_batches * MATCHMAP_Z_RANGE;

	int z_start = -MATCHMAP_Z_RANGE;
	int z_end   = +MATCHMAP_Z_RANGE;

	if(z_batches == 1)
	{
		z_start = -z_range/MATCHMAP_UNIT;
		z_end = +z_range/MATCHMAP_UNIT;
	}

	// 1 deg step is 260mm shift at 15m distance
	// 1.5 deg step is 390mm shift at 15m distance
	double yaw_step = DEGTORAD(1.5);

	// Round yaw_range up so that midpoint is at zero.
	yaw_range = ceil(yaw_range/yaw_step) * yaw_step;
	int yaw_steps = ceil(2.0*yaw_range/yaw_step) + 1;


	int total_batches = yaw_steps * xy_batches * xy_batches * z_batches;
	int total_results = total_batches * MATCHMAP_XYZ_N_RESULTS;

	result_t* results = calloc(total_results, sizeof(result_t));	
	assert(results);

	// TODO: These batches could run in parallel, in multiple threads
	for(int cur_yaw_step = 0; cur_yaw_step < yaw_steps; cur_yaw_step++)
	{
		double cur_yaw_corr = -yaw_range + (double)cur_yaw_step*yaw_step;
		for(int cur_y_batch = 0; cur_y_batch < xy_batches; cur_y_batch++)
		{
			for(int cur_x_batch = 0; cur_x_batch < xy_batches; cur_x_batch++)
			{
				for(int cur_z_batch = 0; cur_z_batch < z_batches; cur_z_batch++)
				{
					// This part is per-thread.

					// cur_*_corrs are very coarse corrections in mm, for the midpoint of the
					// complete batch. For example, with MATCHMAP_UNIT=256mm and MATCHMAP_XY_RANGE=20,
					// cur_x_corr and cur_y_corr is 0 when 1 batch is enough, -2560 then +2560 when
					// 2 batches are needed, or -5120, 0, then +5120 if 3 batches, and so on.
					double cur_x_corr = -(total_xy_steps*MATCHMAP_UNIT)/2
					                    + cur_x_batch*MATCHMAP_XY_RANGE*MATCHMAP_UNIT
					                    + MATCHMAP_XY_RANGE*MATCHMAP_UNIT/2;

					double cur_y_corr = -(total_xy_steps*MATCHMAP_UNIT)/2
					                    + cur_y_batch*MATCHMAP_XY_RANGE*MATCHMAP_UNIT
					                    + MATCHMAP_XY_RANGE*MATCHMAP_UNIT/2;

					double cur_z_corr = -(total_z_steps*MATCHMAP_UNIT)/2
					                    + cur_z_batch*MATCHMAP_Z_RANGE*MATCHMAP_UNIT
					                    + MATCHMAP_Z_RANGE*MATCHMAP_UNIT/2;

					double cx = cur_x_corr - (double)dx;
					double cy = cur_y_corr - (double)dy;
					double cz = cur_z_corr - (double)dz;

					if(fabs(cx) > 15000.0 || fabs(cy) > 15000.0 || fabs(cz) > 8000.0)
						continue;

					printf("  Matching %4d vs %4d: corr (%.0f, %.0f, %.0f, %.2f) shift (%.0f, %.0f, %.0f, %.2f)\n", 
						i_sma, i_smb,
						cur_x_corr, cur_y_corr, cur_z_corr, RADTODEG(cur_yaw_corr),
						cx, cy, cz, RADTODEG(cur_yaw_corr));

					memset(&cmp_matchmap, 0, sizeof(cmp_matchmap));

					int n_vox = cloud_to_cmp_matchmap(&smb, &cmp_matchmap, cx, cy, cz, cur_yaw_corr);

					printf("%d active voxels in cmp_matchmap\n", n_vox);

					match_xyz_result_t batch_results[MATCHMAP_XYZ_N_RESULTS];
					memset(batch_results, 0, sizeof(batch_results));

					match_matchmaps_xyz(z_start, z_end, batch_results);

					int result_idx = 
						cur_yaw_step*xy_batches*xy_batches*z_batches*MATCHMAP_XYZ_N_RESULTS +
						cur_y_batch*xy_batches*z_batches*MATCHMAP_XYZ_N_RESULTS +
						cur_x_batch*z_batches*MATCHMAP_XYZ_N_RESULTS +
						cur_z_batch*MATCHMAP_XYZ_N_RESULTS;

					for(int i=0; i<MATCHMAP_XYZ_N_RESULTS; i++)
					{
						int cur_rel_score = (batch_results[i].score*REL_SCORE_MULT)/n_vox;
						//printf("i=%2d  (%+3d,%+3d,%+3d) score=%+8d, relative=%+6d\n",
						//	i, 
						//	batch_results[i].x_shift, batch_results[i].y_shift, batch_results[i].z_shift, batch_results[i].score, cur_rel_score);

						results[result_idx + i].x = cur_x_corr + batch_results[i].x_shift * MATCHMAP_UNIT;
						results[result_idx + i].y = cur_y_corr + batch_results[i].y_shift * MATCHMAP_UNIT;
						results[result_idx + i].z = cur_z_corr + batch_results[i].z_shift * MATCHMAP_UNIT;
						results[result_idx + i].yaw = cur_yaw_corr;
						results[result_idx + i].score = cur_rel_score;
					}
				}
			}

		}
	}

	qsort(results, total_results, sizeof(result_t), compar_scores);

	//#define POSTFILTER_RESULTS

	#ifdef POSTFILTER_RESULTS
		for(int i=0; i<total_results-1; i++)
		{
			for(int o=i+1; o<total_results; o++)
			{
				double transl_dist = sqrt( sq(results[i].x-results[o].x) + sq(results[i].y-results[o].y) + sq(results[i].z-results[o].z) );
				double rot_dist = fabs(results[i].yaw - results[o].yaw);

				// dist_number: 1 degree is like 256 millimeters
				double dist_number = transl_dist + RADTODEG(rot_dist)*256.0;

		//		printf("i=%d, o=%d, transl_dist=%.0f  rot_dist=%.3f,  dist_number=%.0f, score[i]=%d, score[o]=%d", i, o, transl_dist, rot_dist, dist_number, results[i].score, results[o].score);

				if((dist_number < 300.0) ||
				   (dist_number < 600.0 && 103*results[o].score < 100*results[i].score) ||
				   (dist_number < 900.0 && 106*results[o].score < 100*results[i].score) ||
				   (dist_number < 1200.0 && 110*results[o].score < 100*results[i].score) ||
				   (dist_number < 1500.0 && 115*results[o].score < 100*results[i].score))
				{
					results[o].score = -99999;
		//			printf("  mash it!");
				}

		//		printf("\n");
			}
		}
	#endif

	printf("total results (from %d pcs):\n", total_results);
	for(int i=0; i<total_results; i++)
	{
		if(results[i].score > -99999)
		{
			printf("i=%5d  (%+6d,%+6d,%+6d,%+6.2f) relscore=%+5d\n",
				i, results[i].x, results[i].y, results[i].z, RADTODEG(results[i].yaw), results[i].score);
		}
	}

}



void fine_match_submaps(int i_sma, int i_smb,  // Ref and cmp submap indeces
                   int dx, int dy, int dz) // World coordinate midpoint differences between ref and cmp
{
	static tmp_cloud_t sma, smb;

	// TODO: try removing memsets, shouldn't matter.
	memset(&sma, 0, sizeof(tmp_cloud_t));
	memset(&smb, 0, sizeof(tmp_cloud_t));

	load_tmp_cloud(&sma, i_sma);
	load_tmp_cloud(&smb, i_smb);

	// SMA is ref_fine_matchmap
	// SMB is cmp_fine_matchmap

	// ref_fine_matchmap includes free space, so is slower to calculate
	// cmp_fine_matchmap only includes occupied space, and is compared against ref_fine_matchmap quickly

	memset(&ref_fine_matchmap, 0, sizeof(ref_fine_matchmap));
	cloud_to_ref_fine_matchmap(&sma, &ref_fine_matchmap);


	// 0.5 deg step is 130mm shift at 15m distance
	double yaw_step = DEGTORAD(0.75);

	double yaw_range = DEGTORAD(2.99);
	// Round yaw_range up so that midpoint is at zero.
	yaw_range = ceil(yaw_range/yaw_step) * yaw_step;
	int yaw_steps = ceil(2.0*yaw_range/yaw_step) + 1;

	int total_results = yaw_steps * MATCHMAP_XYZ_N_RESULTS;

	result_t* results = calloc(total_results, sizeof(result_t));	
	assert(results);

	// TODO: These batches could run in parallel, in multiple threads
	for(int cur_yaw_step = 0; cur_yaw_step < yaw_steps; cur_yaw_step++)
	{
		double cur_yaw_corr = -yaw_range + (double)cur_yaw_step*yaw_step;
		// This part is per-thread.

		double cx = -(double)dx;
		double cy = -(double)dy;
		double cz = -(double)dz;

		assert(fabs(cx) < 10000.0 && fabs(cy) < 10000.0 && fabs(cz) < 2000.0);

		printf("  Fine-Matching %4d vs %4d: corr (%.0f, %.0f, %.0f, %.2f) shift (%.0f, %.0f, %.0f, %.2f)\n", 
			i_sma, i_smb,
			0.0,0.0,0.0, RADTODEG(cur_yaw_corr),
			cx, cy, cz, RADTODEG(cur_yaw_corr));

		memset(&cmp_fine_matchmap, 0, sizeof(cmp_fine_matchmap));

		int n_vox = cloud_to_cmp_fine_matchmap(&smb, &cmp_fine_matchmap, cx, cy, cz, cur_yaw_corr);

		printf("%d active voxels in cmp_matchmap\n", n_vox);

		match_xyz_result_t batch_results[MATCHMAP_XYZ_N_RESULTS];
		memset(batch_results, 0, sizeof(batch_results));

		match_fine_matchmaps_xyz(batch_results);

		int result_idx = 
			cur_yaw_step*MATCHMAP_XYZ_N_RESULTS;

		for(int i=0; i<MATCHMAP_XYZ_N_RESULTS; i++)
		{
			int cur_rel_score = (batch_results[i].score*FINE_REL_SCORE_MULT)/n_vox;
			//printf("i=%2d  (%+3d,%+3d,%+3d) score=%+8d, relative=%+6d\n",
			//	i, 
			//	batch_results[i].x_shift, batch_results[i].y_shift, batch_results[i].z_shift, batch_results[i].score, cur_rel_score);

			results[result_idx + i].x = batch_results[i].x_shift * FINE_MATCHMAP_UNIT;
			results[result_idx + i].y = batch_results[i].y_shift * FINE_MATCHMAP_UNIT;
			results[result_idx + i].z = batch_results[i].z_shift * FINE_MATCHMAP_UNIT;
			results[result_idx + i].yaw = cur_yaw_corr;
			results[result_idx + i].score = cur_rel_score;
		}
	}

	qsort(results, total_results, sizeof(result_t), compar_scores);

	//#define FINE_POSTFILTER_RESULTS

	#ifdef FINE_POSTFILTER_RESULTS
		for(int i=0; i<total_results-1; i++)
		{
			for(int o=i+1; o<total_results; o++)
			{
				double transl_dist = sqrt( sq(results[i].x-results[o].x) + sq(results[i].y-results[o].y) + sq(results[i].z-results[o].z) );
				double rot_dist = fabs(results[i].yaw - results[o].yaw);

				// dist_number: 1 degree is like 256 millimeters
				double dist_number = transl_dist + RADTODEG(rot_dist)*256.0;

		//		printf("i=%d, o=%d, transl_dist=%.0f  rot_dist=%.3f,  dist_number=%.0f, score[i]=%d, score[o]=%d", i, o, transl_dist, rot_dist, dist_number, results[i].score, results[o].score);

				if((dist_number < 150.0) ||
				   (dist_number < 300.0 && 103*results[o].score < 100*results[i].score) ||
				   (dist_number < 450.0 && 106*results[o].score < 100*results[i].score) ||
				   (dist_number < 600.0 && 110*results[o].score < 100*results[i].score) ||
				   (dist_number < 750.0 && 115*results[o].score < 100*results[i].score))
				{
					results[o].score = -99999;
		//			printf("  mash it!");
				}

		//		printf("\n");
			}
		}
	#endif

	printf("total results (from %d pcs):\n", total_results);
	for(int i=0; i<total_results; i++)
	{
		if(results[i].score > -99999)
		{
			printf("i=%5d  (%+6d,%+6d,%+6d,%+6.2f) relscore=%+5d\n",
				i, results[i].x, results[i].y, results[i].z, RADTODEG(results[i].yaw), results[i].score);
		}
	}

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


	#if 0
		for(int i=0; i < n_firstsidx_poses; i++)
		{
			printf("%6d (%8d): (%+6d %+6d %+6d  %5.1f) -> (%+6.0f %+6.0f %+6.0f  %5.1f), corr (%+6d %+6d %+6d  %5.1f)\n",
				i, firstsidx_poses[i].idx,
				firstsidx_poses[i].pose.x, firstsidx_poses[i].pose.y, firstsidx_poses[i].pose.z, ANG32TOFDEG(firstsidx_poses[i].pose.ang),
				fposes[i].x, fposes[i].y, fposes[i].z, RADTODEG(fposes[i].ang),
				pose_corrs[i].pose.x, pose_corrs[i].pose.y, pose_corrs[i].pose.z, ANG32TOFDEG(pose_corrs[i].pose.ang));
		}
	#endif

//	FILE* kakkafile = fopen("kakka.kak", "wb");
//	fwrite(fposes, sizeof(fposes), 1, kakkafile);
//	fclose(kakkafile);
//	return 0;


	

	int n_submaps;

	group_submaps(submap_metas, &n_submaps, firstsidx_poses, n_firstsidx_poses, pose_corrs);



//void output_submap(int idx)
//{
//}

	// Create and filter pointclouds
	#if 0
		int64_t total_points = 0;
		int64_t total_after_filtering = 0;

		for(int sm=0; sm<n_submaps; sm+=1)
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
					sprintf(fname, "/home/hrst/robotsoft/tsellari/trace%08d.rb2", idx);
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

			filter_cloud(&tmp_cloud, &tmp_cloud_filtered, submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z);

			printf("--> after filtering = %d\n", tmp_cloud_filtered.n_points);
			total_after_filtering += tmp_cloud_filtered.n_points;


			save_tmp_cloud(&tmp_cloud_filtered, sm);

	//		po_coords_t po;
	//		po = po_coords(submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z, 0);
	//		load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
	//		cloud_to_voxmap(&tmp_cloud_filtered, submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z);
	//		cloud_to_voxmap(&tmp_cloud, submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z);

		}
		printf("Total points: %"PRIi64", after filtering %"PRIi64"\n", total_points, total_after_filtering);

		/*
		Example of voxfilter data reduction: 78836630/119061330 = 0.66215 (threshold 1500)
				                     77905961/119061330 = 0.65433 (threshold 1800)
		*/

	#endif


	#if 1
		// Match adjacent submaps

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


			int dx = submap_metas[sm].avg_x - submap_metas[sm+1].avg_x;
			int dy = submap_metas[sm].avg_y - submap_metas[sm+1].avg_y;
			int dz = submap_metas[sm].avg_z - submap_metas[sm+1].avg_z;

			fine_match_submaps(sm, sm+1, dx, dy, dz);
		}
	#endif

	#if 0
		// Loop closures
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

/*
			for(int ssm=0; ssm<submap_metas[sm].n_subsubmaps; ssm++)
			{
				printf("         Subsubmap %2d: idx %8d .. %8d (len %2d), avg (%+6d %+6d %+6d)\n", 
					ssm, submap_metas[sm].subsubmaps[ssm].start_idx, submap_metas[sm].subsubmaps[ssm].end_idx, 
					submap_metas[sm].subsubmaps[ssm].end_idx-submap_metas[sm].subsubmaps[ssm].start_idx,
					submap_metas[sm].subsubmaps[ssm].avg_x, submap_metas[sm].subsubmaps[ssm].avg_y, submap_metas[sm].subsubmaps[ssm].avg_z);
			}
*/
			find_possible_closures(submap_metas, n_submaps, sm);
		}
	#endif



	free_all_pages();

	return 0;
} 
