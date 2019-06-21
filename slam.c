#include <stdio.h>
#include <math.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>

#define DEFINE_API_VARIABLES
#include "api_board_to_soft.h"
#undef DEFINE_API_VARIABLES

#include "voxmap.h"
#include "voxmap_memdisk.h"


#include "sin_lut.c"
#include "geotables.h"
#include "b2s_prints.c"

#define MATCHER_MAX_THREADS 4

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


int save_submap_metas(submap_meta_t* smm, uint32_t n_submaps)
{
	char fname[1024];
	snprintf(fname, 1024, "submap_metas.bin");
	FILE* f = fopen(fname, "wb");
	assert(f);

	assert(fwrite(&n_submaps, sizeof(uint32_t), 1, f) == 1);
	assert(fwrite(smm, sizeof(submap_meta_t), n_submaps, f) == n_submaps);

	fclose(f);
	return 0;
}

int load_submap_metas(submap_meta_t* smm, uint32_t* n_submaps)
{
	char fname[1024];
	snprintf(fname, 1024, "submap_metas.bin");
	FILE* f = fopen(fname, "rb");
	assert(f);

	uint32_t n_submaps_file = 0;
	assert(fread(&n_submaps_file, sizeof(uint32_t), 1, f) == 1);

	assert(n_submaps_file > 0);

	assert(fread(smm, sizeof(submap_meta_t), n_submaps_file, f) == n_submaps_file);

	*n_submaps = n_submaps_file;

	fclose(f);
	return 0;
}


#define VOXFILTER_N_SCANS 6

#define MAX_SUBMAPS 4096

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
#define Z_ACCUM_UNCERT_BY_SUBMAP 0   // mm
#define YAW_ACCUM_UNCERT_BY_SUBMAP (DEGTORAD(1.8))
#define YAW_ACCUM_UNCERT_SATURATION (DEGTORAD(40))

#define TRY_CLOSURE_MARGIN_XY 5000
#define TRY_CLOSURE_MARGIN_Z  1000


// Maximum possible number of large scale loop closure scan matching results. May return fewer.
#define N_MATCH_RESULTS 8

typedef struct
{
	int32_t x;
	int32_t y;
	int32_t z;
	float yaw;

	int32_t score;
	int32_t abscore;
} result_t;




#define TRACK_MIN(invar_, minvar_) do{if((invar_) < (minvar_)) (minvar_) = (invar_);}while(0)
#define TRACK_MAX(invar_, maxvar_) do{if((invar_) > (maxvar_)) (maxvar_) = (invar_);}while(0)

/*
	tof_slam_set is the low-level (generated by the RobotBoard2 firmware)
	dataset which contains a distance measurement flash from one sensor (9600 distance measurements), and 
	optionally a secondary flash from the same sensor (either a longer-range wide data, or narrow beam data).
	Both include a corresponding hw_pose estimate.

	input_tof_slam_set processes one such flash, adds it to the current submap, and manages
	the submaps, starting a new one and terminating the current one whenever necessary.
*/

//void group_submaps(submap_meta_t* smm_out, uint32_t* n_smm_out, firstsidx_pose_t* fsp, int n_fsp, firstsidx_pose_t* pose_corrs)
void input_tof_slam_set(tof_slam_set_t* tss)
{
	if(tss->sidx == FIRST_SIDX)

	
}

void input_from_file(int file_idx)
{
	char fname[1024];
	sprintf(fname, "/home/hrst/robotsoft/tsellari/trace%08d.rb2", i);
	tof_slam_set_t* tss;
	if(process_file(fname, &tss) == 0) // tof_slam_set record succesfully extracted
	{
		input_tof_slam_set(tss);
	}
}

{
	if(n_fsp < 1)
	{
		printf("Error: n_fsp < 1\n");

	}
	printf(__func__);
	printf(" n_fsp=%d\n", n_fsp);

	// Count limit of full sensor rotations:
	int count_limit = N_SENSORS*VOXFILTER_N_SCANS;
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
	int32_t src_seq_id; // Sequence ID references us to ray_sources table, for exact source x,y,z for raytracing empty space. 0 is an invalid id

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

typedef struct
{
	int x;
	int y;
	int z;
} voxfilter_ray_source_t;

// Remember to zero this struct out first.
typedef struct
{
	voxfilter_ray_source_t ray_sources[VOXFILTER_N_SCANS*N_SENSORS];
	voxfilter_point_t points[VOXFILTER_XS][VOXFILTER_YS][VOXFILTER_ZS][MAX_SRC_POSE_REFS]; // pose reffs refer to ray_sources
} voxfilter_t;

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
} tmp_cloud_point_t;

#define MAX_POINTS (9600*10*64)
typedef struct
{
	int n_points;
	tmp_cloud_point_t points[MAX_POINTS];
} tmp_cloud_t;

/*
	50 typical submaps = 
	400MB, uncompressed
	147MB, compressed at ZLIB_LEVEL=2
*/

#define COMPRESS_TMP_CLOUDS

#ifdef COMPRESS_TMP_CLOUDS
	#include <zlib.h>

	#define ZLIB_CHUNK (256*1024)
	#define ZLIB_LEVEL 2  // from 1 to 9, 9 = slowest but best compression
#endif

int save_tmp_cloud(tmp_cloud_t* cloud, int idx)
{
	char fname[1024];
	snprintf(fname, 1024, "submap%06u", idx);
	FILE* f = fopen(fname, "wb");
	assert(f);

	uint32_t n_points = cloud->n_points;
	assert(fwrite(&n_points, sizeof(uint32_t), 1, f) == 1);

	#ifdef COMPRESS_TMP_CLOUDS
		uint8_t outbuf[ZLIB_CHUNK];
		z_stream strm;
		strm.zalloc = Z_NULL;
		strm.zfree = Z_NULL;
		strm.opaque = Z_NULL;
		if(deflateInit(&strm, ZLIB_LEVEL) != Z_OK)
		{
			printf("ERROR: ZLIB initialization failed\n");
			abort();
		}
		strm.avail_in = n_points*sizeof(tmp_cloud_point_t);
		strm.next_in = (uint8_t*)cloud->points;

		do
		{
			strm.avail_out = ZLIB_CHUNK;
			strm.next_out = outbuf;

			int ret = deflate(&strm, Z_FINISH);
			assert(ret != Z_STREAM_ERROR);

			int produced = ZLIB_CHUNK - strm.avail_out;

			if(fwrite(outbuf, produced, 1, f) != 1 || ferror(f))
			{
				printf("ERROR: fwrite failed\n");
				abort();
			}
		} while(strm.avail_out == 0);

		assert(strm.avail_in == 0);

		deflateEnd(&strm);

	#else
		assert(fwrite(cloud->points, sizeof(tmp_cloud_point_t), n_points, f) == n_points);
	#endif

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

	#ifdef COMPRESS_TMP_CLOUDS
		uint8_t inbuf[ZLIB_CHUNK];
		z_stream strm;
		strm.zalloc = Z_NULL;
		strm.zfree = Z_NULL;
		strm.opaque = Z_NULL;
		strm.avail_in = 0;
		strm.next_in = Z_NULL;

		if(inflateInit(&strm) != Z_OK)
		{
			printf("ERROR: ZLIB initialization failed\n");
			abort();
		}

		int got_bytes = 0;
		int bytes_left = n_points*sizeof(tmp_cloud_point_t);
		int ret = 0;
		do
		{
			strm.avail_in = fread(inbuf, 1, ZLIB_CHUNK, f);
			if(ferror(f))
			{
				printf("ERROR reading submap input file\n");
				abort();
			}
			if(strm.avail_in == 0)
				break;

			strm.next_in = inbuf;
			do
			{
				strm.avail_out = bytes_left;
				strm.next_out = (uint8_t*)cloud->points + got_bytes;

				ret = inflate(&strm, Z_FINISH);
				assert(ret != Z_STREAM_ERROR);

				switch(ret)
				{
					case Z_NEED_DICT:
					case Z_DATA_ERROR:
					case Z_MEM_ERROR:
					{
						printf("ERROR: submap file decompression error, inflate() returned %d\n", ret);
						abort();
					}
					default: break;
				}

				got_bytes += bytes_left - strm.avail_out;

			} while(strm.avail_out == 0);
		} while(ret != Z_STREAM_END);

		inflateEnd(&strm);
	#else
		assert(fread(cloud->points, sizeof(tmp_cloud_point_t), n_points, f) == n_points);
	#endif

	fclose(f);
	return 0;
}

#if 0
// These DO NOT transform the ray sources
static void transform_cloud(tmp_cloud_t* cloud, int32_t transl_x, int32_t transl_y, int32_t transl_z, double yaw)
{
	double cosa = cos(yaw);
	double sina = sin(yaw);

	for(int p=0; p<cloud->n_points; p++)
	{
		double x_in = cloud->points[p].x;
		double y_in = cloud->points[p].y;
		double z_in = cloud->points[p].z;

		double x = x_in*cosa - y_in*sina + transl_x;
		double y = x_in*sina + y_in*cosa + transl_y;
		double z = z_in + transl_z;

		cloud->points[p].x = x;
		cloud->points[p].y = y;
		cloud->points[p].z = z;
	}
}

static void transform_cloud_copy(tmp_cloud_t* cloud_in, tmp_cloud_t* cloud_out, int32_t transl_x, int32_t transl_y, int32_t transl_z, double yaw)
{
	double cosa = cos(yaw);
	double sina = sin(yaw);

	cloud_out->n_points = cloud_in->n_points;
	for(int p=0; p<cloud_in->n_points; p++)
	{
		double x_in = cloud_in->points[p].x;
		double y_in = cloud_in->points[p].y;
		double z_in = cloud_in->points[p].z;

		double x = x_in*cosa - y_in*sina + transl_x;
		double y = x_in*sina + y_in*cosa + transl_y;
		double z = z_in + transl_z;

		cloud_out->points[p].x = x;
		cloud_out->points[p].y = y;
		cloud_out->points[p].z = z;
	}
}
#endif

static void rotate_cloud(tmp_cloud_t* cloud, double yaw)
{
	double cosa = cos(yaw);
	double sina = sin(yaw);

	for(int p=0; p<cloud->n_points; p++)
	{
		double px_in = cloud->points[p].px;
		double py_in = cloud->points[p].py;

		double px = px_in*cosa - py_in*sina;
		double py = px_in*sina + py_in*cosa;

		cloud->points[p].px = px;
		cloud->points[p].py = py;

		double sx_in = cloud->points[p].sx;
		double sy_in = cloud->points[p].sy;

		double sx = sx_in*cosa - sy_in*sina;
		double sy = sx_in*sina + sy_in*cosa;

		cloud->points[p].sx = sx;
		cloud->points[p].sy = sy;

	}
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
			//int rangez = i/LINE_CONE_SLOPE_Z;
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
		int sx = (cloud->points[p].sx)/CLOUDFLT_UNIT + CLOUDFLT_XS/2;
		int sy = (cloud->points[p].sy)/CLOUDFLT_UNIT + CLOUDFLT_YS/2;
		int sz = (cloud->points[p].sz)/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

		int px = (cloud->points[p].px)/CLOUDFLT_UNIT + CLOUDFLT_XS/2;
		int py = (cloud->points[p].py)/CLOUDFLT_UNIT + CLOUDFLT_YS/2;
		int pz = (cloud->points[p].pz)/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

		//printf("p=%d, (%d,%d,%d)->(%d,%d,%d)\n", sx,sy,sz, px,py,pz);
		bresenham3d_cloudflt(sx, sy, sz, px, py, pz);
	}

	out->n_points = 0;
	int n_p = 0;
	// Remove points at empty space
	for(int p=0; p<cloud->n_points; p++)
	{
		int px = (cloud->points[p].px)/CLOUDFLT_UNIT + CLOUDFLT_XS/2;
		int py = (cloud->points[p].py)/CLOUDFLT_UNIT + CLOUDFLT_YS/2;
		int pz = (cloud->points[p].pz)/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

		int val = get_voxel_cloudflt(px, py, pz);
		if(val < 1)
		{
			out->points[n_p] = cloud->points[p];
			n_p++;
		}
			
	}
	out->n_points = n_p;

}

typedef struct __attribute__((packed))
{
	uint64_t occu;
	uint64_t free;
} ref_matchmap_unit_t;

typedef struct __attribute__((packed))
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

typedef struct __attribute__((packed))
{
	uint64_t occu;
} cmp_matchmap_unit_t;

typedef struct __attribute__((packed))
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

// For debug purposes only, generate a 1:1 voxmap representation of the ref_matchmap, ignoring different voxel size (1 voxel in -> 1 voxel out)
static void ref_matchmap_to_voxmap(ref_matchmap_t* matchmap, int ref_x, int ref_y, int ref_z)
{
	int rl = 0;

	for(int yy=0; yy<MATCHMAP_YS; yy++)
	{
		for(int xx=0; xx<MATCHMAP_XS; xx++)
		{
			for(int zz=0; zz<MATCHMAP_ZS; zz++)
			{
				if(matchmap->units[yy][xx].occu & (1ULL<<zz))
				{
					po_coords_t po = po_coords(xx*VOX_UNITS[rl]+ref_x, yy*VOX_UNITS[rl]+ref_y, zz*VOX_UNITS[rl]+ref_z, rl);
					uint8_t* p_vox = get_p_voxel(po, rl);
						(*p_vox) |= 0x0f;
					mark_page_changed(po.px, po.py, po.pz);
				}

				
				if(matchmap->units[yy][xx].free & (1ULL<<zz))
				{
					po_coords_t po = po_coords(xx*VOX_UNITS[rl]+ref_x, yy*VOX_UNITS[rl]+ref_y, zz*VOX_UNITS[rl]+ref_z, rl);
					uint8_t* p_vox = get_p_voxel(po, rl);
						(*p_vox) |= 0xf0;
					mark_page_changed(po.px, po.py, po.pz);
				}
			}
		}
	}
}



ref_matchmap_t ref_matchmap __attribute__((aligned(64)));
cmp_matchmap_t cmp_matchmap __attribute__((aligned(64)));

ref_fine_matchmap_t ref_fine_matchmap __attribute__((aligned(64)));
cmp_fine_matchmap_t cmp_fine_matchmap __attribute__((aligned(64)));

static inline int count_ones_u64(uint64_t in) __attribute__((always_inline));
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
#endif

#ifdef ODROID_XU4
#define MATCHMAP_XY_RANGE 20 // 25600 bytes out of 32768 L1 cache
#endif

#define FINE_MATCHMAP_XY_RANGE 5 // 640mm
#define FINE_MATCHMAP_Z_RANGE  1

#define MATCHMAP_XYZ_N_RESULTS 32

// 100% match with no occu_on_free will be score=10000 (OCCU_MATCH_COEFF*REL_SCORE_MULT)
#define OCCU_MATCH_COEFF 1 // 1 is good for efficiency
#define OCCU_ON_FREE_COEFF -8
#define REL_SCORE_MULT 10000

#define FINE_REL_SCORE_MULT 10000
#define FINE_REL_SCORE_DIV (60*60*10) // to balance weighing


typedef struct __attribute__((packed))
{
	int8_t dummy;
	int8_t x_shift;
	int8_t y_shift;
	int8_t z_shift;

	int32_t score;

} match_xyz_result_t;

typedef struct
{
	unsigned int z_shift;
	match_xyz_result_t* p_results;
} match_matchmaps_xyz_args_t;

void* match_matchmaps_xyz(void* args)
{
//	printf("match_matchmaps_xyz(%d, %d)... ", z_start, z_end); fflush(stdout);
//	double start_time = subsec_timestamp();

	unsigned int iz = ((match_matchmaps_xyz_args_t*)args)->z_shift;
	match_xyz_result_t* p_results = ((match_matchmaps_xyz_args_t*)args)->p_results;


	assert(iz < 16);

	int32_t scores[2*MATCHMAP_XY_RANGE][2*MATCHMAP_XY_RANGE] __attribute__((aligned(64))); 

	memset(scores, 0, sizeof scores);

	for(int yy = MATCHMAP_XY_RANGE; yy < MATCHMAP_YS-MATCHMAP_XY_RANGE; yy++)
	{
		for(int xx = MATCHMAP_XY_RANGE; xx < MATCHMAP_XS-MATCHMAP_XY_RANGE; xx++)
		{
			uint64_t ref_occu = ref_matchmap.units[yy][xx].occu >> iz;
			uint64_t ref_free = ref_matchmap.units[yy][xx].free >> iz;

			// This optimization on one typical dataset: down from 3700ms to 128ms!
			if(ref_occu == 0 && ref_free == 0)
				continue;

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

					scores[iy+MATCHMAP_XY_RANGE][ix+MATCHMAP_XY_RANGE] += cur_score;
				}
			}
		}

	}

	// result array works as binary min heap

	int heap_size = 0;

	for(int iy = -MATCHMAP_XY_RANGE; iy < MATCHMAP_XY_RANGE; iy++)
	{
		for(int ix = -MATCHMAP_XY_RANGE; ix < MATCHMAP_XY_RANGE; ix++)
		{
			// occu_on_free *= -1, so that larger is better
			int32_t cur_score = scores[iy+MATCHMAP_XY_RANGE][ix+MATCHMAP_XY_RANGE];

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

	//double end_time = subsec_timestamp();
//	printf("took %.2f ms\n", (end_time-start_time)*1000.0);
	return NULL;
}


static int quick_match_matchmaps_xyz()
{
//	printf("quick_match_matchmaps_xyz(%d, %d)... ", z_start, z_end); fflush(stdout);
//	double start_time = subsec_timestamp();

	int score = 0;

#define Q_VOX_SKIP_X 3
#define Q_VOX_SKIP_Y 3
#define Q_MATCH_SKIP_X 3
#define Q_MATCH_SKIP_Y 3

	for(int yy = MATCHMAP_XY_RANGE+8; yy < MATCHMAP_YS-MATCHMAP_XY_RANGE-8; yy+=Q_VOX_SKIP_Y)
	{
		for(int xx = MATCHMAP_XY_RANGE+8; xx < MATCHMAP_XS-MATCHMAP_XY_RANGE-8; xx+=Q_VOX_SKIP_X)
		{
			uint64_t ref_occu = 
				#if MATCHER_MAX_THREADS <= 2
					(ref_matchmap.units[yy][xx].occu >> 0);
				#elif MATCHER_MAX_THREADS <= 3
					(ref_matchmap.units[yy][xx].occu >> 1);
				#elif MATCHER_MAX_THREADS == 4
					(ref_matchmap.units[yy][xx].occu >> 0) |
					(ref_matchmap.units[yy][xx].occu >> 2);
				#elif MATCHER_MAX_THREADS == 5
					(ref_matchmap.units[yy][xx].occu >> 1) |
					(ref_matchmap.units[yy][xx].occu >> 3);
				#elif MATCHER_MAX_THREADS == 6
					(ref_matchmap.units[yy][xx].occu >> 0) |
					(ref_matchmap.units[yy][xx].occu >> 2) |
					(ref_matchmap.units[yy][xx].occu >> 4);
				#elif MATCHER_MAX_THREADS == 7
					(ref_matchmap.units[yy][xx].occu >> 1) |
					(ref_matchmap.units[yy][xx].occu >> 3) |
					(ref_matchmap.units[yy][xx].occu >> 5);
				#elif MATCHER_MAX_THREADS == 8
					(ref_matchmap.units[yy][xx].occu >> 0) |
					(ref_matchmap.units[yy][xx].occu >> 2) |
					(ref_matchmap.units[yy][xx].occu >> 4) |
					(ref_matchmap.units[yy][xx].occu >> 6);
				#elif MATCHER_MAX_THREADS <= 10
					(ref_matchmap.units[yy][xx].occu >> 1) |
					(ref_matchmap.units[yy][xx].occu >> 3) |
					(ref_matchmap.units[yy][xx].occu >> 5) |
					(ref_matchmap.units[yy][xx].occu >> 7);
				#else
					(ref_matchmap.units[yy][xx].occu >> 1) |
					(ref_matchmap.units[yy][xx].occu >> 4) |
					(ref_matchmap.units[yy][xx].occu >> 7) |
					(ref_matchmap.units[yy][xx].occu >> 10);
				#endif


			for(int iy = -MATCHMAP_XY_RANGE; iy < MATCHMAP_XY_RANGE; iy+=Q_MATCH_SKIP_Y)
			{
				for(int ix = -MATCHMAP_XY_RANGE; ix < MATCHMAP_XY_RANGE; ix+=Q_MATCH_SKIP_X)
				{
					uint64_t cmp_occu = cmp_matchmap.units[yy+iy][xx+ix].occu;

					uint64_t occu_matches = cmp_occu & ref_occu;

					int32_t cur_n_occu_matches = count_ones_u64(occu_matches);

					score += cur_n_occu_matches;
				}
			}
		}

	}

//	double end_time = subsec_timestamp();
//	printf("took %.2f ms\n", (end_time-start_time)*1000.0);

	const int64_t iy_steps = 2*MATCHMAP_XY_RANGE/Q_MATCH_SKIP_Y;
	const int64_t ix_steps = 2*MATCHMAP_XY_RANGE/Q_MATCH_SKIP_X;

	// normalize as "average" score over full shifting range, over all voxels
	return ((int64_t)Q_VOX_SKIP_X*(int64_t)Q_VOX_SKIP_Y*(int64_t)score)/(iy_steps*ix_steps); 
}




static void match_fine_matchmaps_xyz(match_xyz_result_t* p_results)
{
//	printf("match_fine_matchmaps_xyz... "); fflush(stdout);
//	double start_time = subsec_timestamp();

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

				// This optimization on typical dataset: down from xxx ms to xxx ms! todo:measure
				if(ref_occu == 0 && ref_free == 0)
					continue;


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

	static const int32_t xy_weights[FINE_MATCHMAP_XY_RANGE*2+1] = {55,56,57,58,59,60,59,58,57,56,55};
	static const int32_t z_weights[FINE_MATCHMAP_Z_RANGE*2+1] = {9, 10, 9};

	int heap_size = 0;

	for(int iz = -FINE_MATCHMAP_Z_RANGE; iz <= FINE_MATCHMAP_Z_RANGE; iz++)
	{
		for(int iy = -FINE_MATCHMAP_XY_RANGE; iy <= FINE_MATCHMAP_XY_RANGE; iy++)
		{
			for(int ix = -FINE_MATCHMAP_XY_RANGE; ix <= FINE_MATCHMAP_XY_RANGE; ix++)
			{
				// occu_on_free *= -1, so that larger is better
				int32_t cur_score = scores[iz+FINE_MATCHMAP_Z_RANGE][iy+FINE_MATCHMAP_XY_RANGE][ix+FINE_MATCHMAP_XY_RANGE];

				cur_score *= xy_weights[iy+FINE_MATCHMAP_XY_RANGE] * xy_weights[ix+FINE_MATCHMAP_XY_RANGE] * z_weights[iz+FINE_MATCHMAP_Z_RANGE];

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

//	double end_time = subsec_timestamp();

//	printf("took %.2f ms\n", (end_time-start_time)*1000.0);
}

#define FINE_SMALL_XY_RANGE 2 // Bipolar, range = -range .. 0 .. +range
#define FINE_SMALL_Z_RANGE 2 // Unipolar, range = 0 .. +range (2 means 0,1,2 - keep even to have a center result)

// Small search area, return the best only
static match_xyz_result_t match_fine_matchmaps_xyz_small()
{
//	printf("match_fine_matchmaps_xyz_small... "); fflush(stdout);
//	double start_time = subsec_timestamp();

	int32_t scores[FINE_SMALL_Z_RANGE+1][2*FINE_SMALL_XY_RANGE+1][2*FINE_SMALL_XY_RANGE+1] __attribute__((aligned(64))); 
	memset(scores, 0, sizeof scores);


	for(int iz = 0; iz <= FINE_SMALL_Z_RANGE; iz++)
	{
		for(int yy = FINE_MATCHMAP_XY_RANGE; yy < FINE_MATCHMAP_YS-FINE_MATCHMAP_XY_RANGE-1; yy++)
		{
			for(int xx = FINE_MATCHMAP_XY_RANGE; xx < FINE_MATCHMAP_XS-FINE_MATCHMAP_XY_RANGE-1; xx++) 
			{
				uint64_t ref_occu = ref_fine_matchmap.units[yy][xx].occu >> iz;
				uint64_t ref_free = ref_fine_matchmap.units[yy][xx].free >> iz;

				// This optimization on typical dataset: down from xxx ms to xxx ms! todo:measure
				if(ref_occu == 0 && ref_free == 0)
					continue;

				for(int iy = -FINE_SMALL_XY_RANGE; iy <= FINE_SMALL_XY_RANGE; iy++)
				{
					for(int ix = -FINE_SMALL_XY_RANGE; ix <= FINE_SMALL_XY_RANGE; ix++)
					{
						uint64_t cmp_occu = cmp_fine_matchmap.units[yy+iy][xx+ix].occu;

						uint64_t occu_matches = cmp_occu & ref_occu;
						uint64_t occu_on_free = cmp_occu & ref_free;

						int32_t cur_n_occu_matches = count_ones_u64(occu_matches);
						int32_t cur_n_occu_on_free = count_ones_u64(occu_on_free);
						int32_t cur_score = OCCU_MATCH_COEFF*cur_n_occu_matches + OCCU_ON_FREE_COEFF*cur_n_occu_on_free;

						scores[iz][iy+FINE_SMALL_XY_RANGE][ix+FINE_SMALL_XY_RANGE] += cur_score;

					}
				}
			}

		}
	}

	int32_t best_score = -999999;
	match_xyz_result_t result = {0};

	for(int iz = 0; iz <= FINE_SMALL_Z_RANGE; iz++)
	{
		for(int iy = -FINE_SMALL_XY_RANGE; iy <= FINE_SMALL_XY_RANGE; iy++)
		{
			for(int ix = -FINE_SMALL_XY_RANGE; ix <= FINE_SMALL_XY_RANGE; ix++)
			{
				int32_t cur_score = scores[iz][iy+FINE_SMALL_XY_RANGE][ix+FINE_SMALL_XY_RANGE];
				if(cur_score > best_score)
				{
					best_score = cur_score;
					result = (match_xyz_result_t){0, ix, iy, iz, best_score};
				}
			}
		}
	}


//	double end_time = subsec_timestamp();
//	printf("took %.2f ms\n", (end_time-start_time)*1000.0);

	return result;
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
			//int rangez = i/MATCHMAP_LINE_CONE_SLOPE_Z;
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
			//int rangez = i/FINE_MATCHMAP_LINE_CONE_SLOPE_Z;
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


/*
	For optimization, two functions share the calculation of even, odd.
	Return values can be summed.
	Calling only the first one gives a very good initial guess.
*/
// Returns number of occupied voxels, for later calculation of relative score (percentage of matches)
static int cloud_to_cmp_matchmap_even(tmp_cloud_t* cloud, cmp_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
{
//	printf("cloud_to_cmp_matchmap, adding %d points\n", cloud->n_points);

	float cosa = cosf(yaw_corr);
	float sina = sinf(yaw_corr);

	int vox_cnt = 0;
	for(int p=0; p<cloud->n_points; p+=2) // using a variable in p+=x instead of literal 1 slows down by about 15%
	{
		float x_in = cloud->points[p].px;
		float y_in = cloud->points[p].py;

		float x = x_in*cosa - y_in*sina + x_corr;
		float y = x_in*sina + y_in*cosa + y_corr;

		// Using integer math for z instead of float: 3.4ms --> 2.9ms
		int z = cloud->points[p].pz + z_corr;

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

static int cloud_to_cmp_matchmap_odd(tmp_cloud_t* cloud, cmp_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
{
//	printf("cloud_to_cmp_matchmap, adding %d points\n", cloud->n_points);

	float cosa = cosf(yaw_corr);
	float sina = sinf(yaw_corr);

	int vox_cnt = 0;
	for(int p=1; p<cloud->n_points; p+=2) // using a variable in p+=x instead of literal 1 slows down by about 15%
	{
		float x_in = cloud->points[p].px;
		float y_in = cloud->points[p].py;

		float x = x_in*cosa - y_in*sina + x_corr;
		float y = x_in*sina + y_in*cosa + y_corr;

		// Using integer math for z instead of float: 3.4ms --> 2.9ms
		int z = cloud->points[p].pz + z_corr;

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


static int cloud_to_cmp_fine_matchmap(tmp_cloud_t* cloud, cmp_fine_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
{
//	printf("cloud_to_cmp_fine_matchmap, adding %d points\n", cloud->n_points);

	float cosa = cosf(yaw_corr);
	float sina = sinf(yaw_corr);

	int vox_cnt = 0;
	int oor = 0;
	for(int p=0; p<cloud->n_points; p+=1)
	{
		float x_in = cloud->points[p].px;
		float y_in = cloud->points[p].py;

		float x = x_in*cosa - y_in*sina + x_corr;
		float y = x_in*sina + y_in*cosa + y_corr;
		int z = cloud->points[p].pz + z_corr;

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

	if(oor > cloud->n_points/20)
	{
		printf("cloud_to_cmp_fine_matchmap: WARN: over 5%% OOR points\n");
	}
	return vox_cnt;
}

/*
	The next two functions are both called to build the ref_matchmap.
	They are separate, because if multiple clouds are used, you need to
	first call the _free() version for each, then _occu() for each
	(because the _occu() also removes free space around the points)

	corr params transform the clouds, allowing you to add multiple submaps
	to the same ref_matchmap, positioned correctly.
*/

/*
	Sometimes, we have scenes where the sensor range is not enough to see a wall in a large space.

	Consider this example

	...................
	...................
	...................
	...................
	....A...###########
	...................
	......R............
	...................

	We clearly would see any obstacles near the robot, so it's very likely point A, for exmaple,
	is free, but cannot be raytraced such due to lack of walls behind it.

	Setting ASSUME_FREE_ABOVE_FLOOR adds free voxels above any occupied voxels that are on a certain Z range.
	Note that if occupied space actually exists, the _occu function removes this free space.

	This assumption helps generate negative score for obviously wrong matchings.
*/
#define ASSUME_FREE_ABOVE_FLOOR

static void cloud_to_ref_matchmap_free(tmp_cloud_t* cloud, ref_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
{
	float cosa = cosf(yaw_corr);
	float sina = sinf(yaw_corr);

	// Trace empty space:
	for(int p=0; p<cloud->n_points; p++)
	{
		float sx_in = cloud->points[p].sx;
		float sy_in = cloud->points[p].sy;

		float mmsx = sx_in*cosa - sy_in*sina + x_corr;
		float mmsy = sx_in*sina + sy_in*cosa + y_corr;
		int   mmsz = cloud->points[p].sz + z_corr;

		float px_in = cloud->points[p].px;
		float py_in = cloud->points[p].py;

		float mmpx = px_in*cosa - py_in*sina + x_corr;
		float mmpy = px_in*sina + py_in*cosa + y_corr;
		int   mmpz = cloud->points[p].pz + z_corr;

		int sx = mmsx/MATCHMAP_UNIT + MATCHMAP_XS/2;
		int sy = mmsy/MATCHMAP_UNIT + MATCHMAP_YS/2;
		int sz = mmsz/MATCHMAP_UNIT + MATCHMAP_ZS/2;

		int px = mmpx/MATCHMAP_UNIT + MATCHMAP_XS/2;
		int py = mmpy/MATCHMAP_UNIT + MATCHMAP_YS/2;
		int pz = mmpz/MATCHMAP_UNIT + MATCHMAP_ZS/2;

		bresenham3d_ref_matchmap(sx, sy, sz, px, py, pz);

		#ifdef ASSUME_FREE_ABOVE_FLOOR
			// Pointcloud zero is at the average robot pose; robot Z origin is at the floor level
			if(cloud->points[p].pz > -500 && cloud->points[p].pz < 300)
			{
				// Seven free voxels is 1792mm high 
				if(px > 0 && px < MATCHMAP_XS && py > 0 && py < MATCHMAP_YS)
					matchmap->units[py][px].free |= 0b1111111ULL << pz;
			}
		#endif
	}
}

static void cloud_to_ref_matchmap_occu(tmp_cloud_t* cloud, ref_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
{
	float cosa = cosf(yaw_corr);
	float sina = sinf(yaw_corr);

	// Add points (to neighbor cells as well), and remove empty space around them.

	for(int p=0; p<cloud->n_points; p++)
	{
		float px_in = cloud->points[p].px;
		float py_in = cloud->points[p].py;

		float mmpx = px_in*cosa - py_in*sina + x_corr;
		float mmpy = px_in*sina + py_in*cosa + y_corr;
		int   mmpz = cloud->points[p].pz + z_corr;

		int px = mmpx/MATCHMAP_UNIT + MATCHMAP_XS/2;
		int py = mmpy/MATCHMAP_UNIT + MATCHMAP_YS/2;
		int pz = mmpz/MATCHMAP_UNIT + MATCHMAP_ZS/2;

		// For every matcher thread, reference map is shifted >>1
		if(px < 1 || px > MATCHMAP_XS-2 || py < 1 || py > MATCHMAP_YS-2 || pz < MATCHER_MAX_THREADS || pz > MATCHMAP_ZS-2)
		{
			printf("cloud_to_ref_matchmap: WARN, OOR point %d,%d,%d\n", px, py, pz);
			continue;
		}

		for(int ix=-1; ix<=1; ix++)
		{
			for(int iy=-1; iy<=1; iy++)
			{
				matchmap->units[py+iy][px+ix].occu |= 0b111ULL << (pz-1);
				matchmap->units[py+iy][px+ix].free &= ~(0b111ULL << (pz-1));
			}
		}
	}
}

/*

	These versions only take every third point in the cloud.
	Much quicker. Useful for most statistical purposes, where having holes
	in free space is not an issue.
*/


static void decim_cloud_to_ref_matchmap_free(tmp_cloud_t* cloud, ref_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
{
	float cosa = cosf(yaw_corr);
	float sina = sinf(yaw_corr);

	// Trace empty space:
	for(int p=0; p<cloud->n_points; p+=3)
	{
		float sx_in = cloud->points[p].sx;
		float sy_in = cloud->points[p].sy;

		float mmsx = sx_in*cosa - sy_in*sina + x_corr;
		float mmsy = sx_in*sina + sy_in*cosa + y_corr;
		int   mmsz = cloud->points[p].sz + z_corr;

		float px_in = cloud->points[p].px;
		float py_in = cloud->points[p].py;

		float mmpx = px_in*cosa - py_in*sina + x_corr;
		float mmpy = px_in*sina + py_in*cosa + y_corr;
		int   mmpz = cloud->points[p].pz + z_corr;

		int sx = mmsx/MATCHMAP_UNIT + MATCHMAP_XS/2;
		int sy = mmsy/MATCHMAP_UNIT + MATCHMAP_YS/2;
		int sz = mmsz/MATCHMAP_UNIT + MATCHMAP_ZS/2;

		int px = mmpx/MATCHMAP_UNIT + MATCHMAP_XS/2;
		int py = mmpy/MATCHMAP_UNIT + MATCHMAP_YS/2;
		int pz = mmpz/MATCHMAP_UNIT + MATCHMAP_ZS/2;

		bresenham3d_ref_matchmap(sx, sy, sz, px, py, pz);

		#ifdef ASSUME_FREE_ABOVE_FLOOR
			// Pointcloud zero is at the average robot pose; robot Z origin is at the floor level
			if(cloud->points[p].pz > -500 && cloud->points[p].pz < 300)
			{
				// Seven free voxels is 1792mm high 
				if(px > 0 && px < MATCHMAP_XS && py > 0 && py < MATCHMAP_YS)
					matchmap->units[py][px].free |= 0b1111111ULL << pz;
			}
		#endif
	}
}

static void decim_cloud_to_ref_matchmap_occu(tmp_cloud_t* cloud, ref_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
{
	float cosa = cosf(yaw_corr);
	float sina = sinf(yaw_corr);

	// Add points (to neighbor cells as well), and remove empty space around them.

	for(int p=0; p<cloud->n_points; p+=3)
	{
		float px_in = cloud->points[p].px;
		float py_in = cloud->points[p].py;

		float mmpx = px_in*cosa - py_in*sina + x_corr;
		float mmpy = px_in*sina + py_in*cosa + y_corr;
		int   mmpz = cloud->points[p].pz + z_corr;

		int px = mmpx/MATCHMAP_UNIT + MATCHMAP_XS/2;
		int py = mmpy/MATCHMAP_UNIT + MATCHMAP_YS/2;
		int pz = mmpz/MATCHMAP_UNIT + MATCHMAP_ZS/2;

		// For every matcher thread, reference map is shifted >>1
		if(px < 1 || px > MATCHMAP_XS-2 || py < 1 || py > MATCHMAP_YS-2 || pz < MATCHER_MAX_THREADS || pz > MATCHMAP_ZS-2)
		{
			printf("cloud_to_ref_matchmap: WARN, OOR point %d,%d,%d\n", px, py, pz);
			continue;
		}

		for(int ix=-1; ix<=1; ix++)
		{
			for(int iy=-1; iy<=1; iy++)
			{
				matchmap->units[py+iy][px+ix].occu |= 0b111ULL << (pz-1);
				matchmap->units[py+iy][px+ix].free &= ~(0b111ULL << (pz-1));
			}
		}
	}
}


static inline int uint32_log2(uint32_t in)
{
	int r = 0;
	while(in >>= 1)
		r++;

	return r;
}


static void cloud_to_ref_fine_matchmap_free(tmp_cloud_t* cloud, ref_fine_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
{
	float cosa = cosf(yaw_corr);
	float sina = sinf(yaw_corr);

	// Trace empty space:
	for(int p=0; p<cloud->n_points; p++)
	{
		float sx_in = cloud->points[p].sx;
		float sy_in = cloud->points[p].sy;

		float mmsx = sx_in*cosa - sy_in*sina + x_corr;
		float mmsy = sx_in*sina + sy_in*cosa + y_corr;
		int   mmsz = cloud->points[p].sz + z_corr;

		float px_in = cloud->points[p].px;
		float py_in = cloud->points[p].py;

		float mmpx = px_in*cosa - py_in*sina + x_corr;
		float mmpy = px_in*sina + py_in*cosa + y_corr;
		int   mmpz = cloud->points[p].pz + z_corr;

		int sx = mmsx/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_XS/2;
		int sy = mmsy/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_YS/2;
		int sz = mmsz/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_ZS/2;

		int px = mmpx/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_XS/2;
		int py = mmpy/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_YS/2;
		int pz = mmpz/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_ZS/2;

		bresenham3d_ref_fine_matchmap(sx, sy, sz, px, py, pz);

		#ifdef ASSUME_FREE_ABOVE_FLOOR
			// Pointcloud zero is at the average robot pose; robot Z origin is at the floor level
			if(cloud->points[p].pz > -500 && cloud->points[p].pz < 300)
			{
				// 14 free voxels is 1792mm high 
				if(px > 0 && px < FINE_MATCHMAP_XS && py > 0 && py < FINE_MATCHMAP_YS)
					matchmap->units[py][px].free |= 0b11111111111111ULL << pz;
			}
		#endif

	}
}


static void cloud_to_ref_fine_matchmap_occu(tmp_cloud_t* cloud, ref_fine_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
{
	float cosa = cosf(yaw_corr);
	float sina = sinf(yaw_corr);

	// Add points (to neighbor cells as well), and remove empty space around them.

	int oor = 0;
	for(int p=0; p<cloud->n_points; p++)
	{
		float px_in = cloud->points[p].px;
		float py_in = cloud->points[p].py;

		float mmpx = px_in*cosa - py_in*sina + x_corr;
		float mmpy = px_in*sina + py_in*cosa + y_corr;
		int   mmpz = cloud->points[p].pz + z_corr;

		int px = mmpx/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_XS/2;
		int py = mmpy/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_YS/2;
		int pz = mmpz/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_ZS/2;

		if(px < 1 || px > FINE_MATCHMAP_XS-2 || py < 1 || py > FINE_MATCHMAP_YS-2 || pz < MATCHER_MAX_THREADS || pz > FINE_MATCHMAP_ZS-2)
		{
			oor++;
			continue;
		}

		for(int ix=-1; ix<=1; ix++)
		{
			for(int iy=-1; iy<=1; iy++)
			{
				matchmap->units[py+iy][px+ix].occu |= 0b111ULL << (pz-1);
				matchmap->units[py+iy][px+ix].free &= ~(0b111ULL << (pz-1));
			}
		}
	}

	if(oor > cloud->n_points/20)
	{
		printf("WARN: cloud_to_ref_fine_matchmap_occu: more than 5%% out-of-range voxels\n");
	}
}


static int compar_scores(const void* a, const void* b)
{
	if(((result_t*)a)->score > ((result_t*)b)->score)
		return -1;
	else if(((result_t*)a)->score < ((result_t*)b)->score)
		return 1;

	return 0;
}

typedef struct __attribute__((packed))
{
	/*
		Ratio between objects seen from optimally chosen imaginary axes 90 degrees apart.

		#########################
		.........................
		.........................
		.........................
		#########################

		0%, uncertain_angle = 0 (or pi, or -pi)

		#......#
		  #......#
		    #......#
		      #......#
		        #......#
		          #......#

		0% as well, uncertain_angle = -1/4 pi (or 3/4 pi)

		                        #
		#########################.
		...........................
		............................
		...........#................
		############################

		maybe 5%, uncertain_angle still 0

		       #.....#
		       #.....#
		########.....###########
		........................
		........................
		........................
		##########.....#########
		         #.....#
		         #.....#
		         #.....#

		Near 100%, uncertain_angle irrelevant

		uncertain_angle can be used to weight scores along the uncertain axis when xy_ratio is far below 100%.

		Note that 100% ratio seldom happens, and 50% ratio is quite good already for matching without tricks like this.

		But if the ratio is somewhere around 10%, false matches are very likely.

	*/
	int32_t xtot;
	int32_t ytot;
	int32_t ztot;
	float xy_ratio; 
	float uncertain_angle;
} ref_quality_t;


ref_quality_t calc_ref_matchmap_quality(ref_matchmap_t* matchmap)
{
	// 256*(256-6)*64 at most, int32 enough
	int32_t pos_x_cnt = 0; 
	int32_t neg_x_cnt = 0;
	int32_t pos_y_cnt = 0;
	int32_t neg_y_cnt = 0;
	int32_t pos_z_cnt = 0;
	int32_t neg_z_cnt = 0;

	for(int yy=0; yy<MATCHMAP_YS; yy++)
	{
		for(int xx=3; xx<MATCHMAP_XS-3; xx++)
		{
			// Positive X direction (walls seen to the right of the robot on standard 2D view)
			uint64_t pos_x_match = matchmap->units[yy][xx].free & matchmap->units[yy][xx+2].occu;
			pos_x_cnt += count_ones_u64(pos_x_match);

			// Negative X direction
			uint64_t neg_x_match = matchmap->units[yy][xx].free & matchmap->units[yy][xx-2].occu;
			neg_x_cnt += count_ones_u64(neg_x_match);
		}
	}

	for(int xx=0; xx<MATCHMAP_XS; xx++)
	{
		for(int yy=3; yy<MATCHMAP_YS-3; yy++)
		{
			// Positive Y direction
			uint64_t pos_y_match = matchmap->units[yy][xx].free & matchmap->units[yy+2][xx].occu;
			pos_y_cnt += count_ones_u64(pos_y_match);

			// Negative Y direction
			uint64_t neg_y_match = matchmap->units[yy][xx].free & matchmap->units[yy-2][xx].occu;
			neg_y_cnt += count_ones_u64(neg_y_match);
		}
	}


	for(int yy=0; yy<MATCHMAP_YS; yy++)
	{
		for(int xx=0; xx<MATCHMAP_XS; xx++)
		{
			for(int zz=3; zz<MATCHMAP_ZS-3; zz++)
			{
				// Positive Z direction (ceilings-like stuff)
				if(matchmap->units[yy][xx].free & (1ULL<<zz) && matchmap->units[yy][xx].occu & (1ULL<<(zz+2)))
					pos_z_cnt++;

				// Negative Z direction (floor-like stuff)
				if(matchmap->units[yy][xx].free & (1ULL<<zz) && matchmap->units[yy][xx].occu & (1ULL<<(zz-2)))
					neg_z_cnt++;

			}
		}
	}

	int32_t xtot = pos_x_cnt + neg_x_cnt;
	int32_t ytot = pos_y_cnt + neg_y_cnt;
	int32_t ztot = pos_z_cnt + neg_z_cnt;

	int32_t larger_xy, smaller_xy;
	if(xtot > ytot)
	{
		larger_xy = xtot;
		smaller_xy = ytot;
	}
	else
	{
		larger_xy = ytot;
		smaller_xy = xtot;
	}


//	printf("X+ > %8d   X- < %8d   Y+ ^ %8d   Y- v %8d   Z+ ceil %8d   Z- floor %8d    ",
//		pos_x_cnt, neg_x_cnt, pos_y_cnt, neg_y_cnt, pos_z_cnt, neg_z_cnt);
//	printf("X %8d (%2.0f%%)    Y %8d (%2.0f%%)    Z %8d (%2.0f%%)",
//		xtot, 100.0*(float)xtot/larger_xy, ytot, 100.0*(float)ytot/larger_xy, ztot, 100.0*(float)ztot/larger_xy);

	ref_quality_t ret;

	ret.xtot = xtot;
	ret.ytot = ytot;
	ret.ztot = ztot;
	ret.xy_ratio = (float)smaller_xy/(float)larger_xy;

	return ret;
}


/*
	ca = submap before our submap of interest
	cb = submap of interest
	cc = next submap after our submap of interest
	corrab = transform from ca to cb for smooth joining (based on adjacent match, for example)
	corrcb = transform from cb to cc, similarly

*/
int gen_save_ref_matchmap_set(tmp_cloud_t* ca, tmp_cloud_t* cb, tmp_cloud_t* cc, result_t corrab, result_t corrbc)
{

/*
	kak1: -45.0 ; +45.1 ; 7.5   -7.5 ; +7.5 ; 2.5  13+7 = 20 ops  reference
	kak2: -40   ; +40.1 ; 10.0  -7.5 ; +7.5 ; 2.5  9+7 = 16 ops   OK -> use this
	kak3: -40   ; +40.1 ; 10.0  -5.0 ; +5.0 ; 2.5  9+5 = 14 ops   different result
	kak4: -37.5 ; +37.6 ; 12.5  -7.5 ; +7.5 ; 2.5  7+7 = 14 ops   OK, but let's not go this far.
*/
	float smallest = 999.9;
	float smallest_a = 0.0;
	//int dbg_y = 0;
	// See Screenshot_2019-06-20_18-41-53.png showing how the combined ref_matchmap looks like.
	for(float a=DEGTORAD(-40.0); a<DEGTORAD(40.1); a+=DEGTORAD(10.0))
	{
		memset(&ref_matchmap, 0, sizeof(ref_matchmap));

		// Join ca and cc to the cb.
		// corrab is transform from a to b: apply it in inverse to go from b to a.
		// Note that parameter A rotates the whole thing, so the translation parameters must be rotated as well, hence the sin, cos stuff.
		// ca first, in inverse transform:
		decim_cloud_to_ref_matchmap_free(ca, &ref_matchmap, 
			-corrab.x - corrab.x*cos(a) + corrab.y*sin(a), // X translation to place ca correctly relative to cb
			-corrab.y - corrab.x*sin(a) - corrab.y*cos(a), // Y translation
			-corrab.z, // Z translation
			-corrab.yaw + a); // yaw rotation

		// cb as-is:
		decim_cloud_to_ref_matchmap_free(cb, &ref_matchmap, 0,0,0, a);

		// cc last, transformed:
		decim_cloud_to_ref_matchmap_free(cc, &ref_matchmap, 
			+corrbc.x + corrbc.x*cos(a) - corrbc.y*sin(a), // X translation to place cc correctly relative to cb
			+corrbc.y + corrbc.x*sin(a) + corrbc.y*cos(a), // Y translation
			+corrbc.z, // Z translation
			+corrbc.yaw + a); // yaw rotation


		// Exact same thing for occupied space:

		decim_cloud_to_ref_matchmap_occu(ca, &ref_matchmap, 
			-corrab.x - corrab.x*cos(a) + corrab.y*sin(a), // X translation to place ca correctly relative to cb
			-corrab.y - corrab.x*sin(a) - corrab.y*cos(a), // Y translation
			-corrab.z, // Z translation
			-corrab.yaw + a); // yaw rotation

		decim_cloud_to_ref_matchmap_occu(cb, &ref_matchmap, 0,0,0, a);

		decim_cloud_to_ref_matchmap_occu(cc, &ref_matchmap, 
			+corrbc.x + corrbc.x*cos(a) - corrbc.y*sin(a), // X translation to place cc correctly relative to cb
			+corrbc.y + corrbc.x*sin(a) + corrbc.y*cos(a), // Y translation
			+corrbc.z, // Z translation
			+corrbc.yaw + a); // yaw rotation


		// Test code:
			/*
			po_coords_t po;
			po = po_coords(0,dbg_y,0, 0);
			load_pages(1,1,  po.px-1, po.px+3, po.py-3, po.py+3, po.pz-1, po.pz+2);

			ref_matchmap_to_voxmap(&ref_matchmap, 0, dbg_y, 0);
			dbg_y -= 4096;
			*/

		//printf("a=%+6.1f: ", RADTODEG(a)); fflush(stdout);
		ref_quality_t q = calc_ref_matchmap_quality(&ref_matchmap);

		//printf(" -> q.xy_ratio = %.2f\n", q.xy_ratio);

		if(q.xy_ratio < smallest)
		{
			smallest = q.xy_ratio;
			smallest_a = a;
		}
	}

	smallest = 999.9;
	smallest_a = 0.0;
	ref_quality_t q_best;
	for(float a=smallest_a-DEGTORAD(7.5); a<smallest_a+DEGTORAD(7.6); a+=DEGTORAD(2.5))
	{
		memset(&ref_matchmap, 0, sizeof(ref_matchmap));

		decim_cloud_to_ref_matchmap_free(ca, &ref_matchmap, 
			-corrab.x - corrab.x*cos(a) + corrab.y*sin(a), // X translation to place ca correctly relative to cb
			-corrab.y - corrab.x*sin(a) - corrab.y*cos(a), // Y translation
			-corrab.z, // Z translation
			-corrab.yaw + a); // yaw rotation

		decim_cloud_to_ref_matchmap_free(cb, &ref_matchmap, 0,0,0, a);

		decim_cloud_to_ref_matchmap_free(cc, &ref_matchmap, 
			+corrbc.x + corrbc.x*cos(a) - corrbc.y*sin(a), // X translation to place cc correctly relative to cb
			+corrbc.y + corrbc.x*sin(a) + corrbc.y*cos(a), // Y translation
			+corrbc.z, // Z translation
			+corrbc.yaw + a); // yaw rotation

		decim_cloud_to_ref_matchmap_occu(ca, &ref_matchmap, 
			-corrab.x - corrab.x*cos(a) + corrab.y*sin(a), // X translation to place ca correctly relative to cb
			-corrab.y - corrab.x*sin(a) - corrab.y*cos(a), // Y translation
			-corrab.z, // Z translation
			-corrab.yaw + a); // yaw rotation

		decim_cloud_to_ref_matchmap_occu(cb, &ref_matchmap, 0,0,0, a);

		decim_cloud_to_ref_matchmap_occu(cc, &ref_matchmap, 
			+corrbc.x + corrbc.x*cos(a) - corrbc.y*sin(a), // X translation to place cc correctly relative to cb
			+corrbc.y + corrbc.x*sin(a) + corrbc.y*cos(a), // Y translation
			+corrbc.z, // Z translation
			+corrbc.yaw + a); // yaw rotation

		//printf("a=%+6.1f: ", RADTODEG(a)); fflush(stdout);
		ref_quality_t q = calc_ref_matchmap_quality(&ref_matchmap);

		//printf(" -> q.xy_ratio = %.2f\n", q.xy_ratio);

		if(q.xy_ratio < smallest)
		{
			smallest = q.xy_ratio;
			smallest_a = a;
			q_best = q;
		}
	}

	if(q_best.xtot > q_best.ytot)
	{
		q_best.uncertain_angle = M_PI/2.0 - smallest_a;
	}
	else
	{
		q_best.uncertain_angle = -smallest_a;
	}

	printf("----> ratio %.2f, uncertain_angle = %.1f deg\n", q_best.xy_ratio, RADTODEG(q_best.uncertain_angle));

	return 0;

/*



int match_submaps(int n_sma, int* i_sma, result_t* sma_corrs, // Number of ref submaps, ref submap indeces, and relative corrections compared to the ref midpoint (which is again related to dx,dy,dz)
                   int i_smb,  // cmp submap index
                   double xy_range, double z_range, double yaw_range, // Matching correction ranges (mm, rad)
                   int dx, int dy, int dz, // World coordinate midpoint differences between ref and cmp
                   result_t* results_out)
{
	tmp_cloud_t* smas[4];

	// Glue the ref submaps together to ref_matchmap:
	memset(&ref_matchmap, 0, sizeof(ref_matchmap));
	memset(&ref_fine_matchmap, 0, sizeof(ref_fine_matchmap));
	for(int i=0; i<n_sma; i++)
	{
		smas[i] = malloc(sizeof(tmp_cloud_t));
		load_tmp_cloud(smas[i], i_sma[i]);

		cloud_to_ref_matchmap_free(smas[i], &ref_matchmap, sma_corrs[i].x, sma_corrs[i].y, sma_corrs[i].z, sma_corrs[i].yaw);
		cloud_to_ref_fine_matchmap_free(smas[i], &ref_fine_matchmap, sma_corrs[i].x, sma_corrs[i].y, sma_corrs[i].z, sma_corrs[i].yaw);
	}

	for(int i=0; i<n_sma; i++)
	{
		cloud_to_ref_matchmap_occu(smas[i], &ref_matchmap, sma_corrs[i].x, sma_corrs[i].y, sma_corrs[i].z, sma_corrs[i].yaw);
		cloud_to_ref_fine_matchmap_occu(smas[i], &ref_fine_matchmap, sma_corrs[i].x, sma_corrs[i].y, sma_corrs[i].z, sma_corrs[i].yaw);
		free(smas[i]);
	}





	ref_matchmap_

	char fname[1024];
	snprintf(fname, 1024, "submap%06u", idx);
	FILE* f = fopen(fname, "wb");
	assert(f);

	uint32_t n_points = cloud->n_points;
	assert(fwrite(&n_points, sizeof(uint32_t), 1, f) == 1);

	#ifdef COMPRESS_TMP_CLOUDS
		uint8_t outbuf[ZLIB_CHUNK];
		z_stream strm;
		strm.zalloc = Z_NULL;
		strm.zfree = Z_NULL;
		strm.opaque = Z_NULL;
		if(deflateInit(&strm, ZLIB_LEVEL) != Z_OK)
		{
			printf("ERROR: ZLIB initialization failed\n");
			abort();
		}
		strm.avail_in = n_points*sizeof(tmp_cloud_point_t);
		strm.next_in = (uint8_t*)cloud->points;

		do
		{
			strm.avail_out = ZLIB_CHUNK;
			strm.next_out = outbuf;

			int ret = deflate(&strm, Z_FINISH);
			assert(ret != Z_STREAM_ERROR);

			int produced = ZLIB_CHUNK - strm.avail_out;

			if(fwrite(outbuf, produced, 1, f) != 1 || ferror(f))
			{
				printf("ERROR: fwrite failed\n");
				abort();
			}
		} while(strm.avail_out == 0);

		assert(strm.avail_in == 0);

		deflateEnd(&strm);

	#else
		assert(fwrite(cloud->points, sizeof(tmp_cloud_point_t), n_points, f) == n_points);
	#endif

	fclose(f);
	return 0;

*/
}
/*
int load_tmp_cloud(tmp_cloud_t* cloud, int idx)
{
	char fname[1024];
	snprintf(fname, 1024, "submap%06u", idx);
	FILE* f = fopen(fname, "rb");
	assert(f);

	uint32_t n_points = 0;
	assert(fread(&n_points, sizeof(uint32_t), 1, f) == 1);
	cloud->n_points = n_points;

	#ifdef COMPRESS_TMP_CLOUDS
		uint8_t inbuf[ZLIB_CHUNK];
		z_stream strm;
		strm.zalloc = Z_NULL;
		strm.zfree = Z_NULL;
		strm.opaque = Z_NULL;
		strm.avail_in = 0;
		strm.next_in = Z_NULL;

		if(inflateInit(&strm) != Z_OK)
		{
			printf("ERROR: ZLIB initialization failed\n");
			abort();
		}

		int got_bytes = 0;
		int bytes_left = n_points*sizeof(tmp_cloud_point_t);
		int ret = 0;
		do
		{
			strm.avail_in = fread(inbuf, 1, ZLIB_CHUNK, f);
			if(ferror(f))
			{
				printf("ERROR reading submap input file\n");
				abort();
			}
			if(strm.avail_in == 0)
				break;

			strm.next_in = inbuf;
			do
			{
				strm.avail_out = bytes_left;
				strm.next_out = (uint8_t*)cloud->points + got_bytes;

				ret = inflate(&strm, Z_FINISH);
				assert(ret != Z_STREAM_ERROR);

				switch(ret)
				{
					case Z_NEED_DICT:
					case Z_DATA_ERROR:
					case Z_MEM_ERROR:
					{
						printf("ERROR: submap file decompression error, inflate() returned %d\n", ret);
						abort();
					}
					default: break;
				}

				got_bytes += bytes_left - strm.avail_out;

			} while(strm.avail_out == 0);
		} while(ret != Z_STREAM_END);

		inflateEnd(&strm);
	#else
		assert(fread(cloud->points, sizeof(tmp_cloud_point_t), n_points, f) == n_points);
	#endif

	fclose(f);
	return 0;
}
*/



/*
	kak1: -45.0 ; +45.1 ; 7.5   -7.5 ; +7.5 ; 2.5  13+7 = 20 ops  reference
	kak2: -40   ; +40.1 ; 10.0  -7.5 ; +7.5 ; 2.5  9+7 = 16 ops   OK -> use this
	kak3: -40   ; +40.1 ; 10.0  -5.0 ; +5.0 ; 2.5  9+5 = 14 ops   different result
	kak4: -37.5 ; +37.6 ; 12.5  -7.5 ; +7.5 ; 2.5  7+7 = 14 ops   OK, but let's not go this far.
*/

void test_q(int smi)
{

	float smallest = 999.9;
	float smallest_a = 0.0;
	static tmp_cloud_t cloud;
	load_tmp_cloud(&cloud, smi);
	for(float a=DEGTORAD(-40.0); a<DEGTORAD(40.1); a+=DEGTORAD(10.0))
	{
		memset(&ref_matchmap, 0, sizeof(ref_matchmap));
		decim_cloud_to_ref_matchmap_free(&cloud, &ref_matchmap, 0,0,0, a);
		decim_cloud_to_ref_matchmap_occu(&cloud, &ref_matchmap, 0,0,0, a);

		//printf("a=%+6.1f: ", RADTODEG(a)); fflush(stdout);
		ref_quality_t q = calc_ref_matchmap_quality(&ref_matchmap);

		//printf(" -> q.xy_ratio = %.2f\n", q.xy_ratio);

		if(q.xy_ratio < smallest)
		{
			smallest = q.xy_ratio;
			smallest_a = a;
		}
	}

	smallest = 999.9;
	smallest_a = 0.0;
	ref_quality_t q_best;
	for(float a=smallest_a-DEGTORAD(7.5); a<smallest_a+DEGTORAD(7.6); a+=DEGTORAD(2.5))
	{
		memset(&ref_matchmap, 0, sizeof(ref_matchmap));
		decim_cloud_to_ref_matchmap_free(&cloud, &ref_matchmap, 0,0,0, a);
		decim_cloud_to_ref_matchmap_occu(&cloud, &ref_matchmap, 0,0,0, a);

		//printf("a=%+6.1f: ", RADTODEG(a)); fflush(stdout);
		ref_quality_t q = calc_ref_matchmap_quality(&ref_matchmap);

		//printf(" -> q.xy_ratio = %.2f\n", q.xy_ratio);

		if(q.xy_ratio < smallest)
		{
			smallest = q.xy_ratio;
			smallest_a = a;
			q_best = q;
		}
	}

	if(q_best.xtot > q_best.ytot)
	{
		q_best.uncertain_angle = M_PI/2.0 - smallest_a;
	}
	else
	{
		q_best.uncertain_angle = -smallest_a;
	}

	printf("----> ratio %.2f, uncertain_angle = %.1f deg\n", q_best.xy_ratio, RADTODEG(q_best.uncertain_angle));

}


/*
	Parameters explained a bit by an example:
	Consider matching submap 10 to submap 50. sm10 is the ref, sm50 is the cmp.
	Assumed coordinate difference between them is given in dx, dy, dz parameters. If the submaps, cmp translated by (dx,dy,dz),
	match perfectly, the best result would then be 0,0,0.

	Now consider using several submaps as the reference. This makes a lot of sense, because in a large loop closure,
	you are very unlikely to hit the exact area where the earlier submap was taken. So if we combine multiple, we have
	more area to be matched against (with a small risk that these adjacent submaps produce some alignment error, but it
	should be small compared to large scale matching expected from large loop closures).

	So, you want to match the combined sm9,sm10,sm11 as the ref, and sm50 as cmp, but you want to build the actual loop closure
	correction numbers between sm10 and sm50. Therefore,
	n_sma = 3
	i_sma = {9, 10, 11}
	sma_corrs = { {-(world coord differences between sm9,sm10) - (any adjacent matching corrections between sm9,sm10)},
	              {0},
	              {+(world coord differences between sm10,sm11) + (any adjacent matching corrections between sm10,sm11)} }

	dx, dy, dz = world coordinate differences between sm10 and sm50
*/

int match_submaps(int n_sma, int* i_sma, result_t* sma_corrs, // Number of ref submaps, ref submap indeces, and relative corrections compared to the ref midpoint (which is again related to dx,dy,dz)
                   int i_smb,  // cmp submap index
                   double xy_range, double z_range, double yaw_range, // Matching correction ranges (mm, rad)
                   int dx, int dy, int dz, // World coordinate midpoint differences between ref and cmp
                   result_t* results_out)
{
	memset(results_out, 0, sizeof(result_t)*N_MATCH_RESULTS);

	double start_time = subsec_timestamp();

	assert(n_sma >= 1 && n_sma <= 4);
	tmp_cloud_t* smas[4];

	// Glue the ref submaps together to ref_matchmap:
	memset(&ref_matchmap, 0, sizeof(ref_matchmap));
	memset(&ref_fine_matchmap, 0, sizeof(ref_fine_matchmap));
	for(int i=0; i<n_sma; i++)
	{
		smas[i] = malloc(sizeof(tmp_cloud_t));
		load_tmp_cloud(smas[i], i_sma[i]);

		cloud_to_ref_matchmap_free(smas[i], &ref_matchmap, sma_corrs[i].x, sma_corrs[i].y, sma_corrs[i].z, sma_corrs[i].yaw);
		cloud_to_ref_fine_matchmap_free(smas[i], &ref_fine_matchmap, sma_corrs[i].x, sma_corrs[i].y, sma_corrs[i].z, sma_corrs[i].yaw);
	}

	for(int i=0; i<n_sma; i++)
	{
		cloud_to_ref_matchmap_occu(smas[i], &ref_matchmap, sma_corrs[i].x, sma_corrs[i].y, sma_corrs[i].z, sma_corrs[i].yaw);
		cloud_to_ref_fine_matchmap_occu(smas[i], &ref_fine_matchmap, sma_corrs[i].x, sma_corrs[i].y, sma_corrs[i].z, sma_corrs[i].yaw);
		free(smas[i]);
	}


	// ref_matchmap includes free space, so is slower to calculate
	// cmp_matchmap only includes occupied space, and is compared against ref_matchmap quickly

	int xy_batches = ceil(xy_range/((double)MATCHMAP_XY_RANGE*MATCHMAP_UNIT));
	int z_steps_per_dir  = ceil(z_range/((double)MATCHMAP_UNIT));

	int total_z_steps = 2*z_steps_per_dir + 1;

	/*

	Calculate number of threads used per each batch.

	examples with max threads = 4

	#steps	#batchs	#threds	z shifts for threads	equivalent batch shifts per batch
	1	1	1	0			0

	3	1	3	012			-1,0,1

	5	2	3,2	012,01			-2,-1,0; 1,2

	7	2	4,3	0123,012		-3,-2,-1,0; 1,2,3

	9	3	3,3,3	012,012,012		-4,-3,-2; -1,0,1; 2,3,4

	11	3	4,4,3	0123,0123,012

	13	4	4,3,3,3 0123,012,012,012

	...
		
	*/

	assert(total_z_steps%2 == 1);

	int z_batches = (total_z_steps + MATCHER_MAX_THREADS - 1) / MATCHER_MAX_THREADS; // = ceil(steps/threads)

	// Number of threads for each batch is threads_at_least + (batch_idx < remaining_threads)
	int threads_at_least = total_z_steps / z_batches;
	int remaining_threads = total_z_steps - threads_at_least * z_batches;


	assert(xy_batches > 0 && z_batches > 0);

	int total_xy_steps = xy_batches * MATCHMAP_XY_RANGE;

	// 1 deg step is 260mm shift at 15m distance
	// 1.5 deg step is 390mm shift at 15m distance
	double yaw_step = DEGTORAD(1.5);

	// Round yaw_range up so that midpoint is at zero.
	yaw_range = ceil(yaw_range/yaw_step) * yaw_step;
	int yaw_steps = ceil(2.0*yaw_range/yaw_step) + 1;


	int total_results = yaw_steps * xy_batches * xy_batches * total_z_steps * MATCHMAP_XYZ_N_RESULTS;

	result_t* results = calloc(total_results, sizeof(result_t));	
	assert(results);

	printf("xy_batch=%d, tot_z_step=%d (%d batch, thread %d(+%d)) --> ", xy_batches, total_z_steps, z_batches, threads_at_least, remaining_threads); fflush(stdout);

	tmp_cloud_t* smb = malloc(sizeof(tmp_cloud_t));
	load_tmp_cloud(smb, i_smb);

	for(int cur_yaw_step = 0; cur_yaw_step < yaw_steps; cur_yaw_step++)
	{
		int not_skipped = 0;
		double cur_yaw_corr = -yaw_range + (double)cur_yaw_step*yaw_step;
		for(int cur_y_batch = 0; cur_y_batch < xy_batches; cur_y_batch++)
		{
			for(int cur_x_batch = 0; cur_x_batch < xy_batches; cur_x_batch++)
			{
				double cur_z_corr = -z_steps_per_dir*MATCHMAP_UNIT;
				int cur_th_idx = 0;
				for(int cur_z_batch = 0; cur_z_batch < z_batches; cur_z_batch++)
				{

					int n_threads_now = threads_at_least + (cur_z_batch < remaining_threads);

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


					//printf("cur_z_corr=%.0f\n", cur_z_corr);


					double cx = cur_x_corr - (double)dx;
					double cy = cur_y_corr - (double)dy;
					double cz = cur_z_corr - (double)dz;

					//printf("cur batch: yaw%d, y%d, x%d, z%d, cur_x_corr=%.0f cur_y_corr=%.0f, cur_z_corr=%.0f, cx=%.0f, cy=%.0f, cz=%.0f\n",
					//	cur_yaw_step, cur_y_batch, cur_x_batch, cur_z_batch, cur_x_corr, cur_y_corr, cur_z_corr, cx, cy, cz);

					if(fabs(cx) > 6000+(MATCHMAP_XY_RANGE*MATCHMAP_UNIT) || fabs(cy) > 6000+(MATCHMAP_XY_RANGE*MATCHMAP_UNIT) || fabs(cz) > 4000)
					{
						//printf("SKIP 1\n");
						goto SKIP_BATCH;
					}


					/*
					printf("  Matching %4d vs %4d: corr (%.0f, %.0f, %.0f, %.2f) shift (%.0f, %.0f, %.0f, %.2f)\n", 
						i_sma, i_smb,
						cur_x_corr, cur_y_corr, cur_z_corr, RADTODEG(cur_yaw_corr),
						cx, cy, cz, RADTODEG(cur_yaw_corr));
					*/


					memset(&cmp_matchmap, 0, sizeof(cmp_matchmap));


					int n_vox = cloud_to_cmp_matchmap_even(smb, &cmp_matchmap, cx, cy, cz, cur_yaw_corr);


					// Quick matching finds "average" score over the matching range;
					int qscore = quick_match_matchmaps_xyz();


					/*
						Ratio distribution of one typical dataset
						> 0.3   : 8pcs
						0.2..0.3: 251pcs
						0.1..0.2: 514pcs
						< 0.1   : 1364pcs
					*/
					float qscore_ratio = (float)qscore/(float)n_vox;
					//printf("qscore = %d, n_vox = %d, ratio = %.3f\n", qscore, n_vox, qscore_ratio);
					if(qscore_ratio < 0.20)
					{
						//printf("SKIP 2\n");
						goto SKIP_BATCH;
					}

					not_skipped = 1;

					n_vox += cloud_to_cmp_matchmap_odd(smb, &cmp_matchmap, cx, cy, cz, cur_yaw_corr);



					match_xyz_result_t cur_results[MATCHER_MAX_THREADS][MATCHMAP_XYZ_N_RESULTS];
					memset(cur_results, 0, sizeof(match_xyz_result_t)*MATCHMAP_XYZ_N_RESULTS*n_threads_now);
					match_matchmaps_xyz_args_t args[MATCHER_MAX_THREADS];

					pthread_t threads[MATCHER_MAX_THREADS];

					for(int th=0; th<n_threads_now; th++)
					{
						// Z shift in MATCHMAP_UNITS is the thread index number directly
						args[th] = (match_matchmaps_xyz_args_t){th, cur_results[th]};
						int ret;
						if( (ret = pthread_create(&threads[th], NULL, match_matchmaps_xyz, (void*)&args[th])) )
						{
							printf("ERROR creating thread, ret=%d\n", ret);
							abort();
						}

					}


					for(int th=n_threads_now-1; th>=0; th--)
					{
						pthread_join(threads[th], NULL);

						int result_idx = 
							cur_yaw_step*xy_batches*xy_batches*total_z_steps*MATCHMAP_XYZ_N_RESULTS +
							cur_y_batch*xy_batches*total_z_steps*MATCHMAP_XYZ_N_RESULTS +
							cur_x_batch*total_z_steps*MATCHMAP_XYZ_N_RESULTS +
							(cur_th_idx+th)*MATCHMAP_XYZ_N_RESULTS;

						//printf("cur_th_idx=%d, th=%d, sum %d, result_idx = %d\n", cur_th_idx, th, cur_th_idx+th, result_idx);


						for(int i=0; i<MATCHMAP_XYZ_N_RESULTS; i++)
						{
							int cur_rel_score = (cur_results[th][i].score*REL_SCORE_MULT)/n_vox;
							results[result_idx + i].abscore = cur_results[th][i].score;
							//printf("i=%2d  (%+3d,%+3d,%+3d) score=%+8d, relative=%+6d\n",
							//	i, 
							//	cur_results[th][i].x_shift, cur_results[th][i].y_shift, th, cur_results[th][i].score, cur_rel_score);

							results[result_idx + i].x = cur_x_corr - cur_results[th][i].x_shift * MATCHMAP_UNIT;
							results[result_idx + i].y = cur_y_corr - cur_results[th][i].y_shift * MATCHMAP_UNIT;
							results[result_idx + i].z = cur_z_corr + th * MATCHMAP_UNIT; //TODO: remove z_shift from the struct, not needed as you can see.
							results[result_idx + i].yaw = cur_yaw_corr;
							results[result_idx + i].score = cur_rel_score;
						}
					}


					SKIP_BATCH:
					cur_z_corr += n_threads_now * MATCHMAP_UNIT;
					cur_th_idx += n_threads_now;

				}
			}

		}

		if(!not_skipped)
		{
			// For this yaw, all x,y,z batches were skipped - we are very far off. Skip a few yaw rounds.
			cur_yaw_step+=2;
		}
	}

	qsort(results, total_results, sizeof(result_t), compar_scores); // about 1.5 ms

//	double time2 = subsec_timestamp();

	#define POSTFILTER_RESULTS

	// Postfilter takes about 12 ms
	#ifdef POSTFILTER_RESULTS
		for(int i=0; i<total_results-1; i++)
		{
			if(results[i].score == -99999)
				continue;

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

//	double time3 = subsec_timestamp();

	int n_results = 0;

	#define FINETUNE

	//printf("total results (from %d pcs):\n", total_results);
	for(int i=0; i<total_results; i++)
	{
		//printf("i=%5d  (%+6d,%+6d,%+6d,%+6.2f) relscore=%+5d, abscore=%+5d\n", i, results[i].x, results[i].y, results[i].z, RADTODEG(results[i].yaw), results[i].score, results[i].abscore);
		if(results[i].score > 0 && 
		   results[i].score > results[0].score/2 &&
		   abs(results[i].x) < xy_range+200 &&
		   abs(results[i].y) < xy_range+200)
		{
			#ifdef FINETUNE
				//printf("i=%5d  (%+6d,%+6d,%+6d,%+6.2f) relscore=%+5d, abscore=%+5d ..", i, results[i].x, results[i].y, results[i].z, RADTODEG(results[i].yaw), results[i].score, results[i].abscore); fflush(stdout);;

				const double yaw_fine_step = DEGTORAD(0.75);
				const double yaw_fine_start = DEGTORAD(-2.25);
				const int n_yaw_fine_steps = 7;

	//			const double yaw_fine_step = DEGTORAD(0.5);
	//			const double yaw_fine_start = DEGTORAD(0);
	//			const int n_yaw_fine_steps = 1;

				int best_fine_relscore = -9999;
				result_t best_result = {0};

				for(int yaw_step = 0; yaw_step < n_yaw_fine_steps; yaw_step++)
				{
					double cx = results[i].x -(double)dx;
					double cy = results[i].y -(double)dy;
					double cz = results[i].z - (FINE_SMALL_Z_RANGE/2)*FINE_MATCHMAP_UNIT -(double)dz;
					double cur_yaw_corr = results[i].yaw + yaw_fine_start + (double)yaw_step*yaw_fine_step;


					assert(fabs(cx) < 15000.0 && fabs(cy) < 15000.0 && fabs(cz) < 5000.0);

					//printf("\nFine-Matching %4d vs %4d: corr (%.0f, %.0f, %.0f, %.2f) shift (%.0f, %.0f, %.0f, %.2f)\n", 
					//	i_sma, i_smb,
					//	0.0,0.0,0.0, RADTODEG(cur_yaw_corr),
					//	cx, cy, cz, RADTODEG(cur_yaw_corr));

					//double t1 = subsec_timestamp();

					memset(&cmp_fine_matchmap, 0, sizeof(cmp_fine_matchmap)); // 0.1ms

					//double t2 = subsec_timestamp();

					int n_vox = cloud_to_cmp_fine_matchmap(smb, &cmp_fine_matchmap, cx, cy, cz, cur_yaw_corr);


					//double t3 = subsec_timestamp();

					match_xyz_result_t fine_res = match_fine_matchmaps_xyz_small();

					int fine_relscore = (fine_res.score*REL_SCORE_MULT)/n_vox;

					if(fine_relscore > best_fine_relscore)
					{
						best_fine_relscore = fine_relscore;
						best_result = (result_t){
							results[i].x - fine_res.x_shift*FINE_MATCHMAP_UNIT,
							results[i].y - fine_res.y_shift*FINE_MATCHMAP_UNIT,
							results[i].z + fine_res.z_shift*FINE_MATCHMAP_UNIT - (FINE_SMALL_Z_RANGE/2)*FINE_MATCHMAP_UNIT,
							cur_yaw_corr,
							fine_relscore,
							fine_res.score};

					}
					//double t4 = subsec_timestamp();

					//printf("2: %.3fms  3: %.3fms  4: %.3fms\n", (t2-t1)*1000.0, (t3-t2)*1000.0, (t4-t3)*1000.0);

				}

				//printf(".. fine  (%+6d,%+6d,%+6d,%+6.2f) relscore=%+5d, abscore=%+5d\n", best_result.x, best_result.y, best_result.z, RADTODEG(best_result.yaw), best_result.score, best_result.abscore);

				// If there is a duplicate, only update the score, if it's higher
				for(int o = 0; o <n_results; o++)
				{
					if(
						abs(best_result.x - results_out[o].x) < 10 &&
						abs(best_result.y - results_out[o].y) < 10 &&
						abs(best_result.z - results_out[o].z) < 10 &&
						fabs(best_result.yaw - results_out[o].yaw) < DEGTORAD(0.25))
					{
						//results_out[o] have the same coords - replace if better score

						if(best_result.score > results_out[o].score)
							results_out[o] = best_result;

						goto DUPLICATE;
					}
				}

				results_out[n_results] = best_result;

				n_results++;
				if(n_results >= N_MATCH_RESULTS)
					break;

				DUPLICATE:;

			#else
				// don't finetune:
				results_out[n_results] = results[i];
				n_results++;

				if(n_results >= N_MATCH_RESULTS)
					break;

			#endif

		}
	}

	#ifdef FINETUNE
		qsort(results_out, n_results, sizeof(result_t), compar_scores);
	#endif

//	printf("re-sorted results (%d pcs):\n", n_results);
//	for(int i=0; i<n_results; i++)
//	{
//		printf("i=%5d  (%+6d,%+6d,%+6d,%+6.2f) relscore=%+5d, abscore=%5d\n", i, results_out[i].x, results_out[i].y, results_out[i].z, RADTODEG(results_out[i].yaw), results_out[i].score, results_out[i].abscore);
//	}


	double end_time = subsec_timestamp();

//	printf(" %d results, took %.3f sec\n", n_results, (end_time-start_time));

	free(results);
	free(smb);

	return n_results;
}

#define FINE_MATCH_YAW_STEP  DEGTORAD(0.75)
#define FINE_MATCH_YAW_START DEGTORAD(-3.75)
#define FINE_MATCH_YAW_STEPS 13

#define N_FINE_MATCH_RESULTS 16

typedef struct __attribute__((packed))
{
	int32_t n_results;
	result_t results[N_FINE_MATCH_RESULTS];
} adjacent_t;


adjacent_t adjacent_matches[MAX_SUBMAPS];

int save_adjacent_matches(uint32_t n_submaps)
{
	char fname[1024];
	snprintf(fname, 1024, "adjacent_matches.bin");
	FILE* f = fopen(fname, "wb");
	assert(f);

	assert(fwrite(&n_submaps, sizeof(uint32_t), 1, f) == 1);
	assert(fwrite(adjacent_matches, sizeof(adjacent_t), n_submaps, f) == n_submaps);

	fclose(f);
	return 0;
}

int load_adjacent_matches(uint32_t n_submaps)
{
	char fname[1024];
	snprintf(fname, 1024, "adjacent_matches.bin");
	FILE* f = fopen(fname, "rb");
	assert(f);

	uint32_t n_submaps_file = 0;
	assert(fread(&n_submaps_file, sizeof(uint32_t), 1, f) == 1);

	assert(n_submaps_file > 0);

	if(n_submaps_file < n_submaps)
	{
		printf("ERROR: load_adjacent_matches: adjacent match file has too few submaps. Please rerun the adjacent matching.\n");
		abort();
	}

	if(n_submaps_file > n_submaps)
	{
		printf("INFO: adjacent match file has extra submaps. Ignoring any excess.\n");
	}

	assert(fread(adjacent_matches, sizeof(adjacent_t), n_submaps, f) == n_submaps);

	fclose(f);
	return 0;
}



int  fine_match_submaps(int i_sma, int i_smb,  // Ref and cmp submap indeces
                        int dx, int dy, int dz, // World coordinate midpoint differences between ref and cmp
                        result_t* results_out)
{
	assert(results_out);

	static result_t results[FINE_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS];
	memset(results, 0, sizeof(result_t)*FINE_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS);	


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
	cloud_to_ref_fine_matchmap_free(&sma, &ref_fine_matchmap, 0,0,0,0);
	cloud_to_ref_fine_matchmap_occu(&sma, &ref_fine_matchmap, 0,0,0,0);

	// TODO: These batches could run in parallel, in multiple threads
	for(int cur_yaw_step = 0; cur_yaw_step < FINE_MATCH_YAW_STEPS; cur_yaw_step++)
	{
		double cur_yaw_corr = FINE_MATCH_YAW_START + (double)cur_yaw_step*FINE_MATCH_YAW_STEP;
//		double cur_yaw_corr = 0.0;
		// This part is per-thread.

		double cx = -(double)dx;
		double cy = -(double)dy;
		double cz = -(double)dz;

		assert(fabs(cx) < 10000.0 && fabs(cy) < 10000.0 && fabs(cz) < 2000.0);

//		printf("  Fine-Matching %4d vs %4d: corr (%.0f, %.0f, %.0f, %.2f) shift (%.0f, %.0f, %.0f, %.2f)\n", 
//			i_sma, i_smb,
//			0.0,0.0,0.0, RADTODEG(cur_yaw_corr),
//			cx, cy, cz, RADTODEG(cur_yaw_corr));

		memset(&cmp_fine_matchmap, 0, sizeof(cmp_fine_matchmap));

		int n_vox = cloud_to_cmp_fine_matchmap(&smb, &cmp_fine_matchmap, cx, cy, cz, cur_yaw_corr);

		//printf("%d active voxels in cmp_matchmap\n", n_vox);

		match_xyz_result_t batch_results[MATCHMAP_XYZ_N_RESULTS];
		memset(batch_results, 0, sizeof(batch_results));

		match_fine_matchmaps_xyz(batch_results);

		int result_idx = 
			cur_yaw_step*MATCHMAP_XYZ_N_RESULTS;

		for(int i=0; i<MATCHMAP_XYZ_N_RESULTS; i++)
		{
			int cur_rel_score = ((int64_t)batch_results[i].score*(int64_t)FINE_REL_SCORE_MULT)/((int64_t)n_vox*(int64_t)FINE_REL_SCORE_DIV);
			//printf("i=%2d  (%+3d,%+3d,%+3d) score=%+8d, relative=%+6d\n",
			//	i, 
			//	batch_results[i].x_shift, batch_results[i].y_shift, batch_results[i].z_shift, batch_results[i].score, cur_rel_score);

			results[result_idx + i].x = batch_results[i].x_shift * FINE_MATCHMAP_UNIT;
			results[result_idx + i].y = batch_results[i].y_shift * FINE_MATCHMAP_UNIT;
			results[result_idx + i].z = batch_results[i].z_shift * FINE_MATCHMAP_UNIT;
			results[result_idx + i].yaw = cur_yaw_corr;
			results[result_idx + i].score = cur_rel_score;
			results[result_idx + i].abscore = (int64_t)batch_results[i].score/(int64_t)FINE_REL_SCORE_DIV;
		}
	}

	qsort(results, FINE_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS, sizeof(result_t), compar_scores);

	#define FINE_POSTFILTER_RESULTS

	#ifdef FINE_POSTFILTER_RESULTS
		for(int i=0; i<FINE_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS-1; i++)
		{
			for(int o=i+1; o<FINE_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS; o++)
			{
				double transl_dist = sqrt( sq(results[i].x-results[o].x) + sq(results[i].y-results[o].y) + sq(results[i].z-results[o].z) );
				double rot_dist = fabs(results[i].yaw - results[o].yaw);

				// dist_number: 1 degree is like 256 millimeters (0.75deg is like 192 mm)
				double dist_number = transl_dist + RADTODEG(rot_dist)*256.0;

		//		printf("i=%d, o=%d, transl_dist=%.0f  rot_dist=%.3f,  dist_number=%.0f, score[i]=%d, score[o]=%d", i, o, transl_dist, rot_dist, dist_number, results[i].score, results[o].score);

				if((dist_number < 150.0) ||
				   (dist_number < 300.0 && 105*results[o].score < 100*results[i].score) ||
				   (dist_number < 450.0 && 110*results[o].score < 100*results[i].score) ||
				   (dist_number < 600.0 && 115*results[o].score < 100*results[i].score) ||
				   (dist_number < 750.0 && 120*results[o].score < 100*results[i].score))
				{
					results[o].score = -99999;
		//			printf("  mash it!");
				}

		//		printf("\n");
			}
		}
	#endif

	int n_results = 0;
	printf("total results (from %d pcs):\n", FINE_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS);
	for(int i=0; i<FINE_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS; i++)
	{
		if(results[i].score > 0 && results[i].score > results[0].score/2)
		{
			printf("i=%5d  (%+6d,%+6d,%+6d,%+6.2f) relscore=%+5d abscore=%+5d\n",
				i, results[i].x, results[i].y, results[i].z, RADTODEG(results[i].yaw), results[i].score, results[i].abscore);


			results_out[n_results] = results[i];
			n_results++;
			if(n_results >= N_FINE_MATCH_RESULTS)
				break;
		}
	}

//	assert(n_results >= 1);
//	if(n_results == 1)
//	{
//		results_out[1] = results[1]; // Restore filtered-out second-best result.
		// TODO: .score is set at -99999. Work a way to revert this.
//		n_results++;
//	}

	assert(n_results >= 2);

	return n_results;
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
			po_coords_t po = po_coords(cloud->points[i].px+ref_x, cloud->points[i].py+ref_y, cloud->points[i].pz+ref_z, rl);
			uint8_t* p_vox = get_p_voxel(po, rl);
			if(((*p_vox) & 0x0f) < 15)
				(*p_vox)++;
			mark_page_changed(po.px, po.py, po.pz);
		}
	}
}


static inline void cloud_insert_point(tmp_cloud_t* cloud, int16_t sx, int16_t sy, int16_t sz, int32_t px, int32_t py, int32_t pz)
{
	if(cloud->n_points >= MAX_POINTS)
	{
		printf("WARNING: Ignoring point, cloud full.\n");
		return;
	}
	assert(cloud->n_points >= 0);

	assert(sx != 0 || sy != 0 || sz != 0); // Would be an alarming coincidence. Remove this assert later as it's a real possibility given enough data.

	cloud->points[cloud->n_points].sx = sx;
	cloud->points[cloud->n_points].sy = sy;
	cloud->points[cloud->n_points].sz = sz;

	cloud->points[cloud->n_points].px = px;
	cloud->points[cloud->n_points].py = py;
	cloud->points[cloud->n_points].pz = pz;

	cloud->n_points++;
}

static void voxfilter_to_cloud(voxfilter_t* voxfilter, tmp_cloud_t* cloud)
{
//	int insert_cnt = 0;
	for(int yy=0; yy<VOXFILTER_YS; yy++)
	{
		for(int xx=0; xx<VOXFILTER_XS; xx++)
		{
			for(int zz=0; zz<VOXFILTER_ZS; zz++)
			{
				for(int i=0; i<MAX_SRC_POSE_REFS; i++)
				{
					voxfilter_point_t* p = &(voxfilter->points[xx][yy][zz][i]);
					if(p->cnt == 0)
					{
						break;
					}

					assert(p->src_seq_id > 0 && p->src_seq_id < VOXFILTER_N_SCANS*N_SENSORS);

					int32_t x = p->x / p->cnt;
					int32_t y = p->y / p->cnt;
					int32_t z = p->z / p->cnt;

					cloud_insert_point(cloud, 
						voxfilter->ray_sources[p->src_seq_id].x, 
						voxfilter->ray_sources[p->src_seq_id].y,
						voxfilter->ray_sources[p->src_seq_id].z, 
						x, y, z);
					//insert_cnt++;
				}
			}
		}
	}

	//printf("INFO: voxfilter_to_cloud inserted %d points\n", insert_cnt);
}

static inline void voxfilter_insert_point(tmp_cloud_t* cloud, voxfilter_t* voxfilter, int subsubmap_seq_id, int32_t x, int32_t y, int32_t z)
{
	int vox_x = x/VOXFILTER_STEP + VOXFILTER_XS/2;
	int vox_y = y/VOXFILTER_STEP + VOXFILTER_YS/2;
	int vox_z = z/VOXFILTER_STEP + VOXFILTER_ZS/2;

	assert(subsubmap_seq_id > 0 && subsubmap_seq_id < VOXFILTER_N_SCANS*N_SENSORS);

	if(vox_x < 0 || vox_x > VOXFILTER_XS-1 || vox_y < 0 || vox_y > VOXFILTER_YS-1 || vox_z < 0 || vox_z > VOXFILTER_ZS-1)
	{
//		printf("INFO: voxfilter: skipping OOR point %d, %d, %d\n", vox_x, vox_y, vox_z);
		cloud_insert_point(cloud, voxfilter->ray_sources[subsubmap_seq_id].x,voxfilter->ray_sources[subsubmap_seq_id].y,voxfilter->ray_sources[subsubmap_seq_id].z, x, y, z);
		return;
	}
	
	int found = 0;
	for(int i=0; i < MAX_SRC_POSE_REFS; i++)
	{
		if(voxfilter->points[vox_x][vox_y][vox_z][i].src_seq_id == subsubmap_seq_id)
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
				voxfilter->points[vox_x][vox_y][vox_z][i].src_seq_id = subsubmap_seq_id;
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




// ssm_seq_id is sequence id for voxfilter. 0 is invalid value, must start from 1, and be unique for each unique TOF image
void tof_to_voxfilter_and_cloud(int is_narrow, uint16_t* ampldist, hw_pose_t pose, int ssm_seq_id, int sidx, int32_t ref_x, int32_t ref_y, int32_t ref_z,
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


	int sx = global_sensor_x;
	int sy = global_sensor_y;
	int sz = global_sensor_z;

	if(voxfilter)
	{
		assert(ssm_seq_id > 0 && ssm_seq_id < VOXFILTER_N_SCANS*N_SENSORS);
		voxfilter->ray_sources[ssm_seq_id].x = sx;
		voxfilter->ray_sources[ssm_seq_id].y = sy;
		voxfilter->ray_sources[ssm_seq_id].z = sz;
	}

	int y_ignore=1;
	int x_ignore=1;

	// Horrible temporary kludge
	if(!is_narrow)
	{

		int32_t nearfield_avg_dist = 0;
		int nearfield_avg_n = 0;
		for(volatile int py=10; py<TOF_YS-10; py++)
		{
			for(volatile int px=50; px<TOF_XS-50; px++)
			{
				int32_t dist = ampldist[(py)*TOF_XS+(px)]&DIST_MASK;

//				printf("py=%d, px=%d, dist=%d\n", py, px, dist);

				if(dist == DIST_UNDEREXP) dist = 2000>>DIST_SHIFT;

				nearfield_avg_dist += dist; // -O3 segfaults here for no apparent reason - defining loop variables volatile prevents buggy optimization.
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

	//printf("x_ignore=%d y_ignore=%d\n", x_ignore, y_ignore);
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
					voxfilter_insert_point(cloud, voxfilter, ssm_seq_id, x, y, z);
				else
					cloud_insert_point(cloud, sx, sy, sz, x, y, z);

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

#define MIN_CLOSURE_LEN 5
#define MAX_CLOSURE_LEN 256

typedef struct __attribute__((packed))
{
	int32_t first;
	int32_t last;

	// corrs[0] is the chosen correction from sm#first to sm#first+1, and so on
	result_t corrs[MAX_CLOSURE_LEN];
	// Original direct match from first to last
	result_t match;

	int32_t score;

} closure_t;

#define MAX_CLOSURES 256

int save_closures(closure_t* closures, uint32_t n_closures)
{
	char fname[1024];
	snprintf(fname, 1024, "closures.bin");
	FILE* f = fopen(fname, "wb");
	assert(f);

	assert(fwrite(&n_closures, sizeof(uint32_t), 1, f) == 1);
	assert(fwrite(closures, sizeof(closure_t), n_closures, f) == n_closures);

	fclose(f);
	return 0;
}

int load_closures(closure_t* closures, uint32_t* n_closures)
{
	char fname[1024];
	snprintf(fname, 1024, "closures.bin");
	FILE* f = fopen(fname, "rb");
	if(!f)
		return -1;

	uint32_t n_closures_file = 0;
	assert(fread(&n_closures_file, sizeof(uint32_t), 1, f) == 1);

	assert(n_closures_file < MAX_CLOSURES);

	assert(fread(closures, sizeof(closure_t), n_closures_file, f) == n_closures_file);

	*n_closures = n_closures_file;

	fclose(f);
	return 0;
}


static int build_closure(submap_meta_t* sms, int first, int last, result_t* closure_result, closure_t* closure_out)
{
	int len = last - first;
	assert(len >= MIN_CLOSURE_LEN && len <= MAX_CLOSURE_LEN);

	static adjacent_t adj_copy[MAX_CLOSURE_LEN];

	memcpy(adj_copy, &adjacent_matches[first], sizeof(adjacent_t)*len);

	// Give extra weight for results that help achieving the correct yaw

	float avg_yaw = closure_result->yaw / (float)len;

//	printf("closure_yaw = %.2f deg, avg_yaw = %.3f deg\n", RADTODEG(closure_result->yaw), RADTODEG(avg_yaw));

	float yawsum = 0.0;
//	for(int i = 0; i < len; i++)
//	{
//		yawsum += adj_copy[i].results[0].yaw;
//	}

//	printf("before weighing, yawsum using best results = %.2f deg\n", RADTODEG(yawsum));

/*
	for(int i = 0; i < len; i++)
	{
		int n = adj_copy[i].n_results;
		assert(n > 0 && n <= N_FINE_MATCH_RESULTS);
		for(int r=0; r<n; r++)
		{
			printf("i=%3d r=%3d  (%+6d,%+6d,%+6d,%+6.2f) relscore=%+5d, abscore=%5d\n", i, r, adj_copy[i].results[r].x, adj_copy[i].results[r].y, adj_copy[i].results[r].z, RADTODEG(adj_copy[i].results[r].yaw), adj_copy[i].results[r].score, adj_copy[i].results[r].abscore);
		}
	}
*/

	/*
		We have our chosen loop closure correction (single correction between first and last),
		then each submap between first,last has been matched to its neighbor. For this matching,
		we have several options.

		Finding optimum combination from these begins with weighing. The end-to-end loop closure
		yaw correction is divided per submap, then the scores that already are close to this
		correction value is given a tiny bit of extra weight compared to other scores. In cases
		where scores are already close, which is most of the time, this weighing yields nearly
		the desired loop closure yaw correction; but if there is any strong disagreement in any
		of the adjacent matchings, the small amount of weighing doesn't ruin it.

		After the weighing, the likelihood of the highest scores being correct is higher.
	*/
	for(int i = 0; i < len; i++)
	{
		int n = adj_copy[i].n_results;
		assert(n > 0 && n <= N_FINE_MATCH_RESULTS);

		for(int r = 0; r < N_FINE_MATCH_RESULTS; r++)
		{
			float diff = fabs(avg_yaw - adj_copy[i].results[r].yaw);
			/*
				0 deg: 1.0
				1 deg: 0.975
				2 deg: 0.950
				4 deg: 0.900
			*/
			float weight = 1.0 - RADTODEG(diff)*0.025;

			adj_copy[i].results[r].score = (float)adj_copy[i].results[r].score * weight;
		}

		qsort(adj_copy[i].results, n, sizeof(result_t), compar_scores);
	}
/*
	printf("AFTER QSORT\n");
	for(int i = 0; i < len; i++)
	{
		int n = adj_copy[i].n_results;
		assert(n > 0 && n <= N_FINE_MATCH_RESULTS);
		for(int r=0; r<n; r++)
		{
			printf("i=%3d r=%3d  (%+6d,%+6d,%+6d,%+6.2f) relscore=%+5d, abscore=%5d\n", i, r, adj_copy[i].results[r].x, adj_copy[i].results[r].y, adj_copy[i].results[r].z, RADTODEG(adj_copy[i].results[r].yaw), adj_copy[i].results[r].score, adj_copy[i].results[r].abscore);
		}
	}
*/
	yawsum = 0.0;
	double zsum = 0.0;
	for(int i = 0; i < len; i++)
	{
		yawsum += adj_copy[i].results[0].yaw;
		zsum += adj_copy[i].results[0].z;
	}

//	printf("after weighing, yawsum using best results = %.2f deg, \n", RADTODEG(yawsum));

	assert(len > 1);

	// Optimally, we would try out all combinations of different results playing together.
	// Even though running the coordinates through is a non-intensive task, it's of course
	// far from possible, think about even just testing 4 best results for 50-long loop closure:
	// 4^50 = 1.3e30 combinations!

	int combination[3] = {-1, -1, -1};


	// Overridden by later code, if enabled
	float final_residual_avg_yaw = (closure_result->yaw - yawsum) / (float)len;
	double final_residual_avg_z = ((double)closure_result->z - zsum) / (double)len;


#if 1

	{
		int64_t ref_sqerr = -1; // Reference cumulated matching error when every adjacent results is chosen as [0]
		int64_t min_sqerr = INT64_MAX;

		for(int cur_n0=-3; cur_n0<len; cur_n0++)
		{
			for(int cur_n1=cur_n0+1; cur_n1<len; cur_n1++)
			{
				for(int cur_n2=cur_n1+1; cur_n2<len; cur_n2++)
				{
					if(cur_n2 < 0 && cur_n1 < 0 && cur_n0 < 0 && ref_sqerr >= 0) // already did this
						continue;

					// Adjacent matches MUST always have at least two solutions - not checked here, assumed (assert elsewhere)

					float cur_yawsum = 0.0;
					double cur_zsum = 0.0;
					for(int i = 0; i < len; i++)
					{
						int r = (i==cur_n0||i==cur_n1||i==cur_n2)?1:0;
						cur_yawsum += adj_copy[i].results[r].yaw;
						cur_zsum += adj_copy[i].results[r].z;
					}

					float residual_avg_yaw = (closure_result->yaw - cur_yawsum) / (float)len;
					float residual_avg_z = ((double)closure_result->z - cur_zsum) / (float)len;

					int32_t new_avg_x, new_avg_y, new_avg_z;
					double running_x = 0.0;
					double running_y = 0.0;
					double running_z = 0.0;
					double running_yaw = 0.0;

					for(int i = 0; i < len; i++)
					{
						int r = (i==cur_n0||i==cur_n1||i==cur_n2)?1:0;

						int sm = first + i;

						int dx = sms[sm+1].avg_x - sms[sm].avg_x;
						int dy = sms[sm+1].avg_y - sms[sm].avg_y;
						//int dz = sms[sm+1].avg_z - sms[sm].avg_z;

						double x_corr = -1.0*adj_copy[i].results[r].x;
						double y_corr = -1.0*adj_copy[i].results[r].y;
						double z_corr = -1.0*((double)adj_copy[i].results[r].z + (double)residual_avg_z);
						double yaw_corr = 1.0*adj_copy[i].results[r].yaw + residual_avg_yaw;

						running_x += x_corr;
						running_y += y_corr;
						running_z += z_corr;
						running_yaw += yaw_corr;

						running_x += (double)(dx) * cos(running_yaw) - (double)(dy) * sin(running_yaw)  - (double)(dx);
						running_y += (double)(dx) * sin(running_yaw) + (double)(dy) * cos(running_yaw)  - (double)(dy);

						new_avg_x = sms[sm+1].avg_x + running_x;
						new_avg_y = sms[sm+1].avg_y + running_y;
						new_avg_z = sms[sm+1].avg_z + running_z;

						//printf("i=%d, %d vs %d, using r=%d (%+5.0f, %+5.0f, %+5.0f, %+6.2f), running (%+7.0f, %+7.0f, %+7.0f, %+8.2f), new_avg %d, %d, %d\n",
						//	i, sm, sm+1, r, x_corr, y_corr, z_corr, RADTODEG(yaw_corr), running_x, running_y, running_z, RADTODEG(running_yaw), new_avg_x, new_avg_y, new_avg_z);
					}

					int32_t new_dx = new_avg_x - sms[last].avg_x;
					int32_t new_dy = new_avg_y - sms[last].avg_y;
					int32_t new_dz = new_avg_z - sms[last].avg_z;

					int32_t cdiff_x = -1*closure_result->x - new_dx;
					int32_t cdiff_y = -1*closure_result->y - new_dy;
					int32_t cdiff_z = -1*closure_result->z - new_dz;

					int64_t sqerr = (sq((int64_t)cdiff_x) + sq((int64_t)cdiff_y) + sq((int64_t)cdiff_z));

					if(cur_n2 < 0 && cur_n1 < 0 && cur_n0 < 0)
					{
						assert(ref_sqerr == -1); // verify that we do this only once
						ref_sqerr = sqerr;
					}

					if(sqerr < min_sqerr)
					{
						min_sqerr = sqerr;
						combination[0] = cur_n0;
						combination[1] = cur_n1;
						combination[2] = cur_n2;
						final_residual_avg_yaw = residual_avg_yaw;
						final_residual_avg_z = residual_avg_z;
					}

					//printf("----------------> n(%3d,%3d,%3d), resi_avg_yaw=%.3f, new_d (%+6d, %+6d, %+6d), closure (%+6d, %+6d, %+6d), diff (%+5d, %+5d, %+5d), err=%.0f\n",
					//	cur_n0, cur_n1, cur_n2,
					//	RADTODEG(residual_avg_yaw), 
					//	new_dx, new_dy, new_dz, 
					//	-1*closure_result->x, -1*closure_result->y, -1*closure_result->z,
					//	cdiff_x, cdiff_y, cdiff_z, sqrt(sqerr));
				}
			}
		}

		assert(ref_sqerr >= 0);
		assert(min_sqerr < INT64_MAX);

		//printf("BEST-------------------> Best combination: use 2nd result from (%3d,%3d,%3d), err_ref=%.0f, err=%.0f (%.2f%%)\n",
		//	combination[0], combination[1], combination[2], sqrt(ref_sqerr), sqrt(min_sqerr), 100.0*sqrt(min_sqerr)/sqrt(ref_sqerr));

		double remaining_err_per_submap = sqrt(min_sqerr)/len;
		if(remaining_err_per_submap > (double)XY_ACCUM_UNCERT_BY_SUBMAP*0.8)
		{
		//	printf("BAILED OUT after 2nd result combining, remaining err per submap = %.0f mm\n", remaining_err_per_submap);
			return -1;
		}

	}

#endif

	// But, we do expect that most of the time, the best match is good, and probably there is one
	// specifically bad actor. Let's try out selecting alternative matches for one matching at a time,
	// to find which combination of submap and its result gives biggest reduction in accumulated error.

	int min_at_bad = -1;
	int min_at_solution = -1;

#if 1
	{

		int64_t ref_sqerr = -1; // Reference cumulated matching error when every adjacent results is chosen as [0]
		int64_t min_sqerr = INT64_MAX;

		for(int cur_bad=0; cur_bad<len; cur_bad++)
		{
			int cur_bad_n_solutions = adj_copy[cur_bad].n_results;
			for(int cur_solution = 0; cur_solution<cur_bad_n_solutions; cur_solution++)
			{
				if(cur_bad > 0 && cur_solution == 0)
					continue; // Do the "reference" all [0] only once.

				float cur_yawsum = 0.0;
				double cur_zsum = 0.0;

				for(int i = 0; i < len; i++)
				{
					int r = (i==cur_bad)?cur_solution:((i==combination[0]||i==combination[1]||i==combination[2])?1:0);
					cur_yawsum += adj_copy[i].results[r].yaw;
					cur_zsum += adj_copy[i].results[r].z;
				}

				float residual_avg_yaw = (closure_result->yaw - cur_yawsum) / (float)len;
				float residual_avg_z = ((double)closure_result->z - cur_zsum) / (float)len;


				int32_t new_avg_x, new_avg_y, new_avg_z;
				double running_x = 0.0;
				double running_y = 0.0;
				double running_z = 0.0;
				double running_yaw = 0.0;

				for(int i = 0; i < len; i++)
				{
					int r = (i==cur_bad)?cur_solution:((i==combination[0]||i==combination[1]||i==combination[2])?1:0);

					int sm = first + i;

					int dx = sms[sm+1].avg_x - sms[sm].avg_x;
					int dy = sms[sm+1].avg_y - sms[sm].avg_y;
					//int dz = sms[sm+1].avg_z - sms[sm].avg_z;

					double x_corr = -1.0*adj_copy[i].results[r].x;
					double y_corr = -1.0*adj_copy[i].results[r].y;
					double z_corr = -1.0*((double)adj_copy[i].results[r].z + (double)residual_avg_z);
					double yaw_corr = 1.0*adj_copy[i].results[r].yaw + residual_avg_yaw;

					running_x += x_corr;
					running_y += y_corr;
					running_z += z_corr;
					running_yaw += yaw_corr;

					running_x += (double)(dx) * cos(running_yaw) - (double)(dy) * sin(running_yaw)  - (double)(dx);
					running_y += (double)(dx) * sin(running_yaw) + (double)(dy) * cos(running_yaw)  - (double)(dy);

					new_avg_x = sms[sm+1].avg_x + running_x;
					new_avg_y = sms[sm+1].avg_y + running_y;
					new_avg_z = sms[sm+1].avg_z + running_z;

					//printf("i=%d, %d vs %d, using r=%d (%+5.0f, %+5.0f, %+5.0f, %+6.2f), running (%+7.0f, %+7.0f, %+7.0f, %+8.2f), new_avg %d, %d, %d\n",
					//	i, sm, sm+1, r, x_corr, y_corr, z_corr, RADTODEG(yaw_corr), running_x, running_y, running_z, RADTODEG(running_yaw), new_avg_x, new_avg_y, new_avg_z);
				}

				int32_t new_dx = new_avg_x - sms[last].avg_x;
				int32_t new_dy = new_avg_y - sms[last].avg_y;
				int32_t new_dz = new_avg_z - sms[last].avg_z;

				int32_t cdiff_x = -1*closure_result->x - new_dx;
				int32_t cdiff_y = -1*closure_result->y - new_dy;
				int32_t cdiff_z = -1*closure_result->z - new_dz;

				int64_t sqerr = (sq((int64_t)cdiff_x) + sq((int64_t)cdiff_y) + sq((int64_t)cdiff_z));

				if(cur_solution == 0)
				{
					assert(ref_sqerr == -1);
					ref_sqerr = sqerr;
				}

				if(sqerr < min_sqerr)
				{
					min_sqerr = sqerr;
					min_at_bad = cur_bad;
					min_at_solution = cur_solution;
					final_residual_avg_yaw = residual_avg_yaw;
					final_residual_avg_z = residual_avg_z;

				}

				//printf("----------------> bad = %d, solu=%d, resi_avg_yaw=%.3f, new_d (%+6d, %+6d, %+6d), closure (%+6d, %+6d, %+6d), diff (%+5d, %+5d, %+5d), err=%.0f\n",
				//	cur_bad, cur_solution, 
				//	RADTODEG(residual_avg_yaw), 
				//	new_dx, new_dy, new_dz, 
				//	-1*closure_result->x, -1*closure_result->y, -1*closure_result->z,
				//	cdiff_x, cdiff_y, cdiff_z, sqrt(sqerr));
			}
		}

		assert(ref_sqerr >= 0);
		assert(min_at_bad > -1);
		assert(min_at_solution > -1);
		assert(min_sqerr < INT64_MAX);

		if(min_sqerr >= ref_sqerr)
		{
			//printf("BEST-------------------> REFERENCE (all [0]) is the best\n");
		}
		else
		{
			//printf("BEST-------------------> Best: bad=%d, solu=%d, err_ref=%.0f, err=%.0f (%.2f%%)\n",
			//	min_at_bad, min_at_solution, sqrt(ref_sqerr), sqrt(min_sqerr), 100.0*sqrt(min_sqerr)/sqrt(ref_sqerr));
		}

		double remaining_err_per_submap = sqrt(min_sqerr)/len;
		if(remaining_err_per_submap > (double)XY_ACCUM_UNCERT_BY_SUBMAP*0.7)
		{
		//	printf("BAILED OUT after single nth result experiment, remaining err per submap = %.0f mm\n", remaining_err_per_submap);
			return -1;
		}

	}

#endif

	/*
		Calculate residual x,y,z correction
		If there is a closed-form solution, it's too difficult for me to find.
		For Z, it's easy.
		For x,y, let's just iterate.
	*/

#if 1

	int min_at_x = 0;
	int min_at_y = 0;

	{
		int64_t ref_sqerr = -1;
		int64_t min_sqerr = INT64_MAX;


		//int32_t best_cdiff_x = 0;
		//int32_t best_cdiff_y = 0;
		//int32_t best_cdiff_z = 0;

		for(int x = -XY_ACCUM_UNCERT_BY_SUBMAP; x < XY_ACCUM_UNCERT_BY_SUBMAP; x += 2)
		{
			for(int y = -XY_ACCUM_UNCERT_BY_SUBMAP; y < XY_ACCUM_UNCERT_BY_SUBMAP; y += 2)
			{
				int32_t new_avg_x, new_avg_y, new_avg_z;
				double running_x = 0.0;
				double running_y = 0.0;
				double running_z = 0.0;
				double running_yaw = 0.0;

				for(int i = 0; i < len; i++)
				{
					int r = (i==min_at_bad)?min_at_solution:((i==combination[0]||i==combination[1]||i==combination[2])?1:0);

					int sm = first + i;

					int dx = sms[sm+1].avg_x - sms[sm].avg_x;
					int dy = sms[sm+1].avg_y - sms[sm].avg_y;
					//int dz = sms[sm+1].avg_z - sms[sm].avg_z;

	//					double dlen = sqrt(sq((int64_t)dx)+sq((int64_t)dy)+sq((int64_t)dz));
	//					double unit_dx = (double)dx/dlen;
	//					double unit_dy = (double)dy/dlen;
	//					double unit_dz = (double)dz/dlen;
	//				double drift_ang = atan2(dy,dx) + M_PI/2.0;
	//				double x_drift = cos(drift_ang) * (double)dx * w;
	//				double y_drift = sin(drift_ang) * (double)dy * w;
	//				double x_corr = -1.0*(adj_copy[i].results[r].x + x_drift);
	//				double y_corr = -1.0*(adj_copy[i].results[r].y + y_drift);


					double x_corr = -1.0*(adj_copy[i].results[r].x + x);
					double y_corr = -1.0*(adj_copy[i].results[r].y + y);
					double z_corr = -1.0*((double)adj_copy[i].results[r].z + (double)final_residual_avg_z);
					double yaw_corr = 1.0*adj_copy[i].results[r].yaw + final_residual_avg_yaw;

					running_x += x_corr;
					running_y += y_corr;
					running_z += z_corr;
					running_yaw += yaw_corr;

					running_x += (double)(dx) * cos(running_yaw) - (double)(dy) * sin(running_yaw)  - (double)(dx);
					running_y += (double)(dx) * sin(running_yaw) + (double)(dy) * cos(running_yaw)  - (double)(dy);

					new_avg_x = sms[sm+1].avg_x + running_x;
					new_avg_y = sms[sm+1].avg_y + running_y;
					new_avg_z = sms[sm+1].avg_z + running_z;

					//printf("i=%d, %d vs %d, using r=%d (%+5.0f, %+5.0f, %+5.0f, %+6.2f), running (%+7.0f, %+7.0f, %+7.0f, %+8.2f), new_avg %d, %d, %d\n",
					//	i, sm, sm+1, r, x_corr, y_corr, z_corr, RADTODEG(yaw_corr), running_x, running_y, running_z, RADTODEG(running_yaw), new_avg_x, new_avg_y, new_avg_z);
				}

				int32_t new_dx = new_avg_x - sms[last].avg_x;
				int32_t new_dy = new_avg_y - sms[last].avg_y;
				int32_t new_dz = new_avg_z - sms[last].avg_z;

				int32_t cdiff_x = -1*closure_result->x - new_dx;
				int32_t cdiff_y = -1*closure_result->y - new_dy;
				int32_t cdiff_z = -1*closure_result->z - new_dz;

				int64_t sqerr = (sq((int64_t)cdiff_x) + sq((int64_t)cdiff_y) + sq((int64_t)cdiff_z));

				if(sqerr < min_sqerr)
				{
					min_sqerr = sqerr;
					min_at_x = x;
					min_at_y = y;
					//best_cdiff_x = cdiff_x;
					//best_cdiff_y = cdiff_y;
					//best_cdiff_z = cdiff_z;
				}

				if(x == 0 && y == 0)
				{
					assert(ref_sqerr == -1);
					ref_sqerr = sqerr;
				}

				//printf("---------------------> resi_avg_yaw=%.3f, new_d (%+6d, %+6d, %+6d), closure (%+6d, %+6d, %+6d), diff (%+5d, %+5d, %+5d), err=%.0f\n",
				//	RADTODEG(final_residual_avg_yaw), 
				//	new_dx, new_dy, new_dz, 
				//	-1*closure_result->x, -1*closure_result->y, -1*closure_result->z,
				//	cdiff_x, cdiff_y, cdiff_z, sqrt(sqerr));
			}
		}

		assert(ref_sqerr >= 0);
		assert(min_sqerr < INT64_MAX);


	//	printf("BEST-W-----------------> Best: x=%+5d y=%+5d, err_ref=%.0f, err=%.0f (%.2f%%) diff (%+5d, %+5d, %+5d)\n",
	//		min_at_x, min_at_y, sqrt(ref_sqerr), sqrt(min_sqerr), 100.0*sqrt(min_sqerr)/sqrt(ref_sqerr),
	//		best_cdiff_x, best_cdiff_y, best_cdiff_z);



	}


	// Build the final path:


	#define SCORE_COEFF 5
	#define ABSCORE_COEFF 1
	{
		int64_t score_acc = 0;
		/*
		int32_t new_avg_x, new_avg_y, new_avg_z;
		double running_x = 0.0;
		double running_y = 0.0;
		double running_z = 0.0;
		double running_yaw = 0.0;
		*/
		for(int i = 0; i < len; i++)
		{
			int r = (i==min_at_bad)?min_at_solution:((i==combination[0]||i==combination[1]||i==combination[2])?1:0);

			closure_out->corrs[i].x = adj_copy[i].results[r].x + min_at_x;
			closure_out->corrs[i].y = adj_copy[i].results[r].y + min_at_y;
			closure_out->corrs[i].z = adj_copy[i].results[r].z + final_residual_avg_z;
			closure_out->corrs[i].yaw = adj_copy[i].results[r].yaw + final_residual_avg_yaw;

			int64_t cur_score = SCORE_COEFF*adj_copy[i].results[r].score + ABSCORE_COEFF*adj_copy[i].results[r].abscore;
			//printf("        cur_score = %ld\n", cur_score);
			score_acc += cur_score;

			/*
			int dx = sms[sm+1].avg_x - sms[sm].avg_x;
			int dy = sms[sm+1].avg_y - sms[sm].avg_y;
			int dz = sms[sm+1].avg_z - sms[sm].avg_z;

			double x_corr = -1.0*(closure_out->corrs[i].x);
			double y_corr = -1.0*(closure_out->corrs[i].y);
			double z_corr = -1.0*(closure_out->corrs[i].z);
			double yaw_corr = 1.0*closure_out->corrs[i].yaw;

			running_x += x_corr;
			running_y += y_corr;
			running_z += z_corr;
			running_yaw += yaw_corr;

			running_x += (double)(dx) * cos(running_yaw) - (double)(dy) * sin(running_yaw)  - (double)(dx);
			running_y += (double)(dx) * sin(running_yaw) + (double)(dy) * cos(running_yaw)  - (double)(dy);

			new_avg_x = sms[sm+1].avg_x + running_x;
			new_avg_y = sms[sm+1].avg_y + running_y;
			new_avg_z = sms[sm+1].avg_z + running_z;
			*/
		}



		// Weight:
		// min_at_x (200) == max uncert (200) -> 0
		// min_at_x (100) == 0.5  * max uncert (200) -> 0.75
		// min_at_x  (50) == 0.25 * max uncert (200) -> 0.9375
		// min_at_x  (20) == 0.1  * max uncert (200) -> 0.99
		double x_w = 1.0 - (((double)sq(min_at_x)) / ((double)sq(XY_ACCUM_UNCERT_BY_SUBMAP)));
		double y_w = 1.0 - (((double)sq(min_at_y)) / ((double)sq(XY_ACCUM_UNCERT_BY_SUBMAP)));
//		double z_w = 1.0 - (((double)sq(final_residual_avg_z)) / ((double)sq(Z_ACCUM_UNCERT_BY_SUBMAP)));

//		printf("score_acc = %ld\n", score_acc);
		score_acc /= (len*SCORE_COEFF*ABSCORE_COEFF);
//		printf("score_acc = %ld\n", score_acc);

//		printf("%.5f  %.5f  %.5f\n", x_w, y_w, z_w);
		int32_t score = (double)score_acc * x_w * y_w; // * z_w;
//		printf("score = %d\n", score);

		closure_out->score = score;
		closure_out->first = first;
		closure_out->last = last;
	}
	
	return 0;
#endif

}


uint32_t n_closures = 0;
static closure_t closures[MAX_CLOSURES];


static int find_possible_closures(submap_meta_t* sms, int cur_sms) //, closure_t* out)
{
/*
			static tmp_cloud_t kak;
			load_tmp_cloud(&kak, cur_sms);

			float ref_variance = calc_cloud_ref_quality(&kak);

			printf("sm=%d, variance=%.4f\n", cur_sms, ref_variance);
			closures[n_closures].score = ref_variance*1000.0;
			closures[n_closures].first = cur_sms;
			closures[n_closures].last = cur_sms;
			closures[n_closures].match = (result_t){0,0,0,0,ref_variance*1000.0,ref_variance*1000.0};

			n_closures++;

			return 0;
*/

	if(n_closures >= MAX_CLOSURES)
	{
		printf("WARNING: Too many loop closures, stopping loop closure generation\n");
		return -1;
	}

	int cur_x = sms[cur_sms].avg_x;
	int cur_y = sms[cur_sms].avg_y;
	int cur_z = sms[cur_sms].avg_z;

	#define N_EARLIER_SUBMAPS 16

	int n_submaps = 0;
	// Matching results:
	result_t results[N_EARLIER_SUBMAPS][N_MATCH_RESULTS];
	// Closures built using the matching results:
//	closure_t closures[N_EARLIER_SUBMAPS][N_MATCH_RESULTS];

	int n_results[N_EARLIER_SUBMAPS];

	for(int i=cur_sms-MIN_CLOSURE_LEN; i>=0; i--)
	{
		// Distance of submap sequence numbers:
		int sm_dist = cur_sms - i;

		if(sm_dist >= MAX_CLOSURE_LEN-1)
		{
			break;
		}

		double xy_uncert = sm_dist * XY_ACCUM_UNCERT_BY_SUBMAP + 800.0;
		double z_uncert = sm_dist * Z_ACCUM_UNCERT_BY_SUBMAP +150.0;
		double yaw_uncert = sm_dist * YAW_ACCUM_UNCERT_BY_SUBMAP + DEGTORAD(5.5);

		if(yaw_uncert > YAW_ACCUM_UNCERT_SATURATION)
			yaw_uncert = YAW_ACCUM_UNCERT_SATURATION;


		int smi_x = sms[i].avg_x;
		int smi_y = sms[i].avg_y;
		int smi_z = sms[i].avg_z;

		// TRY_CLOSURE_MARGIN exists because when we visit a place later, it's very unlikely we
		// hit the exact same spot. It's enough that we see the same scene.

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
		int dx = smi_x - cur_x;
		int dy = smi_y - cur_y;
		int dz = smi_z - cur_z;

		printf("Test loop cur_submap %d (%d,%d,%d), earlier_submap %d (%d,%d,%d), xy_uncert=%.0f, z_uncert=%.0f, dx=%d, dy=%d, dz=%d  --> ",
			cur_sms, cur_x, cur_y, cur_z, i, smi_x, smi_y, smi_z, xy_uncert, z_uncert, dx, dy, dz);
		fflush(stdout);

		int n_sma = 1;
		int i_sma[4] = {i, 0,0,0};
		result_t sma_corrs[4];
		memset(sma_corrs, 0, sizeof(sma_corrs));

		int now_n_results = match_submaps(n_sma, i_sma, sma_corrs,
			cur_sms, xy_uncert, z_uncert, yaw_uncert, dx, dy, dz, results[n_submaps]);


		if(now_n_results == 0)
		{
			printf("0 results\n");
			continue;
		}

		printf("%d results, best score = %d -->", now_n_results, results[n_submaps][0].score);


		if(results[n_submaps][0].score > 6000)
		{
			printf("new closure %d\n", n_closures);
			closures[n_closures].score = results[n_submaps][0].score;
			closures[n_closures].first = i;
			closures[n_closures].last = cur_sms;
			closures[n_closures].match = results[n_submaps][0];

			n_closures++;
		}
		else
		{
			printf("no closure\n");
		}
	}

#if 0

		int outr = 0;
		for(int r=0; r<now_n_results; r++)
		{
			//printf("Building loop closure path for result %d/%d   (%+5d,%+5d,%+5d,%+6.2f)\n", r, now_n_results,
			//	results[n_submaps][r].x,results[n_submaps][r].y,results[n_submaps][r].z,RADTODEG(results[n_submaps][r].yaw));
			if(build_closure(sms, i, cur_sms, &results[n_submaps][r], &closures[n_submaps][outr]) != 0)
			{
				// Closure turned out clearly wrong, won't build.
			}
			else
			{
				outr++;
			}
		}

		if(outr == 0)
			continue;

		n_results[n_submaps] = outr;

		n_submaps++;
		if(n_submaps == N_EARLIER_SUBMAPS)
		{
			// TODO: Remove the limit, replace with min heap of N best results instead
			printf("Possible loop closure submap number exceeded\n");
			break;
		}

	}

	/*
	for(int sm=0; sm<n_submaps; sm++)
	{
		for(int c=0; c<n_results[sm]; c++)
		{
			printf("%3d vs. %3d, closure %2d, match score=%+6d, closure score=%+6d, match corr=(%+5d,%+5d,%+5d,%+6.2f)\n",
				cur_sms, closures[sm][c].first, c, results[sm][c].score, closures[sm][c].score,
				results[sm][c].x,results[sm][c].y,results[sm][c].z,RADTODEG(results[sm][c].yaw));
		}
	}
	*/

	if(n_submaps > 1)
	{
		// Try pairing submaps for a single loop closure:
		int32_t best_grouping_score = INT32_MIN;
		int32_t best_sm = 0;
		int32_t best_ca = 0;
		int32_t best_cb = 0;
		int found = 0;
		for(int sm=0; sm<n_submaps-1; sm++)
		{
			int sma = closures[sm][0].first;
			int smb = closures[sm+1][0].first;

			if(smb != sma-1)
				continue;

			for(int ca=0; ca<n_results[sm]; ca++)
			{
				for(int cb=0; cb<n_results[sm+1]; cb++)
				{
					int32_t match_dx = results[sm+1][cb].x - results[sm][ca].x;
					int32_t match_dy = results[sm+1][cb].y - results[sm][ca].y;
					int32_t match_dz = results[sm+1][cb].z - results[sm][ca].z;
					float   match_dyaw = results[sm+1][cb].yaw - results[sm][ca].yaw;

					double minerr = 999999999999.9;

					/*
					int32_t match_ddx = 0;
					int32_t match_ddy = 0;
					int32_t match_ddz = 0;
					float   match_ddyaw = 0.0;
					int best_adjres = -1;
					*/

					// Assume the most successful adjacent match:
					int n_adjres = adjacent_matches[sma].n_results;
					if(n_adjres > 8) n_adjres = 8;
					for(int adjres=0; adjres < n_adjres; adjres++)
					{
						// TODO: verify the direction of subtraction (works too well even with the wrong metric)
						int64_t ddx = -1*(match_dx - adjacent_matches[sma].results[adjres].x);
						int64_t ddy = -1*(match_dy - adjacent_matches[sma].results[adjres].y);
						int64_t ddz = -1*(match_dz - adjacent_matches[sma].results[adjres].z);
						float ddyaw = -1.0*(match_dyaw - adjacent_matches[sma].results[adjres].yaw);

						// 1 deg corresponds to 100 mm.
						double err = sqrt(sq(ddx) + sq(ddy) + sq(ddz)) + fabs(RADTODEG(ddyaw)*100.0);

						if(err < minerr)
						{
							minerr = err;
							/*
							best_adjres = adjres;
							match_ddx = ddx;
							match_ddy = ddy;
							match_ddz = ddz;
							match_ddyaw = ddyaw;
							*/
						}
					}

					// around 7000 is a typical "good" score for both matching and closure.

					int32_t score = 2*results[sm][ca].score/7 + closures[sm][ca].score/7 + 2*results[sm+1][cb].score/7 + closures[sm+1][cb].score/7 /* around 6000 */
						- minerr*3.0; // 2000 equiv-mm brings the otherwise typical good score to zero

/*
					printf("sm=%d,sma=%3d,smb=%3d,ca=%2d,cb=%2d, am=%+6d, ac=%+6d, bm=%+6d, bc=%+6d; (%+5d,%+5d,%+5d,%+6.2f)vs(%+5d,%+5d,%+5d,%+6.2f)=d(%+5d,%+5d,%+5d,%+6.2f), adjres=%d, err=%.0f (%+5d,%+5d,%+5d,%+6.2f) adjcorr (%+5d,%+5d,%+5d,%+6.2f) -> SCORE=%+6d\n",
						sm, sma, smb, ca, cb, results[sm][ca].score, closures[sm][ca].score, results[sm+1][cb].score, closures[sm+1][cb].score,
						results[sm][ca].x, results[sm][ca].y, results[sm][ca].z, RADTODEG(results[sm][ca].yaw),
						results[sm+1][cb].x, results[sm+1][cb].y, results[sm+1][cb].z, RADTODEG(results[sm+1][cb].yaw),
						match_dx, match_dy, match_dz, RADTODEG(match_dyaw),
						best_adjres, minerr,
						match_ddx, match_ddy, match_ddz, RADTODEG(match_ddyaw),
						adjacent_matches[sma].results[best_adjres].x, adjacent_matches[sma].results[best_adjres].y, adjacent_matches[sma].results[best_adjres].z, RADTODEG(adjacent_matches[sma].results[best_adjres].yaw),
						score);
*/
					if(score > best_grouping_score)
					{
						found = 1;
						best_grouping_score = score;
						best_sm = sm;
						best_ca = ca;
						best_cb = cb;
					}
										
				}
			}
		}

		if(!found)
			return -1;

		int best_sma = closures[best_sm][0].first;
		int best_smb = closures[best_sm+1][0].first;

		assert(best_smb == best_sma-1);
		assert(best_sma < cur_sms-3);

		if(best_grouping_score < 4000)
		{
			printf("Best closure is sm#%3d to sms #%3d&#%3d, with closure matchings #%2d and %2d. Score is only %d, not closing loop.\n", 
				cur_sms, best_sma, best_smb, best_ca, best_cb, best_grouping_score);

		}
		else
		{
			printf("Best closure is sm#%3d to sms #%3d&#%3d, with closure matchings #%2d and %2d. Score=%d, closing loop with paths:\n",
				cur_sms, best_sma, best_smb, best_ca, best_cb, best_grouping_score);

			out->first = best_smb;
			out->last = cur_sms;

			for(int i = 0; i < cur_sms-best_smb; i++)
			{
				int32_t avg_x;
				int32_t avg_y;
				int32_t avg_z;
				float avg_yaw;

				if(i == 0)
				{
					printf("  (     ,     ,     ,      )  ");

					avg_x = closures[best_sm+1][best_cb].corrs[i].x;
					avg_y = closures[best_sm+1][best_cb].corrs[i].y;
					avg_z = closures[best_sm+1][best_cb].corrs[i].z;
					avg_yaw = closures[best_sm+1][best_cb].corrs[i].yaw;

				}
				else
				{
					printf("  (%+5d,%+5d,%+5d,%+6.2f)  ",
						closures[best_sm][best_ca].corrs[i-1].x,
						closures[best_sm][best_ca].corrs[i-1].y,
						closures[best_sm][best_ca].corrs[i-1].z,
						RADTODEG(closures[best_sm][best_ca].corrs[i-1].yaw));

					avg_x = (closures[best_sm][best_ca].corrs[i-1].x + closures[best_sm+1][best_cb].corrs[i].x)/2;
					avg_y = (closures[best_sm][best_ca].corrs[i-1].y + closures[best_sm+1][best_cb].corrs[i].y)/2;
					avg_z = (closures[best_sm][best_ca].corrs[i-1].z + closures[best_sm+1][best_cb].corrs[i].z)/2;
					avg_yaw = (closures[best_sm][best_ca].corrs[i-1].yaw + closures[best_sm+1][best_cb].corrs[i].yaw)/2.0;

				}

				printf("  (%+5d,%+5d,%+5d,%+6.2f)  ",
					closures[best_sm+1][best_cb].corrs[i].x,
					closures[best_sm+1][best_cb].corrs[i].y,
					closures[best_sm+1][best_cb].corrs[i].z,
					RADTODEG(closures[best_sm+1][best_cb].corrs[i].yaw));

				printf("  (%+5d,%+5d,%+5d,%+6.2f)  ",
					avg_x,
					avg_y,
					avg_z,
					RADTODEG(avg_yaw));

				out->corrs[i] = (result_t){avg_x, avg_y, avg_z, avg_yaw, 0, 0};
				out->match = results[best_sm+1][best_cb];

				printf("\n");
			}

			return 0;
		}
	}
#endif
	return -1;

}

void tmp_cloud_to_xyz_file(tmp_cloud_t* cloud, char* fname)
{
	FILE* f = fopen(fname, "w");
	assert(f);
	for(int i=0; i<cloud->n_points; i++)
	{
		fprintf(f, "%d %d %d\n", cloud->points[i].px, cloud->points[i].py, cloud->points[i].pz);
	}
	fclose(f);
}

int main(int argc, char** argv)
{

	if(argc < 2)
	{
		printf("Usage: slam 1234 [submap_end_idx]\n");
		printf("Params required:\n");
		printf("1 = create submap_metas & pointclouds\n");
		printf("2 = create adjacent matches\n");
		printf("3 = create loop closures\n");
		printf("4 = visualize closures\n");
		printf("5 = combine loop closures\n");
		printf("6 = create output\n");
		return -1;

	}

	int limit_submaps = -1;

	int create_pointclouds = 0;
	int adjacent_match = 0;
	int do_closures = 0;
	int create_output = 0;
	int visualize_closures = 0;
	int combine_closures = 0;

	{
		char c;
		int i = 0;
		while( (c = argv[1][i++]) != 0)
		{
			if(c == '1')
				create_pointclouds = 1;

			if(c == '2')
				adjacent_match = 1;

			if(c == '3')
				do_closures = 1;

			if(c == '4')
				visualize_closures = 1;

			if(c == '5')
				combine_closures = 1;

			if(c == '6')
				create_output = 1;


		}
	}

	if(argc >= 3)
	{
		sscanf(argv[2], "%d", &limit_submaps);
		if(limit_submaps < 1 || limit_submaps > MAX_SUBMAPS)
		{
			printf("ERROR: invalid [submap_end_idx] parameter\n");
			abort();
		}
	}

	printf("Super Slammings 2.0 2000\n");
	printf("pos_gyro_corr = %lf, neg_gyro_corr = %lf\n", pos_gyro_corr, neg_gyro_corr);

	uint32_t n_submaps;
	static submap_meta_t submap_metas[MAX_SUBMAPS];

	static tmp_cloud_t tmp_cloud;

	if(create_pointclouds)
	{
		init_corr_points();

		static firstsidx_pose_t firstsidx_poses[MAX_FIRSTSIDX_POSES];
		static firstsidx_pose_t pose_corrs[MAX_FIRSTSIDX_POSES];

		int n_firstsidx_poses;

		extract_firstsidx_poses(firstsidx_poses, &n_firstsidx_poses);

		static fpose_t fposes[MAX_FIRSTSIDX_POSES];
		gen_fposes(firstsidx_poses, n_firstsidx_poses, fposes);


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

		
		group_submaps(submap_metas, &n_submaps, firstsidx_poses, n_firstsidx_poses, pose_corrs);

		printf("\n\n      GROUPED DATA IN %d SUBMAPS\n\n", n_submaps);

		if(limit_submaps > 0 && limit_submaps < n_submaps)
		{
			printf("NOTE: limiting number of submaps from %d to %d, requested by user\n", n_submaps, limit_submaps);
			n_submaps = limit_submaps;
		}

		save_submap_metas(submap_metas, n_submaps);

		// Create and filter pointclouds
		int64_t total_points = 0;
		int64_t total_after_filtering = 0;

		for(int sm=0; sm<n_submaps; sm++)
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

					// Voxfilter requires a source index running starting from 1.
					int voxflt_src_idx = idx - submap_metas[sm].subsubmaps[ssm].start_idx + 1;

					char fname[1024];
					sprintf(fname, "/home/hrst/robotsoft/tsellari/trace%08d.rb2", idx);
					tof_slam_set_t* tss;
					if(process_file(fname, &tss) == 0) // tof_slam_set record succesfully extracted
					{
						hw_pose_t pose0 = add_poses(tss->sets[0].pose, pose_corrs[cidx].pose);
						hw_pose_t pose1 = add_poses(tss->sets[1].pose, pose_corrs[cidx].pose);
						tof_to_voxfilter_and_cloud(0, 
							tss->sets[0].ampldist, pose0,
							voxflt_src_idx, tss->sidx, 
							submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z,
							&tmp_voxfilter, &tmp_cloud, 1800, 300);

						if(tss->flags & TOF_SLAM_SET_FLAG_SET1_NARROW)
							tof_to_voxfilter_and_cloud(1, 
								tss->sets[1].ampldist, pose1,
								voxflt_src_idx, tss->sidx, 
								submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z,
								NULL, &tmp_cloud, 1800, 3000);

						if(tss->flags & TOF_SLAM_SET_FLAG_SET1_WIDE)
							tof_to_voxfilter_and_cloud(0, 
								tss->sets[1].ampldist, pose1,
								voxflt_src_idx, tss->sidx, 
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

	}
	else
	{
		load_submap_metas(submap_metas, &n_submaps);

		printf("\n\n      LOADED SUBMAP GROUPING OF %d SUBMAPS\n\n", n_submaps);


		if(limit_submaps > 0 && limit_submaps < n_submaps)
		{
			printf("NOTE: limiting number of submaps from %d to %d, requested by user\n", n_submaps, limit_submaps);
			n_submaps = limit_submaps;
		}


	}

	#if 0
		for(int i=0; i<10; i++)
		{
			static tmp_cloud_t cloud;
			load_tmp_cloud(&cloud, i);
			char name[1000];
			sprintf(name, "submap%05d_x%d_y%d_z%d.xyz", i, submap_metas[i].avg_x, submap_metas[i].avg_y, submap_metas[i].avg_z);
			tmp_cloud_to_xyz_file(&cloud, name);
		}
		return;
	#endif

// idea:
// 01 12 23 34 45 56 67
//    02 13 24 35 46 57 

	// Match adjacent submaps
	if(adjacent_match)
	{
		int32_t running_x = 0, running_y = 0, running_z = 0;
		double running_yaw = 0.0;

		#if 0
		//if(create_output)
		{

			{
				int sm = 0;
				load_tmp_cloud(&tmp_cloud, sm);
				po_coords_t po;
				po = po_coords(submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z, 0);
				load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
				cloud_to_voxmap(&tmp_cloud, submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z);

				store_all_pages();
			}
		}
		#endif


		for(int sm=0; sm<n_submaps-1; sm++)
		{

/*			printf("Submap %3d: idx %8d .. %8d  (len %4d)", sm, submap_metas[sm].start_idx, submap_metas[sm].end_idx, submap_metas[sm].end_idx-submap_metas[sm].start_idx);
			if(sm < n_submaps-1)
			{
				printf(" (overlaps the next by %4d)  ", -1*(submap_metas[sm+1].start_idx - submap_metas[sm].end_idx));
			}
			else
				printf("                              ");

			printf("  avg (%+6d %+6d %+6d) ", submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z);
			printf("  next (%+6d %+6d %+6d)\n", submap_metas[sm+1].avg_x, submap_metas[sm+1].avg_y, submap_metas[sm+1].avg_z);
*/

			int dx = submap_metas[sm].avg_x - submap_metas[sm+1].avg_x;
			int dy = submap_metas[sm].avg_y - submap_metas[sm+1].avg_y;
			int dz = submap_metas[sm].avg_z - submap_metas[sm+1].avg_z;

			result_t results[N_FINE_MATCH_RESULTS];

			int n = fine_match_submaps(sm, sm+1, dx, dy, dz, results);

			assert(n > 0);
			assert(n <= N_FINE_MATCH_RESULTS);

			memcpy(adjacent_matches[sm].results, results, sizeof(result_t)*n);
			adjacent_matches[sm].n_results = n;

			int32_t x_corr = -1*results[0].x;
			int32_t y_corr = -1*results[0].y;
			int32_t z_corr = -1*results[0].z;
			double yaw_corr = 1.0*results[0].yaw;

			running_x += x_corr;
			running_y += y_corr;
//			running_z += z_corr;
			running_yaw += yaw_corr;

			running_x += (-1*dx) * cos(running_yaw) - (-1*dy) * sin(running_yaw)  - (-1*dx);
			running_y += (-1*dx) * sin(running_yaw) + (-1*dy) * cos(running_yaw)  - (-1*dy);

			int32_t new_avg_x = submap_metas[sm+1].avg_x + running_x;
			int32_t new_avg_y = submap_metas[sm+1].avg_y + running_y;
			int32_t new_avg_z = submap_metas[sm+1].avg_z + running_z;

			printf("%4d vs %4d Winner (%+5d, %+5d, %+5d, %+6.2f), running (%+7d, %+7d, %+7d, %+8.2f), new_avg %d, %d, %d\n",
				sm, sm+1, x_corr, y_corr, z_corr, RADTODEG(yaw_corr), running_x, running_y, running_z, RADTODEG(running_yaw), new_avg_x, new_avg_y, new_avg_z);

			//if(create_output)
			#if 0
			{
				load_tmp_cloud(&tmp_cloud, sm+1);
				rotate_cloud(&tmp_cloud, running_yaw);
				po_coords_t po;
				po = po_coords(new_avg_x, new_avg_y, new_avg_z, 0);
				load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
				cloud_to_voxmap(&tmp_cloud, new_avg_x, new_avg_y, new_avg_z);

				store_all_pages();
			}
			#endif

		}

		save_adjacent_matches(n_submaps);
	}
	else if(do_closures || create_output)
	{
		load_adjacent_matches(n_submaps);

		printf("\n\n        LOADED ADJACENT MATCHES\n\n");
	}

	// Test code for testing the joining of three adjacent submaps into ref_matchmap:
	#if 0

	for(int idx=1; idx<20; idx++)
	{
		printf("Line %d, sm%d:  ", idx, idx);

		static tmp_cloud_t ca, cb, cc;
		load_tmp_cloud(&ca, idx-1);
		load_tmp_cloud(&ca, idx);
		load_tmp_cloud(&ca, idx+1);

		result_t adjab = adjacent_matches[idx-1].results[0];
		result_t adjbc = adjacent_matches[idx].results[0];

		result_t corrab = {
			submap_metas[idx].avg_x-submap_metas[idx-1].avg_x+adjab.x,
			submap_metas[idx].avg_y-submap_metas[idx-1].avg_y+adjab.y,
			submap_metas[idx].avg_z-submap_metas[idx-1].avg_z+adjab.z,
			adjab.yaw};

		result_t corrbc = {
			submap_metas[idx+1].avg_x-submap_metas[idx].avg_x+adjbc.x,
			submap_metas[idx+1].avg_y-submap_metas[idx].avg_y+adjbc.y,
			submap_metas[idx+1].avg_z-submap_metas[idx].avg_z+adjbc.z,
			adjbc.yaw};

		gen_save_ref_matchmap_set(&ca, &cb, &cc, corrab, corrbc);

	}
	return 0;
	#endif

	#if 1

//		int clouds[8] = {0,4,5, 11,138,146, 148, 92};
//		int clouds[10] = {91, 92,93,74,76,78, 100, 102, 104, 106};
		for(int i=0; i<173; i++)
		{
			int idx = i; //clouds[i];

			printf("Line %d, sm%d:  ", i, idx);

			test_q(idx);
/*
			static tmp_cloud_t tmp_cloud;
			load_tmp_cloud(&tmp_cloud, idx);

			po_coords_t po;
			po = po_coords(0,i*-15000,0,0);
			load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
			cloud_to_voxmap(&tmp_cloud, 0,i*-15000,0);
*/
//			printf("\n");
		}
		free_all_pages();

		return;


	#endif



//	free_all_pages();



	if(do_closures)
	{
		// Loop closures

//		for(int sm=100; sm<139; sm++)
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

			find_possible_closures(submap_metas, sm);
//			if(find_possible_closures(submap_metas, sm, &closures[n_closures]) == 0)
//			{
//			}
		}

		printf("\n\n        FOUND %d LOOP CLOSURES\n\n", n_closures);

		save_closures(closures, n_closures);
	}
	else if(create_output || visualize_closures || combine_closures)
	{
		if(load_closures(closures, &n_closures) == 0)
			printf("\n\n        LOADED %d LOOP CLOSURES\n\n", n_closures);
		else
			printf("\n\n        LOOP CLOSURE FILE NOT FOUND\n\n");

	}


	if(create_output)
	{
		// Generate output

		int32_t running_x = 0, running_y = 0, running_z = 0;
		double running_yaw = 0.0;

		{
			int sm = 0;
			load_tmp_cloud(&tmp_cloud, sm);
			po_coords_t po;
			po = po_coords(submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z, 0);
			load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
			cloud_to_voxmap(&tmp_cloud, submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z);

			store_all_pages();
		}
		for(int sm=0; sm<n_submaps-1; sm++)
		{

/*			printf("Submap %3d: idx %8d .. %8d  (len %4d)", sm, submap_metas[sm].start_idx, submap_metas[sm].end_idx, submap_metas[sm].end_idx-submap_metas[sm].start_idx);
			if(sm < n_submaps-1)
			{
				printf(" (overlaps the next by %4d)  ", -1*(submap_metas[sm+1].start_idx - submap_metas[sm].end_idx));
			}
			else
				printf("                              ");

			printf("  avg (%+6d %+6d %+6d) ", submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z);
			printf("  next (%+6d %+6d %+6d)\n", submap_metas[sm+1].avg_x, submap_metas[sm+1].avg_y, submap_metas[sm+1].avg_z);
*/

			int dx = submap_metas[sm].avg_x - submap_metas[sm+1].avg_x;
			int dy = submap_metas[sm].avg_y - submap_metas[sm+1].avg_y;
			//int dz = submap_metas[sm].avg_z - submap_metas[sm+1].avg_z;

			// Use the best adjacent match by default:
			result_t result = adjacent_matches[sm].results[0];

			for(int c=0; c<n_closures; c++)
			{
				if(sm >= closures[c].first && sm <= closures[c].last)
				{
					int seq = sm - closures[c].first;
					printf("submap #%3d in loop closure #%3d as %dth element, overriding adjacent default correction\n", sm, c, seq);
					result = closures[c].corrs[seq];
					break;
				}
			}

			int32_t x_corr = -1*result.x;
			int32_t y_corr = -1*result.y;
			int32_t z_corr = -1*result.z;
			double yaw_corr = 1.0*result.yaw;

			running_x += x_corr;
			running_y += y_corr;
			//running_z += z_corr;
			running_yaw += yaw_corr;

			running_x += (-1*dx) * cos(running_yaw) - (-1*dy) * sin(running_yaw)  - (-1*dx);
			running_y += (-1*dx) * sin(running_yaw) + (-1*dy) * cos(running_yaw)  - (-1*dy);

			int32_t new_avg_x = submap_metas[sm+1].avg_x + running_x;
			int32_t new_avg_y = submap_metas[sm+1].avg_y + running_y;
			int32_t new_avg_z = submap_metas[sm+1].avg_z + running_z;

			printf("%4d vs %4d Winner (%+5d, %+5d, %+5d, %+6.2f), running (%+7d, %+7d, %+7d, %+8.2f), new_avg %d, %d, %d\n",
				sm, sm+1, x_corr, y_corr, z_corr, RADTODEG(yaw_corr), running_x, running_y, running_z, RADTODEG(running_yaw), new_avg_x, new_avg_y, new_avg_z);

			#if 1
				load_tmp_cloud(&tmp_cloud, sm+1);
				rotate_cloud(&tmp_cloud, running_yaw);
				po_coords_t po;
				po = po_coords(new_avg_x, new_avg_y, new_avg_z, 0);
				load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
				cloud_to_voxmap(&tmp_cloud, new_avg_x, new_avg_y, new_avg_z);

				store_all_pages();
			#endif

		}
	}

	if(combine_closures)
	{
		for(int c=0; c<n_closures; c++)
		{
			printf("closure #%3d (%3d..%3d)\n", c, closures[c].first, closures[c].last);
		}
/*
		int16_t n_overlaps[MAX_CLOSURES] = {0};
		int16_t is_outside[MAX_CLOSURES] = {0};
		int16_t list_overlaps[MAX_CLOSURES][MAX_CLOSURES] = {0};

		for(int ca=0; ca<n_closures; ca++)
		{
			for(int cb=0; cb<n_closures; cb++)
			{
				if(ca == cb)
					continue;
				if((closures[cb].first > closures[ca].first && closures[cb].first < closures[ca].last) ||
				   (closures[cb].last > closures[ca].first && closures[cb].last < closures[ca].last))

				{
					printf("#%d (%d..%d) is inside #%d (%d..%d)\n",
						cb, closures[cb].first, closures[cb].last, ca, closures[ca].first, closures[ca].last);
					list_overlaps[ca][n_overlaps[ca]] = cb;
					n_overlaps[ca]++;
				}
				else
				{
					printf("#%d (%d..%d) is outside #%d (%d..%d)\n",
						cb, closures[cb].first, closures[cb].last, ca, closures[ca].first, closures[ca].last);
					is_outside[cb]++;
				}
			}
		}

		for(int ca=0; ca<n_closures; ca++)
		{
			printf("closure #%3d (%3d..%3d): %d: %2d overlaps: ", ca, closures[ca].first, closures[ca].last, 
				is_outside[ca], n_overlaps[ca]);
			for(int i=0; i<n_overlaps[ca]; i++)
			{
				int cb = list_overlaps[ca][i];
				printf("#%3d (%3d..%3d); ", cb, closures[cb].first, closures[cb].last);
			}
			printf("\n");
		}
*/
	}

	if(visualize_closures)
	{
		#define STEPX 15000
		#define STEPY 20000
		int32_t running_x = 0, running_y = 0;

		printf("VISUALIZING CLOSURES\n\n");

		int col = 0;
		//int c = 0;
		for(int c=0; c<n_closures; c++)
		{

			int sma = closures[c].first;
			int smb = closures[c].last;
//			int smc = closures[c].first + 1;

			printf("y=%+6d   %3d  %3d  (%+6d,%+6d,%+6d,%+6.2f) SCORE=%d ABSCORE=%d\n", running_y, sma, smb, 
				closures[c].match.x, closures[c].match.y, closures[c].match.z, RADTODEG(closures[c].match.yaw), closures[c].match.score, closures[c].match.abscore);

//			printf("  %3d    %3d    %3d&%3d    %3d&%3d&%3d \n", sma, smb, sma,smb,  sma,smc,smb);


			int ax = submap_metas[sma].avg_x;
			int ay = submap_metas[sma].avg_y;
			int az = submap_metas[sma].avg_z;

			int bx = submap_metas[smb].avg_x;
			int by = submap_metas[smb].avg_y;
			int bz = submap_metas[smb].avg_z;

/*
			int cx = submap_metas[smc].avg_x;
			int cy = submap_metas[smc].avg_y;
			int cz = submap_metas[smc].avg_z;
*/
			result_t result = closures[c].match;

			int32_t x_corr = 1*result.x;
			int32_t y_corr = 1*result.y;
			int32_t z_corr = 1*result.z;
			double yaw_corr = 1.0*result.yaw;
/*
			result_t resultc = closures[c].corrs[0];

			int32_t cx_corr = -1*resultc.x;
			int32_t cy_corr = -1*resultc.y;
			int32_t cz_corr = -1*resultc.z;
			double cyaw_corr = 1.0*resultc.yaw;
*/

			// SMA alone
			{
				load_tmp_cloud(&tmp_cloud, sma);
				//rotate_cloud(&tmp_cloud, 0.0);
				po_coords_t po;
				po = po_coords(0+running_x,0+running_y,0, 0);
				load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
				cloud_to_voxmap(&tmp_cloud, 0+running_x,0+running_y,0);
			}

			// SMB alone
			{
				load_tmp_cloud(&tmp_cloud, smb);
				rotate_cloud(&tmp_cloud, yaw_corr);
				po_coords_t po;
				po = po_coords(STEPX+x_corr+bx-ax+running_x,y_corr+by-ay+running_y,z_corr+bz-az, 0);
				load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
				cloud_to_voxmap(&tmp_cloud, STEPX+x_corr+bx-ax+running_x,y_corr+by-ay+running_y,z_corr+bz-az);
			}

			// SMA, SMB matched together
			{
				load_tmp_cloud(&tmp_cloud, sma);
				//rotate_cloud(&tmp_cloud, 0.0);
				po_coords_t po;
				po = po_coords(2*STEPX+0+running_x,0+running_y,0, 0);
				load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
				cloud_to_voxmap(&tmp_cloud, 2*STEPX+0+running_x,0+running_y,0);
			}


			{
				load_tmp_cloud(&tmp_cloud, smb);
				rotate_cloud(&tmp_cloud, yaw_corr);
				po_coords_t po;
				po = po_coords(2*STEPX+x_corr+bx-ax+running_x,y_corr+by-ay+running_y,z_corr+bz-az, 0);
				load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
				cloud_to_voxmap(&tmp_cloud, 2*STEPX+x_corr+bx-ax+running_x,y_corr+by-ay+running_y,z_corr+bz-az);
			}

/*
			// SMA, SMC, SMB matched together
			{
				load_tmp_cloud(&tmp_cloud, sma);
				//rotate_cloud(&tmp_cloud, 0.0);
				po_coords_t po;
				po = po_coords(3*STEPX+0+running_x,0+running_y,0, 0);
				load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
				cloud_to_voxmap(&tmp_cloud, 3*STEPX+0+running_x,0+running_y,0);
			}

			{
				load_tmp_cloud(&tmp_cloud, smc);
				rotate_cloud(&tmp_cloud, cyaw_corr);
				po_coords_t po;
				po = po_coords(3*STEPX+cx_corr+cx-ax+running_x,cy_corr+cy-ay+running_y,cz_corr+cz-az, 0);
				load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
				cloud_to_voxmap(&tmp_cloud, 3*STEPX+cx_corr+cx-ax+running_x,cy_corr+cy-ay+running_y,+cz_corr+cz-az);
			}

			{
				load_tmp_cloud(&tmp_cloud, smb);
				rotate_cloud(&tmp_cloud, yaw_corr);
				po_coords_t po;
				po = po_coords(3*STEPX+x_corr+bx-ax+running_x,y_corr+by-ay+running_y,z_corr+bz-az, 0);
				load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
				cloud_to_voxmap(&tmp_cloud, 3*STEPX+x_corr+bx-ax+running_x,y_corr+by-ay+running_y,z_corr+bz-az);
			}
*/
			store_all_pages();

			running_y -= STEPY;

		}
		printf("\n");
	}


	free_all_pages();

	return 0;
} 
