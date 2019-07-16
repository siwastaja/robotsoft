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

#ifdef SLAM_STANDALONE
	#define DEFINE_API_VARIABLES
#endif
#include "api_board_to_soft.h"
#undef DEFINE_API_VARIABLES

#include "voxmap.h"
#include "voxmap_memdisk.h"


#ifdef SLAM_STANDALONE
	#include "b2s_prints.c"
#endif

#include "slam_config.h"


#include "slam_cloud.h"
#include "slam_matchers.h"

#include "misc.h"




/*
#define RADTODEG(x) ((x)*(360.0/(2.0*M_PI)))
#define DEGTORAD(x) ((x)*((2.0*M_PI)/360.0))
#define ANG32TORAD(x) ( ((double)((uint32_t)(x)))/683565275.576432)
#define ANG32TOFDEG(x) ( ((double)((uint32_t)(x)))/11930464.7111111)
*/


#define MAX_SUBMAPS 4096


/*typedef struct __attribute__((packed))
{
	int32_t start_idx;
	int32_t end_idx;
	int32_t avg_x;
	int32_t avg_y;
	int32_t avg_z;
} subsubmap_meta_t;
*/
//#define MAX_SUBSUBMAPS 16

// Maximum number of subsubmaps in a submap. After exceeded, a new submap is force-started
// Number of maximum scans is SUBSUBMAP_LIMIT * VOXFILTER_N_SCANS
#define SUBSUBMAP_LIMIT 10
#define SUBMAP_MAX_SCANS (SUBSUBMAP_LIMIT * VOXFILTER_N_SCANS)

// Maximum difference in robot coordinates during a submap, basically the length of the robot pose range before terminating the submap. In mm
#define DX_LIMIT 4000
#define DY_LIMIT 4000
#define DZ_LIMIT 2000

// Cumulative linear travel limit; after exceeded, a new submap is started. In mm.
// Difference to the D*_LIMITs is that this limit can force a new submap even when the robot is
// just moving (back and forth, for example) inside a small area.
#define CUMUL_TRAVEL_LIMIT 6000
// Similar, but for cumulative angular (yaw) motion
#define CUMUL_YAW_LIMIT DEGTORAD(200)


typedef struct  __attribute__((packed))
{
	// Sensor data sequence IDs. start_idx is the first, end_idx is the last, both inclusive.
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

	hw_pose_t trajectory[SUBMAP_MAX_SCANS];

//	int32_t n_subsubmaps;
//	subsubmap_meta_t subsubmaps[MAX_SUBSUBMAPS];

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



int process_file(char* fname, tof_slam_set_t** tss);

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

// Pitch and roll are assumed to have no accumulating error (based on gravity vector always available)
// Accumulating yaw error is eventually limited by compass, but compass will be inaccurate and subject
// to local magnetic fields.

#define XY_ACCUM_UNCERT_BY_SUBMAP 400 // mm
#define Z_ACCUM_UNCERT_BY_SUBMAP 0   // mm
#define YAW_ACCUM_UNCERT_BY_SUBMAP (DEGTORAD(1.5))
#define YAW_ACCUM_UNCERT_SATURATION (DEGTORAD(110))

#define TRY_CLOSURE_MARGIN_XY 6000
#define TRY_CLOSURE_MARGIN_Z  1000




typedef struct __attribute__((packed))
{
	int32_t n_results;
	result_t results[N_FINE_MATCH_RESULTS];
} adjacent_t;


adjacent_t adjacent_matches[MAX_SUBMAPS];

ref_quality_t ref_qs[MAX_SUBMAPS];

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


#define MIN_CLOSURE_LEN 4
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

typedef struct __attribute__((packed))
{
	// Submap indeces, [first] and [last] match together
	int32_t first;
	int32_t last;

	int n_possibilities;
	result_t possibilities[N_MATCH_RESULTS];
} closure_set_t;

uint32_t n_closures = 0;

closure_set_t closures[MAX_CLOSURES];

void print_closure_set(closure_set_t* cs, int n)
{
	for(int i=0; i<n; i++)
	{
		printf("C%03d ", i);

		printf("%d pos: ", cs[i].n_possibilities);
		for(int p=0; p < 3; p++)
		{
			if(p < cs[i].n_possibilities)
				printf("%+6d ", cs[i].possibilities[p].score);
			else
				printf("       ");
		}

		putchar('|');

		for(int a=0; a<cs[i].first; a++)
			putchar(' ');
		printf(" %03d O", cs[i].first);
		for(int a=0; a<cs[i].last-cs[i].first-1; a++)
			putchar('=');
		printf("O %03d ", cs[i].last);

		printf("\n");
	}


}

int save_closures(closure_set_t* closures, uint32_t n_closures)
{
	char fname[1024];
	snprintf(fname, 1024, "closures.bin");
	FILE* f = fopen(fname, "wb");
	assert(f);

	assert(fwrite(&n_closures, sizeof(uint32_t), 1, f) == 1);
	assert(fwrite(closures, sizeof(closure_set_t), n_closures, f) == n_closures);

	fclose(f);
	return 0;
}

int load_closures(closure_set_t* closures, uint32_t* n_closures)
{
	char fname[1024];
	snprintf(fname, 1024, "closures.bin");
	FILE* f = fopen(fname, "rb");
	if(!f)
		return -1;

	uint32_t n_closures_file = 0;
	assert(fread(&n_closures_file, sizeof(uint32_t), 1, f) == 1);

	assert(n_closures_file < MAX_CLOSURES);

	assert(fread(closures, sizeof(closure_set_t), n_closures_file, f) == n_closures_file);

	*n_closures = n_closures_file;

	fclose(f);
	return 0;
}


static int find_possible_closures(submap_meta_t* sms, int cur_sms, cloud_t* cur_cloud)
{
	if(n_closures >= MAX_CLOSURES)
	{
		printf("WARNING: Too many loop closures, stopping loop closure generation\n");
		return -1;
	}

	int cur_x = sms[cur_sms].avg_x;
	int cur_y = sms[cur_sms].avg_y;
	int cur_z = sms[cur_sms].avg_z;

//	#define N_EARLIER_SUBMAPS 16

//	int n_submaps = 0;
	// Matching results:
//	result_t results[N_EARLIER_SUBMAPS][N_MATCH_RESULTS];
	// Closures built using the matching results:
//	closure_t closures[N_EARLIER_SUBMAPS][N_MATCH_RESULTS];

	if(ref_qs[cur_sms].quality < 4000)
	{
		printf("poor cmp quality (%d), won't match\n", ref_qs[cur_sms].quality);
		return 0;
	}

	for(int i=cur_sms-MIN_CLOSURE_LEN; i>=1; i--)
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

		//printf("%d vs %d\n", cur_sms, i);
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

		printf("Test loop cur_sm%d (%d,%d,%d), with sm%d (%d,%d,%d), uncert xy=%.0f, z=%.0f, yaw=%.1f dx=%d, dy=%d, dz=%d  --> ",
			cur_sms, cur_x, cur_y, cur_z, i, smi_x, smi_y, smi_z, xy_uncert, z_uncert, RADTODEG(yaw_uncert), dx, dy, dz);
		fflush(stdout);


		if(ref_qs[i].quality < 4000)
		{
			printf("poor ref quality (%d), won't match\n", ref_qs[i].quality);
			continue;
		}

		result_t result[N_MATCH_RESULTS];
		int now_n_results = match_submaps(i, cur_cloud,
			xy_uncert, z_uncert, yaw_uncert, dx, dy, dz, result);


		if(now_n_results == 0)
		{
			printf("0 results\n");
			continue;
		}

		printf("%d results, best score = %d -->", now_n_results, result[0].score);


		for(int res=0; 
			res < now_n_results &&
			result[res].score > 4000 &&
			result[res].abscore > 6000 &&
			(int64_t)ref_qs[i].quality * (int64_t)ref_qs[cur_sms].quality * (int64_t)result[res].score > 5000LL*5000LL*5000LL;
			res++)
		{
			closures[n_closures].first = i;
			closures[n_closures].last = cur_sms;

			assert(closures[n_closures].n_possibilities < N_MATCH_RESULTS);

			closures[n_closures].possibilities[closures[n_closures].n_possibilities] = result[res];
			closures[n_closures].n_possibilities++;
		}

		if(closures[n_closures].n_possibilities > 0)
		{
			printf("new closure %d (%d possibilities (from %d) added)\n", n_closures, closures[n_closures].n_possibilities, now_n_results);
			n_closures++;

			print_closure_set(closures, n_closures);
			save_closures(closures, n_closures);

		}
		else
		{
			printf("no good closures\n");
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


static int n_sms;
static submap_meta_t submap_metas[MAX_SUBMAPS];

void match_to_prev_submap(int cur_smi, cloud_t* prev_cloud, cloud_t* cur_cloud)
{
	assert(cur_smi > 0);

	int dx = submap_metas[cur_smi].avg_x - submap_metas[cur_smi-1].avg_x;
	int dy = submap_metas[cur_smi].avg_y - submap_metas[cur_smi-1].avg_y;
	int dz = submap_metas[cur_smi].avg_z - submap_metas[cur_smi-1].avg_z;

	result_t results[N_FINE_MATCH_RESULTS];

	int n = fine_match_submaps(prev_cloud, cur_cloud, -dx, -dy, -dz, results);

	assert(n <= N_FINE_MATCH_RESULTS);

	memcpy(adjacent_matches[cur_smi-1].results, results, sizeof(result_t)*n);
	adjacent_matches[cur_smi-1].n_results = n;

	#if 0
		// Code to build output solely on adjacent matches

		static int32_t running_x = 0, running_y = 0, running_z = 0;
		static double running_yaw = 0.0;

		int32_t x_corr=0, y_corr=0, z_corr=0;
		double yaw_corr=0.0;
		if(n > 0)
		{
			x_corr = -1*results[0].x;
			y_corr = -1*results[0].y;
			z_corr = -1*results[0].z;
			yaw_corr = 1.0*results[0].yaw;
		}

		running_x += x_corr;
		running_y += y_corr;
		//running_z += z_corr;
		running_yaw += yaw_corr;


		running_x += dx * cos(running_yaw) - dy * sin(running_yaw)  - dx;
		running_y += dx * sin(running_yaw) + dy * cos(running_yaw)  - dy;

		int32_t new_avg_x = submap_metas[cur_smi].avg_x + running_x;
		int32_t new_avg_y = submap_metas[cur_smi].avg_y + running_y;
		int32_t new_avg_z = submap_metas[cur_smi].avg_z + running_z;

		printf("%4d vs %4d Winner (%+5d, %+5d, %+5d, %+6.2f), running (%+7d, %+7d, %+7d, %+8.2f), new_avg %d, %d, %d\n",
			cur_smi-1, cur_smi, x_corr, y_corr, z_corr, RADTODEG(yaw_corr), running_x, running_y, running_z, RADTODEG(running_yaw), new_avg_x, new_avg_y, new_avg_z);

		{
			static cloud_t rotated;
			rotate_cloud_copy(cur_cloud, &rotated, running_yaw);
			po_coords_t po;
			po = po_coords(new_avg_x, new_avg_y, new_avg_z, 0);
			load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
			cloud_to_voxmap(&rotated, new_avg_x, new_avg_y, new_avg_z);

			store_all_pages();
		}
	#endif

}


static cloud_t filtered_clouds[3];
// Pointers initialized in the beginning of main
static cloud_t* p_cur_cloud;
static cloud_t* p_prev_cloud;
static cloud_t* p_prev_prev_cloud;

/*
	tof_slam_set is the low-level (generated by the RobotBoard2 firmware)
	dataset which contains a distance measurement flash from one sensor (9600 distance measurements), and 
	optionally a secondary flash from the same sensor (either a longer-range wide data, or narrow beam data).
	Both include a corresponding hw_pose estimate.

	input_tof_slam_set processes one such flash, adds it to the current submap, and manages
	the submaps, starting a new one and terminating the current one whenever necessary.

	Returns -1 if the submap is still not finished
	If submap was finished by the function, returns the index of the created submap. n_sms is now set as this index + 1.
*/
int input_tof_slam_set(tof_slam_set_t* tss)
{
	int ret = -1;
	// Cloud and voxfilter data are accumulated, until a full submap is finished
	// Cloud is then copied (for "free") by the cloud filtering function

	// .bss, zeroed by C standard - is manually zeroed after each full submap
	static voxfilter_t current_voxfilter; 
	static cloud_t current_cloud; 


	// Accumulation variables for each submap
	static int32_t min_x = INT32_MAX, min_y = INT32_MAX, min_z = INT32_MAX;
	static int32_t max_x = INT32_MIN, max_y = INT32_MIN, max_z = INT32_MIN;
	static int64_t avg_x = 0, avg_y = 0, avg_z = 0;
	static double cumul_travel = 0;
	static double cumul_yaw = 0.0;
	static int n_scans = 0; // Number of accumulated sensor scan rounds

	static int32_t prev_x = INT32_MIN, prev_y = INT32_MIN, prev_z = INT32_MIN; // INT32_MIN signifies invalid value
	static double prev_yaw = 0.0;

	// Accumulation variables for each subsubmap
	static int64_t ss_avg_x = 0, ss_avg_y = 0, ss_avg_z = 0;
	static int ss_n_scans = 0;

	// Subsubmap (voxfilter accumulation) needs to have a reference point -
	// middle of the subsubmap would be optimal, but can't know that beforehand.
	// Use the robot pose at the start of collecting data to the voxfilter
	// When ref_x is set as INT32_MIN, a new initialization is made.

	static int32_t sm_ref_x = INT32_MIN, sm_ref_y, sm_ref_z;

	// Subsubmap reference position compared to the submap.
	// First subsubmap in the submap has 0,0,0.
	// When ref_x is set as INT32_MIN, a new initialization is made.
	static int32_t ssm_ref_x = INT32_MIN, ssm_ref_y, ssm_ref_z;

	static int n_subsubmaps = 0;

	if(sm_ref_x == INT32_MIN)
	{
		sm_ref_x = tss->sets[0].pose.x;
		sm_ref_y = tss->sets[0].pose.y;
		sm_ref_z = tss->sets[0].pose.z;
	}

	if(ssm_ref_x == INT32_MIN)
	{
		assert(current_voxfilter.n_ray_sources == 0); // voxfilter is cleared, either this is the first thing ever, or we just cleared it.
		ssm_ref_x = sm_ref_x - tss->sets[0].pose.x;
		ssm_ref_y = sm_ref_y - tss->sets[0].pose.y;
		ssm_ref_z = sm_ref_z - tss->sets[0].pose.z;
	}

	//printf("sm_ref = (%d,%d,%d) ssm_ref = (%d,%d,%d)\n", sm_ref_x, sm_ref_y, sm_ref_z, ssm_ref_x, ssm_ref_y, ssm_ref_z);

	tof_to_voxfilter_and_cloud(0, 
		tss->sets[0].ampldist, tss->sets[0].pose,
		tss->sidx, 
		sm_ref_x, sm_ref_y, sm_ref_z,
		&current_voxfilter, ssm_ref_x, ssm_ref_y, ssm_ref_z,
		&current_cloud, 1800, 300);

	if(tss->flags & TOF_SLAM_SET_FLAG_SET1_NARROW)
	{
		tof_to_voxfilter_and_cloud(1, 
			tss->sets[1].ampldist, tss->sets[1].pose,
			tss->sidx, 
			sm_ref_x, sm_ref_y, sm_ref_z,
			NULL, 0,0,0,
			&current_cloud, 1800, 3000);
	}
	else if(tss->flags & TOF_SLAM_SET_FLAG_SET1_WIDE)
	{
		tof_to_voxfilter_and_cloud(0, 
			tss->sets[1].ampldist, tss->sets[1].pose,
			tss->sidx, 
			sm_ref_x, sm_ref_y, sm_ref_z,
			NULL, 0,0,0,
			&current_cloud, 1800, 3000);
	}


	if(tss->sidx == LAST_SIDX)
	{
		hw_pose_t cur_pose = tss->sets[0].pose;

		if(prev_x != INT32_MIN) // prev_x,y,z valid
		{
			cumul_travel += sqrt(VECTSQ_I64(cur_pose.x, cur_pose.y, cur_pose.z, prev_x, prev_y, prev_z));
			double dyaw = ANG32TORAD(cur_pose.ang) - prev_yaw;
			WRAP_RAD_BIPO(dyaw); // dyaw will be -pi .. pi. Small actual rotation never gives a large value after this.
			cumul_yaw += fabs(dyaw);
			
		}

		prev_x = cur_pose.x;
		prev_y = cur_pose.y;
		prev_z = cur_pose.z;
		prev_yaw = ANG32TORAD(cur_pose.ang);

		TRACK_MIN(cur_pose.x, min_x);
		TRACK_MIN(cur_pose.y, min_y);
		TRACK_MIN(cur_pose.z, min_z);

		TRACK_MAX(cur_pose.x, max_x);
		TRACK_MAX(cur_pose.y, max_y);
		TRACK_MAX(cur_pose.z, max_z);

		avg_x += cur_pose.x;
		avg_y += cur_pose.y;
		avg_z += cur_pose.z;
		n_scans++;

		ss_avg_x += cur_pose.x;
		ss_avg_y += cur_pose.y;
		ss_avg_z += cur_pose.z;
		ss_n_scans++;

		int dx = max_x - min_x;
		int dy = max_y - min_y;
		int dz = max_z - min_z;


		if(ss_n_scans >= VOXFILTER_N_SCANS)
		{
			assert(ss_n_scans == VOXFILTER_N_SCANS);

			// Current subsubmap ended.
			// Well, now subsubmaps are nothing else than resets of voxfilters (and calculation of the new voxfilter reference point)

			voxfilter_to_cloud(&current_voxfilter, &current_cloud);
			memset(&current_voxfilter, 0, sizeof(voxfilter_t));
			ssm_ref_x = INT32_MIN; // This forces a new coordinate fetch before anything is applied to now empty voxfilter.
			ss_avg_x = ss_avg_y = ss_avg_z = 0;
			ss_n_scans = 0;
			n_subsubmaps++;

			printf("input_tof_slam_set: subsubmap finished, n_subsubmaps = %d\n", n_subsubmaps);


			// Check if we need to finish the current submap:
			// Let the dx,dy, or dz overshoot a bit, instead ensure that the submap is evenly divisible into subsubmaps
			if(	n_subsubmaps >= SUBSUBMAP_LIMIT ||
				abso(dx) > DX_LIMIT || abso(dy) > DY_LIMIT || abso(dz) > DZ_LIMIT ||
				cumul_travel > CUMUL_TRAVEL_LIMIT || cumul_yaw > CUMUL_YAW_LIMIT)
			{
				n_subsubmaps = 0;

				submap_metas[n_sms].min_x = min_x;
				submap_metas[n_sms].min_y = min_y;
				submap_metas[n_sms].min_z = min_z;
				submap_metas[n_sms].max_x = max_x;
				submap_metas[n_sms].max_y = max_y;
				submap_metas[n_sms].max_z = max_z;
				submap_metas[n_sms].avg_x = avg_x/n_scans;
				submap_metas[n_sms].avg_y = avg_y/n_scans;
				submap_metas[n_sms].avg_z = avg_z/n_scans;

				// Zero accumulation:
				min_x = min_y = min_z = INT32_MAX;
				max_x = max_y = max_z = INT32_MIN;
				avg_x = avg_y = avg_z = 0;
				cumul_travel = 0;
				cumul_yaw = 0.0;
				n_scans = 0;

				prev_x = prev_y = prev_z = INT32_MIN;
				prev_yaw = 0.0;


				// We only know the midpoint of the submap now that it's finished.
				// Our original reference is still stored at sm_ref_*.
				// Translate the finished cloud to get the origin at avg_*.
				int32_t transl_x = sm_ref_x - submap_metas[n_sms].avg_x;
				int32_t transl_y = sm_ref_y - submap_metas[n_sms].avg_y;
				int32_t transl_z = sm_ref_z - submap_metas[n_sms].avg_z;

				//printf("translation: (%d,%d,%d)\n", transl_x, transl_y, transl_z);

				sm_ref_x = INT32_MIN; // Force reload of sm_ref_* at the very next sensor dataset.

				// filter_cloud makes a new cloud.

				filter_cloud(&current_cloud, p_cur_cloud, transl_x, transl_y, transl_z);

				ret = n_sms;
				n_sms++;
				printf("input_tof_slam_set: submap finished, filtered the cloud (%d -> %d points), n_sms = %d\n", current_cloud.n_points, p_cur_cloud->n_points, n_sms);

				current_cloud.n_points = 0; // this is enough to clear a cloud. Old data doesn't matter.

				#if 0
				{
					po_coords_t po;
					po = po_coords(submap_metas[ret].avg_x, submap_metas[ret].avg_y, submap_metas[ret].avg_z, 0);
					load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
					cloud_to_voxmap(&tmp_cloud_filtered, submap_metas[ret].avg_x, submap_metas[ret].avg_y, submap_metas[ret].avg_z);
					store_all_pages();
				}
				#endif

			}

		}


	}

	return ret;

}

void process_after_input(int ret)
{
	save_cloud(p_cur_cloud, ret);

	if(ret >= 1)
		match_to_prev_submap(ret, p_prev_cloud, p_cur_cloud);

	/*
		Matchmap sets (for later loop closure matching) combine three adjacent submaps,
		still in memory. Resulting matchmaps are stored on disk.
	*/
	if(ret >= 2)
	{
		result_t adjab = adjacent_matches[ret-2].results[0];
		result_t adjbc = adjacent_matches[ret-1].results[0];

		//result_t adjab = (result_t){0,0,0,0,0,0};
		//result_t adjbc = (result_t){0,0,0,0,0,0};

		//printf("adjab (%d %d %d %f) adjbc (%d %d %d %f)\n", adjab.x,adjab.y,adjab.z,RADTODEG(adjab.yaw),adjbc.x,adjbc.y,adjbc.z,RADTODEG(adjbc.yaw));
		result_t corrab = {
			(submap_metas[ret-1].avg_x-submap_metas[ret-2].avg_x)-adjab.x,
			(submap_metas[ret-1].avg_y-submap_metas[ret-2].avg_y)-adjab.y,
			(submap_metas[ret-1].avg_z-submap_metas[ret-2].avg_z)-adjab.z,
			adjab.yaw};

		result_t corrbc = {
			(submap_metas[ret].avg_x-submap_metas[ret-1].avg_x)-adjbc.x,
			(submap_metas[ret].avg_y-submap_metas[ret-1].avg_y)-adjbc.y,
			(submap_metas[ret].avg_z-submap_metas[ret-1].avg_z)-adjbc.z,
			adjbc.yaw};

		ref_qs[ret-1] = gen_save_ref_matchmap_set(ret-1, p_prev_prev_cloud, p_prev_cloud, p_cur_cloud, corrab, corrbc);

		#if 0
		{
			static int tmp_run_y = 0;
			printf("IMG Y = %+8d, SCORE ^\n", tmp_run_y); 
			po_coords_t po;
			po = po_coords(0,tmp_run_y,0, 0);
			load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
			cloud_to_voxmap(p_prev_cloud, 0,tmp_run_y,0);

			tmp_run_y -= 20000;
			store_all_pages();
		}
		#endif

	}

	if(ret >= MIN_CLOSURE_LEN+1)
	//if(ret == 97)
	{
		find_possible_closures(submap_metas, ret-1, p_prev_cloud);
	}

	// Swap pointers to reuse the three-level buffer:
	// prev_prev is no longer needed, and will be rewritten as cur
	// The one which was cur, is now prev.
	// The one which was prev, is now prev_prev.
	cloud_t* tmp = p_prev_prev_cloud;
	p_prev_prev_cloud = p_prev_cloud;
	p_prev_cloud = p_cur_cloud;
	p_cur_cloud = tmp;

}

void input_from_file(int file_idx)
{
	char fname[1024];
	sprintf(fname, "/home/hrst/robotsoft/tsellari/trace%08d.rb2", file_idx);
	tof_slam_set_t* tss;
	if(process_file(fname, &tss) == 0) // tof_slam_set record succesfully extracted
	{
		int ret = input_tof_slam_set(tss);

		if(ret >= 0)
		{
			process_after_input(ret);
		}

	}
}

void slam_input_from_tss(tof_slam_set_t* tss)
{
	int ret = input_tof_slam_set(tss);

	if(ret >= 0)
	{
		process_after_input(ret);
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


void visualize_submaps()
{
	const int STEPY=20000;
	int32_t running_y = 0;

	printf("VISUALIZING SUBMAPS\n\n");

	for(int c=0; c<97; c++)
	{
		static cloud_t tmp_cloud;

		printf("Y = %d: submap %d\n", running_y, c);
		{
			load_cloud(&tmp_cloud, c);
			//rotate_cloud(&tmp_cloud, 0.0);
			po_coords_t po;
			po = po_coords(0,0+running_y,0, 0);
			load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
			cloud_to_voxmap(&tmp_cloud, 0,0+running_y,0);
		}
		running_y -= STEPY;

	}
	printf("\n");
	free_all_pages();

}

#ifdef SLAM_STANDALONE
	int main(int argc, char** argv)
	{
		p_cur_cloud = &filtered_clouds[0];
		p_prev_cloud = &filtered_clouds[1];
		p_prev_prev_cloud = &filtered_clouds[2];

		int start_i = 0;
		int end_i = 27294; //12800;

		for(int i = start_i; i<=end_i; i++)
			input_from_file(i);

		//free_all_pages();

		//visualize_submaps();

		return 0;
		load_closures(closures, &n_closures);


		{
			#define STEPX 15000
			#define STEPY 20000
			int32_t running_x = 0, running_y = 0;

			printf("VISUALIZING CLOSURES\n\n");

			int col = 0;
			//int c = 0;
			for(int c=0; c<n_closures; c++)
			{
				static cloud_t tmp_cloud;

				int sma = closures[c].first;
				int smb = closures[c].last;
	//			int smc = closures[c].first + 1;

				result_t result = closures[c].possibilities[0];

				printf("y=%+6d   %3d  %3d  (%+6d,%+6d,%+6d,%+6.2f) SCORE=%d ABSCORE=%d\n", running_y, sma, smb, 
					result.x, result.y, result.z, RADTODEG(result.yaw), result.score, result.abscore);

				// SMA alone
				{
					load_cloud(&tmp_cloud, sma);
					//rotate_cloud(&tmp_cloud, 0.0);
					po_coords_t po;
					po = po_coords(0+running_x,0+running_y,0, 0);
					load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
					cloud_to_voxmap(&tmp_cloud, 0+running_x,0+running_y,0);
				}

				// SMB alone
				{
					load_cloud(&tmp_cloud, smb);
					//rotate_cloud(&tmp_cloud, yaw_corr);
					po_coords_t po;
					po = po_coords(STEPX+running_x,running_y,0, 0);
					load_pages(RESOLEVELS, RESOLEVELS, po.px-3, po.px+3, po.py-3, po.py+3, po.pz-2, po.pz+2);
					cloud_to_voxmap(&tmp_cloud, STEPX+running_x,running_y,0);
				}


				running_y -= STEPY;

			}
			printf("\n");
		}


		free_all_pages();









	#if 0
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

		static cloud_t tmp_cloud;

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

				static cloud_t tmp_cloud_filtered;

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
				static cloud_t cloud;
				load_tmp_cloud(&cloud, i);
				char name[1000];
				sprintf(name, "submap%05d_x%d_y%d_z%d.xyz", i, submap_metas[i].avg_x, submap_metas[i].avg_y, submap_metas[i].avg_z);
				cloud_to_xyz_file(&cloud, name);
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

			static cloud_t ca, cb, cc;
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
				static cloud_t tmp_cloud;
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

	#endif

		return 0;
	} 
#endif
