//#define FILTER_OFF

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

#include "api_board_to_soft.h"

#include "voxmap.h"
#include "voxmap_memdisk.h"

#include "slam_config.h"


#include "slam_cloud.h"
#include "slam_matchers.h"
#include "slam_icp.h"

#include "misc.h"

#include "tcp_parser.h"




/*
#define RADTODEG(x) ((x)*(360.0/(2.0*M_PI)))
#define DEGTORAD(x) ((x)*((2.0*M_PI)/360.0))
#define ANG32TORAD(x) ( ((double)((uint32_t)(x)))/683565275.576432)
#define ANG32TOFDEG(x) ( ((double)((uint32_t)(x)))/11930464.7111111)
*/

	


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



// Pitch and roll are assumed to have no accumulating error (based on gravity vector always available)
// Accumulating yaw error is eventually limited by compass, but compass will be inaccurate and subject
// to local magnetic fields.

#define XY_ACCUM_UNCERT_BY_SUBMAP 400 // mm
#define Z_ACCUM_UNCERT_BY_SUBMAP 0   // mm
#define YAW_ACCUM_UNCERT_BY_SUBMAP (DEGTORAD(1.5))
#define YAW_ACCUM_UNCERT_SATURATION (DEGTORAD(110))

#define TRY_CLOSURE_MARGIN_XY 6000
#define TRY_CLOSURE_MARGIN_Z  1000




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

int input_tof_slam_set_for_gyrocal(tof_slam_set_t* tss, int state)
{
	printf("Sorry, gyro calibration slam matching functions aren't implemented now. TODO: re-implement using the ICP matching.\n");
	exit(1);
}

void gyroslam_process_before()
{
	printf("Sorry, gyro calibration slam matching functions aren't implemented now. TODO: re-implement using the ICP matching.\n");
	exit(1);
}

void gyroslam_empty()
{
	printf("Sorry, gyro calibration slam matching functions aren't implemented now. TODO: re-implement using the ICP matching.\n");
	exit(1);
}

double gyroslam_process_after()
{
	printf("Sorry, gyro calibration slam matching functions aren't implemented now. TODO: re-implement using the ICP matching.\n");
	exit(1);
}

#if 0
// state sequence: 0 = init, 1 = input "before" or "after" data
int input_tof_slam_set_for_gyrocal(tof_slam_set_t* tss, int state)
{

	if(! (tss->flags & TOF_SLAM_SET_FLAG_VALID))
		return -1;

	int ret = -1;

	// Reuse filtered_clouds[0], [1], [2] respectively as: input, before, after.

	static int32_t sm_ref_x = INT32_MIN, sm_ref_y, sm_ref_z;

	if(state == 0)
	{
		sm_ref_x = tss->sets[0].pose.x;
		sm_ref_y = tss->sets[0].pose.y;
		sm_ref_z = tss->sets[0].pose.z;
		//printf("ref is %d,%d,%d\n", sm_ref_x, sm_ref_y, sm_ref_z);
		filtered_clouds[0].n_points = 0;
	}

	assert(sm_ref_x != INT32_MIN);

	tof_to_voxfilter_and_cloud(0, 
		tss->sets[0].ampldist, tss->sets[0].pose,
		tss->sidx, 
		sm_ref_x, sm_ref_y, sm_ref_z,
		NULL, 0,0,0,
		&filtered_clouds[0], 1800, 300,
		NULL, 0);

	if(tss->flags & TOF_SLAM_SET_FLAG_SET1_NARROW)
	{
		tof_to_voxfilter_and_cloud(1, 
			tss->sets[1].ampldist, tss->sets[1].pose,
			tss->sidx, 
			sm_ref_x, sm_ref_y, sm_ref_z,
			NULL, 0,0,0,
			&filtered_clouds[0], 1800, 3000,
			NULL, 1);
	}


	return 0;
}

void gyroslam_process_before()
{
	printf("input_tof_slam_set_for_gyrocal: \"BEFORE\" pointcloud finished, filtering the cloud (%d -> ", filtered_clouds[0].n_points); fflush(stdout);
	filter_cloud(&filtered_clouds[0], &filtered_clouds[1], 0,0,0);
	printf("%d points)\n", filtered_clouds[1].n_points);

	filtered_clouds[0].n_points = 0; // ready to accumulate the "after" one.
}

void gyroslam_empty()
{
	filtered_clouds[0].n_points = 0; // ready to accumulate the "after" one.
}

double gyroslam_process_after()
{
	printf("input_tof_slam_set_for_gyrocal: \"AFTER\" pointcloud finished, filtering the cloud (%d -> ", filtered_clouds[0].n_points); fflush(stdout);
	filter_cloud(&filtered_clouds[0], &filtered_clouds[2], 0,0,0);
	printf("%d points)\n", filtered_clouds[2].n_points);

	result_t res = gyrocal_match_submaps(&filtered_clouds[1], &filtered_clouds[2], 0,0,0);

	return -1.0*res.yaw;
}
#endif







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

// mallocs the output. Remember to free it.
small_cloud_t* convert_cloud_to_small_cloud(cloud_t* in)
{
	small_cloud_t* out = malloc(sizeof(small_cloud_t) * in->m.n_points);
	assert(out);
	for(int i=0; i<in->m.n_points; i++)
	{
		out[i] = set_small_cloud_native_units(0, 0, 0, 0, in->points[i].x, in->points[i].y, in->points[i].z);
	}
	return out;
}

#ifdef SLAM_STANDALONE

int32_t corr_x, corr_y, corr_z;
int64_t corrcorr_x, corrcorr_y;  // x65536, accumulating (x,y) shift due to coordinate rotation from angular corrections
uint32_t corr_yaw, corr_pitch, corr_roll;

int sw_correct_pos(int32_t da, int32_t dx, int32_t dy, int32_t dz)
{
	corr_yaw += (uint32_t)da;
	corr_x += dx;
	corr_y += dy;
	corr_z += dz;
}


// Correct any hw_pose silently:
// This must be called with data in-order, because it keeps track of movement deltas, to correctly rotate the vectors
void fix_pose(hw_pose_t* pose)
{
	static int32_t prev_x = INT32_MIN;
	static int32_t prev_y;
	static int32_t prev_z;

	if(prev_x == INT32_MIN)
	{
		prev_x = pose->x;
		prev_y = pose->y;
		prev_z = pose->z;
	}

	int32_t dx = pose->x - prev_x;
	int32_t dy = pose->y - prev_y;
	int32_t dz = pose->z - prev_z;

	double fcorr_yaw = ANG32TORAD(corr_yaw);
	corrcorr_x += 65536.0*((double)(dx) * cos(fcorr_yaw) - (double)(dy) * sin(fcorr_yaw)  - (double)(dx));
	corrcorr_y += 65536.0*((double)(dx) * sin(fcorr_yaw) + (double)(dy) * cos(fcorr_yaw)  - (double)(dy));

//	printf("raw pose (%d,%d,%d, %.1f deg), d (%d,%d,%d), corr is now %.1f deg, x=%d, y=%d, corrcorr x=%lld, y=%lld\n", 
//		pose->x, pose->y, pose->z, ANG32TOFDEG(pose->ang), dx, dy, dz, 
//		ANG32TOFDEG(corr_yaw), corr_x, corr_y, corrcorr_x>>16, corrcorr_y>>16);


	prev_x = pose->x;
	prev_y = pose->y;
	prev_z = pose->z;

	pose->x += corr_x + (corrcorr_x>>16);
	pose->y += corr_y + (corrcorr_y>>16);
	pose->z += corr_z;
	pose->ang += corr_yaw;
	pose->pitch += corr_pitch;
	pose->roll += corr_roll;
}

#else

extern int sw_correct_pos(int32_t da, int32_t dx, int32_t dy, int32_t dz);
extern void fix_pose(hw_pose_t* pose);
#endif

int ignoring = 0;
int input_tof_slam_set(tof_slam_set_t* tss)
{
	if(tss->sidx == FIRST_SIDX)
		ignoring = !ignoring;

	if(ignoring)
		return 0;

	if(! (tss->flags & TOF_SLAM_SET_FLAG_VALID))
		return -1;

	int ret = -1;

//	fix_pose(&tss->sets[0].pose);
//	fix_pose(&tss->sets[1].pose);

	static cloud_t clouds[N_SENSORS];
	static int cur_sidx = FIRST_SIDX;

	printf("input_tof_slam_set sidx = %d  ", cur_sidx);
	printf("set0 "); print_hw_pose(&tss->sets[0].pose);
//	printf("set1 "); print_hw_pose(&tss->sets[1].pose);

	if(cur_sidx != tss->sidx)
	{
		printf("WARNING: ignoring out-of-order tof_slam_set sidx=%d, expecting %d\n", tss->sidx, cur_sidx);
		return -1;
	}

	static cloud_t submap;


	if(cur_sidx == FIRST_SIDX)
	{
		if(!cloud_is_init(&submap))
		{
			printf("init submap.."); fflush(stdout);
			init_cloud(&submap, 0);
			printf("ok\n");
			submap.m.ref_x = tss->sets[0].pose.x/CLOUD_MM;
			submap.m.ref_y = tss->sets[0].pose.y/CLOUD_MM;
			submap.m.ref_z = tss->sets[0].pose.z/CLOUD_MM;
		}
	}


	//printf("init clouds[%d]..", cur_sidx); fflush(stdout);
	init_cloud(&clouds[cur_sidx], CLOUD_INIT_SMALL);
	//printf("ok\n");


	clouds[cur_sidx].m.ref_x = tss->sets[0].pose.x/CLOUD_MM;
	clouds[cur_sidx].m.ref_y = tss->sets[0].pose.y/CLOUD_MM;
	clouds[cur_sidx].m.ref_z = tss->sets[0].pose.z/CLOUD_MM;

	//printf("ref (%d,%d,%d)\n", (int)clouds[cur_sidx].m.ref_x,(int)clouds[cur_sidx].m.ref_y,(int)clouds[cur_sidx].m.ref_z);
	// Set 0 is always there

	//printf("tof_to_cloud.."); fflush(stdout);

	tof_to_cloud(0, 0, tss,
		tss->sets[0].pose.x, tss->sets[0].pose.y, tss->sets[0].pose.z,
		1, 
		&clouds[cur_sidx], 0);

	//printf("ok\n");


	// Set 1, if available, contains either a narrow beam image, or a longer-range wide beam image 
	// Use the same reference coordinates as the first set
	// Ignore close-by points, and do not use filter because it's only needed for close-by points.

	#ifdef USE_NARROWS
		if(tss->flags & TOF_SLAM_SET_FLAG_SET1_NARROW)
		{
			printf("ADDING LONG NARROW SET\n");

			cloud_t set1;
			init_cloud(&set1, CLOUD_INIT_SMALL);

			set1.m.ref_x = tss->sets[0].pose.x/CLOUD_MM;
			set1.m.ref_y = tss->sets[0].pose.y/CLOUD_MM;
			set1.m.ref_z = tss->sets[0].pose.z/CLOUD_MM;

			tof_to_cloud(1, 1, tss,
				tss->sets[0].pose.x, tss->sets[0].pose.y, tss->sets[0].pose.z,
				0, &set1, 5000);

			match_by_closest_points(&clouds[cur_sidx], &set1, &clouds[cur_sidx],
				0.0f, 0.0f, 0.0f, 0.0f,
				0, // no iters, just combine
				200.0, // match threshold (meaningless because iters=0)
				300.0, // points can't be combined if further away
				50.0, 0.017, // points can't be combined if perpendicularly further away from the sensor->point line,
					    // than 30mm + 1.5% of the distance (sensor to point)
					    // At 4000mm distance, this is 90.0mm
				NULL,NULL,NULL,NULL, ICP_USE_A); // correction results meaningless because iters = 0

			free_cloud(&set1);

			assert(clouds[cur_sidx].m.n_srcs == 1);
		}
	#endif

	#ifdef USE_LONG_WIDES
		// Untested code because the firmware isn't producing wide sets right now...
		if(tss->flags & TOF_SLAM_SET_FLAG_SET1_WIDE)
		{
			printf("ADDING LONG WIDE SET\n");

			cloud_t set1;
			init_cloud(&set1, CLOUD_INIT_SMALL);

			set1.m.ref_x = tss->sets[0].pose.x/CLOUD_MM;
			set1.m.ref_y = tss->sets[0].pose.y/CLOUD_MM;
			set1.m.ref_z = tss->sets[0].pose.z/CLOUD_MM;

			tof_to_cloud(0, 1, tss,
				tss->sets[0].pose.x, tss->sets[0].pose.y, tss->sets[0].pose.z,
				0, &set1, 5000);

			match_by_closest_points(&clouds[cur_sidx], &set1, &clouds[cur_sidx],
				0.0f, 0.0f, 0.0f, 0.0f,
				0, // no iters, just combine
				200.0, // match threshold (meaningless because iters=0)
				300.0, // points can't be combined if further away
				50.0, 0.017, // points can't be combined if perpendicularly further away from the sensor->point line,
					    // than 30mm + 1.5% of the distance (sensor to point)
					    // At 4000mm distance, this is 90.0mm
				NULL,NULL,NULL,NULL, ICP_USE_A); // correction results meaningless because iters = 0

			free_cloud(&set1);

			assert(clouds[cur_sidx].m.n_srcs == 1);
		}
	#endif



	if(cur_sidx == LAST_SIDX)
	{
		cur_sidx = FIRST_SIDX;

		// Build a cloud with just one round of sensors, by combining clouds[N_SENSORS].
		// ICP combining is used, but without any correction iterations; individual sensor clouds
		// are therefore combined by the original hw_pose differences, but the ICP combining algorithm
		// does allow moving individual points a bit when detecting their associations, creating 1 averaged point
		// from 2 input points, keeping the areas of sensor overlap clean, having equivalent point density to
		// the areas of no overlap.

		cloud_t combined_scan;
		init_cloud(&combined_scan, 0);
		combined_scan.m.ref_x = clouds[FIRST_SIDX].m.ref_x;
		combined_scan.m.ref_y = clouds[FIRST_SIDX].m.ref_y;
		combined_scan.m.ref_z = clouds[FIRST_SIDX].m.ref_z;

		cat_cloud(&combined_scan, &clouds[FIRST_SIDX]);
		for(int sidx=FIRST_SIDX+1; sidx<=LAST_SIDX; sidx++)
		{
			match_by_closest_points(&combined_scan, &clouds[sidx], &combined_scan,
				0.0f, 0.0f, 0.0f, 0.0f,
				0, // no iters, just combine
				200.0, // match threshold (meaningless because iters=0)
				200.0, // points can't be combined if further away
				30.0, 0.015, // points can't be combined if perpendicularly further away from the sensor->point line,
					    // than 30mm + 1.5% of the distance (sensor to point)
				            // At 4000mm distance, this is 90.0mm
				NULL,NULL,NULL,NULL, ICP_USE_COMB); // correction results meaningless because iters = 0

			free_cloud(&clouds[sidx]);
		}


		// Output realtime view on tcp:
		{
			small_cloud_t* sc = convert_cloud_to_small_cloud(&combined_scan);
			tcp_send_small_cloud(combined_scan.m.ref_x*CLOUD_MM, combined_scan.m.ref_y*CLOUD_MM, combined_scan.m.ref_x*CLOUD_MM,
				combined_scan.m.n_points, sc);
			free(sc);
		}



		// Now one full sensor scan is in combined_scan. ICP match it to the submap cloud.
		// Here, ICP iterations are used to register the new scan to the existing submap.
		// After registration, sensor scan is combined with the existing submap, again averaging overlapping points,
		// keeping point density acceptable.
		// ICP provides pose correction, which is given to the sw_correct_pos function, which adjusts correction parameters
		// that will correct all incoming data afterwards, making this an "on-line" slam.
		// If loop closures are needed later, it's best to treat this on-line adjusted hw_pose as the pose estimate, forgetting/overwriting
		// the "raw" original hw_poses, because the ICP seems to do quite good job.

		assert(cloud_is_init(&submap));
		if(submap.m.n_points < 1000)
		{
			cat_cloud(&submap, &combined_scan);
		}
		else
		{
			float cx=0.0, cy=0.0, cz=0.0, cyaw=0.0;
			match_by_closest_points(&submap, &combined_scan, &submap,
				0.0f, 0.0f, 0.0f, 0.0f,
				30, // n iters
				200.0, // match threshold
				300.0, // points can't be combined if further away
				100.0, 0.025, // points can't be combined if perpendicularly further away from the sensor->point line,
					    // than 20mm + 4% of the distance (sensor to point)
					    // At 4000mm distance, this is 180.0mm
				&cx,&cy,&cz,&cyaw, ICP_USE_COMB); // correction results

				sw_correct_pos(RADTOANGI32(cyaw), cx, cy, cz);

		}		

		free_cloud(&combined_scan);


		static int32_t min_x = INT32_MAX, min_y = INT32_MAX, min_z = INT32_MAX;
		static int32_t max_x = INT32_MIN, max_y = INT32_MIN, max_z = INT32_MIN;
		static int64_t avg_x = 0, avg_y = 0, avg_z = 0;
		static double cumul_travel = 0.0, cumul_yaw = 0.0;
		static int32_t prev_x = INT32_MIN, prev_y = INT32_MIN, prev_z = INT32_MIN;
		static double prev_yaw;
		static int n_scans;


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

		int dx = max_x - min_x;
		int dy = max_y - min_y;
		int dz = max_z - min_z;

/*
		// Check if we need to finish the current submap:
		if(	n_scans > SCAN_LIMIT || 
			abso(dx) > DX_LIMIT || abso(dy) > DY_LIMIT || abso(dz) > DZ_LIMIT ||
			cumul_travel > CUMUL_TRAVEL_LIMIT || cumul_yaw > CUMUL_YAW_LIMIT)
		{
			n_scans = 0;

			int64_t new_ref_x = avg_x/n_scans;
			int64_t new_ref_y = avg_y/n_scans;
			int64_t new_ref_z = avg_z/n_scans;

			// We only know the midpoint of the submap now that it's finished.
			// Our original reference is in submap.m.ref_*
			// Translate the finished cloud to get the origin at avg_*.

			int transl_x = new_ref_x - submap.m.ref_x;
			int transl_y = new_ref_y - submap.m.ref_y;
			int transl_z = new_ref_z - submap.m.ref_z;

			submap.m.ref_x = new_ref_x;
			submap.m.ref_y = new_ref_y;
			submap.m.ref_z = new_ref_z;

			assert(abs(transl_x) < 10000);
			assert(abs(transl_y) < 10000);
			assert(abs(transl_z) < 10000);

			translate_cloud(&submap, transl_x, transl_y, transl_z);


			// Zero accumulation:
			min_x = min_y = min_z = INT32_MAX;
			max_x = max_y = max_z = INT32_MIN;
			avg_x = avg_y = avg_z = 0;
			cumul_travel = 0.0;
			cumul_yaw = 0.0;
			n_scans = 0;
			prev_x = prev_y = prev_z = INT32_MIN; // invalid value
*/

			{
				small_cloud_t* sc = convert_cloud_to_small_cloud(&submap);
				save_small_cloud("cla.smallcloud", 0,0,0, submap.m.n_points, sc);
				free(sc);
			}


			static int cnt;

			if(++cnt == 16)
			{
				cnt = 0;
				cloud_t filtered_submap;
				init_cloud(&filtered_submap, 0);

				freespace_filter_cloud(&filtered_submap, &submap);
				//free_cloud(&submap);



				{
					small_cloud_t* sc = convert_cloud_to_small_cloud(&filtered_submap);
					save_small_cloud("clb.smallcloud", 0,0,0, filtered_submap.m.n_points, sc);
					free(sc);
				}

				free_cloud(&filtered_submap);
			}
//		}

	}
	else
	{

		cur_sidx++;
	}



	return ret;

}



void input_from_file(int file_idx)
{
	char fname[1024];
	sprintf(fname, "/home/hrst/pulu/tut_trace/trace%08d.rb2", file_idx);
	tof_slam_set_t* tss;
	if(process_file(fname, &tss) == 0) // tof_slam_set record succesfully extracted
	{
		int ret = input_tof_slam_set(tss);
/*
		if(ret >= 0)
		{
			process_after_input(ret);
		}
*/
	}
}

int slam_input_from_tss(tof_slam_set_t* tss, result_t* corr_out)
{
	assert(corr_out);

	int ret = input_tof_slam_set(tss);

	if(ret >= 0)
	{
		//*corr_out = legacy_process_after_input(ret);

		return 1;
	}

	return 0;
}


#if 0
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
#endif

#if 0
void visualize_submaps()
{
	const int STEPY=20000;
	int32_t running_y = 0;

	printf("VISUALIZING SUBMAPS\n\n");

	for(int c=0; c<5; c++)
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
#endif

void init_slam()
{
//	p_cur_cloud = &filtered_clouds[0];
//	p_prev_cloud = &filtered_clouds[1];
//	p_prev_prev_cloud = &filtered_clouds[2];

	assert(sizeof (cloud_point_t) == 16);
}


#ifdef SLAM_STANDALONE
int main(int argc, char** argv)
{
	printf("sizeof (cloud_point_t) = %d\n", (int)sizeof(cloud_point_t));
	init_slam();
	load_sensor_softcals();

	int start_i = argc>=2?atoi(argv[1]):0;
	int end_i = start_i + (argc==3?atoi(argv[2]):15);

	for(int i = start_i; i<=end_i; i++)
		input_from_file(i);

	

	return 0;
} 
#endif
