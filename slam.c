#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>


#define DEFINE_API_VARIABLES
#include "api_board_to_soft.h"
#undef DEFINE_API_VARIABLES

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

	int32_t min_x;
	int32_t avg_x;
	int32_t max_x;

	int32_t min_y;
	int32_t avg_y;
	int32_t max_y;

	int32_t min_z;
	int32_t avg_z;
	int32_t max_z;

} submap_meta_t;

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
	int count_limit = 64;
	// When max_x-min_x exceeds the limit, new submap group is initiated (at the next FIRST_SIDX edge)
	int dx_limit = 5000;
	int dy_limit = 5000;
	int dz_limit = 2000;

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

				n_smm++;
				break;
			}

//			int dx = fsp[cur_fsp].pose.x - fsp[start_fsp].pose.x;
//			int dy = fsp[cur_fsp].pose.y - fsp[start_fsp].pose.y;
//			int dz = fsp[cur_fsp].pose.z - fsp[start_fsp].pose.z;

			int dx = max_x - min_x;
			int dy = max_y - min_y;
			int dz = max_z - min_z;

			if(next_start_fsp == -1 && (abso(dx) > dx_limit/2 || abso(dy) > dy_limit/2 || abso(dz) > dz_limit/2))
			{
				next_start_fsp = cur_fsp;
			}

			if(abso(dx) > dx_limit || abso(dy) > dy_limit || abso(dz) > dz_limit || cur_fsp >= start_fsp + count_limit)
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

void tof_to_voxmap(int is_narrow, uint16_t* ampldist, hw_pose_t* pose, int sidx, int32_t ref_x, int32_t ref_y)
{
	if(sidx < 0 || sidx >= N_SENSORS)
	{
		printf("Invalid sidx\n");
		return;
	}

	int32_t robot_x = pose->x;
	int32_t robot_y = pose->y;

	uint16_t robot_ang = pose->ang>>16;

	static int32_t prev_x, prev_y;
	static uint16_t prev_ang;
	static int ignore = 0;

	if(sidx == 1)
	{
		int dx = robot_x - prev_x;
		int dy = robot_y - prev_y;
		int da = (int16_t)((uint16_t)robot_ang - (uint16_t)prev_ang);


		if(abso(dx) < 40 && abso(dy) < 40 && abso(da) < 700)
		{
			printf("Not much movement, ignoring until next sidx=1\n");
			ignore = 1;
		}
		else
		{
			ignore = 0;

			prev_x = robot_x;
			prev_y = robot_y;
			prev_ang = robot_ang;
		}

	}

	if(ignore)
		return;

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
	int32_t  global_sensor_z = sensor_mounts[sidx].z_rel_ground;


	int32_t  local_sensor_x = sensor_mounts[sidx].x_rel_robot;
	int32_t  local_sensor_y = sensor_mounts[sidx].y_rel_robot;
	int32_t  local_sensor_z = sensor_mounts[sidx].z_rel_ground;


	int insertion_cnt = 0;
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

//						int32_t dist = (ampldist[(npy+iy)*TOF_XS_NARROW+(npx+ix)]&DIST_MASK)<<DIST_SHIFT;
				return; // todo: implement

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


				int z_coord = ((z-BASE_Z)/Z_STEP);
				
				x /= XY_STEP;
				y /= XY_STEP;

				x += XS/2;
				y += YS/2;

				if(x < 0 || x >= XS || y < 0 || y >= YS || z_coord < 0 || z_coord > 63)
				{
				//	printf("Ignore OOR point x=%d, y=%d, z=%d\n", x, y, z_coord);
					continue;

				}
/*
				int oldval = voxmap[y*XS+x][z_coord];
				int increment = d/1000;
				if(increment < 1) increment = 1;
				else if(increment > 10) increment = 10;
				int newval = oldval + increment;
				if(newval > 255)
					newval = 255;
				voxmap[y*XS+x][z_coord] = newval;*/

			}
		}
	}

}

static uint8_t buf[B2S_MAX_LEN];


// Not thread safe, supplies pointer to internal static buf
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
	printf("Kak?\n");
	static firstsidx_pose_t firstsidx_poses[MAX_FIRSTSIDX_POSES];
	static submap_meta_t submap_metas[MAX_SUBMAPS];

	int n_firstsidx_poses;

	extract_firstsidx_poses(firstsidx_poses, &n_firstsidx_poses);

	int n_submaps;

	group_submaps(submap_metas, &n_submaps, firstsidx_poses, n_firstsidx_poses);

	for(int sm=0; sm<n_submaps; sm++)
	{
		printf("Submap %d: idx %8d .. %8d  (len %4d)", sm, submap_metas[sm].start_idx, submap_metas[sm].end_idx, submap_metas[sm].end_idx-submap_metas[sm].start_idx);
		if(sm < n_submaps-1)
		{
			printf(" (overlaps the next by %4d)  ", -1*(submap_metas[sm+1].start_idx - submap_metas[sm].end_idx));
		}
		else
			printf("                              ");

		printf("dx=%4d  dy=%4d  dz=%4d   avg (%+6d %+6d %+6d)\n", submap_metas[sm].max_x-submap_metas[sm].min_x, submap_metas[sm].max_y-submap_metas[sm].min_y,
			submap_metas[sm].max_z-submap_metas[sm].min_z, submap_metas[sm].avg_x, submap_metas[sm].avg_y, submap_metas[sm].avg_z);
	}	


	return 0;
} 
