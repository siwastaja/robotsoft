/*
	Point cloud type, transforming, processing and filtration functions for SLAM
*/

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <errno.h>

#include "slam_config.h"
#include "slam_cloud.h"
#include "api_board_to_soft.h"

#include "voxmap.h"
#include "voxmap_memdisk.h"

#define COMPRESS_CLOUDS

#ifdef COMPRESS_CLOUDS
	#include <zlib.h>

	#define ZLIB_CHUNK (64*1024*1024)
	#define ZLIB_LEVEL 3  // from 1 to 9, 9 = slowest but best compression
#endif


void init_cloud(cloud_t* cloud, int flags)
{
	cloud->m.n_srcs = 0;
	cloud->m.n_points = 0;
	cloud->m.n_poses = 0;
	cloud->m.n_freevects = 0;
	
	if(flags & CLOUD_INIT_SMALL)
	{
//		cloud->alloc_srcs = INITIAL_SRCS_ALLOC_SMALL;
		cloud->alloc_points = INITIAL_POINTS_ALLOC_SMALL;
		cloud->alloc_freevects = INITIAL_POINTS_ALLOC_SMALL;
		cloud->alloc_poses = INITIAL_POSES_ALLOC_SMALL;
	}
	else
	{
//		cloud->alloc_srcs = INITIAL_SRCS_ALLOC_LARGE;
		cloud->alloc_points = INITIAL_POINTS_ALLOC_LARGE;
		cloud->alloc_freevects = INITIAL_POINTS_ALLOC_LARGE;
		cloud->alloc_poses = INITIAL_POSES_ALLOC_LARGE;
	}

//	cloud->srcs = malloc(cloud->alloc_srcs * sizeof (cloud_point_t));
//	assert(cloud->srcs);
	cloud->points = malloc(cloud->alloc_points * sizeof (cloud_point_t));
	assert(cloud->points);
	cloud->freevects = malloc(cloud->alloc_freevects * sizeof (freevect_point_t));
	assert(cloud->freevects);
	cloud->poses = malloc(cloud->alloc_poses * sizeof (hw_pose_t));
	assert(cloud->poses);
}

// You supply a pointer to an otherwise-uninitialized cloud, but you have
// set the m.n_* fields. This function does the rest of the init: allocates for
// this exact amount of space.
void alloc_cloud(cloud_t* cloud)
{
//	cloud->alloc_srcs = cloud->m.n_srcs;
	printf("alloc_cloud n_points = %d, n_poses = %d, n_freevects = %d\n", cloud->m.n_points, cloud->m.n_poses, cloud->m.n_freevects);
	cloud->alloc_points = cloud->m.n_points;
	cloud->alloc_poses = cloud->m.n_poses;
	cloud->alloc_freevects = cloud->m.n_freevects;

//	cloud->srcs = malloc(cloud->alloc_srcs * sizeof (cloud_point_t));
//	assert(cloud->srcs);
	cloud->points = malloc(cloud->alloc_points * sizeof (cloud_point_t));
	assert(cloud->points);
	cloud->freevects = malloc(cloud->alloc_freevects * sizeof (freevect_point_t));
	assert(cloud->freevects);
	cloud->poses = malloc(cloud->alloc_poses * sizeof (hw_pose_t));
	assert(cloud->poses);
}

// Translates all cloud points, sources and freevects.
// Note that the cloud operates on limited numerical range. Use this function to re-reference the cloud, or similar.
// If you just want to move the cloud around, think about adjusting the cloud.m.ref_* fields.
void translate_cloud(cloud_t * cloud, int tx, int ty, int tz)
{
	for(int i=0; i<cloud->m.n_points; i++)
		cloud->points[i] = transform_point(cloud->points[i], tx, ty, tz, 0.0);

	for(int i=0; i<cloud->m.n_srcs; i++)
		cloud->srcs[i] = transform_point(cloud->srcs[i], tx, ty, tz, 0.0);

	for(int i=0; i<cloud->m.n_freevects; i++)
		cloud->freevects[i] = transform_freevect(cloud->freevects[i], tx, ty, tz, 0.0);
}

// translates cloud in and its sources to the reference of out.
// Tries to find matching sources in in, creates new sources if necessary, changes the source indeces.
void cat_cloud(cloud_t * const restrict out, const cloud_t * const restrict in)
{
	uint8_t* src_transl = calloc(in->m.n_srcs, sizeof (uint8_t));
	assert(src_transl);

	int64_t tx = in->m.ref_x - out->m.ref_x;
	int64_t ty = in->m.ref_y - out->m.ref_y;
	int64_t tz = in->m.ref_z - out->m.ref_z;
	assert(tx > -10000 && tx < 10000);
	assert(ty > -10000 && ty < 10000);
	assert(tz > -10000 && tz < 10000);

	for(int i=0; i<in->m.n_srcs; i++)
	{
		int newsrc = cloud_find_source(out, CL_P(in->srcs[i].x+tx, in->srcs[i].y+ty, in->srcs[i].z+tz));
		src_transl[i] = newsrc;
		//printf("cat_cloud: translating in source %d to out source %d\n", i, newsrc);
	}

	cloud_prepare_fast_add_points(out, in->m.n_points);
	for(int i=0; i<in->m.n_points; i++)
	{
		cloud_point_t newpoint = in->points[i];
		for(int i=0; i<newpoint.n_srcs; i++)
		{
			newpoint.srcs[i] = src_transl[newpoint.srcs[i]];
			assert(newpoint.srcs[i] < out->m.n_srcs);
		}

		newpoint.x += tx;
		newpoint.y += ty;
		newpoint.z += tz;

		cloud_fast_add_point(out, newpoint);
	}
	//printf("cat_cloud: added %d points translated by (%d,%d,%d)\n", in->m.n_points, (int)tx, (int)ty, (int)tz);
	free(src_transl);
}

// ... but keep the allocations and other meta fields
void cloud_remove_points(cloud_t* cloud)
{
	cloud->m.n_srcs = 0;
	cloud->m.n_points = 0;
	cloud->m.n_freevects = 0;
	cloud->m.n_poses = 0;
}

void cloud_remove_points_keep_sources(cloud_t* cloud)
{
	cloud->m.n_points = 0;
}

int cloud_is_init(cloud_t* cloud)
{
	return (cloud->points != NULL);
}

void cloud_print_sources(cloud_t* cloud, int analyze)
{
	int cnt_srcs[256] = {0};

	if(analyze)
	{
		for(int p=0; p<cloud->m.n_points; p++)
		{
			for(int i=0; i<cloud->points[p].n_srcs; i++)
				cnt_srcs[cloud->points[p].srcs[i]]++;
		}
	}

	printf("SOURCES [n=%d]:\n", cloud->m.n_srcs);
	for(int i=0; i<cloud->m.n_srcs; i++)
	{
		printf("   %3d: (%+06d,%+06d,%+06d)", i, cloud->srcs[i].x*CLOUD_MM,cloud->srcs[i].y*CLOUD_MM,cloud->srcs[i].z*CLOUD_MM);

		if(analyze)
			printf("   COUNT = %4d\n", cnt_srcs[i]);
		else
			printf("\n");
	}
}

void free_cloud(cloud_t* cloud)
{
	if(cloud->points) free(cloud->points);
	if(cloud->poses) free(cloud->poses);
	if(cloud->freevects) free(cloud->freevects);

	// sets all pointers to NULL, all meta to zero, all sources to zero and so on:
	memset(cloud, 0, sizeof *cloud);
}


// Format: 
// cloud->m header as is (inludes number of srcs, points and poses)
// Optionally, rest is ZLIB compressed:
// srcs
// points
// poses

int save_cloud(cloud_t* cloud, int idx)
{
	char fname[1024];
	snprintf(fname, 1024, "submap%06u.bin", idx);
	FILE* f = fopen(fname, "wb");
	assert(f);

	assert(fwrite(&cloud->m, sizeof cloud->m, 1, f) == 1);

	assert(fwrite(cloud->srcs, sizeof cloud->srcs[0], cloud->m.n_srcs, f) == cloud->m.n_srcs);
	assert(fwrite(cloud->poses, sizeof cloud->poses[0], cloud->m.n_poses, f) == cloud->m.n_poses);
	assert(fwrite(cloud->freevects, sizeof cloud->freevects[0], cloud->m.n_freevects, f) == cloud->m.n_freevects);


	int rc = 0;
	#ifdef COMPRESS_CLOUDS
		uint8_t* outbuf = malloc(ZLIB_CHUNK);
		assert(outbuf);
		z_stream strm;
		strm.zalloc = Z_NULL;
		strm.zfree = Z_NULL;
		strm.opaque = Z_NULL;
		if(deflateInit(&strm, ZLIB_LEVEL) != Z_OK)
		{
			printf("ERROR: ZLIB initialization failed\n");
			abort();
		}

		strm.avail_in = cloud->m.n_points*sizeof *cloud->points;
		strm.next_in = (uint8_t*)cloud->points;

		do
		{
			strm.avail_out = ZLIB_CHUNK;
			strm.next_out = outbuf;

			int ret = deflate(&strm, Z_NO_FLUSH);
			assert(ret != Z_STREAM_ERROR);

			int produced = ZLIB_CHUNK - strm.avail_out;

			//printf("produced=%d\n", produced);

			if(produced > 0)
			{
				if( (rc=fwrite(outbuf, produced, 1, f)) != 1 || ferror(f))
				{
					printf("ERROR: fwrite failed, rc=%d, errno = %s\n", rc, strerror(errno));
					abort();
				}
			}
		} while(strm.avail_out == 0);

		assert(strm.avail_in == 0);

		deflateEnd(&strm);

		free(outbuf);

	#else
		// untested code:
		assert(fwrite(cloud->points, sizeof cloud->points[0], cloud->m.n_points, f) == cloud->m.n_points);
	#endif


	fclose(f);
	return 0;
}

int load_cloud(cloud_t* cloud, int idx)
{
	char fname[1024];
	snprintf(fname, 1024, "submap%06u.bin", idx);
	FILE* f = fopen(fname, "rb");
	if(!f)
		return -1;

	assert(fread(&cloud->m, sizeof cloud->m, 1, f) == 1);

	alloc_cloud(cloud);

	assert(fread(cloud->srcs, sizeof cloud->srcs[0], cloud->m.n_srcs, f) == cloud->m.n_srcs);
	assert(fread(cloud->poses, sizeof cloud->poses[0], cloud->m.n_poses, f) == cloud->m.n_poses);
	assert(fread(cloud->freevects, sizeof cloud->freevects[0], cloud->m.n_freevects, f) == cloud->m.n_freevects);


	#ifdef COMPRESS_CLOUDS
		uint8_t* inbuf = malloc(ZLIB_CHUNK);
		assert(inbuf);

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
		int bytes_left = cloud->m.n_points*sizeof cloud->points[0];
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

				ret = inflate(&strm, Z_NO_FLUSH);
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

		free(inbuf);
	#else
		// untested code
		assert(fread(cloud->points, sizeof cloud->points[0], cloud->m.n_points, f) == cloud->m.n_points);
	#endif

	fclose(f);
	return 0;
}


/*
#define CLOUDFLT_XS 1280
#define CLOUDFLT_YS 1280
#define CLOUDFLT_ZS 384
#define CLOUDFLT_UNIT 32 //mm
*/

#define CLOUDFLT_XS 800 // 448
#define CLOUDFLT_YS 800 // 448
#define CLOUDFLT_ZS 64  // 96
#define CLOUDFLT_UNIT (64/*mm*/ / CLOUD_MM)

#define MAX_OCCU_CNT 15
#define MAX_FREE_CNT 15

typedef struct __attribute__((packed))
{
	uint8_t free_cnt : 4;
	uint8_t occu_cnt : 4;
} flt_vox_t;

static flt_vox_t flt_voxes[CLOUDFLT_XS][CLOUDFLT_YS][CLOUDFLT_ZS];



#define UNCERT_Z 1
#define UNCERT 1
#define LINE_CONE_SLOPE_XY 1500
#define LINE_CONE_SLOPE_Z  2000

ALWAYS_INLINE void output_free_voxel(int x, int y, int z)
{
	if(x >= 1 && x < CLOUDFLT_XS-1 && y >= 1 && y < CLOUDFLT_YS-1 && z > 1 && z <= CLOUDFLT_ZS-1)
		if(flt_voxes[x][y][z].free_cnt < MAX_FREE_CNT)
			flt_voxes[x][y][z].free_cnt++;

}

ALWAYS_INLINE void output_occu_voxel(int x, int y, int z)
{
	if(x >= 1 && x < CLOUDFLT_XS-1 && y >= 1 && y < CLOUDFLT_YS-1 && z > 1 && z <= CLOUDFLT_ZS-1)
		if(flt_voxes[x][y][z].occu_cnt < MAX_FREE_CNT)
			flt_voxes[x][y][z].occu_cnt++;

}

ALWAYS_INLINE void output_n_occu_voxel(int x, int y, int z, int n)
{
	if(x >= 1 && x < CLOUDFLT_XS-1 && y >= 1 && y < CLOUDFLT_YS-1 && z > 1 && z <= CLOUDFLT_ZS-1)
	{
		if(flt_voxes[x][y][z].occu_cnt + n >= MAX_OCCU_CNT)
			flt_voxes[x][y][z].occu_cnt = MAX_OCCU_CNT;
		else
			flt_voxes[x][y][z].occu_cnt+= n;
	}

}


static inline uint8_t get_voxel_free(int x, int y, int z)
{
	if(! (x >= 0 && x < CLOUDFLT_XS && y >= 0 && y < CLOUDFLT_YS && z > 0 && z <= CLOUDFLT_ZS))
		return 0;

	return flt_voxes[x][y][z].free_cnt;
}

static inline uint8_t get_voxel_occu(int x, int y, int z)
{
	if(! (x >= 0 && x < CLOUDFLT_XS && y >= 0 && y < CLOUDFLT_YS && z > 0 && z <= CLOUDFLT_ZS))
		return 0;

	return flt_voxes[x][y][z].occu_cnt;
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
					output_free_voxel(px+ix, py+iy, pz+iz);
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
					output_free_voxel(px+ix, py+iy, pz+iz);
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
					output_free_voxel(px+ix, py+iy, pz+iz);
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

/*
flt_voxes is a voxel map where value >0 means that this voxel has been seen empty
de-edging makes continuous volume smaller by eating the edges by one voxel
Without this, sensor noise causes loss of real structures. Consider the example of floor, seen sideways

	W
	W
	W
	W          R
	FFFFFFFFFFFFFFF
	   NN

	W=actual wall
	F=actual floor
	N=noise
	R=robot

	Weeeee
	Weeeeee
	Weeeeeeee
	WeeeeeeeeeR
	FFFFFxxxxFFFFFF
	   NN

	e= area traced empty correctly
	x= area traced empty because of incorrect observation (N)


	After de-edging the empty map:

	W     
	W eeee 
	W eeeeee 
	W   eeeee R
	FFFFFFFFFFFFFFF
	   NN


*/ 
// if defined, de-edges 2 blocks wide instead of just 1
//#define DE_EDGE_2
#define DONT_DE_EDGE_FROM_ABOVE

static void de_edge_flt_voxes()
{
	// along z axis, up and down
	for(int iy=0; iy < CLOUDFLT_YS; iy++)
	{
		for(int ix=0; ix < CLOUDFLT_XS; ix++)
		{
			for(int iz=2; iz < CLOUDFLT_ZS-1; iz++)
			{
				if(!flt_voxes[ix][iy][iz-2].free_cnt && !flt_voxes[ix][iy][iz-1].free_cnt && flt_voxes[ix][iy][iz].free_cnt)
				{
					flt_voxes[ix][iy][iz].free_cnt = 0;
					#ifdef DE_EDGE_2
					flt_voxes[ix][iy][iz+1].free_cnt = 0;
					iz++;
					#endif
					iz+=2; // skip so that the point we just removed does not cause continuous removal of everything
				}
			}

			#ifndef DONT_DE_EDGE_FROM_ABOVE
				for(int iz=CLOUDFLT_ZS-2-1; iz >= 1; iz--)
				{
					if(!flt_voxes[ix][iy][iz+2].free_cnt && !flt_voxes[ix][iy][iz+1].free_cnt && flt_voxes[ix][iy][iz].free_cnt)
					{
						flt_voxes[ix][iy][iz].free_cnt = 0;
						#ifdef DE_EDGE_2
						flt_voxes[ix][iy][iz-1].free_cnt = 0;
						iz--;
						#endif
						iz-=2;
					}
				}
			#endif

		}
	}

	// along x axis
	for(int iy=0; iy < CLOUDFLT_YS; iy++)
	{
		for(int iz=0; iz < CLOUDFLT_ZS; iz++)
		{
			for(int ix=2; ix < CLOUDFLT_XS-1; ix++)
			{
				if(!flt_voxes[ix-2][iy][iz].free_cnt && !flt_voxes[ix-1][iy][iz].free_cnt && flt_voxes[ix][iy][iz].free_cnt)
				{
					flt_voxes[ix][iy][iz].free_cnt = 0;
					#ifdef DE_EDGE_2
					flt_voxes[ix+1][iy][iz].free_cnt = 0;
					ix++;
					#endif
					ix+=3;
				}
			}

			for(int ix=CLOUDFLT_XS-2-1; ix >= 1; ix--)
			{
				if(!flt_voxes[ix+2][iy][iz].free_cnt && !flt_voxes[ix+1][iy][iz].free_cnt && flt_voxes[ix][iy][iz].free_cnt)
				{
					flt_voxes[ix][iy][iz].free_cnt = 0;
					#ifdef DE_EDGE_2
					flt_voxes[ix-1][iy][iz].free_cnt = 0;
					ix--;
					#endif
					ix-=3;
				}
			}

		}
	}

	// along y axis
	for(int iz=0; iz < CLOUDFLT_ZS; iz++)
	{
		for(int ix=0; ix < CLOUDFLT_XS; ix++)
		{
			for(int iy=2; iy < CLOUDFLT_YS-1; iy++)
			{
				if(!flt_voxes[ix][iy-2][iz].free_cnt && !flt_voxes[ix][iy-1][iz].free_cnt && flt_voxes[ix][iy][iz].free_cnt)
				{
					flt_voxes[ix][iy][iz].free_cnt = 0;
					#ifdef DE_EDGE_2
					flt_voxes[ix][iy+1][iz].free_cnt = 0;
					iy++;
					#endif
					iy+=3; // skip so that the point we just removed does not cause continuous removal of everything
				}
			}

			for(int iy=CLOUDFLT_YS-2-1; iy >= 1; iy--)
			{
				if(!flt_voxes[ix][iy+2][iz].free_cnt && !flt_voxes[ix][iy+1][iz].free_cnt && flt_voxes[ix][iy][iz].free_cnt)
				{
					flt_voxes[ix][iy][iz].free_cnt = 0;
					#ifdef DE_EDGE_2
					flt_voxes[ix][iy-1][iz].free_cnt = 0;
					iy--;
					#endif
					iy-=3;
				}
			}

		}
	}


}

static void flt_vox_midfree()
{
	// along z axis, up and down
	for(int iy=0; iy < CLOUDFLT_YS; iy++)
	{
		for(int ix=0; ix < CLOUDFLT_XS; ix++)
		{
			int strong_occupied_start = -1;
			for(int iz=0; iz < CLOUDFLT_ZS-1; iz++) // coming from below...
			{
				if( (((int)flt_voxes[ix][iy][iz].occu_cnt+(int)flt_voxes[ix][iy][iz+1].occu_cnt) >= 8))//  &&
				    //((flt_voxes[ix][iy][iz].free_cnt+flt_voxes[ix][iy][iz+1].free_cnt) <= 0)) // strong case of "occupied" not free, found
				{
					// test code to delete everything above first appearance of such case:
					//for(int i=iz+2; i<CLOUDFLT_ZS; i++)
					//	flt_voxes[ix][iy][i].free_cnt = 1;

					strong_occupied_start = iz;
					continue;
				}

				// test code to delete everything above first appearance and disappearance of such case:
/*
				if(strong_occupied_start > -1)
				{
					for(int i=iz+2; i<CLOUDFLT_ZS; i++)
						flt_voxes[ix][iy][i].free_cnt = 1;

				}
*/

				if(strong_occupied_start > -1 && flt_voxes[ix][iy][iz].free_cnt >= 1) // "free" also found
				{
					// test code to delete everything above first appearance and disappearane, if free is also found

					/*
					for(int i=strong_occupied_start+2; i<CLOUDFLT_ZS; i++)
					{
						flt_voxes[ix][iy][i].free_cnt = 1;
						iz = 999;
					}
					*/

					int strong_occupied_end = iz;

					// Look for another free, until last free is found. Any strong occupied stops the process.
					for(int i=strong_occupied_end; i<CLOUDFLT_ZS-1; i++)
					{
						if(flt_voxes[ix][iy][i].free_cnt >= 1)
						{
							strong_occupied_end = i;
						}

						if((((int)flt_voxes[ix][iy][i].occu_cnt+(int)flt_voxes[ix][iy][i+1].occu_cnt) >= 8))
						{
							break;
						}
					}

					for(int i=strong_occupied_start+2; i<strong_occupied_end; i++)
						flt_voxes[ix][iy][i].free_cnt = 1; // assume everything inbetween free.

					strong_occupied_start = -1; // continue looking for more
				}

			}
		}
	}

}

void cloud_copy_all_but_points(cloud_t* restrict out, cloud_t* restrict cloud)
{
	out->m = cloud->m;
	out->m.n_points = 0;
	memcpy(out->srcs, cloud->srcs, sizeof out->srcs);
}


void freespace_filter_cloud(cloud_t* out, cloud_t* cloud)
{
//	printf("filter_cloud ref (%d, %d, %d)\n", ref_x, ref_y, ref_z);
	memset(flt_voxes, 0, sizeof(flt_voxes));
	// Trace empty space to points:
	for(int p=0; p<cloud->m.n_points; p++)
	{
		int px = cloud->points[p].x/CLOUDFLT_UNIT + CLOUDFLT_XS/2;
		int py = cloud->points[p].y/CLOUDFLT_UNIT + CLOUDFLT_YS/2;
		int pz = cloud->points[p].z/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

		for(int s=0; s<cloud->points[p].n_srcs; s++)
		{
			if(cloud->points[p].srcs[s] == CLOUD_GENERAL_SRC)
				continue;
			int sx = cloud->srcs[ cloud->points[p].srcs[s] ].x/CLOUDFLT_UNIT + CLOUDFLT_XS/2;
			int sy = cloud->srcs[ cloud->points[p].srcs[s] ].y/CLOUDFLT_UNIT + CLOUDFLT_YS/2;
			int sz = cloud->srcs[ cloud->points[p].srcs[s] ].z/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

			//printf("p=%d, (%d,%d,%d)->(%d,%d,%d)\n", sx,sy,sz, px,py,pz);
			bresenham3d_cloudflt(sx, sy, sz, px, py, pz);
		}

		// If the point is seen from many sources, it's a strong (persistent) point. Add more count.
		output_n_occu_voxel(px,py,pz,cloud->points[p].n_srcs);
	}

	// Trace empty space with freevects:

	printf("FILTER: n_freevects = %d\n", cloud->m.n_freevects);
	for(int p=0; p<cloud->m.n_freevects; p++)
	{
		if(cloud->freevects[p].src == CLOUD_GENERAL_SRC)
			continue;

		int px = cloud->freevects[p].x/CLOUDFLT_UNIT + CLOUDFLT_XS/2;
		int py = cloud->freevects[p].y/CLOUDFLT_UNIT + CLOUDFLT_YS/2;
		int pz = cloud->freevects[p].z/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

		int sx = cloud->srcs[cloud->freevects[p].src].x/CLOUDFLT_UNIT + CLOUDFLT_XS/2;
		int sy = cloud->srcs[cloud->freevects[p].src].y/CLOUDFLT_UNIT + CLOUDFLT_YS/2;
		int sz = cloud->srcs[cloud->freevects[p].src].z/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

		bresenham3d_cloudflt(sx, sy, sz, px, py, pz);
	}

	de_edge_flt_voxes();
	
	// Whenever we have a strong case of free voxel, and a strong case of occupied voxel some distance below or above it, assume
	// everything inbetween as free.

	flt_vox_midfree();

	cloud_copy_all_but_points(out, cloud);
	// The number of points out is typically close to the number of points in. Allocate for full number.
	cloud_prepare_fast_add_points(out, cloud->m.n_points);

	// Remove points at empty space by only outputting points where there is no free voxel.
	for(int p=0; p<cloud->m.n_points; p++)
	{
		int px = cloud->points[p].x/CLOUDFLT_UNIT + CLOUDFLT_XS/2;
		int py = cloud->points[p].y/CLOUDFLT_UNIT + CLOUDFLT_YS/2;
		int pz = cloud->points[p].z/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

		if(get_voxel_free(px, py, pz) < 1)
//		if(get_voxel_occu(px,py,pz) >= 5)
		{
			cloud_fast_add_point(out, cloud->points[p]);
		}
			
	}
}

//#define CLOUD_TO_VOXMAP_TRACE_FREE

#if 0
void cloud_to_voxmap(cloud_t* cloud, int ref_x, int ref_y, int ref_z)
{
	printf("cloud_to_voxmap .."); fflush(stdout);
	#ifdef CLOUD_TO_VOXMAP_TRACE_FREE
		generate_cloudflt_free(cloud, 0, 0, 0);

		printf(" generated free .."); fflush(stdout);

	#endif
	for(int i=0; i<cloud->m.n_points; i++)
	{
		for(int rl=0; rl<MAX_RESOLEVELS; rl++)
		{
			if(!(RESOLEVELS & (1<<rl)))
				continue;

			po_coords_t po_p = po_coords(cloud->points[i].px+ref_x, cloud->points[i].py+ref_y, cloud->points[i].pz+ref_z, rl);
			load_page_quick_and_mark_changed(po_p.px, po_p.py, po_p.pz);

			uint8_t* p_vox = get_p_voxel(po_p, rl);
			if(((*p_vox) & 0x0f) < 15)
				(*p_vox)++;
		}
	}

	printf(" added points .."); fflush(stdout);

	// OPT_TODO: Page numbers are always the same for each resolevel, so one load would be enough

	#ifdef CLOUD_TO_VOXMAP_TRACE_FREE
		// Load multiple pages at once
		for(int xx=0; xx<CLOUDFLT_XS; xx++)
		{
			int freemap_offs_x = (xx-(CLOUDFLT_XS/2))*CLOUDFLT_UNIT;

			// x and y ranges for load. rl irrelevant, only using page numbers
			// End ranges have +32, because rl0 is handled to produce 2*2*2 output voxels, see below
			po_coords_t polx0 = po_coords(freemap_offs_x+ref_x, (-CLOUDFLT_YS/2)*CLOUDFLT_UNIT+ref_y, 0, 0);
			po_coords_t polx1 = po_coords(freemap_offs_x+ref_x+32, ( CLOUDFLT_YS/2-1)*CLOUDFLT_UNIT+ref_y+32, 0, 0);

			// z range for load
			po_coords_t polz0 = po_coords(0, 0, (-CLOUDFLT_ZS/2)*CLOUDFLT_UNIT+ref_z, 0);
			po_coords_t polz1 = po_coords(0, 0, ( CLOUDFLT_ZS/2-1)*CLOUDFLT_UNIT+ref_z+32, 0);

			load_pages(RESOLEVELS, RESOLEVELS, polx0.px, polx1.px, polx0.py, polx1.py, polz0.pz, polz1.pz);
			//printf("LOAD: %d,%d   %d,%d   %d,%d\n", polx0.px, polx1.px, polx0.py, polx1.py, polz0.pz, polz1.pz);
			for(int yy=0; yy<CLOUDFLT_YS; yy++)
			{
				int freemap_offs_y = (yy-(CLOUDFLT_YS/2))*CLOUDFLT_UNIT;
				for(int zz=0; zz<CLOUDFLT_ZS; zz++)
				{
					int freemap_offs_z = (zz-(CLOUDFLT_ZS/2))*CLOUDFLT_UNIT;
					if(free_cnt[xx][yy][zz] > 0) // free space - mark it free, remove objects
					{

						// rl0 separately - step 32mm, freemap step 64mm, copy 2*2*2 times.
						if(RESOLEVELS & 1)
						{
							for(int iy=0; iy<=1; iy++)
							{
								for(int ix=0; ix<=1; ix++)
								{
									for(int iz=0; iz<=1; iz++)
									{
										po_coords_t po_f = po_coords(ref_x+freemap_offs_x+ix*32, ref_y+freemap_offs_y+iy*32, ref_z+freemap_offs_z+iz*32, 0);
										//load_page_quick_and_mark_changed(po_f.px, po_f.py, po_f.pz);
										mark_page_changed(po_f.px, po_f.py, po_f.pz);
										//printf("%d %d %d; ", po_f.px, po_f.py, po_f.pz);

										uint8_t* p_vox = get_p_voxel(po_f, 0);

										*p_vox = 0xf0; // mark free and remove objects.
									}
								}
							}
						}

						for(int rl=1; rl<MAX_RESOLEVELS; rl++)
						{
							if(!(RESOLEVELS & (1<<rl)))
								continue;

							po_coords_t po_f = po_coords(ref_x+freemap_offs_x, ref_y+freemap_offs_y, ref_z+freemap_offs_z, rl);
							//load_page_quick_and_mark_changed(po_f.px, po_f.py, po_f.pz);
							mark_page_changed(po_f.px, po_f.py, po_f.pz);
							//printf("%d %d %d; ", po_f.px, po_f.py, po_f.pz);

							uint8_t* p_vox = get_p_voxel(po_f, rl);

							*p_vox = 0xf0; // mark free and remove objects.

						}
					}

				}
			}
		}

		printf(" added free .."); fflush(stdout);

	#endif

	printf(" done\n");
}
#endif


#define DIST_UNDEREXP 0
#define DIST_OVEREXP 1
#include "sin_lut.c"

sensor_softcal_t sensor_softcals[N_SENSORS];

void load_sensor_softcals()
{
	for(int sidx=0; sidx<N_SENSORS; sidx++)
	{
		char fname[4096];
		snprintf(fname,4096, "./softcals/softcal_%02u.bin", sidx);
		FILE* f = fopen(fname, "rb");
		if(!f || fread(&sensor_softcals[sidx], sizeof(sensor_softcal_t), 1, f) != 1)
		{
			printf("ERROR: Opening sensor calibration file %s failed.\n", fname);
			abort();
		}
		fclose(f);
	}
}

//#define SUBPIX // started implementing angular resolution improvement. Feel free to continue. Now enabling this does nothing.

void tof_to_cloud(int is_narrow, int setnum, tof_slam_set_t* tss, int32_t ref_x, int32_t ref_y, int32_t ref_z,
	 int do_filter, cloud_t* cloud, int dist_ignore_threshold)
{
	// Voxel filter divides the near-field area in discrete blocks
	// On each block, average coordinates of all points hitting that block are calculated
	// Outputs only one point per block (with the average coordinates)

	#define FILTER_XS 192
	#define FILTER_YS 192
	#define FILTER_ZS 128
	#define FILTER_STEP 32
	#define FILTER_IDX(x_, y_, z_) ((x_)*FILTER_YS*FILTER_ZS + (y_)*FILTER_ZS + (z_))

	typedef struct
	{
		int32_t cnt;
		int32_t x_acc;
		int32_t y_acc;
		int32_t z_acc;
	} filter_point_t;

	filter_point_t* filter = NULL;
	
	if(do_filter)
	{
		filter = calloc(FILTER_XS*FILTER_YS*FILTER_ZS, sizeof filter[0]);
		assert(filter);
	}

	int sidx = tss->sidx;	
	assert(sidx >= 0 && sidx < N_SENSORS);

	int32_t robot_x = tss->sets[setnum].pose.x;
	int32_t robot_y = tss->sets[setnum].pose.y;
	int32_t robot_z = tss->sets[setnum].pose.z;

	uint16_t robot_ang = tss->sets[setnum].pose.ang>>16;

	// Rotation: xr = x*cos(a) - y*sin(a)
	//           yr = x*sin(a) + y*cos(a)

	uint16_t global_sensor_hor_ang = sensor_softcals[sidx].mount.ang_rel_robot + robot_ang;
//	uint16_t global_sensor_ver_ang = sensor_softcals[sidx].mount.vert_ang_rel_ground;

	int16_t pitch_ang = tss->sets[setnum].pose.pitch>>16;
	int16_t roll_ang = tss->sets[setnum].pose.roll>>16;

//	printf("pitch=%.2f  roll=%.2f\n", ANGI32TOFDEG(tss->sets[setnum].pose.pitch), ANGI32TOFDEG(tss->sets[setnum].pose.roll));

	// Temporary bodge to "calibrate" the unit on the field I'm writing this for.
	// Gravitation vector calibration could have been done at the time of sensor calibration very easily but didn't cross mind then.
	pitch_ang += DEGTOANG16(0.7);
	roll_ang  += DEGTOANG16(0.7);

	uint16_t global_sensor_ver_ang = 
		(int32_t)((int16_t)sensor_softcals[sidx].mount.vert_ang_rel_ground) +
		((lut_cos_from_u16(sensor_softcals[sidx].mount.ang_rel_robot)*pitch_ang)>>SIN_LUT_RESULT_SHIFT) +
		((lut_sin_from_u16(sensor_softcals[sidx].mount.ang_rel_robot)*roll_ang)>>SIN_LUT_RESULT_SHIFT);

	// The angle of sensor rotating around it's own normal
	uint16_t sensor_rota = 
		((lut_sin_from_u16(sensor_softcals[sidx].mount.ang_rel_robot)*-pitch_ang)>>SIN_LUT_RESULT_SHIFT) +
		((lut_cos_from_u16(sensor_softcals[sidx].mount.ang_rel_robot)*roll_ang)>>SIN_LUT_RESULT_SHIFT);

	int32_t sin_sensor_rota = lut_sin_from_u16(sensor_rota);
	int32_t cos_sensor_rota = lut_cos_from_u16(sensor_rota);

//	printf("sensor_rota=%u  sin=%d  cos=%d\n", sensor_rota, sin_sensor_rota, cos_sensor_rota);

	int32_t  global_sensor_x = robot_x - ref_x +
			(((int64_t)lut_cos_from_u16(pitch_ang)*(int64_t)lut_cos_from_u16(robot_ang)*sensor_softcals[sidx].mount.x_rel_robot)>>(2*SIN_LUT_RESULT_SHIFT)) +
			(((int64_t)lut_cos_from_u16(roll_ang)*(int64_t)lut_sin_from_u16(robot_ang)*-sensor_softcals[sidx].mount.y_rel_robot)>>(2*SIN_LUT_RESULT_SHIFT));

	int32_t  global_sensor_y = robot_y - ref_y + 
			(((int64_t)lut_cos_from_u16(pitch_ang)*(int64_t)lut_sin_from_u16(robot_ang)*sensor_softcals[sidx].mount.x_rel_robot)>>(2*SIN_LUT_RESULT_SHIFT)) +
			(((int64_t)lut_cos_from_u16(roll_ang)*(int64_t)lut_cos_from_u16(robot_ang)*sensor_softcals[sidx].mount.y_rel_robot)>>(2*SIN_LUT_RESULT_SHIFT));

	int32_t  global_sensor_z = robot_z - ref_z + sensor_softcals[sidx].mount.z_rel_ground +
			((lut_sin_from_u16(pitch_ang)*sensor_softcals[sidx].mount.x_rel_robot)>>SIN_LUT_RESULT_SHIFT) +
			((lut_sin_from_u16(roll_ang)*sensor_softcals[sidx].mount.y_rel_robot)>>SIN_LUT_RESULT_SHIFT);

	#ifdef VACUUM_APP
		uint16_t local_sensor_hor_ang = sensor_softcals[sidx].mount.ang_rel_robot;
		uint16_t local_sensor_ver_ang = sensor_softcals[sidx].mount.vert_ang_rel_ground;

		int32_t  local_sensor_x = sensor_softcals[sidx].mount.x_rel_robot;
		int32_t  local_sensor_y = sensor_softcals[sidx].mount.y_rel_robot;
		int32_t  local_sensor_z = sensor_softcals[sidx].mount.z_rel_ground;
	#endif

	int sx = global_sensor_x;
	int sy = global_sensor_y;
	int sz = global_sensor_z;

	// cloud_find_source reuses an old source if close enough, or creates a new one:
	int src_idx = cloud_find_source(cloud, CL_P_MM(sx, sy, sz));

	// Corner ignore:

	#define X_EDGE_IGNORE 8
	static const int px_ignore[TOF_YS] =
	{
		50, 45, 41, 37, 33, 
		30, 27, 24, 21, 18,
		16, 14, 12, 10,  8,
		 7,  6,  5,  4,  3,
		 2,  2,  1,  1,  0,
		 0,  0,  0,  0,  0,

		 0,  0,  0,  0,  0,
		 0,  1,  1,  2,  2,
		 3,  4,  5,  6,  7,
		 8, 10, 12, 14, 16,
		18, 21, 24, 27, 30,
		33, 37, 41, 45, 50
	};

	// Generate points
	for(int py=1; py<TOF_YS-1; py++)
	{
		int px_start = X_EDGE_IGNORE;
		int px_end = TOF_XS-X_EDGE_IGNORE;

		px_start += px_ignore[py];
		px_end   -= px_ignore[py];

		for(int px=px_start; px<px_end; px++)
		{
			// Start by looking at 3x3 pixels. Middle pixel is refdist. Any of the 9 pixels must be
			// close enough to that refdist. If they are too different, they are ignored. If acceptable, they
			// are averaged together to form the final reading (placed at the mid pixel). If too many
			// are ignored, no point is generated.
			// This obviously improves accuracy by averaging, but also removes "mid-lier" pixels caused by
			// exposing a single pixel to a combination of different distances.
			int_fast32_t avg = 0;
			int n_conform = 0;
			int_fast32_t refdist;

			#ifdef SUBPIX
			int_fast32_t ix_sum = 0, iy_sum = 0;
			#endif

			if(is_narrow)
			{
				int npy, npx;
				npx=px-TOF_NARROW_X_START;
				npy=py-TOF_NARROW_Y_START;
				if(npx < 1 || npy < 1 || npx >= TOF_XS_NARROW-1 || npy >= TOF_YS_NARROW-1)
					continue;

				refdist = tss->sets[setnum].ampldist[(npy+0)*TOF_XS_NARROW+(npx+0)]&DIST_MASK;
				if(refdist == DIST_UNDEREXP)
					continue;

				// Acceptance limit depends on the refdist; when further away, larger differences within the 3x3 block
				// are accepted. This is because the further we are, the further the pixels are from each other sideways, now a wall
				// which isn't perpendicular to a sensor has expected larger differences per pixel.
				// This  is done by dividing refdist by 16 then adding 130mm, so that:
				// At refdist = zero-ish, accept everything +/-130mm
				// At refdist = 5m, accept +/-442mm (5000mm * sin 1 degree * tan 79 degrees = 449mm)
				// At refdist = 10m, accept +/- 755mm (10000mm * sin 1 degree * tan 77 degrees = 755mm)
				// This way, non-perpendicular surfaces are not filtered out.
				for(int iy=-1; iy<=1; iy++)
				{
					for(int ix=-1; ix<=1; ix++)
					{
						int32_t dist = tss->sets[setnum].ampldist[(npy+iy)*TOF_XS_NARROW+(npx+ix)]&DIST_MASK;
						if(dist != DIST_UNDEREXP && dist != DIST_OVEREXP && dist > refdist-(((refdist<<DIST_SHIFT)/16+130)>>DIST_SHIFT) && dist < refdist+(((refdist<<DIST_SHIFT)/16+130)>>DIST_SHIFT))
						{
							avg+=dist;
							n_conform++;							
						}
					
					}
				}
			}
			else
			{
				refdist = tss->sets[setnum].ampldist[(py+0)*TOF_XS+(px+0)]&DIST_MASK;
				if(refdist == DIST_UNDEREXP)
					continue;

				for(int iy=-1; iy<=1; iy++)
				{
					for(int ix=-1; ix<=1; ix++)
					{
						int32_t dist = tss->sets[setnum].ampldist[(py+iy)*TOF_XS+(px+ix)]&DIST_MASK;
						if(dist != DIST_UNDEREXP && dist != DIST_OVEREXP && dist > refdist-(((refdist<<DIST_SHIFT)/16+130)>>DIST_SHIFT) && dist < refdist+(((refdist<<DIST_SHIFT)/16+130)>>DIST_SHIFT))
						{
							avg+=dist;
							n_conform++;
						}
					
					}
				}
			}

			if(n_conform >= TOF_N_CONFORM_REQUIRED)
			{
				avg <<= DIST_SHIFT; // Convert to mm
				avg /= n_conform;
				int64_t d = avg;

				//refdist <<= DIST_SHIFT;  // if you ever need refdist, convert to mm first

				int16_t h_ang = -sensor_softcals[sidx].hor_angs[py*TOF_XS+px];
				int16_t v_ang = sensor_softcals[sidx].ver_angs[py*TOF_XS+px];

				// Rotate the pixel angles according to the sensor rotation
				// Rotation: xr = x*cos(a) - y*sin(a)
				//           yr = x*sin(a) + y*cos(a)

				uint16_t hor_ang, ver_ang;
				hor_ang = ((int32_t)h_ang*cos_sensor_rota - (int32_t)v_ang*sin_sensor_rota)>>SIN_LUT_RESULT_SHIFT;
				ver_ang = ((int32_t)h_ang*sin_sensor_rota + (int32_t)v_ang*cos_sensor_rota)>>SIN_LUT_RESULT_SHIFT;
				//hor_ang = h_ang;
				//ver_ang = v_ang;

				uint16_t comb_hor_ang = hor_ang + global_sensor_hor_ang;
				uint16_t comb_ver_ang = ver_ang + global_sensor_ver_ang;

				int32_t x = (((int64_t)d * (int64_t)lut_cos_from_u16(comb_ver_ang) * (int64_t)lut_cos_from_u16(comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + global_sensor_x;
				int32_t y = (((int64_t)d * (int64_t)lut_cos_from_u16(comb_ver_ang) * (int64_t)lut_sin_from_u16(comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + global_sensor_y;
				int32_t z = (((int64_t)d * (int64_t)lut_sin_from_u16(comb_ver_ang))>>SIN_LUT_RESULT_SHIFT) + global_sensor_z;


				//if(z < 250 || z > 500)
				//	continue;

				if(is_narrow && z < 0)
					continue;

				#ifdef VACUUM_APP
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
				#endif


				if(d < dist_ignore_threshold)
					continue;
				
				if(filter)
				{
					int fltx = FILTER_XS/2 + x/FILTER_STEP;
					int flty = FILTER_YS/2 + y/FILTER_STEP;
					int fltz = FILTER_ZS/2 + z/FILTER_STEP;

					if(fltx < 0 || fltx >= FILTER_XS || flty < 0 || flty >= FILTER_YS || fltz < 0 || fltz >= FILTER_ZS)
						cloud_add_point(cloud, CL_SRCP_MM(src_idx, x, y, z));
					else
					{
						filter[FILTER_IDX(fltx, flty, fltz)].cnt++;
						filter[FILTER_IDX(fltx, flty, fltz)].x_acc += x;
						filter[FILTER_IDX(fltx, flty, fltz)].y_acc += y;
						filter[FILTER_IDX(fltx, flty, fltz)].z_acc += z;
					}
					
				}
				else
					cloud_add_point(cloud, CL_SRCP_MM(src_idx, x, y, z));
			}
		}
	}

	// Per each filter voxel, create only one output point, which is the average of the points falling inside each voxel
	if(filter)
	{
		for(int xx=0; xx<FILTER_XS; xx++)
		{
			for(int yy=0; yy<FILTER_YS; yy++)
			{
				for(int zz=0; zz<FILTER_ZS; zz++)
				{
					int cnt = filter[FILTER_IDX(xx, yy, zz)].cnt;
					if(cnt == 0)
						continue;

					int x = filter[FILTER_IDX(xx, yy, zz)].x_acc / cnt;
					int y = filter[FILTER_IDX(xx, yy, zz)].y_acc / cnt;
					int z = filter[FILTER_IDX(xx, yy, zz)].z_acc / cnt;

					cloud_add_point(cloud, CL_SRCP_MM(src_idx, x, y, z));
				}

			}

		}

		free(filter);
	}


	#define FREEVECT_PIXEL_SKIP 3
	#define FREEVECT_PIXEL_AREA 2

	#define ASSUMED_SENSOR_RANGE 2500.0f // going over 2000 is risky, black objects may end up removed


	// Generate freevects.
	// Require that all pixels within square of 2*FREEVECT_PIXEL_AREA x 2*FREEVECT_PIXEL_AREA are underexposed
	// Freevect is an imaginary point where we can assume we have somewhat guaranteed sensor range even under worst cases
	// If we read "underexposed" for multiple pixels, we can assume that, for example, if there is anything there, it
	// has to be further than, say, 1 meter away, otherwise we would have seen it, even if it was a wall covered in black
	// leather.
	if(!is_narrow) for(int py=1; py<TOF_YS-1; py+=FREEVECT_PIXEL_SKIP)
	{
		int px_start = X_EDGE_IGNORE + 2;
		int px_end = TOF_XS-X_EDGE_IGNORE - 2;

		px_start += px_ignore[py]+2;
		px_end   -= px_ignore[py]+2;

		for(int px=px_start; px<px_end; px+=FREEVECT_PIXEL_SKIP)
		{
			for(int iy=-FREEVECT_PIXEL_AREA; iy<=FREEVECT_PIXEL_AREA; iy++)
			{
				for(int ix=-FREEVECT_PIXEL_AREA; ix<=FREEVECT_PIXEL_AREA; ix++)
				{
					int_fast32_t dist = tss->sets[setnum].ampldist[(py+iy)*TOF_XS+(px+ix)]&DIST_MASK;
					if(dist != DIST_UNDEREXP)
						goto NOT_FREEVECT;
				}
			}

			// Assumed worst-case sensor range:
			int64_t d;
			if(px > 50 && px < TOF_XS-50)
				d = ASSUMED_SENSOR_RANGE*1.0;
			else if(px > 40 && px < TOF_XS-40)
				d = ASSUMED_SENSOR_RANGE*0.9;
			else if(px > 30 && px < TOF_XS-30)
				d = ASSUMED_SENSOR_RANGE*0.75;
			else
				d = ASSUMED_SENSOR_RANGE*0.6;

			if(py > 15 && py < TOF_YS-15)
				d += ASSUMED_SENSOR_RANGE*0.05;

			int16_t h_ang = -sensor_softcals[sidx].hor_angs[py*TOF_XS+px];
			int16_t v_ang = sensor_softcals[sidx].ver_angs[py*TOF_XS+px];

			uint16_t hor_ang, ver_ang;
			hor_ang = ((int32_t)h_ang*cos_sensor_rota - (int32_t)v_ang*sin_sensor_rota)>>SIN_LUT_RESULT_SHIFT;
			ver_ang = ((int32_t)h_ang*sin_sensor_rota + (int32_t)v_ang*cos_sensor_rota)>>SIN_LUT_RESULT_SHIFT;
			uint16_t comb_hor_ang = hor_ang + global_sensor_hor_ang;
			uint16_t comb_ver_ang = ver_ang + global_sensor_ver_ang;

			int32_t x = (((int64_t)d * (int64_t)lut_cos_from_u16(comb_ver_ang) * (int64_t)lut_cos_from_u16(comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + global_sensor_x;
			int32_t y = (((int64_t)d * (int64_t)lut_cos_from_u16(comb_ver_ang) * (int64_t)lut_sin_from_u16(comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + global_sensor_y;
			int32_t z = (((int64_t)d * (int64_t)lut_sin_from_u16(comb_ver_ang))>>SIN_LUT_RESULT_SHIFT) + global_sensor_z;

			cloud_add_freevect(cloud, CL_FREEVECT_MM(src_idx, x, y, z));


			NOT_FREEVECT:;
		}
	}


}


void cloud_to_xyz_file(cloud_t* cloud, char* fname, int use_local_coords)
{
	FILE* f = fopen(fname, "w");
	assert(f);

	if(use_local_coords)
	{
		for(int i=0; i<cloud->m.n_points; i++)
			fprintf(f, "%d %d %d\n", cloud->points[i].x*CLOUD_MM, cloud->points[i].y*CLOUD_MM, cloud->points[i].z*CLOUD_MM);
	}
	else
	{
		for(int i=0; i<cloud->m.n_points; i++)
			fprintf(f, "%" PRIi64 " %" PRIi64 " %" PRIi64"\n", 
				(cloud->m.ref_x+(int64_t)cloud->points[i].x)*CLOUD_MM, 
				(cloud->m.ref_y+(int64_t)cloud->points[i].y)*CLOUD_MM, 
				(cloud->m.ref_z+(int64_t)cloud->points[i].z)*CLOUD_MM);
	}

	fclose(f);
}







