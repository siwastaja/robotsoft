/*
	Point cloud type, transforming, processing and filtration functions for SLAM
*/

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "slam_config.h"
#include "slam_cloud.h"
#include "api_board_to_soft.h"

#include "voxmap.h"
#include "voxmap_memdisk.h"

#define COMPRESS_CLOUDS

#ifdef COMPRESS_CLOUDS
	#include <zlib.h>

	#define ZLIB_CHUNK (256*1024)
	#define ZLIB_LEVEL 2  // from 1 to 9, 9 = slowest but best compression
#endif


void init_cloud(cloud_t* cloud, int flags)
{
	cloud->m.n_srcs = 0;
	cloud->m.n_points = 0;
	cloud->m.n_poses = 0;
	
	if(flags & CLOUD_INIT_SMALL)
	{
		cloud->alloc_srcs = INITIAL_SRCS_ALLOC_SMALL;
		cloud->alloc_points = INITIAL_POINTS_ALLOC_SMALL;
		cloud->alloc_poses = INITIAL_POSES_ALLOC_SMALL;
	}
	else
	{
		cloud->alloc_srcs = INITIAL_SRCS_ALLOC_LARGE;
		cloud->alloc_points = INITIAL_POINTS_ALLOC_LARGE;
		cloud->alloc_poses = INITIAL_POSES_ALLOC_LARGE;
	}

	cloud->srcs = malloc(cloud->alloc_srcs * sizeof (cloud_point_t));
	assert(cloud->srcs);
	cloud->points = malloc(cloud->alloc_points * sizeof (cloud_point_t));
	assert(cloud->points);
	cloud->poses = malloc(cloud->alloc_poses * sizeof (hw_pose_t));
	assert(cloud->poses);
}

// You supply a pointer to an otherwise-uninitialized cloud, but you have
// set the m.n_* fields. This function does the rest of the init: allocates for
// this exact amount of space.
void alloc_cloud(cloud_t* cloud)
{
	cloud->alloc_srcs = cloud->m.n_srcs;
	cloud->alloc_points = cloud->m.n_points;
	cloud->alloc_poses = cloud->m.n_poses;

	cloud->srcs = malloc(cloud->alloc_srcs * sizeof (cloud_point_t));
	assert(cloud->srcs);
	cloud->points = malloc(cloud->alloc_points * sizeof (cloud_point_t));
	assert(cloud->points);
	cloud->poses = malloc(cloud->alloc_poses * sizeof (hw_pose_t));
	assert(cloud->poses);
}

void init_cloud_copy(cloud_t* cloud, const cloud_t* const orig, int flags)
{
	cloud->m = orig->m;
	
	cloud->alloc_srcs = orig->alloc_srcs;
	cloud->alloc_points = orig->alloc_points;
	cloud->alloc_poses = orig->alloc_poses;

	cloud->srcs = malloc(cloud->alloc_srcs * sizeof (cloud_point_t));
	assert(cloud->srcs);
	cloud->points = malloc(cloud->alloc_points * sizeof (cloud_point_t));
	assert(cloud->points);
	cloud->poses = malloc(cloud->alloc_poses * sizeof (hw_pose_t));
	assert(cloud->poses);
}


// translates cloud in and its sources to the reference of out.
// Tries to find matching sources in in, creates new sources if necessary, changes the source indeces.
void cat_cloud(cloud_t * const restrict out, const cloud_t * const restrict in)
{
	uint16_t* src_transl = calloc(in->m.n_srcs, sizeof (uint16_t));
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
		printf("cat_cloud: translating in source %d to out source %d\n", i, newsrc);
	}

	cloud_prepare_fast_add_points(out, in->m.n_points);
	for(int i=0; i<in->m.n_points; i++)
	{
		assert(in->points[i].src_idx < in->m.n_srcs); // Check validity of source indeces in in.
		cloud_fast_add_point(out, CL_SRCP(src_transl[in->points[i].src_idx], in->points[i].x+tx, in->points[i].y+ty, in->points[i].z+tz));
	}
	printf("cat_cloud: added %d points translated by (%d,%d,%d)\n", in->m.n_points, (int)tx, (int)ty, (int)tz);
	free(src_transl);
}

// ... but keep the allocations and meta fields
void cloud_remove_points(cloud_t* cloud)
{
	cloud->m.n_srcs = 0;
	cloud->m.n_points = 0;
	cloud->m.n_poses = 0;
}

int cloud_is_init(cloud_t* cloud)
{
	return (cloud->points != NULL);
}

void free_cloud(cloud_t* cloud)
{
	free(cloud->srcs);
	free(cloud->points);
	free(cloud->poses);

	cloud->srcs = NULL;
	cloud->points = NULL;
	cloud->poses = NULL;

	memset(&cloud->m, 0, sizeof cloud->m);
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

	#ifdef COMPRESS_CLOUDS
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

		strm.avail_in = cloud->m.n_srcs*sizeof *cloud->srcs;
		strm.next_in = (uint8_t*)cloud->srcs;

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

		strm.avail_in = cloud->m.n_points * sizeof *cloud->points;
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

		strm.avail_in = cloud->m.n_poses * sizeof *cloud->poses;
		strm.next_in = (uint8_t*)cloud->poses;

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
		// untested code:
		assert(fwrite(cloud->srcs, sizeof *cloud->srcs[0], cloud->m.n_srcs, f) == cloud->m.n_srcs);
		assert(fwrite(cloud->points, sizeof *cloud->points[0], cloud->m.n_points, f) == cloud->m.n_points);
		assert(fwrite(cloud->poses, sizeof *cloud->poses[0], cloud->m.n_poses, f) == cloud->m.n_poses);
	#endif

	fclose(f);
	return 0;
}

int load_cloud(cloud_t* cloud, int idx)
{
	char fname[1024];
	snprintf(fname, 1024, "submap%06u.bin", idx);
	FILE* f = fopen(fname, "rb");
	assert(f);

	assert(fread(&cloud->m, sizeof cloud->m, 1, f) == 1);

	alloc_cloud(cloud);

	#ifdef COMPRESS_CLOUDS
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
		int bytes_left = cloud->m.n_srcs*sizeof cloud->srcs[0];
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
				strm.next_out = (uint8_t*)cloud->srcs + got_bytes;

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

		got_bytes = 0;
		bytes_left = cloud->m.n_points*sizeof cloud->points[0];
		ret = 0;
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

		got_bytes = 0;
		bytes_left = cloud->m.n_poses*sizeof cloud->poses[0];
		ret = 0;
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
				strm.next_out = (uint8_t*)cloud->poses + got_bytes;

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
		// untested code
		assert(fread(cloud->srcs, sizeof *cloud->srcs[0], cloud->m.n_srcs, f) == cloud->m.n_srcs);
		assert(fread(cloud->points, sizeof *cloud->points[0], cloud->m.n_points, f) == cloud->m.n_points);
		assert(fread(cloud->poses, sizeof *cloud->poses[0], cloud->m.n_poses, f) == cloud->m.n_poses);
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

#define CLOUDFLT_XS 448
#define CLOUDFLT_YS 448
#define CLOUDFLT_ZS 96
#define CLOUDFLT_UNIT 64 //mm

static uint8_t free_cnt[CLOUDFLT_XS][CLOUDFLT_YS][CLOUDFLT_ZS];



#define UNCERT_Z 2
#define UNCERT 2
#define LINE_CONE_SLOPE_XY 1500
#define LINE_CONE_SLOPE_Z  2000

static inline void output_voxel_cloudflt(int x, int y, int z)
{
//	assert(x >= 1 && x < CLOUDFLT_XS-1 && y >= 1 && y < CLOUDFLT_YS-1 && z > 1 && z <= CLOUDFLT_ZS-1);

	if(x >= 1 && x < CLOUDFLT_XS-1 && y >= 1 && y < CLOUDFLT_YS-1 && z > 1 && z <= CLOUDFLT_ZS-1)
		if(free_cnt[x][y][z] < 255)
			free_cnt[x][y][z]++;

//	if(x >= 1 && x < CLOUDFLT_XS-1 && y >= 1 && y < CLOUDFLT_YS-1 && z > 1 && z <= CLOUDFLT_ZS-1)
//		free_cnt[x][y][z] = 1;
}

static inline uint8_t get_voxel_cloudflt(int x, int y, int z)
{
//	assert(x >= 0 && x < CLOUDFLT_XS && y >= 0 && y < CLOUDFLT_YS && z > 0 && z <= CLOUDFLT_ZS);
	if(! (x >= 0 && x < CLOUDFLT_XS && y >= 0 && y < CLOUDFLT_YS && z > 0 && z <= CLOUDFLT_ZS))
		return 0;

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

/*
free_cnt is a voxel map where value >0 means that this voxel has been seen empty
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

static void de_edge_free_cnt()
{
	// along z axis, up and down
	for(int iy=0; iy < CLOUDFLT_YS; iy++)
	{
		for(int ix=0; ix < CLOUDFLT_XS; ix++)
		{
			for(int iz=2; iz < CLOUDFLT_ZS-1; iz++)
			{
				if(!free_cnt[ix][iy][iz-2] && !free_cnt[ix][iy][iz-1] && free_cnt[ix][iy][iz])
				{
					free_cnt[ix][iy][iz] = 0;
					#ifdef DE_EDGE_2
					free_cnt[ix][iy][iz+1] = 0;
					iz++;
					#endif
					iz+=2; // skip so that the point we just removed does not cause continuous removal of everything
				}
			}

			for(int iz=CLOUDFLT_ZS-2-1; iz >= 1; iz--)
			{
				if(!free_cnt[ix][iy][iz+2] && !free_cnt[ix][iy][iz+1] && free_cnt[ix][iy][iz])
				{
					free_cnt[ix][iy][iz] = 0;
					#ifdef DE_EDGE_2
					free_cnt[ix][iy][iz-1] = 0;
					iz--;
					#endif
					iz-=2;
				}
			}

		}
	}

	// along x axis
	for(int iy=0; iy < CLOUDFLT_YS; iy++)
	{
		for(int iz=0; iz < CLOUDFLT_ZS; iz++)
		{
			for(int ix=2; ix < CLOUDFLT_XS-1; ix++)
			{
				if(!free_cnt[ix-2][iy][iz] && !free_cnt[ix-1][iy][iz] && free_cnt[ix][iy][iz])
				{
					free_cnt[ix][iy][iz] = 0;
					#ifdef DE_EDGE_2
					free_cnt[ix+1][iy][iz] = 0;
					ix++;
					#endif
					ix+=3;
				}
			}

			for(int ix=CLOUDFLT_XS-2-1; ix >= 1; ix--)
			{
				if(!free_cnt[ix+2][iy][iz] && !free_cnt[ix+1][iy][iz] && free_cnt[ix][iy][iz])
				{
					free_cnt[ix][iy][iz] = 0;
					#ifdef DE_EDGE_2
					free_cnt[ix-1][iy][iz] = 0;
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
				if(!free_cnt[ix][iy-2][iz] && !free_cnt[ix][iy-1][iz] && free_cnt[ix][iy][iz])
				{
					free_cnt[ix][iy][iz] = 0;
					#ifdef DE_EDGE_2
					free_cnt[ix][iy+1][iz] = 0;
					iy++;
					#endif
					iy+=3; // skip so that the point we just removed does not cause continuous removal of everything
				}
			}

			for(int iy=CLOUDFLT_YS-2-1; iy >= 1; iy--)
			{
				if(!free_cnt[ix][iy+2][iz] && !free_cnt[ix][iy+1][iz] && free_cnt[ix][iy][iz])
				{
					free_cnt[ix][iy][iz] = 0;
					#ifdef DE_EDGE_2
					free_cnt[ix][iy-1][iz] = 0;
					iy--;
					#endif
					iy-=3;
				}
			}

		}
	}


}

#if 0
void filter_cloud(cloud_t* cloud, cloud_t* out, int32_t transl_x, int32_t transl_y, int32_t transl_z)
{
//	printf("filter_cloud ref (%d, %d, %d)\n", ref_x, ref_y, ref_z);
	memset(free_cnt, 0, sizeof(free_cnt));

	// Trace empty space:
	for(int p=0; p<cloud->m.n_points; p++)
	{
		int sx = (cloud->points[p].sx + transl_x)/CLOUDFLT_UNIT + CLOUDFLT_XS/2;
		int sy = (cloud->points[p].sy + transl_y)/CLOUDFLT_UNIT + CLOUDFLT_YS/2;
		int sz = (cloud->points[p].sz + transl_z)/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

		int px = (cloud->points[p].x + transl_x)/CLOUDFLT_UNIT + CLOUDFLT_XS/2;
		int py = (cloud->points[p].y + transl_y)/CLOUDFLT_UNIT + CLOUDFLT_YS/2;
		int pz = (cloud->points[p].z + transl_z)/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

		//printf("p=%d, (%d,%d,%d)->(%d,%d,%d)\n", sx,sy,sz, px,py,pz);
		bresenham3d_cloudflt(sx, sy, sz, px, py, pz);
	}

	de_edge_free_cnt();
	out->m.n_points = 0;
	int n_p = 0;
	// Remove points at empty space by only outputting points where there is no free voxel.
	for(int p=0; p<cloud->m.n_points; p++)
	{
		int px = (cloud->points[p].px + transl_x)/CLOUDFLT_UNIT + CLOUDFLT_XS/2;
		int py = (cloud->points[p].py + transl_y)/CLOUDFLT_UNIT + CLOUDFLT_YS/2;
		int pz = (cloud->points[p].pz + transl_z)/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

		int val = get_voxel_cloudflt(px, py, pz);
		if(val < 10)
		{
			out->points[n_p] = TRANSLATE_CLOUD_POINT(cloud->points[p], transl_x, transl_y, transl_z);
			n_p++;
		}
			
	}
	out->m.n_points = n_p;

}

/*
	Use the same free space buffer as filter_cloud, but only generate said buffer; don't output any points.
*/
void generate_cloudflt_free(cloud_t* cloud, int32_t transl_x, int32_t transl_y, int32_t transl_z)
{
//	printf("filter_cloud ref (%d, %d, %d)\n", ref_x, ref_y, ref_z);
	memset(free_cnt, 0, sizeof(free_cnt));

	// Trace empty space:
	for(int p=0; p<cloud->m.n_points; p++)
	{
		int sx = (cloud->points[p].sx + transl_x)/CLOUDFLT_UNIT + CLOUDFLT_XS/2;
		int sy = (cloud->points[p].sy + transl_y)/CLOUDFLT_UNIT + CLOUDFLT_YS/2;
		int sz = (cloud->points[p].sz + transl_z)/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

		int px = (cloud->points[p].px + transl_x)/CLOUDFLT_UNIT + CLOUDFLT_XS/2;
		int py = (cloud->points[p].py + transl_y)/CLOUDFLT_UNIT + CLOUDFLT_YS/2;
		int pz = (cloud->points[p].pz + transl_z)/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

		//printf("p=%d, (%d,%d,%d)->(%d,%d,%d)\n", sx,sy,sz, px,py,pz);
		bresenham3d_cloudflt(sx, sy, sz, px, py, pz);
	}

	// Remove free space next to the points
	for(int p=0; p<cloud->m.n_points; p++)
	{
		int px = (cloud->points[p].px + transl_x)/CLOUDFLT_UNIT + CLOUDFLT_XS/2;
		int py = (cloud->points[p].py + transl_y)/CLOUDFLT_UNIT + CLOUDFLT_YS/2;
		int pz = (cloud->points[p].pz + transl_z)/CLOUDFLT_UNIT + CLOUDFLT_ZS/2;

		//printf("p=%d, (%d,%d,%d)->(%d,%d,%d)\n", sx,sy,sz, px,py,pz);

		if(px >= 2 && px < CLOUDFLT_XS-2 &&
		   py >= 2 && py < CLOUDFLT_YS-2 &&
		   pz >= 2 && pz < CLOUDFLT_ZS-2)
		{
			for(int ix=-2; ix<=+2; ix++)
				for(int iy=-2; iy<=+2; iy++)
					for(int iz=-2; iz<=+2; iz++)
						free_cnt[px+ix][py+iy][pz+iz] = 0;
		}
	}

}
#endif

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

#if 0
void voxfilter_to_cloud(voxfilter_t* voxfilter, cloud_t* cloud)
{
	int insert_cnt = 0, uniq_insert_cnt = 0;
	for(int xx=0; xx<VOXFILTER_XS; xx++)
	{
		for(int yy=0; yy<VOXFILTER_YS; yy++)
		{
			for(int zz=0; zz<VOXFILTER_ZS; zz++)
			{
				voxfilter_point_t* p = &(voxfilter->points[xx][yy][zz]);

				if(p->cnt == 0)
				{
					continue;
				}

				// Rough coordinates based on the voxel edges:
				int_fast32_t x = ((xx - VOXFILTER_XS/2) * VOXFILTER_STEP) - voxfilter->m.ref_x;
				int_fast32_t y = ((yy - VOXFILTER_YS/2) * VOXFILTER_STEP) - voxfilter->m.ref_y;
				int_fast32_t z = ((yy - VOXFILTER_YS/2) * VOXFILTER_STEP) - voxfilter->m.ref_z;

				// Average location within the voxel:
				if(2*(int)p->x >= (int)p->cnt) x++;
				if(2*(int)p->y >= (int)p->cnt) y++;
				if(2*(int)p->z >= (int)p->cnt) z++;

				// Insert the same point for each source
				// (Point cloud datatype doesn't have another mean to express
				// that multiple sources see the same point)

				for(int i=0; i<VOXFILTER_MAX_RAY_SOURCES; i++)
				{
					if(p->src_idxs[i] == 0)
						break;

					cloud_add_point(cloud, CL_SRCP(p->src_idxs[i], x,y,z));
					insert_cnt++;
				}
				uniq_insert_cnt++;
			}
		}
	}

	printf("INFO: voxfilter_to_cloud inserted %d points, %d of which are duplicates due to multiple sources\n", insert_cnt, insert_cnt-uniq_insert_cnt);
}

int warn_voxfilter_ignored_source;
int warn_voxfilter_ignored_point;


ALWAYS_INLINE void voxfilter_insert_point(cloud_t* cloud, voxfilter_t* voxfilter, cloud_point_t p)
{
	int vox_x = (p.x+voxfilter->m.ref_x)/VOXFILTER_STEP + VOXFILTER_XS/2;
	int vox_y = (p.y+voxfilter->m.ref_y)/VOXFILTER_STEP + VOXFILTER_YS/2;
	int vox_z = (p.z+voxfilter->m.ref_z)/VOXFILTER_STEP + VOXFILTER_ZS/2;

	if(vox_x < 0 || vox_x > VOXFILTER_XS-1 || vox_y < 0 || vox_y > VOXFILTER_YS-1 || vox_z < 0 || vox_z > VOXFILTER_ZS-1)
	{
//		printf("INFO: voxfilter: skipping OOR point %d, %d, %d\n", vox_x, vox_y, vox_z);
		cloud_add_point(cloud, p);
		return;
	}

	assert(p.src_idx > 0);


	// If the source index doesn't exist on this voxel, add it.

	for(int i = 0; i < VOXFILTER_MAX_RAY_SOURCES; i++)
	{
		if(voxfilter->points[vox_x][vox_y][vox_z].src_idxs[i] == p.src_idx)
			goto SOURCE_EXISTS;

		if(voxfilter->points[vox_x][vox_y][vox_z].src_idxs[i] == 0)
		{
			// Zero terminator found: did not find the srcid, and this is a suitable place for adding it.
			voxfilter->points[vox_x][vox_y][vox_z].src_idxs[i] = p.src_idx;
			goto SOURCE_EXISTS; // now it's there
		}
	}

	//printf("WARN: voxfilter - no space left to add source, not adding source\n");
	// Did not find the source, nor had space to add it. Still insert the point to the voxfilter - just without source.
	// The consequence is, while the voxfilter still limits number of points, cannot trace all free space.
	// Do nothing here. Count this.

	warn_voxfilter_ignored_source++;

	SOURCE_EXISTS:;	

	// Accumulate the point.
	// The point coordinate higher bits are implicitly available as points[][][] indices.
	// Accumulate the LSb only.

	if(LIKELY(voxfilter->points[vox_x][vox_y][vox_z].cnt < 255)) // If the count is full - unlikely - just ignore the point completely.
	{
		voxfilter->points[vox_x][vox_y][vox_z].cnt++;
		voxfilter->points[vox_x][vox_y][vox_z].x += p.x&1;
		voxfilter->points[vox_x][vox_y][vox_z].y += p.y&1;
		voxfilter->points[vox_x][vox_y][vox_z].z += p.z&1;
	}
	else
	{
		warn_voxfilter_ignored_point++;
	}

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

#define SUBPIX
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

	uint16_t global_sensor_ver_ang = 
		(int32_t)((int16_t)sensor_softcals[sidx].mount.vert_ang_rel_ground) +
		((lut_cos_from_u16(sensor_softcals[sidx].mount.ang_rel_robot)*pitch_ang)>>SIN_LUT_RESULT_SHIFT) +
		((lut_sin_from_u16(sensor_softcals[sidx].mount.ang_rel_robot)*roll_ang)>>SIN_LUT_RESULT_SHIFT);

	uint16_t local_sensor_hor_ang = sensor_softcals[sidx].mount.ang_rel_robot;
	uint16_t local_sensor_ver_ang = sensor_softcals[sidx].mount.vert_ang_rel_ground;

	int32_t  global_sensor_x = robot_x - ref_x +
			((lut_cos_from_u16(robot_ang)*sensor_softcals[sidx].mount.x_rel_robot)>>SIN_LUT_RESULT_SHIFT) +
			((lut_sin_from_u16(robot_ang)*-1*sensor_softcals[sidx].mount.y_rel_robot)>>SIN_LUT_RESULT_SHIFT);

	int32_t  global_sensor_y = robot_y - ref_y + 
			((lut_sin_from_u16(robot_ang)*sensor_softcals[sidx].mount.x_rel_robot)>>SIN_LUT_RESULT_SHIFT) +
			((lut_cos_from_u16(robot_ang)*sensor_softcals[sidx].mount.y_rel_robot)>>SIN_LUT_RESULT_SHIFT);

	int32_t  global_sensor_z = robot_z - ref_z + sensor_softcals[sidx].mount.z_rel_ground;


	int32_t  local_sensor_x = sensor_softcals[sidx].mount.x_rel_robot;
	int32_t  local_sensor_y = sensor_softcals[sidx].mount.y_rel_robot;
	int32_t  local_sensor_z = sensor_softcals[sidx].mount.z_rel_ground;


	int sx = global_sensor_x;
	int sy = global_sensor_y;
	int sz = global_sensor_z;

	// cloud_find_source reuses an old source if close enough, or creates a new one:
	int src_idx = cloud_find_source(cloud, CL_P(sx, sy, sz));

	// Kludgy code to ignore the corners and top/bottom edges
	for(int py=5; py<TOF_YS-5; py++)
	{
		int px_start = 10;
		int px_end = TOF_XS-10;

		if(py < 4 || py > TOF_YS-4)
		{
			px_start += 9;
			px_end -= 9;
		}
		if(py < 8 || py > TOF_YS-8)
		{
			px_start += 8;
			px_end -= 8;
		}
		if(py < 12 || py > TOF_YS-12)
		{
			px_start += 6;
			px_end -= 6;
		}
		if(py < 16 || py > TOF_YS-16)
		{
			px_start += 4;
			px_end -= 4;
		}
		if(py < 20 || py > TOF_YS-20)
		{
			px_start += 3;
			px_end -= 3;
		}
		if(py < 24 || py > TOF_YS-24)
		{
			px_start += 2;
			px_end -= 2;
		}

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

				//refdist <<= DIST_SHIFT;  // if you ever need refdist, convert to mm first

				int32_t d = avg;

				uint16_t hor_ang, ver_ang;

				hor_ang = -sensor_softcals[sidx].hor_angs[py*TOF_XS+px];
				ver_ang = sensor_softcals[sidx].ver_angs[py*TOF_XS+px];

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

				#ifdef VACUUM_APP
					// VACUUM APP: Ignore the nozzle
					#define NOZZLE_WIDTH 760
					if(local_z < 200 && local_x < 520 && local_x > 120 && local_y > -(NOZZLE_WIDTH/2) && local_y < (NOZZLE_WIDTH/2))
						continue;

					// Completely ignore nozzle area obstacles for mapping, but give the floor if visible!
					if(local_z > 100 && local_x < 520 && local_x > 120 && local_y > -(NOZZLE_WIDTH/2) && local_y < (NOZZLE_WIDTH/2))
						continue;
				#endif

//				if(z > 2200)
//					continue;

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

			}
		}
	}

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







