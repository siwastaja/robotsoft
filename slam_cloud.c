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



/*
	50 typical submaps = 
	400MB, uncompressed
	147MB, compressed at ZLIB_LEVEL=2
*/

#define COMPRESS_CLOUDS

#ifdef COMPRESS_CLOUDS
	#include <zlib.h>

	#define ZLIB_CHUNK (256*1024)
	#define ZLIB_LEVEL 2  // from 1 to 9, 9 = slowest but best compression
#endif

int save_cloud(cloud_t* cloud, int idx)
{
	char fname[1024];
	snprintf(fname, 1024, "submap%06u.bin", idx);
	FILE* f = fopen(fname, "wb");
	assert(f);

	uint32_t n_points = cloud->n_points;
	assert(fwrite(&n_points, sizeof(uint32_t), 1, f) == 1);

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
		strm.avail_in = n_points*sizeof(cloud_point_t);
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
		assert(fwrite(cloud->points, sizeof(cloud_point_t), n_points, f) == n_points);
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

	uint32_t n_points = 0;
	assert(fread(&n_points, sizeof(uint32_t), 1, f) == 1);
	cloud->n_points = n_points;

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
		int bytes_left = n_points*sizeof(cloud_point_t);
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
		assert(fread(cloud->points, sizeof(cloud_point_t), n_points, f) == n_points);
	#endif

	fclose(f);
	return 0;
}

#if 1
// These DO NOT transform the ray sources
void transform_cloud(cloud_t* cloud, int32_t transl_x, int32_t transl_y, int32_t transl_z, double yaw)
{
	double cosa = cos(yaw);
	double sina = sin(yaw);

	for(int p=0; p<cloud->n_points; p++)
	{
		double x_in = cloud->points[p].px;
		double y_in = cloud->points[p].py;
		double z_in = cloud->points[p].pz;

		double x = x_in*cosa - y_in*sina + transl_x;
		double y = x_in*sina + y_in*cosa + transl_y;
		double z = z_in + transl_z;

		cloud->points[p].px = x;
		cloud->points[p].py = y;
		cloud->points[p].pz = z;
	}
}

void transform_cloud_copy(cloud_t* cloud_in, cloud_t* cloud_out, int32_t transl_x, int32_t transl_y, int32_t transl_z, double yaw)
{
	double cosa = cos(yaw);
	double sina = sin(yaw);

	cloud_out->n_points = cloud_in->n_points;
	for(int p=0; p<cloud_in->n_points; p++)
	{
		double x_in = cloud_in->points[p].px;
		double y_in = cloud_in->points[p].py;
		double z_in = cloud_in->points[p].pz;

		double x = x_in*cosa - y_in*sina + transl_x;
		double y = x_in*sina + y_in*cosa + transl_y;
		double z = z_in + transl_z;

		cloud_out->points[p].px = x;
		cloud_out->points[p].py = y;
		cloud_out->points[p].pz = z;
	}
}
#endif

void rotate_cloud(cloud_t* cloud, double yaw)
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

void rotate_cloud_copy(cloud_t* cloud, cloud_t* out, double yaw)
{
	double cosa = cos(yaw);
	double sina = sin(yaw);

	for(int p=0; p<cloud->n_points; p++)
	{
		double px_in = cloud->points[p].px;
		double py_in = cloud->points[p].py;

		double px = px_in*cosa - py_in*sina;
		double py = px_in*sina + py_in*cosa;

		out->points[p].px = px;
		out->points[p].py = py;
		out->points[p].pz = cloud->points[p].pz;

		double sx_in = cloud->points[p].sx;
		double sy_in = cloud->points[p].sy;

		double sx = sx_in*cosa - sy_in*sina;
		double sy = sx_in*sina + sy_in*cosa;

		out->points[p].sx = sx;
		out->points[p].sy = sy;
		out->points[p].sz = cloud->points[p].sz;

	}
	out->n_points = cloud->n_points;
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

void filter_cloud(cloud_t* cloud, cloud_t* out, int32_t transl_x, int32_t transl_y, int32_t transl_z)
{
//	printf("filter_cloud ref (%d, %d, %d)\n", ref_x, ref_y, ref_z);
	memset(free_cnt, 0, sizeof(free_cnt));

	// Trace empty space:
	for(int p=0; p<cloud->n_points; p++)
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

	de_edge_free_cnt();
	out->n_points = 0;
	int n_p = 0;
	// Remove points at empty space by only outputting points where there is no free voxel.
	for(int p=0; p<cloud->n_points; p++)
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
	out->n_points = n_p;

}

/*
	Use the same free space buffer as filter_cloud, but only generate said buffer; don't output any points.
*/
void generate_cloudflt_free(cloud_t* cloud, int32_t transl_x, int32_t transl_y, int32_t transl_z)
{
//	printf("filter_cloud ref (%d, %d, %d)\n", ref_x, ref_y, ref_z);
	memset(free_cnt, 0, sizeof(free_cnt));

	// Trace empty space:
	for(int p=0; p<cloud->n_points; p++)
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
	for(int p=0; p<cloud->n_points; p++)
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


//#define CLOUD_TO_VOXMAP_TRACE_FREE
void cloud_to_voxmap(cloud_t* cloud, int ref_x, int ref_y, int ref_z)
{
	printf("cloud_to_voxmap .."); fflush(stdout);
	#ifdef CLOUD_TO_VOXMAP_TRACE_FREE
		generate_cloudflt_free(cloud, 0, 0, 0);

		printf(" generated free .."); fflush(stdout);

	#endif
	for(int i=0; i<cloud->n_points; i++)
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

				int32_t x = p->x / p->cnt;
				int32_t y = p->y / p->cnt;
				int32_t z = p->z / p->cnt;

				// Insert the same point for each source
				// (Point cloud datatype doesn't have another mean to express
				// that multiple sources see the same point)

				for(int i=0; i<VOXFILTER_MAX_RAY_SOURCES; i++)
				{
					if(p->src_idxs[i] == 0)
						break;

					assert(p->src_idxs[i] > 0 && p->src_idxs[i] <= voxfilter->n_ray_sources);

					cloud_insert_point(cloud, 
						voxfilter->ray_sources[p->src_idxs[i]].x, 
						voxfilter->ray_sources[p->src_idxs[i]].y,
						voxfilter->ray_sources[p->src_idxs[i]].z, 
						x, y, z);
					insert_cnt++;
				}
				uniq_insert_cnt++;
			}
		}
	}

	//printf("INFO: voxfilter_to_cloud inserted %d points, %d of which are duplicates due to multiple sources\n", insert_cnt, insert_cnt-uniq_insert_cnt);
}


ALWAYS_INLINE void voxfilter_insert_point(cloud_t* cloud, voxfilter_t* voxfilter, int srcid, 
	int32_t sx, int32_t sy, int32_t sz, // source coordinates are only used when the point cannot go to voxfilter; otherwise, srcid is stored
	int32_t x, int32_t y, int32_t z, // the point
	int32_t ref_x, int32_t ref_y, int32_t ref_z) // Extra translation, so that the voxfilter can live in its own limited space instead of the larger submap span.
	// Because only voxel selection is translated, not the actual stored coordinates (they fit int32_t accumulation variables just fine),
	// there is no need to translate points back later, basically we just store at a certain offset to maximize the span.
{
	int vox_x = (x+ref_x)/VOXFILTER_STEP + VOXFILTER_XS/2;
	int vox_y = (y+ref_y)/VOXFILTER_STEP + VOXFILTER_YS/2;
	int vox_z = (z+ref_z)/VOXFILTER_STEP + VOXFILTER_ZS/2;

	assert(srcid > 0 && srcid <= voxfilter->n_ray_sources);

	if(vox_x < 0 || vox_x > VOXFILTER_XS-1 || vox_y < 0 || vox_y > VOXFILTER_YS-1 || vox_z < 0 || vox_z > VOXFILTER_ZS-1)
	{
//		printf("INFO: voxfilter: skipping OOR point %d, %d, %d\n", vox_x, vox_y, vox_z);
		cloud_insert_point(cloud, sx, sy, sz, x, y, z);
		return;
	}

	// If the source index doesn't exist on this voxel, add it.

	for(int i = 0; i < VOXFILTER_MAX_RAY_SOURCES; i++)
	{
		if(voxfilter->points[vox_x][vox_y][vox_z].src_idxs[i] == srcid)
			goto SOURCE_EXISTS;

		if(voxfilter->points[vox_x][vox_y][vox_z].src_idxs[i] == 0)
		{
			// Zero terminator found: did not find the srcid, and this is a suitable place for adding it.
			voxfilter->points[vox_x][vox_y][vox_z].src_idxs[i] = srcid;
			goto SOURCE_EXISTS; // now it's there
		}
	}

	//printf("WARN: voxfilter - no space left to add source, skipping filter\n");
	// Did not find the source, nor had space to add it. Just insert the point to the cloud, bypassing the filter.	
	cloud_insert_point(cloud, sx, sy, sz, x, y, z);
	return;

	SOURCE_EXISTS:;	

	// Accumulate the point

	voxfilter->points[vox_x][vox_y][vox_z].cnt++;
	voxfilter->points[vox_x][vox_y][vox_z].x += x;
	voxfilter->points[vox_x][vox_y][vox_z].y += y;
	voxfilter->points[vox_x][vox_y][vox_z].z += z;

	//printf("point (%d,%d,%d), voxel (%d,%d,%d), cnt now = %d\n", x, y, z, vox_x, vox_y, vox_z, voxfilter->points[vox_x][vox_y][vox_z].cnt);

}



#define DIST_UNDEREXP 0
#define DIST_OVEREXP 1
#include "sin_lut.c"

#ifdef REV2A

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



	#define DEGTOANG16(x)  ((uint16_t)((float)(x)/(360.0)*65536.0))

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

	#include "geotables.h"

	// voxfilter_ref_*: difference between the submap origin, and the subsubmap (voxfilter) origin, to maximize the span of the limited voxmap.
	// E.g., submap is started at ref_* = 10000,0,0. First voxfilter is at voxfilter_ref_* = 0,0,0, so at the same location.
	// Now the first voxfilter ends, and the robot is at, say 11000,0,0, and we would have already lost half of our voxfilter span available. But
	// luckily, we can now set voxfilter_ref_* = 1000,0,0, and the new data is again translated near the middle of the voxfilter, internally.
	// Actual coordinates in the voxfilter accumulation variables are not translated, only voxel selection code uses this information.

	// Old version to deal with the legacy not individually calibrated data:

	void tof_to_voxfilter_and_cloud(int is_narrow, uint16_t* ampldist, hw_pose_t pose, int sidx, int32_t ref_x, int32_t ref_y, int32_t ref_z,
		 voxfilter_t* voxfilter, int32_t voxfilter_ref_x, int32_t voxfilter_ref_y, int32_t voxfilter_ref_z, cloud_t* cloud, int voxfilter_threshold, int dist_ignore_threshold,
		 realtime_cloud_t* realtime, int rt_flag)

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

		int voxfilter_ray_src_id = 0;

		if(voxfilter)
		{
			for(int i=1; i < voxfilter->n_ray_sources+1; i++)
			{
				int64_t sqdist = 
					sq((int64_t)voxfilter->ray_sources[i].x - (int64_t)sx)	+
					sq((int64_t)voxfilter->ray_sources[i].y - (int64_t)sy)	+
					sq((int64_t)voxfilter->ray_sources[i].z - (int64_t)sz);

				if(sqdist < sq(VOXFILTER_SOURCE_COMBINE_THRESHOLD))
				{
					// We already have a close enough source, let's use it instead:
					//printf("Hooray, could optimize source (%d,%d,%d) to earlier source #%d (%d,%d,%d)\n",
					//	sx, sy, sz, i, voxfilter->ray_sources[i].x, voxfilter->ray_sources[i].y, voxfilter->ray_sources[i].z);
					voxfilter_ray_src_id = i;
					// But do not replace sx,sy,sz, we are OK with the more exact actual coordinates
					// when working outside the voxfilter.
					goto USE_OLD;
				}
			}

			// No appropriate source was found; add a new one.

	 		voxfilter->n_ray_sources++; // Increment first, we want the first one to be at [1], and so on.

			assert(voxfilter->n_ray_sources < VOXFILTER_N_SCANS*N_SENSORS+1);

			voxfilter->ray_sources[voxfilter->n_ray_sources].x = sx;
			voxfilter->ray_sources[voxfilter->n_ray_sources].y = sy;
			voxfilter->ray_sources[voxfilter->n_ray_sources].z = sz;
			voxfilter_ray_src_id = voxfilter->n_ray_sources;

			USE_OLD:;
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
							if(dist != DIST_UNDEREXP && dist != DIST_OVEREXP && dist > refdist-(170>>DIST_SHIFT) && dist < refdist+(170>>DIST_SHIFT))
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
							if(dist != DIST_UNDEREXP && dist != DIST_OVEREXP && dist > refdist-(170>>DIST_SHIFT) && dist < refdist+(170>>DIST_SHIFT))
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
						voxfilter_insert_point(cloud, voxfilter, voxfilter_ray_src_id, sx, sy, sz, x, y, z, voxfilter_ref_x, voxfilter_ref_y, voxfilter_ref_z);
					else
						cloud_insert_point(cloud, sx, sy, sz, x, y, z);

				}
			}
		}
	}
#endif

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


void tof_to_voxfilter_and_cloud(int is_narrow, uint16_t* ampldist, hw_pose_t pose, int sidx, int32_t ref_x, int32_t ref_y, int32_t ref_z,
	 voxfilter_t* voxfilter, int32_t voxfilter_ref_x, int32_t voxfilter_ref_y, int32_t voxfilter_ref_z, cloud_t* cloud, int voxfilter_threshold, int dist_ignore_threshold,
	 realtime_cloud_t* realtime, int rt_flag)
{
	assert(sidx >= 0 && sidx < N_SENSORS);

	int32_t robot_x = pose.x;
	int32_t robot_y = pose.y;
	int32_t robot_z = pose.z;

	uint16_t robot_ang = pose.ang>>16;

	// Rotation: xr = x*cos(a) - y*sin(a)
	//           yr = x*sin(a) + y*cos(a)

	uint16_t global_sensor_hor_ang = sensor_softcals[sidx].mount.ang_rel_robot + robot_ang;
//	uint16_t global_sensor_ver_ang = sensor_softcals[sidx].mount.vert_ang_rel_ground;

	int16_t pitch_ang = pose.pitch>>16;
	int16_t roll_ang = pose.roll>>16;

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

	int voxfilter_ray_src_id = 0;

	if(voxfilter)
	{
		for(int i=1; i < voxfilter->n_ray_sources+1; i++)
		{
			int64_t sqdist = 
				sq((int64_t)voxfilter->ray_sources[i].x - (int64_t)sx)	+
				sq((int64_t)voxfilter->ray_sources[i].y - (int64_t)sy)	+
				sq((int64_t)voxfilter->ray_sources[i].z - (int64_t)sz);

			if(sqdist < sq(VOXFILTER_SOURCE_COMBINE_THRESHOLD))
			{
				// We already have a close enough source, let's use it instead:
				//printf("Hooray, could optimize source (%d,%d,%d) to earlier source #%d (%d,%d,%d)\n",
				//	sx, sy, sz, i, voxfilter->ray_sources[i].x, voxfilter->ray_sources[i].y, voxfilter->ray_sources[i].z);
				voxfilter_ray_src_id = i;
				// But do not replace sx,sy,sz, we are OK with the more exact actual coordinates
				// when working outside the voxfilter.
				goto USE_OLD;
			}
		}

		// No appropriate source was found; add a new one.

 		voxfilter->n_ray_sources++; // Increment first, we want the first one to be at [1], and so on.

		assert(voxfilter->n_ray_sources < VOXFILTER_N_SCANS*N_SENSORS+1);

		voxfilter->ray_sources[voxfilter->n_ray_sources].x = sx;
		voxfilter->ray_sources[voxfilter->n_ray_sources].y = sy;
		voxfilter->ray_sources[voxfilter->n_ray_sources].z = sz;
		voxfilter_ray_src_id = voxfilter->n_ray_sources;

		USE_OLD:;
	}


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
			int32_t avg = 0;
			int n_conform = 0;
			if(is_narrow)
			{
				int npy, npx;
				npx=px-TOF_NARROW_X_START;
				npy=py-TOF_NARROW_Y_START;
				if(npx < 1 || npy < 1 || npx >= TOF_XS_NARROW-1 || npy >= TOF_YS_NARROW-1)
					continue;

				int32_t refdist = ampldist[(npy+0)*TOF_XS_NARROW+(npx+0)]&DIST_MASK;
				if(refdist == DIST_UNDEREXP)
					continue;

				// At refdist = zero-ish, accept everything +/-130mm
				// At refdist = 5m, accept +/-286mm
				// At refdist = 10m, accept +/- 442mm
				// This gets angular walls through.
				for(int iy=-1; iy<=1; iy++)
				{
					for(int ix=-1; ix<=1; ix++)
					{
						int32_t dist = ampldist[(npy+iy)*TOF_XS_NARROW+(npx+ix)]&DIST_MASK;
						if(dist != DIST_UNDEREXP && dist != DIST_OVEREXP && dist > refdist-(((refdist<<DIST_SHIFT)/32+130)>>DIST_SHIFT) && dist < refdist+(((refdist<<DIST_SHIFT)/32+130)>>DIST_SHIFT))
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
						if(dist != DIST_UNDEREXP && dist != DIST_OVEREXP && dist > refdist-(((refdist<<DIST_SHIFT)/32+130)>>DIST_SHIFT) && dist < refdist+(((refdist<<DIST_SHIFT)/32+130)>>DIST_SHIFT))
						{
							avg+=dist;
							n_conform++;
						}
					
					}
				}
			}

			if(n_conform >= 6)
			{
				avg <<= DIST_SHIFT;
				avg /= n_conform;

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

				if(realtime)
				{
					assert(realtime->n_points < MAX_REALTIME_N_POINTS);
					small_cloud_t new_point = set_small_cloud(rt_flag, sx, sy, sz, x, y, z);
					realtime->points[realtime->n_points] = new_point;
					realtime->n_points++;
				}

				if(d < dist_ignore_threshold)
					continue;

				if(voxfilter && d < voxfilter_threshold)
					voxfilter_insert_point(cloud, voxfilter, voxfilter_ray_src_id, sx, sy, sz, x, y, z, voxfilter_ref_x, voxfilter_ref_y, voxfilter_ref_z);
				else
					cloud_insert_point(cloud, sx, sy, sz, x, y, z);


			}
		}
	}
}


void cloud_to_xyz_file(cloud_t* cloud, char* fname)
{
	FILE* f = fopen(fname, "w");
	assert(f);
	for(int i=0; i<cloud->n_points; i++)
	{
		fprintf(f, "%d %d %d\n", cloud->points[i].px, cloud->points[i].py, cloud->points[i].pz);
	}
	fclose(f);
}







