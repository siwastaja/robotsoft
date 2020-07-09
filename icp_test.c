#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "slam_cloud.h"

#define CML_PRIVATE
#define CML_NO_DEPENDENCIES
#include "cml.h"

/*
	Iterative closest point algorithm
	cloud_b is first transformed by x_corr, y_corr, z_corr, yaw_corr
	Closest points within threshold are associated, and transform between them calculated
*/


typedef struct
{
	double x;
	double y;
	double z;
} doublepoint_t;


#define ICP_BLOCK_XS  16
#define ICP_BLOCK_YS  16
#define ICP_BLOCK_ZS  16


#define ICP_N_SEARCH_BLOCKS_X 1
#define ICP_N_SEARCH_BLOCKS_Y 1
#define ICP_N_SEARCH_BLOCKS_Z 1

// Initially, ICP blocks have no allocated memory, and zero points.
// When the first point belonging to a block is found, memory is allocated
// for ICP_BLOCK_INIT_ALLOC_POINTS points at once. Whenever the memory runs
// out, the block is realloc()'d with double the points, each time.
#define ICP_BLOCK_INIT_ALLOC_POINTS 16

typedef uint16_t icp_point_t;

typedef struct
{
	int n_points;
	int n_alloc;
	icp_point_t* points;
} icp_block_t;

#define N_ICP_BLOCKS_X 256
#define N_ICP_BLOCKS_Y 256
#define N_ICP_BLOCKS_Z 64

icp_block_t

static int match_by_closest_points(cloud_t* cloud_a, cloud_t* cloud_b_in,
	double x_corr, double y_corr, double z_corr, double yaw_corr,
	int threshold,
	double* x_corr_out, double* y_corr_out, double* z_corr_out, double* yaw_corr_out)
{
	float mat_rota[9];
	float mat_transl[3];

	int sqthresh = sq(threshold);


	for(int iter=0; iter<30; iter++)
	{
		doublepoint_t mean_a = {0,0,0};
		doublepoint_t mean_b = {0,0,0};

		static cloud_t cloud_b;

		transform_cloud_copy(cloud_b_in, &cloud_b, x_corr, y_corr, z_corr, yaw_corr);

		int n_associations = 0;
		for(int pa=0; pa < cloud_a->n_points; pa+=11)
		{
//			printf("pa = %d            \r", pa); fflush(stdout);
			int nearest_idx = -1;
			int min_sqdist = 999999999;

			for(int pb=0; pb < cloud_b.n_points; pb+=11)
			{
				int sqdist = sq(cloud_a->points[pa].px - cloud_b.points[pb].px) +
					sq(cloud_a->points[pa].py - cloud_b.points[pb].py) +
					sq(cloud_a->points[pa].pz - cloud_b.points[pb].pz);

				if(sqdist > sqthresh)
					continue;

				if(sqdist < min_sqdist)
				{
					min_sqdist = sqdist;
					nearest_idx = pb;
				}
			}

			if(nearest_idx < 0) // no association found (outlier)
				continue;

			assert(min_sqdist < 999999999);
			assert(nearest_idx >= 0);

			// cloud_a[pa] and cloud_b[nearest_idx] are the closest, matched pair.

			n_associations++;

			mean_a.x += cloud_a->points[pa].px;
			mean_a.y += cloud_a->points[pa].py;
			mean_a.z += cloud_a->points[pa].pz;

			mean_b.x += cloud_b.points[nearest_idx].px;
			mean_b.y += cloud_b.points[nearest_idx].py;
			mean_b.z += cloud_b.points[nearest_idx].pz;
		}

		mean_a.x /= (double)n_associations;
		mean_a.y /= (double)n_associations;
		mean_a.z /= (double)n_associations;

		mean_b.x /= (double)n_associations;
		mean_b.y /= (double)n_associations;
		mean_b.z /= (double)n_associations;

		doublepoint_t mean_diff = {mean_b.x-mean_a.x, mean_b.y-mean_a.y, mean_b.z-mean_a.z};

		x_corr -= mean_diff.x;
		y_corr -= mean_diff.y;
		z_corr -= mean_diff.z;

		printf("ICP iter=%2d points=%6d,%6d,asso=%6d (%3.0f%%,%3.0f%%), mean_a=(%+6.0f,%+6.0f,%+6.0f), mean_b=(%+6.0f,%+6.0f,%+6.0f), diff=(%+6.0f,%+6.0f,%+6.0f), corr=(%+6.0f,%+6.0f,%+6.0f)\n",
			iter, cloud_a->n_points, cloud_b.n_points, n_associations,
			100.0*(double)n_associations/(double)cloud_a->n_points, 100.0*(double)n_associations/(double)cloud_b.n_points,
			mean_a.x, mean_a.y, mean_a.z, mean_b.x, mean_b.y, mean_b.z, mean_diff.x, mean_diff.y, mean_diff.z, x_corr, y_corr, z_corr);

	}

	*x_corr_out = x_corr;
	*y_corr_out = y_corr;
	*z_corr_out = z_corr;
	*yaw_corr_out = 0.0;


	return 0;
}

// mallocs the output. Remember to free it.
small_cloud_t* convert_cloud_to_small_cloud(cloud_t* in)
{
	small_cloud_t* out = malloc(sizeof(small_cloud_t) * in->n_points);
	assert(out);
	for(int i=0; i<in->n_points; i++)
	{
		out[i] = set_small_cloud_native_units(0, in->points[i].sx, in->points[i].sy, in->points[i].sz, in->points[i].px, in->points[i].py, in->points[i].pz);
	}
	return out;
}

int main()
{
	static cloud_t cla, clb;

	load_cloud(&cla, 10);
	load_cloud(&clb, 11);

	for(int i=0; i<cla.n_points; i++)
	{
		cla.points[i].px /= SMALL_CLOUD_POINT_RESO_X;
		cla.points[i].py /= SMALL_CLOUD_POINT_RESO_Y;
		cla.points[i].pz /= SMALL_CLOUD_POINT_RESO_Z;

		clb.points[i].px /= SMALL_CLOUD_POINT_RESO_X;
		clb.points[i].py /= SMALL_CLOUD_POINT_RESO_Y;
		clb.points[i].pz /= SMALL_CLOUD_POINT_RESO_Z;

	}


	transform_cloud(&clb, 3200/SMALL_CLOUD_POINT_RESO_X, 3500/SMALL_CLOUD_POINT_RESO_Y, 0, 0.0);	

	static cloud_t clab;

	clab.n_points = cla.n_points + clb.n_points;
	memcpy(clab.points, cla.points, sizeof cla.points[0] * cla.n_points);
	memcpy(&clab.points[cla.n_points], clb.points, sizeof clb.points[0] * clb.n_points);

	double xc, yc, zc, yawc;

	match_by_closest_points(&cla, &clb,
		0.0, 0.0, 0.0, 0.0,
		500,
		&xc, &yc, &zc, &yawc);

	static cloud_t cld;
	transform_cloud_copy(&clb, &cld, xc, yc, zc, yawc);	

	static cloud_t clad;

	clad.n_points = cla.n_points + cld.n_points;
	memcpy(clad.points, cla.points, sizeof cla.points[0] * cla.n_points);
	memcpy(&clad.points[cla.n_points], cld.points, sizeof cld.points[0] * cld.n_points);


	small_cloud_t* scla = convert_cloud_to_small_cloud(&cla);
	small_cloud_t* sclb = convert_cloud_to_small_cloud(&clb);
	small_cloud_t* sclab = convert_cloud_to_small_cloud(&clab);
	small_cloud_t* sclad = convert_cloud_to_small_cloud(&clad);
	save_small_cloud("cla.smallcloud", 0,0,0, cla.n_points, scla);
	save_small_cloud("clb.smallcloud", 0,0,0, clb.n_points, sclb);
	save_small_cloud("clc.smallcloud", 0,0,0, clab.n_points, sclab);
	save_small_cloud("cld.smallcloud", 0,0,0, clad.n_points, sclad);
	free(scla);
	free(sclb);
	free(sclab);
	free(sclad);

	return 0;
}


/*
static int compare_x(const void* a, const void* b)
{
	if(((point_t*)a)->x < ((point_t*)b)->x)
		return -1;
	else if(((point_t*)a)->x > ((point_t*)b)->x)
		return -1;
	return 0;

}

static int compare_y(const void* a, const void* b)
{
	if(((point_t*)a)->y < ((point_t*)b)->y)
		return -1;
	else if(((point_t*)a)->y > ((point_t*)b)->y)
		return -1;
	return 0;
}

static int compare_z(const void* a, const void* b)
{
	if(((point_t*)a)->z < ((point_t*)b)->z)
		return -1;
	else if(((point_t*)a)->z > ((point_t*)b)->z)
		return -1;
	return 0;
}

void kdtree(point_t* p_points, int n_points, int depth)
{

}
*/
