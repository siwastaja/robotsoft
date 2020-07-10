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

#define ICP_BLOCK_SIZE_BITS (4)
#define ICP_BLOCK_SIZE_MASK (0b1111)

#define ICP_BLOCK_XS  (1<<ICP_BLOCK_SIZE_BITS)
#define ICP_BLOCK_YS  (1<<ICP_BLOCK_SIZE_BITS)
#define ICP_BLOCK_ZS  (1<<ICP_BLOCK_SIZE_BITS)



#define ICP_N_SEARCH_BLOCKS_X 1
#define ICP_N_SEARCH_BLOCKS_Y 1
#define ICP_N_SEARCH_BLOCKS_Z 1

// Initially, ICP blocks have no allocated memory, and zero points.
// When the first point belonging to a block is found, memory is allocated
// for ICP_BLOCK_INIT_ALLOC_POINTS points at once. Whenever the memory runs
// out, the block is realloc()'d with double the points, each time.
#define ICP_BLOCK_INIT_ALLOC_POINTS 16

typedef uint16_t icp_point_t;

#define ICP_POINT_X_OFFS   0
#define ICP_POINT_X_BITS   4
#define ICP_POINT_X_MASK   (0b1111)
#define ICP_POINT_Y_OFFS   4
#define ICP_POINT_Y_BITS   4
#define ICP_POINT_Y_MASK   (0b1111)
#define ICP_POINT_Z_OFFS   8
#define ICP_POINT_Z_BITS   4
#define ICP_POINT_Z_MASK   (0b1111)

typedef struct
{
	int n_points;
	int n_alloc;
	icp_point_t* points;
} icp_block_t;

#define N_ICP_BLOCKS_X 256
#define N_ICP_BLOCKS_Y 256
#define N_ICP_BLOCKS_Z 64

#define ICP_BLOCK_ADDR_X_OFFS 0
#define ICP_BLOCK_ADDR_X_BITS 8
#define ICP_BLOCK_ADDR_Y_OFFS 8
#define ICP_BLOCK_ADDR_Y_BITS 8
#define ICP_BLOCK_ADDR_Z_OFFS 16
#define ICP_BLOCK_ADDR_Z_BITS 6

icp_block_t blocks_a[N_ICP_BLOCKS_X*N_ICP_BLOCKS_Y*N_ICP_BLOCKS_Z];
icp_block_t blocks_b[N_ICP_BLOCKS_X*N_ICP_BLOCKS_Y*N_ICP_BLOCKS_Z];


void free_and_zero_blocks(icp_block_t* blocks)
{
	for(int i=0; i<N_ICP_BLOCKS_X*N_ICP_BLOCKS_Y*N_ICP_BLOCKS_Z; i++)
	{
		if(blocks[i].points != NULL)
			free(blocks[i].points);
	}

	memset(blocks, 0, N_ICP_BLOCKS_X*N_ICP_BLOCKS_Y*N_ICP_BLOCKS_Z * sizeof blocks[0]);
}

void build_blocks(cloud_t* cloud, icp_block_t* blocks)
{
	int oor_ignored = 0;
	for(int pi=0; pi<cloud->n_points; pi++)
	{
		int x = cloud->points[pi].px;
		int y = cloud->points[pi].py;
		int z = cloud->points[pi].pz;

		x += N_ICP_BLOCKS_X/2 * ICP_BLOCK_XS;
		y += N_ICP_BLOCKS_Y/2 * ICP_BLOCK_YS;
		z += N_ICP_BLOCKS_Z/2 * ICP_BLOCK_ZS;

		if(x < 0 || y < 0 || z < 0 ||
		   x >= N_ICP_BLOCKS_X*ICP_BLOCK_XS || y >= N_ICP_BLOCKS_Y*ICP_BLOCK_YS || z >= N_ICP_BLOCKS_Z*ICP_BLOCK_ZS)
		{
			oor_ignored++;
			continue;
		}

		unsigned int bx = x>>ICP_BLOCK_SIZE_BITS;
		unsigned int by = y>>ICP_BLOCK_SIZE_BITS;
		unsigned int bz = z>>ICP_BLOCK_SIZE_BITS;

		// Remaining: offset within the block
		unsigned int ox = x & ICP_BLOCK_SIZE_MASK;
		unsigned int oy = y & ICP_BLOCK_SIZE_MASK;
		unsigned int oz = z & ICP_BLOCK_SIZE_MASK;

		assert(ox < 16);
		assert(oy < 16);
		assert(oz < 16);

		unsigned int block_idx = (bx<<ICP_BLOCK_ADDR_X_OFFS) | (by<<ICP_BLOCK_ADDR_Y_OFFS) | (bz<<ICP_BLOCK_ADDR_Z_OFFS);

		if(blocks[block_idx].n_alloc <= blocks[block_idx].n_points)
		{
			// Out of room for points, reallocate more space
			if(blocks[block_idx].n_alloc == 0)
				blocks[block_idx].n_alloc = ICP_BLOCK_INIT_ALLOC_POINTS;
			else
				blocks[block_idx].n_alloc *= 2;

			// realloc works like malloc when the pointer is NULL, the initial case.
			blocks[block_idx].points = realloc(blocks[block_idx].points, blocks[block_idx].n_alloc * sizeof blocks[block_idx].points[0]);

			assert(blocks[block_idx].points);
		}

		// Insert the point:

		blocks[block_idx].points[blocks[block_idx].n_points] = 
			(ox<<ICP_POINT_X_OFFS) | (oy<<ICP_POINT_Y_OFFS) | (oz<<ICP_POINT_Z_OFFS);

		blocks[block_idx].n_points++;
	}

	int64_t total_points = 0;
	int64_t total_allocd = 0;

	int nonnull_a = 0, nonnull_b = 0;

	for(int i=0; i<N_ICP_BLOCKS_X*N_ICP_BLOCKS_Y*N_ICP_BLOCKS_Z; i++)
	{
		total_points += blocks[i].n_points;
		total_allocd += blocks[i].n_alloc;

		if(blocks[i].points != NULL)
			nonnull_a++;

		if(blocks[i].n_points > 0)
			nonnull_b++;
	}

	assert(nonnull_a == nonnull_b);

	printf("build_blocks(): cloud->n_points = %d, oor_ignored=%d, active blocks = %d, points in blocks = %ld, allocated points = %ld (mem %ld KB)\n", 
		cloud->n_points, oor_ignored, nonnull_a, total_points, total_allocd, total_allocd * sizeof blocks[0].points[0] / 1024);
}


static int match_by_closest_points(cloud_t* cloud_a, cloud_t* cloud_b_in,
	double x_corr, double y_corr, double z_corr, double yaw_corr,
	int threshold,
	double* x_corr_out, double* y_corr_out, double* z_corr_out, double* yaw_corr_out)
{
	float mat_rota[9];
	float mat_transl[3];

	build_blocks(cloud_a, blocks_a);

	for(int iter=0; iter<30; iter++)
	{
		doublepoint_t mean_a = {0,0,0};
		doublepoint_t mean_b = {0,0,0};

		static cloud_t cloud_b;

		transform_cloud_copy(cloud_b_in, &cloud_b, x_corr, y_corr, z_corr, yaw_corr);

		build_blocks(&cloud_b, blocks_b);


		int n_associations = 0;

		// Loop over all blocks of a;
		// for each point within the a block, loop over the points of the few adjacent blocks in b
		for(unsigned int a_bx = ICP_N_SEARCH_BLOCKS_X; a_bx < N_ICP_BLOCKS_X-ICP_N_SEARCH_BLOCKS_X; a_bx++)
		{
			for(unsigned int a_by = ICP_N_SEARCH_BLOCKS_Y; a_by < N_ICP_BLOCKS_Y-ICP_N_SEARCH_BLOCKS_Y; a_by++)
			{
				for(unsigned int a_bz = ICP_N_SEARCH_BLOCKS_Z; a_bz < N_ICP_BLOCKS_Z-ICP_N_SEARCH_BLOCKS_Z; a_bz++)
				{

					unsigned int a_bidx = (a_bx<<ICP_BLOCK_ADDR_X_OFFS) | (a_by<<ICP_BLOCK_ADDR_Y_OFFS) | (a_bz<<ICP_BLOCK_ADDR_Z_OFFS);

					for(int ai=0; ai<blocks_a[a_bidx].n_points; ai++)
					{
						unsigned int ax = (a_bx<<ICP_BLOCK_SIZE_BITS) | ((blocks_a[a_bidx].points[ai]>>ICP_POINT_X_OFFS)&ICP_POINT_X_MASK);
						unsigned int ay = (a_by<<ICP_BLOCK_SIZE_BITS) | ((blocks_a[a_bidx].points[ai]>>ICP_POINT_Y_OFFS)&ICP_POINT_Y_MASK);
						unsigned int az = (a_bz<<ICP_BLOCK_SIZE_BITS) | ((blocks_a[a_bidx].points[ai]>>ICP_POINT_Z_OFFS)&ICP_POINT_Z_MASK);


						unsigned int nearest_bx;
						unsigned int nearest_by;
						unsigned int nearest_bz;

						int min_sqdist = 999999999;
						for(unsigned int b_bx = a_bx-ICP_N_SEARCH_BLOCKS_X; b_bx <= a_bx+ICP_N_SEARCH_BLOCKS_X; b_bx++)
						{
							for(unsigned int b_by = a_by-ICP_N_SEARCH_BLOCKS_Y; b_by <= a_by+ICP_N_SEARCH_BLOCKS_Y; b_by++)
							{
								for(unsigned int b_bz = a_bz-ICP_N_SEARCH_BLOCKS_Z; b_bz <= a_bz+ICP_N_SEARCH_BLOCKS_Z; b_bz++)
								{
									unsigned int b_bidx = (b_bx<<ICP_BLOCK_ADDR_X_OFFS) | (b_by<<ICP_BLOCK_ADDR_Y_OFFS) | (b_bz<<ICP_BLOCK_ADDR_Z_OFFS);

									for(int bi=0; bi<blocks_b[b_bidx].n_points; bi++)
									{
										unsigned int bx = (b_bx<<ICP_BLOCK_SIZE_BITS) | ((blocks_b[b_bidx].points[bi]>>ICP_POINT_X_OFFS)&ICP_POINT_X_MASK);
										unsigned int by = (b_by<<ICP_BLOCK_SIZE_BITS) | ((blocks_b[b_bidx].points[bi]>>ICP_POINT_Y_OFFS)&ICP_POINT_Y_MASK);
										unsigned int bz = (b_bz<<ICP_BLOCK_SIZE_BITS) | ((blocks_b[b_bidx].points[bi]>>ICP_POINT_Z_OFFS)&ICP_POINT_Z_MASK);

										int sqdist = sq(bx-ax) + sq(by-ay) + sq(bz-az);

										if(sqdist < min_sqdist)
										{
											min_sqdist = sqdist;
											nearest_bx = bx;
											nearest_by = by;
											nearest_bz = bz;
										}
									}
								}
							}

						}


						if(min_sqdist == 999999999) // no association found (outlier)
							continue;

						assert(min_sqdist < 999999999);

						n_associations++;

						mean_a.x += ax;
						mean_a.y += ay;
						mean_a.z += az;

						mean_b.x += nearest_bx;
						mean_b.y += nearest_by;
						mean_b.z += nearest_bz;
					}
				}
			}

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

		free_and_zero_blocks(blocks_b);
	}


	free_and_zero_blocks(blocks_a);

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
