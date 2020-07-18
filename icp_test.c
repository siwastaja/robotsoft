#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "slam_cloud.h"

#include "misc.h"

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
#define ICP_BLOCK_SIZE_MASK ((1<<ICP_BLOCK_SIZE_BITS)-1)

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

#define ICP_BLOCK_TIGHT_ALLOC

typedef uint16_t icp_point_t;
#define DELETED_POINT  0xffff
#define LAST_POINT     0x8000

#define ICP_POINT_X_OFFS   0
#define ICP_POINT_X_BITS   ICP_BLOCK_SIZE_BITS
#define ICP_POINT_X_MASK   ICP_BLOCK_SIZE_MASK
#define ICP_POINT_Y_OFFS   ICP_BLOCK_SIZE_BITS
#define ICP_POINT_Y_BITS   ICP_BLOCK_SIZE_BITS
#define ICP_POINT_Y_MASK   ICP_BLOCK_SIZE_MASK
#define ICP_POINT_Z_OFFS   (2*ICP_BLOCK_SIZE_BITS)
#define ICP_POINT_Z_BITS   ICP_BLOCK_SIZE_BITS
#define ICP_POINT_Z_MASK   ICP_BLOCK_SIZE_MASK


#define N_ICP_POINTS 16

typedef struct
{
	icp_point_t points[N_ICP_POINTS];
} icp_block_t;


#define ICP_BLOCK_ADDR_X_BITS 8  // 8
#define ICP_BLOCK_ADDR_Y_BITS 8  // 8
#define ICP_BLOCK_ADDR_Z_BITS 6  // 6

#define ICP_BLOCK_ADDR_X_OFFS 0
#define ICP_BLOCK_ADDR_Y_OFFS (ICP_BLOCK_ADDR_X_BITS)
#define ICP_BLOCK_ADDR_Z_OFFS (ICP_BLOCK_ADDR_X_BITS+ICP_BLOCK_ADDR_Y_BITS)


#define N_ICP_BLOCKS_X (1<<ICP_BLOCK_ADDR_X_BITS)
#define N_ICP_BLOCKS_Y (1<<ICP_BLOCK_ADDR_Y_BITS)
#define N_ICP_BLOCKS_Z (1<<ICP_BLOCK_ADDR_Z_BITS)


icp_block_t blocks_a[N_ICP_BLOCKS_X*N_ICP_BLOCKS_Y*N_ICP_BLOCKS_Z];
icp_block_t blocks_b[N_ICP_BLOCKS_X*N_ICP_BLOCKS_Y*N_ICP_BLOCKS_Z];


static void free_and_zero_blocks(icp_block_t* blocks)
{
	for(int i=0; i < N_ICP_BLOCKS_X*N_ICP_BLOCKS_Y*N_ICP_BLOCKS_Z; i++)
		for(int o=0; o<N_ICP_POINTS; o++)
			blocks[i].points[o] = LAST_POINT;
//	memset(blocks, 0, N_ICP_BLOCKS_X*N_ICP_BLOCKS_Y*N_ICP_BLOCKS_Z * sizeof blocks[0]);
}

#define BUILD_BLOCKS_STATS

// reduce: 0 to 100
void build_blocks(cloud_t* cloud, icp_block_t* blocks, int reduce)
{
	static int n_points[N_ICP_BLOCKS_X*N_ICP_BLOCKS_Y*N_ICP_BLOCKS_Z];
	memset(n_points, 0, sizeof n_points);

	#ifdef BUILD_BLOCKS_STATS

	int oor_ignored = 0;
	int n_reduced = 0;
	int n_over_256 = 0;
	int n_over_128 = 0;
	int n_over_64 = 0;
	int n_over_32 = 0;
	int n_over_16 = 0;
	int n_over_8 = 0;

	#endif

	int reduce_lim = 0;
	if(reduce > 0)
		reduce_lim = (sq(reduce*ICP_BLOCK_XS))/(sq(100));

	free_and_zero_blocks(blocks);

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
			#ifdef BUILD_BLOCKS_STATS
				oor_ignored++;
			#endif
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

		if(reduce_lim > 0)
		{
			int_fast32_t sqdist_closest = INT_FAST32_MAX;
			//int closest_idx;
			
			for(int i = 0; i < n_points[block_idx]; i++)
//			for(int i = 0; i < blocks[block_idx].n_points; i++)
			{
				int earlier_ox = (blocks[block_idx].points[i]>>ICP_POINT_X_OFFS)&ICP_POINT_X_MASK;
				int earlier_oy = (blocks[block_idx].points[i]>>ICP_POINT_Y_OFFS)&ICP_POINT_Y_MASK;
				int earlier_oz = (blocks[block_idx].points[i]>>ICP_POINT_Z_OFFS)&ICP_POINT_Z_MASK;

				int_fast32_t sqdist = sq(earlier_ox - (int)ox) + sq(earlier_oy - (int)oy) + sq(earlier_oz - (int)oz);

				if(sqdist < sqdist_closest)
				{
					sqdist_closest = sqdist;
					//closest_idx = i;
				}
			}

			if(sqdist_closest < reduce_lim)
			{
				#ifdef BUILD_BLOCKS_STATS
					n_reduced++;
				#endif
				continue;
			}
		}

		// Insert the point:

		if(n_points[block_idx] < N_ICP_POINTS-1)
		{
			blocks[block_idx].points[n_points[block_idx]] = 
				(ox<<ICP_POINT_X_OFFS) | (oy<<ICP_POINT_Y_OFFS) | (oz<<ICP_POINT_Z_OFFS);

			n_points[block_idx]++;
		}
	}

	#ifdef BUILD_BLOCKS_STATS
		int64_t total_points = 0;

		int nonnull = 0;

		for(int i=0; i<N_ICP_BLOCKS_X*N_ICP_BLOCKS_Y*N_ICP_BLOCKS_Z; i++)
		{
			total_points += n_points[i];

			if(n_points[i] > 0)
				nonnull++;

			if(n_points[i] > 8)
				n_over_8++;
			if(n_points[i] > 16)
				n_over_16++;
			if(n_points[i] > 32)
				n_over_32++;
			if(n_points[i] > 64)
				n_over_64++;
			if(n_points[i] > 128)
				n_over_128++;
			if(n_points[i] > 256)
				n_over_256++;
		}

		printf("build_blocks(): cloud->n_points = %d, oor_ignored=%d, reduced = %d, active blocks = %d, points in blocks = %ld\n", 
			cloud->n_points, oor_ignored, n_reduced, nonnull, total_points);
		printf("    over8: %d,  over16: %d,  over32: %d,  over64: %d,  over128: %d,  over256: %d\n", n_over_8, n_over_16, n_over_32, n_over_64, n_over_128, n_over_256);

	#endif
}


static int match_by_closest_points(cloud_t* cloud_a, cloud_t* cloud_b_in,
	double x_corr, double y_corr, double z_corr, double yaw_corr,
	int threshold,
	double* x_corr_out, double* y_corr_out, double* z_corr_out, double* yaw_corr_out,
	cloud_t* combined_cloud_out)
{
	float mat_rota[9];
	float mat_transl[3];

	build_blocks(cloud_a, blocks_a, 40);

	double t_total_total = 0.0;

	for(int iter=0; iter<20; iter++)
	{
		double ts_initial;
		double ts;
		double t_transform, t_build, t_search, t_free, t_total;

		doublepoint_t mean_diff = {0,0,0};

		int_fast64_t dot_accum = 0;
		int_fast64_t det_accum = 0;

		static cloud_t cloud_b;

		ts_initial = ts = subsec_timestamp();
		transform_cloud_copy(cloud_b_in, &cloud_b, x_corr, y_corr, z_corr, yaw_corr);
		t_transform = subsec_timestamp() - ts;

		ts = subsec_timestamp();
		build_blocks(&cloud_b, blocks_b, 30);
		t_build = subsec_timestamp() - ts;


		ts = subsec_timestamp();

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

					icp_point_t* pa = blocks_a[a_bidx].points;
					while(1)
					{
						if(*pa == LAST_POINT)
							break;

						unsigned int ax = (a_bx<<ICP_BLOCK_SIZE_BITS) | (((*pa)>>ICP_POINT_X_OFFS)&ICP_POINT_X_MASK);
						unsigned int ay = (a_by<<ICP_BLOCK_SIZE_BITS) | (((*pa)>>ICP_POINT_Y_OFFS)&ICP_POINT_Y_MASK);
						unsigned int az = (a_bz<<ICP_BLOCK_SIZE_BITS) | (((*pa)>>ICP_POINT_Z_OFFS)&ICP_POINT_Z_MASK);

						pa++;

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

									icp_point_t* pb = blocks_b[b_bidx].points;
									while(1)
									{
										if(*pb == LAST_POINT)
											break;

										unsigned int bx = (b_bx<<ICP_BLOCK_SIZE_BITS) | (((*pb)>>ICP_POINT_X_OFFS)&ICP_POINT_X_MASK);
										unsigned int by = (b_by<<ICP_BLOCK_SIZE_BITS) | (((*pb)>>ICP_POINT_Y_OFFS)&ICP_POINT_Y_MASK);
										unsigned int bz = (b_bz<<ICP_BLOCK_SIZE_BITS) | (((*pb)>>ICP_POINT_Z_OFFS)&ICP_POINT_Z_MASK);

										pb++;

										int sqdist = sq(bx-ax) + sq(by-ay) + 2*sq(bz-az);

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

						mean_diff.x += (signed int)nearest_bx - (signed int)ax;
						mean_diff.y += (signed int)nearest_by - (signed int)ay;
						mean_diff.z += (signed int)nearest_bz - (signed int)az;

						int sax = (signed int)ax - N_ICP_BLOCKS_X/2 * ICP_BLOCK_XS;
						int say = (signed int)ay - N_ICP_BLOCKS_Y/2 * ICP_BLOCK_YS;
						int saz = (signed int)az - N_ICP_BLOCKS_Z/2 * ICP_BLOCK_ZS;
						int sbx = (signed int)nearest_bx - N_ICP_BLOCKS_X/2 * ICP_BLOCK_XS;
						int sby = (signed int)nearest_by - N_ICP_BLOCKS_Y/2 * ICP_BLOCK_YS;
						int sbz = (signed int)nearest_bz - N_ICP_BLOCKS_Z/2 * ICP_BLOCK_ZS;

						int_fast64_t dot = sax*sbx + say*sby;
						int_fast64_t det = sax*sby - say*sbx;

						dot_accum += dot;
						det_accum += det;
					}
				}
			}

		}

		t_search = subsec_timestamp() - ts;

		ts = subsec_timestamp();
//		free_and_zero_blocks(blocks_b);
		t_free = subsec_timestamp() - ts;

		mean_diff.x /= (double)n_associations;
		mean_diff.y /= (double)n_associations;
		mean_diff.z /= (double)n_associations;

		double yaw = atan2(det_accum, dot_accum);

		x_corr -= mean_diff.x;
		y_corr -= mean_diff.y;
		z_corr -= mean_diff.z;
		yaw_corr -= yaw;

		t_total = subsec_timestamp() - ts_initial;

		printf("ICP iter=%2d points=%6d,%6d,asso=%6d (%3.0f%%,%3.0f%%), transl=(%+6.0f,%+6.0f,%+6.0f), yaw=%.2f deg, corr=(%+6.0f,%+6.0f,%+6.0f; %.2f deg)\n",
			iter, cloud_a->n_points, cloud_b.n_points, n_associations,
			100.0*(double)n_associations/(double)cloud_a->n_points, 100.0*(double)n_associations/(double)cloud_b.n_points,
			mean_diff.x, mean_diff.y, mean_diff.z, RADTODEG(yaw), x_corr, y_corr, z_corr, RADTODEG(yaw_corr));


		printf("Performance: transform %.2f ms  build %.2f ms  search %.2f ms  free %.2f ms  total %.2f ms\n\n",
			t_transform*1000.0, t_build*1000.0, t_search*1000.0, t_free*1000.0, t_total*1000.0);

		t_total_total += t_total;
	}

	printf("total time = %.1f ms\n", t_total_total*1000.0);


//	free_and_zero_blocks(blocks_a);

	build_blocks(cloud_a, blocks_a, 0);

	combined_cloud_out->n_points = 0;
	// PRODUCE THE OUTPUT
	{
		static cloud_t cloud_b;

		transform_cloud_copy(cloud_b_in, &cloud_b, x_corr, y_corr, z_corr, yaw_corr);

		build_blocks(&cloud_b, blocks_b, 0);
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


					icp_point_t* pa = blocks_a[a_bidx].points;
					while(1)
					{
						if(*pa == LAST_POINT)
							break;

						unsigned int ax = (a_bx<<ICP_BLOCK_SIZE_BITS) | (((*pa)>>ICP_POINT_X_OFFS)&ICP_POINT_X_MASK);
						unsigned int ay = (a_by<<ICP_BLOCK_SIZE_BITS) | (((*pa)>>ICP_POINT_Y_OFFS)&ICP_POINT_Y_MASK);
						unsigned int az = (a_bz<<ICP_BLOCK_SIZE_BITS) | (((*pa)>>ICP_POINT_Z_OFFS)&ICP_POINT_Z_MASK);

						pa++;

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

									icp_point_t* pb = blocks_b[b_bidx].points;
									while(1)
									{
										if(*pb == LAST_POINT)
											break;

										unsigned int bx = (b_bx<<ICP_BLOCK_SIZE_BITS) | (((*pb)>>ICP_POINT_X_OFFS)&ICP_POINT_X_MASK);
										unsigned int by = (b_by<<ICP_BLOCK_SIZE_BITS) | (((*pb)>>ICP_POINT_Y_OFFS)&ICP_POINT_Y_MASK);
										unsigned int bz = (b_bz<<ICP_BLOCK_SIZE_BITS) | (((*pb)>>ICP_POINT_Z_OFFS)&ICP_POINT_Z_MASK);

										pb++;

										int sqdist = sq(bx-ax) + sq(by-ay) + 2*sq(bz-az);

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


						if(min_sqdist > sq(3)) // no close association found - copy A to output as is
						{
							combined_cloud_out->points[combined_cloud_out->n_points++] =
								(cloud_point_t){0,0,0,
								ax - N_ICP_BLOCKS_X/2 * ICP_BLOCK_XS,
								ay - N_ICP_BLOCKS_Y/2 * ICP_BLOCK_YS,
								az - N_ICP_BLOCKS_Z/2 * ICP_BLOCK_ZS};

							assert(combined_cloud_out->n_points <= MAX_POINTS);
						}
						else
						{
							// Let 
						}
	

					}
				}
			}

		}

		// Copy all transformed B points
		// The two code snippets have the same result.

		for(int i=0; i<cloud_b.n_points; i++)
		{
			combined_cloud_out->points[combined_cloud_out->n_points++] = 
				cloud_b.points[i];

			assert(combined_cloud_out->n_points <= MAX_POINTS);
		}


/*
		for(unsigned int b_bx = ICP_N_SEARCH_BLOCKS_X; b_bx < N_ICP_BLOCKS_X-ICP_N_SEARCH_BLOCKS_X; b_bx++)
		{
			for(unsigned int b_by = ICP_N_SEARCH_BLOCKS_Y; b_by < N_ICP_BLOCKS_Y-ICP_N_SEARCH_BLOCKS_Y; b_by++)
			{
				for(unsigned int b_bz = ICP_N_SEARCH_BLOCKS_Z; b_bz < N_ICP_BLOCKS_Z-ICP_N_SEARCH_BLOCKS_Z; b_bz++)
				{
					unsigned int b_bidx = (b_bx<<ICP_BLOCK_ADDR_X_OFFS) | (b_by<<ICP_BLOCK_ADDR_Y_OFFS) | (b_bz<<ICP_BLOCK_ADDR_Z_OFFS);

					for(int bi=0; bi<blocks_b[b_bidx].n_points; bi++)
					{
						if(blocks_b[b_bidx].points[bi] != DELETED_POINT)
						{
							unsigned int bx = (b_bx<<ICP_BLOCK_SIZE_BITS) | ((blocks_b[b_bidx].points[bi]>>ICP_POINT_X_OFFS)&ICP_POINT_X_MASK);
							unsigned int by = (b_by<<ICP_BLOCK_SIZE_BITS) | ((blocks_b[b_bidx].points[bi]>>ICP_POINT_Y_OFFS)&ICP_POINT_Y_MASK);
							unsigned int bz = (b_bz<<ICP_BLOCK_SIZE_BITS) | ((blocks_b[b_bidx].points[bi]>>ICP_POINT_Z_OFFS)&ICP_POINT_Z_MASK);

							combined_cloud_out->points[combined_cloud_out->n_points++] =
								(cloud_point_t){0,0,0,
								bx - N_ICP_BLOCKS_X/2 * ICP_BLOCK_XS,
								by - N_ICP_BLOCKS_Y/2 * ICP_BLOCK_YS,
								bz - N_ICP_BLOCKS_Z/2 * ICP_BLOCK_ZS};

							assert(combined_cloud_out->n_points <= MAX_POINTS);
						}
					}
				}
			}
		}
*/


	}



//	free_and_zero_blocks(blocks_a);

	*x_corr_out = x_corr;
	*y_corr_out = y_corr;
	*z_corr_out = z_corr;
	*yaw_corr_out = yaw_corr;


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
	printf("sizeof icp_block_t = %u\n", sizeof (icp_block_t));
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


	transform_cloud(&clb, 3200/SMALL_CLOUD_POINT_RESO_X, 3500/SMALL_CLOUD_POINT_RESO_Y, 0, DEGTORAD(0.0));	

	static cloud_t clab;

	clab.n_points = cla.n_points + clb.n_points;
	memcpy(clab.points, cla.points, sizeof cla.points[0] * cla.n_points);
	memcpy(&clab.points[cla.n_points], clb.points, sizeof clb.points[0] * clb.n_points);

	double xc, yc, zc, yawc;

	static cloud_t clad;

	match_by_closest_points(&cla, &clb,
		0.0, 0.0, 0.0, 0.0,
		500,
		&xc, &yc, &zc, &yawc, &clad);

	static cloud_t cld;
	transform_cloud_copy(&clb, &cld, xc, yc, zc, yawc);	


	static cloud_t cle;
	cle.n_points = cla.n_points + cld.n_points;
	memcpy(cle.points, cla.points, sizeof cla.points[0] * cla.n_points);
	memcpy(&cle.points[cla.n_points], cld.points, sizeof cld.points[0] * cld.n_points);


	small_cloud_t* scla = convert_cloud_to_small_cloud(&cla);
	small_cloud_t* sclb = convert_cloud_to_small_cloud(&clb);
	small_cloud_t* sclab = convert_cloud_to_small_cloud(&clab);
	small_cloud_t* sclad = convert_cloud_to_small_cloud(&clad);
	small_cloud_t* scle = convert_cloud_to_small_cloud(&cle);
	save_small_cloud("cla.smallcloud", 0,0,0, cla.n_points, scla);
	save_small_cloud("clb.smallcloud", 0,0,0, clb.n_points, sclb);
	save_small_cloud("clc.smallcloud", 0,0,0, clab.n_points, sclab);
	save_small_cloud("cld.smallcloud", 0,0,0, clad.n_points, sclad);
	save_small_cloud("cle.smallcloud", 0,0,0, cle.n_points, scle);
	free(scla);
	free(sclb);
	free(sclab);
	free(sclad);
	free(scle);

	printf("%d   %d   %d   %d\n", cla.n_points, clb.n_points, clad.n_points, cle.n_points);

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
