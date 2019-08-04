/*
	Matching functions for SLAM.

	These are the low-level core of the SLAM.
	We have quite some many different types of functions, reducing code readability, sorry for that.

	These functions need to be heavily tailored for their specific use cases, for performance.

	We heavily utilize the inherent "vectorization" of any CPU, namely the automatic vectorization
	of 1-bit datatypes on any CPU using the and, or, bitshift etc. operations.

	Coarse matching is a brute-force scoring operation. Compare to, for example, classical 
	iterative point matching algorithms, which fail to find global minima and get stuck
	to the local minima nearest to the starting point.

	Our coarse matching functions return a list of possible matches, which are then
	filtered and sorted to create a list of possible matches. 

	Consider matching this observation:

	------------------



	-------+    +-----
	       |    |

	
	to this group of submaps:

	--------------------------------------------------------------------
	


	-------+    +-------+    +-------+    +-------+    +-------+     +-- 
	       |    |       |    |       |    |       |    |       |     |


	After coming back from a long trip through one of these corridors,
	so that we don't have a precise enough pose estimate to weigh the matching.

	The only way is to brute-force through the entire range of possibilities
	and store as many local minima as possible, then optimize the graph
	later by utilizing other closures

*/

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>

#include "slam_cloud.h"
#include "slam_matchers.h"

#include "voxmap.h"
#include "voxmap_memdisk.h"


#define HEAP_PARENT(i) (((i) - 1) >> 1)
#define HEAP_LEFT(i)   (((i) << 1) + 1)
#define HEAP_RIGHT(i) (((i) << 1) + 2)


#define MATCHER_MAX_THREADS 4


#define COMPRESS_CLOUDS

#ifdef COMPRESS_CLOUDS
	#include <zlib.h>

	#define ZLIB_CHUNK (256*1024)
	#define ZLIB_LEVEL 2  // from 1 to 9, 9 = slowest but best compression
#endif

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


#define MATCHMAP_UNIT 256 //mm

#define MATCHMAP_XS 256
#define MATCHMAP_YS 256
#define MATCHMAP_ZS 64


#define FINE_MATCHMAP_UNIT 128 //mm

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

typedef struct __attribute__((packed))
{
	ref_matchmap_unit_t units[MATCHMAP_YS][MATCHMAP_XS];
} ref_matchmap_t;

typedef struct __attribute__((packed))
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

typedef struct __attribute__((packed))
{
	cmp_matchmap_unit_t units[MATCHMAP_YS][MATCHMAP_XS];
} cmp_matchmap_t;

typedef struct __attribute__((packed))
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

static void cmp_matchmap_to_voxmap(cmp_matchmap_t* matchmap, int ref_x, int ref_y, int ref_z)
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
			}
		}
	}
}


static void ref_fine_matchmap_to_voxmap(ref_fine_matchmap_t* matchmap, int ref_x, int ref_y, int ref_z)
{
	int rl = 0;

	for(int yy=0; yy<FINE_MATCHMAP_YS; yy++)
	{
		for(int xx=0; xx<FINE_MATCHMAP_XS; xx++)
		{
			for(int zz=0; zz<FINE_MATCHMAP_ZS; zz++)
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

static void voxmap_to_ref_fine_matchmap(ref_fine_matchmap_t* matchmap, int ref_x, int ref_y, int ref_z)
{
	int rl = 0;

	// Find the VOX_UNIT which allows direct copy
	// Assumption: it exists.
	// Benefit: no "aliasing" effects
	while(VOX_UNITS[rl] < FINE_MATCHMAP_UNIT && rl < 8)
		rl++;

	// If it isn't possible, use the next better resolevel. Warning: gaps.
	if(VOX_UNITS[rl] > FINE_MATCHMAP_UNIT && rl > 0)
	{
		rl--;
		printf("WARNING: voxmap_to_ref_fine_matchmap: no compatible world and ref_fine_matchmap resolution, rl=%d\n", rl);
	}


	// Load multiple pages at once, but not too many - only 1 page in Y direction
	for(int yy=0; yy<FINE_MATCHMAP_YS; yy++)
	{
		int iyy = yy - FINE_MATCHMAP_YS/2;
		// x and y ranges for load
		po_coords_t polx0 = po_coords((-FINE_MATCHMAP_XS/2-1)*VOX_UNITS[rl]+ref_x, iyy*VOX_UNITS[rl]+ref_y, 0, rl);
		po_coords_t polx1 = po_coords((FINE_MATCHMAP_XS/2+1)*VOX_UNITS[rl]+ref_x, iyy*VOX_UNITS[rl]+ref_y, 0, rl);

		// z range for load
		po_coords_t polz0 = po_coords(0, 0, (-FINE_MATCHMAP_ZS/2-1)*VOX_UNITS[rl]+ref_z, rl);
		po_coords_t polz1 = po_coords(0, 0, (FINE_MATCHMAP_ZS/2+1)*VOX_UNITS[rl]+ref_z, rl);

		load_pages(RESOLEVELS, RESOLEVELS, polx0.px, polx1.px, polx0.py, polx1.py, polz0.pz, polz1.pz);

		for(int xx=0; xx<FINE_MATCHMAP_XS; xx++)
		{
			int ixx = xx - FINE_MATCHMAP_XS/2;
			for(int zz=0; zz<FINE_MATCHMAP_ZS; zz++)
			{
				int izz = zz - FINE_MATCHMAP_ZS/2;

				po_coords_t po = po_coords(ixx*VOX_UNITS[rl]+ref_x, iyy*VOX_UNITS[rl]+ref_y, izz*VOX_UNITS[rl]+ref_z, rl);

				uint8_t* p_vox = get_p_voxel(po, rl);

				if((*p_vox) & 0x0f)
				{
					matchmap->units[yy][xx].occu |= (1ULL<<zz);
				}
				else if((*p_vox) & 0xf0)
				{
					matchmap->units[yy][xx].free |= (1ULL<<zz);
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

//#define ODROID_XU4
#define RASPI3

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
#define FINE_MATCHMAP_Z_RANGE  0

#define MATCHMAP_XYZ_N_RESULTS 32

// 100% match with no occu_on_free will be score=10000 (OCCU_MATCH_COEFF*REL_SCORE_MULT)
#define OCCU_MATCH_COEFF 1 // 1 is good for efficiency
#define OCCU_ON_FREE_COEFF -8 //-8
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
//	static const int32_t z_weights[FINE_MATCHMAP_Z_RANGE*2+1] = {9, 10, 9};
	static const int32_t z_weights[FINE_MATCHMAP_Z_RANGE*2+1] = {10};

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



// remember that clouds are centered on zero (so that the ref_x,y,z from the submap meta corresponds to 0,0,0)
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
static int cloud_to_cmp_matchmap_even(cloud_t* cloud, cmp_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
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

static int cloud_to_cmp_matchmap_odd(cloud_t* cloud, cmp_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
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


static int cloud_to_cmp_fine_matchmap(cloud_t* cloud, cmp_fine_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
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

static void cloud_to_ref_matchmap_free(cloud_t* cloud, ref_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
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

// "Free" bits are removed at the highest "bold" level
#define REF_BOLD_MODE 2
static void cloud_to_ref_matchmap_occu(cloud_t* cloud, ref_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
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


		#if REF_BOLD_MODE==1 || REF_BOLD_MODE==2
			// Calculate the voxel coordinates for double resolution
			int px = mmpx/(MATCHMAP_UNIT/2) + MATCHMAP_XS;
			int py = mmpy/(MATCHMAP_UNIT/2) + MATCHMAP_YS;
			int pz = mmpz/(MATCHMAP_UNIT/2) + MATCHMAP_ZS;

			// For every matcher thread, reference map is shifted >>1
			if(px < 1 || px > 2*MATCHMAP_XS-2 || py < 1 || py > 2*MATCHMAP_YS-2 || pz < MATCHER_MAX_THREADS || pz > 2*MATCHMAP_ZS-2)
			{
				printf("cloud_to_ref_matchmap: WARN, OOR point %d,%d,%d\n", px, py, pz);
				continue;
			}

			// Remove free at highest bold level
			for(int ix=-1; ix<=1; ix++)
				for(int iy=-1; iy<=1; iy++)
					matchmap->units[py/2+iy][px/2+ix].free &= ~(0b111ULL << (pz/2-1));

		#else
			int px = mmpx/MATCHMAP_UNIT + MATCHMAP_XS/2;
			int py = mmpy/MATCHMAP_UNIT + MATCHMAP_YS/2;
			int pz = mmpz/MATCHMAP_UNIT + MATCHMAP_ZS/2;

			// For every matcher thread, reference map is shifted >>1
			if(px < 1 || px > MATCHMAP_XS-2 || py < 1 || py > MATCHMAP_YS-2 || pz < MATCHER_MAX_THREADS || pz > MATCHMAP_ZS-2)
			{
				printf("cloud_to_ref_matchmap: WARN, OOR point %d,%d,%d\n", px, py, pz);
				continue;
			}


			// Remove free at highest bold level
			for(int ix=-1; ix<=1; ix++)
				for(int iy=-1; iy<=1; iy++)
					matchmap->units[py+iy][px+ix].free &= ~(0b111ULL << (pz-1));

		#endif

		#if REF_BOLD_MODE==0 // Create voxels directly, no bolding
			matchmap->units[py][px].occu |= 1ULL << (pz);

		#elif REF_BOLD_MODE==1 // Create four voxels, by adding one on each axis (x,y,z) to the side nearer the actual voxel
			matchmap->units[py/2][px/2].occu |= 1ULL << (pz/2);

			matchmap->units[py/2 - 1 + 2*(py%2)][px/2].occu |= 1ULL << (pz/2);

			matchmap->units[py/2][px/2 - 1 + 2*(px%2)].occu |= 1ULL << (pz/2);

			matchmap->units[py/2][px/2].occu |= 1ULL << (pz/2 - 1 + 2*(pz%2));

		#elif REF_BOLD_MODE==2 // Similarly, but create 8 voxels, a 2x2x2 cube biased correctly

			// comment 1 = use the neighbor on this axis (has the math thing in the index), 0 = actual voxel on this axis
			// y x z
			// 0 0 0
			matchmap->units[py/2][px/2].occu |= 1ULL << (pz/2);

			// 0 0 1
			matchmap->units[py/2][px/2].occu |= 1ULL << (pz/2 - 1 + 2*(pz%2));

			// 0 1 0
			matchmap->units[py/2][px/2 - 1 + 2*(px%2)].occu |= 1ULL << (pz/2);

			// 0 1 1
			matchmap->units[py/2][px/2 - 1 + 2*(px%2)].occu |= 1ULL << (pz/2 - 1 + 2*(pz%2));

			// 1 0 0
			matchmap->units[py/2 - 1 + 2*(py%2)][px/2].occu |= 1ULL << (pz/2);

			// 1 0 1
			matchmap->units[py/2 - 1 + 2*(py%2)][px/2].occu |= 1ULL << (pz/2 - 1 + 2*(pz%2));

			// 1 1 0
			matchmap->units[py/2 - 1 + 2*(py%2)][px/2 - 1 + 2*(px%2)].occu |= 1ULL << (pz/2);

			// 1 1 1
			matchmap->units[py/2 - 1 + 2*(py%2)][px/2 - 1 + 2*(px%2)].occu |= 1ULL << (pz/2 - 1 + 2*(pz%2));

		
		#elif REF_BOLD_MODE==3 // Create extra voxel in every neighbor, i.e., create a cube of 3x3x3 = 27 voxels
			for(int ix=-1; ix<=1; ix++)
				for(int iy=-1; iy<=1; iy++)
					matchmap->units[py+iy][px+ix].occu |= 0b111ULL << (pz-1);
		#else
			#error set REF_BOLD_MODE correctly
		#endif

	}
}

/*

	These versions only take every third point in the cloud.
	Much quicker. Useful for most statistical purposes, where having holes
	in free space is not an issue.
*/


static void decim_cloud_to_ref_matchmap_free(cloud_t* cloud, ref_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
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

static void decim_cloud_to_ref_matchmap_occu(cloud_t* cloud, ref_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
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


		#if REF_BOLD_MODE==1 || REF_BOLD_MODE==2
			// Calculate the voxel coordinates for double resolution
			int px = mmpx/(MATCHMAP_UNIT/2) + MATCHMAP_XS;
			int py = mmpy/(MATCHMAP_UNIT/2) + MATCHMAP_YS;
			int pz = mmpz/(MATCHMAP_UNIT/2) + MATCHMAP_ZS;

			// For every matcher thread, reference map is shifted >>1
			if(px < 1 || px > 2*MATCHMAP_XS-2 || py < 1 || py > 2*MATCHMAP_YS-2 || pz < MATCHER_MAX_THREADS || pz > 2*MATCHMAP_ZS-2)
			{
				printf("cloud_to_ref_matchmap: WARN, OOR point %d,%d,%d\n", px, py, pz);
				continue;
			}

			// Remove free at highest bold level
			for(int ix=-1; ix<=1; ix++)
				for(int iy=-1; iy<=1; iy++)
					matchmap->units[py/2+iy][px/2+ix].free &= ~(0b111ULL << (pz/2-1));

		#else
			int px = mmpx/MATCHMAP_UNIT + MATCHMAP_XS/2;
			int py = mmpy/MATCHMAP_UNIT + MATCHMAP_YS/2;
			int pz = mmpz/MATCHMAP_UNIT + MATCHMAP_ZS/2;

			// For every matcher thread, reference map is shifted >>1
			if(px < 1 || px > MATCHMAP_XS-2 || py < 1 || py > MATCHMAP_YS-2 || pz < MATCHER_MAX_THREADS || pz > MATCHMAP_ZS-2)
			{
				printf("cloud_to_ref_matchmap: WARN, OOR point %d,%d,%d\n", px, py, pz);
				continue;
			}

			// Remove free at highest bold level
			for(int ix=-1; ix<=1; ix++)
				for(int iy=-1; iy<=1; iy++)
					matchmap->units[py+iy][px+ix].free &= ~(0b111ULL << (pz-1));

		#endif

		#if REF_BOLD_MODE==0 // Create voxels directly, no bolding
			matchmap->units[py][px].occu |= 1ULL << (pz);

		#elif REF_BOLD_MODE==1 // Create four voxels, by adding one on each axis (x,y,z) to the side nearer the actual voxel
			matchmap->units[py/2][px/2].occu |= 1ULL << (pz/2);

			matchmap->units[py/2 - 1 + 2*(py%2)][px/2].occu |= 1ULL << (pz/2);

			matchmap->units[py/2][px/2 - 1 + 2*(px%2)].occu |= 1ULL << (pz/2);

			matchmap->units[py/2][px/2].occu |= 1ULL << (pz/2 - 1 + 2*(pz%2));

		#elif REF_BOLD_MODE==2 // Similarly, but create 8 voxels, a 2x2x2 cube biased correctly

			// comment 1 = use the neighbor on this axis (has the math thing in the index), 0 = actual voxel on this axis
			// y x z
			// 0 0 0
			matchmap->units[py/2][px/2].occu |= 1ULL << (pz/2);

			// 0 0 1
			matchmap->units[py/2][px/2].occu |= 1ULL << (pz/2 - 1 + 2*(pz%2));

			// 0 1 0
			matchmap->units[py/2][px/2 - 1 + 2*(px%2)].occu |= 1ULL << (pz/2);

			// 0 1 1
			matchmap->units[py/2][px/2 - 1 + 2*(px%2)].occu |= 1ULL << (pz/2 - 1 + 2*(pz%2));

			// 1 0 0
			matchmap->units[py/2 - 1 + 2*(py%2)][px/2].occu |= 1ULL << (pz/2);

			// 1 0 1
			matchmap->units[py/2 - 1 + 2*(py%2)][px/2].occu |= 1ULL << (pz/2 - 1 + 2*(pz%2));

			// 1 1 0
			matchmap->units[py/2 - 1 + 2*(py%2)][px/2 - 1 + 2*(px%2)].occu |= 1ULL << (pz/2);

			// 1 1 1
			matchmap->units[py/2 - 1 + 2*(py%2)][px/2 - 1 + 2*(px%2)].occu |= 1ULL << (pz/2 - 1 + 2*(pz%2));

		
		#elif REF_BOLD_MODE==3 // Create extra voxel in every neighbor, i.e., create a cube of 3x3x3 = 27 voxels
			for(int ix=-1; ix<=1; ix++)
				for(int iy=-1; iy<=1; iy++)
					matchmap->units[py+iy][px+ix].occu |= 0b111ULL << (pz-1);
		#else
			#error set REF_BOLD_MODE correctly
		#endif
	}
}



static inline int uint32_log2(uint32_t in)
{
	int r = 0;
	while(in >>= 1)
		r++;

	return r;
}


static void cloud_to_ref_fine_matchmap_free(cloud_t* cloud, ref_fine_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
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

#define REF_FINE_BOLD_MODE 3

static void cloud_to_ref_fine_matchmap_occu(cloud_t* cloud, ref_fine_matchmap_t* matchmap, float x_corr, float y_corr, int z_corr, float yaw_corr)
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

		#if REF_FINE_BOLD_MODE==1 || REF_FINE_BOLD_MODE==2
			int px = mmpx/(FINE_MATCHMAP_UNIT/2) + FINE_MATCHMAP_XS;
			int py = mmpy/(FINE_MATCHMAP_UNIT/2) + FINE_MATCHMAP_YS;
			int pz = mmpz/(FINE_MATCHMAP_UNIT/2) + FINE_MATCHMAP_ZS;

			if(px < 1 || px > 2*FINE_MATCHMAP_XS-2 || py < 1 || py > 2*FINE_MATCHMAP_YS-2 || pz < MATCHER_MAX_THREADS || pz > 2*FINE_MATCHMAP_ZS-2)
			{
				oor++;
				continue;
			}

			// Remove free at highest bold level
			for(int ix=-1; ix<=1; ix++)
				for(int iy=-1; iy<=1; iy++)
					matchmap->units[py/2+iy][px/2+ix].free &= ~(0b111ULL << (pz/2-1));

		#else
			int px = mmpx/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_XS/2;
			int py = mmpy/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_YS/2;
			int pz = mmpz/FINE_MATCHMAP_UNIT + FINE_MATCHMAP_ZS/2;

			if(px < 1 || px > FINE_MATCHMAP_XS-2 || py < 1 || py > FINE_MATCHMAP_YS-2 || pz < MATCHER_MAX_THREADS || pz > FINE_MATCHMAP_ZS-2)
			{
				oor++;
				continue;
			}

			// Remove free at highest bold level
			for(int ix=-1; ix<=1; ix++)
				for(int iy=-1; iy<=1; iy++)
					matchmap->units[py+iy][px+ix].free &= ~(0b111ULL << (pz-1));

		#endif

		#if REF_FINE_BOLD_MODE==0 // Create voxels directly, no bolding
			matchmap->units[py][px].occu |= 1ULL << (pz);

		#elif REF_FINE_BOLD_MODE==1 // Create four voxels, by adding one on each axis (x,y,z) to the side nearer the actual voxel
			matchmap->units[py/2][px/2].occu |= 1ULL << (pz/2);

			matchmap->units[py/2 - 1 + 2*(py%2)][px/2].occu |= 1ULL << (pz/2);

			matchmap->units[py/2][px/2 - 1 + 2*(px%2)].occu |= 1ULL << (pz/2);

			matchmap->units[py/2][px/2].occu |= 1ULL << (pz/2 - 1 + 2*(pz%2));

		#elif REF_FINE_BOLD_MODE==2 // Similarly, but create 8 voxels, a 2x2x2 cube biased correctly

			// comment 1 = use the neighbor on this axis (has the math thing in the index), 0 = actual voxel on this axis
			// y x z
			// 0 0 0
			matchmap->units[py/2][px/2].occu |= 1ULL << (pz/2);

			// 0 0 1
			matchmap->units[py/2][px/2].occu |= 1ULL << (pz/2 - 1 + 2*(pz%2));

			// 0 1 0
			matchmap->units[py/2][px/2 - 1 + 2*(px%2)].occu |= 1ULL << (pz/2);

			// 0 1 1
			matchmap->units[py/2][px/2 - 1 + 2*(px%2)].occu |= 1ULL << (pz/2 - 1 + 2*(pz%2));

			// 1 0 0
			matchmap->units[py/2 - 1 + 2*(py%2)][px/2].occu |= 1ULL << (pz/2);

			// 1 0 1
			matchmap->units[py/2 - 1 + 2*(py%2)][px/2].occu |= 1ULL << (pz/2 - 1 + 2*(pz%2));

			// 1 1 0
			matchmap->units[py/2 - 1 + 2*(py%2)][px/2 - 1 + 2*(px%2)].occu |= 1ULL << (pz/2);

			// 1 1 1
			matchmap->units[py/2 - 1 + 2*(py%2)][px/2 - 1 + 2*(px%2)].occu |= 1ULL << (pz/2 - 1 + 2*(pz%2));

		
		#elif REF_FINE_BOLD_MODE==3 // Create extra voxel in every neighbor, i.e., create a cube of 3x3x3 = 27 voxels
			for(int ix=-1; ix<=1; ix++)
				for(int iy=-1; iy<=1; iy++)
					matchmap->units[py+iy][px+ix].occu |= 0b111ULL << (pz-1);
		#else
			#error set REF_FINE_BOLD_MODE correctly
		#endif

	}

	if(oor > cloud->n_points/20)
	{
		printf("WARN: cloud_to_ref_fine_matchmap_occu: more than 5%% out-of-range voxels\n");
	}
}




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


	ref_quality_t ret;

	float xy_ratio = (float)smaller_xy/(float)larger_xy;
	float z_ratio = (float)ztot/(float)larger_xy;

	float xy_ratio_satur = xy_ratio*1.4; 
	if(xy_ratio_satur > 1.0)
		xy_ratio_satur = 1.0;

	float z_ratio_inv = (z_ratio>1.0) ? (1.0/z_ratio) : z_ratio;

	float z_ratio_inv_satur = z_ratio_inv*1.4;
	if(z_ratio_inv_satur > 1.0)
		z_ratio_inv_satur = 1.0;

	ret.xtot = xtot;
	ret.ytot = ytot;
	ret.ztot = ztot;

	// Good counts are around 2000
	// Really bad counts are around 200
	// Log2 makes the differences too small.
	// sqrt sounds allright.
	// sqrt(2000) = 44, sqrt(150) = 12

	ret.quality = 
		(sqrt(pos_x_cnt) + sqrt(neg_x_cnt) +
		sqrt(pos_y_cnt) + sqrt(neg_y_cnt) +
		sqrt(pos_z_cnt) + sqrt(neg_z_cnt) - 6.0*12.0) * 80.0 *
		(xy_ratio_satur*z_ratio_inv_satur);

		
	ret.xy_ratio = xy_ratio;

/*
	printf("lin: X+ > %8d   X- < %8d   Y+ ^ %8d   Y- v %8d   Z+ ceil %8d   Z- flr %8d\n",
		pos_x_cnt, neg_x_cnt, pos_y_cnt, neg_y_cnt, pos_z_cnt, neg_z_cnt);
	printf("sqr: X+ > %8.1f   X- < %8.1f   Y+ ^ %8.1f   Y- v %8.1f   Z+ ceil %8.1f   Z- flr %8.1f\n",
		sqrt(pos_x_cnt), sqrt(neg_x_cnt), sqrt(pos_y_cnt), sqrt(neg_y_cnt), sqrt(pos_z_cnt), sqrt(neg_z_cnt));

	printf("xy_ratio=%.3f z_ratio=%.3f  xy_ratio_satur=%.3f  z_ratio_inv_satur=%.3f quality=%d\n",
		xy_ratio, z_ratio, xy_ratio_satur, z_ratio_inv_satur, ret.quality);
*/
//	if(ret.quality > 10000)
//		ret.quality = 10000;


	return ret;
}



/*
	ca = submap before our submap of interest
	cb = submap of interest
	cc = next submap after our submap of interest
	corrab = transform from ca to cb for smooth joining (based on adjacent match, for example)
	corrcb = transform from cb to cc, similarly

*/
ref_quality_t calc_ref_quality_3sm(cloud_t* ca, cloud_t* cb, cloud_t* cc, result_t corrab, result_t corrbc)
{

/*
	Tested how many operations (step sizes) are needed, to get the same result.
	Compared over the full development dataset (173 submaps):

	test1: -45.0 ; +45.1 ; 7.5   -7.5 ; +7.5 ; 2.5  13+7 = 20 ops  reference
	test2: -40   ; +40.1 ; 10.0  -7.5 ; +7.5 ; 2.5  9+7 = 16 ops   same as ref -> use this
	test3: -40   ; +40.1 ; 10.0  -5.0 ; +5.0 ; 2.5  9+5 = 14 ops   different result
	test4: -37.5 ; +37.6 ; 12.5  -7.5 ; +7.5 ; 2.5  7+7 = 14 ops   same as ref, but let's not go this far.
*/
	float smallest = 999.9;
	float smallest_a = 0.0;
	// See Screenshot_2019-06-20_18-41-53.png showing how the combined ref_matchmap looks like.
	for(float a=DEGTORAD(-40.0); a<DEGTORAD(40.1); a+=DEGTORAD(10.0))
	{
		memset(&ref_matchmap, 0, sizeof(ref_matchmap));

		// Join ca and cc to the cb.
		// corrab is transform from a to b: apply it in inverse to go from b to a.
		// Note that parameter A rotates the whole thing, so the translation parameters must be rotated as well, hence the sin, cos stuff.
		// ca first, in inverse transform:
		decim_cloud_to_ref_matchmap_free(ca, &ref_matchmap, 
			- corrab.x*cos(a) + corrab.y*sin(a), // X translation to place ca correctly relative to cb
			- corrab.x*sin(a) - corrab.y*cos(a), // Y translation
			-corrab.z, // Z translation
			-corrab.yaw + a); // yaw rotation

		// cb as-is:
		decim_cloud_to_ref_matchmap_free(cb, &ref_matchmap, 0,0,0, a);

		// cc last, transformed:
		decim_cloud_to_ref_matchmap_free(cc, &ref_matchmap, 
			+ corrbc.x*cos(a) - corrbc.y*sin(a), // X translation to place cc correctly relative to cb
			+ corrbc.x*sin(a) + corrbc.y*cos(a), // Y translation
			+corrbc.z, // Z translation
			+corrbc.yaw + a); // yaw rotation


		// Exact same thing for occupied space:

		decim_cloud_to_ref_matchmap_occu(ca, &ref_matchmap, 
			- corrab.x*cos(a) + corrab.y*sin(a), // X translation to place ca correctly relative to cb
			- corrab.x*sin(a) - corrab.y*cos(a), // Y translation
			-corrab.z, // Z translation
			-corrab.yaw + a); // yaw rotation

		decim_cloud_to_ref_matchmap_occu(cb, &ref_matchmap, 0,0,0, a);

		decim_cloud_to_ref_matchmap_occu(cc, &ref_matchmap, 
			+ corrbc.x*cos(a) - corrbc.y*sin(a), // X translation to place cc correctly relative to cb
			+ corrbc.x*sin(a) + corrbc.y*cos(a), // Y translation
			+corrbc.z, // Z translation
			+corrbc.yaw + a); // yaw rotation

		// Test code:
		#if 0
			static int dbg_y = 0;
			po_coords_t po;
			po = po_coords(0,dbg_y,0, 0);
			load_pages(1,1,  po.px-1, po.px+3, po.py-3, po.py+3, po.pz-1, po.pz+2);

			ref_matchmap_to_voxmap(&ref_matchmap, 0, dbg_y, 0);
			dbg_y -= 4096;

			store_all_pages();

		#endif

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
			-corrab.x - corrab.x*cos(-corrab.yaw + a) + corrab.y*sin(-corrab.yaw + a), // X translation to place ca correctly relative to cb
			-corrab.y - corrab.x*sin(-corrab.yaw + a) - corrab.y*cos(-corrab.yaw + a), // Y translation
			-corrab.z, // Z translation
			-corrab.yaw + a); // yaw rotation

		decim_cloud_to_ref_matchmap_free(cb, &ref_matchmap, 0,0,0, a);

		decim_cloud_to_ref_matchmap_free(cc, &ref_matchmap, 
			+corrbc.x + corrbc.x*cos(+corrbc.yaw + a) - corrbc.y*sin(+corrbc.yaw + a), // X translation to place cc correctly relative to cb
			+corrbc.y + corrbc.x*sin(+corrbc.yaw + a) + corrbc.y*cos(+corrbc.yaw + a), // Y translation
			+corrbc.z, // Z translation
			+corrbc.yaw + a); // yaw rotation

		decim_cloud_to_ref_matchmap_occu(ca, &ref_matchmap, 
			-corrab.x - corrab.x*cos(-corrab.yaw + a) + corrab.y*sin(-corrab.yaw + a), // X translation to place ca correctly relative to cb
			-corrab.y - corrab.x*sin(-corrab.yaw + a) - corrab.y*cos(-corrab.yaw + a), // Y translation
			-corrab.z, // Z translation
			-corrab.yaw + a); // yaw rotation

		decim_cloud_to_ref_matchmap_occu(cb, &ref_matchmap, 0,0,0, a);

		decim_cloud_to_ref_matchmap_occu(cc, &ref_matchmap, 
			+corrbc.x + corrbc.x*cos(+corrbc.yaw + a) - corrbc.y*sin(+corrbc.yaw + a), // X translation to place cc correctly relative to cb
			+corrbc.y + corrbc.x*sin(+corrbc.yaw + a) + corrbc.y*cos(+corrbc.yaw + a), // Y translation
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

	//printf("----> ratio %.2f, uncertain_angle = %.1f deg\n", q_best.xy_ratio, RADTODEG(q_best.uncertain_angle));

	return q_best;
}


int save_ref_matchmap(int idxb, cloud_t* ca, cloud_t* cb, cloud_t* cc, result_t corrab, result_t corrbc)
{
	memset(&ref_matchmap, 0, sizeof(ref_matchmap));

	cloud_to_ref_matchmap_free(ca, &ref_matchmap, -corrab.x, -corrab.y, -corrab.z, -corrab.yaw);
	cloud_to_ref_matchmap_free(cb, &ref_matchmap, 0,0,0, 0);
	cloud_to_ref_matchmap_free(cc, &ref_matchmap, +corrbc.x, +corrbc.y, +corrbc.z, +corrbc.yaw);

	cloud_to_ref_matchmap_occu(ca, &ref_matchmap, -corrab.x, -corrab.y, -corrab.z, -corrab.yaw);
	cloud_to_ref_matchmap_occu(cb, &ref_matchmap, 0,0,0, 0);
	cloud_to_ref_matchmap_occu(cc, &ref_matchmap, +corrbc.x, +corrbc.y, +corrbc.z, +corrbc.yaw);

	// Test code:
	#if 0
	{
		static int dbg_y = 0;
		po_coords_t po;
		po = po_coords(0,dbg_y,0, 0);
		load_pages(1,1,  po.px-1, po.px+3, po.py-3, po.py+3, po.pz-1, po.pz+2);

		ref_matchmap_to_voxmap(&ref_matchmap, 0, dbg_y, 0);
		dbg_y -= 4096;
		store_all_pages();
	}
	#endif

	char fname[1024];
	snprintf(fname, 1024, "ref_matchmap_%06u.bin", idxb);
	FILE* f = fopen(fname, "wb");
	assert(f);

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
		strm.avail_in = sizeof(ref_matchmap_t);
		strm.next_in = (uint8_t*)&ref_matchmap;

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
		assert(fwrite(&ref_matchmap, sizeof(ref_matchmap_t), 1, f) == 1);
	#endif

	fclose(f);
	return 0;
}

int save_ref_fine_matchmap(int idxb, cloud_t* ca, cloud_t* cb, cloud_t* cc, result_t corrab, result_t corrbc)
{

	memset(&ref_fine_matchmap, 0, sizeof(ref_fine_matchmap));

	cloud_to_ref_fine_matchmap_free(ca, &ref_fine_matchmap, -corrab.x, -corrab.y, -corrab.z, -corrab.yaw);
	cloud_to_ref_fine_matchmap_free(cb, &ref_fine_matchmap, 0,0,0, 0);
	cloud_to_ref_fine_matchmap_free(cc, &ref_fine_matchmap, +corrbc.x, +corrbc.y, +corrbc.z, +corrbc.yaw);

	cloud_to_ref_fine_matchmap_occu(ca, &ref_fine_matchmap, -corrab.x, -corrab.y, -corrab.z, -corrab.yaw);
	cloud_to_ref_fine_matchmap_occu(cb, &ref_fine_matchmap, 0,0,0, 0);
	cloud_to_ref_fine_matchmap_occu(cc, &ref_fine_matchmap, +corrbc.x, +corrbc.y, +corrbc.z, +corrbc.yaw);


	// Test code:
	#if 0
	{
		static int dbg_y = 0;
		po_coords_t po;
		po = po_coords(0,dbg_y,0, 0);
		load_pages(1,1,  po.px-1, po.px+3, po.py-3, po.py+3, po.pz-1, po.pz+2);

		ref_fine_matchmap_to_voxmap(&ref_fine_matchmap, 0, dbg_y, 0);
		dbg_y -= 8192;
		store_all_pages();
	}
	#endif


	char fname[1024];
	snprintf(fname, 1024, "ref_fine_matchmap_%06u.bin", idxb);
	FILE* f = fopen(fname, "wb");
	assert(f);

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
		strm.avail_in = sizeof(ref_fine_matchmap_t);
		strm.next_in = (uint8_t*)&ref_fine_matchmap;

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
		assert(fwrite(&ref_fine_matchmap, sizeof(ref_fine_matchmap_t), 1, f) == 1);
	#endif

	fclose(f);
	return 0;

}

/*
	idxb = index of the submap of interest, used to generate the filename
	ca = submap before our submap of interest
	cb = submap of interest
	cc = next submap after our submap of interest
	corrab = transform from ca to cb for smooth joining (combination of submap avg delta + adjacent match, for example)
	corrcb = transform from cb to cc, similarly
*/

ref_quality_t gen_save_ref_matchmap_set(int idxb, cloud_t* ca, cloud_t* cb, cloud_t* cc, result_t corrab, result_t corrbc)
{
	ref_quality_t quality = calc_ref_quality_3sm(ca, cb, cc, corrab, corrbc);
	//	printf("idxb = %3d, quality = %+05d\n", idxb, quality.quality);
	save_ref_matchmap(idxb, ca, cb, cc, corrab, corrbc);
	save_ref_fine_matchmap(idxb, ca, cb, cc, corrab, corrbc);
	return quality;
}


int load_ref_matchmap(int idxb, ref_matchmap_t* mm)
{
	memset(mm, 0, sizeof(ref_matchmap_t));

	char fname[1024];
	snprintf(fname, 1024, "ref_matchmap_%06u.bin", idxb);
	FILE* f = fopen(fname, "rb");
	assert(f);

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
		int bytes_left = sizeof(ref_matchmap_t);
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
				strm.next_out = (uint8_t*)mm + got_bytes;

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
		assert(fread(mm, sizeof(ref_matchmap_t), 1, f) == 1);
	#endif


	fclose(f);

	return 0;

}

int load_ref_fine_matchmap(int idxb, ref_fine_matchmap_t* mm)
{
	memset(mm, 0, sizeof(ref_fine_matchmap_t));

	char fname[1024];
	snprintf(fname, 1024, "ref_fine_matchmap_%06u.bin", idxb);
	FILE* f = fopen(fname, "rb");
	assert(f);

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
		int bytes_left = sizeof(ref_fine_matchmap_t);
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
				strm.next_out = (uint8_t*)mm + got_bytes;

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
		assert(fread(mm, sizeof(ref_fine_matchmap_t), 1, f) == 1);
	#endif


	fclose(f);

	return 0;
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
//#define DBG_OUTPUT_MATCHMAPS

int match_submaps( int i_ref,  // reference submap index, to load the correct ref_matchmap file
                   cloud_t* smb,  // cmp submap
                   double xy_range, double z_range, double yaw_range, // Matching correction ranges (mm, rad)
                   int dx, int dy, int dz, // World coordinate midpoint differences between ref and cmp
                   result_t* results_out)
{
	double start_time = subsec_timestamp();

	memset(results_out, 0, sizeof(result_t)*N_MATCH_RESULTS);

	load_ref_matchmap(i_ref, &ref_matchmap);
	load_ref_fine_matchmap(i_ref, &ref_fine_matchmap);


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
	double yaw_step = DEGTORAD(2.5);

	// Round yaw_range up so that midpoint is at zero.
	yaw_range = ceil(yaw_range/yaw_step) * yaw_step;
	int yaw_steps = ceil(2.0*yaw_range/yaw_step) + 1;


	int total_results = yaw_steps * xy_batches * xy_batches * total_z_steps * MATCHMAP_XYZ_N_RESULTS;

	result_t* results = calloc(total_results, sizeof(result_t));	
	assert(results);

	//printf("xy_batch=%d, tot_z_step=%d (%d batch, thread %d(+%d)) --> ", xy_batches, total_z_steps, z_batches, threads_at_least, remaining_threads); fflush(stdout);

	#ifdef DBG_OUTPUT_MATCHMAPS
		static int dbg_y = 0;
		int dbg_x = 0;
		po_coords_t po;
		po = po_coords(0,dbg_y,0, 0);
		load_pages(1,1,  po.px-1, po.px+3, po.py-3, po.py+3, po.pz-1, po.pz+2);

		ref_matchmap_to_voxmap(&ref_matchmap, 0, dbg_y, 0);
	#endif


	for(int cur_yaw_step = 0; cur_yaw_step < yaw_steps; cur_yaw_step++)
	{
		int32_t best_score_now = INT32_MIN;
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
					//	printf("SKIP 1\n");
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
					//	printf("SKIP 2\n");
						goto SKIP_BATCH;
					}

					n_vox += cloud_to_cmp_matchmap_odd(smb, &cmp_matchmap, cx, cy, cz, cur_yaw_corr);

					#ifdef DBG_OUTPUT_MATCHMAPS
						dbg_x += 4096;
						po_coords_t po;
						po = po_coords(dbg_x,dbg_y,0, 0);
						load_pages(1,1,  po.px-1, po.px+3, po.py-3, po.py+3, po.pz-1, po.pz+2);

						cmp_matchmap_to_voxmap(&cmp_matchmap, dbg_x, dbg_y, 0);
					#endif



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
							if(cur_rel_score > best_score_now)
								best_score_now = cur_rel_score;
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

		// For this yaw, all x,y,z batches were skipped (best_score_now == INT32_MIN),
		// or best score was very low - we are very far off. Skip a few yaw rounds.
		if(best_score_now < 2000)
			cur_yaw_step++;

		if(best_score_now < 1500)
			cur_yaw_step++;

		if(best_score_now < 1000)
			cur_yaw_step++;

		if(best_score_now < 0)
			cur_yaw_step++;

	}

	#ifdef DBG_OUTPUT_MATCHMAPS
		dbg_y -= 4096;
		store_all_pages();
	#endif

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
				const double yaw_fine_start = DEGTORAD(-3.00);
				const int n_yaw_fine_steps = 9;

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

	return n_results;
}

#define FINE_MATCH_YAW_STEP  DEGTORAD(0.75)
#define FINE_MATCH_YAW_START DEGTORAD(-3.75)
#define FINE_MATCH_YAW_STEPS 13



int  fine_match_submaps(cloud_t* sma, cloud_t* smb,
                        int dx, int dy, int dz, // World coordinate midpoint differences between ref and cmp
                        result_t* results_out)
{
	assert(results_out);

	static result_t results[FINE_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS];
	memset(results, 0, sizeof(result_t)*FINE_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS);	

	// SMA is ref_fine_matchmap
	// SMB is cmp_fine_matchmap

	// ref_fine_matchmap includes free space, so is slower to calculate
	// cmp_fine_matchmap only includes occupied space, and is compared against ref_fine_matchmap quickly



	memset(&ref_fine_matchmap, 0, sizeof(ref_fine_matchmap));
	cloud_to_ref_fine_matchmap_free(sma, &ref_fine_matchmap, 0,0,0,0);
	cloud_to_ref_fine_matchmap_occu(sma, &ref_fine_matchmap, 0,0,0,0);

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

		int n_vox = cloud_to_cmp_fine_matchmap(smb, &cmp_fine_matchmap, cx, cy, cz, cur_yaw_corr);

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
//	printf("total results (from %d pcs):\n", FINE_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS);
	for(int i=0; i<FINE_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS; i++)
	{
		if(results[i].score > 0 && results[i].score > results[0].score/2)
		{
//			printf("i=%5d  (%+6d,%+6d,%+6d,%+6.2f) relscore=%+5d abscore=%+5d\n",
//				i, results[i].x, results[i].y, results[i].z, RADTODEG(results[i].yaw), results[i].score, results[i].abscore);


			results_out[n_results] = results[i];
			n_results++;
			if(n_results >= N_FINE_MATCH_RESULTS)
				break;
		}
	}

	return n_results;
}

/*
	For simplistic, classic localization on existing map
	Works as a SLAM in easy conditions - fast, no loop closures, just matches a submap to the existing voxmap world.
*/
#define CLASSIC_MATCH_YAW_STEP  DEGTORAD(0.75)
#define CLASSIC_MATCH_YAW_START DEGTORAD(-5.25)

#define CLASSIC_MATCH_YAW_STEPS 11
static const double classic_yaws[CLASSIC_MATCH_YAW_STEPS] = 
{
	DEGTORAD(-7.75), DEGTORAD(-5.25), DEGTORAD(-3.25), DEGTORAD(-1.75), DEGTORAD(-0.75),
	DEGTORAD(0),
	DEGTORAD(0.75), DEGTORAD(1.75), DEGTORAD(3.25), DEGTORAD(5.25), DEGTORAD(7.75)
};


result_t match_submap_to_voxmap(cloud_t* sm, int32_t ref_x, int32_t ref_y, int32_t ref_z)
{
	static result_t results[CLASSIC_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS];
	memset(results, 0, sizeof(result_t)*CLASSIC_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS);	

	memset(&ref_fine_matchmap, 0, sizeof(ref_fine_matchmap));
	voxmap_to_ref_fine_matchmap(&ref_fine_matchmap, ref_x, ref_y, ref_z);

	// TODO: These batches could run in parallel, in multiple threads
	for(int cur_yaw_step = 0; cur_yaw_step < CLASSIC_MATCH_YAW_STEPS; cur_yaw_step++)
	{
		double cur_yaw_corr = classic_yaws[cur_yaw_step];
//		double cur_yaw_corr = 0.0;
		// This part is per-thread.

		memset(&cmp_fine_matchmap, 0, sizeof(cmp_fine_matchmap));

		int n_vox = cloud_to_cmp_fine_matchmap(sm, &cmp_fine_matchmap, 0, 0, 0, cur_yaw_corr);

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

	qsort(results, CLASSIC_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS, sizeof(result_t), compar_scores);

	#define CLASSIC_POSTFILTER_RESULTS

	#ifdef CLASSIC_POSTFILTER_RESULTS
		for(int i=0; i<CLASSIC_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS-1; i++)
		{
			for(int o=i+1; o<CLASSIC_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS; o++)
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
	printf("total results (from %d pcs):\n", CLASSIC_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS);
	for(int i=0; i<CLASSIC_MATCH_YAW_STEPS*MATCHMAP_XYZ_N_RESULTS; i++)
	{
		if(results[i].score > 0 && results[i].score > results[0].score/2)
		{
			printf("i=%5d  (%+6d,%+6d,%+6d,%+6.2f) relscore=%+5d abscore=%+5d\n",
				i, results[i].x, results[i].y, results[i].z, RADTODEG(results[i].yaw), results[i].score, results[i].abscore);

			//results_out[n_results] = results[i];
			n_results++;
			if(n_results >= N_FINE_MATCH_RESULTS)
				break;
		}
	}


	if(n_results == 0)
	{
		return (result_t){0};
	}
	else if(n_results == 1)
	{
		return results[0];
	}
	else
	{
		double ratio = (double)results[1].score/(double)results[0].score; // always < 1.0

		double coeff0, coeff1;
		if(ratio > 0.3)
		{
			ratio = ratio*ratio*ratio;
			ratio /= 2.0; // max 0.5
			coeff1 = ratio;
			coeff0 = 1.0 - ratio;

			return (result_t){
				coeff0*results[0].x + coeff1*results[1].x,
				coeff0*results[0].y + coeff1*results[1].y,
				coeff0*results[0].z + coeff1*results[1].z,
				coeff0*results[0].yaw + coeff1*results[1].yaw,
				results[0].score,
				results[0].abscore};

		}
		else
		{
			return results[0];
		}

	}

}





