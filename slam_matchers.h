#pragma once

// Maximum possible number of large scale loop closure scan matching results. May return fewer.
#define N_MATCH_RESULTS 8

#define N_FINE_MATCH_RESULTS 16


typedef struct
{
	int32_t x;
	int32_t y;
	int32_t z;
	float yaw;

	int32_t score;
	int32_t abscore;
} result_t;

static int compar_scores(const void* a, const void* b)
{
	if(((result_t*)a)->score > ((result_t*)b)->score)
		return -1;
	else if(((result_t*)a)->score < ((result_t*)b)->score)
		return 1;

	return 0;
}




int match_submaps(int n_sma, int* i_sma, result_t* sma_corrs, // Number of ref submaps, ref submap indeces, and relative corrections compared to the ref midpoint (which is again related to dx,dy,dz)
                   int i_smb,  // cmp submap index
                   double xy_range, double z_range, double yaw_range, // Matching correction ranges (mm, rad)
                   int dx, int dy, int dz, // World coordinate midpoint differences between ref and cmp
                   result_t* results_out);

int  fine_match_submaps(cloud_t* sma, cloud_t* smb,
                        int dx, int dy, int dz, // World coordinate midpoint differences between ref and cmp
                        result_t* results_out);


