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


typedef struct __attribute__((packed))
{
	/*
		Ratio between objects seen from optimally chosen imaginary axes 90 degrees apart.

		#########################
		.........................
		.........................
		.........................
		#########################

		0%, uncertain_angle = 0 (or pi, or -pi)

		#......#
		  #......#
		    #......#
		      #......#
		        #......#
		          #......#

		0% as well, uncertain_angle = -1/4 pi (or 3/4 pi)

		                        #
		#########################.
		...........................
		............................
		...........#................
		############################

		maybe 5%, uncertain_angle still 0

		       #.....#
		       #.....#
		########.....###########
		........................
		........................
		........................
		##########.....#########
		         #.....#
		         #.....#
		         #.....#

		Near 100%, uncertain_angle irrelevant

		uncertain_angle can be used to weight scores along the uncertain axis when xy_ratio is far below 100%.

		Note that 100% ratio seldom happens, and 50% ratio is quite good already for matching without tricks like this.

		But if the ratio is somewhere around 10%, false matches are very likely.

	*/
	int32_t xtot;
	int32_t ytot;
	int32_t ztot;
	float xy_ratio; 
	float uncertain_angle;
	int32_t quality; // Combined metric, 0 - 10000 (approximately)
} ref_quality_t;




int match_submaps( int i_ref,  // reference submap index, to load the correct ref_matchmap file
                   cloud_t* smb,  // cmp submap
                   double xy_range, double z_range, double yaw_range, // Matching correction ranges (mm, rad)
                   int dx, int dy, int dz, // World coordinate midpoint differences between ref and cmp
                   result_t* results_out);

int  fine_match_submaps(cloud_t* sma, cloud_t* smb,
                        int dx, int dy, int dz, // World coordinate midpoint differences between ref and cmp
                        result_t* results_out);

ref_quality_t gen_save_ref_matchmap_set(int idxb, cloud_t* ca, cloud_t* cb, cloud_t* cc, result_t corrab, result_t corrbc);



