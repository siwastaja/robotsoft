#pragma once

#define ICP_USE_A  1
#define ICP_USE_B  2 
#define ICP_USE_COMB 0
int match_by_closest_points(cloud_t * cloud_a, cloud_t * cloud_b, cloud_t * combined_cloud_out,
	float x_corr, float y_corr, float z_corr, float yaw_corr,
	int max_iters,
	float match_threshold_mm, float combine_threshold_mm, float perpendicular_combine_threshold_mm,
	float perp_combine_threshold_mm_extra_per_mm,
	float* x_corr_out, float* y_corr_out, float* z_corr_out, float* yaw_corr_out, int use);

