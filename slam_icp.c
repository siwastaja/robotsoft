#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "slam_cloud.h"
#include "slam_icp.h"

#include "misc.h"

/*
	Iterative closest point algorithm
	cloud_b is first transformed by x_corr, y_corr, z_corr, yaw_corr
	Closest points within threshold are associated, and transform between them calculated
*/


typedef struct
{
	float x;
	float y;
	float z;
} floatpoint_t;

typedef int16_t kd_unit_t;
typedef int32_t kd_unit_tmpsq_t;
#define KD_UNIT_TMPSQ_T_MAX INT32_MAX
#define KD_UNIT_TMPSQ_T_MIN INT32_MIN

typedef struct __attribute__((packed)) kd_node_struct
{
	cloud_point_t point;
	struct kd_node_struct *left;
	struct kd_node_struct *right;
} kd_node_t;
 
ALWAYS_INLINE kd_unit_tmpsq_t dist(kd_node_t *a, kd_node_t *b)
{
	return 
		sq((kd_unit_tmpsq_t)a->point.x-(kd_unit_tmpsq_t)b->point.x) +
		sq((kd_unit_tmpsq_t)a->point.y-(kd_unit_tmpsq_t)b->point.y) +
		sq((kd_unit_tmpsq_t)a->point.z-(kd_unit_tmpsq_t)b->point.z);
}

// Swaps the points, but not the pointers:
static inline void swap(kd_node_t * const a, kd_node_t * const b)
{
	cloud_point_t tmp = a->point;
	a->point = b->point;
	b->point = tmp;
}
 
 
// quickselect implementation
// https://rosettacode.org/wiki/K-d_tree#C used as an implementation reference, but beware,
// it's buggy and produces wrong results (see the discussion page)
// ii is current depth in tree
static kd_node_t* find_median(kd_node_t *start, kd_node_t *end, int ii)
{
	if(end <= start)
		return NULL;
 
	int idx = ii%3;
	kd_node_t *p, *store, *md = start + (end - start) / 2;
	kd_unit_t pivot;
	while(1)
	{
		if(end == start + 1)
			return start;

		pivot = md->point.coords[idx];
		swap(md, end - 1);
		for(store = p = start; p < end; p++)
		{
			if(p->point.coords[idx] < pivot)
			{
				if(p != store)
					swap(p, store);
				store++;
			}
		}
		swap(store, end - 1);
 
		if(store == md)
			return md;
 
		if(store > md)
			end = store;
		else
			start = store + 1;
	}
}

// statistics:

//#define KDTREE_STATS

#ifdef KDTREE_STATS
int64_t branch_len_cumul;
int n_branch;
int max_branch_len;
#endif
 
static kd_node_t* make_tree(kd_node_t *t, int len, int i)
{
	kd_node_t *median_node;

	if(!len)
	{
		#ifdef KDTREE_STATS
			if(i>max_branch_len)
				max_branch_len = i;

			n_branch++;
			branch_len_cumul += i;
		#endif
		return 0;
	}
 
	if( (median_node = find_median(t, t + len, i)) )
	{
		i++;
		median_node->left  = make_tree(t, median_node - t, i);
		median_node->right = make_tree(median_node + 1, t + len - (median_node + 1), i);
	}

	return median_node;
}
 
#ifdef KDTREE_STATS
int visited;
#endif

static void nearest_two(kd_node_t *root, kd_node_t *node, int i, kd_node_t **best, kd_unit_tmpsq_t *best_dist, kd_node_t **second_best, kd_unit_tmpsq_t *second_best_dist)
{
	if (!root) return;

	#ifdef KDTREE_STATS
		visited++;
	#endif
	kd_unit_tmpsq_t d = dist(root, node);
	kd_unit_tmpsq_t d_curdir = root->point.coords[i] - node->point.coords[i];
  
	if(d < *best_dist)
	{
		*second_best_dist = *best_dist;
		*second_best = *best;
		*best_dist = d;
		*best = root;
	}
	else if(d < *second_best_dist)
	{
		*second_best_dist = d;
		*second_best = root;
	}
 
	if(++i >= 3)
		i = 0;

	nearest_two(d_curdir > 0 ? root->left : root->right, node, i, best, best_dist, second_best, second_best_dist);
	if(sq(d_curdir) >= *best_dist && sq(d_curdir) >= *second_best_dist)
		return;
	nearest_two(d_curdir > 0 ? root->right : root->left, node, i, best, best_dist, second_best, second_best_dist);

}

static void nearest(kd_node_t *root, kd_node_t *node, int i, kd_node_t **best, kd_unit_tmpsq_t *best_dist)
{
	if (!root) return;

	#ifdef KDTREE_STATS
		visited++;
	#endif
	kd_unit_tmpsq_t d = dist(root, node);
	kd_unit_tmpsq_t d_curdir = root->point.coords[i] - node->point.coords[i];
  
	if(d < *best_dist)
	{
		*best_dist = d;
		*best = root;
	}
 
	if(++i >= 3)
		i = 0;

	nearest(d_curdir > 0 ? root->left : root->right, node, i, best, best_dist);
	if(sq(d_curdir) >= *best_dist)
		return;
	nearest(d_curdir > 0 ? root->right : root->left, node, i, best, best_dist);

}

 




// Z_SPREAD resizes the point z coordinates, strecthing it, for nearest neighbour search purposes.
// The final coordinates are not affected.
// The idea is to exaggerate distances in Z direction, so that the algorithm more preferably picks a
// point from the similar Z level as the closest match, even if the euclidean distance would be a bit
// longer. Z axis is most accurate, and large errors mostly exist on X,Y axes. Using Z_SPREAD > 1
// makes sure there are not too many points where the low wall point is incorrectly matched to the floor
// point near that wall.
#define Z_SPREAD 1



kd_node_t* cloud_to_kd(cloud_t* cloud)
{
	kd_node_t *kd = calloc(cloud->m.n_points, sizeof kd[0]);
	for(int i=0; i<cloud->m.n_points; i++)
	{
		kd[i].point = cloud->points[i];
		kd[i].point.z *= Z_SPREAD;
	}

	return kd;
}

#define combine_verbose 0

// When combining point clouds, each point in b can match multiple times to the same point in a. Normally this is allowed.
// To force 1:1 relationship, define COMBINE_ONE_TO_ONE. Note, this stupidly removes further matches when the point in a is matched for the first time,
// the first match might not be the best possible. If you define COMBINE_ONE_TO_TWO, matching is prevented only if BOTH A_closest and
// A_2ndclosest have both already been matched to another point. This quite well prevents massive clumping where a single B point is matched to a huge
// number of A points.
// 
//#define COMBINE_ONE_TO_ONE
#define COMBINE_ONE_TO_TWO

int match_by_closest_points(cloud_t * cloud_a, cloud_t * cloud_b, cloud_t * combined_cloud_out,
	float x_corr, float y_corr, float z_corr, float yaw_corr,
	int max_iters,
	float match_threshold_mm, float combine_threshold_mm, float perpendicular_combine_threshold_mm,
	float perp_combine_threshold_mm_extra_per_mm,
	float* x_corr_out, float* y_corr_out, float* z_corr_out, float* yaw_corr_out, int use)
{
	int a_n_points = cloud_a->m.n_points;
	int b_n_points = cloud_b->m.n_points;
	int b_n_srcs = cloud_b->m.n_srcs;

	assert(cloud_a->m.n_srcs > 0);
	assert(cloud_b->m.n_srcs > 0);

	double ts;
	ts = subsec_timestamp();
	kd_node_t* kda = cloud_to_kd(cloud_a);
	double t_cloud_to_kd = subsec_timestamp() - ts;

	ts = subsec_timestamp();
	#ifdef KDTREE_STATS
	branch_len_cumul = 0;
	n_branch = 0;
	max_branch_len = -1;
	#endif
	kd_node_t* root = make_tree(kda, a_n_points, 0);
	double t_make_tree = subsec_timestamp() - ts;
	

	#ifdef KDTREE_STATS
	printf("%d branches, avg len = %d, max len = %d\n", n_branch, (int)(branch_len_cumul/(int64_t)n_branch), max_branch_len);
	#endif
	printf("cloud_to_kd %.2fms  build_tree %.2fms\n", t_cloud_to_kd*1000.0, t_make_tree*1000.0);

	double t_total_total = 0.0;

	uint8_t* cloud_b_remove_point = NULL;

	if(combined_cloud_out)
	{
		// We need to mark combined (associated) points in cloud_b, so that we don't copy them to the output again
		// One bit is enough. When '1', the same point index must be removed.
		cloud_b_remove_point = calloc(b_n_points/8+1, 1); // calloc(b_n_points, 1); 

		// using same pointer for cloud_a and combined output is acceptable.
		// In this case, A points are removed first. They are added back from the kdtree (whenever not combined with b points)
		// But, keep the source table of A.
		if(combined_cloud_out == cloud_a)
		{
			cloud_remove_points_keep_sources(combined_cloud_out);
		}
		else
		{
			assert(combined_cloud_out->m.n_points == 0 && combined_cloud_out->m.n_srcs == 0); // combined_cloud_out must be either cloud_a, or empty
		}

	}


	int sq_match_th = sq(match_threshold_mm/(float)CLOUD_MM);
	int sq_combine_th = sq(combine_threshold_mm/(float)CLOUD_MM) + sq(perpendicular_combine_threshold_mm/(float)CLOUD_MM);
	float combine_perp_th = perpendicular_combine_threshold_mm/(float)CLOUD_MM;
	float sq_combine_perp_th = sq(combine_perp_th);
	float perp_reduce_per_unit_len = perp_combine_threshold_mm_extra_per_mm;

	int64_t tx = cloud_b->m.ref_x - cloud_a->m.ref_x;
	int64_t ty = cloud_b->m.ref_y - cloud_a->m.ref_y;
	int64_t tz = cloud_b->m.ref_z - cloud_a->m.ref_z;
	assert(tx > -10000 && tx < 10000);
	assert(ty > -10000 && ty < 10000);
	assert(tz > -10000 && tz < 10000);
	
	int n_associations = 0;
	int n_sole_best=0, n_sole_secbest=0, n_both_best_secbest=0;

	for(int iter=0; iter<max_iters; iter++)
	{
		double t_search;

		floatpoint_t mean_diff = {0,0,0};

		int_fast64_t dot_accum = 0;
		int_fast64_t det_accum = 0;


		ts = subsec_timestamp();

		n_associations = 0;

		#ifdef KDTREE_STATS
			int max_visited = -1;
			int visited_cnt = 0;
			int64_t visited_acc = 0;
		#endif


		for(int bi=0; bi<b_n_points; bi++)
		{
			kd_unit_tmpsq_t best_dist = KD_UNIT_TMPSQ_T_MAX;
			kd_node_t* p_best_a;

			cloud_point_t b_point = transform_point(cloud_b->points[bi], x_corr+(float)tx, y_corr+(float)ty, z_corr+(float)tz, yaw_corr);

			b_point.z *= Z_SPREAD;

			kd_node_t test_node = {b_point, NULL, NULL};

			#ifdef KDTREE_STATS
				visited=0;
			#endif
			nearest(root, &test_node, 0, &p_best_a, &best_dist);

			#ifdef KDTREE_STATS
				visited_cnt++;
				visited_acc += visited;
				if(visited > max_visited)
				{
					max_visited = visited;
				}
			#endif

			// Accumulate the mean transformation
			if(best_dist < sq_match_th)
			{

				//printf("closest to bi%d (%.1f %.1f %.1f) is (%.1f %.1f %.1f)\n", bi,
				//	testNode.coords[0], testNode.coords[1], testNode.coords[2], 
				//	p_best_a->coords[0], p_best_a->coords[1], p_best_a->coords[2]);

				n_associations++;

				mean_diff.x += b_point.x - p_best_a->point.x;
				mean_diff.y += b_point.y - p_best_a->point.y;
				mean_diff.z += b_point.z - p_best_a->point.z;

				int sax = p_best_a->point.x;
				int say = p_best_a->point.y;
				//int saz = p_best_a->point.z;
				int sbx = b_point.x;
				int sby = b_point.y;
				//int sbz = b_point.z;

				int_fast64_t dot = sax*sbx + say*sby;
				int_fast64_t det = sax*sby - say*sbx;

				dot_accum += dot;
				det_accum += det;
			}

		}

		t_search = subsec_timestamp() - ts;

		float yaw = atan2f(det_accum, dot_accum);

		if(n_associations > 0)
		{
			mean_diff.x /= (float)n_associations;
			mean_diff.y /= (float)n_associations;
			mean_diff.z /= (float)n_associations;


			x_corr -= mean_diff.x;
			y_corr -= mean_diff.y;
			z_corr -= mean_diff.z;
			yaw_corr -= yaw;
		}

		printf("ICP iter=%2d points=%6d,%6d,asso=%6d (%3.0f%%,%3.0f%%), transl=(%+6.0f,%+6.0f,%+6.0f), yaw=%.2f deg, corr=(%+6.0f,%+6.0f,%+6.0f; %.2f deg)\n",
			iter, a_n_points, b_n_points, n_associations,
			100.0*(float)n_associations/(float)a_n_points, 100.0*(float)n_associations/(float)b_n_points,
			mean_diff.x, mean_diff.y, mean_diff.z, RADTODEG(yaw), x_corr, y_corr, z_corr, RADTODEG(yaw_corr));

		#ifdef KDTREE_STATS
			printf("visited avg=%d  max=%d\n", (int) ((int64_t)visited_acc/(int64_t)visited_cnt), max_visited);
			printf("Performance:  search %.2f ms  \n\n", t_search*1000.0);
		#endif


		t_total_total += t_search;
	}

	printf("total time = %.1f ms\n", t_total_total*1000.0);

	if(combined_cloud_out)
	{
		assert(b_n_srcs > 0);

		int n_comb_points_with_n_srcs[POINT_MAX_SRCS+1] = {0};

		// For combining clouds: whenever b_cloud.srcs[b_idx] matches (closely enough) a_cloud.srcs[a_idx],
		// matching_srcs[b_idx] := a_idx
		uint8_t* matching_srcs = malloc(b_n_srcs * sizeof matching_srcs[0]);
		assert(matching_srcs);

		// use the same transform the points are going to be tranformed with, to transform 
		// sources (and multisources) of cloud b. Try to find an acceptable match from the
		// existing cloud_a sources (and multisources). If can't, add new sources (and multisources) to cloud a.

		if(combine_verbose >= 3) cloud_print_sources(cloud_a, 0);

		if(combine_verbose >= 3) printf("Begin combining sources, a_n_srcs = %d, b_n_srcs = %d\n", cloud_a->m.n_srcs, b_n_srcs);

		for(int i=0; i<b_n_srcs; i++)
		{
			// cloud_find_source adds a new source for us if suitable is not found existing.
			cloud_point_t trans_src = transform_point(cloud_b->srcs[i], x_corr+(float)tx, y_corr+(float)ty, z_corr+(float)tz, yaw_corr);
			int b_in_a = cloud_find_source(cloud_a, trans_src);

			matching_srcs[i] = b_in_a;

			if(combine_verbose >= 2) printf("Mapping B src %d (%.0f,%.0f,%.0f) transformed (%.0f,%.0f,%.0f) to A src %d (%.0f,%.0f,%.0f)\n",
				i, (float)cloud_b->srcs[i].x*CLOUD_MM, (float)cloud_b->srcs[i].y*CLOUD_MM, (float)cloud_b->srcs[i].z*CLOUD_MM,
				(float)trans_src.x*CLOUD_MM, (float)trans_src.y*CLOUD_MM, (float)trans_src.z*CLOUD_MM,
				b_in_a, (float)cloud_a->srcs[b_in_a].x*CLOUD_MM, (float)cloud_a->srcs[b_in_a].y*CLOUD_MM, (float)cloud_a->srcs[b_in_a].z*CLOUD_MM);

		}

		if(combine_verbose >= 3) printf("Done combining sources.\n");

		if(combine_verbose >= 3) cloud_print_sources(cloud_a, 0);


		for(int bi=0; bi<b_n_points; bi++)
		{
			kd_unit_tmpsq_t best_dist[2] = {KD_UNIT_TMPSQ_T_MAX, KD_UNIT_TMPSQ_T_MAX}; // best and second best
			kd_node_t* p_best_a[2]; // best and second best

			cloud_point_t b_point = transform_point(cloud_b->points[bi], x_corr+(float)tx, y_corr+(float)ty, z_corr+(float)tz, yaw_corr);

			b_point.z *= Z_SPREAD;

			kd_node_t test_node = {b_point, NULL, NULL};

			nearest_two(root, &test_node, 0, &p_best_a[0], &best_dist[0], &p_best_a[1], &best_dist[1]);

			// At this point, b_point is already transformed to the calculated correction (from the ICP).
			// What's left, output either a_points as-is, b_points as-is, or combined (associated)
			// avg(a,b) points.

			if((best_dist[0] < sq_combine_th || best_dist[0] < sq_combine_perp_th) &&
				#ifdef COMBINE_ONE_TO_ONE
				(!(p_best_a[0]->point.removed) && !(p_best_a[1]->point.removed)))
				#endif
				#ifdef COMBINE_ONE_TO_TWO
				(!(p_best_a[0]->point.removed) || !(p_best_a[1]->point.removed)))
				#endif
				#if !defined(COMBINE_ONE_TO_ONE) && !defined(COMBINE_ONE_TO_TWO)
				1)
				#endif

			{
				// Sometimes we want to match the "second best" (euclidean 2nd closest) instead of the euclidean
				// closest point, because the second best lines up with the sensor->point direction vector.
				// See which one is better, and match to it. Most importantly: whichever (or both) of the two points
				// (best and/or second best) have to be removed from A.

				float prdist[2]; // "projection distance" aka. the distance of point A from the line defined by the src_b -> point_b vector.
				cloud_point_t a_point[2];
				int pass[2] = {0,0};
				for(int round=0; round<2; round++)
				{

					a_point[round] = p_best_a[round]->point;
					a_point[round].z /= Z_SPREAD;

					// Calculate projection distance from line defined by vector q := srcb -> pointb, to the associated point_a.
					// Larger projection distance is allowed (if the user wants; see the function args) further away from the sensor;
					// instead of adjusting the threshold value, reduce the projection distance itself.
					// What if we have many sources on point_b? Obviously, calculate the projection distance with each different
					// source and use the best one.

					cloud_point_t w = cloud_point_minus(a_point[round], b_point);

					float min_proj_dist_reduced = 999999999.0f;
					assert(b_point.n_srcs >= 1);
					for(int si=0; si<b_point.n_srcs; si++)
					{
						cloud_point_t src = transform_point(cloud_b->srcs[b_point.srcs[si]], x_corr+(float)tx, y_corr+(float)ty, z_corr+(float)tz, yaw_corr);
						cloud_point_t q = cloud_point_minus(b_point, src);

						float proj_dist = cloud_point_len(cloud_point_cross(w, q)) / cloud_point_len(q);

						float proj_dist_reduced = proj_dist - perp_reduce_per_unit_len*cloud_point_len(q);

						if(proj_dist_reduced < min_proj_dist_reduced)
						{
							min_proj_dist_reduced = proj_dist_reduced;
						}
						
						if(combine_verbose >= 3) printf("round %d: #%dsrc (%d,%d,%d) b_point (%d,%d,%d) q (%d,%d,%d) |q|=%.1f, a_point (%d,%d,%d), w (%d,%d,%d)\n",
							round, si, src.x*CLOUD_MM,src.y*CLOUD_MM,src.z*CLOUD_MM, b_point.x*CLOUD_MM,b_point.y*CLOUD_MM,b_point.z*CLOUD_MM,
							q.x*CLOUD_MM,q.y*CLOUD_MM,q.z*CLOUD_MM, cloud_point_len(q)*CLOUD_MM, a_point[round].x*CLOUD_MM,a_point[round].y*CLOUD_MM,a_point[round].z*CLOUD_MM,
							w.x*CLOUD_MM,w.y*CLOUD_MM,w.z*CLOUD_MM);

						if(combine_verbose >= 3) printf("  dist=%.1f proj_dist=%.1f, reduced=%.1f\n", 
							sqrt(best_dist[round])*CLOUD_MM, proj_dist*CLOUD_MM, proj_dist_reduced*CLOUD_MM);

					}


					prdist[round] = min_proj_dist_reduced;
						
					if(prdist[round] < combine_perp_th && best_dist[round] < sq_combine_th)
						pass[round] = 1;

					if(combine_verbose >= 3) printf("  --> dist=%.1f prdist=%.1f --> round%d %s\n", 
						sqrt(best_dist[round])*CLOUD_MM, prdist[round]*CLOUD_MM, round, pass[round]?"PASS":"fail");

				}

				if(pass[0] || pass[1])
				{
					//printf("        PASS\n");
					n_associations++;

					if(use == ICP_USE_COMB)
					{
						// Average the point coordinates together to build a one new point at the euclidean middle point
						cloud_point_t comb = b_point;

						// comb now has the sources of b. Translate their indeces to the numbering of combined_cloud_out
						for(int i=0; i<comb.n_srcs; i++)
							comb.srcs[i] = matching_srcs[comb.srcs[i]];

						// Choose whether to combine B with best A, 2nd best A, or both.

						if(!pass[1] || prdist[0] < prdist[1]*0.6)
						{ // second-best is useless, or the best one has a lot better projection distance -> use best alone for avg.
							n_sole_best++;
							comb.x += a_point[0].x; comb.x /= 2;
							comb.y += a_point[0].y; comb.y /= 2;
							comb.z += a_point[0].z; comb.z /= 2;

							point_add_src_idxs(&comb, &a_point[0]);
						}
						else if(!pass[0] || prdist[1] < prdist[0]*0.6)
						{ // best is useless, or the second-best one has a lot better projection distance -> use second best alone for avg.
							n_sole_secbest++;
							comb.x += a_point[1].x; comb.x /= 2;
							comb.y += a_point[1].y; comb.y /= 2;
							comb.z += a_point[1].z; comb.z /= 2;

							point_add_src_idxs(&comb, &a_point[1]);
						}
						else // both pass, and are fairly close together
						{ // Average three points together: best A, second best A, and obviously B.
							assert(prdist[0] < combine_perp_th);
							assert(prdist[1] < combine_perp_th);
							assert(best_dist[0] < sq_combine_th);
							assert(best_dist[1] < sq_combine_th);
							n_both_best_secbest++;
							comb.x += a_point[0].x + a_point[1].x; comb.x /= 3;
							comb.y += a_point[0].y + a_point[1].y; comb.y /= 3;
							comb.z += a_point[0].z + a_point[1].z; comb.z /= 3;

							point_add_src_idxs(&comb, &a_point[0]);
							point_add_src_idxs(&comb, &a_point[1]);
						}

						assert(comb.n_srcs >= 0 && comb.n_srcs <= POINT_MAX_SRCS);

						n_comb_points_with_n_srcs[comb.n_srcs]++;

						cloud_add_point(combined_cloud_out, comb);
					}

					// Remove points from both A and B
					// A is the KD tree; we just mark the data as invalid.
					// B is not stored locally, we just have the input pointer and don't want to modify
					// the data, so use the remove_point table.

					if(use == ICP_USE_COMB || use == ICP_USE_B)
					{
						if(pass[0])
							p_best_a[0]->point.removed = 1;

						if(pass[1])
							p_best_a[1]->point.removed = 1;
					}

					if(use == ICP_USE_COMB || use == ICP_USE_A)
					{
						//cloud_b_remove_point[bi] = 1;
						cloud_b_remove_point[bi/8] |= 1<<(bi%8);
					}
				}
				else
				{
					//printf("        NOPE\n");
				}

			}
		}



		// Copy non-removed cloud A points
		// Just loop through the KD tree. It's continuous like the kda array was originally built, the
		// points are just sorted in different order and pointers set, we ignore pointers here and the order of
		// points doesn't matter.
		// A source indeces copy through as they are.

		if(combine_verbose >= 3) printf("add cloud a\n");

		int a_add = 0;
		int b_add = 0;
		for(int i=0; i<a_n_points; i++)
		{
			cloud_point_t p = kda[i].point;
			if(p.removed)
				continue;

			p.z /= Z_SPREAD;

			a_add++;
			cloud_add_point(combined_cloud_out, p);

		}


		if(combine_verbose >= 3) printf("add cloud b\n");

		// Copy non-removed cloud B points
		// Source matching table built earlier is used to convert source indeces
		for(int i=0; i<b_n_points; i++)
		{
			//if(cloud_b_remove_point[i])
			if(cloud_b_remove_point[i/8] & (1<<(i%8)))
				continue;
			//cloud_point_t p = cloud_b->points[i];
			cloud_point_t p = transform_point(cloud_b->points[i], x_corr+(float)tx, y_corr+(float)ty, z_corr+(float)tz, yaw_corr);

			for(int i=0; i<p.n_srcs; i++)
				p.srcs[i] = matching_srcs[p.srcs[i]];

			b_add++;
			cloud_add_point(combined_cloud_out, p);

		}

		free(cloud_b_remove_point);

		// Copy cloud B freevects. Transform them with the same correction; also translate the source indeces.

		for(int i=0; i<cloud_b->m.n_freevects; i++)
		{
			freevect_point_t p = transform_freevect(cloud_b->freevects[i], x_corr+(float)tx, y_corr+(float)ty, z_corr+(float)tz, yaw_corr);
			p.src = matching_srcs[p.src];
			cloud_add_freevect(combined_cloud_out, p);
		}

		free(matching_srcs);

		if(combine_verbose >= 1) printf("combined, a was %d, b was %d (sum %d), assocomb %d points (sole best %d, sole 2ndbest %d, both %d), as-is a %d, b %d points, tot comb out %d points (%.1f%% of b reduced)\n", 
			a_n_points, b_n_points, a_n_points+b_n_points, n_associations, n_sole_best, n_sole_secbest, n_both_best_secbest, 
				a_add, b_add, combined_cloud_out->m.n_points, 
				100.0f*(float)((a_n_points+b_n_points)-combined_cloud_out->m.n_points)/(float)b_n_points);

		if(combine_verbose >= 2)
		{
			printf("Combined points: n_srcs   n_points\n");
			for(int i=0; i<=POINT_MAX_SRCS; i++)
				printf("    %2d    %5d\n", i, n_comb_points_with_n_srcs[i]);
		}
	}

	if(combine_verbose >= 2) cloud_print_sources(cloud_a, 1);

	if(x_corr_out) *x_corr_out = x_corr;
	if(y_corr_out) *y_corr_out = y_corr;
	if(z_corr_out) *z_corr_out = z_corr/Z_SPREAD;
	if(yaw_corr_out) *yaw_corr_out = yaw_corr;

	free(kda);

	return 0;
}


