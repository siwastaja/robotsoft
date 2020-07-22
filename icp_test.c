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
	float x;
	float y;
	float z;
} floatpoint_t;
/*
typedef struct
{
	int32_t x;
	int32_t y;
	int32_t z;
	int32_t dummy;
} point_t;
*/


typedef int32_t kd_unit_t;
typedef int32_t kd_unit_tmpsq_t;

typedef struct kd_node_struct
{
	kd_unit_t coords[3];
	struct kd_node_struct *left;
	struct kd_node_struct *right;
} kd_node_t;
 
static inline kd_unit_tmpsq_t dist(kd_node_t *a, kd_node_t *b)
{
	return 
		sq((kd_unit_tmpsq_t)a->coords[0]-(kd_unit_tmpsq_t)b->coords[0]) +
		sq((kd_unit_tmpsq_t)a->coords[1]-(kd_unit_tmpsq_t)b->coords[1]) +
		sq((kd_unit_tmpsq_t)a->coords[2]-(kd_unit_tmpsq_t)b->coords[2]);
}

static inline void swap(kd_node_t *a, kd_node_t *b)
{
	kd_unit_t tmp[3];
	tmp[0] = a->coords[0];
	tmp[1] = a->coords[1];
	tmp[2] = a->coords[2];
	a->coords[0] = b->coords[0];
	a->coords[1] = b->coords[1];
	a->coords[2] = b->coords[2];
	b->coords[0] = tmp[0];
	b->coords[1] = tmp[1];
	b->coords[2] = tmp[2];
}
 
 
// quickselect implementation
// ii is current depth in tree
static kd_node_t* find_median(kd_node_t *start, kd_node_t *end, int ii)
{
	if(end <= start)
		return NULL;
	if(end == start + 1)
		return start;
 
	int idx = ii%3;
	kd_node_t *p, *store, *md = start + (end - start) / 2;
	kd_unit_t pivot;
	while (1)
	{
		pivot = md->coords[idx];
		swap(md, end - 1);
		for(store = p = start; p < end; p++)
		{
			if (p->coords[idx] < pivot)
			{
				if(p != store)
					swap(p, store);
				store++;
			}
		}
		swap(store, end - 1);
 
		if (store->coords[idx] == md->coords[idx])
			return md;
 
		if (store > md)
			end = store;
		else
			start = store;
	}
}

// statistics:
int64_t branch_len_cumul;
int n_branch;
int max_branch_len;
 
static kd_node_t* make_tree(kd_node_t *t, int len, int i)
{
	kd_node_t *median_node;

	if(!len)
	{
		if(i>max_branch_len)
			max_branch_len = i;

		n_branch++;
		branch_len_cumul += i;
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
 

int visited;
 
static void nearest(kd_node_t *root, kd_node_t *node, int i, kd_node_t **best, kd_unit_t *best_dist)
{
	kd_unit_tmpsq_t d, dx;

	if (!root) return;

	visited++;
	d = dist(root, node);
	dx = root->coords[i] - node->coords[i];
  
	if (d < *best_dist)
	{
		*best_dist = d;
		*best = root;
	}
 
	// implementation I looked at returned here if !*best_dist.
	// were they thinking returning if *best_dist == 0.0? If yes, ! is incorrect operation.
	// That return seemed to almost never happen, only reducing performance by a few %.
	// even if comparing to a higher value, something already affecting results, still no measurable performance benefit.
	

	if(++i >= 3)
		i = 0;

	nearest(dx > 0 ? root->left : root->right, node, i, best, best_dist);
	if(sq(dx) >= *best_dist)
		return;
	nearest(dx > 0 ? root->right : root->left, node, i, best, best_dist);


/*
	// test: don't call just to return immediately. (remove the if(!root) return)
	// no performance benefit measured
	if(dx > 0 && root->left)
		nearest(root->left, node, i, best, best_dist);
	else if(dx <= 0 && root->right)
		nearest(root->right, node, i, best, best_dist);

	if(sq(dx) >= *best_dist)
		return;

	if(dx > 0 && root->right)
		nearest(root->right, node, i, best, best_dist);
	else if(dx <= 0 && root->left)
		nearest(root->left, node, i, best, best_dist);
*/

}
 











void cloud_to_kd(kd_node_t* kd, cloud_t* cloud)
{
	for(int i=0; i<cloud->n_points; i++)
	{
		if(
			cloud->points[i].px > 32767 || cloud->points[i].px < -32768 ||
			cloud->points[i].py > 32767 || cloud->points[i].py < -32768 ||
			cloud->points[i].pz > 32767 || cloud->points[i].pz < -32768)
		{
			printf("WARNING: IGNORING POINT\n");
			continue;
		}
		kd[i].coords[0] = cloud->points[i].px;
		kd[i].coords[1] = cloud->points[i].py;
		kd[i].coords[2] = cloud->points[i].pz;
	}
}


static int match_by_closest_points(cloud_t* cloud_a, cloud_t* cloud_b_in,
	float x_corr, float y_corr, float z_corr, float yaw_corr,
	int threshold,
	float* x_corr_out, float* y_corr_out, float* z_corr_out, float* yaw_corr_out,
	cloud_t* combined_cloud_out)
{
//	SWAP(cloud_t*, cloud_a, cloud_b_in);

	float mat_rota[9];
	float mat_transl[3];

	kd_node_t* kda = calloc(cloud_a->n_points, sizeof (kd_node_t));

	cloud_to_kd(kda, cloud_a);

	branch_len_cumul = 0;
	n_branch = 0;
	max_branch_len = -1;

	kd_node_t* root = make_tree(kda, cloud_a->n_points, 0);
	
	printf("%d branches, avg len = %d, max len = %d\n", n_branch, (int)(branch_len_cumul/(int64_t)n_branch), max_branch_len);
	printf("root at %d: (%d %d %d)\n", (int)(root-kda), root->coords[0], root->coords[1],root->coords[2]);
	printf("left child at %d: (%d %d %d)\n", (int)((root->left)-kda), root->left->coords[0], root->left->coords[1],root->left->coords[2]);
	printf("right child at %d: (%d %d %d)\n", (int)((root->right)-kda), root->right->coords[0], root->right->coords[1],root->right->coords[2]);

	double t_total_total = 0.0;


	for(int iter=0; iter<20; iter++)
	{
		double ts_initial;
		double ts;
		double t_transform, t_build, t_search, t_free, t_total;

		floatpoint_t mean_diff = {0,0,0};

		int_fast64_t dot_accum = 0;
		int_fast64_t det_accum = 0;

		static cloud_t cloud_b;

		ts_initial = ts = subsec_timestamp();
		transform_cloud_copy(cloud_b_in, &cloud_b, x_corr, y_corr, z_corr, yaw_corr);
		t_transform = subsec_timestamp() - ts;

		ts = subsec_timestamp();
		t_build = subsec_timestamp() - ts;

		ts = subsec_timestamp();

		int n_associations = 0;

		int max_visited = -1;
		int visited_cnt = 0;
		int64_t visited_acc = 0;

		for(int bi=0; bi<cloud_b.n_points; bi+=3)
		{
			kd_unit_t best_dist = 32000;
			kd_node_t* found;
			kd_node_t testNode = {{cloud_b.points[bi].px, cloud_b.points[bi].py, cloud_b.points[bi].pz}};

			visited=0;
			nearest(root, &testNode, 0, &found, &best_dist);
			visited_cnt++;
			visited_acc += visited;
			if(visited > max_visited)
			{
				max_visited = visited;
			}

			if(best_dist < sq(20))
			{

				//printf("closest to bi%d (%.1f %.1f %.1f) is (%.1f %.1f %.1f)\n", bi,
				//	testNode.coords[0], testNode.coords[1], testNode.coords[2], 
				//	found->coords[0], found->coords[1], found->coords[2]);

				n_associations++;

				mean_diff.x += testNode.coords[0] - found->coords[0];
				mean_diff.y += testNode.coords[1] - found->coords[1];
				mean_diff.z += testNode.coords[2] - found->coords[2];

				int sax = found->coords[0];
				int say = found->coords[1];
				int saz = found->coords[2];
				int sbx = testNode.coords[0];
				int sby = testNode.coords[1];
				int sbz = testNode.coords[2];

				int_fast64_t dot = sax*sbx + say*sby;
				int_fast64_t det = sax*sby - say*sbx;

				dot_accum += dot;
				det_accum += det;
			}

		}

		t_search = subsec_timestamp() - ts;

		ts = subsec_timestamp();
		t_free = subsec_timestamp() - ts;

		mean_diff.x /= (float)n_associations;
		mean_diff.y /= (float)n_associations;
		mean_diff.z /= (float)n_associations;

		float yaw = atan2f(det_accum, dot_accum);

		x_corr -= mean_diff.x;
		y_corr -= mean_diff.y;
		z_corr -= mean_diff.z;
		yaw_corr -= yaw;

		t_total = subsec_timestamp() - ts_initial;

		printf("ICP iter=%2d points=%6d,%6d,asso=%6d (%3.0f%%,%3.0f%%), transl=(%+6.0f,%+6.0f,%+6.0f), yaw=%.2f deg, corr=(%+6.0f,%+6.0f,%+6.0f; %.2f deg)\n",
			iter, cloud_a->n_points, cloud_b.n_points, n_associations,
			100.0*(float)n_associations/(float)cloud_a->n_points, 100.0*(float)n_associations/(float)cloud_b.n_points,
			mean_diff.x, mean_diff.y, mean_diff.z, RADTODEG(yaw), x_corr, y_corr, z_corr, RADTODEG(yaw_corr));


		printf("Performance: transform %.2f ms  build %.2f ms  search %.2f ms  free %.2f ms  total %.2f ms\n\n",
			t_transform*1000.0, t_build*1000.0, t_search*1000.0, t_free*1000.0, t_total*1000.0);
		printf("visited avg=%d  max=%d\n", (int) ((int64_t)visited_acc/(int64_t)visited_cnt), max_visited);


		t_total_total += t_total;
	}

	printf("total time = %.1f ms\n", t_total_total*1000.0);



	combined_cloud_out->n_points = 0;

	*x_corr_out = x_corr;
	*y_corr_out = y_corr;
	*z_corr_out = z_corr;
	*yaw_corr_out = yaw_corr;

	free(kda);

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
//	kdtreetest();
//	return 0;

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

	float xc, yc, zc, yawc;

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



