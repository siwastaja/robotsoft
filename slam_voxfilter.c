// 24MB of memory with these, coverage 4.1x4.1x2.0 m
// Outside of the filter, all points go through as they are.
// When robot is fully stationary, max two sensors can see the same spot per full scan.
// When moving, three sensors can see the same spot per full scan.
// It's quite unlikely that 10 references would run out for 6 full scans. If it happens, no big harm done, data goes through without filtration

#define VOXFILTER_MAX_RAY_SOURCES 10 // 10 -> voxfilter_point_t = 24 bytes (alignable by 4 and 8)
#define VOXFILTER_XS 128
#define VOXFILTER_YS 128
#define VOXFILTER_ZS 64
#define VOXFILTER_STEP 32


/*
When the voxfilter accumulates (averages) close points from different sources, we must store the information of all
sources involved, so that the free space can be traced from all correct sources. 

In order to filter better, we allow another source point to be used instead, if it's close enough to
the correct source. This happens, for example, if the robot is stationary or near stationary. Or, in some cases, when
the robot happens to run forward at a certain lucky speed, so that another sensor gets an image close to the point
where the another sensor picked an image earlier.

Upside in increasing this: point clouds gets smaller, because each different source needs to generate a new point even if
the target point is combined by the voxfilter. 

The only downside to increasing this: free space very close to the robot may not be traced perfectly - some
small stripes and spots *might* remain unknown (not free) even when they are actually free. Note that if you have
a lot of moving people around the robot while mapping, this might prevent the later free space filter from removing
some artefacts.
*/
#define VOXFILTER_SOURCE_COMBINE_THRESHOLD 32 // mm

typedef struct __attribute__((packed))
{
	uint8_t src_idxs[VOXFILTER_MAX_RAY_SOURCES]; // Zero-terminated list of indeces to voxfilter.ray_sources[]  (zero termination not required, if max length used)
	uint16_t cnt;
	int32_t x;
	int32_t y;
	int32_t z;
} voxfilter_point_t;

typedef struct
{
	int x;
	int y;
	int z;
} voxfilter_ray_source_t;

// Remember to zero this struct out first.
typedef struct
{
	int n_ray_sources;
	voxfilter_ray_source_t ray_sources[VOXFILTER_N_SCANS*N_SENSORS+1]; // Start collecting at index 1
	voxfilter_point_t points[VOXFILTER_XS][VOXFILTER_YS][VOXFILTER_ZS];
} voxfilter_t;


static void voxfilter_to_cloud(voxfilter_t* voxfilter, cloud_t* cloud)
{
	int insert_cnt = 0, uniq_insert_cnt = 0;
	for(int yy=0; yy<VOXFILTER_YS; yy++)
	{
		for(int xx=0; xx<VOXFILTER_XS; xx++)
		{
			for(int zz=0; zz<VOXFILTER_ZS; zz++)
			{
				voxfilter_point_t* p = &(voxfilter->points[xx][yy][zz]);
				if(p->cnt == 0)
				{
					break;
				}

				assert(p->src_seq_id > 0 && p->src_seq_id < VOXFILTER_N_SCANS*N_SENSORS);

				int32_t x = p->x / p->cnt;
				int32_t y = p->y / p->cnt;
				int32_t z = p->z / p->cnt;

				// Insert the same point for each source
				// (Point cloud datatype doesn't have another mean to express
				// that multiple sources see the same point)

				for(int i=0; i<VOXFILTER_MAX_SOURCE_IDS; i++)
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

	printf("INFO: voxfilter_to_cloud inserted %d points, %d of which are duplicates due to multiple sources\n", insert_cnt, insert_cnt-uniq_insert_cnt);
}


static inline void voxfilter_insert_point(cloud_t* cloud, voxfilter_t* voxfilter, int srcid, 
	int32_t sx, int32_t sy, int32_t sz, // source coordinates are only used when the point cannot go to voxfilter; otherwise, srcid is stored
	int32_t x, int32_t y, int32_t z, // the point
	int32_t ref_x, int32_t ref_y, int32_t ref_z) // Extra translation, so that the voxfilter can live in its own limited space instead of the larger submap span.
	// Because only voxel selection is translated, not the actual stored coordinates (they fit int32_t accumulation variables just fine),
	// there is no need to translate points back later, basically we just store at a certain offset to maximize the span.
{
	int vox_x = (x-ref_x)/VOXFILTER_STEP + VOXFILTER_XS/2;
	int vox_y = (y-ref_x)/VOXFILTER_STEP + VOXFILTER_YS/2;
	int vox_z = (z-ref_x)/VOXFILTER_STEP + VOXFILTER_ZS/2;

	assert(srcid > 0 && srcid <= voxfilter->n_ray_sources);

	if(vox_x < 0 || vox_x > VOXFILTER_XS-1 || vox_y < 0 || vox_y > VOXFILTER_YS-1 || vox_z < 0 || vox_z > VOXFILTER_ZS-1)
	{
//		printf("INFO: voxfilter: skipping OOR point %d, %d, %d\n", vox_x, vox_y, vox_z);
		cloud_insert_point(cloud, sx, sy, sz, x, y, z);
		return;
	}


	// Accumulate the point
	voxfilter->points[vox_x][vox_y][vox_z].cnt++;
	voxfilter->points[vox_x][vox_y][vox_z].x += x;
	voxfilter->points[vox_x][vox_y][vox_z].y += y;
	voxfilter->points[vox_x][vox_y][vox_z].z += z;

	// If the source index doesn't exist on this voxel, add it.

	for(int i = 0; i < VOXFILTER_MAX_SOURCE_IDS; i++)
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

	// Did not find the source, nor had space to add it. Just insert the point to the cloud, bypassing the filter.	
	cloud_insert_point(cloud, sx, sy, sz, x, y, z);

	SOURCE_EXISTS:;	
}

