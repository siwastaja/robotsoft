#pragma once

// Range of connected sensors
// This is important for detecting when a "scan" starts and ends
// It's ok if there are missing sensors in-between.
#define FIRST_SIDX 1
#define LAST_SIDX 9

// First level of filtration: how many points in a 3x3 pixel block needs to be close enough to the middle pixel of that block,
// in order to get a point generated.
// >= 9 gets only points with "perfect" surrounding through. Not recommended.
// >= 6 is good because every "four" sides must have at least one good point, so mid-lier removal works very well.
// If you want larger number of more unreliable points (for example, you have a good algorithm afterwards), feel
// free to lower this threshold

#define TOF_N_CONFORM_REQUIRED 6


#define N_SENSORS 10
#define RESOLEVELS 0b1111


#define SOURCE_COMBINE_THRESHOLD 16   // point cloud units


// A "scan" is the combination of one acquisition of each sensor (one full round)
// VOXFILTER_N_SCANS scans are combined together without any matching.
#define VOXFILTER_N_SCANS 4

// After the voxfilter is full (VOXFILTER_N_SCANS reached), subsubmap is finished.


// Maximum number of subsubmaps in a submap. After exceeded, a new submap is force-started
#define SUBSUBMAP_LIMIT 12


// Maximum difference in robot coordinates during a submap, basically the length of the robot pose range before terminating the submap. In mm
#define DX_LIMIT 8000 // 2000
#define DY_LIMIT 8000 // 2000
#define DZ_LIMIT 1000 // 1000

// Cumulative linear travel limit; after exceeded, a new submap is started. In mm.
// Difference to the D*_LIMITs is that this limit can force a new submap even when the robot is
// just moving (back and forth, for example) inside a small area.
#define CUMUL_TRAVEL_LIMIT 16000  // 6000
// Similar, but for cumulative angular (yaw) motion
#define CUMUL_YAW_LIMIT DEGTORAD(800)  // 200


