#pragma once

// Range of connected sensors
// This is important for detecting when a "scan" starts and ends
// It's ok if there are missing sensors in-between.
#define N_SENSORS 10 // maximum index N_SENSORS-1
#ifdef VACUUM_APP
	#define FIRST_SIDX 1
#else
	#define FIRST_SIDX 0
#endif
#define LAST_SIDX 9

//#define USE_NARROWS
//#define USE_LONG_WIDES

// First level of filtration: how many points in a 3x3 pixel block needs to be close enough to the middle pixel of that block,
// in order to get a point generated.
// >= 9 gets only points with "perfect" surrounding through. Not recommended.
// >= 6 is good because every "four" sides must have at least one good point, so mid-lier removal works very well.
// If you want larger number of more unreliable points (for example, you have a good algorithm afterwards), feel
// free to lower this threshold

#define TOF_N_CONFORM_REQUIRED 6


#define RESOLEVELS 0b1111


#define SOURCE_COMBINE_THRESHOLD (192 /*mm*/ /16)   // point cloud units



// Maximum difference in robot coordinates during a submap, basically the length of the robot pose range before terminating the submap. In mm
#define DX_LIMIT 8000
#define DY_LIMIT 8000
#define DZ_LIMIT 1000

// Cumulative linear travel limit; after exceeded, a new submap is started. In mm.
// Difference to the D*_LIMITs is that this limit can force a new submap even when the robot is
// just moving (back and forth, for example) inside a small area.
#define CUMUL_TRAVEL_LIMIT 16000
// Similar, but for cumulative angular (yaw) motion
#define CUMUL_YAW_LIMIT DEGTORAD(360*5)


