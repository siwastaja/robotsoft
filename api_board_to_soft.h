/*
	This header provides multiple ways of reuse & write-once (and don't worry about keeping in sync).

	1) The same file is used on the microcontroller (robotboard2-fw) and the host computer (robotsoft).
	   When compiling the robotboard2-fw, just check out the robotsoft repo as well. API sync is guaranteed.
	   If the repository versions are incompatible, that will result in compiler errors (such as trying to
	   access p_datatype_which_doesnt_exist) instead of indexing wrong things.

	2) The same file is used both to declare and define the datatypes accessed from multiple places. Normally,
	   you would do this in .h:
		extern int shared_thing;
	   and this in a c file which actually defines this (linker allocating memory for it only once):
		int shared_thing;
	   This leads in error-prone copy-paste, which is unacceptable when it comes to an API with up to 256
	   message types (and related pointers).

	   So, when the .h is included from the .c actually defining the things, do this:
	   #define DEFINE_API_VARIABLES
	   #include "api_board_to_soft.h"
	   #undef DEFINE_API_VARIABLES

	   When you want to declare the shared things, just include normally.
*/

#pragma once


#ifdef DEFINE_API_VARIABLES
#define MAYBE_EXTERN
#else
#define MAYBE_EXTERN extern
#endif


#define B2S_MAX_LEN 73728


#define B2S_MAX_MSGIDS 128
#if (B2S_MAX_MSGIDS%64 != 0)
#error "B2S_MAX_MSGIDS must be multiple of 64"
#endif

#define B2S_SUBS_U64_ITEMS (B2S_MAX_MSGIDS/64)
#define B2S_SUBS_LEN (B2S_SUBS_U64_ITEMS*8)

#define HEADER_LEN 8
#define SUBLIST_START_OFFSET (HEADER_LEN)
#define MSGS_START_OFFSET (HEADER_LEN+B2S_SUBS_LEN)
#define FOOTER_LEN 4
#define CRC_LEN 1

#define B2S_TOTAL_OVERHEAD_WITHOUT_CRC (HEADER_LEN+B2S_SUBS_LEN+FOOTER_LEN)
#define B2S_TOTAL_OVERHEAD (B2S_TOTAL_OVERHEAD_WITHOUT_CRC+CRC_LEN)

#if (B2S_MAX_LEN%8 != 0)
#error B2S_MAX_LEN must be multiple of 8
#endif


#if (HEADER_LEN%8 != 0)
#error "HEADER_LEN must be multiple of 8"
#endif

#ifdef CALIBRATOR
#define SPI_GENERATION_INTERVAL 5
#else
#define SPI_GENERATION_INTERVAL 20 // was 40
#endif

#include "datatypes.h"

typedef struct __attribute__((packed))
{
	uint32_t a;
	uint32_t b;
	uint8_t  c;
	uint16_t d;
} test_msg1_t;
void print_test_msg1(void* m);

typedef struct __attribute__((packed))
{
	uint8_t buf[2000];
} test_msg2_t;
void print_test_msg2(void* m);

typedef struct __attribute__((packed))
{
	uint8_t buf[40000];
} test_msg3_t;
void print_test_msg3(void* m);


#define PWR_STATUS_FLAG_CHARGING  1
#define PWR_STATUS_FLAG_FULL      2
#define PWR_STATUS_FLAG_TURNOFF   128
// Power flowing into the battery can be calculated as
// bat_mv/1000.0 * (pha_charging_current_ma+phb_charging_current_ma)/1000.0   (in watts)
typedef struct __attribute__((packed))
{
	uint8_t  flags;
	uint8_t  bat_percent;
	uint16_t bat_mv;
	uint16_t charger_input_mv;
	uint16_t pha_charging_current_ma;
	uint16_t phb_charging_current_ma;
} pwr_status_t;
void print_pwr_status(void* m);

typedef struct __attribute__((packed))
{
	uint8_t  sensor_idx;
	uint8_t  sensor_orientation;
	uint16_t dummy2;
	uint16_t dist[160*60];
	uint16_t dist_narrow[44*32];
	uint16_t wide_stray_estimate_adc;
	uint16_t narrow_stray_estimate_adc;
} tof_raw_dist_t;
void print_tof_raw_dist(void* m);

typedef struct __attribute__((packed))
{
	uint8_t  sensor_idx;
	uint8_t  dummy1;
	uint16_t dummy2;
	uint8_t  ampl[160*60];
	uint8_t  ampl_narrow[44*32];
} tof_raw_ampl8_t;
void print_tof_raw_ampl8(void* m);


typedef struct __attribute__((packed))
{
	uint8_t  sensor_idx;
	uint8_t  ambient[160*60];
	int16_t  temperature;
} tof_raw_ambient8_t;
void print_tof_raw_ambient8(void* m);

typedef struct __attribute__((packed))
{
	uint8_t  sensor_idx;
	int16_t  temperature;
	uint16_t timestamps[32];
	int32_t  dbg_i32[8];
} tof_diagnostics_t;
void print_tof_diagnostics(void* m);

#define IMG_WID_DCS0  0
#define IMG_WID_DCS1  1
#define IMG_WID_DCS2  2
#define IMG_WID_DCS3  3
#define IMG_WID_AMB   4
#define IMG_NAR_DCS0  5
#define IMG_NAR_DCS1  6
#define IMG_NAR_DCS2  7
#define IMG_NAR_DCS3  8
#define IMG_NAR_AMB   9

typedef struct __attribute__((packed))
{
	uint8_t  sensor_idx;
	uint8_t  img_type;
	union
	{
		uint16_t u16[160*60];
		int16_t  i16[160*60];
	} img;
	int16_t temperature;
	uint8_t param1;
	uint32_t param2;
} tof_raw_img_t;
void print_tof_raw_img(void* m);


typedef struct __attribute__((packed))
{
	uint32_t ang;
	uint32_t pitch;
	uint32_t roll;
	int32_t x;
	int32_t y;
	int32_t z;
} hw_pose_t;
void print_hw_pose(void* m);


typedef struct __attribute__((packed))
{
	int32_t ang_err;
	int32_t lin_err;
	int32_t cur_x;
	int32_t cur_y;
	int32_t target_x;
	int32_t target_y;
	int32_t id;
	int32_t remaining;
	uint32_t micronavi_stop_flags;
	uint8_t run;
	int32_t dbg1;
	int32_t dbg2;
	int32_t dbg3;
	int32_t dbg4;
	int32_t ang_speed_i;
	int32_t lin_speed_i;
	int32_t dbg5;
	int32_t dbg6;
} drive_diag_t;
void print_drive_diag(void* m);

#define TOF_SLAM_SET_FLAG_VALID (1<<0)
#define TOF_SLAM_SET_FLAG_SET1_WIDE (1<<1)
#define TOF_SLAM_SET_FLAG_SET1_NARROW (1<<2)

#ifndef DIST_MASK
	#define DIST_MASK (0x0fff)
	#define DIST_SHIFT (3)
#endif

#ifndef TOF_XS
	#define TOF_XS (160)
	#define TOF_YS (60)
#endif

#ifndef TOF_XS_NARROW
	#define TOF_XS_NARROW (32)
	#define TOF_YS_NARROW (44)
	#define TOF_NARROW_Y_START (8)
	#define TOF_NARROW_X_START (64)
#endif

typedef struct __attribute__((packed))
{
	/*
		Two distance buffers & robot poses at corresponding times.

		Set 0: basic_set: 19200 bytes
		Also includes potential flags
		Reconstructed HDR distance image based on two or three acquisitions,
		ambient, stray light, lens corrections etc., all the magic.
		12-bit resolution, unit 8mm, range 16..32760 mm
		0 (DIST_UNDEREXP) = underexposed (no object in range)
		1 (DIST_OVEREXP) = overexposed (obstacle very close, can assume 8mm)

		Bits 15..12: amplitude val 0..15


		Set 1: long_set:
		Reconstructed HDR distance image based on two acquisitions,
		all corrections.

		Contains either wide image (160*60) (when TOF_SLAM_SET_FLAG_LONG_IS_NARROW = 0)
		or narrow image (TOF_XS_NARROW*TOF_YS_NARROW), (when TOF_SLAM_SET_FLAG_LONG_IS_NARROW = 1).
		Narrow image is linearly in the beginning of the array (not a square within big image),
		indexed y*TOF_XS_NARROW+x.

		12-bit resolution, unit 8mm, range 16..32760mm
		0 = underexposed (no object in range)
		1 = overexposed (obstacle very close, can assume 8mm)

		Bits 15..12: amplitude val 0..15

	*/

	uint16_t flags;
	uint8_t sidx;
	uint8_t  sensor_orientation; // for diagnostics display of the image in correct orientation
	uint32_t dummy; 

	/*
		The second set is:
		- Invalid (FLAG_SET1_WIDE, FLAG_SET1_NARROW both unset). It was not acquired, to save time/energy.

		- An extra long range, less accurate wide image, FLAG_SET1_WIDE
		  This is exposed right after the basic set, but in high motion
		   situations, there is enough lag so that using the separate pose is crucial, thus included.

		- An extra long range, accurate narrow image, FLAG_SET1_NARROW. Similarly,
		  exposed right after the basic set, but you should use the separate pose to map it.
	*/
	struct __attribute__((packed))
	{
		hw_pose_t pose;
		uint16_t ampldist[160*60];
	} sets[2];

} tof_slam_set_t;
void print_tof_slam_set(void* m);


typedef struct __attribute__((packed))
{
	uint8_t  imus_valid; // bit0 = IMU0, bit1 = IMU1, etc.
	uint32_t heading_per_imu[6];
	uint32_t combined_heading;
} compass_heading_t;
void print_compass_heading(void* m);


// Voxel map is for legacy reasons. Likely to be completely removed very soon. One existing machine (REV2A) uses it.
// z_step = 100 mm
// base_z = -250 mm
// 0: -250 .. -150
// 1: -150 ..  -50
// 2:  -50 ..  +50
// 3:  +50 .. +150
// 4: +150 .. +250
// 5: +250 .. +350
// 6: +350 .. +450
// 7: +450 .. +550
// 8: +550 .. +650
// 9: +650 .. +750
//10: +750 .. +850
//11: +850 .. +950
//12: +950 ..+1050
//13:+1050 ..+1150
//14:+1150 ..+1250
//15:+1250 ..+1350


// bit index = (z-base_z)/z_step

#define BASE_Z (-250)
//#define BASE_Z (-50)
#define Z_STEP (100)
//#define Z_STEP (20)
#define MAX_Z ((Z_STEP*16 + BASE_Z)-1)
#define VOX_SEG_XS 100
#define VOX_SEG_YS 100

typedef struct __attribute__((packed))
{
	uint32_t running_cnt;
	int32_t  ref_x;
	int32_t  ref_y;
	uint8_t block_id;
	uint16_t z_step;
	int32_t  base_z;
	uint16_t map[VOX_SEG_XS*VOX_SEG_YS];
} mcu_voxel_map_t;
void print_mcu_voxel_map(void* m);


typedef struct __attribute__((packed))
{
	uint32_t running_cnt;
	int32_t  ref_x;
	int32_t  ref_y;
	uint8_t first_block_id;
	uint16_t z_step;
	int32_t  base_z;
	uint16_t maps[3][VOX_SEG_XS*VOX_SEG_YS];
} mcu_multi_voxel_map_t;
void print_mcu_multi_voxel_map(void* m);


typedef struct __attribute__((packed))
{
	// Current status:
	uint8_t cur_state;

	// Live calculation steps and results
	int32_t nearest_hit_x;
	int32_t nearest_hit_y;
	int32_t nearest_hit_z;
	int32_t middle_bar_min_y;
	int32_t middle_bar_max_y;
	int32_t left_accum_cnt;
	int32_t mid_accum_cnt;
	int32_t right_accum_cnt;
	int32_t backwall_ang;
	int32_t midmark_x;
	int32_t midmark_y;
	int32_t midmark_y_oldway;
	int32_t shift;
	int32_t dist;

	// Results, updated on the fly:
	int16_t first_movement_needed; // distance the robot needed to go fwd or back as the very first operation. 0 if within tolerances. in mm.
	uint8_t turning_passes_needed; // optical positioning needed to move the robot this many passes without needing to back of / go forward again (adjusting angle was enough alone)
	uint8_t vexling_passes_needed; // optical positioning needed to move the robot this many passes, doing a back-off-go-forward pass.
	uint8_t accepted_pos;          // 1, if optical positioning succesful. 0 if failed there.
	int16_t dist_before_push;      // after succesful optical positioning, the measured distance to the charger right before the push. in mm.
	uint8_t result;                // 100 = success. Others = failure.
} chafind_results_t;
void print_chafind_results(void* m);

typedef struct __attribute__((packed))
{
	uint8_t state; // 0 = idle, 1 = starting, 2=operating, 3 = success, 4 = failure
	uint8_t n_rounds; // full rotation counter (almost full may count as full).
	uint16_t dummy2;
	int32_t avg_rate;
} gyrocal_results_t;
void print_gyrocal_results(void* m);

/*
	The pointers
*/

MAYBE_EXTERN test_msg1_t* test_msg1;
MAYBE_EXTERN test_msg2_t* test_msg2;
MAYBE_EXTERN test_msg3_t* test_msg3;
MAYBE_EXTERN pwr_status_t* pwr_status;
MAYBE_EXTERN tof_raw_dist_t*  tof_raw_dist;
MAYBE_EXTERN tof_raw_ampl8_t* tof_raw_ampl8;
MAYBE_EXTERN tof_raw_ambient8_t* tof_raw_ambient8;
MAYBE_EXTERN tof_diagnostics_t* tof_diagnostics;
MAYBE_EXTERN tof_raw_img_t* tof_raw_img;
MAYBE_EXTERN hw_pose_t* hw_pose;
MAYBE_EXTERN drive_diag_t* drive_diag;
MAYBE_EXTERN mcu_voxel_map_t* mcu_voxel_map;
MAYBE_EXTERN mcu_multi_voxel_map_t* mcu_multi_voxel_map;
MAYBE_EXTERN chafind_results_t* chafind_results;
MAYBE_EXTERN tof_slam_set_t* tof_slam_set;
MAYBE_EXTERN compass_heading_t* compass_heading;
MAYBE_EXTERN gyrocal_results_t* gyrocal_results;


/*

For MCU:
This table holds pointers to pointers of generated TX data. Index of the table is the message id.
When the pointer of pointer is 0, the data type is not implemented. When implemented, this
contains a pointer to the pointer which can be used to write data to. The pointer management system
automatically adjusts these pointers in tx_fifo_push().

A sensor module (or whatever producing data) only needs to check if this pointer is NULL (meaning
the subscription is disabled, data is not wanted), and if not, just write the latest data to it.
You can safely modify the structures as you want before the tx_fifo_push() is finally called.

When tx_fifo_push() is called, the pointers move to the next free slot. Note that the memory content
is NOT automatically cleared, so old values hang out there; when the subscriptions change on-the-fly,
which happens frequently, this data can be complete gibberish. So always write all members, or memset
the whole struct.


For robotsoft:
Works in a very similar way. Same data structures are used.



--

Only the .c module responsible of handling the data needs to know about these tables.
*/

#ifdef FIRMWARE
	typedef struct
	{
		void**   p_accessor;
		uint16_t size;
	}  b2s_message_t;

	#define B2S_MESSAGE_STRUCT(msg, comment) {(void**)(&msg), sizeof(*msg)}

#else

	typedef struct
	{
		void**   p_accessor;
		uint16_t size;
		char*    name;
		char*    comment;
		void     (*p_print)(void*);
	}  b2s_message_t;

	#define B2S_MESSAGE_STRUCT(msg, comment) {(void**)(&msg), sizeof(*msg), #msg, comment, print_ ## msg}

#endif

#ifdef DEFINE_API_VARIABLES

b2s_message_t const b2s_msgs[B2S_MAX_MSGIDS] = {
	{0},
	B2S_MESSAGE_STRUCT(mcu_multi_voxel_map, "Low level voxel map, 3 parts of 12"), // 1 - automatically force-subscribed when relevant. Others may drop if they won't fit.
	B2S_MESSAGE_STRUCT(test_msg2, "Test message 2"),  // 2
	B2S_MESSAGE_STRUCT(test_msg3, "Test message 3"),  // 3
	B2S_MESSAGE_STRUCT(pwr_status, "Power status"), // 4
	B2S_MESSAGE_STRUCT(tof_raw_dist, "TOF raw distances"), // 5
	B2S_MESSAGE_STRUCT(tof_raw_ampl8, "TOF raw amplitudes"), // 6
	B2S_MESSAGE_STRUCT(tof_raw_ambient8, "TOF raw ambient light image"), // 7
	B2S_MESSAGE_STRUCT(tof_diagnostics, "TOF diagnostics"), // 8
	B2S_MESSAGE_STRUCT(tof_raw_img, "TOF raw image (multiple types)"), // 9
	B2S_MESSAGE_STRUCT(hw_pose, "Sensor fusion accumulated pose estimate"), // 10
	B2S_MESSAGE_STRUCT(drive_diag, "Drive module (mech feedbacks) diagnostics"), // 11
	B2S_MESSAGE_STRUCT(mcu_voxel_map, "Low level voxel map, 1 part of 12"), // 12
	B2S_MESSAGE_STRUCT(chafind_results, "Automatic charger mounting diagnostics"), // 13
	B2S_MESSAGE_STRUCT(tof_slam_set, "TOF distance set for SLAM"), // 14
	B2S_MESSAGE_STRUCT(compass_heading, "MEMS compass heading(s)"), // 15
	B2S_MESSAGE_STRUCT(gyrocal_results, "Gyro self-calibration results"), // 16
	{0}  
};

#else

extern b2s_message_t const b2s_msgs[B2S_MAX_MSGIDS];

#endif // DEFINE_API_VARIABLES




typedef struct __attribute__((packed))
{
	/*
		magic:
		for "no data avail": 0x00?? little endian (? = don't care)
		when data is available: 0xabcd little endian
	*/
	uint16_t magic;

	/*
		fifo_status:
		bit0: fifo occupancy more than 1 (if you are getting this with a full read, you are guaranteed to
		      be able to do another full read with another packet, without needing to poll inbetween)
		rest: reserved
	*/
	uint8_t fifo_status;
	uint8_t err_flags;
	uint16_t payload_len; // in bytes, not including header, subscription list and footer
	uint16_t reserved2;
	uint64_t subs[B2S_SUBS_U64_ITEMS];
} b2s_header_t;

typedef struct __attribute__((packed))
{
	/*
		magic:
		for "no data avail": 0x00?? little endian (? = don't care)
		when data is available: 0xabcd little endian
	*/
	uint16_t magic;

	/*
		fifo_status:
		bit0: fifo occupancy more than 1 (if you are getting this with a full read, you are guaranteed to
		      be able to do another full read with another packet, without needing to poll inbetween)
		rest: reserved
	*/
	uint8_t fifo_status;
	uint8_t err_flags;
	uint16_t payload_len; // in bytes, not including header, subscription list and footer
	uint16_t reserved2;
} b2s_poll_t;


#if 0
// TODO: maybe implement this kind of IMU raw data format?
typedef stuct __attribute__((packed))
{
	int16_t min;
	int16_t max;
	uint16_t n;
	int32_t acc;
} i16_minavgmax_t;

// In 0.1ms
#define FRAME_PERIOD 500 // 50ms = 20Hz


#define IMUX_PERIOD 40   // 250Hz
#define IMUG_PERIOD 25   // 400Hz
#define IMUM_PERIOD 1000 // 10Hz

#define IMU_RAW_MAX_N_X  (FRAME_PERIOD/IMUX_PERIOD+2)
#define IMU_RAW_MAX_N_G  (FRAME_PERIOD/IMUG_PERIOD+2)
#define IMU_RAW_MAX_N_M  (FRAME_PERIOD/IMUM_PERIOD+2)

typedef struct __attribute__((packed))
{
	uint16_t xcel_n;
	i16_xyz_t xcel[IMU_RAW_MAX_N_X];

	uint16_t gyro_n;
	i16_xyz_t gyro[IMU_RAW_MAX_N_G];

	uint16_t mag_n;
	i16_xyz_t mag [IMU_RAW_MAX_N_M];
} imu_raw_t;

#endif


