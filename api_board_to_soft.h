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


#define B2S_MAX_LEN 55000


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

#if (HEADER_LEN%8 != 0)
#error "HEADER_LEN must be multiple of 8"
#endif


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
	uint8_t  dummy1;
	uint16_t dummy2;
	uint16_t dist[160*60];
} tof_raw_dist_t;
void print_tof_raw_dist(void* m);

typedef struct __attribute__((packed))
{
	uint8_t  sensor_idx;
	uint8_t  dummy1;
	uint16_t dummy2;
	uint8_t  ampl[160*60];
} tof_raw_ampl8_t;
void print_tof_raw_ampl8(void* m);



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


/*
	The pointers
*/

MAYBE_EXTERN test_msg1_t* test_msg1;
MAYBE_EXTERN test_msg2_t* test_msg2;
MAYBE_EXTERN test_msg3_t* test_msg3;
MAYBE_EXTERN pwr_status_t* pwr_status;
MAYBE_EXTERN tof_raw_dist_t*  tof_raw_dist;
MAYBE_EXTERN tof_raw_ampl8_t* tof_raw_ampl8;



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
is NOT automatically cleared, so old values hang out there.

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
#endif

#ifdef ROBOTSOFT
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
	B2S_MESSAGE_STRUCT(test_msg1, "Test message 1"),  // 1
	B2S_MESSAGE_STRUCT(test_msg2, "Test message 2"),  // 2
	B2S_MESSAGE_STRUCT(test_msg3, "Test message 3"),  // 3
	B2S_MESSAGE_STRUCT(pwr_status, "Power status"), // 4
	B2S_MESSAGE_STRUCT(tof_raw_dist, "TOF raw distances"), // 5
	B2S_MESSAGE_STRUCT(tof_raw_ampl8, "TOF raw amplitudes"), // 6
	{0}  
};

#else

extern b2s_message_t const b2s_msgs[B2S_MAX_MSGIDS];

#endif // DEFINE_API_VARIABLES


#if 0

#define TOF_FLAG_CERTAINTY_MASK 0b111

typedef struct __attribute__((packed))
{
	uint8_t flags;
	int16_t x;
	int16_t y;
	int16_t z;
} tof_relative_point_t; // relative to robot origin


typedef struct __attribute__((packed))
{

} b2s_tof3d_development_data_t;


#ifdef API_MCU
extern b2s_tof3d_development_data_t *b2s_tof3d_development_data;
#endif

typedef struct __attribute__((packed))
{

} b2s_tof3d_distance_data_t;


typedef struct __attribute__((packed))
{
	uint8_t sensor_idx;
	uint16_t n_points;
	tof_relative_point_t

} b2s_tof3d_pointcloud_data_t;

typedef stuct __attribute__((packed))
{
	int16_t min;
	int16_t max;
	uint16_t n;
	int32_t acc;
} i16_minavgmax_t;


typedef stuct __attribute__((packed))
{
	int16_t x;
	int16_t y;
	int16_t z;
} i16_xyz_t;

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
