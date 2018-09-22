/*
	This same header can be used on both projects (robotsoft and robotboard2-fw), so that the API
	definitions will always be in sync (write once). Since the different sides require different
	structures, #define API_MCU will use the structures for firmware; otherwise, computer side is
	assumed.
*/

#pragma once


typedef struct __attribute__((packed))
{
	uint32_t a;
	uint32_t b;
	uint8_t  c;
	uint16_t d;
} test_msg1_t;

typedef struct __attribute__((packed))
{
	uint8_t buf[1024];
} test_msg2_t;

typedef struct __attribute__((packed))
{
	uint8_t buf[40000];
} test_msg3_t;


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


typedef struct __attribute__((packed))
{
	// msgid is the table index, no need to store here.
	uint16_t paylen;
#ifdef API_MCU
	void** p_p_txbuf;
#else
	char* name;
#endif

} board_to_soft_entry_t;

const board_to_soft_entry_t BOARD_TO_SOFT_MSGS[256] =
{

#ifdef API_MCU
	&
#else
	"3DTOF development data"
#endif
}

#endif
