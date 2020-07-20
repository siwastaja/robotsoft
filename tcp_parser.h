/*
	PULUROBOT RN1-HOST Computer-on-RobotBoard main software

	(c) 2017-2018 Pulu Robotics and other contributors
	Maintainer: Antti Alhonen <antti.alhonen@iki.fi>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.



*/

#ifndef TCP_PARSER_H
#define TCP_PARSER_H

#include "datatypes.h"
#include "routing.h"

typedef struct
{
	// Where to write when receiving. This field is ignored for tx. 
	// Remember to use packed structs
	void* p_data; 
	// Message ID
	uint16_t mid;
	// Number of bytes of data expected / sent
	int size;
	// Zero-terminated string: Interpretation of the bytes:
	char types[32]; 
	/*
		'b' int8_t
		'B' uint8_t
		's' int16_t
		'S' uint16_t
		'i' int32_t
		'I' uint32_t
		'l' int64_t
		'L' uint64_t
	*/
	int ret;
} tcp_message_t;

// Dest: go directly (no routing)
#define TCP_CR_DEST_MID    355
typedef struct __attribute__ ((packed))
{
	int32_t x;
	int32_t y;
	int8_t backmode;
} tcp_cr_dest_t;

extern tcp_cr_dest_t   msg_cr_dest;

// Route: Robot plans a route to the given coordinates
#define TCP_CR_ROUTE_MID    356
typedef struct __attribute__ ((packed))
{
	int32_t x;
	int32_t y;
	int8_t dummy;
} tcp_cr_route_t;

extern tcp_cr_route_t   msg_cr_route;


#define TCP_CR_CHARGE_MID    357
typedef struct __attribute__ ((packed))
{
	uint8_t params;
} tcp_cr_charge_t;

extern tcp_cr_charge_t   msg_cr_charge;

#define TCP_CR_MODE_MID    358
typedef struct __attribute__ ((packed))
{
	uint8_t mode;
} tcp_cr_mode_t;

extern tcp_cr_mode_t   msg_cr_mode;

#define TCP_CR_MANU_MID    359
typedef struct __attribute__ ((packed))
{
	uint8_t op;
} tcp_cr_manu_t;

extern tcp_cr_manu_t   msg_cr_manu;

#define TCP_CR_ADDCONSTRAINT_MID    360
typedef struct __attribute__ ((packed))
{
	int32_t x;
	int32_t y;
} tcp_cr_addconstraint_t;

extern tcp_cr_addconstraint_t   msg_cr_addconstraint;

#define TCP_CR_REMCONSTRAINT_MID    361
typedef struct __attribute__ ((packed))
{
	int32_t x;
	int32_t y;
} tcp_cr_remconstraint_t;

extern tcp_cr_remconstraint_t   msg_cr_remconstraint;

#define TCP_CR_MAINTENANCE_MID    362
typedef struct __attribute__ ((packed))
{
	int32_t magic;
	int32_t retval;
} tcp_cr_maintenance_t;

extern tcp_cr_maintenance_t   msg_cr_maintenance;

#define TCP_CR_SPEEDLIM_MID    363
typedef struct __attribute__ ((packed))
{
	uint8_t speedlim_linear_fwd;  // right now, sets both linear (fwd and back) and angular speedlimit. 1..100. 0 = use default limit. Note that actual speedlimit may always be lower due to nearby obstacles.
	uint8_t speedlim_linear_back; // for future use, not implemented yet 
	uint8_t speedlim_angular; // for future use, not implemented yet
	uint8_t accellim_linear;  // for future use, not implemented yet
	uint8_t accellim_angular; // for future use, not implemented yet
} tcp_cr_speedlim_t;

extern tcp_cr_speedlim_t   msg_cr_speedlim;


#define TCP_CR_STATEVECT_MID        364


/*
SETPOS: Set new robot coordinates. Doesn't change states. Flushes lidar scan mapping queue.

*/
#define TCP_CR_SETPOS_MID    365
typedef struct __attribute__ ((packed))
{
	int16_t ang;
	int32_t x;
	int32_t y;
} tcp_cr_setpos_t;

extern tcp_cr_setpos_t msg_cr_setpos;

// Manual drive control, basically a "joystick" like command, which will be directly relayed to the firmware, which decides
// how to move the motors according to the buttons.
// Each button is one bit, active high.
// (fast<<4) | (up<<3) | (down<<2) | (left<<1) | (right<<0), rest reserved for future buttons

#define TCP_CR_MANCTRL_MID    366

typedef struct __attribute__ ((packed))
{
	uint8_t control;
} tcp_cr_manctrl_t;

extern tcp_cr_manctrl_t msg_cr_manctrl;



#define TCP_RC_POS_MID    430
typedef struct __attribute__ ((packed))
{
	int16_t ang;
	int32_t x;
	int32_t y;
	uint8_t  cmd_state; // Message ID of the command/job the robot is currently taking (for example, TCP_CR_ROUTE_MID)
} tcp_rc_pos_t;

extern tcp_message_t   msgmeta_rc_pos;
extern tcp_rc_pos_t    msg_rc_pos;


#define TCP_RC_MOVEMENT_STATUS_SUCCESS 0
#define TCP_RC_MOVEMENT_STATUS_STOPPED 1
#define TCP_RC_MOVEMENT_STATUS_STOPPED_BY_FEEDBACK_MODULE 1
#define TCP_RC_MOVEMENT_STATUS_MID  443
typedef struct __attribute__ ((packed))
{
	int16_t start_ang;
	int32_t start_x;
	int32_t start_y;

	int32_t requested_x;
	int32_t requested_y;
	int8_t requested_backmode;

	int16_t cur_ang;
	int32_t cur_x;
	int32_t cur_y;

	uint8_t status;
	uint32_t obstacle_flags;
} tcp_rc_movement_status_t;
extern tcp_message_t   msgmeta_rc_movement_status;
extern tcp_rc_movement_status_t    msg_rc_movement_status;


#define TCP_RC_ROUTE_STATUS_SUCCESS 0
#define TCP_RC_ROUTE_STATUS_NOTFOUND 1
#define TCP_RC_ROUTE_STATUS_UNDEFINED 2
#define TCP_RC_ROUTE_STATUS_MID  444
typedef struct __attribute__ ((packed))
{
	int16_t start_ang;
	int32_t start_x;
	int32_t start_y;

	int32_t requested_x;
	int32_t requested_y;

	int16_t cur_ang;
	int32_t cur_x;
	int32_t cur_y;

	uint8_t status;
	int16_t num_reroutes;
} tcp_rc_route_status_t;
extern tcp_message_t   msgmeta_rc_route_status;
extern tcp_rc_route_status_t    msg_rc_route_status;


#define TCP_RC_LIDAR_LOWRES_MID     431
#define TCP_RC_DBG_MID              432
#define TCP_RC_SONAR_MID            433
#define TCP_RC_BATTERY_MID          434
#define TCP_RC_ROUTEINFO_MID        435
#define TCP_RC_SYNCREQ_MID          436
#define TCP_RC_DBGPOINT_MID         437
#define TCP_RC_HMAP_MID             438
#define TCP_RC_INFOSTATE_MID        439
#define TCP_RC_ROBOTINFO_MID        440
#define TCP_RC_LIDAR_HIGHRES_MID    441
#define TCP_RC_PICTURE_MID	    442

#define TCP_RC_STATEVECT_MID        445
#define TCP_RC_LOCALIZATION_RESULT_MID 446

// See small_cloud.h for definitions:
#define TCP_RC_SMALL_CLOUD_MID      447

// See voxmap.h for definitions:
#define TCP_RC_VOXMAP_MID           448


int tcp_parser(int sock);

int tcp_send_msg(tcp_message_t* msg_type, void* msg);

void tcp_send_hwdbg(int32_t* dbg);
void tcp_send_sonar(sonar_point_t* p_son);
void tcp_send_battery();
void tcp_send_route(int32_t first_x, int32_t first_y, route_unit_t **route);
void tcp_send_sync_request();
void tcp_send_dbgpoint(int x, int y, uint8_t r, uint8_t g, uint8_t b, int persistence);
void tcp_send_hmap(int xsamps, int ysamps, int32_t ang, int xorig_mm, int yorig_mm, int unit_size_mm, int8_t *hmap);
void tcp_send_info_state(info_state_t state);
void tcp_send_robot_info();
void tcp_send_picture(int16_t id, uint8_t bytes_per_pixel, int xs, int ys, uint8_t *pict);
void tcp_send_statevect();
void tcp_send_localization_result(int32_t da, int32_t dx, int32_t dy, uint8_t success_code, int32_t score);
#include "small_cloud.h"
void tcp_send_small_cloud(int32_t ref_x, int32_t ref_y, int32_t ref_z, int n_points, small_cloud_t* points);
#include "voxmap.h"
void tcp_send_voxmap(voxmap_t* vm);

#endif
