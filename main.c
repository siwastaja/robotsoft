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

float main_robot_xs = 600.0;
float main_robot_ys = 450.0;
float main_robot_middle_to_lidar = -120.0;

/*

	Currently, we recommend the following procedure to localize on existing maps:

	A) If possible, it's always most intuitive to map the new area by first time booting the robot
	in a logical position and angle: for example, (almost) mounted in the charger is a good place.
	If you do this to an accuracy of +/- 40cm and about +/- 4 degrees, you never need to do anything,
	localization succeeds to the existing map, since robot boots to the same zero coordinate with enough
	accuracy for the "normal" SLAM correction.


	B) If you want to localize to another place than the origin in an existing map, or to a more uncertain position,
	   please do the following:

	1) As the very first step, send TCP_CR_STATEVECT_MID: disable mapping_*, enable loca_*, so that the map isn't messed up
	   before succesful localization happens.

	2) If necessary, also set localize_with_big_search_area=1 or =2 in the statevect.

	3) Use TCP_CR_SETPOS_MID to send your estimate of the robot coordinates, with the following precision:
		+/- 4 degree angle,  +/-  400mm x&y, if localize_with_big_search_area state is 0 (normal operation)
		+/- 45 degree angle, +/- 2000mm x&y, if localize_with_big_search_area state is 1
		    Any angle,       +/- 2400mm x&y, if localize_with_big_search_area state is 2

	4) Instruct manual move(s) towards any desired direction where you can/want go to. If localize_with_big_search_area
	   state is set, more than normal number of lidar scans will be accumulated before the localization happens -
	   this typically means you need to move about 2-3 meters. Similarly, the localization with big search area
	   will take up to 20-30 seconds typically, or even minutes when set to 2.

	5) TCP_RC_LOCALIZATION_RESULT_MID is sent. If the localization results in high enough score,
	   localize_with_big_search_area is automatically unset. If the score is low, it remains set, until localization
	   is good. If you face a problem that you can't get high enough score, we advice you localize around a place with enough
	   visual clues, and make sure the map built earlier actually shows them (enough time to build a detailed, strong map).

	6) You can send TCP_CR_STATEVECT_MID with mapping_* turned on as well. These are not turned on automatically.



*/

#define _BSD_SOURCE  // glibc backwards incompatibility workaround to bring usleep back.
#define _POSIX_C_SOURCE 200809L
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/select.h>
#include <errno.h>
#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <signal.h>

#include "datatypes.h"
#include "map_memdisk.h"
#include "tcp_comm.h"
#include "tcp_parser.h"
#include "utlist.h"

#include "misc.h"

#include "api_board_to_soft.h"
#include "api_soft_to_board.h"

#include "spi.h"

#include "mcu_micronavi_docu.c"
#include "config.h"

#include "slam_top.h"
#include "slam_cloud.h"

#define DEFAULT_SPEEDLIM 45
#define MAX_CONFIGURABLE_SPEEDLIM 70

volatile int verbose_mode = 0;

int max_speedlim = DEFAULT_SPEEDLIM;
int cur_speedlim = DEFAULT_SPEEDLIM;

int32_t move_run;
int32_t move_id;
int32_t move_remaining;


uint32_t micronavi_stop_flags;
uint32_t feedback_stop_flags;


pwr_status_t latest_pwr_status;

state_vect_t state_vect =
{
	.v = {
	.loca_2d = 1,
	.loca_3d = 0,
	.mapping_2d = 1,
	.mapping_3d = 0,
	.mapping_collisions = 1,
	.keep_position = 1,
	.command_source = USER_IN_COMMAND,
	.localize_with_big_search_area = 0
	}
};



#define SPEED(x_) do{ cur_speedlim = ((x_)>max_speedlim)?(max_speedlim):(x_); } while(0);

int live_obstacle_checking_on = 0; // only temporarily disabled by charger mounting code.
int pos_corr_id = 42;
#define INCR_POS_CORR_ID() {pos_corr_id++; if(pos_corr_id > 99) pos_corr_id = 0;}


int map_significance_mode = MAP_SEMISIGNIFICANT_IMGS | MAP_SIGNIFICANT_IMGS;

uint32_t robot_id = 0xacdcabba; // Hopefully unique identifier for the robot.

int cmd_state;

//extern world_t world;
#define BUFLEN 2048

int32_t cur_ang, cur_x, cur_y;
double robot_pos_timestamp;
int32_t cur_compass_ang;
int compass_round_active;

typedef struct
{
	int x;
	int y;
	int backmode;
	int take_next_early;
	int timeout;
} route_point_t;

#define THE_ROUTE_MAX 200
route_point_t the_route[THE_ROUTE_MAX];
int the_route_len = 0;

int do_follow_route = 0;
int route_finished_for_charger = 0;
int route_finished_or_notfound = 0;
int route_pos = 0;
int start_route = 0;
int partial_route = 0;
int id_cnt = 1;
int good_time_for_lidar_mapping = 0;

int route_reverse = 1;

void send_info(info_state_t state)
{
	if(tcp_client_sock >= 0) tcp_send_info_state(state);
}

int32_t prev_search_dest_x, prev_search_dest_y;

route_unit_t *some_route = NULL;

int32_t search_thread_dest_x, search_thread_dest_y;

static volatile sig_atomic_t search_thread_running;
static volatile sig_atomic_t search_thread_retval;
int search_instructed;

#if 0
pthread_t thread_search;
void* search_thread()
{
	int ret = search_route(&world, &some_route, ANG32TORAD(cur_ang), cur_x, cur_y, search_thread_dest_x, search_thread_dest_y, route_reverse);

	search_thread_retval = ret;
	search_thread_running = 0;

	return NULL;
}

// Call from sequential main thread only.
// Check search_thread_running first.
int run_search(int32_t dest_x, int32_t dest_y, int dont_map_lidars)
{
	if(search_thread_running)
	{
		printf("ERROR: run_search() called while previous search active. Ignoring the search request.\n");
		return -1234;
	}

	do_follow_route = 0;
	route_finished_or_notfound = 0;

	send_info(INFO_STATE_THINK);

	prev_search_dest_x = dest_x;
	prev_search_dest_y = dest_y;

	search_thread_dest_x = dest_x;
	search_thread_dest_y = dest_y;

	search_thread_running = 1;
	search_instructed = 1;

	int ret;

	if( (ret = pthread_create(&thread_search, NULL, search_thread, NULL)) )
	{
		printf("ERROR: thread_search thread creation, ret = %d\n", ret);
		return -1;
	}

	if( (ret = pthread_detach(thread_search)) )
	{
		printf("ERROR: thread_search detaching, ret = %d\n", ret);
		return -1;
	}

	return 0;
}

int poll_search_status(int act_as_well)
{
	int running = search_thread_running;
	int retval = search_thread_retval;

	if(running)
		return 12345;

	if(!act_as_well)
	{
		// not running, in other words, is finished, return:
		return retval;
	}
	
	// Act as well:

	if(!search_instructed)
		return 23456;

	search_instructed = 0;

	if(retval==-999)
		partial_route = 1;
	else
		partial_route = 0;

	route_unit_t *rt;
	int len = 0;
	DL_FOREACH(some_route, rt)
	{
		if(route_reverse)
			rt->backmode = !rt->backmode;

//		if(rt->backmode)
//			printf(" REVERSE ");
//		else
//			printf("         ");

		int x_mm, y_mm;
		mm_from_unit_coords(rt->loc.x, rt->loc.y, &x_mm, &y_mm);					
//		printf("to %d,%d\n", x_mm, y_mm);

		the_route[len].x = x_mm; 
		the_route[len].y = y_mm;
		the_route[len].backmode = rt->backmode;
		the_route[len].take_next_early = 50;
		len++;
		if(len >= THE_ROUTE_MAX)
			break;
	}

	for(int i = 1; i < len; i++)
	{
		float dist = sqrt(sq(the_route[i-1].x-the_route[i].x) + sq(the_route[i-1].y-the_route[i].y));
		int new_early = dist/10.0;
		if(new_early < 30) new_early = 30;
		else if(new_early > 250) new_early = 250;
		printf("[%d]=(%d,%d), [%d]=(%d,%d), dist=%.1f, new_early = %d\n", i-1, the_route[i-1].x, the_route[i-1].y, i, the_route[i].x, the_route[i].y, dist, new_early);
		the_route[i].take_next_early = new_early;
	}

	the_route[len-1].take_next_early = 20;

	msg_rc_route_status.num_reroutes++;

	tcp_send_route(cur_x, cur_y, &some_route);

	if(some_route)
	{
		the_route_len = len;
		do_follow_route = 1;
		start_route = 1;
		route_pos = 0;
		route_finished_or_notfound = 0;
		id_cnt++; if(id_cnt > 7) id_cnt = 1;
	}
	else
	{
		do_follow_route = 0;
		route_finished_or_notfound = 1;
		send_info(INFO_STATE_IDLE);
		retval = 99;
	}

	return retval;

}

void send_route_end_status(uint8_t reason)
{
	if(cmd_state == TCP_CR_ROUTE_MID)
	{
		if(tcp_client_sock >= 0)
		{
			msg_rc_route_status.cur_ang = cur_ang>>16;
			msg_rc_route_status.cur_x = cur_x;
			msg_rc_route_status.cur_y = cur_y;
			msg_rc_route_status.status = reason;
			tcp_send_msg(&msgmeta_rc_route_status, &msg_rc_route_status);
		}

		cmd_state = 0;
	}
}

int rerun_search()
{
	return run_search(prev_search_dest_x, prev_search_dest_y, 0);
}
#endif
int run_search(int32_t dest_x, int32_t dest_y, int dont_map_lidars){}
int poll_search_status(int act_as_well){}
void send_route_end_status(uint8_t reason){}
int rerun_search() {}

int new_move_to(int32_t x, int32_t y, int8_t backmode, int id, int speedlimit, int accurate_turn);
int rerequest_move_to();
int new_stop_movement();

void limit_speed(int x)
{
}

void daiju_mode(int x)
{
}

void do_live_obstacle_checking(){}
void route_fsm(){}

#if 0
static int maneuver_cnt = 0; // to prevent too many successive maneuver operations
void do_live_obstacle_checking()
{
	if(the_route[route_pos].backmode == 0)
	{
		int32_t target_x, target_y;
		int dx = the_route[route_pos].x - cur_x;
		int dy = the_route[route_pos].y - cur_y;

		int32_t dist_to_next = sqrt(sq(dx)+sq(dy));

		// Obstacle avoidance looks towards the next waypoint; if more than max_dist_to_next away,
		// it looks towards the staight line from cur_pos to the next waypoint, but only for the lenght of max_dist_to_next.
		// If there are obstacles on the straight line-of-sight, extra waypoint is searched for so that the number of
		// obstacles are minimized.
		// so target_x, target_y is only the target which is looked at. It's never a move_to target directly, the next waypoint is.

		const int32_t max_dist_to_next = 1200;
		if(dist_to_next < max_dist_to_next)
		{
			target_x = the_route[route_pos].x;
			target_y = the_route[route_pos].y;
		}
		else
		{
			float ang_to_target = atan2(dy, dx);
			target_x = cur_x + max_dist_to_next*cos(ang_to_target);
			target_y = cur_y + max_dist_to_next*sin(ang_to_target);
		}

		int hitcnt = check_direct_route_non_turning_hitcnt_mm(cur_x, cur_y, target_x, target_y);

		//printf("    HITCNT = %d, dist_to_next=%d\n", hitcnt, dist_to_next);

#if 0
		if(hitcnt > 0 && maneuver_cnt < 2)
		{
			// See what happens if we steer left or right

			int best_hitcnt = 9999;
			int best_drift_idx = 0;
			int best_angle_idx = 0;
			int best_new_x = 0, best_new_y = 0;

			const int side_drifts[12] = {320,-320, 240,-240,200,-200,160,-160,120,-120,80,-80};
			const float drift_angles[4] = {M_PI/6.0, M_PI/8.0, M_PI/12.0, M_PI/16.0};

			int predicted_cur_x = cur_x + cos(ANG32TORAD(cur_ang))*(float)cur_speedlim*2.0;
			int predicted_cur_y = cur_y + sin(ANG32TORAD(cur_ang))*(float)cur_speedlim*2.0;

			for(int angle_idx=0; angle_idx<4; angle_idx++)
			{
				for(int drift_idx=0; drift_idx<12; drift_idx++)
				{
					int new_x, new_y;
					if(side_drifts[drift_idx] > 0)
					{
						new_x = predicted_cur_x + cos(ANG32TORAD(cur_ang)+drift_angles[angle_idx])*side_drifts[drift_idx];
						new_y = predicted_cur_y + sin(ANG32TORAD(cur_ang)+drift_angles[angle_idx])*side_drifts[drift_idx];
					}
					else
					{
						new_x = predicted_cur_x + cos(ANG32TORAD(cur_ang)-drift_angles[angle_idx])*(-1*side_drifts[drift_idx]);
						new_y = predicted_cur_y + sin(ANG32TORAD(cur_ang)-drift_angles[angle_idx])*(-1*side_drifts[drift_idx]);
					}
					int drifted_hitcnt = check_direct_route_hitcnt_mm(cur_ang, new_x, new_y, the_route[route_pos].x, the_route[route_pos].y);
//					printf("a=%.1f deg  drift=%d mm  cur(%d,%d) to(%d,%d)  hitcnt=%d\n",
//						RADTODEG(drift_angles[angle_idx]), side_drifts[drift_idx], predicted_cur_x, predicted_cur_y, new_x, new_y, drifted_hitcnt);
					if(drifted_hitcnt <= best_hitcnt)
					{
						best_hitcnt = drifted_hitcnt;
						best_drift_idx = drift_idx;
						best_angle_idx = angle_idx;
						best_new_x = new_x; best_new_y = new_y;
					}
				}
			}

			if(best_hitcnt < hitcnt && best_hitcnt < 2)
			{

//				do_follow_route = 0;
//				new_stop_movement();

				if( (abs(side_drifts[best_drift_idx]) < 50) || ( abs(side_drifts[best_drift_idx]) < 100 && drift_angles[best_angle_idx] < M_PI/13.0))
				{
					SPEED(18);
					limit_speed(cur_speedlim);
//					printf("!!!!!!!!!!   Steering is almost needed (not performed) to maintain line-of-sight, hitcnt now = %d, optimum drift = %.1f degs, %d mm (hitcnt=%d), cur(%d,%d) to(%d,%d)\n",
//						hitcnt, RADTODEG(drift_angles[best_angle_idx]), side_drifts[best_drift_idx], best_hitcnt, cur_x, cur_y, best_new_x, best_new_y);
//					if(tcp_client_sock > 0) tcp_send_dbgpoint(cur_x, cur_y, 210, 210,   110, 1);
//					if(tcp_client_sock > 0) tcp_send_dbgpoint(best_new_x, best_new_y,   0, 255,   110, 1);
				}
				else
				{
					printf("Steering is needed, hitcnt now = %d, optimum drift = %.1f degs, %d mm (hitcnt=%d), cur(%d,%d) to(%d,%d)\n", 
						hitcnt, RADTODEG(drift_angles[best_angle_idx]), side_drifts[best_drift_idx], best_hitcnt, cur_x, cur_y, best_new_x, best_new_y);
					if(tcp_client_sock > 0) tcp_send_dbgpoint(cur_x, cur_y, 200, 200,   0, 1);
					if(tcp_client_sock > 0) tcp_send_dbgpoint(best_new_x, best_new_y,   0, 40,   0, 1);
					if(tcp_client_sock > 0) tcp_send_dbgpoint(target_x, target_y, 0, 130, 230, 1);

					// Do the steer
					id_cnt = 0; // id0 is reserved for special maneuvers during route following.
					new_move_to(best_new_x, best_new_y, 0, (id_cnt<<4) | ((route_pos)&0b1111), 12, 2 /* auto backmode*/);
					send_info((side_drifts[best_drift_idx] > 0)?INFO_STATE_RIGHT:INFO_STATE_LEFT);
					maneuver_cnt++;
				}
			}
			else
			{
//				printf("!!!!!!!!  Steering cannot help in improving line-of-sight.\n");
#endif
				if(hitcnt < 3)
				{
//					printf("!!!!!!!!!!!  Direct line-of-sight to the next point has 1..2 obstacles, slowing down.\n");
					SPEED(18);
					limit_speed(cur_speedlim);
				}
				else
				{
//					printf("Direct line-of-sight to the next point has disappeared! Trying to solve.\n");
					SPEED(18);
					limit_speed(cur_speedlim);
					new_stop_movement();
				}
#if 0
			}
		}
#endif
	}
}

static int route_finished_for_charger;
void route_fsm()
{
	static int micronavi_stops = 0;
	static double timestamp;
	static int nothing_happening = 0;
	static int did_reroute = 0;
	if(start_route)
	{
		printf("Start going id=%d!\n", id_cnt<<4);
		new_move_to(the_route[route_pos].x, the_route[route_pos].y, the_route[route_pos].backmode, (id_cnt<<4), cur_speedlim, 0);
		send_info(the_route[route_pos].backmode?INFO_STATE_REV:INFO_STATE_FWD);

		start_route = 0;
	}

	if(do_follow_route)
	{
		int id = move_id;

		static int32_t prev_x, prev_ang, prev_y;

		if(cur_x == prev_x && cur_y == prev_y && prev_ang == cur_ang)
		{
			nothing_happening++;
			if(nothing_happening > 2000)
			{
				printf("Nothing happening, re-requesting previous move\n");
				rerequest_move_to();
				nothing_happening = 0;
			}
		}
		else
		{
			nothing_happening = 0;
		}
		prev_x = cur_x;
		prev_y = cur_y;
		prev_ang = cur_ang;

		if(((id&0b1110000) == (id_cnt<<4)) && ((id&0b1111) == ((route_pos)&0b1111)))
		{
			if(micronavi_stop_flags || feedback_stop_flags)
			{
				if(!did_reroute)
				{
					printf("Micronavi STOP, rerouting.\n");
					rerun_search();
					did_reroute = 1;
				}

				int reret = poll_search_status(1);

				if(reret != 12345 && reret > 0)
				{
					printf("Routing failed.\n");
					send_route_end_status(4);
				}
			}
			else if(id_cnt == 0) // Zero id move is a special move during route following
			{
				did_reroute = 0;
				if(move_remaining < 30)
				{
					while(the_route[route_pos].backmode == 0 && route_pos < the_route_len-1)
					{
						if( (sq(cur_x-the_route[route_pos+1].x)+sq(cur_y-the_route[route_pos+1].y) < sq(800) )
						    && check_direct_route_mm(cur_ang, cur_x, cur_y, the_route[route_pos+1].x, the_route[route_pos+1].y))
						{
							printf("Maneuver done; skipping point (%d, %d), going directly to (%d, %d)\n", the_route[route_pos].x,
							       the_route[route_pos].y, the_route[route_pos+1].x, the_route[route_pos+1].y);
							route_pos++;
						}
						else
						{
							break;
						}
					}
					id_cnt = 1;
					printf("Maneuver done, redo the waypoint, id=%d!\n", (id_cnt<<4) | ((route_pos)&0b1111));
					new_move_to(the_route[route_pos].x, the_route[route_pos].y, the_route[route_pos].backmode, (id_cnt<<4) | ((route_pos)&0b1111), cur_speedlim, 0);
					send_info(the_route[route_pos].backmode?INFO_STATE_REV:INFO_STATE_FWD);

				}
			}
			else
			{
				did_reroute = 0;

				if(move_remaining < 250)
				{
					good_time_for_lidar_mapping = 1;
				}

				if(move_remaining < the_route[route_pos].take_next_early)
				{
					maneuver_cnt = 0;
					if(route_pos < the_route_len-1)
					{
						route_pos++;

						// Check if we can skip some points:
						while(the_route[route_pos].backmode == 0 && route_pos < the_route_len-1)
						{
							if( (sq(cur_x-the_route[route_pos+1].x)+sq(cur_y-the_route[route_pos+1].y) < sq(800) )
							  && check_direct_route_mm(cur_ang, cur_x, cur_y, the_route[route_pos+1].x, the_route[route_pos+1].y))
							{
								printf("skipping point (%d, %d), going directly to (%d, %d)\n", the_route[route_pos].x,
								       the_route[route_pos].y, the_route[route_pos+1].x, the_route[route_pos+1].y);
								route_pos++;
							}
							else
							{
								break;
							}
						}
						printf("cur id=%d, remaining = %d -> take the next with id=%d!\n", id, move_remaining, (id_cnt<<4) | ((route_pos)&0b1111));
						new_move_to(the_route[route_pos].x, the_route[route_pos].y, the_route[route_pos].backmode, (id_cnt<<4) | ((route_pos)&0b1111), cur_speedlim, 0);
						send_info(the_route[route_pos].backmode?INFO_STATE_REV:INFO_STATE_FWD);
						micronavi_stops = 0;
					}
					else
					{
						if(partial_route)
						{
							printf("Done following the partial route.\n");
							rerun_search();
							//if(reret > 0)
							//{
							//	printf("Routing failed.\n");
							//	send_route_end_status(reret);
							//}
						}
						else
						{
							printf("Done following the route.\n");
							send_info(INFO_STATE_IDLE);
							micronavi_stops = 0;
							do_follow_route = 0;
							route_finished_or_notfound = 1;
							route_finished_for_charger = 1;
							send_route_end_status(TCP_RC_ROUTE_STATUS_SUCCESS);
						}
					}
				}
#if 0
				else if(live_obstacle_checking_on)
				{
					// Check if obstacles have appeared in the map.

					static double prev_incr = 0.0;
					double stamp;
					if( (stamp=subsec_timestamp()) > prev_incr+0.20)
					{
						prev_incr = stamp;

//						if(robot_pos_timestamp < stamp-0.20)
//						{
							//printf("Skipping live obstacle checking due to stale robot pos.\n");
//						}
//						else
						{
							do_live_obstacle_checking();
						}
					}
				}
#endif
			}
		}

	}
	else // Not following route
	{
		nothing_happening = 0;
		did_reroute = 0;
	}

}
#endif

int32_t charger_ang;
int charger_fwd;
int charger_first_x, charger_first_y, charger_second_x, charger_second_y;
#define CHARGER_FIRST_DIST 1000
#define CHARGER_SECOND_DIST 500
#define CHARGER_THIRD_DIST  170

void save_robot_pos()
{
	FILE* f_cha = fopen(MAP_DIR"/robot_pos.txt", "w");
	if(f_cha)
	{
		fprintf(f_cha, "%d %d %d\n", cur_ang, cur_x, cur_y);
		fclose(f_cha);
	}
}

void retrieve_robot_pos()
{
	int32_t ang;
	int x; int y;
	FILE* f_cha = fopen(MAP_DIR"/robot_pos.txt", "r");
	if(f_cha)
	{
		fscanf(f_cha, "%d %d %d", &ang, &x, &y);
		fclose(f_cha);
		//set_robot_pos(ang, x, y);
	}
}

void conf_charger_pos()  // call when the robot is *in* the charger.
{
	int32_t cha_ang = cur_ang; int cha_x = cur_x; int cha_y = cur_y;


	printf("Set charger pos at ang=%d, x=%d, y=%d\n", cha_ang, cha_x, cha_y);
	charger_first_x = (float)cha_x - cos(ANG32TORAD(cha_ang))*(float)CHARGER_FIRST_DIST;
	charger_first_y = (float)cha_y - sin(ANG32TORAD(cha_ang))*(float)CHARGER_FIRST_DIST;	
	charger_second_x = (float)cha_x - cos(ANG32TORAD(cha_ang))*(float)CHARGER_SECOND_DIST;
	charger_second_y = (float)cha_y - sin(ANG32TORAD(cha_ang))*(float)CHARGER_SECOND_DIST;
	charger_fwd = CHARGER_SECOND_DIST-CHARGER_THIRD_DIST;
	charger_ang = cha_ang;

	FILE* f_cha = fopen(MAP_DIR"/charger_pos.txt", "w");
	if(f_cha)
	{
		fprintf(f_cha, "%d %d %d %d %d %d\n", charger_first_x, charger_first_y, charger_second_x, charger_second_y, charger_ang, charger_fwd);
		fclose(f_cha);
	}
}

void read_charger_pos()
{
	FILE* f_cha = fopen(MAP_DIR"/charger_pos.txt", "r");
	if(f_cha)
	{
		fscanf(f_cha, "%d %d %d %d %d %d", &charger_first_x, &charger_first_y, &charger_second_x, &charger_second_y, &charger_ang, &charger_fwd);
		fclose(f_cha);
		printf("charger position retrieved from file: %d, %d --> %d, %d, ang=%d, fwd=%d\n", charger_first_x, charger_first_y, charger_second_x, charger_second_y, charger_ang, charger_fwd);
	}
}


void save_pointcloud(int n_points, xyz_t* cloud)
{
	static int pc_cnt = 0;
	char fname[256];
	snprintf(fname, 255, "cloud%05d.xyz", pc_cnt);
	printf("Saving pointcloud with %d samples to file %s.\n", n_points, fname);
	FILE* pc_csv = fopen(fname, "w");
	if(!pc_csv)
	{
		printf("Error opening file for write.\n");
	}
	else
	{
		for(int i=0; i < n_points; i++)
		{
			fprintf(pc_csv, "%d %d %d\n",cloud[i].x, -1*cloud[i].y, cloud[i].z);
		}
		fclose(pc_csv);
	}

	pc_cnt++;
	if(pc_cnt > 99999) pc_cnt = 0;
}



int cal_x_d_offset = 0;
int cal_y_d_offset = 0;
float cal_x_offset = 40.0;
float cal_y_offset = 0.0;
float cal_x_sin_mult = 1.125;
float cal_y_sin_mult = 1.125;

#ifdef PULUTOF1
void request_tof_quit(void);
#endif

volatile int retval = 0;

int cmd_send_to_robot;

int new_stop_movement()
{
	printf("STOP MOVEMENT\n");
	s2b_stop_movement_t *p_msg = spi_init_cmd(CMD_STOP_MOVEMENT);

	if(!p_msg)
	{
		printf(__func__);
		printf("ERROR: p_msg null\n");
		return -1;
	}

	memset(p_msg, 0, sizeof(*p_msg));

	cmd_send_to_robot = 1;
	return 0;
}

int new_mount_charger()
{
	printf("MOUNT CHARGER\n");
	s2b_mount_charger_t *p_msg = spi_init_cmd(CMD_MOUNT_CHARGER);

	if(!p_msg)
	{
		printf(__func__);
		printf("ERROR: p_msg null\n");
		return -1;
	}

	memset(p_msg, 0, sizeof(*p_msg));

	cmd_send_to_robot = 1;
	return 0;
}

int new_self_calib_request()
{
	printf("SELF_CALIB_REQUEST\n");
	s2b_self_calib_request_t *p_msg = spi_init_cmd(CMD_SELF_CALIB_REQUEST);

	if(!p_msg)
	{
		printf(__func__);
		printf("ERROR: p_msg null\n");
		return -1;
	}

	memset(p_msg, 0, sizeof(*p_msg));

	cmd_send_to_robot = 1;
	return 0;
}


static int move_to_prev_x, move_to_prev_y, move_to_prev_id, move_to_prev_backmode, move_to_prev_speedlimit, move_to_prev_accurate_turn;
int new_move_to(int32_t x, int32_t y, int8_t backmode, int id, int speedlimit, int accurate_turn)
{
	move_to_prev_x = x;
	move_to_prev_y = y;
	move_to_prev_backmode = backmode;
	move_to_prev_id = id;
	move_to_prev_speedlimit = speedlimit;
	move_to_prev_accurate_turn = accurate_turn;

	s2b_move_abs_t *p_msg = spi_init_cmd(CMD_MOVE_ABS);

	if(!p_msg)
	{
		printf(__func__);
		printf("ERROR: p_msg null\n");
		return -1;
	}

	memset(p_msg, 0, sizeof(*p_msg));
	p_msg->x = x;
	p_msg->y = y;
	p_msg->id = id;
	p_msg->backmode = backmode;

	cmd_send_to_robot = 1;
	return 0;
}

int rerequest_move_to()
{
	return new_move_to(move_to_prev_x, move_to_prev_y, move_to_prev_backmode, move_to_prev_id, move_to_prev_speedlimit, move_to_prev_accurate_turn);
}

int ext_vacuum_cmd(int power, int nozzle)
{
	if(power < 0 || power > 100 || nozzle < 0 || nozzle > 1)
	{
		printf("WARNING: Illegal parameters to ext_vacuum_cmd(%d, %d)\n", power, nozzle);
		return -1;
	}

	s2b_ext_vacuum_t *p_msg = spi_init_cmd(CMD_EXT_VACUUM);

	if(!p_msg)
	{
		printf(__func__);
		printf("ERROR: p_msg null\n");
		return -1;
	}

	memset(p_msg, 0, sizeof(*p_msg));
	p_msg->power = power;
	p_msg->nozzle = nozzle;

	cmd_send_to_robot = 1;
	return 0;
}

int motor_enable_keepalive(int enabled)
{
	s2b_motors_t *p_msg = spi_init_cmd(CMD_MOTORS);

	if(!p_msg)
	{
		printf(__func__);
		printf("ERROR: p_msg null\n");
		return -1;
	}

	memset(p_msg, 0, sizeof(*p_msg));
	p_msg->enabled = enabled;

	cmd_send_to_robot = 1;

	return 0;
}

int new_correct_pos(int32_t da, int32_t dx, int32_t dy)
{
	s2b_corr_pos_t *p_msg = spi_init_cmd(CMD_CORR_POS);

	if(!p_msg)
	{
		printf(__func__);
		printf("ERROR: p_msg null\n");
		return -1;
	}

	memset(p_msg, 0, sizeof(*p_msg));
	p_msg->da = da;
	p_msg->dx = dx;
	p_msg->dy = dy;

	cmd_send_to_robot = 1;

	return 0;
}

int trace_ena = 0;
int trace_seq = 0;

void save_trace(int seq, uint8_t* p_data)
{
	char fname[256];
	sprintf(fname, "trace%08d.rb2", seq);

	FILE* fil = fopen(fname, "wb");
	if(!fil)
	{
		printf("ERROR: Trace: Error opening %s for write, errno=%d (%s)\n", fname, errno, strerror(errno));
		return;
	}

	int size = ((b2s_header_t*)p_data)->payload_len + B2S_TOTAL_OVERHEAD_WITHOUT_CRC;
	if(size < B2S_TOTAL_OVERHEAD_WITHOUT_CRC || size > B2S_MAX_LEN)
	{
		printf("ERROR: Trace: invalid size %d\n", size);
		return;
	}

	if(fwrite(p_data, size, 1, fil) != 1)
	{
		printf("ERROR: Trace: fwrite failed\n");
		fclose(fil);
		return;
	}
	fclose(fil);
	trace_seq++;
	if(trace_seq%500==0)
		printf("trace_seg at %d...\n", trace_seq);
}


void* main_thread()
{
	init_slam();
	load_sensor_softcals();


	int find_charger_state = 0;

	if(init_tcp_comm())
	{
		fprintf(stderr, "TCP communication initialization failed.\n");
		return NULL;
	}


	sleep(1);
	uint64_t subs[B2S_SUBS_U64_ITEMS] = {0, 0};
	ADD_SUB(subs, 4); // power status
//	ADD_SUB(subs, 5); // tof dists
//	ADD_SUB(subs, 6); // tof ampls
//	ADD_SUB(subs, 8); // tof diagnostics
	ADD_SUB(subs, 10); // hw_pose
	ADD_SUB(subs, 11); // drive module
//	ADD_SUB(subs, 13); // charger mount diagnostics
//	ADD_SUB(subs, 15); // compass headings
	ADD_SUB(subs, 14); // TOF slam set
	printf("Subscribing...\n");
	subscribe_to(subs);
	usleep(SPI_GENERATION_INTERVAL*20*1000);
	static int hwmsg_decim[B2S_MAX_MSGIDS];
	hwmsg_decim[4] = 4;
	hwmsg_decim[10] = 2;
	hwmsg_decim[11] = 2;
//	hwmsg_decim[5] = 1;
//	hwmsg_decim[6] = 1;

	static int stdout_msgids[B2S_MAX_MSGIDS];
//	stdout_msgids[11] = 1;
//	stdout_msgids[8] = 1;
//	stdout_msgids[13] = 1;
//	stdout_msgids[15] = 1;
//	stdout_msgids[14] = 1;


	srand(time(NULL));

	daiju_mode(0);
/*	turn_and_go_rel_rel(-5*ANG_1_DEG, 0, 25, 1);
	sleep(1);
	send_keepalive();
	turn_and_go_rel_rel(10*ANG_1_DEG, 0, 25, 1);
	sleep(1);
	send_keepalive();
	turn_and_go_rel_rel(-5*ANG_1_DEG, 50, 25, 1);
	sleep(1);
	send_keepalive();
	turn_and_go_rel_rel(0, -50, 25, 1);
	sleep(1);
*/

	if(spi_init_cmd_queue() < 0)
	{
		printf("ERROR: initial spi_init_cmd_queue error\n");
	}

	printf("Run.\n");

	double chafind_timestamp = 0.0;
	while(1)
	{
		uint8_t* p_data = spi_rx_pop();
		if(p_data != NULL)
		{
//			printf("Got something! Messages:\n");

			if(trace_ena)
			{
				save_trace(trace_seq, p_data);
			}

			int offs = sizeof(b2s_header_t);
			for(int i=0; i<B2S_SUBS_U64_ITEMS; i++)
			{
				uint64_t t = ((b2s_header_t*)p_data)->subs[i];
				for(int s=i*64; s<(i+1)*64; s++)
				{
					if(t & 1)
					{
						// id #s content received
						if(stdout_msgids[s])
						{
							printf("msgid=%u  name=%s  comment=%s\n", s, b2s_msgs[s].name, b2s_msgs[s].comment);
							if(b2s_msgs[s].p_print)
								b2s_msgs[s].p_print(&p_data[offs]);
						}

						if(s==10)
						{
							cur_x = ((hw_pose_t*)&p_data[offs])->x;
							cur_y = ((hw_pose_t*)&p_data[offs])->y;
							cur_ang = ((hw_pose_t*)&p_data[offs])->ang;

							//if(state_vect.v.vacuum_on || state_vect.v.command_source)
							//	mark_current_as_visited(&world, cur_ang, cur_x, cur_y);

							//clear_within_robot(&world, cur_ang, cur_x, cur_y);

						}

						if(s==4)
						{
							memcpy(&latest_pwr_status, &p_data[offs], sizeof(pwr_status_t));

							if(latest_pwr_status.flags & PWR_STATUS_FLAG_TURNOFF)
							{
								retval = 136;
								goto BREAK_LOOP;
							}
						}


						if(s==11)
						{
							move_run = ((drive_diag_t*)&p_data[offs])->run; 
							move_id = ((drive_diag_t*)&p_data[offs])->id;
							move_remaining = ((drive_diag_t*)&p_data[offs])->remaining;
							micronavi_stop_flags = ((drive_diag_t*)&p_data[offs])->micronavi_stop_flags;
					//		printf("GOT: move_id=%d, move_remaining=%d, stop_flags=%x\n", move_id, move_remaining, micronavi_stop_flags);
						}

						if(s==14)
						{
							slam_input_from_tss((tof_slam_set_t*)&p_data[offs]);
						}


						if(tcp_client_sock >= 0 && tcp_is_space_for_noncritical_message())
						{
							static int hwmsg_decim_cnt[B2S_MAX_MSGIDS];
							if(++hwmsg_decim_cnt[s] >= hwmsg_decim[s])
							{
								tcp_send(s, b2s_msgs[s].size, &p_data[offs]);
								hwmsg_decim_cnt[s] = 0;
							}
						}

						offs += b2s_msgs[s].size;
					}
					t >>= 1;
				}
			}

			if(is_error())
			{
				printf("Communication error reported -- clearing it.\n");
				ack_error();
				continue;
			}
		}


		// Calculate fd_set size (biggest fd+1)
		int fds_size = 0;
		if(tcp_listener_sock > fds_size) fds_size = tcp_listener_sock;
		if(tcp_client_sock > fds_size) fds_size = tcp_client_sock;
		if(STDIN_FILENO > fds_size) fds_size = STDIN_FILENO;
		fds_size+=1;


		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(STDIN_FILENO, &fds);
		FD_SET(tcp_listener_sock, &fds);
		if(tcp_client_sock >= 0)
			FD_SET(tcp_client_sock, &fds);

		struct timeval select_time = {0, 1000}; // has been 200us for a very long time -> use 1000us now

		if(select(fds_size, &fds, NULL, NULL, &select_time) < 0)
		{
			fprintf(stderr, "select() error %d", errno);
			return NULL;
		}

		if(FD_ISSET(STDIN_FILENO, &fds))
		{
			int cmd = fgetc(stdin);
			if(cmd == 'q')
			{
				retval = 0;
				break;
			}
			if(cmd == 'Q')
			{
				retval = 5;
				break;
			}

			if(cmd == 'S')
			{
//				save_map_pages(&world);
//				save_robot_pos();
			}
			if(cmd == 's')
			{
//				save_map_pages(&world);
//				retrieve_robot_pos();
			}

			if(cmd == 'c')
			{
				new_self_calib_request();
			}

			if(cmd == '0')
			{
				//set_robot_pos(0,0,0);
			}
			if(cmd == 'M')
			{
				printf("Requesting massive search.\n");
				state_vect.v.localize_with_big_search_area = 2;
			}
			if(cmd == 'L')
			{
				conf_charger_pos();
			}
			if(cmd == 'l')
			{
				new_mount_charger();

//				read_charger_pos();
//				find_charger_state = 1;
			}
			if(cmd == 'v')
			{
				if(state_vect.v.keep_position)
				{
					state_vect.v.keep_position = 0;
					printf("Robot is free to move manually.\n");
				}
				else
				{
					state_vect.v.keep_position = 1;
					printf("Robot motors enabled again.\n");
				}
			}

			if(cmd == 'V')
			{
			//	verbose_mode = verbose_mode?0:1;
				state_vect.v.vacuum_on = state_vect.v.vacuum_on?0:1;
			}

			if(cmd == '1')
			{
				trace_ena = 1;
				trace_seq = 0;
				printf("Start saving trace from %d\n", trace_seq);
			}

			if(cmd == '2')
			{
				trace_ena = 0;
				printf("Paused/stopped trace at %d\n", trace_seq);
			}

			if(cmd == '3')
			{
				trace_ena = 1;
				printf("Resumed trace at %d\n", trace_seq);
			}

			

		}

		if(tcp_client_sock >= 0 && FD_ISSET(tcp_client_sock, &fds))
		{
			int ret = handle_tcp_client();
			cmd_state = ret;
			if(ret == TCP_CR_DEST_MID)
			{
				state_vect.v.keep_position = 1;
				daiju_mode(0);

				msg_rc_movement_status.start_ang = cur_ang>>16;
				msg_rc_movement_status.start_x = cur_x;
				msg_rc_movement_status.start_y = cur_y;

				msg_rc_movement_status.requested_x = msg_cr_dest.x;
				msg_rc_movement_status.requested_y = msg_cr_dest.y;
				msg_rc_movement_status.requested_backmode = msg_cr_dest.backmode;

				move_remaining = 0; //999999; // invalidate

				printf("  ---> DEST params: X=%d Y=%d backmode=0x%02x\n", msg_cr_dest.x, msg_cr_dest.y, msg_cr_dest.backmode);
//				if(msg_cr_dest.backmode & 0b1000) // Rotate pose
//				{
//					float ang = atan2(msg_cr_dest.y-cur_y, msg_cr_dest.x-cur_x);
//					turn_and_go_abs_rel(RADTOANG32(ang), 0, cur_speedlim, 1);
//				}
//				else
				{
					new_move_to(msg_cr_dest.x, msg_cr_dest.y, msg_cr_dest.backmode, 0, cur_speedlim, 1);
				}
				find_charger_state = 0;
				do_follow_route = 0;
				send_info(msg_cr_dest.backmode?INFO_STATE_REV:INFO_STATE_FWD);

			}
			else if(ret == TCP_CR_ROUTE_MID)
			{

				printf("  ---> ROUTE params: X=%d Y=%d dummy=%d\n", msg_cr_route.x, msg_cr_route.y, msg_cr_route.dummy);

				msg_rc_route_status.start_ang = cur_ang>>16;
				msg_rc_route_status.start_x = cur_x;
				msg_rc_route_status.start_y = cur_y;

				msg_rc_route_status.requested_x = msg_cr_route.x;
				msg_rc_route_status.requested_y = msg_cr_route.y;
				msg_rc_route_status.status = TCP_RC_ROUTE_STATUS_UNDEFINED;
				msg_rc_route_status.num_reroutes = -1;

				state_vect.v.keep_position = 1;
				daiju_mode(0);
				find_charger_state = 0;
				run_search(msg_cr_route.x, msg_cr_route.y, 0);
			}
			else if(ret == TCP_CR_CHARGE_MID)
			{
				read_charger_pos();
				find_charger_state = 1;
			}
			else if(ret == TCP_CR_ADDCONSTRAINT_MID)
			{
				printf("  ---> ADD CONSTRAINT params: X=%d Y=%d\n", msg_cr_addconstraint.x, msg_cr_addconstraint.y);
//				add_map_constraint(&world, msg_cr_addconstraint.x, msg_cr_addconstraint.y);
			}
			else if(ret == TCP_CR_REMCONSTRAINT_MID)
			{
				printf("  ---> REMOVE CONSTRAINT params: X=%d Y=%d\n", msg_cr_remconstraint.x, msg_cr_remconstraint.y);
				for(int xx=-2; xx<=2; xx++)
				{
					for(int yy = -2; yy<=2; yy++)
					{
//						remove_map_constraint(&world, msg_cr_remconstraint.x + xx*40, msg_cr_remconstraint.y + yy*40);
					}
				}
			}
			else if(ret == TCP_CR_MODE_MID)	// Most mode messages deprecated, here for backward-compatibility, will be removed soon.
			{
				printf("Request for MODE %d\n", msg_cr_mode.mode);
				switch(msg_cr_mode.mode)
				{
					case 0:
					{
						state_vect.v.keep_position = 1;
						daiju_mode(0);
						//stop_automapping();
						state_vect.v.mapping_collisions = state_vect.v.mapping_3d = state_vect.v.mapping_2d = state_vect.v.loca_3d = state_vect.v.loca_2d = 0;
					} break;

					case 1:
					{
						state_vect.v.keep_position = 1;
						daiju_mode(0);
						//stop_automapping();
						find_charger_state = 0;
						do_follow_route = 0;
						send_info(INFO_STATE_IDLE);
						state_vect.v.mapping_collisions = state_vect.v.mapping_3d = state_vect.v.mapping_2d = state_vect.v.loca_3d = state_vect.v.loca_2d = 1;

					} break;

					case 2:
					{
						state_vect.v.keep_position = 1;
						daiju_mode(0);
//						routing_set_world(&world);
						//start_automapping_skip_compass();
						state_vect.v.mapping_collisions = state_vect.v.mapping_3d = state_vect.v.mapping_2d = state_vect.v.loca_3d = state_vect.v.loca_2d = 1;
					} break;

					case 3:
					{
						state_vect.v.keep_position = 1;
						daiju_mode(0);
//						routing_set_world(&world);
						//start_automapping_from_compass();
						state_vect.v.mapping_collisions = state_vect.v.mapping_3d = state_vect.v.mapping_2d = state_vect.v.loca_3d = state_vect.v.loca_2d = 1;
					} break;

					case 4:
					{
						//stop_automapping();
						find_charger_state = 0;
						do_follow_route = 0;
						state_vect.v.keep_position = 1;
						send_info(INFO_STATE_DAIJUING);
						daiju_mode(1);
						state_vect.v.mapping_collisions = state_vect.v.mapping_3d = state_vect.v.mapping_2d = state_vect.v.loca_3d = state_vect.v.loca_2d = 0;
					} break;

					case 5:
					{
						//stop_automapping();
						find_charger_state = 0;
						do_follow_route = 0;
						send_info(INFO_STATE_IDLE);
						state_vect.v.keep_position = 0;
						state_vect.v.mapping_collisions = state_vect.v.mapping_3d = state_vect.v.mapping_2d = state_vect.v.loca_3d = state_vect.v.loca_2d = 1;
					} break;

					case 6:
					{
						//stop_automapping();
						find_charger_state = 0;
						send_info(INFO_STATE_IDLE);
						do_follow_route = 0;
						state_vect.v.keep_position = 0;
						state_vect.v.mapping_collisions = state_vect.v.mapping_3d = state_vect.v.mapping_2d = state_vect.v.loca_3d = state_vect.v.loca_2d = 0;
					} break;

					case 7:
					{
						conf_charger_pos();
					} break;

					case 8:
					{
						//stop_automapping();
						find_charger_state = 0;
						do_follow_route = 0;
						new_stop_movement();
						send_info(INFO_STATE_IDLE);
					} break;

					case 9:
					{
						
					} break;

					case 10:
					{
					//	clear_visited(&world, cur_x, cur_y);

					} break;

					default: break;
				}
			}
			else if(ret == TCP_CR_MANU_MID)
			{
				#define MANU_FWD   10
				#define MANU_BACK  11
				#define MANU_LEFT  12
				#define MANU_RIGHT 13
				//stop_automapping();
				daiju_mode(0);
				state_vect.v.keep_position = 1;
				printf("Manual OP %d\n", msg_cr_manu.op);
				switch(msg_cr_manu.op)
				{
					case MANU_FWD:
						//turn_and_go_abs_rel(cur_ang, 100, 10, 1);
					break;
					case MANU_BACK:
						//turn_and_go_abs_rel(cur_ang, -100, 10, 1);
					break;
					case MANU_LEFT:
						//turn_and_go_abs_rel(cur_ang-10*ANG_1_DEG, 0, 10, 1);
					break;
					case MANU_RIGHT:
						//turn_and_go_abs_rel(cur_ang+10*ANG_1_DEG, 0, 10, 1);
					break;
					default:
					break;
				}
			}		
			else if(ret == TCP_CR_MAINTENANCE_MID)
			{
				if(msg_cr_maintenance.magic == 0x12345678)
				{
					retval = msg_cr_maintenance.retval;
					break;
				}
				else
				{
					printf("WARN: Illegal maintenance message magic number 0x%08x.\n", msg_cr_maintenance.magic);
				}
			}		
			else if(ret == TCP_CR_SPEEDLIM_MID)
			{
				int new_lim = msg_cr_speedlim.speedlim_linear_fwd;
				printf("INFO: Speedlim msg %d\n", new_lim);
				if(new_lim < 1 || new_lim > MAX_CONFIGURABLE_SPEEDLIM)
					max_speedlim = DEFAULT_SPEEDLIM;
				else
					max_speedlim = new_lim;

				if(cur_speedlim > max_speedlim)
				{
					cur_speedlim = max_speedlim;
					limit_speed(cur_speedlim);
				}
			}
			else if(ret == TCP_CR_STATEVECT_MID)
			{
				tcp_send_statevect();
			}
			else if(ret == TCP_CR_SETPOS_MID)
			{
				//set_robot_pos(msg_cr_setpos.ang<<16, msg_cr_setpos.x, msg_cr_setpos.y);

			}
		}

		if(FD_ISSET(tcp_listener_sock, &fds))
		{
			handle_tcp_listener();
		}

//		static int prev_compass_ang = 0;

//		if(cur_compass_ang != prev_compass_ang)
//		{
//			prev_compass_ang = cur_compass_ang;
//			printf("Compass ang=%.1f deg\n", ANG32TOFDEG(cur_compass_ang));
//		}

		if(!do_follow_route)
		{
			int ret;
			ret = poll_search_status(1);
			if(ret != 12345 && ret != 23456 && ret != 0 && ret != -999)
			{
				send_route_end_status(ret);
			}
		}

		static int micronavi_stop_flags_printed = 0;
		static int prev_move_remaining;

		if(micronavi_stop_flags)
		{
			if(!micronavi_stop_flags_printed)
			{
				micronavi_stop_flags_printed = 1;
				printf("MCU-level micronavigation: STOP. Reason flags:\n");
				for(int i=0; i<32; i++)
				{
					if(micronavi_stop_flags&(1UL<<i))
					{
						printf("bit %2d: %s\n", i, MCU_NAVI_STOP_NAMES[i]);
					}
				}

				printf("\n");

				if(cmd_state == TCP_CR_DEST_MID)
				{
					if(tcp_client_sock >= 0)
					{
						msg_rc_movement_status.cur_ang = cur_ang>>16;
						msg_rc_movement_status.cur_x = cur_x;
						msg_rc_movement_status.cur_y = cur_y;
						msg_rc_movement_status.status = TCP_RC_MOVEMENT_STATUS_STOPPED;
						msg_rc_movement_status.obstacle_flags = micronavi_stop_flags;
						tcp_send_msg(&msgmeta_rc_movement_status, &msg_rc_movement_status);
						send_info(INFO_STATE_IDLE);

					}

					cmd_state = 0;
				}
			}
		}
		else
			micronavi_stop_flags_printed = 0;




		if(cmd_state == TCP_CR_DEST_MID && tcp_client_sock >= 0)
		{
			static int32_t prev_move_run;

			if(prev_move_run && (!move_run))
			{
			
//				printf(" MOVEMENT FINISHED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
				msg_rc_movement_status.cur_ang = cur_ang>>16;
				msg_rc_movement_status.cur_x = cur_x;
				msg_rc_movement_status.cur_y = cur_y;
				msg_rc_movement_status.status = TCP_RC_MOVEMENT_STATUS_SUCCESS;
				msg_rc_movement_status.obstacle_flags = 0;
				tcp_send_msg(&msgmeta_rc_movement_status, &msg_rc_movement_status);
				send_info(INFO_STATE_IDLE);

				cmd_state = 0;
			}
			prev_move_remaining = move_remaining;
			prev_move_run = move_run;
		}
	
		if(find_charger_state < 4)
			live_obstacle_checking_on = 1;
		else
			live_obstacle_checking_on = 0;


		if(find_charger_state == 1)
		{
			route_finished_for_charger = 0;
			state_vect.v.keep_position = 1;
			daiju_mode(0);
			if(run_search(charger_first_x, charger_first_y, 0) != 0)
			{
				printf("Finding charger (first point) failed (run_search() called).\n");
				find_charger_state = 0;
			}
			else
				find_charger_state++;
		}
		else if(find_charger_state == 2)
		{
			if(route_finished_for_charger)
			{
				printf("Search succeeded, route followed.\n");
				find_charger_state++;
			}
			else if(route_finished_or_notfound)
			{
				printf("Finding charger (first point) failed\n");
				find_charger_state = 0;
			}

		}
		else if(find_charger_state == 3)
		{
			if(!do_follow_route)
			{
				if(sq(cur_x-charger_first_x) + sq(cur_y-charger_first_y) > sq(300))
				{
					printf("We are not at the first charger point, trying again.\n");
					find_charger_state = 1;
				}
				else
				{
					send_info(INFO_STATE_THINK);
//					printf("At first charger point, turning for charger.\n");
					printf("At first charger point...\n");
//					turn_and_go_abs_rel(charger_ang, 0, 23, 1);
					find_charger_state++;
					chafind_timestamp = subsec_timestamp();

				}
			}
		}
		else if(find_charger_state == 4)
		{
			if(subsec_timestamp() > chafind_timestamp+3.0)
			{
				printf("Going to second charger point.\n");
				send_info(INFO_STATE_FWD);
				new_move_to(charger_second_x, charger_second_y, 1, 0x7f, 20, 1);
				find_charger_state++;
			}
		}
		else if(find_charger_state == 5)
		{
			if(move_id == 0x7f && move_remaining < 40)
			{
				if(sq(cur_x-charger_second_x) + sq(cur_y-charger_second_y) > sq(180))
				{
					printf("We are not at the second charger point, trying again.\n");
					find_charger_state = 1;
				}
				else
				{
					chafind_timestamp = subsec_timestamp();
					send_info(INFO_STATE_THINK);
					printf("Requesting charger mount.\n");
					new_mount_charger();
					find_charger_state++;
				}
			}
		}
		else if(find_charger_state == 6)
		{
			if(!(latest_pwr_status.flags & PWR_STATUS_FLAG_CHARGING) && !(latest_pwr_status.flags & PWR_STATUS_FLAG_FULL))
			{
				if(subsec_timestamp() > chafind_timestamp+180.0)
				{
					printf("WARNING: Not charging (charger mount failure?). Retrying driving to charger.\n");
					find_charger_state = 1;
				}
			}
			else
			{
				send_info(INFO_STATE_CHARGING);
				find_charger_state = 0;
				printf("Robot charging succesfully.\n");
			}
		}

		//route_fsm();
		//autofsm();


		{
			static double prev_incr = 0.0;
			double stamp;
			if( (stamp=subsec_timestamp()) > prev_incr+0.15)
			{
				prev_incr = stamp;

				if(cur_speedlim < max_speedlim)
				{
					cur_speedlim++;
					//printf("cur_speedlim++ to %d\n", cur_speedlim);
					limit_speed(cur_speedlim);
				}

				if(cur_speedlim > max_speedlim)
				{
					cur_speedlim--;
					limit_speed(cur_speedlim);
				}
			}
		}


		/*
		static uint8_t prev_keep_position;
		if(!state_vect.v.keep_position && prev_keep_position)
			release_motors();
		prev_keep_position = state_vect.v.keep_position;
		*/

		static uint8_t prev_vacuum_on;

		if(state_vect.v.vacuum_on && !prev_vacuum_on)
			ext_vacuum_cmd(100, 0);
		else if(!state_vect.v.vacuum_on && prev_vacuum_on)
			ext_vacuum_cmd(0, 1);

		prev_vacuum_on = state_vect.v.vacuum_on;



		static uint8_t prev_autonomous;
		if(state_vect.v.command_source && !prev_autonomous)
		{
			daiju_mode(0);
//			routing_set_world(&world);
//			start_automapping_skip_compass();
			state_vect.v.mapping_collisions = state_vect.v.mapping_3d = state_vect.v.mapping_2d = state_vect.v.loca_3d = state_vect.v.loca_2d = 1;
		}
		if(!state_vect.v.command_source && prev_autonomous)
		{
//			stop_automapping();
		}
		prev_autonomous = state_vect.v.command_source;

		static int keepalive_cnt = 0;
		if(++keepalive_cnt > 500)
		{
			keepalive_cnt = 0;
			if(state_vect.v.keep_position)
				motor_enable_keepalive(1);
			else
				motor_enable_keepalive(0);
		}
		


		static double prev_sync = 0;
		double stamp;

		double write_interval = 30.0;
		if(tcp_client_sock >= 0)
			write_interval = 7.0;

#if 0
		if( (stamp=subsec_timestamp()) > prev_sync+write_interval)
		{
			prev_sync = stamp;

			int idx_x, idx_y, offs_x, offs_y;
//			page_coords(cur_x, cur_y, &idx_x, &idx_y, &offs_x, &offs_y);

			// Do some "garbage collection" by disk-syncing and deallocating far-away map pages.
//			unload_map_pages(&world, idx_x, idx_y);

			// Sync all changed map pages to disk
			if(save_map_pages(&world))
			{
				if(tcp_client_sock >= 0 && tcp_is_space_for_noncritical_message()) tcp_send_sync_request();
			}
			if(tcp_client_sock >= 0)
			{
				tcp_send_statevect();
			}

			fflush(stdout); // syncs log file.

		}
#endif

		if(cmd_send_to_robot)
		{
			cmd_send_to_robot = 0;
			spi_send_queue();
			usleep(200000);
			if(spi_init_cmd_queue() < 0)
			{
				usleep(300000);
			}

			if(spi_init_cmd_queue() < 0)
			{
				printf("ERROR: spi_init_cmd_queue error\n");
			}

		}

		usleep(1000);

	}

	BREAK_LOOP:;

	spi_comm_thread_quit();
	return NULL;
}


int main(int argc, char** argv)
{
	pthread_t thread_main, thread_spi;

	int ret;

	if( (ret = pthread_create(&thread_spi, NULL, spi_comm_thread, NULL)) )
	{
		printf("ERROR: spi access thread creation, ret = %d\n", ret);
		return -1;
	}

	if( (ret = pthread_create(&thread_main, NULL, main_thread, NULL)) )
	{
		printf("ERROR: main thread creation, ret = %d\n", ret);
		return -1;
	}

	pthread_join(thread_main, NULL);
	pthread_join(thread_spi, NULL);

	return retval;
}
