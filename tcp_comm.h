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



	All multi-byte values are presented in little endian. Why? Because we
	need to choose something. In the robot side, we have the following situation:
	- Almost vendor-lock-in in many, many components anyway. Right now, we
	  are supporting Raspberry Pi 3 and Odroid XU4. 99.9999% of relevant computers
	  on the market are all little endian
	- The robot runs on a battery, and must be low-cost. So everything needs to be
	  as efficient as possible.

	In most cases today, little endian requires no swapping at either side and is the
	most efficient. If we chose big endian, we would need to swap at both sides.
	At the non-robot side (server, or standalone client software), big endian will of
	course be supported. In that case, swapping only happens there - where performance
	is not such a big issue - instead of the robot where it is a big issue.

	As such, there is much less parsing going on on the robot side. Non-robot side
	needs to conform to the convetions expected at the robot.


*/

#ifndef TCP_COMM_H
#define TCP_COMM_H

#include <stdint.h>

extern int tcp_listener_sock;
extern int tcp_client_sock; // One client at the time is allowed.

int init_tcp_comm();
int handle_tcp_client();
int handle_tcp_listener();
int tcp_send(uint16_t msgid, uint32_t paylen, uint8_t* buf);
void tcp_comm_close();




#endif
