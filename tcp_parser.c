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


	TCP communication functions.
	Robot is the "server", client can be a client software directly, or a relaying server.
	

*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "datatypes.h"
#include "tcp_comm.h"
#include "tcp_parser.h"
#include "utlist.h"
#include "routing.h"
#include "misc.h"

// Client->Robot messages

tcp_cr_dest_t msg_cr_dest;
tcp_message_t msgmeta_cr_dest =
{
	&msg_cr_dest,
	TCP_CR_DEST_MID,
	9, "iib"
};

tcp_cr_route_t msg_cr_route;
tcp_message_t msgmeta_cr_route =
{
	&msg_cr_route,
	TCP_CR_ROUTE_MID,
	9, "iib"
};

tcp_cr_charge_t msg_cr_charge;
tcp_message_t msgmeta_cr_charge =
{
	&msg_cr_charge,
	TCP_CR_CHARGE_MID,
	1, "b"
};

tcp_cr_mode_t msg_cr_mode;
tcp_message_t msgmeta_cr_mode =
{
	&msg_cr_mode,
	TCP_CR_MODE_MID,
	1, "b"
};

tcp_cr_manu_t msg_cr_manu;
tcp_message_t msgmeta_cr_manu =
{
	&msg_cr_manu,
	TCP_CR_MANU_MID,
	1, "b"
};

tcp_cr_addconstraint_t msg_cr_addconstraint;
tcp_message_t msgmeta_cr_addconstraint =
{
	&msg_cr_addconstraint,
	TCP_CR_ADDCONSTRAINT_MID,
	8, "ii"
};

tcp_cr_remconstraint_t msg_cr_remconstraint;
tcp_message_t msgmeta_cr_remconstraint =
{
	&msg_cr_remconstraint,
	TCP_CR_REMCONSTRAINT_MID,
	8, "ii"
};

tcp_cr_maintenance_t msg_cr_maintenance;
tcp_message_t msgmeta_cr_maintenance =
{
	&msg_cr_maintenance,
	TCP_CR_MAINTENANCE_MID,
	8, "ii"
};

tcp_cr_speedlim_t msg_cr_speedlim;
tcp_message_t msgmeta_cr_speedlim =
{
	&msg_cr_speedlim,
	TCP_CR_SPEEDLIM_MID,
	5, "BBBBB"
};


tcp_message_t msgmeta_cr_statevect =
{
	&state_vect,
	TCP_CR_STATEVECT_MID,
	16, "BBBBBBBBBBBBBBBB"
};

tcp_cr_setpos_t msg_cr_setpos;
tcp_message_t msgmeta_cr_setpos =
{
	&msg_cr_setpos,
	TCP_CR_SETPOS_MID,
	10, "sii"
};



#define NUM_CR_MSGS 11
tcp_message_t* CR_MSGS[NUM_CR_MSGS] =
{
	&msgmeta_cr_dest,
	&msgmeta_cr_route,
	&msgmeta_cr_charge,
	&msgmeta_cr_mode,
	&msgmeta_cr_manu,
	&msgmeta_cr_addconstraint,
	&msgmeta_cr_remconstraint,
	&msgmeta_cr_maintenance,
	&msgmeta_cr_speedlim,
	&msgmeta_cr_statevect,
	&msgmeta_cr_setpos
};

// Robot->Client messages
tcp_message_t msgmeta_rc_pos =
{
	0,
	TCP_RC_POS_MID,
	11, "siiB"
};
tcp_rc_pos_t    msg_rc_pos;


tcp_message_t msgmeta_rc_movement_status =
{
	0,
	TCP_RC_MOVEMENT_STATUS_MID,
	34, "siiiibsiiBI"
};
tcp_rc_movement_status_t    msg_rc_movement_status;


tcp_message_t msgmeta_rc_route_status =
{
	0,
	TCP_RC_ROUTE_STATUS_MID,
	31, "siiiisiiBs"
};
tcp_rc_route_status_t    msg_rc_route_status;


#define I32TOBUF(i_, b_, s_) {b_[(s_)] = ((i_)>>24)&0xff; b_[(s_)+1] = ((i_)>>16)&0xff; b_[(s_)+2] = ((i_)>>8)&0xff; b_[(s_)+3] = ((i_)>>0)&0xff; }
#define I16TOBUF(i_, b_, s_) {b_[(s_)] = ((i_)>>8)&0xff; b_[(s_)+1] = ((i_)>>0)&0xff; }

void tcp_send_picture(int16_t id, uint8_t bytes_per_pixel, int xs, int ys, uint8_t *pict)
{
	PR_FUNC();
}

void tcp_send_robot_info()
{
	PR_FUNC();
}


void tcp_send_info_state(info_state_t state)
{
	PR_FUNC();
}

void tcp_send_sync_request()
{
	PR_FUNC();
}

void tcp_send_hmap(int xsamps, int ysamps, int32_t ang, int xorig_mm, int yorig_mm, int unit_size_mm, int8_t *hmap)
{
	PR_FUNC();
}

void tcp_send_route(int32_t first_x, int32_t first_y, route_unit_t **route)
{
	uint8_t buf[2000];
	int i = 4+4;
	route_unit_t *rt;
	DL_FOREACH(*route, rt)
	{
		if(i > 1900)
		{
			printf("WARNING: Route too long to be sent to the client. Ignoring the rest.\n");
			break;
		}

		int x_mm, y_mm;
		mm_from_unit_coords(rt->loc.x, rt->loc.y, &x_mm, &y_mm);					
		buf[i+0] = rt->backmode?1:0;
		I32TOBUF(x_mm, buf, i+1);
		I32TOBUF(y_mm, buf, i+5);
		i += 9;
	}

	I32TOBUF(first_x, buf, 0);
	I32TOBUF(first_y, buf, 4);

	tcp_send(TCP_RC_ROUTEINFO_MID, i, buf);

}

void tcp_send_dbgpoint(int x, int y, uint8_t r, uint8_t g, uint8_t b, int persistence)
{
	PR_FUNC();
}

void tcp_send_statevect()
{
	const int size = sizeof(state_vect);
	uint8_t buf[size];
	memcpy(buf, state_vect.table, sizeof(state_vect.table));
	tcp_send(TCP_RC_STATEVECT_MID, size, buf);
}

void tcp_send_localization_result(int32_t da, int32_t dx, int32_t dy, uint8_t success_code, int32_t score)
{
	PR_FUNC();
}


int tcp_send_msg(tcp_message_t* msg_type, void* msg)
{
	PR_FUNC();

	return 0;
}

/*
Return value:
< 0: Error.
0: Message was not parsed (not fully received yet)
>0: Message ID of the parsed message.
*/

int tcp_parser(int sock)
{
	int ret;
	static int state = 0; // Num of bytes read
	static struct __attribute__ ((packed)) { uint8_t mid_msb; uint8_t mid_lsb; uint8_t size_msb; uint8_t size_middle; uint8_t size_lsb;} header; // Stored header of incoming message
	static int bytes_left = 0;
	static tcp_message_t* msg = 0; // Once message is recognized, pointer to message type struct.
	static int unrecog = 0;

	static uint8_t buf[65536];
	static uint8_t* p_buf = buf;

	if(state < 5)
	{
		msg = 0;
		unrecog = 0;
		ret = read(sock, (uint8_t*)&(header.mid_msb) + state, 5-state);
		if(ret < 0)
		{
			fprintf(stderr, "ERROR: TCP stream read error %d (%s)\n", errno, strerror(errno));
			state = 0;
			return -11;
		}
		else if(ret == 0)
		{
			fprintf(stderr, "Client closed connection\n");
			state = 0;
			return -10;
		}
		else
			state += ret;
	}

	if(state >= 5)
	{
		int mid = ((int)header.mid_msb<<8) | ((int)header.mid_lsb);
		if(msg == 0 && unrecog == 0)
		{		
			for(int i=0; i<NUM_CR_MSGS; i++)
			{
				if(CR_MSGS[i]->mid == mid)
				{
					msg = CR_MSGS[i];
					break;
				}
			}

			int size_from_header = ((int)header.size_msb<<16) | ((int)header.size_middle<<8) | (int)header.size_lsb;
			if(!msg)
			{
				fprintf(stderr, "WARN: Ignoring unrecognized message with msgid 0x%02x\n", mid);
				unrecog = 1;
			}
			else if(size_from_header != msg->size)
			{
				fprintf(stderr, "WARN: Ignoring message with msgid 0x%02x because of size mismatch (got:%u, expected:%u)\n",
					mid, size_from_header, msg->size);
				unrecog = 1;
			}
			bytes_left = size_from_header;
			p_buf = buf;
		}

		// Read as much data as the size field shows.
		if(bytes_left)
		{
			ret = read(sock, p_buf, bytes_left);
			if(ret < 0)
			{
				fprintf(stderr, "ERROR: TCP stream read error %d (%s), closing socket.\n", errno, strerror(errno));
				tcp_comm_close();
			}
			else if(ret == 0)
			{
				fprintf(stderr, "ERROR: TCP stream read() returned 0 bytes even when it shoudln't, closing socket.\n");
				tcp_comm_close();
			}
			else
			{
				bytes_left -= ret;
				p_buf += ret;
			}
		}

		if(bytes_left < 0)
		{
			fprintf(stderr, "ERROR: bytes_left < 0, closing socket.\n");
			tcp_comm_close();
			bytes_left = 0;
		}

		if(bytes_left == 0)
		{
			state = 0;

			if(!unrecog)
			{
				// Parse the message
				uint8_t* p_src = buf;
				void* p_dest = msg->p_data;

				if(p_dest == 0)
				{
					fprintf(stderr, "ERROR: message parsing destination pointer NULL\n");
					return -1;
				}

				for(int field=0;;field++)
				{
					switch(msg->types[field])
					{
						case 'b':
						case 'B':
							*((uint8_t*)p_dest) = *(p_src++);
							p_dest = ((uint8_t*)p_dest) + 1;
						break;

						case 's':
						case 'S':
						{
							uint16_t tmp  = ((uint16_t)(*(p_src++))<<8);
								 tmp |=  *(p_src++);
							*((uint16_t*)p_dest) = tmp;
							p_dest = ((uint16_t*)p_dest) + 1;
						}
						break;

						case 'i':
						case 'I':
						{
							uint32_t tmp  = ((uint32_t)(*(p_src++))<<24);
								 tmp |= ((uint32_t)(*(p_src++))<<16);
								 tmp |= ((uint32_t)(*(p_src++))<<8);
								 tmp |=  *(p_src++);
							*((uint32_t*)p_dest) = tmp;
							p_dest = ((uint32_t*)p_dest) + 1;
						}
						break;

						case 'l':
						case 'L':
						{
							uint64_t tmp  = ((uint64_t)(*(p_src++))<<56);
								 tmp |= ((uint64_t)(*(p_src++))<<48);
								 tmp |= ((uint64_t)(*(p_src++))<<40);
								 tmp |= ((uint64_t)(*(p_src++))<<32);
								 tmp |= ((uint64_t)(*(p_src++))<<24);
								 tmp |= ((uint64_t)(*(p_src++))<<16);
								 tmp |= ((uint64_t)(*(p_src++))<<8);
								 tmp |=  *(p_src++);
							*((uint64_t*)p_dest) = tmp;
							p_dest = ((uint64_t*)p_dest) + 1;
						}
						break;


						case 0:
							goto PARSE_END;

						default:
							fprintf(stderr, "ERROR: parse type string has invalid character 0x%02x\n", msg->types[field]);
							return -2;

					}
				}

				PARSE_END: ;
				return mid;
			}
		}

	}

	return 0;
}

