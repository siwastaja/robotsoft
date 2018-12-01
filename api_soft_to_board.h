#pragma once

/*
	robotsoft -> robotboard SPI messages

	First 8 bytes: header

		u32 little endian 0x9876fedb: 	Reserved maintenance magic code (runs the flasher)
		u32 don't care

		OR

		u16 little endian 0x2345:     	Packet will include something (don't ignore it)
		u8: Reserved
		u8: # of commands		Number of command messages in the packet
		u32 Reserved

	Then: The commands. Each one has a 4-byte header:
		u8: msgid			Message id (what the command is about)
		u8: 				Reserved
		u16: len			Message payload length (in bytes)

*/

#define S2B_MAX_LEN 4096

#define S2B_HEADER_LEN 8
#define S2B_TOTAL_OVERHEAD (S2B_HEADER_LEN)


#if (S2B_MAX_LEN%8 != 0)
#error S2B_MAX_LEN must be multiple of 8
#endif


typedef struct __attribute__((packed))
{
	uint16_t magic;
	uint8_t reserved1;
	uint8_t n_cmds;
	uint32_t reserved2;
} s2b_header_t;

typedef struct __attribute__((packed))
{
	uint8_t msgid;
	uint8_t reserved;
	uint16_t paylen;
} s2b_cmdheader_t;



#define CMD_SUBSCRIBE 1
typedef struct __attribute__((packed))
{
	uint64_t subs_vector[B2S_SUBS_U64_ITEMS]; // 1 bit per message ID (1 = on, 0 = off), [0] LSb = id0, [0] MSb = id63, [1] LSb = id64, and so on.

} s2b_subscribe_t;

#define CMD_MOVE_REL 2
typedef struct __attribute__((packed))
{
	int32_t dx;
	int32_t dy;
	uint32_t reserved;
} s2b_move_rel_t;

#define CMD_MOVE_ABS 3
typedef struct __attribute__((packed))
{
	int32_t x;
	int32_t y;
	uint32_t reserved;
} s2b_move_abs_t;

#define CMD_ACK_ERROR 4
typedef struct __attribute__((packed))
{
	uint32_t reserved;
} s2b_ack_error_t;


// For Pulu's internal factory use
#define CMD_CALIBRATION 5
typedef struct __attribute__((packed))
{
	uint8_t cmd;
	int32_t param1;
	int32_t param2;
	int32_t param3;
	int32_t param4;
	int32_t param5;
} s2b_calibration_t;








typedef struct __attribute__((packed))
{
	uint16_t size;
} s2b_message_t;

#define S2B_MESSAGE_STRUCT(msg_t) {sizeof(msg_t)}

#ifdef DEFINE_API_VARIABLES

s2b_message_t const s2b_msgs[256]  =
{
	{0},
	S2B_MESSAGE_STRUCT(s2b_subscribe_t),
	S2B_MESSAGE_STRUCT(s2b_move_rel_t),
	S2B_MESSAGE_STRUCT(s2b_move_abs_t),
	S2B_MESSAGE_STRUCT(s2b_ack_error_t),
	S2B_MESSAGE_STRUCT(s2b_calibration_t),
	{0}
};


#endif

