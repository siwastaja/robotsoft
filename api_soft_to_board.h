#pragma once

/*
	robotsoft -> robotboard SPI messages

	First 4 bytes: header

		u32 little endian 0x9876fedb: 	Reserved maintenance magic code (runs the flasher)
		u16 little endian 0x2345:     	Packet will include something (don't ignore it)
			u8:			Reserved
			u8: # of commands	Number of command messages in the packet

	Then: The commands. Each one has a 4-byte header:
		u8: msgid			Message id (what the command is about)
		u8: 				Reserved
		u16: len			Message payload length (in bytes)

*/

#define S2B_MAX_LEN 6000

#define S2B_HEADER_LEN 8
#define S2B_TOTAL_OVERHEAD (S2B_HEADER_LEN)


typedef struct __attribute__((packed))
{
	uint8_t msgid;
	uint8_t reserved;
	uint16_t paylen;
} s2b_cmdheader_t;




typedef struct __attribute__((packed))
{
	uint64_t subs_vector[4]; // 1 bit per message ID (1 = on, 0 = off), [0] LSb = id0, [0] MSb = id63, [1] LSb = id64, and so on.

} s2b_subscribe_t;

typedef struct __attribute__((packed))
{
	int32_t dx;
	int32_t dy;
	uint32_t reserved;
} s2b_move_rel_t;


MAYBE_EXTERN s2b_subscribe_t* s2b_subscribe;
MAYBE_EXTERN s2b_move_rel_t*  s2b_move_rel;

#ifdef DEFINE_API_VARIABLES
void * * const p_p_s2b_msgs[256]  =
{
	0,
	(void**)&s2b_subscribe,
	(void**)&s2b_move_rel,
	0
};

uint16_t const s2b_msg_sizes[256] =
{
	0,
	sizeof(s2b_subscribe_t),
	sizeof(s2b_move_rel_t),
	0
};
#endif

