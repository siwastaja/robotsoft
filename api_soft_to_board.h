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

typedef struct __attribute__((packed))
{
	uint64_t subs_vector[4]; // 1 bit per message ID (1 = on, 0 = off), [0] LSb = id0, [0] MSb = id63, [1] LSb = id64, and so on.

} s2b_subscribe_t;


