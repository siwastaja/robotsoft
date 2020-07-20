#include <inttypes.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <zlib.h>
#include <assert.h>

#include "small_cloud.h"

#define SAVE_ZLIB_LEVEL 2

void save_small_cloud(const char* fname, int32_t ref_x, int32_t ref_y, int32_t ref_z, int n_points, small_cloud_t* points)
{
	assert(n_points < 5000000);

	int max_compressed_size = sizeof(small_cloud_t)*n_points + 128; // A bit extra for zlib overhead
	int max_size = sizeof(small_cloud_header_t) + max_compressed_size; 
	uint8_t* buf = malloc(max_size);

	small_cloud_header_t head;
	head.magic = 0xaa14;
	head.api_version = 0x0420;
	head.compression = 1;
	head.dummy = 0;
	head.ref_x_mm = ref_x;
	head.ref_y_mm = ref_y;
	head.ref_z_mm = ref_z;
	head.n_points = n_points;

	memcpy(buf, &head, sizeof(small_cloud_header_t));

	z_stream strm;
	strm.zalloc = Z_NULL;
	strm.zfree = Z_NULL;
	strm.opaque = Z_NULL;
	if(deflateInit(&strm, SAVE_ZLIB_LEVEL) != Z_OK)
	{
		printf("ERROR: ZLIB initialization failed\n");
		abort();
	}
	strm.avail_in = n_points * sizeof(small_cloud_t);
	strm.next_in = (uint8_t*)points;

	strm.avail_out = sizeof(small_cloud_t)*n_points + 128;
	strm.next_out = buf + sizeof(small_cloud_header_t);

	int ret = deflate(&strm, Z_FINISH);
	assert(ret != Z_STREAM_ERROR);

	int produced = max_compressed_size - strm.avail_out;

	assert(strm.avail_out > 0);
	assert(produced > 0);
	assert(strm.avail_in == 0);

	deflateEnd(&strm);

	FILE* f = fopen(fname, "wb");
	assert(f);
	assert(fwrite(buf, sizeof(small_cloud_header_t) + produced, 1, f) == 1);
	fclose(f);
	free(buf);
}

