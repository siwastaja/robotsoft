#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "voxmap.h"
#include "voxmap_memdisk.h"


int main()
{
	po_coords_t po;

	po = po_idx_coords(12345, 23456, 500, 0);

	load_pages(0b1111, 0b1111, po.px-2, po.px+2, po.py-2, po.py+2, po.pz-2, po.pz+2);

	for(int i = 0; i < 20; i++)
	{
		mem_manage_pages(25);
	}

	mark_page_accessed(po.px, po.py, po.pz);

	for(int i = 0; i < 20; i++)
	{
		mem_manage_pages(25);
	}
	

	return 0;
}
