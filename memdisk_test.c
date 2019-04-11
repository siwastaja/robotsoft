#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "voxmap.h"
#include "voxmap_memdisk.h"


int main()
{
	po_coords_t po;

	po = po_coords(12345, 23456, 500, 0);

	printf("\n\n1\n");
	load_pages(0b1111, 0b1111, po.px-2, po.px+2, po.py-2, po.py+2, po.pz-2, po.pz+2);

	po = po_coords(12345, 23456, 500, 0);

	printf("\n\n2\n");
	load_pages(0b1111, 0b1111, po.px-2, po.px+2, po.py-2, po.py+2, po.pz-2, po.pz+2);

	po = po_coords(12345, 33333, 500, 0);

	printf("\n\n3\n");
	load_pages(0b1111, 0b1111, po.px-2, po.px+2, po.py-2, po.py+2, po.pz-2, po.pz+2);

	po = po_coords(50000, -50000, 500, 0);

	printf("\n\n4\n");
	load_pages(0b1111, 0b1111, po.px-2, po.px+2, po.py-2, po.py+2, po.pz-2, po.pz+2);

	po = po_coords(50000, -500000, 10000, 0);

	printf("\n\n5\n");
	load_pages(0b1111, 0b1111, po.px-2, po.px+2, po.py-2, po.py+2, po.pz-2, po.pz+2);

	po = po_coords(12345, 23456, 500, 0);

	printf("\n\n6\n");
	load_pages(0b1111, 0b1111, po.px-2, po.px+2, po.py-2, po.py+2, po.pz-2, po.pz+2);

	VOXEL(*P_VOXMAP(po.px, po.py, po.pz, 0), po.ox, po.oy, po.oz) = 0x0f;
	mark_page_changed(po.px, po.py, po.pz);

	po = po_coords(700000, 500000, -10000, 0);

	printf("\n\n7\n");
	load_pages(0b1111, 0b1111, po.px-2, po.px+2, po.py-2, po.py+2, po.pz-2, po.pz+2);

	po = po_coords(800000, 500000, -10000, 0);

	printf("\n\n8\n");
	load_pages(0b1111, 0b1111, po.px-2, po.px+2, po.py-2, po.py+2, po.pz-2, po.pz+2);

	po = po_coords(809000, 500000, -10000, 0);

	printf("\n\n9\n");
	load_pages(0b1111, 0b1111, po.px-2, po.px+2, po.py-2, po.py+2, po.pz-2, po.pz+2);

	po = po_coords(12345, 23456, 500, 0);

	printf("\n\n10\n");
	load_pages(0b1111, 0b1111, po.px-2, po.px+2, po.py-2, po.py+2, po.pz-2, po.pz+2);


	free_all_pages();
/*
	for(int i = 0; i < 20; i++)
	{
		mem_manage_pages(25);
	}

	mark_page_accessed(po.px, po.py, po.pz);

	for(int i = 0; i < 20; i++)
	{
		mem_manage_pages(25);
	}
	*/

	return 0;
}
