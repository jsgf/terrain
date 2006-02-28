/* 
   Generate the 16 different forms of patch geometry
 */
#include <stdio.h>
#include <GL/gl.h>
#include "quadtree.h"

int main()
{
	printf("#include <GL/gl.h>\n");
	printf("#include \"quadtree.h\"\n\n");
	printf("const patch_index_t patch_indices[9][INDICES_PER_PATCH] = {\n");

	/* This generates 9 sets of indices for the 9 possible
	   combinations of neighbour relations.  While there are 4
	   sides to a patch, and any side may be adjacent to a patch
	   with a lower level, it isn't possible to have two opposite
	   sides adjacent to a lower-level patch.

	   This means the possible combinations are:

	   UD LR
	   00 00
	   00 01
	   00 10

	   01 00
	   01 01
	   01 10

	   10 00
	   10 01
	   10 10
	*/

#define L	(1<<1)
#define R	(1<<0)
#define U	(1<<1)
#define D	(1<<0)
	for(int ud = 0; ud < 3; ud++) {
		for(int lr = 0; lr < 3; lr++) {
			printf("\t/* ud=%d lr=%d */\n", ud, lr);
			printf("\t{ ");
			for(int y = 0; y < MESH_SAMPLES-1; y++) {
				for(int x = 0; x < MESH_SAMPLES; x++) {
					int xmask0 = ~0, xadd0 = 0;
					int xmask1 = ~0, xadd1 = 0;
					int ymask = ~0, yadd = 0;

					if ((lr & L) && x == 0) {
						ymask = ~1;
						yadd = 1;
					}

					if ((lr & R) && x == MESH_SAMPLES-1)
						ymask = ~1;

					if ((ud & D) && y == 0) {
						xmask0 = ~1;
						xadd0 = 1;
					}

					if ((ud & U) && y == MESH_SAMPLES-2)
						xmask1 = ~1;

					if (y != 0 && x == 0)
						printf("\n\t  %u, ", ((y+1 + yadd) & ymask) * MESH_SAMPLES + ((x + xadd1) & xmask1));

					printf("%u, %u, ",
					       ((y+1 + yadd) & ymask) * MESH_SAMPLES + ((x + xadd1) & xmask1),
					       ((y+0 + yadd) & ymask) * MESH_SAMPLES + ((x + xadd0) & xmask0));

					if (y != MESH_SAMPLES-2 && x == MESH_SAMPLES-1)
						printf("%u, ", ((y+0 + yadd) & ymask) * MESH_SAMPLES + ((x + xadd0) & xmask0));
				}

			}
			printf(" },\n\n");
		}
	}
	printf("};\n");
}
