#ifndef QUADTREE_H
#define QUADTREE_H

#include "list.h"

/* 
   A patch is a node in the quadtree.  The tree starts with N root
   patches, at level 0 (where N is either 1 or 6, depending on whether
   the world has a planar or cubic basis).

   All but root patchs logically have a parent, and all have 2, 3 or 4
   neighbours; when using a cubic basis, all patches have 4
   neighbours.

   Every patch has the same number of samples; higher resolution is
   gained by drawing the same number of samples at a smaller scale.
   This means that all patches at all resolutions have exactly the
   same representation.

   Neighbouring patches may only differ by one level in tree height.
   In order to avoid cracks, a finer patch ignores its own edge
   vertices adjacent to the coarser patch, and uses the coarse patch
   vertices in a fan-like shape.  A fine patch can have at most two
   coarse neighbours.

   A patch is split by replacing it by four sub-patches; the parent
   patch is not kept.  This means the in-memory representation is only
   of the leaves of the quadtree.  This means there are always a fixed
   number of patches, regardless of the level of subdivision.

   Because there are always a fixed number of patches, there is
   therefore always has a fixed number of primitives to draw.  It
   maintains a queue of all patches in priority order; patches with
   lot priority are merged in order to free up patches for use when
   splitting (each merge releases 3 patches for reuse; the 4th is
   reallocated as the implicit parent).  This means the rendering
   complexity is always fixed, and.

   Patches are allocated from a freelist.  If a patch is required and
   the freelist is empty, then the lowest priority existing patch is
   merged, and the sub-patches are added to the freelist.  When
   patches are added to the freelist, they're left intact; if a patch
   is required again when it is still on the freelist, it is removed
   and used as-is.  In other words, the freelist is also a cache of
   recently used patches.

   TO DECIDE: does each patch 1) need copies of the neighbours edge
   vertices, or 2) indexes them directly?

   1) has the advantage of having a small fixes set of possible patch
   topologies, which means we can use a small set of fixed index
   buffers for drawing.  This disadvantage is that if one of the patch
   neighbours splits, merges or is otherwise modified, we need to
   update the copies of all the affected vertices.

   The PSP is much faster when not using indexing at all, so 1 with a
   linearized vertex array would be most performance efficient (but
   not memory efficient).

   2) has the advantage of only requiring an update of the indices of
   to neighbouring vertices if the neighbour splits or merges, and no
   change at all if the vertices are modified.  This disadvantage is
   that each patch would need a unique set of indices for indexing
   neighbouring vertices.
 */

#define PATCH_SAMPLES	16	/* N */
#define VERTICES_PER_PATCH	((PATCH_SAMPLES + 1) * (PATCH_SAMPLES + 1))

typedef short elevation_t;	/* basic sample type of a heightfield */

/*
  These are the offsets in the neighbour array.  There are two of each
  neighbour pointer, to deal with split neighbours (ie, at a higher
  level).  If the neighbour's level is <= the patch, then both point
  to the neighbour.
 */
enum patch_neighbour {
	PN_RIGHT	= 0,
	PN_RIGHT_1	= 1,
	PN_UP		= 2,
	PN_UP_1		= 3,
	PN_LEFT		= 4,
	PN_LEFT_1	= 5,
	PN_DOWN		= 6,
	PN_DOWN_1	= 7,

	PN_BADDIR	= -1
};

enum patch_sibling {
	SIB_DL = 0,		/* down, left */
	SIB_DR = 1,		/* down, right */
	SIB_UR = 2,		/* up, right */
	SIB_UL = 3,		/* up, left */
};

struct patch {
	/* Coords of patch edges.  These are integers so that the
	   exact values are deterministic (useful as seeds for
	   proceedural generators).  The coords are in 3 dimensions,
	   but the patch is parallel to one of the axies. */
	signed int x0, x1, y0, y1, z0, z1;

	/* The parent-child links are not really used as part of the
	   quadtree structure, since the parent is replaced by the
	   children on split.  But on split/merge the old
	   parent/children are added to the freelist and preserved
	   until they're reallocated.  If the parent/child is still
	   around when this patch is merged/split, then they can just
	   be reused directly; that's what these references are for.
	   If a patch referred to here is reallocated, then these are
	   all NULLed out. */
	struct patch *parent;
	struct patch *kids[4];

	/* 
	   The patch can have up to 8 neighbours: 4 sides, 2 per side.
	   If a side only has 1 neighbour, then both pointers point to
	   the same neighbour; otherwise the even points to the 0-N/2
	   neighbour, and the odd points to N/2-N neighbour.
	 */
	struct patch *neigh[8];

	/* Node identifier in quadtree.  This represents the path down
	   the tree structure to this patch; it can only be interpreted
	   properly in conjunction with the level. */
	unsigned long id;
	unsigned char level;	/* level in quadtree */

	unsigned flags;
#define PF_VISITED	(1<<0)	/* visited in this pass */
#define PF_CULLED	(1<<1)	/* not visible */
#define PF_UNUSED	(1<<2)	/* no valid contents */
#define PF_ACTIVE	(1<<3)	/* active part of the structure */

	int pinned;		/* pinned count; this is set to non-0
				   when this patch is required to
				   remain as-is */

	int priority;	/* high-priority: more splittable;
			   low: more mergable */

	struct list_head list;	/* list pointers for whatever list we're on */

	/* Offset into the vertex array, in units of
	   VERTICES_PER_PATCH */
	unsigned short vertex_offset;

	/* 
	   Each patch has NxN samples:

	     N0 N1 N2 N3 ... NN
             :  :  :  :    / :
	     :  :  :  :   /  :
             :  :  :  :  /   :
	     10 11 12 13 ... 1N
	     00 01 02 03 ... 0N
	 */
	elevation_t samples[PATCH_SAMPLES * PATCH_SAMPLES]; /* core samples */

	unsigned char col[4];
};

enum plane {
	P_XY,
	P_YZ,
	P_ZX,
};

struct vertex {
	unsigned char s,t;
	unsigned short col;
	signed char nx, ny, nz;	/* needed? */
	float x,y,z;		/* short? */
};

#define PRIO_BUCKETS	16
struct quadtree {
	/* List of all visible patches, in priority order.  These are
	   all the visible patches which are currently part of the
	   terrain. */
	struct list_head visible;
	/* Culled patches are not visible.  They're still active
	   (they're required for the terrain to have proper topology),
	   but they're not visible.  They are therefore prime
	   candidates for merging (and therefore free up 3 patches per
	   merge). */
	struct list_head culled;

	unsigned nactive;		/* number of culled+visible patches */

	/* The freelist; things are added to the tail and removed from
	   the head, giving an LRU reuse order.  These patches still
	   contain useful information so they're ready to be reused in
	   their old positions, but they can also be recycled for use
	   elsewhere in the terrain. */
	struct list_head freelist;
	unsigned nfree;
	int reclaim;		/* currently reclaiming patches */

	/* Array of patch structures.  All patches are allocated out
	   of this pool.  It is fixed size. */
	int npatches;
	struct patch *patches;

	/* ID of vertex buffer object */
	GLuint vtxbufid;

	/* Radius of the terrain sphere, and the function used to
	   generate elevation for a particular point on its
	   surface. */
	long radius;
	int (*landscape)(int x, int y, int z);
};


struct quadtree *quadtree_create(int num_patches, long radius,
				 int (*generator)(long x, long y, long z));
void quadtree_update_view(struct quadtree *qt, 
			  const float modelview[16],
			  const float projection[16],
			  const int viewport[4]);

#endif	/* QUADTREE_H */
