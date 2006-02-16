#ifndef QUADTREE_H
#define QUADTREE_H

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

	/* List pointers on either the list/merge queue, or on the
	   freelist */
	struct patch *prev, *next;

	unsigned long id;	/* Node identifier in quadtree.  Child
				   nodes are numbered:
				       32
				       01
				 */

	signed char level;	/* level in quadtree */
	unsigned char priority;	/* high-priority: more splittable;
				   low: more mergable */

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

	/* 
	   But needs (N+1)x(N+1) samples to actually draw; the N+1
	   edge samples are derived from the right/top neighbour's 0
	   row/column samples.  If either neighbour is at a coarser
	   resolution, the mesh uses a fan-structure to fill the gaps.

	   If the left and/or bottom neighbours are coarser, then we
	   also need to use their vertices rather than our own to fill
	   cracks: (M = (N/2)-1)

	     M0|N1 N2 N3 ... NN
	     : |:   :  :      :
             20|41  :  :    / :
	       |31  :  :   /  :
             10|21  :  :  /   :
	       |11 12 13 ... 1N
	     00|01 02 03 ... 0N
	 */
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
	/* List of all active patches, in (approx) priority order.
	   Patches are on a circular list in each bucket; the head is
	   highest priority, tail lowest.  Patches are added to the
	   head or tail depending on how its priority compares to the
	   current head or tail; no finer-grained sorting is used.  */
	struct patch *active[PRIO_BUCKETS];
	
	/* pointer to head of circular freelist; things are added to
	   the tail and removed from the head, giving an LRU reuse
	   order */
	struct patch *freelist;

	struct patch *patches;
};


struct quadtree *quadtree_create(int num_patches, int radius);

#endif	/* QUADTREE_H */
