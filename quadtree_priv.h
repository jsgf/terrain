#ifndef _QUADTREE_PRIV_H
#define _QUADTREE_PRIV_H

#include "quadtree.h"

#define MESH_SAMPLES	(PATCH_SAMPLES+1)

/* Number of indicies needed to construct a triangle strip to cover a
   whole patch mesh, including the overhead to stitch the strip
   together. */
#define INDICES_PER_PATCH	((2*MESH_SAMPLES) * (MESH_SAMPLES-1) + (2*(MESH_SAMPLES-2)))

#define USE_INDEX	1	

#if USE_INDEX
#define VERTICES_PER_PATCH	(MESH_SAMPLES * MESH_SAMPLES)
#else  /* !USE_INDEX */
#define VERTICES_PER_PATCH	INDICES_PER_PATCH
#endif	/* USE_INDEX */

#if INDICES_PER_PATCH < 256
typedef GLubyte patch_index_t;
#define PATCH_INDEX_TYPE	GL_UNSIGNED_BYTE
#else
typedef GLshort patch_index_t;
#define PATCH_INDEX_TYPE	GL_UNSIGNED_SHORT
#endif

extern const patch_index_t patch_indices[9][INDICES_PER_PATCH];


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

	box_t bbox;

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
#define PF_CULLED	(1<<0)	/* not visible */
#define PF_UNUSED	(1<<1)	/* no valid contents */
#define PF_ACTIVE	(1<<2)	/* active part of the structure */
#define PF_UPDATE_GEOM	(1<<3)	/* geometry needs updating */
#define PF_STITCH_GEOM	(1<<4)	/* geometry needs stitching */

#define PF_LATECULL	(1<<5)

	int phase;

	int pinned;		/* pinned count; this is set to non-0
				   when this patch is required to
				   remain as-is */

	/* Patch priority.  When a patch is visible, higher priority
	   means a patch is more splittable.  When a patch is culled,
	   higher priority means a patch is more
	   mergable/recyclable. */
	float priority;

	float error;		/* accumulated error from desired target size */

	struct list_head list;	/* list pointers for whatever list we're on */

	/* Offset into the vertex array, in units of
	   VERTICES_PER_PATCH */
	unsigned vertex_offset;

	unsigned char col[4];
};

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
	unsigned nvisible;

	/* The freelist; things are added to the tail and removed from
	   the head, giving an LRU reuse order.  These patches still
	   contain useful information so they're ready to be reused in
	   their old positions, but they can also be recycled for use
	   elsewhere in the terrain.  If the freelist gets too empty,
	   the allocator will automatically start merging patches to
	   add more patches to the freelist. */
	struct list_head freelist;
	unsigned nfree;
	int reclaim;		/* currently reclaiming patches */

	/* Array of patch structures.  All patches are allocated out
	   of this pool.  It is fixed size. */
	int npatches;
	struct patch *patches;

	GLuint vtxbufid;	/* ID of vertex buffer object (0 if not used) */
	struct vertex *varray;	/* vertex array (NULL if using a VBO) */

	int phase;		/* used for marking patches */

	/* Radius of the terrain sphere, and the function used to
	   generate elevation for a particular point on its
	   surface. */
	long radius;
	elevation_t (*landscape)(const vec3_t *v, GLubyte col[4]);
};

#endif	/* _QUADTREE_PRIV_H */
