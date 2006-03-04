#ifndef QUADTREE_H
#define QUADTREE_H

#include "list.h"
#include "geom.h"

/* 
   A patch is a node in the quadtree.  The tree starts with N root
   patches, at level 0 (where N is either 1 or 6, depending on whether
   the world has a planar or cubic basis).

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
   reallocated as the parent).  This means the rendering complexity is
   always fixed.

   Patches are allocated from a freelist.  If a patch is required and
   the freelist is empty, then the lowest priority existing patch is
   merged, and the sub-patches are added to the freelist.  When
   patches are added to the freelist, they're left intact; if a patch
   is required again when it is still on the freelist, it is removed
   and used as-is.  In other words, the freelist is also a cache of
   recently used patches.

   Each patch has N^2 samples.  The mesh generated for each patch has
   (N+1)^2 samples; the +1 row/column are copied from neighbouring
   patches.
 */

#define PATCH_SAMPLES	8	/* N */

struct quadtree;
struct patch;

typedef long elevation_t;	/* basic sample type of a heightfield */

struct quadtree *quadtree_create(int num_patches, long radius,
				 elevation_t (*generator)(const vec3_t *v, GLubyte colour[4]));
void quadtree_update_view(struct quadtree *qt, const matrix_t *mat);
void quadtree_render(const struct quadtree *qt, void (*prerender)(const struct patch *p));

int patch_level(const struct patch *p);
unsigned long patch_id(const struct patch *p);
char *patch_name(const struct patch *p, char buf[16 * 2 + 1]);

#endif	/* QUADTREE_H */
