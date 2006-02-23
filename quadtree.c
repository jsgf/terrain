#include <stddef.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>

#include "quadtree.h"

static int patch_merge(struct quadtree *qt, struct patch *p);

static inline int clamp(int x, int lower, int upper)
{
	if (x > upper)
		x = upper;
	if (x < lower)
		x = lower;
	return x;
}

/* ID management.  The ID is an integer which uniquely identifies all
   nodes in the quadtree.  It uses 2 bits per level. */

static const char *id2str(const struct patch *p)
{
	int id = p->id;
	int level = p->level;
	static char buf[40];
	char *cp = buf;

	//cp += sprintf(cp, "%d:", level);
	//*cp++ = '>';
	cp += sprintf(cp, "%d:", id >> (level * 2));

	for(int i = level-1; i >= 0; i--) {
		cp += sprintf(cp, "%d.", (id >> (i * 2)) & 3);
	}
	cp[-1] = '\0';

	return buf;
}

static void emitdotpatch(FILE *f, const struct patch *p)
{
	static const char *dirname[] = {
#define DN(x)	[PN_##x] = #x, [PN_##x##_1] = #x "_1"
		DN(RIGHT),
		DN(LEFT),
		DN(UP),
		DN(DOWN)
#undef DN
	};

	fprintf(f, "\t\"%p\" [label=\"%s\", shape=%s];\n",
		p, id2str(p), (p->flags & PF_CULLED) ? "box" : "diamond");
		
	for(enum patch_neighbour d = 0; d < 8; d += 2) {
		if (p->neigh[d] == p->neigh[d+1]) {
			fprintf(f, "\t\t\"%p\" -> \"%p\" [label=\"%s+\"];\n", 
				p, p->neigh[d], dirname[d]);
		} else {
			fprintf(f, "\t\t\"%p\" -> \"%p\" [label=\"%s\"];\n", 
				p, p->neigh[d], dirname[d]);
			fprintf(f, "\t\t\"%p\" -> \"%p\" [label=\"%s\"];\n", 
				p, p->neigh[d+1], dirname[d+1]);
		}
	}

}

void emitdot(struct quadtree *qt, const char *name)
{
	FILE *f = fopen(name, "w");

	fprintf(f, "digraph \"%s\" {\n", name);

	struct list_head *pp;

	list_for_each(pp, &qt->visible) {
		struct patch *p = list_entry(pp, struct patch, list);

		emitdotpatch(f, p);
	}
	list_for_each(pp, &qt->culled) {
		struct patch *p = list_entry(pp, struct patch, list);

		emitdotpatch(f, p);
	}
	fprintf(f, "}\n");
}

static int on_freelist(const struct quadtree *qt, const struct patch *p)
{
	struct list_head *pp;

	list_for_each(pp, &qt->freelist) {
		const struct patch *fp = list_entry(pp, struct patch, list);

		if (fp == p)
			return 1;
	}
	return 0;
}

/* Construct an ID for a child */
static unsigned long childid(unsigned long parentid, unsigned char id)
{
	return (parentid << 2) | id;
}

/* Return the ID for a node's Nth parent */
static unsigned long parentid(const struct patch *p, int n)
{
	return p->id >> (2 * n);
}

static enum patch_sibling siblingid(const struct patch *p)
{
	return p->id % 4;
}

/* Find neighbour 'n' of patch 'p', and find the appropriate backwards
   direction by looking for 'oldp', which is presumably a reference we
   want to replace.  Always returns an even direction. */
static inline enum patch_neighbour neigh_opposite(const struct patch *p, enum patch_neighbour n, 
						  const struct patch *oldp)
{
	const struct patch *np = p->neigh[n];

	for(enum patch_neighbour dir = 0; dir < 8; dir++)
		if (np->neigh[dir] == oldp)
			return dir & ~1;

	return PN_BADDIR;
}

static inline int are_siblings(const struct patch *p1, const struct patch *p2)
{
	return p1->level == p2->level &&
		parentid(p1, 1) == parentid(p2, 1);
}

#if 0
static int is_updown(enum patch_neighbour n)
{
	n &= ~1;
	return n == PN_UP || n == PN_DOWN;
}
#endif

static int is_leftright(enum patch_neighbour n)
{
	n &= ~1;
	return n == PN_LEFT || n == PN_RIGHT;
}

/* List of neighbours for a patch. ccw and cw refer to the patch's
   siblings (neighbours sharing a parent), and lr+ud are the patch's
   horizontal and vertical neighbours. */
static const struct neighbours {
	enum patch_neighbour ccw, cw, lr, ud;
} neighbours[] = {
	[SIB_DL] = { PN_RIGHT, PN_UP,    PN_LEFT,  PN_DOWN },
	[SIB_DR] = { PN_UP,    PN_LEFT,  PN_RIGHT, PN_DOWN },
	[SIB_UR] = { PN_LEFT,  PN_DOWN,  PN_RIGHT, PN_UP   },
	[SIB_UL] = { PN_DOWN,  PN_RIGHT, PN_LEFT,  PN_UP   },
};

static const struct {
	enum patch_sibling ccw, cw;
	char sx, sy;		/* x,y of sibling */
} siblings[] = {
	[SIB_DL] = { SIB_DR, SIB_UL, 0, 0 },
	[SIB_DR] = { SIB_UR, SIB_DL, 1, 0 },
	[SIB_UR] = { SIB_UL, SIB_DR, 1, 1 },
	[SIB_UL] = { SIB_DL, SIB_UR, 0, 1 },
};

/* Predicate to check the constraint that neighbours should only
   differ by at most 1 level, and that when pointing to a neighbour
   with <= level, both neighbour pointers should point to it.  */
static int check_neighbour_levels(const struct patch *p)
{
	for(enum patch_neighbour dir = 0; dir < 8; dir++) {
		const struct patch *n = p->neigh[dir];

		if (n == NULL) {
			goto fail;
		}

		if (abs(n->level - p->level) > 1) {
			printf("p->level=%d n(%d)->level=%d\n",
			       p->level, dir, n->level);
			goto fail;
		}

		if (p->level >= n->level) {
			if (p->neigh[dir] != p->neigh[dir ^ 1]) {
				goto fail;
			}
		} else {
			if (p->neigh[dir] == p->neigh[dir ^ 1]) {
				printf("p->level=%d n(%d)->level=%d\n",
				       p->level, dir, n->level);
				goto fail;
			}
		}
	}

	return 1;
  fail:
	return 0;
}

/* Once a patch has been linked to all its neighbours, then fix up all
   the neighbour's backlinks */
static void backlink_neighbours(struct patch *p, struct patch *oldp)
{
	enum patch_sibling sib = siblingid(p);

	assert(check_neighbour_levels(p));

	for(enum patch_neighbour dir = 0; dir < 8; dir++) {
		enum patch_neighbour opp = neigh_opposite(p, dir, oldp);
		struct patch *n = p->neigh[dir];

		if (opp == PN_BADDIR)
			continue;

		assert(abs(n->level - p->level) <= 1);

		if (p->level <= n->level) {
			/* p is bigger, so both n->neigh pointers point to it */
			n->neigh[opp+0] = p;
			n->neigh[opp+1] = p;
		} else {
			int idx = is_leftright(dir) ? siblings[sib].sy : siblings[sib].sx;
			n->neigh[opp + idx] = p;
		}			
	}
}

/* Update p's neighbour links from its own siblings and its parent's
   neighbours.  A precondition is that all the neighbours have had their
   levels adjusted appropriately. */
static void link_neighbours_from_parent(struct patch *p)
{
	struct patch *parent = p->parent;

	enum patch_sibling sib = siblingid(p);
	enum patch_sibling sib_ccw = siblings[sib].ccw;
	enum patch_sibling sib_cw = siblings[sib].cw;
	const struct neighbours *n = &neighbours[sib];

	assert(parent != NULL);
	assert(parentid(p, 1) == parent->id);
	assert(parent->kids[sib] == p);
	assert(p->level == parent->level + 1);

	assert(check_neighbour_levels(parent));

	/* link to siblings; both pointers should be the same */
	assert(p->level == parent->kids[sib_ccw]->level);
	assert(p->level == parent->kids[sib_cw]->level);

	p->neigh[n->ccw + 0] = parent->kids[sib_ccw];
	p->neigh[n->ccw + 1] = parent->kids[sib_ccw];
	p->neigh[n->cw  + 0] = parent->kids[sib_cw];
	p->neigh[n->cw  + 1] = parent->kids[sib_cw];

	/* Link to neighbours. We're being created from a parent,
	   which means it was just a split, so the neighbours must
	   have a level <= p->level.  */
	assert(p->level >= parent->neigh[n->lr + siblings[sib].sy]->level);
	assert(p->level >= parent->neigh[n->ud + siblings[sib].sx]->level);

	p->neigh[n->lr + 0] = parent->neigh[n->lr + siblings[sib].sy];
	p->neigh[n->lr + 1] = parent->neigh[n->lr + siblings[sib].sy];
	p->neigh[n->ud + 0] = parent->neigh[n->ud + siblings[sib].sx];
	p->neigh[n->ud + 1] = parent->neigh[n->ud + siblings[sib].sx];

	assert(check_neighbour_levels(p));
}

static void patch_insert_active(struct quadtree *qt, struct patch *p)
{
	assert((p->flags & PF_ACTIVE) == 0);

	p->flags |= PF_ACTIVE;

	qt->nactive++;
	assert(qt->nactive <= qt->npatches);

	if (p->flags & PF_CULLED)
		list_add_tail(&p->list, &qt->culled);
	else {
		if (list_empty(&qt->visible))
			list_add(&p->list, &qt->visible);
		else {
			struct list_head *pp;

			list_for_each(pp, &qt->visible) {
				struct patch *ap = list_entry(pp, struct patch, list);

				if (p->priority >= ap->priority) {
					/* insert p before ap */
					list_add_tail(&p->list, &ap->list);
					return;
				}
			}
			/* p's prio less than everything else on the list */
			list_add_tail(&p->list, &qt->visible);
		}
	}
}

static void patch_remove_active(struct quadtree *qt, struct patch *p)
{
	assert(p->flags & PF_ACTIVE);

	p->flags &= ~PF_ACTIVE;
	list_del(&p->list);

	assert(qt->nactive > 0);
	qt->nactive--;
}

static struct patch *find_lowest(struct quadtree *qt)
{
	struct list_head *pp;
	struct patch *ret = NULL;

	list_for_each_prev(pp, &qt->culled) {
		struct patch *p = list_entry(pp, struct patch, list);
			
		if (p->level > 0 &&
		    p->pinned == 0 &&
		    (p->flags & PF_VISITED) == 0) {
			ret = p;
			break;
		}
	}

	if (ret == NULL) {
		list_for_each_prev(pp, &qt->visible) {
			struct patch *p = list_entry(pp, struct patch, list);
			
			if (p->level > 0 &&
			    p->pinned == 0 &&
			    (p->flags & PF_VISITED) == 0) {
				ret = p;
				break;
			}
		}
	}

	if (ret) {
		assert(ret->pinned == 0);
		assert((ret->flags & (PF_ACTIVE|PF_VISITED)) == PF_ACTIVE);

		printf("find_lowest returning %s (prio %d %s), flags=%x\n",
		       id2str(ret), ret->priority, ret->flags & PF_CULLED ? "culled" : "",
		       ret->flags);
	}

	return ret;
}

static void patch_init(struct patch *p, int level, unsigned id)
{
	assert((p->flags & PF_ACTIVE) == 0);

	if ((p->flags & PF_UNUSED) == 0) {
		/* patch still linked in; break links */
		printf("recycling %p\n", p);

		/* unlink from parent */
		if (p->parent)
			for(int i = 0; i < 4; i++)
				if (p->parent->kids[i] == p)
					p->parent->kids[i] = NULL;
		/* unlink from kids */
		for(int i = 0; i < 4; i++) {
			if (p->kids[i] && p->kids[i]->parent == p)
				p->kids[i]->parent = NULL;
		}

		/* unlink from neighbours */
		for(int i = 0; i < 8; i++) {
			struct patch *n = p->neigh[i];

			if (n == NULL)
				continue;

			for(int j = 0; j < 8; j++)
				if (n->neigh[j] == p)
					n->neigh[j] = NULL;
		}
	}

	for(int i = 0; i < 4; i++)
		p->col[i] = rand();

	for(int i = 0; i < 4; i++)
		p->kids[i] = NULL;

	for(int i = 0; i < 8; i++)
		p->neigh[i] = NULL;

	p->flags = 0;
	p->parent = NULL;
	p->level = level;
	p->id = id;

	p->priority = 0;
}

/* Allocate a patch from the freelist.  Caller should call
   patch_init() on it.  Also checks to see how much space is left on
   the freelist, and does some merging to free up patches if it gets
   too small. */
static struct patch *patch_alloc(struct quadtree *qt)
{
	static const unsigned MINLIST = 10;

	/* If the freelist is getting small, reclaim some stuff by
	   merging the most mergable patches. */
	if (!qt->reclaim && qt->nfree < MINLIST) {
		qt->reclaim = 1;

		while (qt->nfree < MINLIST) {
			struct patch *lowest = find_lowest(qt);
			if (lowest == NULL)
				break;

			printf("freelist refill merge %s, freelist %d\n",
			       id2str(lowest), qt->nfree);
			patch_merge(qt, lowest);
		}
		qt->reclaim = 0;
	}

	if (list_empty(&qt->freelist)) {
		printf("patch allocation failed!\n");
		assert(qt->nfree == 0);
		return NULL;
	}

	struct list_head *pp = qt->freelist.next;
	list_del(pp);

	assert(qt->nfree > 0);
	qt->nfree--;

	struct patch *p = list_entry(pp, struct patch, list);

	//printf("allocated %p\n", p);

	return p;
}

/* Add a patch to the freelist. */
static void patch_free(struct quadtree *qt, struct patch *p)
{
	if ((p->flags & PF_UNUSED) == 0)
		printf("freeing %p %s\n", p, id2str(p));

	assert((p->flags & PF_ACTIVE) == 0);
	assert(p->pinned == 0);

	list_add_tail(&p->list, &qt->freelist);
	qt->nfree++;
	assert(qt->nfree <= qt->npatches);
}

/* Remove a patch from the freelist, because it has been recovered. */
static void patch_remove_freelist(struct quadtree *qt, struct patch *p)
{
	assert(on_freelist(qt, p));

	list_del(&p->list);
	assert(qt->nfree > 0);
	qt->nfree--;
}


/* Merge a specific patch. 

   Given patch p, find the 4 sibling patches
   it was created with.  If one or more of those have been further
   subdivided, then merge them before going on (this cannot recur more
   than once because of the constraint of neighbours not differing by
   more than one level).

   Then remove the siblings from their active list(s), and add their
   parent into the active list.

   For example, the patch to be merged is sibling 1 in the target
   parent patch. Sibling 2's right neighbour has been subdivied, and
   so needs to be merged to maintain the "at most 1 level difference"
   constraint.  Similarly, sibling 3 is itself split, so needs to be
   merged before we can merge the rest.

      |       |       |
   ---+-------+---+---+--
      |-3-| 2 |-+-+   |
      |---+---|---+---+
      | 0 |p1 |   |   |
   ---+-------+---+---+--
      |       |       |
*/
static int patch_merge(struct quadtree *qt, struct patch *p)
{
	struct patch *parent;	/* the new patch we're creating */
	struct patch *sib[4] = {};	/* the group of siblings including p */
	unsigned long start_id;

	if (p == NULL)
		return 0;

	printf("merging %s\n", id2str(p));

	p->flags |= PF_VISITED;

	if (p->level == 0) {
		printf("merge %s failed: level 0\n", id2str(p));
		return 0;
	}

	/* Visit p and all its siblings, make sure they're all at the
	   same level as p.  Add them all to sib[]. */
	start_id = siblingid(p);
	do {
		unsigned long sibid = siblingid(p);
		const struct neighbours *pn = &neighbours[sibid];
		struct patch *sibling = p->neigh[pn->ccw];

		assert(!on_freelist(qt, p));
		assert(p->flags & PF_ACTIVE);

		if (p->pinned) {
			printf("merge %s failed: pinned\n", id2str(p));
			goto out_fail;
		}

		sib[sibid] = p;
		p->pinned++;

		assert(parentid(p, 1) == 
		       parentid(sibling, sibling->level - p->level + 1));

		if (p->level != sibling->level) {
			/* Levels don't match, need to merge the
			   sibling before going on. */
			assert(p->level == sibling->level - 1);
			assert(p->neigh[pn->ccw + 1] != p->neigh[pn->ccw]);
			assert(p->neigh[pn->ccw + 1]->level ==
			       sibling->level);

			if (!patch_merge(qt, sibling))
				goto out_fail;
			sibling = p->neigh[pn->ccw];
		}

		assert(p->level == sibling->level);
		assert(p->neigh[pn->ccw + 1] == p->neigh[pn->ccw]);

		/* check non-sibling neighbours */
		if (p->neigh[pn->ud]->level > p->level) {
			assert(p->neigh[pn->ud]->level == p->level+1);
			if (!patch_merge(qt, p->neigh[pn->ud]))
				goto out_fail;
		}
		assert(p->neigh[pn->ud]->level <= p->level);
		assert(p->neigh[pn->ud] == p->neigh[pn->ud+1]);

		if (p->neigh[pn->lr]->level > p->level) {
			assert(p->neigh[pn->lr]->level == p->level+1);
			if (!patch_merge(qt, p->neigh[pn->lr]))
				goto out_fail;
		}
		assert(p->neigh[pn->lr]->level <= p->level);
		assert(p->neigh[pn->lr] == p->neigh[pn->lr+1]);


		p = sibling;
	} while(siblingid(p) != start_id);

	if (p->parent != NULL) {
		parent = p->parent;		/* cached parent */

		patch_remove_freelist(qt, parent);
	} else {
		unsigned long id = parentid(p, 1);
		int level = p->level - 1;

		/* allocate a new parent */
		parent = patch_alloc(qt);

		if (parent == NULL)
			goto out_fail;

		assert(parent != NULL);
		assert(!on_freelist(qt, parent));

		patch_init(parent, level, id);
		
		parent->x0 = sib[0]->x0;
		parent->x1 = sib[2]->x1;
		parent->y0 = sib[0]->y0;
		parent->y1 = sib[2]->y1;
		parent->z0 = sib[0]->z0;
		parent->z1 = sib[2]->z1;
	}
	parent->priority = 0;
	parent->flags |= PF_VISITED;
	parent->pinned++;

	for(int i = 0; i < 4; i++)
		assert(parent->kids[i] == NULL ||
		       parent->kids[i]->parent == parent);

	/* Patch in all the neighbour pointers.  The parent gets them
	   from its kids.  */
	for(int i = 0; i < 4; i++) {
		const struct neighbours *pn = &neighbours[i];
		int sx = siblings[i].sx;
		int sy = siblings[i].sy;

		assert(sib[i]->neigh[pn->lr] == sib[i]->neigh[pn->lr+1]);
		parent->neigh[pn->lr+sy] = sib[i]->neigh[pn->lr];

		assert(sib[i]->neigh[pn->ud] == sib[i]->neigh[pn->ud+1]);
		parent->neigh[pn->ud+sx] = sib[i]->neigh[pn->ud];

		parent->priority = clamp(parent->priority + sib[i]->priority, 0, 255);
	}

	/* now that the forward-links are set up, do the backlinks */
	for(int i = 0; i < 4; i++)
		backlink_neighbours(parent, sib[i]);

	for(int i = 0; i < 4; i++)
		patch_remove_active(qt, sib[i]);

	/* free all the siblings */
	for(int i = 0; i < 4; i++) {
		sib[i]->parent = parent;
		parent->kids[i] = sib[i];

		assert(sib[i]->pinned);
		sib[i]->pinned--;
		patch_free(qt, sib[i]);
	}

	/* make parent active */
	patch_insert_active(qt, parent);

	assert(parent->pinned);
	parent->pinned--;

	assert(check_neighbour_levels(parent));

	return 1;

  out_fail:
	for(int i = 0; i < 4; i++) {
		if (sib[i]) {
			assert(sib[i]->pinned);
			sib[i]->pinned--;
		}
	}
	return 0;
}

static inline int mid(int a, int b)
{
	return (a+b)/2;
}

/* 
   Patch splitting is the converse operation to patch merging; the
   single initial parent patch is replaced by 4 child patches.

   In order to maintain the max one level difference invariant, we
   first need to visit all the patches neighbours, and make sure their
   level is >= than p; if it is less, then that neighbour also needs
   to be split.
 */
static int patch_split(struct quadtree *qt, struct patch *parent)
{
	struct patch *k[4] = { NULL, NULL, NULL, NULL };

	if (parent == NULL)
		return 0;

	printf("splitting %s\n", id2str(parent));

	if (parent->pinned) {
		printf("%s pinned\n", id2str(parent));
		return 0;
	}

	/* don't split if we're getting too small */
	if ((parent->x0 != parent->x1 && abs(parent->x0 - parent->x1) / 2 < PATCH_SAMPLES) ||
	    (parent->y0 != parent->y1 && abs(parent->y0 - parent->y1) / 2 < PATCH_SAMPLES) ||
	    (parent->z0 != parent->z1 && abs(parent->z0 - parent->z1) / 2 < PATCH_SAMPLES))
		return 0;

	assert(check_neighbour_levels(parent));
	assert(parent->flags & PF_ACTIVE);
	assert(!on_freelist(qt, parent));

	parent->pinned++;

	assert(parent->flags & PF_ACTIVE);

	/* allocate and initialize the 4 new patches */
	for(int i = 0; i < 4; i++) {
		k[i] = parent->kids[i];

		assert(parent->flags & PF_ACTIVE);
		if (k[i] == NULL) {
			k[i] = patch_alloc(qt);
			assert(parent->flags & PF_ACTIVE);
			if (k[i] == NULL)
				goto out_fail;
			patch_init(k[i], parent->level + 1, childid(parent->id, i));

			k[i]->parent = parent;
			parent->kids[i] = k[i];
		} else {
			assert(k[i]->parent == parent);
			patch_remove_freelist(qt, k[i]); /* reclaimed */
		}

		assert(parent->flags & PF_ACTIVE);

		/* XXX ROUGH: each child is roughly 1/4 the screen size of the parent */
		k[i]->priority = parent->priority / 4;

		k[i]->flags |= PF_VISITED;
		k[i]->pinned++;

		assert(k[i]->parent == parent);
	}

	assert(parent->flags & PF_ACTIVE);

	/* Check all the parent patches neighbours to make sure
	   they're a suitable level, and split them if not */
	for(enum patch_neighbour dir = 0; dir < 8; dir++) {
		assert(parent->neigh[dir] != NULL);

		if (parent->neigh[dir]->level < parent->level) {
			assert(parent->neigh[dir]->level == (parent->level-1));

			if (!patch_split(qt, parent->neigh[dir]))
				goto out_fail; /* split failed */
		}
		assert(parent->neigh[dir]->level >= parent->level &&
		       parent->neigh[dir]->level <= parent->level+1);

		parent->neigh[dir]->pinned++;
	}

	for(int i = 0; i < 4; i++)
		link_neighbours_from_parent(k[i]);

	for(int i = 0; i < 4; i++)
		backlink_neighbours(k[i], parent);

	signed int mx = mid(parent->x0, parent->x1);
	signed int my = mid(parent->y0, parent->y1);
	signed int mz = mid(parent->z0, parent->z1);

	/* subdivide the coords */
	if (parent->x0 == parent->x1) { /* left, right */
		k[0]->x0 = parent->x0;    k[0]->x1 = parent->x0;
		k[0]->y0 = parent->y0;    k[0]->y1 = my;
		k[0]->z0 = parent->z0;    k[0]->z1 = mz;

		k[1]->x0 = parent->x0;    k[1]->x1 = parent->x0;
		k[1]->y0 = my;            k[1]->y1 = parent->y1;
		k[1]->z0 = parent->z0;    k[1]->z1 = mz;

		k[2]->x0 = parent->x0;    k[2]->x1 = parent->x0;
		k[2]->y0 = my;            k[2]->y1 = parent->y1;
		k[2]->z0 = mz;            k[2]->z1 = parent->z1;

		k[3]->x0 = parent->x0;    k[3]->x1 = parent->x0;;
		k[3]->y0 = parent->y0;    k[3]->y1 = my;
		k[3]->z0 = mz;            k[3]->z1 = parent->z1;
	} else if (parent->y0 == parent->y1) { /* front, back */
		k[0]->x0 = parent->x0;    k[0]->x1 = mx;
		k[0]->y0 = parent->y0;    k[0]->y1 = parent->y0;
		k[0]->z0 = parent->z0;    k[0]->z1 = mz;

		k[1]->x0 = mx;            k[1]->x1 = parent->x1;
		k[1]->y0 = parent->y0;    k[1]->y1 = parent->y0;
		k[1]->z0 = parent->z0;    k[1]->z1 = mz;

		k[2]->x0 = mx;            k[2]->x1 = parent->x1;
		k[2]->y0 = parent->y0;    k[2]->y1 = parent->y0;
		k[2]->z0 = mz;            k[2]->z1 = parent->z1;

		k[3]->x0 = parent->x0;    k[3]->x1 = mx;
		k[3]->y0 = parent->y0;    k[3]->y1 = parent->y0;
		k[3]->z0 = mz;            k[3]->z1 = parent->z1;
	} else if (parent->z0 == parent->z1) { /* top, bottom */
		k[0]->x0 = parent->x0;    k[0]->x1 = mx;
		k[0]->y0 = parent->y0;    k[0]->y1 = my;
		k[0]->z0 = parent->z0;    k[0]->z1 = parent->z0;

		k[1]->x0 = mx;            k[1]->x1 = parent->x1;
		k[1]->y0 = parent->y0;    k[1]->y1 = my;
		k[1]->z0 = parent->z0;    k[1]->z1 = parent->z0;

		k[2]->x0 = mx;            k[2]->x1 = parent->x1;
		k[2]->y0 = my;            k[2]->y1 = parent->y1;
		k[2]->z0 = parent->z0;    k[2]->z1 = parent->z0;

		k[3]->x0 = parent->x0;    k[3]->x1 = mx;
		k[3]->y0 = my;            k[3]->y1 = parent->y1;
		k[3]->z0 = parent->z0;    k[3]->z1 = parent->z0;
	} else
		abort();

	for(enum patch_neighbour dir = 0; dir < 8; dir++) {
		assert(parent->neigh[dir]->pinned);
		parent->neigh[dir]->pinned--;
	}

	patch_remove_active(qt, parent);

	for(int i = 0; i < 4; i++) {
		parent->kids[i] = k[i];
		patch_insert_active(qt, k[i]);

		assert(k[i]->pinned);
		k[i]->pinned--;
	}

	assert(parent->pinned);
	parent->pinned--;
	patch_free(qt, parent);

	return 1;

  out_fail:
	for(int i = 0; i < 4; i++)
		if (k[i]) {
			patch_init(k[i], -1, 0);
			patch_free(qt, k[i]);
		}
	return 0;
}

struct quadtree *quadtree_create(int num_patches, long radius, 
				 int (*generator)(long x, long y, long z))
{
	struct quadtree *qt = NULL;

	if (num_patches < 6)
		goto out;

	qt = malloc(sizeof(*qt));

	if (qt == NULL)
		goto out;

	qt->radius = radius;

	qt->patches = malloc(sizeof(struct patch) * num_patches);
	if (qt->patches == NULL)
		goto out;
	qt->npatches = num_patches;

	INIT_LIST_HEAD(&qt->visible);
	INIT_LIST_HEAD(&qt->culled);
	INIT_LIST_HEAD(&qt->freelist);
	qt->nfree = 0;
	qt->reclaim = 0;
	qt->nactive = 0;

	/* add patches to freelist */
	for(int i = 0; i < num_patches; i++) {
		struct patch *p = &qt->patches[i];

		p->flags = PF_UNUSED; /* never used */
		p->vertex_offset = i * VERTICES_PER_PATCH;
		p->pinned = 0;
		INIT_LIST_HEAD(&p->list);

		patch_free(qt, p);
	}

	glGenBuffersARB(1, &qt->vtxbufid);
	glBindBufferARB(GL_ARRAY_BUFFER_BINDING, qt->vtxbufid);
	glBufferDataARB(GL_ARRAY_BUFFER_BINDING,
			sizeof(struct vertex) * VERTICES_PER_PATCH * num_patches,
			NULL, GL_DYNAMIC_DRAW_ARB);
	glBindBufferARB(GL_ARRAY_BUFFER_BINDING, 0);

	/* 

	   Coord:
	     +Z
	      | +Y
	      | / 0
	      |/___+X

	-y        +---^---++z
	          |       |
	          |   2   |
	          |      .|
	+y        +---^---++z +++
		  |       |
		  |   1   |
		  |      .|-z
	+y+-------+---^---+-------+ +z
	  |      .|       |       |
	  <   4   |   0   |   5   >
	  |       |      .|.      |
	-y+-------+-------+-------+ +z
	      --- |.      |-z
	          |   3   |
	          |       |
	-y        +---v---+         +z

          -x     -x      +x       +x


	 */
	static const struct cube {
		int x0, x1, y0, y1, z0, z1;
		int neigh[4]; /* right, up, left, down */
	} cube[6] = {
		{  1, -1,  -1,  1,  -1, -1,  { 4, 1, 5, 3 } }, /* 0 */
		{  1, -1,   1,  1,  -1,  1,  { 4, 2, 5, 0 } }, /* 1 */
		{  1, -1,   1, -1,   1,  1,  { 4, 3, 5, 1 } }, /* 2 */
		{ -1,  1,  -1, -1,  -1,  1,  { 5, 2, 4, 0 } }, /* 3 */
		{ -1, -1,   1, -1,  -1,  1,  { 3, 2, 1, 0 } }, /* 4 */
		{  1,  1,  -1,  1,  -1,  1,  { 1, 2, 3, 0 } }, /* 5 */
	};

	/* create basis patches in cube form */
	struct patch *basis[6];
	for(int i = 0; i < 6; i++) {
		struct patch *p = patch_alloc(qt);

		if (p == NULL)
			goto out;

		basis[i] = p;
		patch_init(p, 0, i);
	}

	for(int i = 0; i < 6; i++) {
		struct patch *b = basis[i];
		const struct cube *c = &cube[i];

		b->x0 = c->x0 * radius;
		b->x1 = c->x1 * radius;

		b->y0 = c->y0 * radius;
		b->y1 = c->y1 * radius;

		b->z0 = c->z0 * radius;
		b->z1 = c->z1 * radius;

		for(int j = 0; j < 4; j++) {
			b->neigh[j * 2 + 0] = basis[c->neigh[j]];
			b->neigh[j * 2 + 1] = basis[c->neigh[j]];
		}

		patch_insert_active(qt, b);
	}

	return qt;

  out: 
	free(qt);
	return NULL;
}

static void project_to_sphere(int radius, long x, long y, long z, long *ox, long *oy, long *oz)
{
	float xf = (float)x;
	float yf = (float)y;
	float zf = (float)z;

	float len = radius / sqrtf(xf*xf + yf*yf + zf*zf);

	*ox = xf * len;
	*oy = yf * len;
	*oz = zf * len;
}

static void update_prio(struct patch *p,
			long radius,
			const double dmv[16],
			const double dproj[16],
			const int viewport[4])
{
	struct {
		long x,y,z;
	} sph[4];
	struct {
		GLdouble x, y, z;
	} proj[4];

	if (p->z0 == p->z1 ||
	    p->y0 == p->y1) {
		project_to_sphere(radius, p->x0, p->y0, p->z0, &sph[0].x, &sph[0].y, &sph[0].z);
		project_to_sphere(radius, p->x1, p->y0, p->z0, &sph[1].x, &sph[1].y, &sph[1].z);
		project_to_sphere(radius, p->x1, p->y1, p->z1, &sph[2].x, &sph[2].y, &sph[2].z);
		project_to_sphere(radius, p->x0, p->y1, p->z1, &sph[3].x, &sph[3].y, &sph[3].z);
	} else {
		project_to_sphere(radius, p->x0, p->y0, p->z0, &sph[0].x, &sph[0].y, &sph[0].z);
		project_to_sphere(radius, p->x1, p->y1, p->z0, &sph[1].x, &sph[1].y, &sph[1].z);
		project_to_sphere(radius, p->x0, p->y1, p->z1, &sph[2].x, &sph[2].y, &sph[2].z);
		project_to_sphere(radius, p->x0, p->y0, p->z1, &sph[3].x, &sph[3].y, &sph[3].z);
	}

	for(int i = 0; i < 4; i++)
		gluProject(sph[i].x, sph[i].y, sph[i].z, dmv, dproj, viewport, &proj[i].x, &proj[i].y, &proj[i].z);

	float area = 0.f;
	for(unsigned i = 0; i < 4; i++) {
		unsigned n = (i+1) % 4;
		area += (proj[i].x * proj[n].y) - (proj[n].x * proj[i].y);
	}

	area *= 0.5f;
	area /= viewport[2] * viewport[3];

	if (area < 0) {
		p->priority = 0;
		p->flags |= PF_CULLED;
	} else
		p->priority = clamp(255 * area, 0, 255);

#if 0
	glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT);
	glColor3f(1,1,1);
	glDisable(GL_BLEND);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(viewport[0], viewport[0]+viewport[2], viewport[1], viewport[1]+viewport[3], 0, 1);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glBegin(GL_LINE_LOOP);
	for(int i = 0; i < 4; i++)
		glVertex2d(proj[i].x, proj[i].y);
	glEnd();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glPopAttrib();
#endif
}

void quadtree_update_view(struct quadtree *qt, 
			  const float modelview[16],
			  const float projection[16],
			  const int viewport[4])
{
	const double dmv[16] = { 
		modelview[ 0], modelview[ 1], modelview[ 2], modelview[ 3],
		modelview[ 4], modelview[ 5], modelview[ 6], modelview[ 7],
		modelview[ 8], modelview[ 9], modelview[10], modelview[11],
		modelview[12], modelview[13], modelview[14], modelview[15],
	};
	const double dproj[16] = { 
		projection[ 0], projection[ 1], projection[ 2], projection[ 3],
		projection[ 4], projection[ 5], projection[ 6], projection[ 7],
		projection[ 8], projection[ 9], projection[10], projection[11],
		projection[12], projection[13], projection[14], projection[15],
	};

	/* remove all active patches into a local list */
	struct list_head local, *pp, *pnext;
	INIT_LIST_HEAD(&local);
	list_splice_init(&qt->visible, &local);
	list_splice_init(&qt->culled, &local);
	qt->nactive = 0;

	/* Project each patch using the view matrix, and compute the
	   projected area. Use this to generate the priority for each
	   patch, and update the active list.  */
	list_for_each_safe(pp, pnext, &local) {
		struct patch *p = list_entry(pp, struct patch, list);

		list_del(pp);	/* remove from local list */

		p->flags &= ~(PF_VISITED | PF_CULLED | PF_ACTIVE);

		update_prio(p, qt->radius, dmv, dproj, viewport);

		patch_insert_active(qt, p);
	}
	assert(list_empty(&local));

#if 0
	for(int limit = 0; limit < 10 && (qt->nfree < qt->npatches/10); limit++) {
		struct patch *lowest = find_lowest(qt);
		if (lowest == NULL || lowest->priority > 255 * 2 / 100)
			break;

		printf("incremental free merge %s, freelist %d\n",
		       id2str(lowest), qt->nfree);
		patch_merge(qt, lowest);
	}
	//printf("freelist=%d\n", qt->nfree);
#endif

	/* try to maintain a policy of having a patch no larger than
	   N% of the screen, and no smaller than M% */
	static const int MAXSIZE = 5;
	static const int MINSIZE = 1;

  restart_list:
	list_for_each(pp, &qt->visible) {
		struct patch *p = list_entry(pp, struct patch, list);

		assert((p->flags & (PF_CULLED|PF_ACTIVE)) == PF_ACTIVE);
		assert(p->pinned == 0);
			
		if (p->flags & PF_VISITED)
			continue;

		if (p->priority >= (255 * MAXSIZE / 100)) {
			p->flags |= PF_VISITED;
			
			printf(">>>split %p %s %d%%\n", p, id2str(p),
				p->priority * 100 / 255);
			
			patch_split(qt, p);
			goto restart_list;
		}
#if 0
		if (p->priority < (255 * MINSIZE / 100)) {
			p->flags |= PF_VISITED;
			printf(">>>merge %p %s %d%%\n",p, id2str(p),
				p->priority * 100 / 255);
			
			patch_merge(qt, p);
			goto restart_list;
		}
#endif
	}
}
