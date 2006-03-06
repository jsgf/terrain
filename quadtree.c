#include <stddef.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "geom.h"

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>

#include "quadtree.h"
#include "quadtree_priv.h"

#define DEBUG		1
#define ANNOTATE	0

static int have_vbo = -1;
static int have_cva = -1;

static GLuint index_bufid = 0;
static const patch_index_t (*patchidx)[9][INDICES_PER_PATCH] = &patch_indices;

#define GLERROR()							\
do {									\
	GLenum err = glGetError();					\
	if (err != GL_NO_ERROR) {					\
		printf("GL error at %s:%d: %s\n",			\
		       __FILE__, __LINE__, gluErrorString(err));	\
	}								\
} while(0)

struct vertex {
	//signed char s,t;		// PSP only
	GLshort s,t;
	GLubyte col[4];
	GLbyte nx, ny, nz;	/* needed? */
	GLfloat x,y,z;		/* short? */
};

static int patch_merge(struct quadtree *qt, struct patch *p,
		       int (*maymerge)(const struct patch *));

static inline int clamp(int x, int lower, int upper)
{
	if (x > upper)
		x = upper;
	if (x < lower)
		x = lower;
	return x;
}

static void project_to_sphere(int radius,
			      long x, long y, long z,
			      vec3_t *out)
{
	*out = VEC3(x, y, z);

	vec3_normalize(out);
	vec3_scale(out, radius);
}


/* The ID is an integer which uniquely identifies all nodes in the
   quadtree.  It uses 2 bits per level. */

char *patch_name(const struct patch *p, char buf[40])
{
	int id = p->id;
	int level = p->level;
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

int patch_level(const struct patch *p)
{
	return p->level;
}

unsigned long patch_id(const struct patch *p)
{
	return p->id;
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
	char buf[40];

	fprintf(f, "\t\"%p\" [label=\"%s\", shape=%s];\n",
		p, patch_name(p, buf),
		(p->flags & PF_CULLED) ? "box" : "diamond");
		
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

		n->flags |= PF_STITCH_GEOM;

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
	struct list_head *list;

	assert((p->flags & PF_ACTIVE) == 0);

	p->flags |= PF_ACTIVE;

	qt->nactive++;
	assert(qt->nactive <= qt->npatches);

	if (p->flags & PF_CULLED)
		list = &qt->culled;
	else {
		qt->nvisible++;
		list = &qt->visible;
	}

	if (list_empty(list))
		list_add(&p->list, list);
	else {
		struct list_head *pp;

		list_for_each(pp, list) {
			struct patch *ap = list_entry(pp, struct patch, list);

			if (p->priority >= ap->priority) {
				/* insert p before ap */
				list_add_tail(&p->list, &ap->list);
				return;
			}
		}
		/* p's prio less than everything else on the list */
		list_add_tail(&p->list, list);
	}
}

static void patch_remove_active(struct quadtree *qt, struct patch *p)
{
	assert(p->flags & PF_ACTIVE);

	p->flags &= ~PF_ACTIVE;
	list_del(&p->list);

	assert(qt->nactive > 0);
	qt->nactive--;
	if ((p->flags & PF_CULLED) == 0) {
		assert(qt->nvisible > 0);
		qt->nvisible--;
	}
}

static struct patch *find_lowest(struct quadtree *qt)
{
	struct list_head *pp;
	struct patch *ret = NULL;

	list_for_each(pp, &qt->culled) {
		struct patch *p = list_entry(pp, struct patch, list);
			
		if (p->level > 0 &&
		    p->pinned == 0 &&
		    p->phase != qt->phase) {
			ret = p;
			break;
		}
	}

	if (ret == NULL) {
		list_for_each_prev(pp, &qt->visible) {
			struct patch *p = list_entry(pp, struct patch, list);
			
			if (p->level > 0 &&
			    p->pinned == 0 &&
			    p->phase != qt->phase) {
				ret = p;
				break;
			}
		}
	}

	if (ret) {
		char buf[40];

		assert(ret->pinned == 0);
		assert(ret->phase != qt->phase);
		assert(ret->flags & PF_ACTIVE);

		if (DEBUG)
			printf("find_lowest returning %s (prio %g %s), flags=%x\n",
			       patch_name(ret, buf), ret->priority, ret->flags & PF_CULLED ? "culled" : "",
			       ret->flags);
	}

	return ret;
}

static void patch_init(struct patch *p, int level, unsigned id)
{
	assert((p->flags & PF_ACTIVE) == 0);

	if ((p->flags & PF_UNUSED) == 0) {
		/* patch still linked in; break links */
		if (DEBUG)
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

	p->flags = PF_UPDATE_GEOM | PF_STITCH_GEOM;
	p->parent = NULL;
	p->level = level;
	p->id = id;
	p->phase = 0;

	p->priority = 0.f;
}

static int mergeculledonly(const struct patch *p)
{
	return (p->flags & PF_CULLED) != 0;
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

		while (qt->nfree < MINLIST*2) {
			char buf[40];
			struct patch *lowest = find_lowest(qt);
			if (lowest == NULL)
				break;

			if (DEBUG)
				printf("freelist refill merge %s, freelist %d\n",
				       patch_name(lowest, buf), qt->nfree);
			if (!patch_merge(qt, lowest,
					 (lowest->flags & PF_CULLED) ? mergeculledonly : NULL))
				lowest->phase = qt->phase;
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
	if (DEBUG && (p->flags & PF_UNUSED) == 0) {
		char buf[40];
		printf("freeing %p %s freelist=%d\n",
		       p, patch_name(p, buf), qt->nfree+1);
	}

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

static void compute_bbox(const struct quadtree *qt, struct patch *p)
{
	vec3_t sph[8];
	long radius = qt->radius;

	if (p->z0 == p->z1) {
		long size = abs(p->x0 - p->x1) / 2;
		assert(size == abs(p->y0 - p->y1) / 2);

		project_to_sphere(radius, p->x0, p->y0, p->z0 - size, &sph[0]);
		project_to_sphere(radius, p->x1, p->y0, p->z0 - size, &sph[1]);
		project_to_sphere(radius, p->x1, p->y1, p->z0 - size, &sph[2]);
		project_to_sphere(radius, p->x0, p->y1, p->z0 - size, &sph[3]);

		project_to_sphere(radius, p->x0, p->y0, p->z0 + size, &sph[4]);
		project_to_sphere(radius, p->x1, p->y0, p->z0 + size, &sph[5]);
		project_to_sphere(radius, p->x1, p->y1, p->z0 + size, &sph[6]);
		project_to_sphere(radius, p->x0, p->y1, p->z0 + size, &sph[7]);
	} else if (p->y0 == p->y1) {
		long size = abs(p->x0 - p->x1) / 2;
		assert(size == abs(p->z0 - p->z1) / 2);

		project_to_sphere(radius, p->x0, p->y0 - size, p->z0, &sph[0]);
		project_to_sphere(radius, p->x1, p->y0 - size, p->z0, &sph[1]);
		project_to_sphere(radius, p->x1, p->y0 - size, p->z1, &sph[2]);
		project_to_sphere(radius, p->x0, p->y0 - size, p->z1, &sph[3]);

		project_to_sphere(radius, p->x0, p->y0 + size, p->z0, &sph[4]);
		project_to_sphere(radius, p->x1, p->y0 + size, p->z0, &sph[5]);
		project_to_sphere(radius, p->x1, p->y0 + size, p->z1, &sph[6]);
		project_to_sphere(radius, p->x0, p->y0 + size, p->z1, &sph[7]);
	} else if (p->x0 == p->x1) {
		long size = abs(p->y0 - p->y1) / 2;
		assert(size == abs(p->z0 - p->z1) / 2);

		project_to_sphere(radius, p->x0 - size, p->y0, p->z0, &sph[0]);
		project_to_sphere(radius, p->x0 - size, p->y1, p->z0, &sph[1]);
		project_to_sphere(radius, p->x0 - size, p->y1, p->z1, &sph[2]);
		project_to_sphere(radius, p->x0 - size, p->y0, p->z1, &sph[3]);

		project_to_sphere(radius, p->x0 + size, p->y0, p->z0, &sph[4]);
		project_to_sphere(radius, p->x0 + size, p->y1, p->z0, &sph[5]);
		project_to_sphere(radius, p->x0 + size, p->y1, p->z1, &sph[6]);
		project_to_sphere(radius, p->x0 + size, p->y0, p->z1, &sph[7]);
	} else
		abort();

	p->bbox.centre = VEC3(0,0,0);
	for(int i = 0; i < 8; i++)
		vec3_add(&p->bbox.centre, &p->bbox.centre, &sph[i]);
	vec3_scale(&p->bbox.centre, 1./8);

	p->bbox.extent = VEC3(0,0,0);
	for(int i = 0; i < 8; i++) {
		vec3_t d;
		vec3_sub(&d, &p->bbox.centre, &sph[i]);
		vec3_abs(&d);
		vec3_max(&p->bbox.extent, &p->bbox.extent, &d);
	}
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
static int patch_merge(struct quadtree *qt, struct patch *p,
		       int (*maymerge)(const struct patch *p))
{
	struct patch *parent;	/* the new patch we're creating */
	struct patch *sib[4] = {};	/* the group of siblings including p */
	unsigned long start_id;
	char buf[40];
	unsigned culled = PF_CULLED;

	if (p == NULL)
		return 0;

	if (DEBUG)
		printf("merging %s\n", patch_name(p, buf));

	if (maymerge && !(*maymerge)(p)) {
		printf("merge %s failed: maymerge failed\n", patch_name(p, buf));
		p->phase = qt->phase;
		return 0;
	}

	if (p->level == 0) {
		printf("merge %s failed: level 0\n", patch_name(p, buf));
		p->phase = qt->phase;
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
			printf("merge %s failed: pinned\n",
			       patch_name(p, buf));
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

			if (!patch_merge(qt, sibling, maymerge))
				goto out_fail;
			sibling = p->neigh[pn->ccw];
		}

		culled &= p->flags;

		assert(p->level == sibling->level);
		assert(p->neigh[pn->ccw + 1] == p->neigh[pn->ccw]);

		/* check non-sibling neighbours */
		if (p->neigh[pn->ud]->level > p->level) {
			assert(p->neigh[pn->ud]->level == p->level+1);
			if (!patch_merge(qt, p->neigh[pn->ud], maymerge))
				goto out_fail;
		}
		assert(p->neigh[pn->ud]->level <= p->level);
		assert(p->neigh[pn->ud] == p->neigh[pn->ud+1]);

		if (p->neigh[pn->lr]->level > p->level) {
			assert(p->neigh[pn->lr]->level == p->level+1);
			if (!patch_merge(qt, p->neigh[pn->lr], maymerge))
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

		compute_bbox(qt, parent);
	}
	parent->priority = 0.f;
	parent->flags |= culled;
	parent->pinned++;
	parent->phase = p->phase;

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

		parent->priority += sib[i]->priority;
	}

	/* when culled, the parent prio is the average of the kids */
	if (parent->flags & PF_CULLED)
		parent->priority *= .25f;

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
	char buf[40];

	if (parent == NULL)
		return 0;

	if (DEBUG)
		printf("splitting %s\n", patch_name(parent, buf));

	if (parent->pinned) {
		printf("%s pinned\n", patch_name(parent, buf));
		return 0;
	}

	/* don't split if we're getting too small */
	if ((parent->x0 != parent->x1 && abs(parent->x0 - parent->x1) / 2 < PATCH_SAMPLES) ||
	    (parent->y0 != parent->y1 && abs(parent->y0 - parent->y1) / 2 < PATCH_SAMPLES) ||
	    (parent->z0 != parent->z1 && abs(parent->z0 - parent->z1) / 2 < PATCH_SAMPLES)) {
		parent->phase = qt->phase;
		return 0;
	}

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
			if (k[i] == NULL)
				goto out_fail;
			patch_init(k[i], parent->level + 1, childid(parent->id, i));

			k[i]->parent = parent;
			parent->kids[i] = k[i];
		} else {
			assert(k[i]->parent == parent);
			patch_remove_freelist(qt, k[i]); /* reclaimed */
		}

		if (k[i]->flags & PF_CULLED) {
			/* if culled, the kids have the same prio as
			   the parent */
			k[i]->priority = parent->priority;
		} else {
			/* XXX ROUGH: each child is roughly 1/4 the
			   screen size of the parent */
			k[i]->priority = parent->priority / 4;
		}

		k[i]->flags |= parent->flags & PF_CULLED;
		k[i]->pinned++;
		k[i]->phase = parent->phase;

		assert(k[i]->parent == parent);
	}

	assert(parent->flags & PF_ACTIVE);

	/* Check all the parent patches neighbours to make sure
	   they're a suitable level, and split them if not */
	for(enum patch_neighbour dir = 0; dir < 8; dir++) {
		assert(parent->neigh[dir] != NULL);

		if (parent->neigh[dir]->level < parent->level) {
			assert(parent->neigh[dir]->level == (parent->level-1));

			if (!patch_split(qt, parent->neigh[dir])) {
				/* unpin what we've done so far */
				for(enum patch_neighbour dd = 0; dd < dir; dd++) {
					assert(parent->neigh[dd]->pinned > 0);
					parent->neigh[dd]->pinned--;
				}
				goto out_fail; /* split failed */
			}
		}
		assert(parent->neigh[dir]->level >= parent->level &&
		       parent->neigh[dir]->level <= parent->level+1);

		parent->neigh[dir]->pinned++;
	}

	for(int i = 0; i < 4; i++)
		link_neighbours_from_parent(k[i]);

	for(int i = 0; i < 4; i++)
		backlink_neighbours(k[i], parent);

	signed int mx = (parent->x0 + parent->x1) / 2;
	signed int my = (parent->y0 + parent->y1) / 2;
	signed int mz = (parent->z0 + parent->z1) / 2;

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

	for(int i = 0; i < 4; i++)
		compute_bbox(qt, k[i]);

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
	assert(parent->pinned > 0);
	parent->pinned--;
	for(int i = 0; i < 4; i++)
		if (k[i]) {
			assert(k[i]->pinned > 0);
			k[i]->pinned--;
			patch_init(k[i], -1, 0);
			patch_free(qt, k[i]);
		}
	return 0;
}

struct quadtree *quadtree_create(int num_patches, long radius, 
				 elevation_t (*generator)(const vec3_t *v, GLubyte col[4]))
{
	struct quadtree *qt = NULL;

	if (num_patches < 6)
		goto out;

	qt = malloc(sizeof(*qt));

	if (qt == NULL)
		goto out;

	qt->landscape = generator;
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
	qt->nvisible = 0;

	qt->phase = 0;

	/* add patches to freelist */
	for(int i = 0; i < num_patches; i++) {
		struct patch *p = &qt->patches[i];

		p->flags = PF_UNUSED; /* has never been used */
		p->pinned = 0;
		p->vertex_offset = i * VERTICES_PER_PATCH;
		INIT_LIST_HEAD(&p->list);

		patch_free(qt, p);
	}

	if (DEBUG) {
		printf("vendor: %s\n", glGetString(GL_VENDOR));
		printf("renderer: %s\n", glGetString(GL_RENDERER));
		printf("version: %s\n", glGetString(GL_VERSION));
	}

	const GLubyte *extensions = glGetString(GL_EXTENSIONS);

	if (have_vbo == -1) {
		if (strncmp("1.5", (char *)glGetString(GL_VERSION), 3) == 0)
			have_vbo = 1;
		else
			have_vbo = gluCheckExtension((GLubyte *)"GL_ARB_vertex_buffer_object",
						     extensions);
	}

	if (have_cva == -1)
		have_cva = gluCheckExtension((GLubyte *)"GL_EXT_compiled_vertex_array",
					     extensions);

	if (DEBUG)
		printf("vbo: %d  cva:%d\n", have_vbo, have_cva);

	if (have_vbo) {
		glGenBuffers(1, &qt->vtxbufid);
		GLERROR();
		glBindBuffer(GL_ARRAY_BUFFER, qt->vtxbufid);
		glBufferData(GL_ARRAY_BUFFER,
			     sizeof(struct vertex) * VERTICES_PER_PATCH * num_patches,
			     NULL, GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		GLERROR();
		qt->varray = NULL;

		if (index_bufid == 0) {
			glGenBuffers(1, &index_bufid);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_bufid);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(patch_indices), patch_indices,
				     GL_STATIC_DRAW);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

			patchidx = NULL;
		}
	} else {
		qt->varray = malloc(sizeof(struct vertex) * VERTICES_PER_PATCH * num_patches);
		qt->vtxbufid = 0;
	}

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

		compute_bbox(qt, b);

		patch_insert_active(qt, b);
	}

	return qt;

  out: 
	free(qt);
	return NULL;
}

static void patch_bbox(const struct patch *p)
{
	const box_t *b = &p->bbox;

	glBegin(GL_LINES);

	glVertex3f(b->centre.x - b->extent.x,
		   b->centre.y - b->extent.y,
		   b->centre.z - b->extent.z);
	glVertex3f(b->centre.x - b->extent.x,
		   b->centre.y - b->extent.y,
		   b->centre.z + b->extent.z);

	glVertex3f(b->centre.x - b->extent.x,
		   b->centre.y - b->extent.y,
		   b->centre.z - b->extent.z);
	glVertex3f(b->centre.x - b->extent.x,
		   b->centre.y + b->extent.y,
		   b->centre.z - b->extent.z);

	glVertex3f(b->centre.x - b->extent.x,
		   b->centre.y - b->extent.y,
		   b->centre.z - b->extent.z);
	glVertex3f(b->centre.x + b->extent.x,
		   b->centre.y - b->extent.y,
		   b->centre.z - b->extent.z);


	glVertex3f(b->centre.x + b->extent.x,
		   b->centre.y + b->extent.y,
		   b->centre.z + b->extent.z);
	glVertex3f(b->centre.x + b->extent.x,
		   b->centre.y + b->extent.y,
		   b->centre.z - b->extent.z);

	glVertex3f(b->centre.x + b->extent.x,
		   b->centre.y + b->extent.y,
		   b->centre.z + b->extent.z);
	glVertex3f(b->centre.x + b->extent.x,
		   b->centre.y - b->extent.y,
		   b->centre.z + b->extent.z);

	glVertex3f(b->centre.x + b->extent.x,
		   b->centre.y + b->extent.y,
		   b->centre.z + b->extent.z);
	glVertex3f(b->centre.x - b->extent.x,
		   b->centre.y + b->extent.y,
		   b->centre.z + b->extent.z);

	glVertex3f(b->centre.x + b->extent.x,
		   b->centre.y - b->extent.y,
		   b->centre.z - b->extent.z);
	glVertex3f(b->centre.x + b->extent.x,
		   b->centre.y - b->extent.y,
		   b->centre.z + b->extent.z);

	glVertex3f(b->centre.x + b->extent.x,
		   b->centre.y - b->extent.y,
		   b->centre.z + b->extent.z);
	glVertex3f(b->centre.x - b->extent.x,
		   b->centre.y - b->extent.y,
		   b->centre.z + b->extent.z);

	glVertex3f(b->centre.x - b->extent.x,
		   b->centre.y - b->extent.y,
		   b->centre.z + b->extent.z);
	glVertex3f(b->centre.x - b->extent.x,
		   b->centre.y + b->extent.y,
		   b->centre.z + b->extent.z);

	glVertex3f(b->centre.x - b->extent.x,
		   b->centre.y + b->extent.y,
		   b->centre.z + b->extent.z);
	glVertex3f(b->centre.x - b->extent.x,
		   b->centre.y + b->extent.y,
		   b->centre.z - b->extent.z);

	glVertex3f(b->centre.x - b->extent.x,
		   b->centre.y + b->extent.y,
		   b->centre.z - b->extent.z);
	glVertex3f(b->centre.x + b->extent.x,
		   b->centre.y + b->extent.y,
		   b->centre.z - b->extent.z);

	glVertex3f(b->centre.x + b->extent.x,
		   b->centre.y + b->extent.y,
		   b->centre.z - b->extent.z);
	glVertex3f(b->centre.x + b->extent.x,
		   b->centre.y - b->extent.y,
		   b->centre.z - b->extent.z);

	glEnd();

}

static void patch_outline(const struct patch *p, long radius)
{
	vec3_t sph[4];

	if (p->z0 == p->z1 ||
	    p->y0 == p->y1) {
		project_to_sphere(radius, p->x0, p->y0, p->z0, &sph[0]);
		project_to_sphere(radius, p->x1, p->y0, p->z0, &sph[1]);
		project_to_sphere(radius, p->x1, p->y1, p->z1, &sph[2]);
		project_to_sphere(radius, p->x0, p->y1, p->z1, &sph[3]);
	} else {
		project_to_sphere(radius, p->x0, p->y0, p->z0, &sph[0]);
		project_to_sphere(radius, p->x0, p->y1, p->z0, &sph[1]);
		project_to_sphere(radius, p->x0, p->y1, p->z1, &sph[2]);
		project_to_sphere(radius, p->x0, p->y0, p->z1, &sph[3]);
	}

	glBegin(GL_LINE_LOOP);
	  glVertex3fv(sph[0].v);
	  glVertex3fv(sph[1].v);
	  glVertex3fv(sph[2].v);
	  glVertex3fv(sph[3].v);
	glEnd();
}

static void update_prio(struct patch *p,
			long radius,
			const matrix_t *mat,
			plane_t cullplanes[7],
			const vec3_t *camera)
{
	p->flags &= ~PF_CULLED;

	if (box_cull(&p->bbox, cullplanes, 7) == CULL_OUT) {
		vec3_t distv;

		vec3_sub(&distv, &p->bbox.centre, camera);

		p->flags |= PF_CULLED;

		/* higher prio = more reusable */
		p->priority = vec3_magnitude(&distv) / (2.f * radius);

		//printf("culled dist =%g ->prio=%d\n", dist, p->priority);
	} else {
		vec3_t sph[4];
		vec3_t proj[4];

		if (p->z0 == p->z1 ||
		    p->y0 == p->y1) {
			project_to_sphere(radius, p->x0, p->y0, p->z0, &sph[0]);
			project_to_sphere(radius, p->x1, p->y0, p->z0, &sph[1]);
			project_to_sphere(radius, p->x1, p->y1, p->z1, &sph[2]);
			project_to_sphere(radius, p->x0, p->y1, p->z1, &sph[3]);
		} else {
			project_to_sphere(radius, p->x0, p->y0, p->z0, &sph[0]);
			project_to_sphere(radius, p->x0, p->y1, p->z0, &sph[1]);
			project_to_sphere(radius, p->x0, p->y1, p->z1, &sph[2]);
			project_to_sphere(radius, p->x0, p->y0, p->z1, &sph[3]);
		}

		/* project into normalized 0-1 coord space, so area is
		   computed directly as a fraction of viewport area */
		for(int i = 0; i < 4; i++) {
			matrix_project(mat, &sph[i], &proj[i]);
			proj[i].x = proj[i].x * .5 + .5;
			proj[i].y = proj[i].y * .5 + .5;
		}

		if (ANNOTATE) {
			glPushAttrib(GL_ENABLE_BIT);
			glDisable(GL_LIGHTING);
			glDisable(GL_TEXTURE_2D);

			glColor3f(1,1,1);
			patch_outline(p, radius);

			glPopAttrib();
		}

		float area = 0.f;

		for(unsigned i = 0; i < 4; i++) {
			unsigned n = (i+1) % 4;
			area += (proj[i].x * proj[n].y) -
				(proj[n].x * proj[i].y);
		}

		area = fabsf(area * 0.5f);

		p->priority = area;

		if (DEBUG && 0) {
			char buf[40];
			printf("%s p->priority = %g%%, area=%g\n",
			       patch_name(p, buf),
			       p->priority * 100.f,
			       area);
		}
	}
}

static void generate_geom(struct quadtree *qt);

/* try to maintain a policy of having a patch no larger than
   N% of the screen, and no smaller than M% */
static const float MAXSIZE = 10.f / 100;
static const float MINSIZE = 1.f / 100;

static int mergesmall(const struct patch *p)
{
	return (p->priority < MINSIZE);
}

static void compute_cull_planes(const struct quadtree *qt, const matrix_t *mat,
				const vec3_t *camerapos, plane_t cullplanes[7])
{
	/* get the view frustum in object space */
	plane_extract(mat, cullplanes);

	{
		/* Compute the horizion cull plane.  The plane's
		   normal is the same direction as the camera's
		   position vector.  The distance depends on the
		   camera's altitude. */
		plane_t *h = &cullplanes[6];
		float alt = vec3_magnitude(camerapos);
		float radius = qt->radius;

		radius *= .99;

		h->normal = *camerapos;
		vec3_normalize(&h->normal);

		if (alt <= radius) {
			/* if the camera appears to be inside the
			   planet, just split the difference */
			h->dist = alt / 2;
		} else {
			/* Find inverse of camera point P with respect
			   to the sphere.  The plane is formed where
			   the tangents from tne camera meet the
			   sphere. */
			h->dist = (radius*radius) / alt;
		}
		h->dist = -h->dist;
	}

	for(int i = 0; i < 7; i++)
		plane_normalize(&cullplanes[i]);

}

void quadtree_update_view(struct quadtree *qt, const matrix_t *mat,
			  const vec3_t *camerapos)
{
	char buf[40];
	plane_t cullplanes[7];	/* 6 frustum and 1 horizon */

	/* remove all active patches into a local list */
	struct list_head local, *pp, *pnext;
	INIT_LIST_HEAD(&local);
	list_splice_init(&qt->visible, &local);
	list_splice_init(&qt->culled, &local);
	qt->nactive = 0;
	qt->nvisible = 0;

	compute_cull_planes(qt, mat, camerapos, cullplanes);

	if (ANNOTATE) {
		/* display cull planes */
		static const float col[] = {
			0,0,1,	/* blue - left */
			0,1,0,	/* green - right */
			1,0,0,	/* red - top */
			1,0,1,	/* magenta - bottom */
			1,1,0,	/* yellow - near */
			1,1,1,	/* white - far */
			0,1,1,	/* cyan - horizion */
		};

		glPushAttrib(GL_ENABLE_BIT);
		glDisable(GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);

		glBegin(GL_LINES);
		for(int i = 0; i < 7; i++) {
			vec3_t v = cullplanes[i].normal;

			/* scale and flip to be vector from origin
			   rather than normal vector */
			vec3_scale(&v, -cullplanes[i].dist);

			glColor3fv(&col[i * 3]);

			glVertex3fv(v.v);

			/* draw partial length so that all sides have
			   some chance of being seen */
			vec3_scale(&v, .75);
			glVertex3fv(v.v);
		}
		glEnd();
		glPopAttrib();
	}

	qt->phase++;

	/* For each patch, check if it is culled or not.  In either
	   case, compute a priority which decides how
	   splittable/mergable it is. */
	list_for_each_safe(pp, pnext, &local) {
		struct patch *p = list_entry(pp, struct patch, list);

		list_del(pp);	/* remove from local list */

		p->flags &= ~(PF_CULLED | PF_ACTIVE | PF_LATECULL);

		update_prio(p, qt->radius, mat, cullplanes, camerapos);

		patch_insert_active(qt, p);
	}
	assert(list_empty(&local));

	if (DEBUG)
		printf("%d active, %d visible, %d culled\n",
		       qt->nactive, qt->nvisible, qt->nactive - qt->nvisible);


	qt->phase++;
  restart_merge_list:
	list_for_each(pp, &qt->visible) {
		struct patch *p = list_entry(pp, struct patch, list);

		if (p->phase == qt->phase)
			continue;

		if (0 && mergesmall(p)) {
			p->phase = qt->phase;

			if (DEBUG)
				printf(">>>merge %p %s %g%%\n",p, patch_name(p, buf),
				       p->priority * 100);
			
			patch_merge(qt, p, mergesmall);

			/* restart from the beginning of the list,
			   because everything might have been
			   rearranged (and anyway, the list head is
			   where the most splittable patches are
			   anyway) */
			goto restart_merge_list;
		}
	}

	qt->phase++;
  restart_split_list:
	list_for_each(pp, &qt->visible) {
		struct patch *p = list_entry(pp, struct patch, list);

		assert((p->flags & (PF_CULLED|PF_ACTIVE)) == PF_ACTIVE);
		assert(p->pinned == 0);
			
		if (p->phase == qt->phase)
			continue;

		if (p->priority >= MAXSIZE) {
			p->phase = qt->phase;
			
			if (DEBUG)
				printf(">>>split %p %s %g%%\n",
				       p, patch_name(p, buf),
				       p->priority * 100);
			
			patch_split(qt, p);
			goto restart_split_list;
		}
	}

	{
		qt->phase++;

	  restart_recull_list:
		list_for_each(pp, &qt->visible) {
			struct patch *p = list_entry(pp, struct patch, list);

			if (p->phase == qt->phase)
				continue;
			p->phase = qt->phase;

			assert((p->flags & PF_CULLED) == 0);
			if (box_cull(&p->bbox, cullplanes, 7) == CULL_OUT) {
				printf("%s: needs culling\n", patch_name(p, buf));
				patch_remove_active(qt, p);
				p->flags |= PF_CULLED | PF_LATECULL;
				patch_insert_active(qt, p);
				goto restart_recull_list;
			}
		}
	}

	generate_geom(qt);
}

/* Return the a classification of a patch's neighbours to determine
   which need special handling in generating a mesh.  The return is an
   index into patch_indices[], and must match genpatchidx.c. */
static unsigned neighbour_class(const struct patch *p)
{
	unsigned ud = 0;
	unsigned lr = 0;

	lr |= (p->neigh[PN_RIGHT]->level < p->level) << 0;
	lr |= (p->neigh[PN_LEFT ]->level < p->level) << 1;

	ud |= (p->neigh[PN_DOWN ]->level < p->level) << 0;
	ud |= (p->neigh[PN_UP   ]->level < p->level) << 1;

	assert(lr < 3);
	assert(ud < 3);

	return ud * 3 + lr;
}

static void set_vertex(struct quadtree *qt, struct vertex *v,
		       int i, int j,
		       long x, long y, long z)
{
	vec3_t ov, ev;
	elevation_t elev;

	project_to_sphere(qt->radius,
			  x, y, z,
			  &ov);
					
	elev = (*qt->landscape)(&ov, v->col);

	ev = ov;
	vec3_normalize(&ev);
	vec3_scale(&ev, elev);
	vec3_add(&ev, &ev, &ov);

	v->s = i;					
	v->t = PATCH_SAMPLES - j;

	v->x = ev.x;
	v->y = ev.y;
	v->z = ev.z;
}

static void generate_geom(struct quadtree *qt)
{
	struct list_head *pp;

	if (have_vbo)
		glBindBuffer(GL_ARRAY_BUFFER, qt->vtxbufid);

	list_for_each(pp, &qt->visible) {
		struct patch *p = list_entry(pp, struct patch, list);

		if (USE_INDEX) {
			/* with indexed drawing, stitching happens at
			   render time */
			p->flags &= ~PF_STITCH_GEOM;
		}

		if ((p->flags & (PF_UPDATE_GEOM|PF_STITCH_GEOM)) == 0)
			continue;

		p->flags &= ~(PF_UPDATE_GEOM|PF_STITCH_GEOM);

		struct vertex samples[MESH_SAMPLES * MESH_SAMPLES];

		if (p->x0 == p->x1) {
			int dy = p->y1 - p->y0;
			int dz = p->z1 - p->z0;

			for(int j = 0; j < MESH_SAMPLES; j++) {
				for(int i = 0; i < MESH_SAMPLES; i++) {
					long x, y, z;
					struct vertex *v = &samples[j * MESH_SAMPLES + i];

					x = p->x0;
					y = p->y0 + (dy * i) / PATCH_SAMPLES;
					z = p->z0 + (dz * j) / PATCH_SAMPLES;

					memcpy(v->col, p->col, 4);
					set_vertex(qt, v, i, j, x, y, z);
				}
			}
		} else if (p->y0 == p->y1) {
			int dx = p->x1 - p->x0;
			int dz = p->z1 - p->z0;

			for(int j = 0; j < MESH_SAMPLES; j++) {
				for(int i = 0; i < MESH_SAMPLES; i++) {
					long x, y, z;
					struct vertex *v = &samples[j * MESH_SAMPLES + i];

					x = p->x0 + (dx * i) / PATCH_SAMPLES;
					y = p->y0;
					z = p->z0 + (dz * j) / PATCH_SAMPLES;

					memcpy(v->col, p->col, 4);
					set_vertex(qt, v, i, j, x, y, z);
				}
			}
		} else if (p->z0 == p->z1) {
			int dx = p->x1 - p->x0;
			int dy = p->y1 - p->y0;

			for(int j = 0; j < MESH_SAMPLES; j++) {
				for(int i = 0; i < MESH_SAMPLES; i++) {
					long x, y, z;
					struct vertex *v = &samples[j * MESH_SAMPLES + i];

					x = p->x0 + (dx * i) / PATCH_SAMPLES;
					y = p->y0 + (dy * j) / PATCH_SAMPLES;
					z = p->z0;

					memcpy(v->col, p->col, 4);
					set_vertex(qt, v, i, j, x, y, z);
				}
			}
		}

		/* quick and dirty normals */
		for(int y = 0; y < PATCH_SAMPLES; y++) {
			for(int x = 0; x < PATCH_SAMPLES; x++) {
				struct vertex *v = &samples[y * MESH_SAMPLES + x];
				struct vertex *vx = &samples[y * MESH_SAMPLES + x + 1];
				struct vertex *vy = &samples[(y+1) * MESH_SAMPLES + x];
				vec3_t vecx = VEC3(vx->x - v->x,
						   vx->y - v->y,
						   vx->z - v->z);
				vec3_t vecy = VEC3(vy->x - v->x,
						   vy->y - v->y,
						   vy->z - v->z);
				vec3_t cross;

				vec3_cross(&cross, &vecx, &vecy);
				vec3_normalize(&cross);

				v->nx = cross.x * 127;
				v->ny = cross.y * 127;
				v->nz = cross.z * 127;
			}
		}

		/* just make edge normals copies of their neighbours */
		for(int x = 0; x < PATCH_SAMPLES; x++) {
			struct vertex *a, *b;

			a = &samples[PATCH_SAMPLES * MESH_SAMPLES + x];
			b = a - (PATCH_SAMPLES+1);

			a->nx = b->nx;
			a->ny = b->ny;
			a->nz = b->nz;
		}

		for(int y = 0; y < PATCH_SAMPLES+1; y++) {
			struct vertex *a, *b;

			a = &samples[y * MESH_SAMPLES + PATCH_SAMPLES];
			b = a - 1;

			a->nx = b->nx;
			a->ny = b->ny;
			a->nz = b->nz;
		}


		if (USE_INDEX) {
			if (have_vbo)
				glBufferSubData(GL_ARRAY_BUFFER,
						p->vertex_offset * sizeof(struct vertex),
						sizeof(samples), samples);
			else
				memcpy(&qt->varray[p->vertex_offset],
				       samples, sizeof(samples));
		} else {
			struct vertex strip[VERTICES_PER_PATCH];
			unsigned nclass = neighbour_class(p);

			for(int idx = 0; idx < INDICES_PER_PATCH; idx++)
				strip[idx] = samples[patch_indices[nclass][idx]];
			
			if (have_vbo) {
				glBufferSubData(GL_ARRAY_BUFFER,
						p->vertex_offset * sizeof(struct vertex),
						sizeof(strip), strip);
			} else {
				memcpy(&qt->varray[p->vertex_offset],
				       strip, sizeof(strip));
			}
		}
	}

	if (have_vbo)
		glBindBuffer(GL_ARRAY_BUFFER, 0);

}

/* set up vertex array pointers, starting at vertex offset "offset" */
static void set_array_pointers(const struct quadtree *qt, unsigned offset)
{
	glVertexPointer(3, GL_FLOAT, sizeof(struct vertex),
			(char *)&qt->varray[offset] + offsetof(struct vertex, x));
	glColorPointer(3, GL_UNSIGNED_BYTE, sizeof(struct vertex),
		       (char *)&qt->varray[offset] + offsetof(struct vertex, col));
	glTexCoordPointer(2, GL_SHORT, sizeof(struct vertex),
			  (char *)&qt->varray[offset] + offsetof(struct vertex, s));
	glNormalPointer(GL_BYTE, sizeof(struct vertex), 
			(char *)&qt->varray[offset] + offsetof(struct vertex, nx));
}

void quadtree_render(const struct quadtree *qt, void (*prerender)(const struct patch *p))
{
	assert(have_vbo != -1);
	assert(have_cva != -1);

	if (have_vbo) {
		glBindBuffer(GL_ARRAY_BUFFER, qt->vtxbufid);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_bufid);
		GLERROR();
	}

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	if (!USE_INDEX)
		set_array_pointers(qt, 0);

	struct list_head *pp;
	list_for_each(pp, &qt->visible) {
		const struct patch *p = list_entry(pp, struct patch, list);

		assert((p->flags & (PF_ACTIVE|PF_CULLED|PF_UPDATE_GEOM|PF_STITCH_GEOM)) == PF_ACTIVE);

		if (prerender)
			(*prerender)(p);

		if (USE_INDEX) {
			unsigned nclass = neighbour_class(p);
			
			set_array_pointers(qt, p->vertex_offset);
			
			glDrawRangeElements(GL_TRIANGLE_STRIP,
					    0, VERTICES_PER_PATCH, 
					    INDICES_PER_PATCH,
					    PATCH_INDEX_TYPE, (*patchidx)[nclass]);
		} else
			glDrawArrays(GL_TRIANGLE_STRIP, p->vertex_offset, 
				     VERTICES_PER_PATCH);
		GLERROR();
	}

	if (1 || ANNOTATE) {
		glPushAttrib(GL_ENABLE_BIT);
		glDisable(GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);

		list_for_each(pp, &qt->culled) {
			const struct patch *p = list_entry(pp, struct patch, list);
			const box_t *b = &p->bbox;

			glColor3f(1,1,0);
			glBegin(GL_POINTS);
			glVertex3fv(b->centre.v);
			glEnd();

			if (p->flags & PF_LATECULL)
				glColor3f(p->priority, 0, p->priority);
			else if (p->phase == qt->phase) 
				glColor3f(p->priority, p->priority, 0);
			else
				glColor3f(p->priority, p->priority, p->priority);
			patch_outline(p, qt->radius);

			glColor3f(.75,0,0);
			//patch_bbox(p);
		}
		glPopAttrib();
	}

	if (have_vbo) {
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
	GLERROR();
}
