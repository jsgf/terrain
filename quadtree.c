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

#define DEBUG		0
#define ANNOTATE	1

#define TARGETSIZE (1.f / 100.f)		/* target size as fraction of screen area */
static const float MARGIN  = TARGETSIZE;	/* size of error needed before updating */
static const float MAXSIZE =  3*TARGETSIZE;	/* error threshold for splitting */
static const float MINSIZE = -3*TARGETSIZE;	/* error threshold for merging */


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
	texcoord_t s,t;
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

static int patch_flip(const vec3_t *face)
{
	float s = face->x + face->y + face->z;

	assert(s == 1.f || s == -1.f);

	return s == -1.f;
}

static void patch_sample_normal(const struct quadtree *qt, const struct patch *p,
				int si, int sj, vec3_t *v)
{
	vec3_t iv, jv, rv;
	int i, j, radius;

	radius = qt->radius;
	rv = *p->face;

	if (patch_flip(p->face)) {
		/* for -ve faces, transpose i&j to make the patch
		   triangles outward-facing */
		int t = si;
		si = sj;
		sj = t;

		vec3_abs(&rv);
		radius = -radius;
	}

	i = p->i0 + (p->i1 - p->i0) * si / PATCH_SAMPLES;
	j = p->j0 + (p->j1 - p->j0) * sj / PATCH_SAMPLES;

	iv = VEC3(rv.z, rv.x, rv.y);
	vec3_scale(&iv, i);

	jv = VEC3(rv.y, rv.z, rv.x);
	vec3_scale(&jv, j);

	vec3_scale(&rv, radius);

	*v = rv;
	vec3_add(v, v, &iv);
	vec3_add(v, v, &jv);

	vec3_normalize(v);
}

static void patch_corner_normals(const struct quadtree *qt, const struct patch *p,
				 vec3_t v[4])
{
	patch_sample_normal(qt, p, 0, 0, &v[0]);
	patch_sample_normal(qt, p, PATCH_SAMPLES, 0, &v[1]);
	patch_sample_normal(qt, p, PATCH_SAMPLES, PATCH_SAMPLES, &v[2]);
	patch_sample_normal(qt, p, 0, PATCH_SAMPLES, &v[3]);
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

	if (patch_flip(p->face)) {
		/* patch_sample_normal() transposes i&j for -ve faces */
		unsigned t = lr;
		lr = ud;
		ud = t;

		/* swap l<->r and u<->d */
		lr = ((lr | (lr << 2)) >> 1) & 3;
		ud = ((ud | (ud << 2)) >> 1) & 3;
	}
	assert(lr < 3);
	assert(ud < 3);

	return ud * 3 + lr;
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

static void patch_init(struct patch *p, int level, unsigned id,
		       const vec3_t *face)
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
	p->face = face;

	p->priority = 0.f;
	p->error = 0.f;
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
	vec3_t sph[5];
	const float terrain_factor = qt->radius * .05f; /* 5% */

	/* Model the patch as a pyramid, with the apex being the
	   centre.  The base is lowered and the apex raised to make
	   sure the bbox fits all the terrain.  */

	patch_corner_normals(qt, p, sph);
	patch_sample_normal(qt, p, PATCH_SAMPLES/2, PATCH_SAMPLES/2, &sph[4]);

	for(int i = 0; i < 4; i++)
		vec3_scale(&sph[i], qt->radius - terrain_factor);

	vec3_scale(&sph[4], qt->radius + terrain_factor);

	p->bbox.centre = VEC3(0,0,0);
	for(int i = 0; i < sizeof(sph)/sizeof(*sph); i++)
		vec3_add(&p->bbox.centre, &p->bbox.centre, &sph[i]);
	vec3_scale(&p->bbox.centre, 1. / (sizeof(sph)/sizeof(*sph)));

	p->bbox.extent = VEC3(0,0,0);
	for(int i = 0; i < sizeof(sph)/sizeof(*sph); i++) {
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
		//printf("merge %s failed: maymerge failed\n", patch_name(p, buf));
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
			if (0)
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

		assert(parent->face == p->face);
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

		patch_init(parent, level, id, p->face);
		
		parent->i0 = sib[0]->i0;
		parent->j0 = sib[0]->j0;
		parent->i1 = sib[2]->i1;
		parent->j1 = sib[2]->j1;

		compute_bbox(qt, parent);
	}
	parent->priority = 0.f;
	parent->error = 0.f;
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
	if ((parent->j1 - parent->j0) / 2 < PATCH_SAMPLES) {
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
			patch_init(k[i], parent->level + 1,
				   childid(parent->id, i), parent->face);

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
		k[i]->error = 0.f;

		k[i]->flags |= parent->flags & PF_CULLED;
		k[i]->pinned++;
		k[i]->phase = parent->phase;

		assert(k[i]->parent == parent);
		assert(k[i]->face == parent->face);
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

	int mi = (parent->i0 + parent->i1) / 2;
	int mj = (parent->j0 + parent->j1) / 2;
	k[0]->i0 = parent->i0;
	k[0]->i1 = mi;
	k[0]->j0 = parent->j0;
	k[0]->j1 = mj;

	k[1]->i0 = mi;
	k[1]->i1 = parent->i1;
	k[1]->j0 = parent->j0;
	k[1]->j1 = mj;

	k[2]->i0 = mi;
	k[2]->i1 = parent->i1;
	k[2]->j0 = mj;
	k[2]->j1 = parent->j1;

	k[3]->i0 = parent->i0;
	k[3]->i1 = mi;
	k[3]->j0 = mj;
	k[3]->j1 = parent->j1;

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
			k[i]->flags = PF_UNUSED;
			patch_init(k[i], -1, 0, NULL);
			patch_free(qt, k[i]);
		}
	return 0;
}

struct quadtree *quadtree_create(int num_patches, long radius, generator_t *generator)
{
	struct quadtree *qt = NULL;

	if (num_patches < 6)
		goto out;

	qt = malloc(sizeof(*qt));

	if (qt == NULL)
		goto out;

	qt->generator = generator;
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

	/* face normals for the cube */
	static const vec3_t *cube[6] = {
		&vec_px,
		&vec_nx,
		&vec_py,
		&vec_ny,
		&vec_pz,
		&vec_nz,
	};

	/* create patches as faces of a cube */
	struct patch *faces[6];

	for(int i = 0; i < 6; i++) {
		struct patch *p = patch_alloc(qt);

		if (p == NULL)
			goto out;

		faces[i] = p;

		p->i0 = p->j0 = -radius;
		p->i1 = p->j1 =  radius;

		patch_init(p, 0, i, cube[i]);

		compute_bbox(qt, p);
	}

	/* For each face, work out the normal of the neighbouring
	   faces, and link up the neighbours appropriately */
	for(int i = 0; i < 6; i++) {
		struct patch *f = faces[i];
		vec3_t sides[4];

		patch_sample_normal(qt, f, PATCH_SAMPLES+1, PATCH_SAMPLES/2, &sides[0]); /* right */
		patch_sample_normal(qt, f, PATCH_SAMPLES/2, PATCH_SAMPLES+1, &sides[1]); /* up */
		patch_sample_normal(qt, f, -1, PATCH_SAMPLES/2, &sides[2]); /* left */
		patch_sample_normal(qt, f, PATCH_SAMPLES/2, -1, &sides[3]); /* down */

		for(int i = 0; i < 4; i++) {
			vec3_majoraxis(&sides[i], &sides[i]);

			if (1 || DEBUG) {
				char buf[40];
				printf("%s: neighbour %d = (%g,%g,%g)\n",
				       patch_name(f, buf), i, sides[i].x, sides[i].y, sides[i].z);
			}

			for(int j = 0; j < 6; j++) {
				if (vec3_equal(&sides[i], faces[j]->face)) {
					if (1 || DEBUG)
						printf("  -> face %d\n", j);

					f->neigh[i * 2 + 0] = faces[j];
					f->neigh[i * 2 + 1] = faces[j];
					break;
				}
				assert(j != 5);	/* can't not find a neighbour */
			}
		}

		patch_insert_active(qt, f);
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

static void patch_outline(const struct quadtree *qt, const struct patch *p)
{
	vec3_t sph[4];
	int radius = qt->radius;

	patch_corner_normals(qt, p, sph);
	for(int i = 0; i < 4; i++)
		vec3_scale(&sph[i], radius);

	glBegin(GL_LINE_LOOP);
	for(int i = 0; i < 4; i++)
		glVertex3fv(sph[i].v);
	glEnd();
}

static float projected_quad_area(const struct quadtree *qt, const struct patch *p,
				 const matrix_t *mat,
				 int i0, int i1, int j0, int j1)
{
	vec3_t v[4];
	vec3_t proj[4];

	patch_sample_normal(qt, p, i0, j0, &v[0]);
	patch_sample_normal(qt, p, i0, j1, &v[1]);
	patch_sample_normal(qt, p, i1, j1, &v[2]);
	patch_sample_normal(qt, p, i1, j0, &v[3]);

	for(int i = 0; i < 4; i++) {
		static const vec3_t half = VEC3i(.5,.5,.5);
		char buf[40];

		vec3_scale(&v[i], qt->radius);
		matrix_project(mat, &v[i], &proj[i]);

		vec3_scale(&proj[i], .5);
		vec3_add(&proj[i], &proj[i], &half);

		if (0)
			printf("%s: %d-%d %d-%d vtx:%d = %g,%g\n",
			       patch_name(p, buf), i0, i1, j0, j1, i, proj[i].x, proj[i].y);
	}

	float area = 0.f;

	for(unsigned i = 0; i < 4; i++) {
		unsigned n = (i+1) % 4;
		area += (proj[i].x * proj[n].y) -
			(proj[n].x * proj[i].y);
	}

	area = -area;		/* hm, get an even number of sign bugs */

	if (0)
		printf("    %d-%d %d-%d area=%g\n",
		       i0, i1, j0, j1, area * .5);

	if (area < 0)
		area = 0;

	return area * 0.5f;
}

static void update_prio(const struct quadtree *qt,
			struct patch *p,
			const matrix_t *mat,
			plane_t cullplanes[7],
			const vec3_t *camera)
{
	long radius = qt->radius;

	p->flags &= ~PF_CULLED;

	if (box_cull(&p->bbox, cullplanes, 7) == CULL_OUT) {
		vec3_t distv;

		vec3_sub(&distv, &p->bbox.centre, camera);

		p->flags |= PF_CULLED;

		/* higher prio = more reusable */
		p->priority = vec3_magnitude(&distv) / (2.f * radius);
		p->error = 0.f;
	} else {
		float area = 0.f;

		area += projected_quad_area(qt, p, mat,
					    0, PATCH_SAMPLES/2, 0, PATCH_SAMPLES/2);
		area += projected_quad_area(qt, p, mat,
					    PATCH_SAMPLES/2, PATCH_SAMPLES, 0, PATCH_SAMPLES/2);
		area += projected_quad_area(qt, p, mat,
					    PATCH_SAMPLES/2, PATCH_SAMPLES, PATCH_SAMPLES/2, PATCH_SAMPLES);
		area += projected_quad_area(qt, p, mat,
					    0, PATCH_SAMPLES/2, PATCH_SAMPLES/2, PATCH_SAMPLES);
		p->priority = area;
		if (fabsf(area - TARGETSIZE) > MARGIN)
			p->error += area - TARGETSIZE;

		if (DEBUG && 0) {
			char buf[40];
			printf("%s p->priority = %g%%, area=%g\n",
			       patch_name(p, buf),
			       p->priority * 100.f,
			       area);
		}
	}
}

static void generate_geom(const struct quadtree *qt);

static int mergesmall(const struct patch *p)
{
	return (p->error < MINSIZE);
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

		update_prio(qt, p, mat, cullplanes, camerapos);

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

		if (mergesmall(p)) {
			p->phase = qt->phase;

			if (DEBUG)
				printf(">>>merge %p %s pri=%g%%, error=%g%%\n",p, patch_name(p, buf),
				       p->priority * 100, p->error * 100);
			
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

		if (p->error >= MAXSIZE) {
			p->phase = qt->phase;
			
			if (DEBUG)
				printf(">>>split %p %s pri=%g%%, error=%g%%\n",
				       p, patch_name(p, buf),
				       p->priority * 100, p->error * 100);
			
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
				//printf("%s: needs culling\n", patch_name(p, buf));
				patch_remove_active(qt, p);
				p->flags |= PF_CULLED | PF_LATECULL;
				patch_insert_active(qt, p);
				goto restart_recull_list;
			}
		}
	}

	generate_geom(qt);
}


void vertex_set_colour(struct vertex *vtx, const unsigned char col[4])
{
	memcpy(vtx->col, col, sizeof(vtx->col));
}

void vertex_set_texcoord(struct vertex *vtx, texcoord_t s, texcoord_t t)
{
	vtx->s = s;
	vtx->t = t;
}

static void compute_vertex(const struct quadtree *qt, const struct patch *p,
			   int i, int j, struct vertex *vtx)
{
	vec3_t sv;
	elevation_t elev;

	vtx->s = i;
	vtx->t = PATCH_SAMPLES - j;

	vtx->col[0] = 255;
	vtx->col[1] = 255;
	vtx->col[2] = 255;
	vtx->col[3] = 255;

	patch_sample_normal(qt, p, i, j, &sv);

	elev = (*qt->generator)(&sv, vtx);
	vec3_scale(&sv, qt->radius + elev);

	vtx->x = sv.x;
	vtx->y = sv.y;
	vtx->z = sv.z;
}


static void generate_geom(const struct quadtree *qt)
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

		for(int j = 0; j < MESH_SAMPLES; j++) {
			for(int i = 0; i < MESH_SAMPLES; i++) {
				struct vertex *v = &samples[j * MESH_SAMPLES + i];

				compute_vertex(qt, p, i, j, v);

				if (ANNOTATE) {
					if (i == 0) { /* left - red*/
						v->col[0] = 255;
						v->col[1] = 0;
						v->col[2] = 0;
						v->col[3] = 0;
					} else if (i == MESH_SAMPLES-1) { /* right - green */
						v->col[0] = 0;
						v->col[1] = 255;
						v->col[2] = 0;
						v->col[3] = 0;
					} else if (j == MESH_SAMPLES-1) { /* top - cyan */
						v->col[0] = 0;
						v->col[1] = 255;
						v->col[2] = 255;
						v->col[3] = 0;
					} else if (j == 0) { /* bottom - yellow */
						v->col[0] = 255;
						v->col[1] = 255;
						v->col[2] = 0;
						v->col[3] = 0;
					}
				}
			}
		}

		/* quick and dirty normals */
		for(int j = 0; j < MESH_SAMPLES; j++) {
			for(int i = 0; i < MESH_SAMPLES; i++) {
				struct vertex *v = &samples[j * MESH_SAMPLES + i];
				struct vertex *vn[4]; /* vertex neighbours */
				struct vertex a, b;

				if (i == 0) {
					vn[0] = &a;
					compute_vertex(qt, p, i-1, j, vn[0]);
				} else
					vn[0] = &samples[j * MESH_SAMPLES + (i - 1)];

				if (i == MESH_SAMPLES-1) {
					vn[2] = &a;
					compute_vertex(qt, p, i+1, j, vn[2]);
				} else
					vn[2] = &samples[j * MESH_SAMPLES + (i + 1)];


				if (j == 0) {
					vn[1] = &b;
					compute_vertex(qt, p, i, j-1, vn[1]);
				} else
					vn[1] = &samples[(j - 1) * MESH_SAMPLES + i];

				if (j == MESH_SAMPLES-1) {
					vn[3] = &b;
					compute_vertex(qt, p, i, j+1, vn[3]);
				} else
					vn[3] = &samples[(j + 1) * MESH_SAMPLES + i];

				vec3_t norm = VEC3(0,0,0);

				for(unsigned x = 0; x < 3; x++) {
					int nx = (x + 1) % 4;
					vec3_t v1 = VEC3(vn[x]->x - v->x, 
							 vn[x]->y - v->y, 
							 vn[x]->z - v->z);
					vec3_t v2 = VEC3(vn[nx]->x - v->x, 
							 vn[nx]->y - v->y, 
							 vn[nx]->z - v->z);
					vec3_t cross;

					vec3_cross(&cross, &v1, &v2);
					vec3_normalize(&cross);

					vec3_add(&norm, &norm, &cross);
				}

				vec3_scale(&norm, 1./4);

				v->nx = norm.x * 127;
				v->ny = norm.y * 127;
				v->nz = norm.z * 127;
			}
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

		if (ANNOTATE && !have_vbo) {
			struct vertex *va = &qt->varray[p->vertex_offset];

			glPushAttrib(GL_ENABLE_BIT);
			glDisable(GL_LIGHTING);
			glDisable(GL_TEXTURE_2D);

			glBegin(GL_LINES);
			for(int i = 0; i < MESH_SAMPLES * MESH_SAMPLES; i++) {
				struct vertex *v = &va[i];

				glColor4ubv(v->col);
				glVertex3fv(&v->x);
				glVertex3f(v->x + v->nx, v->y + v->ny, v->z + v->nz);
			}
			glEnd();

			glPopAttrib();
		}

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
			patch_outline(qt, p);

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
