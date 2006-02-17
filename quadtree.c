#include <stddef.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>

#include "quadtree.h"

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

void emitdot(struct quadtree *qt, const char *name)
{
	FILE *f = fopen(name, "w");

	fprintf(f, "digraph \"%s\" {\n", name);

	for(int i = 0; i < PRIO_BUCKETS; i++) {
		struct patch *head = qt->active[i];

		if (head) {
			struct patch *p = head;

			do {
				static const char *dirname[] = {
#define DN(x)	[PN_##x] = #x, [PN_##x##_1] = #x "_1"
					DN(RIGHT),
					DN(LEFT),
					DN(UP),
					DN(DOWN)
#undef DN
				};

				fprintf(f, "\t\"%p\" [label=\"%s\", shape=box];\n",
				       p, id2str(p));
		
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

				p = p->next;
			} while(p != head);
		}
	}
	fprintf(f, "}\n");
}

static int on_freelist(const struct quadtree *qt, const struct patch *p)
{

	if (qt->freelist) {
		const struct patch *pp = qt->freelist;
		do {
			if (pp == p)
				return 1;
			pp = pp->next;
		} while(pp != qt->freelist);
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

static int is_updown(enum patch_neighbour n)
{
	n &= ~1;
	return n == PN_UP || n == PN_DOWN;
}

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

static void patch_insert_tail(struct patch **headp, struct patch *p)
{
	if (*headp == NULL) {
		p->prev = p;
		p->next = p;
		*headp = p;
	} else {
		p->next = *headp;
		p->prev = (*headp)->prev;
		p->prev->next = p;
		p->next->prev = p;
	}
}

static void patch_insert_head(struct patch **headp, struct patch *p)
{
	patch_insert_tail(headp, p);
	*headp = p;
}

static void patch_remove(struct patch **headp, struct patch *p)
{
	if (*headp == p) {
		if (p->next == p)
			*headp = NULL;
		else
			*headp = p->next;
	}

	p->next->prev = p->prev;
	p->prev->next = p->next;
}

static struct patch *patch_remove_head(struct patch **headp)
{
	struct patch *p = NULL;

	if (*headp != NULL) {
		p = *headp;
		patch_remove(headp, p);
	}

	return p;
}

static void patch_insert_active(struct quadtree *qt, struct patch *p)
{
	int queue = p->priority / PRIO_BUCKETS;

	if (qt->active[queue] == NULL)
		patch_insert_head(&qt->active[queue], p);
	else {
		if (p->priority > qt->active[queue]->priority)
			patch_insert_tail(&qt->active[queue], p);
		else
			patch_insert_head(&qt->active[queue], p);
	}
}

static void patch_remove_active(struct quadtree *qt, struct patch *p)
{
	int queue = p->priority / PRIO_BUCKETS;

	patch_remove(&qt->active[queue], p);
}

static void patch_init(struct patch *p, int level, unsigned id)
{
	if (p->level != -1) {
		/* patch still linked in; break links */
		printf("recycling %p\n", p);

		/* unlink from parent */
		if (p->parent)
			for(int i = 0; i < 4; i++)
				if (p->parent->kids[i] == p)
					p->parent->kids[i] = NULL;

		/* unlink from kids */
		for(int i = 0; i < 4; i++)
			if (p->kids[i] && p->kids[i]->parent == p)
				p->kids[i]->parent = NULL;

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

	p->kids[0] = NULL;
	p->kids[1] = NULL;
	p->kids[2] = NULL;
	p->kids[3] = NULL;

	memset(p->neigh, 0, sizeof(p->neigh));

	p->parent = NULL;
	p->level = level;
	p->id = id;

	p->priority = 0;
}

static struct patch *patch_alloc(struct quadtree *qt)
{
	struct patch *p;

	if (qt->freelist == NULL) {
		printf("patch allocation failed!\n");
		return NULL;
	}

	p = patch_remove_head(&qt->freelist);

	//printf("allocated %p\n", p);

	return p;
}

static void patch_free(struct quadtree *qt, struct patch *p)
{
	//printf("freeing %p\n", p);
	patch_insert_tail(&qt->freelist, p);
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
static struct patch *patch_merge(struct quadtree *qt, struct patch *p)
{
	struct patch *parent;	/* the new patch we're creating */
	struct patch *sib[4];	/* the group of siblings including p */
	unsigned long start_id;

	printf("merging %s\n", id2str(p));
	assert(!on_freelist(qt, p));

	if (p->level == 0)
		return p;

	/* Visit p and all its siblings, make sure they're all at the
	   same level as p.  Add them all to sib[]. */
	start_id = siblingid(p);
	do {
		unsigned long sibid = siblingid(p);
		const struct neighbours *pn = &neighbours[sibid];
		struct patch *sibling = p->neigh[pn->ccw];

		sib[sibid] = p;

		assert(parentid(p, 1) == 
		       parentid(sibling, sibling->level - p->level + 1));

		if (p->level != sibling->level) {
			/* Levels don't match, need to merge the
			   sibling before going on. */
			assert(p->level == sibling->level - 1);
			assert(p->neigh[pn->ccw + 1] != p->neigh[pn->ccw]);
			assert(p->neigh[pn->ccw + 1]->level ==
			       sibling->level);

			sibling = patch_merge(qt, sibling);
		}

		assert(p->level == sibling->level);
		assert(p->neigh[pn->ccw + 1] == p->neigh[pn->ccw]);

		/* check non-sibling neighbours */
		if (p->neigh[pn->ud]->level > p->level) {
			assert(p->neigh[pn->ud]->level == p->level+1);
			patch_merge(qt, p->neigh[pn->ud]);
		}
		assert(p->neigh[pn->ud]->level <= p->level);
		assert(p->neigh[pn->ud] == p->neigh[pn->ud+1]);

		if (p->neigh[pn->lr]->level > p->level) {
			assert(p->neigh[pn->lr]->level == p->level+1);
			patch_merge(qt, p->neigh[pn->lr]);
		}
		assert(p->neigh[pn->lr]->level <= p->level);
		assert(p->neigh[pn->lr] == p->neigh[pn->lr+1]);


		p = sibling;
	} while(siblingid(p) != start_id);

	for(int i = 0; i < 4; i++)
		patch_remove_active(qt, sib[i]);

	if (p->parent != NULL) {
		parent = p->parent; /* cached parent */
		assert(on_freelist(qt, parent));
		patch_remove(&qt->freelist, parent);
	} else {
		unsigned long id = parentid(p, 1);
		int level = p->level - 1;

		/* allocate a new parent */
		parent = patch_alloc(qt);

		assert(parent != NULL);
		assert(!on_freelist(qt, parent));

		patch_init(parent, level, id);
	}

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
	}

	/* now that the forward-links are set up, do the backlinks */
	for(int i = 0; i < 4; i++)
		backlink_neighbours(parent, sib[i]);

	/* free all the siblings */
	for(int i = 0; i < 4; i++) {
		sib[i]->parent = parent;
		parent->kids[i] = sib[i];

		patch_free(qt, sib[i]);
	}

	/* make parent active */
	patch_insert_active(qt, parent);

	assert(check_neighbour_levels(parent));

	return parent;
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
static void patch_split(struct quadtree *qt, struct patch *parent)
{
	struct patch *k[4] = { NULL, NULL, NULL, NULL };

	printf("splitting %s\n", id2str(parent));

	assert(!on_freelist(qt, parent));

	/* Check all the parent patches neighbours to make sure
	   they're a suitable level, and split them if not */
	for(enum patch_neighbour dir = 0; dir < 8; dir++) {
		assert(parent->neigh[dir] != NULL);

		if (parent->neigh[dir]->level < parent->level) {
			assert(parent->neigh[dir]->level == (parent->level-1));
			patch_split(qt, parent->neigh[dir]);
		}
		assert(parent->neigh[dir]->level >= parent->level);
	}

	/* allocate and initialize the 4 new patches */
	for(int i = 0; i < 4; i++) {
		k[i] = parent->kids[i];

		if (k[i] == NULL) {
			k[i] = patch_alloc(qt);
			if (k[i] == NULL)
				goto out_fail;
			patch_init(k[i], parent->level + 1, childid(parent->id, i));

			k[i]->parent = parent;
			parent->kids[i] = k[i];
		} else
			patch_remove(&qt->freelist, k[i]); /* recycled */

		assert(k[i]->parent == parent);
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

	patch_remove_active(qt, parent);
	patch_free(qt, parent);

	for(int i = 0; i < 4; i++)
		patch_insert_active(qt, k[i]);

	return;

  out_fail:
	for(int i = 0; i < 4; i++)
		if (k[i]) {
			patch_init(k[i], -1, 0);
			patch_free(qt, k[i]);
		}
}

struct quadtree *quadtree_create(int num_patches, int radius)
{
	struct quadtree *qt = NULL;

	if (num_patches < 6)
		goto out;

	qt = malloc(sizeof(*qt));

	if (qt == NULL)
		goto out;

	qt->patches = malloc(sizeof(struct patch) * num_patches);
	if (qt->patches == NULL)
		goto out;

	glGenBuffersARB(1, &qt->vtxbufid);
	glBindBufferARB(GL_ARRAY_BUFFER_BINDING, qt->vtxbufid);
	glBufferDataARB(GL_ARRAY_BUFFER_BINDING,
			sizeof(struct vertex) * VERTICES_PER_PATCH * num_patches,
			NULL, GL_DYNAMIC_DRAW_ARB);
	glBindBufferARB(GL_ARRAY_BUFFER_BINDING, 0);

	/* add patches to freelist */
	for(int i = 0; i < num_patches; i++) {
		struct patch *p = &qt->patches[i];

		p->level = -1; /* mark as never used */
		p->vertex_offset = i * VERTICES_PER_PATCH;

		patch_free(qt, p);
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

		patch_insert_active(qt, b);
	}

#if 1
	emitdot(qt, "base.dot");
	//patch_split(qt, basis[0]);
	patch_split(qt, basis[1]);
	emitdot(qt, "split1.dot");

	//patch_split(qt, basis[4]);
	//emitdot(qt, "split4.dot");
	//patch_split(qt, basis[2]);
	patch_split(qt, basis[1]->kids[2]);
	emitdot(qt, "split1-2.dot");
	patch_split(qt, basis[1]->kids[2]->kids[3]);
	emitdot(qt, "split1-2-3.dot");
	patch_split(qt, basis[1]->kids[2]->kids[1]);
	emitdot(qt, "split1-2-1.dot");
	patch_split(qt, basis[1]->kids[2]->kids[1]->kids[1]);
	emitdot(qt, "split1-2-1-1.dot");

	patch_split(qt, basis[1]->kids[2]->kids[1]->kids[1]->kids[0]);
	emitdot(qt, "split1-2-1-1-0.dot");
#endif

#if 0
	patch_merge(qt, basis[1]->kids[2]->kids[0]);
	emitdot(qt, "merge1-2-0.dot");

	patch_merge(qt, basis[0]->kids[1]);
	emitdot(qt, "merge0-1.dot");
	patch_merge(qt, basis[0]);
	emitdot(qt, "merge0.dot");
	patch_merge(qt, basis[4]->kids[1]);
	emitdot(qt, "merge4-1.dot");
	patch_merge(qt, basis[1]->kids[1]);
	emitdot(qt, "merge1-1.dot");
#endif

	return qt;

  out: 
	free(qt);
	return NULL;
}

void quadtree_update_view(struct quadtree *qt, const float view[16])
{
	
}
