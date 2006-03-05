#include <math.h>

#include "geom.h"

void vec3_cross(vec3_t *out, const vec3_t *a, const vec3_t *b)
{
	out->x = a->y * b->z - a->z * b->y;
	out->y = a->z * b->x - a->x * b->z;
	out->z = a->x * b->y - a->y * b->x;
}

float vec3_dot(const vec3_t *a, const vec3_t *b)
{
	return a->x*b->x + a->y*b->y + a->z*b->z;
}

float vec3_magnitude(const vec3_t *v)
{
	return sqrtf(vec3_dot(v, v));;
}

void vec3_scale(vec3_t *v, float scale)
{
	v->x *= scale;
	v->y *= scale;
	v->z *= scale;
}

void vec3_normalize(vec3_t *v)
{
	float len = vec3_magnitude(v);

	if (len < 1.e-5)
		return;

	len = 1.f / len;

	vec3_scale(v, len);
}

void vec3_add(vec3_t *out, const vec3_t *a, const vec3_t *b)
{
	out->x = a->x + b->x;
	out->y = a->y + b->y;
	out->z = a->z + b->z;
}

void vec3_sub(vec3_t *out, const vec3_t *a, const vec3_t *b)
{
	out->x = a->x - b->x;
	out->y = a->y - b->y;
	out->z = a->z - b->z;
}

void vec3_abs(vec3_t *v)
{
	v->x = fabsf(v->x);
	v->y = fabsf(v->y);
	v->z = fabsf(v->z);
}

void vec3_min(vec3_t *out, const vec3_t *a, const vec3_t *b)
{
	out->x = a->x < b->x ? a->x : b->x;
	out->y = a->y < b->y ? a->y : b->y;
	out->z = a->z < b->z ? a->z : b->z;
}

void vec3_max(vec3_t *out, const vec3_t *a, const vec3_t *b)
{
	out->x = a->x > b->x ? a->x : b->x;
	out->y = a->y > b->y ? a->y : b->y;
	out->z = a->z > b->z ? a->z : b->z;
}

void matrix_transform(const matrix_t *m, const vec3_t *in, vec3_t *out)
{
	vec3_t t;

	t.x = m->m[0] * in->x + m->m[4] * in->y + m->m[ 8] * in->z + m->m[12];
	t.y = m->m[1] * in->x + m->m[5] * in->y + m->m[ 9] * in->z + m->m[13];
	t.z = m->m[2] * in->x + m->m[6] * in->y + m->m[10] * in->z + m->m[14];

	*out = t;
}

void matrix_multiply(const matrix_t *a, const matrix_t *b, matrix_t *out)
{
#define A(r,c) a->m[4*c+r]
#define B(r,c) b->m[4*c+r]
#define P(r,c) out->m[4*c+r]

	for(int i = 0; i < 4; i++) {
		const float ai0 = A(i, 0), ai1=A(i,1), ai2=A(i,2), ai3=A(i,3);
                P(i,0) = ai0 * B(0,0) + ai1 * B(1,0) + ai2 * B(2,0) + ai3 * B(3,0);
                P(i,1) = ai0 * B(0,1) + ai1 * B(1,1) + ai2 * B(2,1) + ai3 * B(3,1);
                P(i,2) = ai0 * B(0,2) + ai1 * B(1,2) + ai2 * B(2,2) + ai3 * B(3,2);
                P(i,3) = ai0 * B(0,3) + ai1 * B(1,3) + ai2 * B(2,3) + ai3 * B(3,3);
	}
#undef A
#undef B
#undef P
}

void plane_normalize(plane_t *p)
{
	float len = 1.f / vec3_magnitude(&p->normal);

	vec3_scale(&p->normal, len);
	p->dist *= len;
}

/* 
   Taken from "Fast Extraction of Viewing Frustum Planes from the
   World- View-Projection Matrix" Gil Gribb, Klaus Hartmann.  Planes
   will be in whatever space your matrix transforms from.
 */
void plane_extract(const matrix_t *mat, plane_t planes[6])
{
	// Left clipping plane
	planes[PLANE_LEFT].normal.x = mat->_41 + mat->_11;
	planes[PLANE_LEFT].normal.y = mat->_42 + mat->_12;
	planes[PLANE_LEFT].normal.z = mat->_43 + mat->_13;
	planes[PLANE_LEFT].dist     = mat->_44 + mat->_14;

	// Right clipping plane
	planes[PLANE_RIGHT].normal.x = mat->_41 - mat->_11;
	planes[PLANE_RIGHT].normal.y = mat->_42 - mat->_12;
	planes[PLANE_RIGHT].normal.z = mat->_43 - mat->_13;
	planes[PLANE_RIGHT].dist     = mat->_44 - mat->_14;

	// Top clipping plane
	planes[PLANE_TOP].normal.x = mat->_41 - mat->_21;
	planes[PLANE_TOP].normal.y = mat->_42 - mat->_22;
	planes[PLANE_TOP].normal.z = mat->_43 - mat->_23;
	planes[PLANE_TOP].dist     = mat->_44 - mat->_24;

	// Bottom clipping plane
	planes[PLANE_BOTTOM].normal.x = mat->_41 + mat->_21;
	planes[PLANE_BOTTOM].normal.y = mat->_42 + mat->_22;
	planes[PLANE_BOTTOM].normal.z = mat->_43 + mat->_23;
	planes[PLANE_BOTTOM].dist     = mat->_44 + mat->_24;

	// Near clipping plane
	planes[PLANE_NEAR].normal.x = mat->_41 + mat->_31;
	planes[PLANE_NEAR].normal.y = mat->_42 + mat->_32;
	planes[PLANE_NEAR].normal.z = mat->_43 + mat->_33;
	planes[PLANE_NEAR].dist     = mat->_44 + mat->_34;

	// Far clipping plane
	planes[PLANE_FAR].normal.x = mat->_41 - mat->_31;
	planes[PLANE_FAR].normal.y = mat->_42 - mat->_32;
	planes[PLANE_FAR].normal.z = mat->_43 - mat->_33;
	planes[PLANE_FAR].dist     = mat->_44 - mat->_34;
}

enum cull_result box_cull(const box_t *box, const plane_t *planes, int nplanes)
{
	int visible = 0;

	for(int i = 0; i < nplanes; i++) {
		const plane_t *p = &planes[i];
		float reff =
			fabsf(box->extent.x * p->normal.x) +
			fabsf(box->extent.y * p->normal.y) +
			fabsf(box->extent.z * p->normal.z);

		float dot = vec3_dot(&p->normal, &box->centre) + p->dist;

		if (dot <= -reff)
			return CULL_OUT;
		else {
			if (++visible == nplanes)
				return CULL_IN;
		}
	}
	
	return CULL_PARTIAL;
}
