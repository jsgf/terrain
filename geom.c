#define _GNU_SOURCE		/* for sincosf */
#include <math.h>

#include "geom.h"

#define EPSILON	(1e-6f)

const vec3_t vec_px = VEC3i( 1, 0, 0);
const vec3_t vec_nx = VEC3i(-1, 0, 0);
const vec3_t vec_py = VEC3i( 0, 1, 0);
const vec3_t vec_ny = VEC3i( 0,-1, 0);
const vec3_t vec_pz = VEC3i( 0, 0, 1);
const vec3_t vec_nz = VEC3i( 0, 0,-1);

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
	return sqrtf(vec3_dot(v, v));
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

void vec3_majoraxis(vec3_t *out, const vec3_t *v)
{
	float x, y, z;

	x = fabsf(v->x);
	y = fabsf(v->y);
	z = fabsf(v->z);

	if (x > y && x > z)
		*out = VEC3(v->x < 0 ? -1 : 1, 0, 0);
	else if (y > x && y > z)
		*out = VEC3(0, v->y < 0 ? -1 : 1, 0);
	else
		*out = VEC3(0, 0, v->z < 0 ? -1 : 1);
}

void vec3_rotate(vec3_t *out, const vec3_t *v, float angle, const vec3_t *axis)
{
	quat_t q;
	quat_axis_angle(&q, axis, angle);
	quat_rotate(out, &q, v);
}

/* epsilon? */
int vec3_equal(const vec3_t *a, const vec3_t *b)
{
	return a->x == b->x && a->y == b->y && a->z == b->z;
}


void quat_axis_angle(quat_t *q, const vec3_t *v, float angle)
{
	float s, c;
	sincosf(angle/2, &s, &c);

	q->w = c;
	q->v = *v;
	vec3_normalize(&q->v);
	vec3_scale(&q->v, s);
}

void quat_vector_vector(quat_t *q, const vec3_t *a, const vec3_t *b)
{
	float cost = vec3_dot(a, b);

	if (cost > 0.99999f) {
		/* parallel */
		*q = QUAT_IDENT;
	} else if (cost < -0.99999f) {
		/* opposite */
		vec3_t t = VEC3(0, a->x, -a->y); /* cross with (1,0,0) */
		if (vec3_magnitude(&t) < EPSILON)
			t = VEC3(-a->z, 0, a->x); /* nope, use (0,1,0) */

		vec3_normalize(&t);

		q->v = t;
		q->w = 0.f;
	} else {
		vec3_t t;

		vec3_cross(&t, a, b);
		vec3_normalize(&t);

		/* sin^2 t = (1 - cos(2t)) / 2 */
		float ss = sqrt(.5f * (1.f - cost));
		vec3_scale(&t, ss);
		q->v = t;

		/* cos^2 t = (1 + cos(2t) / 2 */
		q->w = sqrt(.5f * (1.f + cost));
	}
}

void quat_mult(quat_t *r, const quat_t *a, const quat_t *b)
{
	quat_t ret;
	vec3_t t;

	ret.w = a->w * b->w - vec3_dot(&a->v, &b->v);
	vec3_cross(&ret.v, &a->v, &b->v);

	t = a->v;
	vec3_scale(&t, b->w);
	vec3_add(&ret.v, &ret.v, &t);

	t = b->v;
	vec3_scale(&t, a->w);
	vec3_add(&ret.v, &ret.v, &t);

	*r = ret;
}

void quat_normalize(quat_t *q)
{
	float m = q->w * q->w + vec3_dot(&q->v, &q->v);

	m = 1.f / m;

	q->w *= m;
	vec3_scale(&q->v, m);
}

void quat_conj(quat_t *out, const quat_t *in)
{
	out->v.x = -in->v.x;
	out->v.y = -in->v.y;
	out->v.z = -in->v.z;
	out->w   =  in->w;
}

void quat_invert(quat_t *out, const quat_t *in)
{
	float m = 1.f / (in->w * in->w + vec3_dot(&in->v, &in->v));

	out->v.x = -in->v.x * m;
	out->v.y = -in->v.y * m;
	out->v.z = -in->v.z * m;
	out->w   =  in->w   * m;
}

void quat_rotate(vec3_t *res, const quat_t *q, const vec3_t *v)
{
	quat_t tq, itq, tv;

	quat_conj(&itq, q);

	tv = QUAT(0, *v);

	quat_mult(&tq, q, &tv);
	quat_mult(&tq, &tq, &itq);

	*res = tq.v;
}


void matrix_quat(matrix_t *m, const quat_t *q)
{
	float wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;

	x2 = q->v.x + q->v.x; y2 = q->v.y + q->v.y; z2 = q->v.z + q->v.z;
	xx = q->v.x * x2;   xy = q->v.x * y2;   xz = q->v.x * z2;
	yy = q->v.y * y2;   yz = q->v.y * z2;   zz = q->v.z * z2;
	wx = q->w   * x2;   wy = q->w   * y2;   wz = q->w   * z2;

	m->_11 = 1.0 - (yy + zz);
	m->_21 = xy - wz;
	m->_31 = xz + wy;
	m->_41 = 0.0;
 
	m->_12 = xy + wz;
	m->_22 = 1.0 - (xx + zz);
	m->_32 = yz - wx;
	m->_42 = 0.0;

	m->_31 = xz - wy;
	m->_32 = yz + wx;
	m->_33 = 1.0 - (xx + yy);
	m->_34 = 0.0;

	m->_41 = 0;
	m->_42 = 0;
	m->_43 = 0;
	m->_44 = 1;
}

void matrix_transform(const matrix_t *m, const vec3_t *in, vec3_t *out)
{
	vec3_t t;

	t.x = m->m[0] * in->x + m->m[4] * in->y + m->m[ 8] * in->z + m->m[12];
	t.y = m->m[1] * in->x + m->m[5] * in->y + m->m[ 9] * in->z + m->m[13];
	t.z = m->m[2] * in->x + m->m[6] * in->y + m->m[10] * in->z + m->m[14];

	*out = t;
}

void matrix_project(const matrix_t *m, const vec3_t *in, vec3_t *out)
{
	vec3_t t;
	float w;

	t.x = m->m[0] * in->x + m->m[4] * in->y + m->m[ 8] * in->z + m->m[12];
	t.y = m->m[1] * in->x + m->m[5] * in->y + m->m[ 9] * in->z + m->m[13];
	t.z = m->m[2] * in->x + m->m[6] * in->y + m->m[10] * in->z + m->m[14];
	w   = m->m[3] * in->x + m->m[7] * in->y + m->m[11] * in->z + m->m[15];

	if (w != 0.f)
		vec3_scale(&t, 1.f/w);

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
