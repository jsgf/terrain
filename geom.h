#ifndef _GEOM_H
#define _GEOM_H

typedef union vec3 {
	float v[3];
	struct {
		float x, y, z;
	};
} vec3_t;

#define PURE	__attribute__((pure))

void  vec3_cross(vec3_t *out, const vec3_t *a, const vec3_t *b);
float vec3_dot(const vec3_t *a, const vec3_t *b);
void  vec3_normalize(vec3_t *v);
float vec3_magnitude(const vec3_t *v);
void  vec3_scale(vec3_t *v, float scale);
void  vec3_add(vec3_t *out, const vec3_t *a, const vec3_t *b);
void  vec3_sub(vec3_t *out, const vec3_t *a, const vec3_t *b);
void  vec3_abs(vec3_t *v);
void  vec3_min(vec3_t *out, const vec3_t *a, const vec3_t *b);
void  vec3_max(vec3_t *out, const vec3_t *a, const vec3_t *b);
void  vec3_rotate(vec3_t *out, const vec3_t *v, float angle, const vec3_t *axis);
void  vec3_majoraxis(vec3_t *out, const vec3_t *v);

extern const vec3_t vec_px, vec_nx, vec_py, vec_ny, vec_pz, vec_nz;

#define VEC3(x,y,z)	((vec3_t){ .v = { (x), (y), (z) } })
#define VEC3i(x,y,z)	{ .v = { (x), (y), (z) } }

typedef struct quat {
	vec3_t v;
	float w;
} quat_t;

#define QUAT(_s, _v)	((quat_t) { .v = (_v), .w = (_s) })
#define QUAT_IDENT	QUAT(1, VEC3(0,0,0))
#define QUAT_ZERO	QUAT(0, VEC3(0,0,0))

void quat_axis_angle(quat_t *q, const vec3_t *v, float angle);
void quat_vector_vector(quat_t *q, const vec3_t *a, const vec3_t *b);
void quat_normalize(quat_t *q);
void quat_conj(quat_t *out, const quat_t *in);
void quat_invert(quat_t *out, const quat_t *in);
void quat_rotate(vec3_t *res, const quat_t *q, const vec3_t *v);

typedef struct plane {
	vec3_t normal;
	float dist;
} plane_t;

typedef struct box {
	vec3_t centre;
	vec3_t extent;
} box_t;

typedef union matrix {
	float m[16];
	struct {
		/* _RC */
		float _11, _21, _31, _41;
		float _12, _22, _32, _42;
		float _13, _23, _33, _43;
		float _14, _24, _34, _44;
	};
} matrix_t;

#define MATRIX_IDENT	((matrix_t) { .m = { [0] = 1.f, [5] = 1.f, [10] = 1.f, [15] = 1.f } })

void matrix_quat(matrix_t *mat, const quat_t *q);
void matrix_transform(const matrix_t *mat, const vec3_t *in, vec3_t *out);
void matrix_project(const matrix_t *mat, const vec3_t *in, vec3_t *out);
void matrix_multiply(const matrix_t *a, const matrix_t *b, matrix_t *out);

enum {
	PLANE_LEFT,
	PLANE_RIGHT,
	PLANE_TOP,
	PLANE_BOTTOM,
	PLANE_NEAR,
	PLANE_FAR,
};

/* Extract frustum planes from a viewing matrix */
void plane_extract(const matrix_t *mat, plane_t planes[6]);
void plane_normalize(plane_t *plane);

enum cull_result {
	CULL_IN,
	CULL_OUT,
	CULL_PARTIAL,
};

enum cull_result box_cull(const box_t *box, const plane_t *planes, int nplanes);

#endif	/* _GEOM_H */
