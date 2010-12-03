#define _GNU_SOURCE

#include <GL/glut.h>
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "quadtree.h"
#include "noise.h"
#include "font.h"

#define RADIUS (1<<20)

#define LABELS 0

GLuint buildtexture(float variance);

static struct quadtree *qt;

static float elevation, bearing;
static int wireframe = 0;
static int update_view = 1;

static struct fractal *frac;
static float maxvariance, variance, offset;

static GLuint texture;

#define GLERROR()							\
do {									\
	GLenum err = glGetError();					\
	if (err != GL_NO_ERROR) {					\
		printf("GL error at %s:%d: %s\n",			\
		       __FILE__, __LINE__, gluErrorString(err));	\
	}								\
} while(0)

static unsigned pow2(unsigned x)
{
	return 1 << (32 - __builtin_clz(x-1));
}

static void texprintf(const char *fmt, ...)
{
	unsigned len;
	char buf[1024];
	va_list ap;
	unsigned pix;

	va_start(ap, fmt);
	len = vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	pix = pow2(len * 8);

	//printf("  %dx%d tex \"%s\"\n", pix, pix, buf);

	unsigned char tex[pix*pix];
	memset(tex, 0, pix*pix);

	for(int i = 0; i < len; i++) {
		const unsigned char *src = &font[buf[i] * 64];
		unsigned char *dst = &tex[((pix - 8) * pix + (pix - len*8)) / 2 + i * 8];

		for(int y = 0; y < 8; y++) {
			memcpy(dst, src, 8);
			dst += pix;
			src += 8;
		}
	}

	glTexImage2D(GL_TEXTURE_2D, 0, GL_INTENSITY, 
		     pix, pix, 0,
		     GL_LUMINANCE, GL_UNSIGNED_BYTE,
		     tex);
	GLERROR();
}

static int width, height;

static
void reshape (int w, int h)
{
	width = w;
	height = h;

	glEnable(GL_SCISSOR_TEST);
	glScissor(0,0,w,h);
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(50., 16./9., 10, RADIUS*4);

	glMatrixMode(GL_MODELVIEW);
}

static void set_texture(const struct patch *p)
{
#if LABELS
	GLuint texid = (patch_id(p)+1) + (1 << (patch_level(p) * 2 + 4));

	if (!glIsTexture(texid)) {
		char s[40];
		patch_name(p, s);

		if (0)
			printf("generating tex %u for p=%p %d:%lu %s\n",
			       texid, p, patch_level(p), patch_id(p), s);

		glBindTexture(GL_TEXTURE_2D, texid);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		texprintf("%s", s);
	} else
		glBindTexture(GL_TEXTURE_2D, texid);
#else
	glBindTexture(GL_TEXTURE_2D, texture);
#endif
}

static float delta = 1;

static float dolly = RADIUS * 2.5;
static void display()
{
	static float angle;

	glClearColor(.2,.2,.2,1);
	glDepthMask(GL_TRUE);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	float centre = dolly > RADIUS*(1+M_SQRT2) ? 0 : RADIUS*(1+M_SQRT2)-dolly;
	gluLookAt(0,0,-dolly, 0,centre*1.1,0, 0,1,0);

	glRotatef(elevation, 1, 0, 0);
	glRotatef(bearing, 0, 1, 0);
	//glRotatef(angle, 0, 1, 1);
	angle += delta;
	GLERROR();


	if (update_view) {
		matrix_t mv, proj, combined;
		vec3_t camdir = VEC3(0,0,-dolly);

		/* surely this can be prettier... */
		vec3_rotate(&camdir, &camdir, -elevation * M_PI / 180.f, &vec_px);
		vec3_rotate(&camdir, &camdir, -bearing   * M_PI / 180.f, &vec_py);

		glGetFloatv(GL_MODELVIEW_MATRIX, mv.m);
		GLERROR();
		glGetFloatv(GL_PROJECTION_MATRIX, proj.m);
		GLERROR();

		matrix_multiply(&proj, &mv, &combined);

		//printf("camerapos=%g, %g, %g, alt=%g\n", camdir.x, camdir.y, camdir.z, vec3_magnitude(&camdir));

		quadtree_update_view(qt, &combined, &camdir);
	}


	glEnable(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	GLERROR();

	glColor3f(1,1,1);
	glEnable(GL_CULL_FACE);
	if (wireframe) {
		glPolygonMode(GL_FRONT, GL_LINE);
		glDisable(GL_LIGHTING);
	} else {
		glPolygonMode(GL_FRONT, GL_FILL);
		glEnable(GL_LIGHTING);
		glEnable(GL_COLOR_MATERIAL);
	}

	//glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	GLERROR();

	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();

	if (LABELS)
		glScalef(1./PATCH_SAMPLES, 1./PATCH_SAMPLES, 1);
	else {
		glTranslatef(.4, 0, 0);
		glScalef(1./32767, 1./32767, 1);
	}

	glMatrixMode(GL_MODELVIEW);

	if (LABELS) {
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_BLEND);
		GLERROR();
	} else {
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		GLERROR();
	}

	quadtree_render(qt, set_texture);

#if 0
	glEnable(GL_POLYGON_OFFSET_LINE);
	glPolygonMode(GL_FRONT, GL_LINE);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);
	glDisable(GL_LIGHTING);
	glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(.2,.2,.2,.2);
	glPolygonOffset(2,4);
	glDepthMask(GL_FALSE);
	GLERROR();

	quadtree_render(qt, NULL);
#endif

	glPushAttrib(GL_VIEWPORT_BIT | GL_SCISSOR_BIT);
	glPushMatrix();
	glLoadIdentity();

	gluLookAt(0,0,-RADIUS*2.5, 0,0,0, 0,1,0);
	glRotatef(elevation, 1, 0, 0);
	glRotatef(bearing, 0, 1, 0);

	glViewport(0,0,width/3, height/3);
	glScissor(0,0,width/3,height/3);
	glClearColor(.3,.3,.3,1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glDisable(GL_CULL_FACE);

	quadtree_render(qt, set_texture);
	glPopMatrix();
	glPopAttrib();

	glutSwapBuffers();

	//usleep(10000);
	//glutPostRedisplay();
}

static void keydown(unsigned char key, int x, int y)
{
	switch(key) {
	case 'o':
		update_view = !update_view;
		break;

	case 'd':
		wireframe = !wireframe;
		break;

	case 'x':
	case 27:
		exit(0);
	}

	glutPostRedisplay();
}

static void keyup(unsigned char key, int x, int y)
{
	switch(key) {
	case 'o':
		break;
	}
}

static int drag = 0, spin = 0;
static int lastx = 0, lasty = 0;

static void move_dolly(float delta)
{
	dolly += delta * RADIUS/1024;
	if (dolly < RADIUS * 1.01)
		dolly = RADIUS * 1.05;

	//printf("dolly = %g\n", dolly);
}

static float base_elevation, base_bearing;

static void spin_start(int x, int y)
{
	spin = 1;

	lastx = x;
	lasty = y;

	base_elevation = elevation;
	base_bearing = bearing;
}

static void spin_stop(void)
{
	spin = 0;
}

static void motion(int x, int y)
{
	if (drag) {
		move_dolly(y - lasty);
		lasty = y;
	} else if (spin) {
		elevation = base_elevation + (y - lasty) * 360 / height;
		bearing = base_bearing + (x - lastx) * 360 / width;
	}

	glutPostRedisplay();
}

static void mouse(int buttons, int state, int x, int y)
{
	if (state != GLUT_DOWN) {
		spin = drag = 0;
		return;
	}

	switch (buttons) {
	case 4:
		move_dolly(20);
		break;

	case 3:
		move_dolly(-20);
		break;

	case GLUT_LEFT_BUTTON:
		spin_start(x, y);
		break;

	case GLUT_MIDDLE_BUTTON:
		drag = 1;
		lasty = y;
		break;
	}

	glutPostRedisplay();
}

#if LABELS
static const GLubyte gradient[] = {
	0x06, 0x1d, 0x98,
	0x07, 0x1e, 0x99,
	0x08, 0x1e, 0x99,
	0x08, 0x1f, 0x99,
	0x09, 0x1f, 0x9a,
	0x09, 0x20, 0x9a,
	0x0a, 0x20, 0x9b,
	0x0b, 0x21, 0x9b,
	0x0a, 0x21, 0x9b,
	0x0b, 0x21, 0x9b,
	0x0c, 0x22, 0x9c,
	0x0c, 0x23, 0x9d,
	0x0c, 0x23, 0x9d,
	0x0d, 0x24, 0x9d,
	0x0e, 0x24, 0x9e,
	0x0e, 0x25, 0x9e,
	0x0f, 0x25, 0x9f,
	0x0f, 0x25, 0x9f,
	0x0f, 0x26, 0x9f,
	0x10, 0x26, 0x9f,
	0x11, 0x26, 0xa0,
	0x11, 0x28, 0xa0,
	0x12, 0x27, 0xa1,
	0x12, 0x28, 0xa2,
	0x13, 0x29, 0xa2,
	0x14, 0x29, 0xa3,
	0x14, 0x2a, 0xa3,
	0x14, 0x2a, 0xa3,
	0x14, 0x2b, 0xa3,
	0x15, 0x2b, 0xa4,
	0x16, 0x2b, 0xa5,
	0x17, 0x2c, 0xa5,
	0x17, 0x2d, 0xa5,
	0x17, 0x2d, 0xa6,
	0x18, 0x2d, 0xa6,
	0x19, 0x2e, 0xa7,
	0x19, 0x2f, 0xa6,
	0x1a, 0x2f, 0xa7,
	0x1a, 0x2f, 0xa7,
	0x1b, 0x30, 0xa8,
	0x1b, 0x31, 0xa9,
	0x1c, 0x31, 0xa9,
	0x1c, 0x31, 0xa9,
	0x1c, 0x32, 0xa9,
	0x1d, 0x32, 0xaa,
	0x1e, 0x33, 0xab,
	0x1e, 0x33, 0xab,
	0x1e, 0x34, 0xab,
	0x20, 0x35, 0xac,
	0x20, 0x35, 0xac,
	0x20, 0x36, 0xad,
	0x21, 0x36, 0xad,
	0x21, 0x36, 0xae,
	0x22, 0x36, 0xad,
	0x23, 0x37, 0xae,
	0x23, 0x38, 0xae,
	0x24, 0x38, 0xaf,
	0x24, 0x38, 0xb0,
	0x24, 0x39, 0xb0,
	0x25, 0x3a, 0xb0,
	0x25, 0x3a, 0xb0,
	0x26, 0x3a, 0xb1,
	0x26, 0x3a, 0xb2,
	0x27, 0x3c, 0xb2,
	0x28, 0x3c, 0xb2,
	0x28, 0x3d, 0xb3,
	0x29, 0x3d, 0xb3,
	0x29, 0x3d, 0xb4,
	0x29, 0x3e, 0xb4,
	0x2a, 0x3f, 0xb5,
	0x2b, 0x3f, 0xb5,
	0x2b, 0x3f, 0xb6,
	0x2c, 0x40, 0xb6,
	0x2c, 0x41, 0xb6,
	0x2c, 0x41, 0xb7,
	0x2d, 0x41, 0xb7,
	0x2e, 0x41, 0xb8,
	0x2e, 0x43, 0xb8,
	0x2f, 0x43, 0xb8,
	0x2f, 0x43, 0xb8,
	0x2f, 0x44, 0xb9,
	0x31, 0x44, 0xb9,
	0x30, 0x45, 0xb9,
	0x31, 0x45, 0xba,
	0x32, 0x46, 0xba,
	0x32, 0x46, 0xbb,
	0x33, 0x47, 0xbc,
	0x33, 0x47, 0xbc,
	0x34, 0x47, 0xbc,
	0x35, 0x48, 0xbc,
	0x35, 0x48, 0xbd,
	0x35, 0x49, 0xbe,
	0x35, 0x49, 0xbe,
	0x37, 0x4a, 0xbe,
	0x37, 0x4b, 0xbf,
	0x38, 0x4b, 0xbf,
	0x37, 0x4b, 0xbf,
	0x38, 0x4c, 0xc0,
	0x39, 0x4c, 0xc0,
	0x39, 0x4d, 0xc0,
	0x3a, 0x4e, 0xc1,
	0x3a, 0x4e, 0xc2,
	0x3b, 0x4e, 0xc2,
	0x3b, 0x4f, 0xc2,
	0x3c, 0x4f, 0xc3,
	0x3c, 0x50, 0xc3,
	0x3d, 0x50, 0xc4,
	0x3d, 0x51, 0xc4,
	0x3e, 0x51, 0xc4,
	0x3f, 0x51, 0xc5,
	0x3f, 0x52, 0xc5,
	0x3f, 0x53, 0xc6,
	0x40, 0x53, 0xc6,
	0x40, 0x53, 0xc7,
	0x41, 0x54, 0xc7,
	0x42, 0x55, 0xc7,
	0x42, 0x54, 0xc8,
	0x43, 0x55, 0xc8,
	0x43, 0x55, 0xc8,
	0x43, 0x56, 0xc9,
	0x44, 0x57, 0xc9,
	0x44, 0x58, 0xca,
	0x45, 0x58, 0xca,
	0x46, 0x58, 0xcb,
	0x48, 0x5a, 0xcd,
	0x4d, 0x5f, 0xd0,
	0x53, 0x65, 0xd5,
	0x58, 0x69, 0xd9,
	0x5d, 0x6e, 0xdd,
	0x62, 0x73, 0xe1,
	0x67, 0x78, 0xe5,
	0x6d, 0x7d, 0xea,
	0x71, 0x81, 0xee,
	0x77, 0x86, 0xf2,
	0x7c, 0x8b, 0xf6,
	0x81, 0x90, 0xfa,
	0x87, 0x95, 0xff,
	0x8b, 0x99, 0xfa,
	0x8e, 0x9c, 0xf4,
	0x92, 0x9f, 0xee,
	0x97, 0xa3, 0xe7,
	0x9b, 0xa6, 0xe2,
	0x9e, 0xa9, 0xdc,
	0xa2, 0xad, 0xd6,
	0xa7, 0xb0, 0xd0,
	0xaa, 0xb4, 0xca,
	0xaf, 0xb7, 0xc4,
	0xb3, 0xba, 0xbe,
	0xb6, 0xbe, 0xb8,
	0xbb, 0xc1, 0xb3,
	0xbe, 0xc5, 0xad,
	0xc2, 0xc9, 0xa7,
	0xd6, 0xda, 0x89,
	0xee, 0xed, 0x66,
	0xf6, 0xf6, 0x4a,
	0xcf, 0xe0, 0x42,
	0xa8, 0xca, 0x3a,
	0x89, 0xb9, 0x34,
	0x85, 0xb6, 0x34,
	0x81, 0xb4, 0x33,
	0x7e, 0xb1, 0x32,
	0x7a, 0xaf, 0x31,
	0x75, 0xad, 0x31,
	0x71, 0xab, 0x2f,
	0x6d, 0xa8, 0x2f,
	0x69, 0xa6, 0x2d,
	0x66, 0xa4, 0x2d,
	0x62, 0xa1, 0x2d,
	0x5d, 0x9f, 0x2b,
	0x59, 0x9d, 0x2b,
	0x55, 0x9a, 0x29,
	0x51, 0x98, 0x28,
	0x4d, 0x96, 0x28,
	0x49, 0x94, 0x27,
	0x46, 0x91, 0x26,
	0x41, 0x90, 0x26,
	0x3e, 0x8c, 0x24,
	0x39, 0x8a, 0x24,
	0x35, 0x88, 0x23,
	0x32, 0x86, 0x23,
	0x2d, 0x84, 0x21,
	0x2a, 0x81, 0x21,
	0x25, 0x7f, 0x20,
	0x22, 0x7d, 0x1f,
	0x1e, 0x7b, 0x1f,
	0x19, 0x78, 0x1e,
	0x16, 0x76, 0x1d,
	0x19, 0x77, 0x20,
	0x1e, 0x78, 0x25,
	0x23, 0x78, 0x29,
	0x27, 0x7a, 0x2d,
	0x2b, 0x7a, 0x31,
	0x2f, 0x7b, 0x36,
	0x34, 0x7c, 0x3a,
	0x38, 0x7d, 0x3e,
	0x3d, 0x7e, 0x42,
	0x41, 0x7f, 0x46,
	0x45, 0x80, 0x4a,
	0x4a, 0x81, 0x4e,
	0x4f, 0x82, 0x52,
	0x53, 0x83, 0x57,
	0x58, 0x84, 0x5c,
	0x5d, 0x84, 0x60,
	0x63, 0x86, 0x66,
	0x68, 0x86, 0x6a,
	0x6d, 0x88, 0x6f,
	0x72, 0x89, 0x73,
	0x78, 0x8a, 0x79,
	0x7c, 0x8b, 0x7e,
	0x82, 0x8d, 0x83,
	0x87, 0x8d, 0x88,
	0x8c, 0x8f, 0x8d,
	0x90, 0x90, 0x90,
	0x93, 0x93, 0x93,
	0x95, 0x96, 0x95,
	0x98, 0x98, 0x98,
	0x9b, 0x9b, 0x9b,
	0x9d, 0x9d, 0x9d,
	0x9f, 0x9f, 0xa0,
	0xa2, 0xa2, 0xa2,
	0xa5, 0xa4, 0xa5,
	0xa7, 0xa7, 0xa7,
	0xaa, 0xaa, 0xaa,
	0xac, 0xac, 0xac,
	0xaf, 0xaf, 0xaf,
	0xb1, 0xb1, 0xb1,
	0xb4, 0xb4, 0xb4,
	0xb6, 0xb6, 0xb7,
	0xb9, 0xb9, 0xb9,
	0xbb, 0xbb, 0xbc,
	0xbe, 0xbe, 0xbe,
	0xc0, 0xc1, 0xc0,
	0xc3, 0xc2, 0xc3,
	0xc5, 0xc5, 0xc6,
	0xc8, 0xc8, 0xc7,
	0xca, 0xcb, 0xcb,
	0xcd, 0xcd, 0xcd,
	0xd0, 0xd0, 0xd0,
	0xd3, 0xd2, 0xd3,
	0xd5, 0xd5, 0xd5,
	0xd8, 0xd7, 0xd8,
	0xda, 0xda, 0xda,
	0xdc, 0xdd, 0xdc,
	0xdf, 0xdf, 0xdf,
	0xe1, 0xe2, 0xe2,
	0xe4, 0xe4, 0xe4,
	0xe6, 0xe6, 0xe6,
	0xe9, 0xe9, 0xe9,
	0xec, 0xeb, 0xeb,
	0xed, 0xed, 0xed,
	0xef, 0xef, 0xef,
	0xf2, 0xf1, 0xf1,
	0xf3, 0xf2, 0xf2,
	0xf4, 0xf4, 0xf4,
	0xf5, 0xf4, 0xf4,
	0xf4, 0xf5, 0xf4,
};

static elevation_t generate(const vec3_t *v, struct vertex *vtx)
{
	float height;
	elevation_t e;
	int idx;
	vec3_t nv = *v;

	vec3_normalize(&nv);

	height = fractal_fBmtest(frac, nv.v, 8);

	//printf("height(%g, %g, %g) = %g, variance=%g\n", v[0], v[1], v[2], height, variance);
	idx = ((height * .5f) + .5f) * 255;

	e = height * variance + offset;

	if (idx < 0)
		idx = 0;
	if (idx > 255)
		idx = 255;

	unsigned char col[] = { gradient[idx * 3 + 0],
				gradient[idx * 3 + 1],
				gradient[idx * 3 + 2],
				0 };
	vertex_set_colour(vtx, col);

	return e;
}
#else
static elevation_t generate(const vec3_t *v, struct vertex *vtx)
{
	float height;
	elevation_t e;
	vec3_t nv = *v;

	vec3_normalize(&nv);

	height = fractal_fBmtest(frac, nv.v, 8);

	//printf("height(%g, %g, %g) = %g, variance=%g\n", v[0], v[1], v[2], height, variance);
	e = height * variance + offset;

	texcoord_t s = e * 16384 / maxvariance;
	texcoord_t t = (fabsf(v->z) + .1f * fractal_fBm(frac, v->v, 4)) * 32767;

	if (0)
		printf("set texcoord(%g,%g,%g) = st=(%d,%d), maxvariance=%g\n",
		       v->x, v->y, v->z, s, t, maxvariance);

	vertex_set_texcoord(vtx, s, t);

	return e;
}
#endif	/* LABELS */

int main(int argc, char **argv)
{
	float _variance = RADIUS * .03;
	float _roughness = .1;

	frac = fractal_create(3, 210, 1.0 - _roughness, 5);
	
	maxvariance = _variance;
	variance = maxvariance / .75f;
	offset = -maxvariance + .75f * variance;

	printf("maxvariance=%g variance=%g offset=%g\n",
	       maxvariance, variance, offset);

	glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
        glutInitWindowSize(480*2, 272*2);
	glutCreateWindow( __FILE__ );

	qt = quadtree_create(500, RADIUS, generate);
	
	//glutSpecialFunc(special_down);
	glutKeyboardFunc(keydown);
	glutKeyboardUpFunc(keyup);
	glutReshapeFunc(reshape);
	glutDisplayFunc(display);
	//glutPassiveMotionFunc(motion);
	glutMotionFunc(motion);
	glutMouseFunc(mouse);

	{
		GLfloat diffcol0[] = { .4, .4, 1, 1 };
		GLfloat lightdir0[] = { 0, 0, 1, 0 };
		GLfloat diffcol1[] = { 1, .9, .6, 1 };
		GLfloat lightdir1[] = { -1, 0, .5, 0 };

		glEnable(GL_LIGHTING);

		//glEnable(GL_LIGHT0);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, diffcol0);
		glLightfv(GL_LIGHT0, GL_POSITION, lightdir0);

		glEnable(GL_LIGHT1);
		glLightfv(GL_LIGHT1, GL_DIFFUSE, diffcol1);
		glLightfv(GL_LIGHT1, GL_POSITION, lightdir1);
	}
	GLERROR();

	texture = buildtexture(variance);
	GLERROR();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glutMainLoop();
}
