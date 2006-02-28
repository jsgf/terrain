#include <GL/glut.h>
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "quadtree.h"
#include "font.h"

#define RADIUS (1<<20)

static struct quadtree *qt;

static float elevation, bearing;
static int wireframe = 0;
static int update_view = 1;

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

static const char *id2str(const struct patch *p)
{
	int id = p->id;
	int level = p->level;
	static char buf[40];
	char *cp = buf;

	//cp += sprintf(cp, "%d:", level);
	*cp++ = '>';
	cp += sprintf(cp, "%d:", id >> (level * 2));

	for(int i = level-1; i >= 0; i--) {
		cp += sprintf(cp, "%d.", (id >> (i * 2)) & 3);
	}
	cp[-1] = '\0';

	return buf;
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
	gluPerspective(50., 16./9., RADIUS/100, RADIUS*4);

	glMatrixMode(GL_MODELVIEW);
}

static void set_texture(const struct patch *p)
{
	GLuint texid = (p->id+1) + (1 << (p->level * 2 + 4));

	if (!glIsTexture(texid)) {
		const char *s = id2str(p);
		if (0)
			printf("generating tex %u for p=%p %d:%lu %s\n",
			       texid, p, p->level,p->id, s);

		glBindTexture(GL_TEXTURE_2D, texid);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		texprintf("%s", s);
	} else
		glBindTexture(GL_TEXTURE_2D, texid);
}

static float delta = 1;

static float dolly = -RADIUS * 2.5;
static void display()
{
	static float angle;

	glClearColor(.2,.2,.2,1);
	glDepthMask(GL_TRUE);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//gluLookAt(500,-3000,1100, 0,0,0, 0,0,1);
	glTranslatef(0, 0, dolly);
	glRotatef(elevation, 1, 0, 0);
	glRotatef(bearing, 0, 1, 0);
	//glRotatef(angle, 0, 1, 1);
	angle += delta;
	GLERROR();


	if (update_view) {
		GLfloat mv[16], proj[16];
		GLint viewport[4];

		glGetFloatv(GL_MODELVIEW_MATRIX, mv);
		GLERROR();
		glGetFloatv(GL_PROJECTION_MATRIX, proj);
		GLERROR();
		glGetIntegerv(GL_VIEWPORT, viewport);
		GLERROR();

		quadtree_update_view(qt, mv, proj, viewport);
	}


	glEnable(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	GLERROR();
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_BLEND);
	GLERROR();

	glColor3f(1,1,1);
	glEnable(GL_CULL_FACE);
	if (wireframe) {
		glPolygonMode(GL_FRONT, GL_LINE);
		glDisable(GL_LIGHTING);
	} else {
		glPolygonMode(GL_FRONT, GL_FILL);
		glEnable(GL_LIGHTING);
	}

	//glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	GLERROR();

	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();
	glScalef(1./PATCH_SAMPLES, 1./PATCH_SAMPLES, 1);
	glMatrixMode(GL_MODELVIEW);

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
static int lasty = 0;

static void motion(int x, int y)
{
	if (drag) {
		dolly += (y - lasty) * RADIUS/1024;
		if (dolly > -RADIUS * 1.01)
			dolly = -RADIUS * 1.01;

		//printf("dolly = %g\n", dolly);
		lasty = y;
	} else if (spin) {
		elevation = y * 360 / height;
		bearing = x * 360 / width;
	}

	glutPostRedisplay();
}

static void mouse(int buttons, int state, int x, int y)
{
	if (state != GLUT_DOWN) {
		spin = drag = 0;
		return;
	}

	if (buttons == GLUT_LEFT_BUTTON)
		spin = 1;
	else if (buttons == GLUT_MIDDLE_BUTTON) {
		lasty = y;
		drag = 1;
	}

	glutPostRedisplay();
}

static elevation_t generate(long x, long y, long z)
{
	//printf("x=%ld y=%ld z=%ld\n", x, y, z);

	return (cos((float) x * M_PI * 2 / RADIUS) + 
		sin((float) y * M_PI * 2 / (RADIUS*2)) +
		sin((float) z * M_PI * 2 / (RADIUS*.8))*1.2)* (RADIUS * .02);
}

int main(int argc, char **argv)
{
	glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
        glutInitWindowSize(480*2, 272*2);
	glutCreateWindow( __FILE__ );

	qt = quadtree_create(200, RADIUS, generate);
	
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
		GLfloat diffcol1[] = { .6, .2, .1, 1 };
		GLfloat lightdir1[] = { .707, .707, 0, 0 };

		glEnable(GL_LIGHTING);

		glEnable(GL_LIGHT0);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, diffcol0);
		glLightfv(GL_LIGHT0, GL_POSITION, lightdir0);

		glEnable(GL_LIGHT1);
		glLightfv(GL_LIGHT1, GL_DIFFUSE, diffcol1);
		glLightfv(GL_LIGHT1, GL_POSITION, lightdir1);
	}

	glutMainLoop();
}
