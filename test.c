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

float elevation, bearing;

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

static void project_to_sphere(int radius, int x, int y, int z)
{
	float xf = (float)x;
	float yf = (float)y;
	float zf = (float)z;

	float len = sqrtf(xf*xf + yf*yf + zf*zf);

	glVertex3f(xf * radius / len,
		   yf * radius / len,
		   zf * radius / len);
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

static void draw_patch(const struct patch *p, int culled)
{
	if (culled) 
		glColor3f(.4,.4,.4);
	else
		glColor4ub(p->col[0], p->col[1], p->col[2], .7*255);

	if (0)
		printf("p=%p id=%lu x=(%d,%d) y=(%d,%d), z=(%d,%d)\n",
		       p, p->id, p->x0, p->x1, p->y0, p->y1, p->z0, p->z1);

#if 1
	set_texture(p);
#else
	printf("generating tex for p=%p %lu %s\n",
	       p, p->id, id2str(p));
	texprintf("%s", id2str(p));
#endif

	glBegin(GL_QUADS);

	if (p->z0 == p->z1 || /* top, bottom */
	    p->y0 == p->y1) { /* front, back */
		glTexCoord2f(0,1);
		project_to_sphere(RADIUS, p->x0, p->y0, p->z0);
		glTexCoord2f(1,1);
		project_to_sphere(RADIUS, p->x1, p->y0, p->z0);
		glTexCoord2f(1,0);
		project_to_sphere(RADIUS, p->x1, p->y1, p->z1);
		glTexCoord2f(0,0);
		project_to_sphere(RADIUS, p->x0, p->y1, p->z1);
	} else if (p->x0 == p->x1) { /* left, right */
		glTexCoord2f(0,1);
		project_to_sphere(RADIUS, p->x0, p->y0, p->z0);
		glTexCoord2f(1,1);
		project_to_sphere(RADIUS, p->x0, p->y1, p->z0);
		glTexCoord2f(1,0);
		project_to_sphere(RADIUS, p->x0, p->y1, p->z1);
		glTexCoord2f(0,0);
		project_to_sphere(RADIUS, p->x0, p->y0, p->z1);
	}
	glEnd();

	GLERROR();
}

static void draw()
{
	glBegin(GL_LINES);
	  glColor3f(1,0,0);
	  glVertex3i(-2000, 0, 0);
	  glColor3f(1,1,1);
	  glVertex3i( 2000, 0, 0);

	  glColor3f(0,1,0);
	  glVertex3i(0, -2000, 0);
	  glColor3f(1,1,1);
	  glVertex3i(0,  2000, 0);

	  glColor3f(0,0,1);
	  glVertex3i(0, 0, -2000);
	  glColor3f(1,1,1);
	  glVertex3i(0, 0,  2000);
	glEnd();

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glDisable(GL_BLEND);
	GLERROR();

	struct list_head *pp;
	list_for_each(pp, &qt->culled) {
		struct patch *p = list_entry(pp, struct patch, list);

		draw_patch(p, 1);
	}

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	GLERROR();
	list_for_each(pp, &qt->visible) {
		struct patch *p = list_entry(pp, struct patch, list);

		draw_patch(p, 0);
	}
}

static float delta = 1;

static float dolly = -RADIUS * 2.5;
static void display()
{
	static float angle;
	static int frame;

	glClearColor(.2,.2,.2,1);
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


	if (1 || frame++ % 5 == 0) {
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

#if 0
	draw();
#else
	glColor3f(1,1,1);
	//glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);
	glDisable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glPolygonMode(GL_FRONT, GL_LINE);
	GLERROR();

	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();
	glScalef(1./16, 1./16, 1);
	glMatrixMode(GL_MODELVIEW);

	quadtree_render(qt, set_texture);
#endif

	glutSwapBuffers();

	usleep(10000);
	glutPostRedisplay();
}

static void keydown(unsigned char key, int x, int y)
{
	switch(key) {
	case 'o':
		delta = 0;
		break;

	case 'x':
	case 27:
		exit(0);
	}
}

static void keyup(unsigned char key, int x, int y)
{
	switch(key) {
	case 'o':
		delta = 1;
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
}

static elevation_t generate(long x, long y, long z)
{
	//printf("x=%ld y=%ld z=%ld\n", x, y, z);

	return (cos((float) x * M_PI * 2 / 1000) + 
		sin((float) y * M_PI * 2 / 2000) +
		sin((float) z * M_PI * 2 / 800)*1.2)* 20;
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
