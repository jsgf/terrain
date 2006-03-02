#include <math.h>
#include <stdlib.h>
#include <assert.h>

#include "noise.h"

#define EPSILON	(1e-6f)

struct noise {
	unsigned ndim;
	unsigned char map[256];
	float buffer[256][MAX_DIMENSIONS];
};

struct fractal {
	struct noise noise;

	float H;
	float lacunarity;
	float exponent[MAX_OCTAVES];
};

static void normalize(float *f, int n)
{
	float mag = 0;

	for(int i = 0; i < n; i++)
		mag += f[i] * f[i];

	mag = 1 / sqrtf(mag);
	for(int i = 0; i < n; i++)
		f[i] *= mag;
}

static float lerp(float a, float b, float x)
{
	//printf("lerp(%g, %g, %g) = %g\n", a, b, x, (a + x * (b-a)));
	return a + x * (b - a);
}

static float clamp(float min, float max, float f)
{
	f = (f < min) ? min : f;		
	f = (f < max) ? f : max;

	return f;
}

static float cubic(float a)
{
	return a * a * (3 - 2*a);
}

struct noise *noise_create(int ndim, unsigned int seed)
{
	struct noise *n = malloc(sizeof(*n));

	noise_init(n, ndim, seed);

	return n;
}

void noise_init(struct noise *n, int ndim, unsigned int seed)
{
	if (ndim > MAX_DIMENSIONS)
		ndim = MAX_DIMENSIONS;

	n->ndim = ndim;

	random_init(seed);

	for(int i = 0; i < 256; i++) {
		n->map[i] = i;
		for(int j = 0; j < ndim; j++)
			n->buffer[i][j] = random_range(-0.5f, 0.5f);
		normalize(n->buffer[i], ndim);
	}

	for(int i = 0; i < 256; i++) {
		int j = random_irange(0, 255);
		int t = n->map[i];
		n->map[i] = n->map[j];
		n->map[j] = t;
	}
}

static float lattice4(const struct noise *noise,
		     int ix, float fx,
		     int iy, float fy,
		     int iz, float fz,
		     int iw, float fw)
{
	int n[] = { ix, iy, iz, iw };
	float f[] = { fx, fy, fz, fw };
	int index = 0;

	assert(noise->ndim == 4);

	for(int i = 0; i < 4; i++)
		index = noise->map[(index + n[i]) % 256];

	float value = 0;
	for(int i = 0; i < 4; i++)
		value += noise->buffer[index][i] * f[i];

	return value;
}

static float lattice1(const struct noise *noise,
		      int ix, float fx)
{
	unsigned index = 0;

	assert(noise->ndim == 1);

	index = noise->map[ix % 256];
	float value = noise->buffer[index][0] * fx;

	return value;
}

static float lattice2(const struct noise *noise,
		      int ix, float fx,
		      int iy, float fy)
{
	int n[] = { ix, iy };
	float f[] = { fx, fy };
	unsigned index = 0;

	assert(noise->ndim == 2);

	for(int i = 0; i < 2; i++)
		index = noise->map[(index + n[i]) % 256];

	float value = 0;
	for(int i = 0; i < 2; i++)
		value += noise->buffer[index][i] * f[i];

	return value;
}

static float lattice3(const struct noise *noise,
		      int ix, float fx,
		      int iy, float fy,
		      int iz, float fz)
{
	int n[] = { ix, iy, iz };
	float f[] = { fx, fy, fz };
	unsigned index = 0;

	assert(noise->ndim == 3);
	for(int i = 0; i < 3; i++)
		index = noise->map[(index + n[i]) % 256];

	float value = 0;
	for(int i = 0; i < 3; i++)
		value += noise->buffer[index][i] * f[i];

	return value;
}

float noise_gen(const struct noise *noise, float *f)
{
	int n[MAX_DIMENSIONS];
	float r[MAX_DIMENSIONS];
	float w[MAX_DIMENSIONS];

	for(int i = 0; i < noise->ndim; i++) {
		n[i] = floor(f[i]);
		r[i] = f[i] - n[i];
		w[i] = cubic(r[i]);
	}

	float value;
	switch(noise->ndim) {
	case 1:
		value = lerp(lattice1(noise, n[0]  , r[0]  ),
			     lattice1(noise, n[0]+1, r[0]+1),
			     w[0]);
		break;

	case 2:
		value = lerp(lerp(lattice2(noise, n[0], r[0], n[1], r[1]),
				  lattice2(noise, n[0]+1, r[0]-1, n[1], r[1]),
				  w[0]),
			     lerp(lattice2(noise, n[0], r[0], n[1]+1, r[1]-1),
				  lattice2(noise, n[0]+1, r[0]-1, n[1]+1, r[1]-1),
				  w[0]),
			     w[1]);
		break;

	case 3:
		//printf("noise3 start\n");
		value = lerp(lerp(lerp(lattice3(noise, n[0]  , r[0]  , n[1], r[1], n[2], r[2]),
				       lattice3(noise, n[0]+1, r[0]-1, n[1], r[1], n[2], r[2]),
				       w[0]),
				  lerp(lattice3(noise, n[0]  , r[0]  , n[1]+1, r[1]-1, n[2], r[2]),
				       lattice3(noise, n[0]+1, r[0]-1, n[1]+1, r[1]-1, n[2], r[2]),
				       w[0]),
				  w[1]),
			     lerp(lerp(lattice3(noise, n[0]  , r[0]  , n[1], r[1], n[2]+1, r[2]-1),
				       lattice3(noise, n[0]+1, r[0]-1, n[1], r[1], n[2]+1, r[2]-1),
				       w[0]),
				  lerp(lattice3(noise, n[0]  , r[0]  , n[1]+1, r[1]-1, n[2]+1, r[2]-1),
				       lattice3(noise, n[0]+1, r[0]-1, n[1]+1, r[1]-1, n[2]+1, r[2]-1),
				       w[0]),
				  w[1]),
			     w[2]);
		//printf("noise3=%g\n", value);
		break;

	case 4:
		value = lerp(lerp(lerp(lerp(lattice4(noise, n[0], r[0], n[1], r[1], n[2], r[2], n[3], r[3]),
					    lattice4(noise, n[0]+1, r[0]-1, n[1], r[1], n[2], r[2], n[3], r[3]),
					    w[0]),
				       lerp(lattice4(noise, n[0], r[0], n[1]+1, r[1]-1, n[2], r[2], n[3], r[3]),
					    lattice4(noise, n[0]+1, r[0]-1, n[1]+1, r[1]-1, n[2], r[2], n[3], r[3]),
					    w[0]),
				       w[1]),
				  lerp(lerp(lattice4(noise, n[0], r[0], n[1], r[1], n[2]+1, r[2]-1, n[3], r[3]),
					    lattice4(noise, n[0]+1, r[0]-1, n[1], r[1], n[2]+1, r[2]-1, n[3], r[3]),
					    w[0]),
				       lerp(lattice4(noise, n[0], r[0], n[1]+1, r[1]-1, n[2]+1, r[2]-1, n[3], r[3]),
					    lattice4(noise, n[0]+1, r[0]-1, n[1]+1, r[1]-1, n[2]+1, r[2]-1, n[3], r[3]),
					    w[0]),
				       w[1]),
				  w[2]),
			     lerp(lerp(lerp(lattice4(noise, n[0], r[0], n[1], r[1], n[2], r[2], n[3]+1, r[3]-1),
					    lattice4(noise, n[0]+1, r[0]-1, n[1], r[1], n[2], r[2], n[3]+1, r[3]-1),
					    w[0]),
				       lerp(lattice4(noise, n[0], r[0], n[1]+1, r[1]-1, n[2], r[2], n[3]+1, r[3]-1),
					    lattice4(noise, n[0]+1, r[0]-1, n[1]+1, r[1]-1, n[2], r[2], n[3]+1, r[3]-1),
					    w[0]),
				       w[1]),
				  lerp(lerp(lattice4(noise, n[0], r[0], n[1], r[1], n[2]+1, r[2]-1, n[3]+1, r[3]-1),
					    lattice4(noise, n[0]+1, r[0]-1, n[1], r[1], n[2]+1, r[2]-1, n[3]+1, r[3]-1),
					    w[0]),
				       lerp(lattice4(noise, n[0], r[0], n[1]+1, r[1]-1, n[2]+1, r[2]-1, n[3]+1, r[3]-1),
					    lattice4(noise, n[0]+1, r[0]-1, n[1]+1, r[1]-1, n[2]+1, r[2]-1, n[3]+1, r[3]-1),
					    w[0]),
				       w[1]),
				  w[2]),
			     w[3]);
		break;

	default:
		abort();
	}

	return clamp(-0.99999f, 0.99999f, value);
}

void random_init(unsigned int seed)
{
	srand(seed);
}

float random_gen(void)
{
	return (float)rand() / (float)RAND_MAX;
}

float random_range(float min, float max)
{
	float interval = max - min;
	float d = interval * random_gen();
	return min + (d < interval ? d : interval);
}

unsigned int random_irange(unsigned int min, unsigned int max)
{
	unsigned interval = max - min;
	unsigned i = (interval + 1.0f) * random_gen();
	return min + (i < interval ? i : interval);
}

struct fractal *fractal_create(int ndim, unsigned int seed, float H, float lacunarity)
{
	struct fractal *f = malloc(sizeof(*f));

	fractal_init(f, ndim, seed, H, lacunarity);

	return f;
}

void fractal_init(struct fractal *frac, int ndim, unsigned int seed,
		  float H, float lacunarity)
{
	noise_init(&frac->noise, ndim, seed);

	frac->H = H;
	frac->lacunarity = lacunarity;

	float f = 1;
	for(int i = 0; i < MAX_OCTAVES; i++) {
		frac->exponent[i] = powf(f, -H);
		f *= lacunarity;
	}
}

float fractal_fBm(const struct fractal *frac, const float *f, float octaves)
{
	float value = 0;
	float tmp[MAX_DIMENSIONS];
	for(int i = 0; i < frac->noise.ndim; i++)
		tmp[i] = f[i];

	int i;
	for(i = 0; i < octaves; i++) {
		value += noise_gen(&frac->noise, tmp) * frac->exponent[i];
		for(int j = 0; j < frac->noise.ndim; j++)
			tmp[j] *= frac->lacunarity;
	}

	octaves -= (int)octaves;
	if (octaves > EPSILON)
		value += octaves * noise_gen(&frac->noise, tmp) * frac->exponent[i];

	return clamp(-0.99999f, 0.99999, value);
}

float fractal_fBmtest(const struct fractal *frac, const float *f, float octaves)
{
	float value = 0;
	float tmp[MAX_DIMENSIONS];
	for(int i = 0; i < frac->noise.ndim; i++)
		tmp[i] = f[i] * 2;

	int i;
	for(i = 0; i < octaves; i++) {
		value += noise_gen(&frac->noise, tmp) * frac->exponent[i];
		for(int j = 0; j < frac->noise.ndim; j++)
			tmp[j] *= frac->lacunarity;
	}

	octaves -= (int)octaves;
	if (octaves > EPSILON)
		value += octaves * noise_gen(&frac->noise, tmp) * frac->exponent[i];

	if (value < 0.f)
		return -powf(-value, 0.7f);
	return powf(value, 1 + noise_gen(&frac->noise, tmp) * value);
}
