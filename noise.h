#ifndef _NOISE_H
#define _NOISE_H

#define MAX_DIMENSIONS	4	// Maximum number of dimensions in a noise object
#define MAX_OCTAVES	128	// Maximum # of octaves in an fBm object

struct noise;

struct noise *noise_create(int ndim, unsigned int seed);
void noise_init(struct noise *n, int ndim, unsigned int seed);
float noise_gen(const struct noise *, float *);

void random_init(unsigned int seed);
float random_gen(void);
float random_range(float min, float max);
unsigned int random_irange(unsigned int min, unsigned int max);


struct fractal;

struct fractal *fractal_create(int ndim, unsigned int seed, float H, float lacunarity);
void fractal_init(struct fractal *f, int ndim, unsigned int seed,
		  float H, float lacunarity);
float fractal_fBm(const struct fractal *frac, const float *f, float octaves);
float fractal_turbulence(const struct fractal *frac, float *f, float octaves);
float fractal_multifractal(const struct fractal *frac, float *f,
			   float octaves, float offset);
float fractal_heterofractal(const struct fractal *frac, float *f, 
			    float octaves, float offset);
float fractal_hybrid_multifractal(const struct fractal *frac, float *f,
				  float octaves, float offset, float gain);
float fractal_ridged_multifractal(const struct fractal *frac, float *f,
				  float octaves, float offset, float thresh);
float fractal_fBmtest(const struct fractal *frac, const float *f, float octaves);

#endif	/* _NOISE_H */
