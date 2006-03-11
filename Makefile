CFLAGS=-Wall -g -std=gnu99 # -O4 -msse -msse2 -mfpmath=sse

test: test.o quadtree.o patchidx.o noise.o geom.o gentexture.o
	$(CC) -o $@ test.o quadtree.o patchidx.o noise.o geom.o gentexture.o -lglut

test.o: quadtree.h font.h noise.h geom.h
quadtree.o: quadtree.h quadtree_priv.h geom.h
noise.o: noise.h
geom.o: geom.h

font.h: msx
	./msx > font.h

patchidx.c: genpatchidx
	genpatchidx > patchidx.c

genpatchidx.o patchidx.o: quadtree.h

clean:
	rm -f font.h msx test *.o *.dot *.ps *~ core

%.ps: %.dot
	dot -Tps $< > $@

dotps: $(addsuffix .ps,$(basename $(wildcard *.dot)))
