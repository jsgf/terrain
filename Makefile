CFLAGS=-Wall -g -std=gnu99

test: test.o quadtree.o patchidx.o
	$(CC) -o $@ test.o quadtree.o patchidx.o -lglut

test.o: quadtree.h font.h
quadtree.o: quadtree.h

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
