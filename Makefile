# 
# For lazy guys, really...
#

CFLAGS= -Wall

.PHONY: doc

all:
	gcc $(CFLAGS) src/cats.c -o cats

doc:
	doxygen doc/Doxyfile

clean:
	rm -f cats
	rm -rf doc/{latex,html}
