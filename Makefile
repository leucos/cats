# 
# For lazy guys, really...
#


.PHONY: doc

all:
	gcc src/cats.c -o cats

doc:
	doxygen doc/Doxyfile

clean:
	rm -f cats
	rm -rf doc/{latex,html}
