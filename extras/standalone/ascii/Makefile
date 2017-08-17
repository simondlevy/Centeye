# Makefile for ASCII image-capture program
#
# Copyright (C) 2017 Simon D. Levy
#
# Requires: OpenCV

all: asciicap

asciicap: asciicap.o CYE_Images_v1.o
	g++  -g -o asciicap  asciicap.o CYE_Images_v1.o `pkg-config opencv --libs`

asciicap.o: asciicap.cpp ../CYE_Images_v1.h Makefile
	g++  -Wall -I.. -c asciicap.cpp  `pkg-config opencv --cflags`

CYE_Images_v1.o: ../CYE_Images_v1.cpp ../CYE_Images_v1.h Makefile
	g++  -Wall -c ../CYE_Images_v1.cpp

clean:
	rm -rf asciicap *.o *~ 

run: asciicap
	./asciicap
