# Makefile for ASCII image-capture program
#
# Copyright (C) 2017 Simon D. Levy
#
# Requires: OpenCV

SRC = ../../../src

all: asciicap

asciicap: asciicap.o ImageUtils.o
	g++  -g -o asciicap  asciicap.o ImageUtils.o `pkg-config opencv --libs`

asciicap.o: asciicap.cpp $(SRC)/ImageUtils.h Makefile
	g++  -Wall -I$(SRC) -c asciicap.cpp  `pkg-config opencv --cflags`

ImageUtils.o: $(SRC)/ImageUtils.cpp $(SRC)/ImageUtils.h Makefile
	g++  -Wall -c $(SRC)/ImageUtils.cpp

clean:
	rm -rf asciicap *.o *~ 

run: asciicap
	./asciicap
