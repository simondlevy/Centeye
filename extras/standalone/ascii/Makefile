# Makefile for ASCII image-capture program
#
# Copyright (C) 2017 Simon D. Levy
#
# Requires: OpenCV

SRC = ../../../src

all: asciicap

asciicap: asciicap.o CenteyeImages.o
	g++  -g -o asciicap  asciicap.o CenteyeImages.o `pkg-config opencv --libs`

asciicap.o: asciicap.cpp $(SRC)/CenteyeImages.h Makefile
	g++  -Wall -I$(SRC) -c asciicap.cpp  `pkg-config opencv --cflags`

CenteyeImages.o: $(SRC)/CenteyeImages.cpp $(SRC)/CenteyeImages.h Makefile
	g++  -Wall -c $(SRC)/CenteyeImages.cpp

clean:
	rm -rf asciicap *.o *~ 

run: asciicap
	./asciicap
