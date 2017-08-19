# Makefile for flow-capture program
#
# Copyright (C) 2017 Simon D. Levy
#
# Requires: OpenCV

SRC = ../../../src

all: flowcap

flowcap: flowcap.o OpticalFlow.o
	g++  -g -o flowcap  flowcap.o OpticalFlow.o `pkg-config opencv --libs`

flowcap.o: flowcap.cpp $(SRC)/OpticalFlow.h Makefile
	g++  -Wall -I$(SRC) -c flowcap.cpp  `pkg-config opencv --cflags`

OpticalFlow.o: $(SRC)/OpticalFlow.cpp $(SRC)/OpticalFlow.h Makefile
	g++ -Wall -I$(SRC) -c $(SRC)/OpticalFlow.cpp

clean:
	rm -rf flowcap *.o *~ 

run: flowcap
	./flowcap
