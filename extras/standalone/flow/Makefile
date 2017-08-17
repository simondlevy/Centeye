# Makefile for flow-capture program
#
# Copyright (C) 2017 Simon D. Levy
#
# Requires: OpenCV

SRC = ../../../src

all: flowcap

flowcap: flowcap.o CenteyeOpticalFlow.o
	g++  -g -o flowcap  flowcap.o CenteyeOpticalFlow.o `pkg-config opencv --libs`

flowcap.o: flowcap.cpp $(SRC)/CenteyeOpticalFlow.h Makefile
	g++  -Wall -I$(SRC) -c flowcap.cpp  `pkg-config opencv --cflags`

CenteyeOpticalFlow.o: $(SRC)/CenteyeOpticalFlow.cpp $(SRC)/CenteyeOpticalFlow.h Makefile
	g++ -Wall -I$(SRC) -c $(SRC)/CenteyeOpticalFlow.cpp

clean:
	rm -rf flowcap *.o *~ 

run: flowcap
	./flowcap
