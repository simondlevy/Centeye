# Makefile for flow-capture program
#
# Copyright (C) 2017 Simon D. Levy
#
# Requires: OpenCV

all: flowcap

flowcap: flowcap.o ArduEye_OFO.o
	g++  -g -o flowcap  flowcap.o ArduEye_OFO.o `pkg-config opencv --libs`

flowcap.o: flowcap.cpp ../src/ArduEye_OFO.h Makefile
	g++  -Wall -I../src -c flowcap.cpp  `pkg-config opencv --cflags`

ArduEye_OFO.o: ../src/ArduEye_OFO.cpp ../src/ArduEye_OFO.h Makefile
	g++ -Wall -I../src -c ../src/ArduEye_OFO.cpp

clean:
	rm -rf flowcap *.o *~ 

run: flowcap
	./flowcap
