# makefile for total station code
# David Kohanbash 2011 (dkohanba@cmu.edu)

# GeoCOM_Scarab uses IPC so you need the IPC libraries
# GeoCOM_Test just writes the output to a text file

PROJECT_DIR = ../..
CFLAGS  = -g -Wall -ansi -pedantic -O2 \
          -I $(PROJECT_DIR)/include \
CXXFLAGS := $(CFLAGS)
CXXFLAGS := -g -Wall -ansi -pedantic -O2

all: GeoCOM_Test
#all: GeoCOM_Scarab GeoCOM_Test

OBJS := CommUtil.o GeoCOM_commands.o GeoCOM_displayInfo.o

GeoCOM_Test: GeoCOM_main_Test.o ${OBJS}
	g++ -o $@ $^  -pthread

GeoCOM_Scarab: GeoCOM_main_Scarab.o ${OBJS}
	g++ -o $@ $^ 

clean:
	rm -f *~ *.o GeoCOM_Scarab GeoCOM_Test

install:
	mkdir -p $(PROJECT_DIR)/bin
	cp GeoCOM_Scarab $(PROJECT_DIR)/bin
