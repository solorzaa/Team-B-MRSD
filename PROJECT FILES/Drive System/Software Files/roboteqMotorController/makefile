CC=g++


all: sample.o teleop.o

sample.o: RoboteqDevice.o
	$(CC) RoboteqDevice.o sample.cpp -o sample.o

teleop.o: RoboteqDevice.o
	$(CC) RoboteqDevice.o teleop.cpp -o teleop.o

RoboteqDevice.o: RoboteqDevice.cpp
	$(CC) -c RoboteqDevice.cpp

clean:
	rm *.o
