CC=g++


all: newNavigationMethod.o

newNavigationMethod.o: RoboteqDevice.o
	$(CC) RoboteqDevice.o newNavigationMethod.cpp -o newNavigationMethod.o

RoboteqDevice.o: RoboteqDevice.cpp
	$(CC) -c RoboteqDevice.cpp

clean:
	rm *.o
