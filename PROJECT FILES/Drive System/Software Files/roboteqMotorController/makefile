CC=g++


all: sample.o teleop.o square.o encoderSample.o navigation.o

sample.o: RoboteqDevice.o
	$(CC) RoboteqDevice.o sample.cpp -o sample.o

teleop.o: RoboteqDevice.o
	$(CC) RoboteqDevice.o teleop.cpp -o teleop.o

square.o: RoboteqDevice.o
	$(CC) RoboteqDevice.o square.cpp -o square.o

RoboteqDevice.o: RoboteqDevice.cpp
	$(CC) -c RoboteqDevice.cpp

encoderSample.o: RoboteqDevice.o
	$(CC) RoboteqDevice.o encoderSample.cpp -o encoderSample.o

navigation.o: RoboteqDevice.o
	$(CC) RoboteqDevice.o navigation.cpp -o navigation.o

clean:
	rm *.o
