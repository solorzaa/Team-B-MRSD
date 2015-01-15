#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

using namespace std;

////////////////////////////////////////////
//Function Declarations
////////////////////////////////////////////

bool readAbsoluteEncoderCount(RoboteqDevice device, int &count, int index);

void setLeft(RoboteqDevice device, int speed);
void setRight(RoboteqDevice device, int speed);

/////////////////////////////////
//Main
/////////////////////////////////
int main(int argc, char *argv[])
{
	///////////////////////////////////////////////////////
	string response = "";
	RoboteqDevice device;
	int status = device.Connect("/dev/ttyACM0");

	if(status != RQ_SUCCESS)
	{
		cout<<"Error connecting to device: "<<status<<"."<<endl;
		return 1;
	}

	cout<<"- SetConfig(_DINA, 1, 1)...";
	if((status = device.SetConfig(_DINA, 1, 1)) != RQ_SUCCESS)
		cout<<"failed --> "<<status<<endl;
	else
		cout<<"succeeded."<<endl;

	//Wait 10 ms before sending another command to device
	sleepms(10);

	int result;
	cout<<"- GetConfig(_DINA, 1)...";
	if((status = device.GetConfig(_DINA, 1, result)) != RQ_SUCCESS)
		cout<<"failed --> "<<status<<endl;
	else
		cout<<"returned --> "<<result<<endl;

	//Wait 10 ms before sending another command to device
	sleepms(10);

	cout<<"- GetValue(_ANAIN, 1)...";
	if((status = device.GetValue(_ANAIN, 1, result)) != RQ_SUCCESS)
		cout<<"failed --> "<<status<<endl;
	else
		cout<<"returned --> "<<result<<endl;

	//Wait 10 ms before sending another command to device
	sleepms(10);
	
	/////////////////////////////////////////////////////////

	int leftEncoder;
	int rightEncoder;
	int leftEncoderStart;
	int rightEncoderStart;
	
	// Store the initial values of the encoder into a base value to allow the measurements
	// to start at zero
	readAbsoluteEncoderCount(device, leftEncoder,1);
	leftEncoderStart = leftEncoder;
	readAbsoluteEncoderCount(device, rightEncoder,2);
	rightEncoderStart = rightEncoder;

	// Cycle through powers from 0 to 245 for 2 seconds each
	for(int i = 0; i<250; i+=10) {
		setLeft(device, i);
		setRight(device, i);
		cout << "Power: " << i << endl;
		int startTime = clock();
		// For two seconds, run at the power and print the encoder values
		while((clock()-startTime)/CLOCKS_PER_SEC < 2) {
			readAbsoluteEncoderCount(device, leftEncoder,1);
			readAbsoluteEncoderCount(device, rightEncoder,2);
			cout << "Left Encoder: " << leftEncoder << endl;
			cout << "Right Encoder: " << rightEncoder << endl;
		}
	}
	
	////////////////////////////////////////////////////////
	device.Disconnect();
	return 0;
}

////////////////////////////////////////////
//Function Definitions
////////////////////////////////////////////

bool readAbsoluteEncoderCount(RoboteqDevice device, int &count, int index)
{
	if(device.GetValue(_ABCNTR, index, count) != RQ_SUCCESS) {
		cout << "Failed to read encoder!!!" << endl;
		return false;
	}
	usleep(10);


	//cout << "Absolute Encoder Count:" << count << endl;//[0] << " ch2:" << counts[1] << endl;
	//return true;
}

void setLeft(RoboteqDevice motorDevice, int speed) {
	motorDevice.SetCommand(_GO, 1, speed);
}

void setRight(RoboteqDevice motorDevice, int speed) {
	motorDevice.SetCommand(_GO, 2, speed);
}

void stopAll(RoboteqDevice motorDevice) {
	motorDevice.SetCommand(_GO, 1, 0);
	motorDevice.SetCommand(_GO, 2, 0);
}
