//Generally needed C++ libraries
#include <iostream>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

//These are the libraries supplied by Roboteq
#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

//These are custom made libraries containing functions for Fieldroid


using namespace std;

int encoderCounts;
int status;
int result;
RoboteqDevice device;
string port = "/dev/ttyACM0";


////////////////////////////////////////////
//Function Declarations
////////////////////////////////////////////
void turnRight(RoboteqDevice motorDevice);

void goStraight(RoboteqDevice motorDevice);

void moveSquare(RoboteqDevice motorDevice);

bool initialize();

bool readAbsoluteEncoderCount(int &count, int index);

/////////////////////////////////
//Main
/////////////////////////////////
int main(int argc, char *argv[])
{
	if(initialize() == false){
	return 1;
	}

	//cout<<"- SetCommand(_GO, 2, 150)...";
	//if((status = device.SetCommand(_GO, 2, 150)) != RQ_SUCCESS)
	//	cout<<"failed --> "<<status<<endl;
	//else
	//	cout<<"succeeded."<<endl;

	//cout<<"- SetCommand(_GO, 2, 150)...";
	//if((status = device.SetCommand(_GO, 1, -150)) != RQ_SUCCESS)
	//	cout<<"failed --> "<<status<<endl;
	//else
	//	cout<<"succeeded."<<endl;

	while(1)
	{
	//if(clock()%500000 == 0){
		readAbsoluteEncoderCount(encoderCounts, 0);
		//cout << "Encoder 1, Channel A:" << encoderCounts << endl;
		//readAbsoluteEncoderCount(encoderCounts, 1);
		//cout << "Encoder 1, Channel B:" << encoderCounts << endl;
		//readAbsoluteEncoderCount(encoderCounts, 2);
		//cout << "Encoder 2, Channel A:" << encoderCounts << endl;
		//readAbsoluteEncoderCount(encoderCounts, 3);
		//cout << "Encoder 2, Channel B:" << encoderCounts << endl;
	//}
	}

	device.Disconnect();
	return 0;
}

////////////////////////////////////////////
//Function Definitions
////////////////////////////////////////////
void turnRight(RoboteqDevice motorDevice)
{
	clock_t initialTime, elapsedTime;
	int motionTime = 9000;

	initialTime = clock();
	elapsedTime = 0;

	while(elapsedTime < motionTime){
	motorDevice.SetCommand(_GO, 1, -150);
	motorDevice.SetCommand(_GO, 2, 150);
	cout << elapsedTime << "    " << CLOCKS_PER_SEC << endl;
	elapsedTime = clock() - initialTime;
	}

	motorDevice.SetCommand(_GO, 1, 0);
	motorDevice.SetCommand(_GO, 2, 0);
}
////////////////////////////////////////////
void goStraight(RoboteqDevice motorDevice)
{
	clock_t initialTime, elapsedTime;
	int motionTime = 15000;

	initialTime = clock();
	elapsedTime = 0;

	while(elapsedTime < motionTime){
	motorDevice.SetCommand(_GO, 1, 150);
	motorDevice.SetCommand(_GO, 2, 150);
	cout << elapsedTime << "    " << CLOCKS_PER_SEC << endl;
	elapsedTime = clock() - initialTime;
	}

	motorDevice.SetCommand(_GO, 1, 0);
	motorDevice.SetCommand(_GO, 2, 0);
}
////////////////////////////////////////////
void moveSquare(RoboteqDevice motorDevice)
{
	clock_t initialTime, elapsedTime;
	int straightTime = 30000;
	int turnTime = 9000;

	//go straight
	initialTime = clock();
	elapsedTime = 0;

	while(elapsedTime < straightTime){
	motorDevice.SetCommand(_GO, 1, 150);
	motorDevice.SetCommand(_GO, 2, 150);
	cout << elapsedTime << "    " << CLOCKS_PER_SEC << endl;
	elapsedTime = clock() - initialTime;
	}

	motorDevice.SetCommand(_GO, 1, 0);
	motorDevice.SetCommand(_GO, 2, 0);

	//turn right
	initialTime = clock();
	elapsedTime = 0;

	while(elapsedTime < turnTime){
	motorDevice.SetCommand(_GO, 1, 150);
	motorDevice.SetCommand(_GO, 2, -150);
	cout << elapsedTime << "    " << CLOCKS_PER_SEC << endl;
	elapsedTime = clock() - initialTime;
	}

	//go straight
	initialTime = clock();
	elapsedTime = 0;

	while(elapsedTime < straightTime){
	motorDevice.SetCommand(_GO, 1, 150);
	motorDevice.SetCommand(_GO, 2, 150);
	cout << elapsedTime << "    " << CLOCKS_PER_SEC << endl;
	elapsedTime = clock() - initialTime;
	}

	motorDevice.SetCommand(_GO, 1, 0);
	motorDevice.SetCommand(_GO, 2, 0);

	//turn right
	initialTime = clock();
	elapsedTime = 0;

	while(elapsedTime < turnTime){
	motorDevice.SetCommand(_GO, 1, 150);
	motorDevice.SetCommand(_GO, 2, -150);
	cout << elapsedTime << "    " << CLOCKS_PER_SEC << endl;
	elapsedTime = clock() - initialTime;
	}

	//go straight
	initialTime = clock();
	elapsedTime = 0;

	while(elapsedTime < straightTime){
	motorDevice.SetCommand(_GO, 1, 150);
	motorDevice.SetCommand(_GO, 2, 150);
	cout << elapsedTime << "    " << CLOCKS_PER_SEC << endl;
	elapsedTime = clock() - initialTime;
	}

	//turn right
	initialTime = clock();
	elapsedTime = 0;

	while(elapsedTime < turnTime){
	motorDevice.SetCommand(_GO, 1, 150);
	motorDevice.SetCommand(_GO, 2, -150);
	cout << elapsedTime << "    " << CLOCKS_PER_SEC << endl;
	elapsedTime = clock() - initialTime;
	}

	//go straight
	initialTime = clock();
	elapsedTime = 0;

	while(elapsedTime < straightTime){
	motorDevice.SetCommand(_GO, 1, 150);
	motorDevice.SetCommand(_GO, 2, 150);
	cout << elapsedTime << "    " << CLOCKS_PER_SEC << endl;
	elapsedTime = clock() - initialTime;
	}

	//turn right
	initialTime = clock();
	elapsedTime = 0;

	while(elapsedTime < turnTime){
	motorDevice.SetCommand(_GO, 1, 150);
	motorDevice.SetCommand(_GO, 2, -150);
	cout << elapsedTime << "    " << CLOCKS_PER_SEC << endl;
	elapsedTime = clock() - initialTime;
	}

	motorDevice.SetCommand(_GO, 1, 0);
	motorDevice.SetCommand(_GO, 2, 0);

	//goStraight(motorDevice);
	//turnRight(motorDevice);
	//goStraight(motorDevice);
	//turnRight(motorDevice);
	//goStraight(motorDevice);
	//turnRight(motorDevice);
	//goStraight(motorDevice);
	//turnRight(motorDevice);
}
/////////////////////////////////////////////////////////////////
bool initialize(){
	string response = "";
	status = device.Connect(port);

	if(status != RQ_SUCCESS)
	{
		cout<<"Error connecting to motor controller: "<<status<<"."<<endl;
		return false;
	}

	cout<<"- SetConfig(_DINA, 1, 1)...";
	if((status = device.SetConfig(_DINA, 1, 1)) != RQ_SUCCESS)
		cout<<"failed --> "<<status<<endl;
	else
		cout<<"succeeded."<<endl;

	//Wait 10 ms before sending another command to device
	sleepms(10);

	return true;

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
}
/////////////////////////////////////////////////////////////////
bool readAbsoluteEncoderCount(int &count, int index)
{
  if(device.GetValue(_ABCNTR, index, count) != RQ_SUCCESS) {
    cout << "Failed to read encoder!!!" << endl;
    return false;
  }
  usleep(10);


  cout << "Absolute Encoder Count:" << count << endl;//[0] << " ch2:" << counts[1] << endl;
  return true;
}
