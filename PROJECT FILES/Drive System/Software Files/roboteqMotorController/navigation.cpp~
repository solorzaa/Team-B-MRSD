///////////////////////////////////////////////////////////////////////
//Roboteq SDC 2130 Motor Controller Code
///////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////
//Includes
///////////////////////////////////////////////////////////////////////////
//Generally needed C++ libraries
#include <iostream>
#include <vector>
#include <cmath>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

//These are the libraries supplied by Roboteq
#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

using namespace std;


//////////////////////////////////////////
//Global Variables
//////////////////////////////////////////
int encoderCounts;
int motorSpeed = 0;
int status;
int result;
RoboteqDevice device;
string motorControllerPort = "/dev/ttyACM1";
int counts_per_revolution = 5500;
float wheelDiameter = 13.2; //wheel diameter in inches
float wheelRadius = wheelDiameter/2; //wheel radius in inches
float botRadius = 11.25; //radius from each wheel to the midpoint between wheels in inches
int stallPower = 40;


double absoluteX = 0;
double absoluteY = 0;
double absoluteTheta = 0;
int prevRightEncoder;
int prevLeftEncoder;

//PID Settings
float KPX = 1.2/3;
float KPY = 1.2/3;
float KPTheta = 50;  // Stephen thinks 100

int errorXThresh = 50;
int errorYThresh = 50;
int errorThetaThresh = 0.3;

////////////////////////////////////////////
//Function Declarations
////////////////////////////////////////////
bool initialize();

bool readAbsoluteEncoderCount(int &count, int index);

double* getDeltaPose();

bool poseControl(double * deltaPose, double desiredX, double desiredY, double desiredTheta);


/////////////////////////////////
//Main
/////////////////////////////////
int main(int argc, char *argv[])
{
	device.Disconnect();
	if(initialize() == false){
	return 1;
	}

	device.SetCommand(_GO, 1, 0);
	device.SetCommand(_GO, 2, 0);	

	sleep(1);

	readAbsoluteEncoderCount(prevLeftEncoder, 2);	// Initialize the encoder counts 
	readAbsoluteEncoderCount(prevRightEncoder, 1);		

	errorXThresh = 100;		//When are we close enough to the desired location
	errorYThresh = 100;
	errorThetaThresh = 1.2;
	
	bool done = false;
	
	while(!done) {
		
		done = poseControl(getDeltaPose(), -200, 200, M_PI/4);		//Drive straight 200 inches
	}

	//disconnect roboteq
	device.Disconnect();
	return 0;
}


////////////////////////////////////////////
//Function Definitions
////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
bool initialize(){
	string response = "";
	status = device.Connect(motorControllerPort);

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


  //cout << "Absolute Encoder Count:" << count << endl;//[0] << " ch2:" << counts[1] << endl;
  //return true;
}



//////////////////////////////////////////////////////////////////////
double * getDeltaPose()
{
	int currRightEncoder;
	int currLeftEncoder;

	double rightDeltaPhi;
	double leftDeltaPhi;
	double leftArcLength;
	double rightArcLength;
	//double leftTurningRadius;
	double rightTurningRadius;
	double deltaTheta;
	double deltaX;
	double deltaY;
	
	//Grab absolute number of encoder counts
	readAbsoluteEncoderCount(currRightEncoder, 1);
	readAbsoluteEncoderCount(currLeftEncoder, 2);

	cout << "rightEncoder: " << currRightEncoder << endl;
	cout << "leftEncoder: " << currLeftEncoder << endl;

	//Calculate the current angle of rotation for each wheel
	rightDeltaPhi = (double) (currRightEncoder - prevRightEncoder) * 2 * M_PI/counts_per_revolution;
	leftDeltaPhi = (double) (currLeftEncoder - prevLeftEncoder) * 2 * M_PI/counts_per_revolution;

	//Update encoder count
	prevRightEncoder =  currRightEncoder;
	prevLeftEncoder =  currLeftEncoder;

	//Find arc length of the turn for each wheel
	rightArcLength = rightDeltaPhi * (wheelRadius);
	leftArcLength = leftDeltaPhi * (wheelRadius);		

	cout << "rightArcLength: " << rightArcLength << endl;
	cout << "leftArcLength: " << leftArcLength << endl;

	if(rightArcLength == leftArcLength) {
		deltaTheta = 0;
		deltaX = 0;
		deltaY = rightArcLength;
	}
	else {
		//Find turning radius of the current turn for each wheel
		rightTurningRadius = (2 * botRadius * rightArcLength) / (rightArcLength - leftArcLength);
		//leftTurningRadius = (2 * botRadius * leftArcLength) / (rightArcLength - leftArcLength); // This is probably wrong
		
		cout << "rightTurningRadius: " << rightTurningRadius << endl;
		//cout << "leftTurningRadius: " << leftTurningRadius << endl;
		
		// Delta Theta
		deltaTheta = (rightArcLength - leftArcLength)/(2*botRadius);
		
		// Delta X
		deltaX = -(rightTurningRadius-botRadius)*(1-cos(deltaTheta));
		
		// Delta Y
		deltaY = (rightTurningRadius-botRadius)*sin(deltaTheta);
	}

	cout << "deltaTheta" << deltaTheta << endl;
	cout << "deltaX" << deltaX << endl;
	cout << "deltaY" << deltaY << endl;

	double deltas[3] = {deltaX, deltaY, deltaTheta};
	double* frame = deltas;
	return frame;
}


///////////////////////////////////////////////////
bool poseControl(double * pose, double desiredX, double desiredY, double desiredTheta) {
	double deltaX = pose[0];
	double deltaY = pose[1];
	double deltaTheta = pose[2];
	
	double errorWorldTheta;
	double errorWorldX;
	double errorWorldY;
	double errorBotTheta;
	double errorBotX;
	double errorBotY;
	double leftProportional;
	double rightProportional;
	int leftOutputPower;
	int rightOutputPower;
	
	absoluteTheta += deltaTheta;
	absoluteX += deltaX*cos(absoluteTheta) - deltaY*sin(absoluteTheta);
	absoluteY += deltaX*sin(absoluteTheta) + deltaY*cos(absoluteTheta);

	cout << "absoluteTheta: " << absoluteTheta << endl;
	cout << "absoluteX: " << absoluteX << endl;	
	cout << "absoluteY: " << absoluteY << endl;
	
	errorWorldX = desiredX-absoluteX;
	errorWorldY = desiredY-absoluteY;
	errorWorldTheta = desiredTheta-absoluteTheta;

	//Transform absolute error in position into the robot's frame
	errorBotX = errorWorldX*cos(absoluteTheta) + errorWorldY*sin(absoluteTheta);
	errorBotY = -errorWorldX*sin(absoluteTheta) + errorWorldY*cos(absoluteTheta);
	errorBotTheta = -errorWorldTheta;

	cout << "deisredTheta: " << desiredTheta << endl;
	cout << "desiredX: " << desiredX << endl;	
	cout << "desiredY: " << desiredY << endl;

	cout << "errorWorldTheta: " << errorWorldTheta << endl;
	cout << "errorWorldX: " << errorWorldX << endl;	
	cout << "errorWorldY: " << errorWorldY << endl;
	
	leftOutputPower = KPX*errorBotX + KPY*errorBotY + KPTheta*errorBotTheta; //+ KIX*sumDeltaX + KIY*sumDeltaY - KITheta*sumDeltaTheta;
	rightOutputPower = -KPX*errorBotX + KPY*errorBotY - KPTheta*errorBotTheta; // - KIX*sumDeltaX + KIY*sumDeltaY + KITheta*sumDeltaTheta;
	
	cout << "leftOutputPower: " << leftOutputPower << endl;	
	cout << "rightOutputPower: " << rightOutputPower << endl;

	// Send POWER
	device.SetCommand(_GO, 2, -leftOutputPower);
	device.SetCommand(_GO, 1, -rightOutputPower);
	
	//Stop when the bot is within an inch
	if(abs(errorWorldX) < errorXThresh && abs(errorWorldY) < errorYThresh && abs(errorWorldTheta) < errorThetaThresh)
	{	
		device.SetCommand(_GO, 2, 0);
		device.SetCommand(_GO, 1, 0);
		return true;
	}
	else {
		return false;
	}
}
