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

//These are custom made libraries containing functions for Fieldroid


using namespace std;


//////////////////////////////////////////
//Global Variables
//////////////////////////////////////////
int encoderCounts;
int motorSpeed = 0;
int status;
int result;
RoboteqDevice device;
string motorControllerPort = "/dev/ttyACM0";
int counts_per_revolution = 5500;
float wheelDiameter = 13.2; //wheel diameter in inches
float wheelRadius = wheelDiameter/2; //wheel radius in inches
float botRadius = 11.25; //radius from each wheel to the midpoint between wheels in inches
int stallPower = 40;


double absoluteX = 0;
double absoluteY = 0;
double absoluteTheta = 0;
int leftEncoderCount;
int rightEncoderCount;

//PID Settings
float KPX = 0;//1.5;
float KPY = 0;//.5;
float KPTheta = 35;

int errorXThresh = 100;
int errorYThresh = 100;
int errorThetaThresh = 1.2;




////////////////////////////////////////////
//Function Declarations
////////////////////////////////////////////
void turnRight(RoboteqDevice motorDevice);

void goStraight(RoboteqDevice motorDevice);

void moveSquare(RoboteqDevice motorDevice);

bool initialize();

bool readAbsoluteEncoderCount(int &count, int index);

bool setRelativePosition(int desiredCount);

bool poseControl(double desiredX, double desiredY, double desiredTheta);

//bool getRPM();







/////////////////////////////////
//Main
/////////////////////////////////
int main(int argc, char *argv[])
{
	device.Disconnect();
	if(initialize() == false){
	return 1;
	}

	//cout<<"- SetCommand(_GO, 2, 150)...";
	//if((status = device.SetCommand(_GO, 2, 150)) != RQ_SUCCESS)
	//	cout<<"failed --> "<<status<<endl;
	//else
	//	cout<<"succeeded."<<endl;


	//Update encoder counts
	readAbsoluteEncoderCount(leftEncoderCount, 2);
	readAbsoluteEncoderCount(rightEncoderCount, 1);	
			
	errorXThresh = 100;
	errorYThresh = 100;
	errorThetaThresh = 1.2;
	KPX = 3;
	KPY = 1;
	poseControl(0, 200, 0);

	readAbsoluteEncoderCount(leftEncoderCount, 2);
	readAbsoluteEncoderCount(rightEncoderCount, 1);	
	//sleep(2);

	errorXThresh = 200;
	errorYThresh = 200;
	KPX = 0;
	KPY = 0;
	poseControl(0, 0, 2.67*5.8);

	//sleep(2);

	readAbsoluteEncoderCount(leftEncoderCount, 2);
	readAbsoluteEncoderCount(rightEncoderCount, 1);	

	errorXThresh = 100;
	errorYThresh = 100;
	errorThetaThresh = 1.2;
	KPX = 3;
	KPY = 1;
	poseControl(0, 200, 0);

	readAbsoluteEncoderCount(leftEncoderCount, 2);
	readAbsoluteEncoderCount(rightEncoderCount, 1);	
	//sleep(2);

	errorXThresh = 200;
	errorYThresh = 200;
	KPX = 0;
	KPY = 0;
	poseControl(0, 0, 2.67*5.8);

	//sleep(2);

	readAbsoluteEncoderCount(leftEncoderCount, 2);
	readAbsoluteEncoderCount(rightEncoderCount, 1);	

	errorXThresh = 100;
	errorYThresh = 100;
	errorThetaThresh = 1.2;
	KPX = 3;
	KPY = 1;
	poseControl(0, 200, 0);

	readAbsoluteEncoderCount(leftEncoderCount, 2);
	readAbsoluteEncoderCount(rightEncoderCount, 1);	
	//sleep(2);

	errorXThresh = 200;
	errorYThresh = 200;
	KPX = 0;
	KPY = 0;
	poseControl(0, 0, 2.67*5.8);

	//sleep(2);

	readAbsoluteEncoderCount(leftEncoderCount, 2);
	readAbsoluteEncoderCount(rightEncoderCount, 1);	

	errorXThresh = 100;
	errorYThresh = 100;
	errorThetaThresh = 1.2;
	KPX = 3;
	KPY = 1;
	poseControl(0, 200, 0);

	readAbsoluteEncoderCount(leftEncoderCount, 2);
	readAbsoluteEncoderCount(rightEncoderCount, 1);	
	//sleep(2);

	errorXThresh = 200;
	errorYThresh = 200;
	KPX = 0;
	KPY = 0;
	poseControl(0, 0, 2.67*5.8);

	
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
////////////////////////////////////////////////////////////////////
bool setRelativePosition(int desiredCount){

int threshold = 50;
int encoderCount1;
int encoderCount2;
int velocity1;
int velocity2;

readAbsoluteEncoderCount(encoderCount1, 1);
readAbsoluteEncoderCount(encoderCount2, 2);
int desiredCount1 = encoderCount1 + desiredCount;
int desiredCount2 = encoderCount2 + desiredCount;

int error1 = desiredCount1 - encoderCount1;
int error2 = desiredCount2 - encoderCount2;

while(error1 > threshold)
{
	readAbsoluteEncoderCount(encoderCount1, 1);
	readAbsoluteEncoderCount(encoderCount2, 2);

	error1 = desiredCount1 - encoderCount1;
	error2 = desiredCount2 - encoderCount2;

	//cout << "Encoder Count:" << encoderCounts << endl;
	cout << "error1:" << error1 << endl;
	cout << "error2:" << error2 << endl;
	cout << "error diff: " << error1-error2 << endl;

	if(error1 > error2)
		velocity2 = -50 - 0.1*error2;
	else if(error1 < error2)
		velocity1 = -50 - 0.1*error1;
	else if(error1 == error2)
	{
		velocity1 = -50 - 0.05*error1;
		velocity2 = -50 - 0.05*error2;
	}

	device.SetCommand(_GO, 1, velocity1);
	device.SetCommand(_GO, 2, velocity2);

	//getRPM();

}

device.SetCommand(_GO, 1, 0);
device.SetCommand(_GO, 2, 0);
return true;
/*
int threshold = 5;

readAbsoluteEncoderCount(encoderCounts, 1);
desiredCount = encoderCounts + desiredCount;
int error = desiredCount - encoderCounts;

while(error > threshold)
{
	readAbsoluteEncoderCount(encoderCounts, 1);
	error = desiredCount - encoderCounts;

	cout << "Encoder Count:" << encoderCounts << endl;
	cout << "error:" << error << endl;

	device.SetCommand(_GO, 1, -50 - 0.05*error);

	getRPM();
}

device.SetCommand(_GO, 1, 0);
return true;
*/
/*
//float PositionP = 1;
//float PositionI = 0;
//float PositiondD = 0;
	float proportionalTerm;
	int error = 1000;

	while ( abs(error) > 100 )
	{
		readAbsoluteEncoderCount(encoderCounts, 2);
		cout << "Encoder 1 Count:" << encoderCounts << endl;
		error = desiredCount - encoderCounts;
		cout << "desiredCount:" << desiredCount << endl;
		cout << "error:" << error << endl;

		//proportionalTerm = PositionP * error;

		//motorSpeed = -int(proportionalTerm);
		cout << "motorSpeed:" << motorSpeed << endl;
	
		//Set limits on motor speed

		if(motorSpeed > 255)
		{
			motorSpeed = 255;
		}
		else if(motorSpeed < 50)
		{
			motorSpeed = 0;
		}


		//Send motor command
		if((status = device.SetCommand(_GO, 2, 70)) != RQ_SUCCESS)
		{
			cout<<"failed to send motor command!"<<status<<endl;
			return false;
		}

		device.SetCommand(_GO, 2, 0);
	}

return true;
*/
}

/////////////////////////////////////////////////////////////////////////////////
/*
bool getRPM()
{

	int velocity;
	device.SetCommand(_GO, 1, 150);
	device.SetCommand(_GO, 2, 0);
	device.GetValue(_ABSPEED, 1, velocity);
	cout << velocity << endl;

	return true;

}
*/
//////////////////////////////////////////////////////////////////////
bool poseControl(double desiredX, double desiredY, double desiredTheta)
{
	int currRightEncoder;
	int currLeftEncoder;
	int prevRightEncoder = rightEncoderCount;
	int prevLeftEncoder = leftEncoderCount;

	double rightDeltaPhi;
	double leftDeltaPhi;
	double leftArcLength;
	double rightArcLength;
	double leftTurningRadius;
	double rightTurningRadius;
	double deltaTheta;
	double deltaX;
	double deltaY;
	double errorTheta;
	double errorX;
	double errorY;
	double leftProportional;
	double rightProportional;
	int leftOutputPower;
	int rightOutputPower;


	absoluteX = 0;
	absoluteY = 0;
	absoluteTheta = 0;

	while(1)
	{
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

		//Find turning radius of the current turn for each wheel
		rightTurningRadius = (2 * rightArcLength) / (leftArcLength - rightArcLength);
		leftTurningRadius = (2 * leftArcLength) / (rightArcLength - leftArcLength);


		//In this case, we are making a point turn
		if(rightTurningRadius != rightTurningRadius)
			rightTurningRadius = botRadius;
		if(leftTurningRadius != leftTurningRadius)
			leftTurningRadius = botRadius;

		cout << "rightTurningRadius: " << rightTurningRadius << endl;
		cout << "leftTurningRadius: " << leftTurningRadius << endl;
		
		//Find the change in theta
		if(abs(rightArcLength) > abs(leftArcLength)) //Bot is making a Left Turn (positive change in x)
		{
			//Is it turning about one wheel
			if(leftTurningRadius == 0)
				deltaTheta = rightTurningRadius/ (2*botRadius);
			else
				deltaTheta = leftArcLength / leftTurningRadius;

			//If the center of rotation is inside the bot, else...
			if( (rightArcLength/abs(rightArcLength)) != leftArcLength/abs(leftArcLength) )
			{
				deltaX = (leftTurningRadius + botRadius) * (1 - cos(deltaTheta));
				deltaY = -(leftTurningRadius + botRadius) * sin(deltaTheta);
			}
			else
			{
				deltaX = -(leftTurningRadius + botRadius) * (1 - cos(deltaTheta));
				deltaY = (leftTurningRadius + botRadius) * sin(deltaTheta);
			}
		}
		else if(abs(leftArcLength) > abs(rightArcLength)) //Bot is making a Right Turn (negative change in x)
		{
			//Is it turning about one wheel
			if(rightTurningRadius == 0)
				deltaTheta = -leftTurningRadius/ (2*botRadius);
			else 
				deltaTheta = -rightArcLength / rightTurningRadius;

			//If the center of rotation is inside the bot, else...
			if( (rightArcLength/abs(rightArcLength)) != leftArcLength/abs(leftArcLength) )
			{
				deltaX = -(leftTurningRadius + botRadius) * (1 - cos(deltaTheta));
				deltaY = (leftTurningRadius + botRadius) * sin(deltaTheta);
			}
			else
			{
				deltaX = (rightTurningRadius + botRadius) * (1 - cos(deltaTheta));
				deltaY = -(rightTurningRadius + botRadius) * sin(deltaTheta);
			}
		}
		else // bot is going straight
		{
			deltaTheta = 0;
			deltaX = 0;
			deltaY = wheelRadius * rightDeltaPhi;
		}	

		cout << "delta Theta: " << deltaTheta << endl;
		cout << "delta X: " << deltaX << endl;
		cout << "delta Y: " << deltaY << endl;

		//Calculate absolte position and orientation
		absoluteX += deltaX;
		absoluteY += deltaY;
		absoluteTheta += deltaTheta;

		//Calculate errors
		errorX = desiredX - absoluteX;
		errorY = desiredY - absoluteY;
		errorTheta = desiredTheta - absoluteTheta;

		cout << "Abosulte Theta: " << absoluteTheta << endl;
		cout << "Absolute X: " << absoluteX << endl;
		cout << "Absolute Y: " << absoluteY << endl;

		cout << "Error x: " << errorX << endl;
		cout << "Error y: " << errorY << endl;
		cout << "Error Theta: " << errorTheta << endl;

		cout << "-KPX*errorX: " << -KPX*errorX << endl;
		cout << "-KPY*errorY: " << -KPY*errorY << endl;
		cout << "-KPTheta*errorTheta: " << -KPTheta*errorTheta << endl;

		//Calculate the proportional feedback response
		leftProportional = (-KPX * errorX - KPY * errorY + KPTheta * errorTheta);
		rightProportional = (KPX * errorX - KPY * errorY - KPTheta * errorTheta);

		//Calculate the feedback response
		leftOutputPower = (int) leftProportional;
		rightOutputPower = (int) rightProportional;

		cout << "leftOutputPower: " << leftOutputPower << endl;
		cout << "rightOutputPower: " << rightOutputPower << endl;
/*
		if(abs(leftOutputPower) > 150)
		{
			leftOutputPower = 150*(leftOutputPower/abs(leftOutputPower));
		}

		if(abs(rightOutputPower) > 150)
		{
			rightOutputPower = 150*(rightOutputPower/abs(rightOutputPower));
		}
*/

		device.SetCommand(_GO, 2, leftOutputPower);
		device.SetCommand(_GO, 1, rightOutputPower);

		//cout << "error Theta: " << errorTheta << endl;
		//cout << "error X: " << errorX << endl;
		//cout << "error Y: " << errorY << endl;

		//Stop when the bot is within an inch
		if(abs(errorX) < errorXThresh && abs(errorY) < errorYThresh && abs(errorTheta) < errorThetaThresh)
		{	
			device.SetCommand(_GO, 2, 0);
			device.SetCommand(_GO, 1, 0);
			return true;
		}
	}

	return true;
}
