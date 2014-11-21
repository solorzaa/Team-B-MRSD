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
string motorControllerPort = "/dev/ttyACM0";
int counts_per_revolution = 5500;
float wheelDiameter = 13.2; //wheel diameter in inches
float wheelRadius = wheelDiameter/2; //wheel radius in inches
float botRadius = 11.25; //radius from each wheel to the midpoint between wheels in inches
int stallPower = 40;

float * xPoints = new float[100]{0,0,0,0,0,0,0,0,0,0,0,-0.48,-4.2,-11.28,-21.36,-33.48,-47.16,-61.2,-74.88,-87,-96.96,-103.92,-107.52,-108,-108,-108,-108,-108,-108,-108,-108,-108,-108,-108,-104.52,-96.96,-82.2,-64.44,-43.44,-21.24,-4.2,12.36,26.4,36.48,44.4,50.4,54.6,54.36,48,36.48,19.92,0,-13.5,-27,-40.5,-54,-67.5,-81,-94.5,-108,-119.4,-129.84,-138.6,-144.84,-148.08,-148.08,-144.84,-138.6,-129.84,-119.4,-108,-94.5,-81,-67.5,-54,-40.5,-27,-13.5,0,11.4,21.84,30.6,36.84,40.08,40.08,36.84,30.6,21.84,11.4,0,-13.5,-27,-40.5,-54,-67.5,-81,-94.5,-108,-121.5,-135};
float * yPoints = new float[100]{0,16.2,32.4,48.6,64.8,81,97.2,113.4,129.6,145.8,162,169.2,182.76,195,204.96,211.92,215.52,215.52,211.8,204.72,194.76,182.52,168.84,162,145.8,129.6,113.4,97.2,81,64.8,48.6,32.4,16.2,0,-12,-23.76,-35.4,-42.36,-45.48,-43.92,-39.12,-30.24,-16.68,0,19.56,40.56,66.36,93.12,121.56,142.92,156.84,162,162,162,162,162,162,162,162,162,160.32,155.52,148.08,138.36,127.32,115.68,104.64,94.92,87.48,82.68,81,81,81,81,81,81,81,81,81,79.32,74.52,67.08,57.36,46.32,34.68,23.64,13.92,6.48,1.68,0,0,0,0,0,0,0,0,0,0,0};
float * thetaPoints = new float[100]{0,0,0,0,0,0,0,0,0,0,0,0.134390352,0.39618974,0.657989128,0.919788516,1.181587904,1.443387291,1.705186679,1.966986067,2.228785455,2.490584843,2.75238423,3.014183618,3.141592654,3.141592654,3.141592654,3.141592654,3.141592654,3.141592654,3.141592654,3.141592654,3.141592654,3.141592654,3.141592654,3.558726345,3.865904293,4.204498168,4.454080251,4.672246408,4.890412564,5.08763477,5.335471523,5.626941508,5.831145031,5.955063408,6.051056517,6.201154832,0.105766953,0.347320521,0.663225116,1.0978121,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.857030324,2.141518992,2.42775299,2.713986987,2.998475655,3.284709652,3.56919832,3.855432318,4.141666315,4.426154983,4.71238898,4.71238898,4.71238898,4.71238898,4.71238898,4.71238898,4.71238898,4.71238898,4.71238898,4.426154983,4.141666315,3.855432318,3.56919832,3.284709652,2.998475655,2.713986987,2.42775299,2.141518992,1.857030324,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327};
int * paintPoints = new int[100]{1,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,-1,0,0,};

double absoluteX = 0;
double absoluteY = 0;
double absoluteTheta = 0;
int prevRightEncoder;
int prevLeftEncoder;


clock_t prevTime;
clock_t currTime;
double sumErrorX = 0;
double sumErrorY = 0;
double sumErrorTheta = 0;

//PID Settings
float KPX = 50;		//30;
float KPY = 14;		//14;
float KPTheta = 200;	//200;

float KIX = 48;		//48;
float KIY = 80;		//80;
float KITheta = 32000;	//32000;

double errorXThresh = 5;
double errorYThresh = 5;
double errorThetaThresh = 0.1;

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
	prevTime = clock();
	
	for(int i=0;i<100;i++) {

		bool done = false;
		while(!done) {
			cout << "            point #: " << i << endl << endl;
			done = poseControl(getDeltaPose(),xPoints[i],yPoints[i],thetaPoints[i]);
		}
	}
/*	bool done = false;

	while(!done) {
		done = poseControl(getDeltaPose(), -0.96, 14.4, 0.13);		
	}
	done = false;
	while(!done) {
		done = poseControl(getDeltaPose(), -8.4, 40.8, .4);		
	}
	done = false;
	while(!done) {
		done = poseControl(getDeltaPose(), -22.56, 67.2, .66);		
	}
	done = false;
	while(!done) {
		done = poseControl(getDeltaPose(), -42.48, 86.4, 0.92);		
	}	
	done = false;
	while(!done) {
		done = poseControl(getDeltaPose(), -66.96, 100.8, 1.18);		
	}
	done = false;
	while(!done) {
		done = poseControl(getDeltaPose(), -94.32, 108.0, 1.44);		
	}
	done = false;
	while(!done) {
		done = poseControl(getDeltaPose(), -122.4, 108.0, 1.71);		
	}
	done = false;
	while(!done) {
		done = poseControl(getDeltaPose(), -149.76, 100.8, 1.97);		
	}
	done = false;
	while(!done) {
		done = poseControl(getDeltaPose(), -173.76, 86.4, 2.23);		
	}
	done = false;
	while(!done) {
		done = poseControl(getDeltaPose(), -193.92, 64.8, 2.49);		
	}*/

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
	
	if(abs(currRightEncoder-prevRightEncoder) > 500 ||abs(currLeftEncoder-prevLeftEncoder)>500) {
		double deltas[3] = {0, 0, 0};
		double* frame = deltas;
		return frame;
	}

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


/////////////////////////////////////////////////////////////////////////////////////////
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
	double diffTime;
	double leftProportionalFeedback;
	double leftIntegralFeedback;
	double leftDerivativeFeedback;
	double rightProportionalFeedback;
	double rightIntegralFeedback;
	double rightDerivativeFeedback;
	
	//Keep the theta between 2 pi
	absoluteTheta = fmod(absoluteTheta + 2*M_PI, 2*M_PI);

	absoluteTheta += deltaTheta;
	absoluteX += deltaX*cos(absoluteTheta) - deltaY*sin(absoluteTheta);
	absoluteY += deltaX*sin(absoluteTheta) + deltaY*cos(absoluteTheta);

	cout << "absoluteTheta: " << absoluteTheta << endl;
	cout << "absoluteX: " << absoluteX << endl;	
	cout << "absoluteY: " << absoluteY << endl;
	
	errorWorldX = desiredX-absoluteX;
	errorWorldY = desiredY-absoluteY;
	errorWorldTheta = desiredTheta-absoluteTheta;

	//Don't spin all the way around the wrong direction when absolute theta crosses 2*M_PI
	if(errorWorldTheta > M_PI){
		errorWorldTheta = errorWorldTheta - 2*M_PI;	
	}
	if(errorWorldTheta < -M_PI){
		errorWorldTheta = errorWorldTheta + 2*M_PI;	
	}

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
	
	//Proportional Feedback
	leftProportionalFeedback = KPX*errorBotX + KPY*errorBotY + KPTheta*errorBotTheta;
	rightProportionalFeedback = - KPX*errorBotX + KPY*errorBotY - KPTheta*errorBotTheta;

	//Integral Feedback
	currTime = clock();
	diffTime = (double) (currTime - prevTime)/CLOCKS_PER_SEC;

	cout << "diffTime: " << diffTime << endl;

	sumErrorX += errorBotX * diffTime;
	sumErrorY += errorBotY * diffTime;
	sumErrorTheta += errorBotTheta * diffTime;

	cout << "errorBotX: " << errorBotX << endl;
	cout << "errorBotY: " << errorBotY << endl;	
	cout << "errorBotTheta: " << errorBotTheta << endl;

	cout << "sumErrorX: " << sumErrorX << endl;
	cout << "sumErrorY: " << sumErrorY << endl;	
	cout << "sumErrorTheta: " << sumErrorTheta << endl;

	prevTime = currTime;

	//cout << "    KIX: " << KIX <<endl;
	//cout << "    KIY: " << KIY <<endl;
	//cout << "    KITheta: " << KITheta <<endl;

	leftIntegralFeedback = KIX*sumErrorX + KIY*sumErrorY + KITheta*sumErrorTheta;
	rightIntegralFeedback = - KIX*sumErrorX + KIY*sumErrorY - KITheta*sumErrorTheta;

	cout << " left integral: " << leftIntegralFeedback << endl;
	cout << " right integral: " << rightIntegralFeedback << endl;

	leftOutputPower = leftProportionalFeedback + leftIntegralFeedback;//KPX*errorBotX + KPY*errorBotY + KPTheta*errorBotTheta + KIX*sumErrorX + KIY*sumErrorY - KITheta*sumErrorTheta;
	rightOutputPower = rightProportionalFeedback + rightIntegralFeedback;//-KPX*errorBotX + KPY*errorBotY - KPTheta*errorBotTheta - KIX*sumErrorX + KIY*sumErrorY + KITheta*sumErrorTheta;
	
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

		//Reset integral fedback terms
		sumErrorX = 0;
		sumErrorY = 0;
		sumErrorTheta = 0;

		return true;
	}
	else {
		return false;
	}
}
