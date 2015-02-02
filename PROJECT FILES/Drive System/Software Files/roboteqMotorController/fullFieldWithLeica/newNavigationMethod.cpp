//////////////////////////////////////////////////////////////////////
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
#include <stdint.h>
#include <unistd.h> /* for write() */
#include <errno.h>
#include <assert.h>
#include <sys/time.h>
#include <cstdlib>
#include <sstream>

//Proprietary Libraries
#include "Pose.cpp"

//These are the libraries supplied by Roboteq
#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

//Fieldroid
#include<termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>

using namespace std;


//////////////////////////////////////////
//Global Variables
//////////////////////////////////////////
#define METERS_TO_INCHES 39.3701

int encoderCounts;
int motorSpeed = 0;
int status;
int result;
RoboteqDevice device;
string motorControllerPort = "/dev/ttyUSB0";
int counts_per_revolution = 5500;
float wheelDiameter = 13.2; //wheel diameter in inches
float wheelRadius = wheelDiameter/2; //wheel radius in inches
float botRadius = 11.25; //radius from each wheel to the midpoint between wheels in inches
int stallPower = 40;
float distanceCorrection = 1;
float thetaCorrection = .9;
int leftWheelCode = 1;
int rightWheelCode =2;

double linearVelocity = 0;
double angularVelocity = 0;
double leftWheelRPM = 0;
double rightWheelRPM = 0;


//Leica communication variables
static int fd;
int ARRAY_SIZE = 200;
char port[20] = "/dev/ttyUSB0"; /* port to connect to */

//Point File
float xPoints[] = {0,0,0,0,0,0,0,0,0,0,0,0,-1.84,-7.2346,-15.816,-27,-40.024,-54,-67.976,-81,-92.184,-100.77,-106.16,-108,-108,-108,-108,-108,-108,-108,-108,-108,-108,-108,-108,-106.16,-100.77,-92.184,-81,-67.976,-54,-40.5,-27,-13.5,0,-9.9196e-15,13.976,27,38.184,46.765,52.16,54,54,54,54,54,54,54,54,54,52.16,46.765,38.184,27,13.976,3.3065e-15,-13.5,-27,-40.5,-54,-67.5,-81,-94.5,-108,-108,-123.5,-136.64,-145.42,-148.5,-145.42,-136.64,-123.5,-108,-94.5,-81,-67.5,-54,-40.5,-27,-13.5,-1.4211e-14,2.4799e-15,15.499,28.638,37.417,40.5,37.417,28.638,15.499,2.4799e-15,-13.5,-27,-40.5,-54,-67.5,-81,-94.5,-108,-121.5,-135};
float yPoints[] = {0,16.2,32.4,48.6,64.8,81,97.2,113.4,129.6,145.8,162,162,175.98,189,200.18,208.77,214.16,216,214.16,208.77,200.18,189,175.98,162,145.8,129.6,113.4,97.2,81,64.8,48.6,32.4,16.2,7.1054e-15,-0,-13.976,-27,-38.184,-46.765,-52.16,-54,-54,-54,-54,-54,-54,-52.16,-46.765,-38.184,-27,-13.976,-1.3226e-14,15.4,30.8,46.2,61.6,77,92.4,107.8,108,121.98,135,146.18,154.77,160.16,162,162,162,162,162,162,162,162,162,162,158.92,150.14,137,121.5,106,92.862,84.083,81,81,81,81,81,81,81,81,81,81,77.917,69.138,55.999,40.5,25.001,11.862,3.0829,0,0,0,0,0,0,0,0,0,0,0};
float thetaPoints[] = {0,0,0,0,0,0,0,0,0,0,0,0,0.2618,0.5236,0.7854,1.0472,1.309,1.5708,1.8326,2.0944,2.3562,2.618,2.8798,3.1416,-3.1416,-3.1416,-3.1416,-3.1416,-3.1416,-3.1416,-3.1416,-3.1416,-3.1416,-3.1416,3.1416,3.4034,3.6652,3.927,4.1888,4.4506,4.7124,4.7124,4.7124,4.7124,4.7124,4.7124,4.9742,5.236,5.4978,5.7596,6.0214,6.2832,0,0,0,0,0,0,0,0,0.2618,0.5236,0.7854,1.0472,1.309,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.9635,2.3562,2.7489,3.1416,3.5343,3.927,4.3197,4.7124,4.7124,4.7124,4.7124,4.7124,4.7124,4.7124,4.7124,4.7124,-1.5708,-1.9635,-2.3562,-2.7489,-3.1416,-3.5343,-3.927,-4.3197,-4.7124,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708};
int paintPoints[] = {1,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,-1,0,0};

float location[3] = {0,0,0};
float prevLocation[2] = {0,0};
float calibrateTheta[2] = {0,0};
float xOrigin = 0;
float yOrigin = 0;
float thetaOrigin = 0;
double absoluteX = 0;
double absoluteY = 0;
double absoluteTheta = 0;
int prevRightEncoder;
int prevLeftEncoder;
vector<float> testData;
double pathHorizon = 1; 	//Model Predictive Control will look ahead 1 sec to predict a path
double pathResolution = 0.1; 	//The resolution of the MPC path will be 0.1 seconds

clock_t prevTime;
clock_t currTime;
clock_t startTime;
double pathTime;
double sumErrorX = 0;
double sumErrorY = 0;
double sumErrorTheta = 0;

bool leicaConnected = false;
bool dataOnly = false;

//PID Settings
float KPX = 30;		// 18
float KPY = 5;		//5
float KPTheta = 300;	//200

float KIX = 40;	//20
float KIY = 80;	//50
float KITheta = 16000;	//2000

double errorXThresh = 12;
double errorYThresh = 10;
double errorThetaThresh = 0.1;

////////////////////////////////////////////
//Function Declarations
////////////////////////////////////////////
bool initialize();

bool readAbsoluteEncoderCount(int &count, int index);

bool velocitiesPolar2Wheel(double _linearVelocity, double _angularVelocity, double &_leftWheelRPM, double &_rightWheelRPM);

double* getDeltaPose();

bool poseControl(double * deltaPose, double desiredX, double desiredY, double desiredTheta);

int openPort();

int readPort(char* fs);

vector<float> leicaStoF(char* recievedData);

void sphericalToPlanar(float horAngle, float verAngle, float radius);

Pose* projectPath(double linearVelocity, double angularVelocity, double t_interval, double t_step);

Pose*** constructLUT(double _vMin, double _vMax, double _vNumberOfEntries, double _wMin, double _wMax, double __wNumberOfEntries);

/////////////////////////////////
//Main
/////////////////////////////////
int main(int argc, char *argv[])
{
	// Correct the angles
        for(int i=0; i<110; i++) {
                thetaPoints[i]+=2*M_PI;
                thetaPoints[i]=fmod(thetaPoints[i],2*M_PI);
        }
	// Correct the distance measurements
	wheelRadius = wheelRadius * distanceCorrection;
	botRadius = botRadius * distanceCorrection;

	// If you don't want motion
	if(!dataOnly) {
		// Set up the motor controller
		device.Disconnect();
		if(initialize() == false){
			return 1;
		}

		// Turn off the motors (just in case)
		device.SetCommand(_GO, leftWheelCode, 0);
		device.SetCommand(_GO, rightWheelCode, 0);	
	}

	//Start clock to keep time along path
	startTime = clock();

	//Move bot based on linear and angular velocities
	while( pathTime < 0.01)	
	{
		linearVelocity = 0;
		angularVelocity = 60;	
		velocitiesPolar2Wheel(linearVelocity, angularVelocity, leftWheelRPM, rightWheelRPM);
		device.SetCommand(_GO, leftWheelCode, leftWheelRPM);
		device.SetCommand(_GO, rightWheelCode, rightWheelRPM);
		pathTime = (double) (clock() - startTime)/CLOCKS_PER_SEC;
		cout << pathTime << endl;
	}

	//Stop the bot
	device.SetCommand(_GO, leftWheelCode, 0);
	device.SetCommand(_GO, rightWheelCode, 0);

	if(!dataOnly) {
		// Disconnect roboteq
		device.Disconnect();
	}
	return 0;
}


/////////////////////////////////////////////////////////////////
//Function Definitions
/////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////////
bool velocitiesPolar2Wheel(double _linearVelocity, double _angularVelocity, double &_leftWheelRPM, double &_rightWheelRPM)
{
	_leftWheelRPM = ( _linearVelocity + _angularVelocity * botRadius ) / wheelRadius;	
	_rightWheelRPM = ( _linearVelocity - _angularVelocity * botRadius ) / wheelRadius;	
	return true;
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

	/*if(abs(currRightEncoder-prevRightEncoder) > 500 ||abs(currLeftEncoder-prevLeftEncoder)>500) {
	  double deltas[3] = {0, 0, 0};
	  double* frame = deltas;
	  return frame;
	  }*/

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
		deltaTheta = (rightArcLength - leftArcLength)/(2*botRadius)/thetaCorrection;

		// Delta X
		deltaX = -(rightTurningRadius-botRadius)*(1-cos(deltaTheta));

		// Delta Y
		deltaY = (rightTurningRadius-botRadius)*sin(deltaTheta);
	}

	cout << "deltaTheta: " << deltaTheta << endl;
	cout << "deltaX: " << deltaX << endl;
	cout << "deltaY: " << deltaY << endl;

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

	if(leicaConnected) {
		//If tracking data is new, update the absolute position
		if((location[0]!=prevLocation[0])||(location[1]!=prevLocation[1])) {
			if(abs(location[0]-absoluteX)>10||abs(location[1]-absoluteY)>10) {
				cout << "Updating the absolute Position of the robot" << endl;
				absoluteX = location[0];
				absoluteY = location[1];
				prevLocation[0] = location[0];
				prevLocation[1] = location[1];
			}
		}
	}

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

	cout << "KPXerrorX: " << KPX*errorBotX << endl;
	cout << "KPYerrorY: " << KPY*errorBotY << endl;
	cout << "KPThetaerrorTheta: " << KPTheta*errorBotTheta << endl;

	cout << "KIXsumErrorX: " << KIX*sumErrorX << endl;
	cout << "KIYsumErrorY: " << KIY*sumErrorY << endl;
	cout << "KIThetasumErrorTheta: " << KITheta*sumErrorTheta << endl;

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
		//device.SetCommand(_GO, 2, 0);
		//device.SetCommand(_GO, 1, 0);

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

/////////////////////////////////////////////////////////////////
int openPort(){


	fd = open(port, O_RDWR | O_NONBLOCK);
	if(-1 == fd){
		//if(verbosity>=1){
		fprintf(stderr, "Could not open %s: ",port);
		perror("");
		//}
		return -1;
	}

	struct termios term;
	memset(term.c_cc, 0, NCCS);
	term.c_iflag=0x0;
	term.c_oflag=0x0;
	term.c_cflag = (CS8|CREAD)| (B57600);
	term.c_lflag = 0;
	term.c_line=0;
	term.c_cc[VMIN]=1;
	term.c_cc[VTIME]=5;
	if (ioctl(fd, TCSETS, &term) != 0 ||
			ioctl(fd, TCFLSH, 2) != 0){
		perror("serialInit() ioctl failed:");
		close(fd);
		return -1;
	}

}

////////////////////////////////////////////////////////////////
int readPort(char* fs){
	char byte_in[100];
	int bytes_read = 0;
	int n = 100; //this is packet size
	int count = 0;
	char last_char = 1;
	bool end_now = true; // end Now WAS FALSE YOU DUMMY!
	int i;
	while(1){
		int l = read(fd, byte_in, ARRAY_SIZE);
		cout<<"Length of the string : " << l<< endl;

		for(i = 0; i < l; i++){
			if(l<40) {
				return 0; // If the data is null, return 0
			}
			//else if(byte_in[i] == '\n' || byte_in[i]=='\0')
			else if(byte_in[i]=='!') { 
				end_now = true;
				break;
			}
			else if(byte_in[0] == '$'){
				if(byte_in[i]!='$'){
					*(fs+count) = byte_in[i];
					count++;
				}
			}
		}
		if(end_now) {
			for(int i=0; i<l; i++){
				cout<<byte_in[i];
			}
			break;
		}
	}
	cout<<endl;
	return count;
}


////////////////////////////////////////////////////////////////////
vector<float> leicaStoF(char* recievedData){

	string leicaString(recievedData);
	string entry;
	string delimeter = ","; //CHANGE THIS TO COMMAS WHEN TESTING WITH LEICA!!!
	size_t pos;
	float number;
	vector<float> leicaData;

	while((pos = leicaString.find(delimeter)) != string::npos){

		entry = leicaString.substr(0, pos).c_str();
		istringstream(entry) >> number;
		leicaData.push_back(number);
		leicaString.erase(0, pos + delimeter.length());
		//cout << "print the number" << number << endl;

	}

	istringstream(leicaString.c_str()) >> number;
	leicaData.push_back(number);

	cout<< "The vector: ";
	for(vector<float>::const_iterator i = leicaData.begin(); i!= leicaData.end(); ++i)
		cout<< *i << ' ';
	cout<<endl;

	return leicaData;
}

///////////////////////////////////////////////////////////////////////
void sphericalToPlanar(float horAngle, float verAngle, float radius){
	//Convert to radians
	horAngle = -horAngle * (M_PI/180) + (2*M_PI);
	verAngle = verAngle * (M_PI/180);

	cout<< "Horizontal Angle: " << horAngle << endl;
	cout<< "Vertical  Angle: " << verAngle << endl;
	cout<< "Radius: " << radius << endl;
	//cout << xOrigin << endl;
	//cout << yOrigin << endl;

	cout<<"no transformation x: "<< ((radius*cos(horAngle)*cos(verAngle-M_PI/2)*METERS_TO_INCHES))<<endl;
	cout<<"no transformation y: "<< ((radius*sin(horAngle)*cos(verAngle-M_PI/2)*METERS_TO_INCHES))<<endl;
	float translateX = ((radius*cos(horAngle)*cos(verAngle-M_PI/2)*METERS_TO_INCHES)-xOrigin);
	float translateY = ((radius*sin(horAngle)*cos(verAngle-M_PI/2)*METERS_TO_INCHES)-yOrigin);
	location[0] = translateX*cos(thetaOrigin)-translateY*sin(thetaOrigin);
	location[1] = translateX*sin(thetaOrigin)+translateY*cos(thetaOrigin);

	cout<< "xOrigin: " << xOrigin <<endl;
	cout<< "yOrigin: " << yOrigin <<endl;
	cout<< "thetaOrigin: " << thetaOrigin <<endl;
	return;
}

//////////////////////////////////////////////////////////////////////////////////
//projectPath:
//Returns an array of multiple points each described by (X,Y,Theta). This array 
//forms the projected path for the given velocities and time interval (how far 
//ahead in time the projected path goes).
Pose* projectPath(double _linearVelocity, double _angularVelocity, double t_interval, double t_step)
{

	//Get the path reoslution, or number of points in the projected path.
	//Then initialize the array of the path points.
	int numberOfPoints = (int) ( t_interval / t_step );
	Pose* path = new Pose[numberOfPoints];

	//intialize the beginning of the path and variables
	double t = 0;

	//Project path forward in time
	for(int i = 0; i < numberOfPoints; i += 1)
	{
		//increment time step
		t += t_step;

		//find the next point in the path
		//Don't divide by zero
		if(_angularVelocity == 0)
		{
			path[i].X = _linearVelocity * t;
			path[i].Y = 0;
			path[i].Theta = 0;	
		}
		else
		{
			path[i].X = (_linearVelocity/_angularVelocity) * cos(_angularVelocity*t);
			path[i].Y = (_linearVelocity/_angularVelocity) * sin(_angularVelocity*t);
			path[i].Theta = _angularVelocity * t;
		}
	}

	return path; 

}

//////////////////////////////////////////////////////////////////////////////////
//constructLUT():
//Iterates through all possible combinations of linear velocity and angular
//velocity to construct a look up table of all possible paths to be taken by the
//the robot
Pose*** constructLUT(double _vMin, double _vMax, const int _vNumberOfEntries, double _wMin, double _wMax, const int _wNumberOfEntries)
{
	//Find the difference between each linear velocity and angular velocity entry in the Look Up Table (this is the resolution)
	double _vResolution = abs(_vMax - _vMin) / _vNumberOfEntries;
	double _wResolution = abs(_wMax - _wMin) / _wNumberOfEntries;

	//Allocate memory
	Pose*** LUT;
	LUT = new Pose**[_vNumberOfEntries];
	for (int i = 0; i < _vNumberOfEntries; ++i)
		LUT[i] = new Pose*[_wNumberOfEntries];

	//Store projected path for each linear and angular velocity combination
    	for(int i = 0; i < _vNumberOfEntries; i++){
        	for(int j = 0; j < _wNumberOfEntries; j++){

        		LUT[i][j] = projectPath(_vMin + (_vResolution * i), _wMin + (_wResolution * j), pathHorizon, pathResolution);
        	
		}
    	}

    	return LUT;
}

///////////////////////////////////////////////////////////////////////////////////
//optimizePath:
//Finds the path that minimizes error in pose with respect to the desired path. 
//Returns the linear velocity and angular velocity that will give that path


















