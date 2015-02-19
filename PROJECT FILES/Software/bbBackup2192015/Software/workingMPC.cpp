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
string motorControllerPort = "/dev/ttyO4";
int counts_per_revolution = 5500; //1250;
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

double pathTime;
double sumErrorX = 0;
double sumErrorY = 0;
double sumErrorTheta = 0;

/////  SETTINGS ////////
bool leicaConnected = false;
bool dataOnly = false;
bool verbose = false;

double errorXThresh = 12;
double errorYThresh = 10;
double errorThetaThresh = 0.1;


////////Model Predictive Control Parameters///////////////////////////////
//Look Up Table Settings
double vMin = 0;
double vMax = 50;
double vResolution = 0.5;
const int vNumberOfEntries = (int) ( (vMax-vMin) / vResolution );
double wMin = -1;
double wMax = 1;
double wResolution = 0.01;
const int wNumberOfEntries = (int) ( (wMax - wMin) / wResolution );

//Path length and resolution settings
double pathHorizon = 10; 	//Model Predictive Control will look ahead 1 sec to predict a path
double timeStep = 0.5; 	//The resolution of the MPC path will be 0.1 seconds
int numPathPoints = (int) (pathHorizon / timeStep);

clock_t prevTime;
double currentTime;
clock_t startTime;
double stephensComputer = 2.17;
double clock2sec = 1.3/CLOCKS_PER_SEC;//3.22463169065/CLOCKS_PER_SEC;//stephensComputer;

////////////////////////////////////////////
//Function Declarations
////////////////////////////////////////////
bool initialize();

bool readAbsoluteEncoderCount(int &count, int index);

bool velocitiesPolar2Wheel(double _linearVelocity, double _angularVelocity, double &_leftWheelRPM, double &_rightWheelRPM);

bool getPose();

int openPort();

int readPort(char* fs);

vector<float> leicaStoF(char* recievedData);

void sphericalToPlanar(float horAngle, float verAngle, float radius);

Pose* projectPath(double linearVelocity, double angularVelocity, double t_interval, double t_step);

Pose*** constructLUT(double _vMin, double _vMax, const int _vNumberOfEntries, double _wMin, double _wMax, const int _wNumberOfEntries);

bool projectGoal(double horizon, vector<double> & xDesired, vector<double> & yDesired, vector<double> & thDesired);

bool desiredPathXY(double t, double & x, double & y, double & th);

bool desiredPathVW(double t, double & v, double & w);

double getPathError(Pose* projectedPath,  vector<Pose> goalPoints, int _numPathPoints);

double*  getOptimalVelocities(Pose*** projectedPaths, int _vNumberOfEntries, int _wNumberOfEntries, int _numPathPoints ,vector<double> xDesiredPath, vector<double> yDesiredPath, vector<double> thetaDesiredPath);

bool pathToRobotFrame(vector<double> projectedPathX, vector<double> projectedPathY, vector<double> projectedPathTheta, vector<Pose> & newProjectedPath);

bool sendVelocityCommands(double linearVelocity, double angularVelocity);

/////////////////////////////////
//Main

int main(int argc, char *argv[])
{
	// Correct the distance measurements
	wheelRadius = wheelRadius * distanceCorrection;
	botRadius = botRadius * distanceCorrection;

	//setup and initialize the motor controller
	if(!dataOnly) {
		device.Disconnect();
		if(initialize() == false){
			return 1;
		}

		// Turn off the motors (just in case)
		device.SetCommand(_GO, leftWheelCode, 0);
		device.SetCommand(_GO, rightWheelCode, 0);	

		// Initialize Encoder Values
		readAbsoluteEncoderCount(prevLeftEncoder, leftWheelCode);
		readAbsoluteEncoderCount(prevRightEncoder, rightWheelCode);
	}

	//Initialize variables to keep time and track position along path
	bool inProgress;				//Keep track of whether or not the bot is still painting
	vector<double> xPathGoal;			//the desired parametric x path
	vector<double> yPathGoal;			//the deisred parametric y path
	vector<double> thetaPathGoal;			//the desired parametric theta path
	double* velocityCommands;
	double xGoal, yGoal, thetaGoal;		//variables not needed in main but needed to run desiredPathXY

	if(verbose) cout << "preceding look up table" << endl;

	//Construct look up table for model predictive control paths
	Pose*** MPC_LUT = constructLUT(vMin, vMax, vNumberOfEntries, wMin, wMax, wNumberOfEntries);

	if(verbose) cout << "look up table created" << endl;
	if(verbose) cout << "currentTime before while loop: " << currentTime << endl;

	startTime = clock();				//Time the bot received first motion command
	currentTime = (clock() - startTime)*clock2sec;	//Initialize the current Time

	while(inProgress = desiredPathXY(currentTime, xGoal, yGoal, thetaGoal)){
		//get current position
		getPose();

		if(!verbose) cout << "xGoal:" << xGoal << endl;
		if(!verbose) cout << "yGoal:" << yGoal << endl;
		if(!verbose) cout << "thetaGoal:" << thetaGoal << endl;

		//get current time
		currentTime = (double) (clock() - startTime)*clock2sec;
		if(!verbose) cout << "current time: " << currentTime << endl;

		//retrieve the path we want to travel along for the next pathHorizon period
		projectGoal(pathHorizon, xPathGoal, yPathGoal, thetaPathGoal);

		//Retrieve the velocities which minimizes the error between our predicted path and the desired (goal) path

		for(int i=0; i<10;i++) if(verbose) cout << yPathGoal[i] << " ";
		if(verbose) cout << endl;

		velocityCommands =  getOptimalVelocities(MPC_LUT, vNumberOfEntries, wNumberOfEntries, numPathPoints , xPathGoal, yPathGoal, thetaPathGoal);

		if(!verbose) cout << "linearVelocity:" << velocityCommands[0] << endl;
		if(!verbose) cout << "angularVelocity:" << velocityCommands[1] << endl;

		//convert the linear and angular velocity commands to wheel RPMs and send commands to the motors (linear = [0], angular = [1])
		sendVelocityCommands(velocityCommands[0], velocityCommands[1]);

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
bool getPose()
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
	readAbsoluteEncoderCount(currRightEncoder, rightWheelCode);
	readAbsoluteEncoderCount(currLeftEncoder, leftWheelCode);

	/*if(abs(currRightEncoder-prevRightEncoder) > 500 ||abs(currLeftEncoder-prevLeftEncoder)>500) {
	  double deltas[3] = {0, 0, 0};
	  double* frame = deltas;
	  return frame;
	  }*/

	//cout << "rightEncoderCurrent: " << currRightEncoder << endl;
	//cout << "leftEncoderCurrent: " << currLeftEncoder << endl;

	//Calculate the current angle of rotation for each wheel
	rightDeltaPhi = (double) (currRightEncoder - prevRightEncoder) * 2 * M_PI/counts_per_revolution;
	leftDeltaPhi = (double) (currLeftEncoder - prevLeftEncoder) * 2 * M_PI/counts_per_revolution;

	//Update encoder count
	prevRightEncoder =  currRightEncoder;
	prevLeftEncoder =  currLeftEncoder;

	//Find arc length of the turn for each wheel
	rightArcLength = rightDeltaPhi * (wheelRadius);
	leftArcLength = leftDeltaPhi * (wheelRadius);		

	//	cout << "rightArcLength: " << rightArcLength << endl;
	//	cout << "leftArcLength: " << leftArcLength << endl;

	if(rightArcLength == leftArcLength) {
		deltaTheta = 0;
		deltaX = 0;
		deltaY = rightArcLength;
	}
	else {
		//Find turning radius of the current turn for each wheel
		rightTurningRadius = (2 * botRadius * rightArcLength) / (rightArcLength - leftArcLength);
		//leftTurningRadius = (2 * botRadius * leftArcLength) / (rightArcLength - leftArcLength); // This is probably wrong

		//cout << "rightTurningRadius: " << rightTurningRadius << endl;
		//cout << "leftTurningRadius: " << leftTurningRadius << endl;

		// Delta Theta
		deltaTheta = (rightArcLength - leftArcLength)/(2*botRadius)/thetaCorrection;

		// Delta X
		deltaX = -(rightTurningRadius-botRadius)*(1-cos(deltaTheta));

		// Delta Y
		deltaY = (rightTurningRadius-botRadius)*sin(deltaTheta);
	}

		//if(!verbose) cout << "deltaTheta: " << deltaTheta << endl;
		//if(!verbose) cout << "deltaX: " << deltaX << endl;
		//if(!verbose) cout << "deltaY: " << deltaY << endl;


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


	absoluteTheta += deltaTheta;
	//Keep the theta between 2 pi
	absoluteTheta = fmod(absoluteTheta + 2*M_PI, 2*M_PI);
	absoluteX += deltaX*cos(absoluteTheta) - deltaY*sin(absoluteTheta);
	absoluteY += deltaX*sin(absoluteTheta) + deltaY*cos(absoluteTheta);

	//Print out current position in the absolute frame
	if(!verbose) cout << "absoluteTheta: " << absoluteTheta << endl;
	if(!verbose) cout << "absoluteX: " << absoluteX << endl;	
	if(!verbose) cout << "absoluteY: " << absoluteY << endl;

	return true;
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

	if(verbose) cout<< "The vector: ";
	for(vector<float>::const_iterator i = leicaData.begin(); i!= leicaData.end(); ++i)
		cout<< *i << ' ';
	if(verbose) cout<<endl;

	return leicaData;
}

///////////////////////////////////////////////////////////////////////
void sphericalToPlanar(float horAngle, float verAngle, float radius){
	//Convert to radians
	horAngle = -horAngle * (M_PI/180) + (2*M_PI);
	verAngle = verAngle * (M_PI/180);

	if(verbose) cout<< "Horizontal Angle: " << horAngle << endl;
	if(verbose) cout<< "Vertical  Angle: " << verAngle << endl;
	if(verbose) cout<< "Radius: " << radius << endl;
	//cout << xOrigin << endl;
	//cout << yOrigin << endl;

	if(verbose) cout<<"no transformation x: "<< ((radius*cos(horAngle)*cos(verAngle-M_PI/2)*METERS_TO_INCHES))<<endl;
	if(verbose) cout<<"no transformation y: "<< ((radius*sin(horAngle)*cos(verAngle-M_PI/2)*METERS_TO_INCHES))<<endl;
	float translateX = ((radius*cos(horAngle)*cos(verAngle-M_PI/2)*METERS_TO_INCHES)-xOrigin);
	float translateY = ((radius*sin(horAngle)*cos(verAngle-M_PI/2)*METERS_TO_INCHES)-yOrigin);
	location[0] = translateX*cos(thetaOrigin)-translateY*sin(thetaOrigin);
	location[1] = translateX*sin(thetaOrigin)+translateY*cos(thetaOrigin);

	if(verbose) cout<< "xOrigin: " << xOrigin <<endl;
	if(verbose) cout<< "yOrigin: " << yOrigin <<endl;
	if(verbose) cout<< "thetaOrigin: " << thetaOrigin <<endl;
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
			path[i].X = 0;
			path[i].Y = _linearVelocity * t;
			path[i].Theta = 0;	
		}
		else
		{
			path[i].X = -(_linearVelocity/_angularVelocity) * (1-cos(_angularVelocity*t));
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
	double _vResolution = (double) abs(_vMax - _vMin) / _vNumberOfEntries;
	double _wResolution = (double) abs(_wMax - _wMin) / _wNumberOfEntries;

	//Allocate memory
	Pose*** LUT;
	LUT = new Pose**[_vNumberOfEntries];
	for (int i = 0; i < _vNumberOfEntries; ++i)
		LUT[i] = new Pose*[_wNumberOfEntries];

	//Store projected path for each linear and angular velocity combination
	for(int i = 0; i < _vNumberOfEntries; i++){
		for(int j = 0; j < _wNumberOfEntries; j++){

			LUT[i][j] = projectPath(_vMin + (_vResolution * i), _wMin + (_wResolution * j), pathHorizon, timeStep);

		}
	}

	return LUT;
}

///////////////////////////////////////////////////////////////////////

bool projectGoal(double horizon, vector<double> & xHorizon,
                                 vector<double> & yHorizon,
                                 vector<double> & thHorizon) {
        /// This Function:
        ///
        /// Outputs by reference a subset of the desired path
        /// given a length of time and pulling from the current
        /// time that the program is running on
        ///
        /// Assume globals: timeStep, currentTime

	xHorizon.clear();
	yHorizon.clear();
	thHorizon.clear();

        for(double t = currentTime + timeStep; t < currentTime+horizon+timeStep ; t+= timeStep) {
                double x,y,th;
                desiredPathXY(t,x,y,th);
                xHorizon.push_back(x);
                yHorizon.push_back(y);
                thHorizon.push_back(th);
		//cout << "x desired path:" << x << endl;
		//cout << "y desired path:" << y << endl;
        }
        return true;
}

///////////////////////////////////////////////////////////////////////
bool desiredPathXY(double t, double & x, double & y, double & th) {
        /// This Function:
        ///
        /// Returns true if moving, false if not
        ///
        /// Returns (by reference) desired x, y, theta
        /// of the path

        double desiredVelocity = 6; // 6 in/s

        // Segment 0: The negative time case (shouldn't happen)
        if ( t < 0 ) {
                x=0;
                y=0;
                th=0;
                return false;
        }
        // Segment 1: Straight Line - 18 ft
        else if ( t < 36 ) {
                x = 0;
                y = desiredVelocity*t;
                th = 0;
                return true;
        }
        // Segment 2: 3/4ths of a circle to go around a corner (6 ft radius)
        else if ( t < 92.5487 ) {
                x = (6*12)*cos((t-36)/56.5487*3*M_PI/2)-(6*12);
                y = (6*12)*sin((t-36)/56.5487*3*M_PI/2)+(18*12);
                th = 3*M_PI/2*((t-36)/56.5487);
                return true;
        }
        // Segment 3: Straight line - 12 ft
        else if ( t < 116.5487 ) {
                x = -(6*12)+(desiredVelocity*(t-92.5487));
                y = (12*12);
                th = 3*M_PI/2;
                return true;
        }
        // Segment 4: Stop at the end
        else {
                x = (6*12);
                y = (12*12);
                th = 3*M_PI/2;
                return false;
        }
}

///////////////////////////////////////////////////////////////////////////////////
bool desiredPathVW(double t, double & v, double & w) {
        /// This function:
        ///
        /// Returns true if moving, false if not moving
        ///
        /// Returns (by reference) a desired v and w at 
        /// a given point in time 

        double desiredVelocity = 6; // 6 in/s
        double trackRadius = 6*12; // 6 ft

        if ( t < 0 ) {
                v = 0;
                w = 0;
                return false;
        }
        else if ( t < 36 ) {
                v = desiredVelocity;
                w = 0;
                return true;
        }
        else if ( t < 92.5487 ) {
                v = 6;
                w = desiredVelocity/trackRadius;
                return true;
        }
        else if ( t < 116.5487 ) {
                v = desiredVelocity;
                w = 0;
                return true;
        }
        else {
                v = 0;
                w = 0;
                return false;
        }
}

///////////////////////////////////////////////////////////////////////////////////
//getPathError:
//returns the error, or cost function, for a single projected path corresponding to a linear and angualr velocity command
double getPathError(Pose* projectedPath,  vector<Pose> goalPoints, int _numPathPoints)
{
	double error = 0;
	double xError, yError;

	for(int i = 0; i < _numPathPoints; i++){

		xError = projectedPath[i].X - goalPoints[i].X;
		yError = projectedPath[i].Y - goalPoints[i].Y;

		//No need to take the square root of the sum of squares because it will still be the minimum
		error += pow(xError, 2) +  pow(yError, 2);

	}

	//cout << error << " ";

	return error;
}

///////////////////////////////////////////////////////////////////////////////////
//getOptimalVelocities:
//Finds the path that minimizes error in pose with respect to the desired path. 
//Returns the linear velocity and angular velocity that will give that path
double*  getOptimalVelocities(Pose*** projectedPaths, int _vNumberOfEntries, int _wNumberOfEntries, int _numPathPoints ,vector<double> xDesiredPath, vector<double> yDesiredPath, vector<double> thetaDesiredPath)
{
	
	double* velocities = new double[2];
	double errorCurrent, errorMin;
	int _vMinIndex, _wMinIndex;

	//if(verbose) cout << "First y point: " << yDesiredPath[0] << endl;
	
	vector<Pose> goalPath(xDesiredPath.size());
	pathToRobotFrame(xDesiredPath, yDesiredPath, thetaDesiredPath, goalPath);

	//if(verbose) cout << "Rotated y Point: " << goalPath[0].Y << endl;

	for(int i = 0; i < _vNumberOfEntries; i++){
		for(int j = 0; j < _wNumberOfEntries; j++){
		
			//retried error for current path
			errorCurrent = getPathError(projectedPaths[i][j], goalPath, _numPathPoints);
			//set this path error as the minimum if its less than the current minimum
			if((errorCurrent < errorMin) || (i==0 && j==0)){
				errorMin = errorCurrent;
				_vMinIndex = i;
				_wMinIndex = j;
			}

		}
		//cout << endl;
	}
	//cout << endl;
	//cout << "             _vNumberOfEntries: " << _vNumberOfEntries << "          _wNumberOfEntries: " << _wNumberOfEntries << endl;	
	//cout << "                     _vMinIndex:" << _vMinIndex << "       _wMinIndex:"  << _wMinIndex << endl;

	//for (int i = 0; i < numPathPoints; i++){
	//cout << "Projected Path" << projectedPaths[_vMinIndex][_wMinIndex][i].X << " " << projectedPaths[_vMinIndex][_wMinIndex][i].Y << " " << goalPath[i].X  << " " << goalPath[i].Y  << endl;
	
	//}
	
	//convert the index values to the corresponding velocity commands and return them
	velocities[0] =  vMin + (_vMinIndex * vResolution);
	velocities[1] = ( wMin + (_wMinIndex * wResolution) ) * 2;		

	return velocities;

}

////////////////////////////////////////////////////////////////
//sendVelocityCommands:
//Takes in a linear velocity and angular velocity command and
//converts these to individual RPM commands sent to each wheel
bool sendVelocityCommands(double linearVelocity, double angularVelocity)
{

//Find the RPM of each wheel that will give the desired linear and angular velocity
double rightWheel_RPM = 150/M_PI*( linearVelocity + angularVelocity * botRadius ) / wheelRadius;
double leftWheel_RPM = 150/M_PI*( linearVelocity - angularVelocity * botRadius ) / wheelRadius;

//if(!verbose) cout << "rightWheel_RPM:" << rightWheel_RPM << endl;
//if(!verbose) cout << "leftWheel_RPM:" << leftWheel_RPM << endl;


//Send RPM commands to wheels
if((status = device.SetCommand(_GO, leftWheelCode, leftWheel_RPM)) != RQ_SUCCESS)
	cout<<"failed to send left motor command --> "<<status<<endl;
else if((status = device.SetCommand(_GO, rightWheelCode, rightWheel_RPM)) != RQ_SUCCESS)
	cout<<"failed to send right motor command --> "<<status<<endl;
else
	return true;
}

// Rotates the projected goal path into the frame of the robot
bool pathToRobotFrame(vector<double> projectedPathX, vector<double> projectedPathY, vector<double> projectedPathTheta, vector<Pose> & newProjectedPath) {
	
	Pose* translatedPath = new Pose[projectedPathX.size()];

	for (int i=0; i<projectedPathX.size(); i++) {

		translatedPath[i].X = projectedPathX[i] - absoluteX;
		translatedPath[i].Y = projectedPathY[i] - absoluteY;
		newProjectedPath[i].X = cos(absoluteTheta)*translatedPath[i].X + sin(absoluteTheta)*translatedPath[i].Y;
		newProjectedPath[i].Y = -sin(absoluteTheta)*translatedPath[i].X + cos(absoluteTheta)*translatedPath[i].Y;
		newProjectedPath[i].Theta += absoluteTheta;
	
//		newProjectedPath[i].X = cos(absoluteTheta)*projectedPathX[i] + sin(absoluteTheta)*projectedPathY[i];
//		newProjectedPath[i].Y = -sin(absoluteTheta)*projectedPathX[i] + cos(absoluteTheta)*projectedPathY[i];
//		newProjectedPath[i].X -= absoluteX;
//		newProjectedPath[i].Y -= absoluteY;
//		newProjectedPath[i].Theta += absoluteTheta;
	}
}
