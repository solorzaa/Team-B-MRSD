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
int counts_per_revolution = 5461; //1250;
float wheelDiameter = 13.2; //wheel diameter in inches
float wheelRadius = wheelDiameter/2; //wheel radius in inches
float botRadius = 11.25; //radius from each wheel to the midpoint between wheels in inches
int stallPower = 40;
float distanceCorrection = 1;
float thetaCorrection = 0.937;
int leftWheelCode = 1;
int rightWheelCode =2;

double linearVelocity = 0;
double angularVelocity = 0;
double leftWheelRPM = 0;
double rightWheelRPM = 0;
double RPM_Constant = (double) counts_per_revolution/1250.0;

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
bool leicaConnected = true;
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
double pathHorizon = 1; 	//Model Predictive Control will look ahead 1 sec to predict a path
double timeStep = 0.1; 	//The resolution of the MPC path will be 0.1 seconds
int numPathPoints = (int) (pathHorizon / timeStep);

clock_t prevTime;
double currentTime;
double diffTime;
clock_t startTime;

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

double getUnixTime();
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Leica Code

// Create variable to handle serial input
	char* full_string;
	full_string = (char*) malloc(100*sizeof(char));

	// Leica Mode Only
	if(leicaConnected) {
		if(!dataOnly) {	
			// Initialize the encoders
			readAbsoluteEncoderCount(prevLeftEncoder, 2); 
			readAbsoluteEncoderCount(prevRightEncoder, 1);		
			prevTime = getUnixTime();
			cout << "Encoder at beginning: "<< prevLeftEncoder<<", "<<prevRightEncoder<<endl;
		}
		// Open leica comm port for radios
		openPort();
		cout << "comm port opened successfully" << endl;

		// Check for first data
		readPort(full_string);

		// Don't start until we have recieved data from the Leica Tracking Station
		while(readPort(full_string)==0){
			printf("no data\n");
			//readPort(full_string);
			//printf("Returned here- string is: %s\n",full_string);
			usleep(100000);
		}
		cout << "Received First set of Leica Data" << endl;

		// Get initial robot data
		testData = leicaStoF(full_string);

		// Convert intial robot data to planar coordinate system
		// Stores in location - {x,y}
		sphericalToPlanar(testData[2], testData[3], testData[4]);

		// Set robot origin to initial planar data ---- STEPHEN THINKS THIS IS BAD
		//xOrigin = location[0];
		//yOrigin = location[1];

		// Initialize the previous Locations
		prevLocation[0] = location[0];
		prevLocation[1] = location[1];
		calibrateTheta[0] = location[0];
		calibrateTheta[1] = location[1];
		float timestamp = testData[0];

		// Get the first point of the orientation calibration
		while(readPort(full_string)==0){
			cout << "Calibrating Origin" << endl;
			usleep(100000);
		}
		testData = leicaStoF(full_string);
		sphericalToPlanar(testData[2], testData[3], testData[4]);

		//sleep(.5);

		// Move 24 inches forward (actually close to 19) to calibrate absolute theta
		//if(!dataOnly) {
		//	bool keepGoing = false;
		//	while(!keepGoing) {
		//		keepGoing = poseControl(getDeltaPose(),0,48,0);
		//		cout<<"                  I am at: "<<location[0]<<", "<<location[1]<<endl;
		//		readPort(full_string);
		//	}
		//}

		while(sqrt(pow((location[0]-calibrateTheta[0]),2)+pow((location[1]-calibrateTheta[1]),2))<30) {
	
			sendVelocityCommands(10,0);

			// Get 2nd point to calibrate theta
			cout << "Waiting for 2nd point " << endl;
			while(readPort(full_string)==0) {
				cout << "Calibrating Theta" << endl;
				usleep(100000);
			}
			testData = leicaStoF(full_string);
			sphericalToPlanar(testData[2], testData[3], testData[4]);
		}

		sendVelocityCommands(0,0);		

		thetaOrigin = atan2((location[1]-calibrateTheta[1]),(location[0]-calibrateTheta[0]));
		cout << "First Point: " << calibrateTheta[0] << ", " << calibrateTheta[1] << " Timestamp: " << fmod(timestamp,1.0) << endl;
		cout << "Second Point: " << location[0] << ", " << location[1] << " Timestamp: " << fmod(testData[0],1.0) <<endl;
		thetaOrigin -= M_PI/2;
		thetaOrigin *= -1;
		
		// Reset the origin at the last data point from calibration.  
		xOrigin = location[0];
		yOrigin = location[1];

	//	sleep(1);
	}

	if(leicaConnected) {
		while(readPort(full_string)==0) {
			printf("Need more data...");
			usleep(100000);
		}
		testData = leicaStoF(full_string);
		sphericalToPlanar(testData[2], testData[3], testData[4]);
		cout << "I think I am at... " << location[0] << ", " << location[1] << endl;
	}

	// Set Encoder Absolute Position to zero
	absoluteX = 0;
	absoluteY = 0;
	absoluteTheta = 0;

	//if(!dataOnly) {	
	//	// Initialize the encoders
	//	readAbsoluteEncoderCount(prevLeftEncoder, 2); 
	//	readAbsoluteEncoderCount(prevRightEncoder, 1);		
	//	prevTime = clock();
	//	cout << "Encoder at beginning: "<< prevLeftEncoder<<", "<<prevRightEncoder<<endl;
	//}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	//Initialize variables to keep time and track position along path
	bool inProgress;				//Keep track of whether or not the bot is still painting
	vector<double> xPathGoal;			//the desired parametric x path
	vector<double> yPathGoal;			//the deisred parametric y path
	vector<double> thetaPathGoal;			//the desired parametric theta path
	double* velocityCommands;
	double xGoal, yGoal, thetaGoal;		//variables not needed in main but needed to run desiredPathXY

	//Construct look up table for model predictive control paths
	Pose*** MPC_LUT = constructLUT(vMin, vMax, vNumberOfEntries, wMin, wMax, wNumberOfEntries);

	if(verbose) cout << "look up table created" << endl;

	startTime = getUnixTime();				//Time the bot received first motion command
	currentTime = getUnixTime() - startTime;	//Initialize the current Time

	while(inProgress = desiredPathXY(currentTime, xGoal, yGoal, thetaGoal)){
		//get current position
		getPose();

		//get current time
		prevTime = currentTime;
		currentTime = getUnixTime() - startTime;
		diffTime = currentTime - prevTime;

		//retrieve the path we want to travel along for the next pathHorizon period
		projectGoal(pathHorizon, xPathGoal, yPathGoal, thetaPathGoal);

		//Retrieve the velocities which minimizes the error between our predicted path and the desired (goal) path

		for(int i=0; i<10;i++) if(verbose) cout << yPathGoal[i] << " ";
		if(verbose) cout << endl;

		velocityCommands =  getOptimalVelocities(MPC_LUT, vNumberOfEntries, wNumberOfEntries, numPathPoints , xPathGoal, yPathGoal, thetaPathGoal);

		if(!verbose) cout << "linearVelocity:" << velocityCommands[0] << endl;
		if(!verbose) cout << "angularVelocity:" << velocityCommands[1] << endl;
			
		if(!verbose) cout << "xGoal:" << xGoal << endl;
		if(!verbose) cout << "yGoal:" << yGoal << endl;
		if(!verbose) cout << "thetaGoal:" << thetaGoal << endl;

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

	//Grab absolute number of encoder counts
	readAbsoluteEncoderCount(currRightEncoder, rightWheelCode);
	readAbsoluteEncoderCount(currLeftEncoder, leftWheelCode);
	/*if(abs(currRightEncoder-prevRightEncoder) > 500 ||abs(currLeftEncoder-prevLeftEncoder)>500) {
	  double deltas[3] = {0, 0, 0};
	  double* frame = deltas;
	  return frame;
	  }*/
	


	cout << "rightEncoderChange: " << currRightEncoder - prevRightEncoder << endl;
	cout << "leftEncoderChange: " << currLeftEncoder - prevLeftEncoder << endl;

	//Calculate the current angle of rotation for each wheel
	rightDeltaPhi = (double) (currRightEncoder - prevRightEncoder) * 2 * M_PI/counts_per_revolution;
	leftDeltaPhi = (double) (currLeftEncoder - prevLeftEncoder) * 2 * M_PI/counts_per_revolution;

	double rightRPM = rightDeltaPhi/(2*M_PI)*60/diffTime;
	double leftRPM = leftDeltaPhi/(2*M_PI)*60/diffTime;

	cout << "rightRPM: " << rightRPM << endl;
	cout << "leftRPM: " << leftRPM << endl;

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

	//Settings
	double desiredVelocity = 10; // 6 in/s
	double fieldLength = 9; // 18 ft
	double fieldWidth = 5; // 10 ft
	double cornerRadius = 3; // 6 ft
	double startingDistance = 1; // 2 ft
	
	// NOTE: do not make fieldLength < 4*cornerRadius
	
	//Calculated times
	double t0 = startingDistance*12/desiredVelocity;
	double t1 = t0 + fieldLength*12/desiredVelocity;
	double t2 = t1 + cornerRadius*12/desiredVelocity;
	double t3 = t2 + 3*M_PI/2*cornerRadius*12/desiredVelocity;
	double t4 = t3 + cornerRadius*12/desiredVelocity;
	double t5 = t4 + fieldWidth*12/desiredVelocity;
	double t6 = t5 + cornerRadius*12/desiredVelocity;
	double t7 = t6 + 3*M_PI/2*cornerRadius*12/desiredVelocity;
	double t8 = t7 + cornerRadius*12/desiredVelocity;
    	double t9 = t8 + fieldLength*12/desiredVelocity;
    	double t10 = t9 + cornerRadius*12/desiredVelocity;
    	double t11 = t10 + 3*M_PI/2*cornerRadius*12/desiredVelocity;
	double t12 = t11 + cornerRadius*12/desiredVelocity;
	double t13 = t12 + fieldWidth*12/desiredVelocity;
    	double t14 = t13 + cornerRadius*12/desiredVelocity;
    	double t15 = t14 + M_PI/2*cornerRadius*12/desiredVelocity;
    	// breaks if length/2 < 2*cornerRadius
    	double t16 = t15 + (fieldLength/2-(2*cornerRadius))*12/desiredVelocity; 
    	double t17 = t16 + M_PI/2*cornerRadius*12/desiredVelocity;
    	double t18 = t17 + cornerRadius*12/desiredVelocity;
    	double t19 = t18 + fieldWidth*12/desiredVelocity;
    	double t20 = t19 + startingDistance*12/desiredVelocity;
	
	// Segment 0: The negative time case (shouldn't happen)
	if ( t < 0 ) {
		x=0;
		y=0;
		th=0;
		return false;
	}
	// Segment 1: Starting Distance (no puddles)
	else if ( t < t0 ) {
		x = 0;
		y = desiredVelocity*t;
		th = 0;
		return true;
	}
	// Segment 2: Straight Line Left Field Length
	else if ( t < t1 ) {
		x = 0;
		y = desiredVelocity*(t-t0)+desiredVelocity*t0;
		th = 0;
		return true;
	}
	// Segment 3: Prepare for circle
	else if ( t < t2 ) {
		x = 0;
		y = desiredVelocity*(t-t1)+desiredVelocity*t1;
		th = 0;
		return true;
	}
	// Segment 4: 3/4ths of a circle to go around a corner (6 ft radius)
	else if ( t < t3 ) {
		x = (cornerRadius*12)*cos((t-t2)/(t3-t2)*3*M_PI/2)-(cornerRadius*12);
		y = (cornerRadius*12)*sin((t-t2)/(t3-t2)*3*M_PI/2)+(desiredVelocity*t2);
		th = 3*M_PI/2*((t-t2)/(t3-t2));
		return true;
	}
	// Segment 5: Return to field from corner circle
	else if ( t < t4 ) {
		x = -(cornerRadius*12)+(desiredVelocity*(t-t3));
		y = (startingDistance+fieldLength)*12;
		th = 3*M_PI/2;
		return true;
	}
	// Segment 6: Straight line Top Field Width
	else if ( t < t5 ) {
		x = desiredVelocity*(t-t4);
		y = (startingDistance+fieldLength)*12;
		th = 3*M_PI/2;
		return true;
	}
	// Segment 7: Prepare for corner
	else if ( t < t6 ) {
		x = desiredVelocity*(t-t5)+fieldWidth*12;
		y = (startingDistance+fieldLength)*12;
		th = 3*M_PI/2;
		return true;
	}
	// segment 8: 2nd Corner
	else if ( t < t7 ) {
		x = (cornerRadius*12)*sin((t-t6)/(t7-t6)*3*M_PI/2)+(fieldWidth+cornerRadius)*12;
		y = -(cornerRadius*12)*cos((t-t6)/(t7-t6)*3*M_PI/2)+(cornerRadius+startingDistance+fieldLength)*12;
		th = 3*M_PI/2+3*M_PI/2*((t-t6)/(t7-t6));
		return true;
	}
	// Segment 9: Exit corner
	else if ( t < t8 ) {
		x = fieldWidth*12;
	        y = (startingDistance+fieldLength+cornerRadius)*12-(desiredVelocity*(t-t7));
		th = M_PI;
		return true;
	}
	// Segment 10: Right Field Length
	else if ( t < t9 ) {
		x = fieldWidth*12;
        	y = (startingDistance+fieldLength)*12-(desiredVelocity*(t-t8));
		th = M_PI;
		return true;
	}
	// Segment 11:  Prepare for corner
	else if ( t < t10 ) {
		x = fieldWidth*12;
        	y = (startingDistance)*12-(desiredVelocity*(t-t9));
		th = M_PI;
		return true;
	}
	// Segment 12: 3rd Corner
	else if ( t < t11 ) {
		x = -(cornerRadius*12)*cos((t-t10)/(t11-t10)*3*M_PI/2)+(cornerRadius*12)+(fieldWidth*12);
        	y = -(cornerRadius*12)*sin((t-t10)/(t11-t10)*3*M_PI/2)-(cornerRadius*12)+(startingDistance*12);
		th = M_PI+3*M_PI/2*((t-t10)/(t11-t10));
		return true;
	}
	// Segment 13: Exit corner
	else if ( t < t12 ) {
		x = (fieldWidth+cornerRadius)*12-(desiredVelocity*(t-t11));
        	y = startingDistance*12;
		th = M_PI/2;
		return true;
	}
	// Segment 14: Bottom Field Width
	else if ( t < t13 ) {
		x = (fieldWidth*12)-(desiredVelocity*(t-t12));
	        y = startingDistance*12;
		th = M_PI/2;
		return true;
	}
	// Segment 15: Prepare to turn towards center
	else if ( t < t14 ) {
		x = -desiredVelocity*(t-t13);
        	y = startingDistance*12;
		th = M_PI/2;
		return true;
	}
	// Segment 16: Turn towards center
	else if ( t < t15 ) {
		x = -(cornerRadius*12)*sin((t-t14)/(t15-t14)*M_PI/2)-(cornerRadius*12);
	        y = -(cornerRadius*12)*cos((t-t14)/(t15-t14)*M_PI/2)+(startingDistance*12)+(cornerRadius*12);
		th = M_PI/2 + M_PI/2*((t-t14)/(t15-t14));
		return true;
	}
	// Segment 17: Travel towards center
	else if ( t < t16 ) {
		x = -(2*cornerRadius*12);
        	y = ((startingDistance+cornerRadius)*12)+desiredVelocity*(t-t15);
		th = 0;
		return true;
	}
	// Segment 18: Turn towards center line
	else if ( t < t17 ) {
		x = -(cornerRadius*12)*cos((t-t16)/(t17-t16)*M_PI/2)-(cornerRadius*12);
	        y = (cornerRadius*12)*sin((t-t16)/(t17-t16)*M_PI/2)+(startingDistance-cornerRadius+fieldLength/2)*12;
		th = M_PI/2*((t-t16)/(t17-t16));
		return true;
	}
	// Segment 19: Prepare to do center
	else if ( t < t18 ) {
		x = -(cornerRadius*12)+desiredVelocity*(t-t17);
        	y = (startingDistance+fieldLength/2)*12;
		th = 3*M_PI/2;
		return true;
	}
	// Segment 20: Do Center Line
	else if ( t < t19 ) {
		x = desiredVelocity*(t-t18);
	        y = (startingDistance+fieldLength/2)*12;
		th = 3*M_PI/2;
		return true;
	}
	// Segment 21: exit field
	else if ( t < t20 ) {
		x = fieldWidth*12+desiredVelocity*(t-t19);
        	y = (startingDistance+fieldLength/2)*12;
		th = 3*M_PI/2;
		return true;
	}
	// Segment X: Stop at the end
	else {
		x = (fieldWidth+startingDistance)*12;
		y = (startingDistance+fieldLength/2)*12;
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
double rightWheel_RPM = RPM_Constant * 30/M_PI*( linearVelocity + angularVelocity * botRadius ) / wheelRadius;
double leftWheel_RPM = RPM_Constant * 30/M_PI*( linearVelocity - angularVelocity * botRadius ) / wheelRadius;

cout << "rightSent_RPM:" << rightWheel_RPM << endl;
cout << "leftSent_RPM:" << leftWheel_RPM << endl;


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

///////////////////////////////////////////////////////////////////////////////////
//getUnixTime:
//Get the time is seconds. This time is indpendent of the machine running this program.
//The library supporting this function is only on Unix machines

double getUnixTime()
{
    struct timespec tv;

    if(clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (((double) tv.tv_sec) + (double) (tv.tv_nsec / 1000000000.0));
}





