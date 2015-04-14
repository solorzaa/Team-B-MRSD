/////////////////////////////////////////////////////////////////////
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
#include <Eigen/Dense>
#include <Eigen/Core>

//Proprietary Libraries
#include "Pose.cpp"
//These are the libraries supplied by Roboteq
#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"
#include "SimpleGPIO.h"

//Fieldroid
#include<termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>

//IMU
#include "i2c.h"
#include <pthread.h>

#define DEVICE_ADDR 0x68
#define I2C_DEVICE "/dev/i2c-1"

#define CTRL_REG1 0x10
#define STATUS_REG 0x12

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
float thetaCorrection = 1;// 0.915;//0.937;
int leftWheelCode = 2;
int rightWheelCode = 1;

double linearVelocity = 0;
double angularVelocity = 0;
double leftWheelRPM = 0;
double rightWheelRPM = 0;
double RPM_Constant = (double) counts_per_revolution/1250.0;

//Leica communication variables
static int fd;
int ARRAY_SIZE = 200;
char port[20] = "/dev/ttyO2"; /* port to connect to */

float location[3] = {0,0,0};
float prevLocation[3] = {0,0,0};
bool newLeicaData = false;
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
bool imuConnected = false;
bool paintConnected = true;
double desiredVelocity = 10; // 10 in/s
double fieldLength = 16; // 110 yards
double fieldWidth = 11; // 84 yards
double cornerRadius = 3; // 5 feet
double startingDistance = 3; // 1 foot

////////Model Predictive Control Parameters///////////////////////////////
//Look Up Table Settings
double vMin = 0;
double vMax = 4*desiredVelocity;
double vResolution = vMax*.02;
int vNumberOfEntries = (int) ( (vMax - vMin) / vResolution );
double wMin = -.5;
double wMax = .5;
double wResolution = 0.02;
const int wNumberOfEntries = (int) ( (wMax - wMin) / wResolution );

//Path length and resolution settings
double pathHorizon = 3.5; 	//Model Predictive Control will look ahead 1 sec to predict a path
double timeStep = pathHorizon*0.1; 	//The resolution of the MPC path will be 0.1 seconds
int numPathPoints = (int) (pathHorizon / timeStep);

double prevTime;
double currentTime;
double diffTime;
double startTime;

// IMU
float IMUoffsetGyro;
float headingOffset = 0;
float headingIMU = 0;
bool robotIsDone = false;
bool calibrateTheIMU = true;
bool newIMUData = false;

struct thread_data{
	int thread_id;
	myI2C *i2cptr;
};

//Kalman Filter
Eigen::VectorXd X(6);
Eigen::MatrixXd P(6,6);   //Variance in filter
Eigen::MatrixXd Q(6,6);           //Variance in the motion model
Eigen::Matrix3d qp;
Eigen::Matrix3d qv;
Eigen::MatrixXd Ra(6,6);     
Eigen::MatrixXd F(6,6);

// LEICA BLIP
float previousRadius = 0;

// Painting Port
unsigned int paintGPIO = 44; // GPIO p8_12 (the row closest to paint)
bool painting = false;
double paintTime = 0;
bool moving = false;

////////////////////////////////////////////
//Function Declarations
////////////////////////////////////////////
bool parseCommandLine(int argc, char* argv[]);

bool initialize();

bool readAbsoluteEncoderCount(int &count, int index);

bool velocitiesPolar2Wheel(double _linearVelocity, double _angularVelocity, double &_leftWheelRPM, double &_rightWheelRPM);

bool getPose();

int openPort();

int readPort(char* fs);

vector<float> leicaStoF(char* recievedData);

bool sphericalToPlanar(float horAngle, float verAngle, float radius);

Pose* projectPath(double linearVelocity, double angularVelocity, double t_interval, double t_step);

Pose*** constructLUT(double _vMin, double _vMax, const int _vNumberOfEntries, double _wMin, double _wMax, const int _wNumberOfEntries);

bool projectGoal(double horizon, vector<double> & xDesired, vector<double> & yDesired, vector<double> & pDesired);

bool desiredPathXY(double t, double & x, double & y, int & p);

bool desiredPathVW(double t, double & v, double & w);

double getPathError(Pose* projectedPath,  vector<Pose> goalPoints, int _numPathPoints);

double*  getOptimalVelocities(Pose*** projectedPaths, int _vNumberOfEntries, int _wNumberOfEntries, int _numPathPoints ,vector<double> xDesiredPath, vector<double> yDesiredPath, vector<double> paintDesiredPath);

bool pathToRobotFrame(vector<double> projectedPathX, vector<double> projectedPathY, vector<double> projectedPathPaint, vector<Pose> & newProjectedPath);

bool sendVelocityCommands(double linearVelocity, double angularVelocity);

double getUnixTime();

bool calibrateLeica(char* dataString, float leicaTimeStamp);

//IMU
void *getHeading(void *threadarg);

//Kalman Filter
void runKalman(double rightDeltaPhi, double leftDeltaPhi);

/////////////////////////////////
//Main

int main(int argc, char *argv[])
{
	//read in field painting paramters (field length, width, bot velocity...)
	parseCommandLine(argc, argv);
	
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

	if(paintConnected) {
		gpio_export(paintGPIO);
		gpio_set_dir(paintGPIO, OUTPUT_PIN);
	}

	// IMU Values
	int imuLag = 0;
	float gyroValue = 0;
	float headingValue = 0;
	float prevHeading = 0;
	
	if (imuConnected) {
		// Setup The IMU
		myI2C *i2cptr = new myI2C();
		usleep(10);
		cout << "Calibrating The IMU..." << endl;
		// Initialization
		while(true) {
			try {
				i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_CONFIG_REG_A,0x70);
				i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_CONFIG_REG_B,0x10);
				i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_MODE_REG,0X00);
				break;
			}
			catch(...) {
				perror("Recalibrating IMU...");
			}
		}

		cout << "I completed the callibration " <<endl<<endl <<endl;

		pthread_t threadIMU;
		struct thread_data td;
		td.thread_id = 0;
		td.i2cptr = i2cptr;
		int rc = pthread_create(&threadIMU, NULL, getHeading, (void *)&td);
		if(rc) cout << "Unable to create thread..." << endl;
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
		bool leicaBlip = sphericalToPlanar(testData[2], testData[3], testData[4]);

		// Set robot origin to initial planar data ---- STEPHEN THINKS THIS IS BAD
		//xOrigin = location[0];
		//yOrigin = location[1];

		// Initialize the previous Locations
		prevLocation[0] = location[0];
		prevLocation[1] = location[1];
		calibrateTheta[0] = location[0];
		calibrateTheta[1] = location[1];
		float timestamp = testData[0];

		if(!calibrateLeica(full_string, timestamp))
		{
			cout << "Problem with calibration!" << endl;
		}
	}

	if(leicaConnected) {
		while(readPort(full_string)==0) {
			printf("Need more data...");
			usleep(100000);
		}
		previousRadius = 0;
		testData = leicaStoF(full_string);
		sphericalToPlanar(testData[2], testData[3], testData[4]);
		cout << "I think I am at... " << location[0] << ", " << location[1] << endl;
	}

	// Set Encoder Absolute Position to zero
	absoluteX = 0;
	absoluteY = 0;
	absoluteTheta = 0;

	if(!dataOnly) {	
		// Initialize the encoders
		readAbsoluteEncoderCount(prevLeftEncoder, 2); 
		readAbsoluteEncoderCount(prevRightEncoder, 1);		
		prevTime = getUnixTime();
		cout << "Encoder at beginning: "<< prevLeftEncoder<<", "<<prevRightEncoder<<endl;
	}

	// Fuck the water in the tank
	for(int iii=0; iii<10; iii++) {
		if(leicaConnected) {
			readPort(full_string);
			testData = leicaStoF(full_string);
			sphericalToPlanar(testData[2], testData[3], testData[4]);
		}
		sleep(.2);
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	//Kalman Filter
	cout << "begin initializing Kalman filter" << endl;
	X << 0, 0, 0, 0, 0, 0;                 //State variable
	cout << "initial X: " << X << endl;
	P = Eigen::MatrixXd::Identity(6,6);   //Variance in filter
	P = P*25;
	cout << "initial P: " << P << endl;
	Q = Eigen::MatrixXd::Zero(6,6);           //Variance in the motion model
	cout << "initial Q: " << Q << endl;
	qv << .001, -.05, -.007,
		 -.05,   5,   .04,
		 -.007,  .04, .5;
	qp << .01, 0, 0,
	      0, .01, 0,
	      0, 0, 1;
		
		// 0.0057   -0.0274   -0.0052
		//-0.0274    3.5970    0.0239
		//-0.0052    0.0239    0.0047
	cout << "initial qv: " << qv << endl;
	cout << "initial qp: " << qp << endl;
	Q.bottomRightCorner(3, 3) = qv;                  //put covariance into Q
	Q.topLeftCorner(3,3) = qp;
	cout << "initial completeQ: " << Q << endl;

	// Sensor Covariance (left, right wheel)
	//R << 0.0959, 0.0616,
	//	 0.0616, 0.0750; 
	// Changed to .01 from .1
	Ra << .0286, .0295, 0, 0, 0, 0,
	     .0295, .0328, 0, 0, 0, 0,
	         0,  0, 20, -.0812, -.0001, 0,
		 0,  0, -.0812, 20, -.0016, 0,
		 0,  0, -.0001,  -.0016, .01, 0,
		 0,  0,      0,       0,   0, 100;
	cout << "initial R: " << Ra << endl;



	//Initialize variables to keep time and track position along path
	bool inProgress;				//Keep track of whether or not the bot is still painting
	vector<double> xPathGoal;			//the desired parametric x path
	vector<double> yPathGoal;			//the deisred parametric y path
	vector<double> paintPathGoal;			//the desired parametric theta path
	double* velocityCommands;
	double xGoal, yGoal;		//variables not needed in main but needed to run desiredPathXY
	int paintGoal;

	//Construct look up table for model predictive control paths
	Pose*** MPC_LUT = constructLUT(vMin, vMax, vNumberOfEntries, wMin, wMax, wNumberOfEntries);

	if(verbose) cout << "look up table created" << endl;

	startTime = getUnixTime();				//Time the bot received first motion command
	currentTime = getUnixTime() - startTime;	//Initialize the current Time

	while(inProgress = desiredPathXY(currentTime, xGoal, yGoal, paintGoal)){

		bool leicaBlip = false;

		//Get new position data from tracking station
		if(leicaConnected) {
			readPort(full_string);
			testData = leicaStoF(full_string);
			leicaBlip = sphericalToPlanar(testData[2], testData[3], testData[4]);
		}

		//get current time
		prevTime = currentTime;
		currentTime = getUnixTime() - startTime;
		diffTime = currentTime - prevTime;

		//get current position
		getPose();

		if(imuConnected) {
			if(abs(prevHeading-headingIMU)>0) {
				cout << "Heading in Main: " << headingIMU << endl;
				if(abs(prevHeading-headingIMU)>.17)
					newIMUData = false;
				else {
					prevHeading = headingIMU;
					newIMUData = true;
				}
			}
		}

		//retrieve the path we want to travel along for the next pathHorizon period
		projectGoal(pathHorizon, xPathGoal, yPathGoal, paintPathGoal);

		velocityCommands =  getOptimalVelocities(MPC_LUT, vNumberOfEntries, wNumberOfEntries, numPathPoints , xPathGoal, yPathGoal, paintPathGoal);

		cout << "time:" << currentTime << endl;
		cout << "diffTime: " << diffTime << endl;
		cout << "linearVelocity:" << velocityCommands[0] << endl;
		cout << "angularVelocity:" << velocityCommands[1] << endl;

		cout << "xGoal:" << xGoal << endl;
		cout << "yGoal:" << yGoal << endl;
		cout << "paintGoal:" << paintGoal << endl;

		//convert the linear and angular velocity commands to wheel RPMs and send commands to the motors (linear = [0], angular = [1])
		sendVelocityCommands(velocityCommands[0], velocityCommands[1]);
		
		double lagDistance = sqrt(pow((xGoal-absoluteX),2)+pow((yGoal-absoluteY),2));
		double paintTuningStart = .8*lagDistance/desiredVelocity;
		double paintTuningEnd = 1.15*lagDistance/desiredVelocity;
		int paintGoalStart;	
		int paintGoalEnd;
		desiredPathXY(currentTime-paintTuningStart,xGoal,yGoal,paintGoalStart);
		desiredPathXY(currentTime-paintTuningEnd,xGoal,yGoal,paintGoalEnd);

		if(paintConnected) {
			if(moving) {
				if(painting) {
					if(paintGoalEnd==1) {
						gpio_set_value(paintGPIO, HIGH);
						painting = true;
					}
					else {
						if( (currentTime-paintTime) > 2.5 ) {
							gpio_set_value(paintGPIO, LOW);
							painting = false;
						}
					}
				}
				else {
					if(paintGoalStart==1) {
						gpio_set_value(paintGPIO, HIGH);
						painting = true;
						paintTime = currentTime;
					}
					else {
						gpio_set_value(paintGPIO, LOW);
					painting = false;
					}
				}
			}
			else {
				gpio_set_value(paintGPIO, LOW);
				painting = false;
			}
		}
	}

	//Stop the bot
	device.SetCommand(_GO, leftWheelCode, 0);
	device.SetCommand(_GO, rightWheelCode, 0);

	if(!dataOnly) {
		// Disconnect roboteq
		device.Disconnect();
	}

	cout << "program exited completely" << endl;
	return 0;
}


/////////////////////////////////////////////////////////////////
//Function Definitions
/////////////////////////////////////////////////////////////////
bool parseCommandLine(int argc, char* argv[])
{
    std::cout << argv[0] << std::endl;

    if (argc < 9){
	cout << "Not all parameters were set explicitly" << endl;	
	bool leicaConnected = true;        
	bool imuConnected = false;
	bool paintConnected = true;
	double desiredVelocity = 10; // 10 in/s
	double fieldLength = 16; // 110 yards
	double fieldWidth = 11; // 84 yards
	double cornerRadius = 3; // 5 feet
	double pathHorizon = 3.5; 	//Model Predictive Control will look ahead 1 sec to predict a path
    }
    else{
        imuConnected = atof(argv[1]);		//Boolean
        leicaConnected = atof(argv[2]);		//Boolean
        paintConnected = atof(argv[3]);		//Boolean
        desiredVelocity = atof(argv[4]);	//in/s
        fieldWidth = atof(argv[5])*3;		//yards
        fieldLength = atof(argv[6])*3;		//yards
        cornerRadius = atof(argv[7])*3;		//yards
        pathHorizon = atof(argv[8]);		//seconds
		//counts_per_revolution = atof(argv[9]);

	cout << "Args: " << argv[1] << " " << argv[2] << " " << argv[3] << " " << argv[4] << " " << argv[5] << " " << argv[6] << " " << argv[7] << " " << argv[8] << endl;

	cout << "Args Real: " << imuConnected << " " << leicaConnected << " " << paintConnected << " " << desiredVelocity << " " << fieldWidth << " " << fieldLength << " " << cornerRadius << " " << pathHorizon << endl;

	vMin = 0;
	vMax = 4*desiredVelocity;
	vResolution = vMax*.02;
    }
    vNumberOfEntries = (int) ( (vMax - vMin) / vResolution );
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
	double deltaTheta = 0;
	double deltaX = 0;
	double deltaY = 0;
	double absDeltaX = 0;
	double absDeltaY = 0;
	if(leicaConnected) {
                //If tracking data is new, update the absolute position
                if((location[0]!=prevLocation[0])||(location[1]!=prevLocation[1])) {
					
					if(currentTime > 2){
						newLeicaData = true;
					}

					location[2] = atan2(location[1]-prevLocation[1],location[0]-prevLocation[0]);
					location[2] -= M_PI/2;
					location[2] = fmod(location[2]+2*M_PI,2*M_PI);
					prevLocation[0] = location[0];
					prevLocation[1] = location[1];
					cout << endl << "Leica X: " << location[0] << endl;
					cout << "Leica Y: " << location[1] << endl;
					cout << "Leica Theta: " << location[2] << endl;
                        /*if(abs(location[0]-absoluteX)>10||abs(location[1]-absoluteY)>10) {
                                cout << "Updating the absolute Position of the robot" << endl;
                                absoluteX = absoluteX*0.95 + location[0] * 0.05;
                                absoluteY = absoluteY*0.95 + location[1] * 0.05 ;
                                prevLocation[0] = location[0];
                                prevLocation[1] = location[1];
                        }*/
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
	


	//cout << "rightEncoderChange: " << currRightEncoder - prevRightEncoder << endl;
	//cout << "leftEncoderChange: " << currLeftEncoder - prevLeftEncoder << endl;

	//Calculate the current angle of rotation for each wheel
	rightDeltaPhi = (double) (currRightEncoder - prevRightEncoder) * 2 * M_PI/counts_per_revolution;
	leftDeltaPhi = (double) (currLeftEncoder - prevLeftEncoder) * 2 * M_PI/counts_per_revolution;
	if(rightDeltaPhi==0 && leftDeltaPhi==0) moving = false;
	else moving = true;

	double rightRPM = rightDeltaPhi/(2*M_PI)*60/diffTime;
	double leftRPM = leftDeltaPhi/(2*M_PI)*60/diffTime;
	
	cout << "Wheel Velocities: " << rightDeltaPhi/diffTime << " " << leftDeltaPhi/diffTime << endl;

	//cout << "rightRPM: " << rightRPM << endl;
	//cout << "leftRPM: " << leftRPM << endl;

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

	//absoluteTheta += deltaTheta;
	//Keep the theta between 2 pi
	//absoluteTheta = fmod(absoluteTheta + 2*M_PI, 2*M_PI);
	absDeltaX = deltaX*cos(absoluteTheta) - deltaY*sin(absoluteTheta);
	absDeltaY = deltaX*sin(absoluteTheta) + deltaY*cos(absoluteTheta);
	//absoluteX += absDeltaX;
	//absoluteY += absDeltaY;
	runKalman(rightDeltaPhi, leftDeltaPhi);
	
	//Print out current position in the absolute frame
	if(!verbose) cout << "absoluteTheta: " << absoluteTheta << endl;
	//if(!verbose) cout << "absoluteX: " << absoluteX << endl;	
	//if(!verbose) cout << "absoluteY: " << absoluteY << endl;
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
bool sphericalToPlanar(float horAngle, float verAngle, float radius){
	//Convert to radians
	horAngle = -horAngle * (M_PI/180) + (2*M_PI);
	verAngle = verAngle * (M_PI/180);

	if(verbose) cout<< "Horizontal Angle: " << horAngle << endl;
	if(verbose) cout<< "Vertical  Angle: " << verAngle << endl;
	if(verbose) cout<< "Radius: " << radius << endl;
	//cout << xOrigin << endl;
	//cout << yOrigin << endl;

	// Leica Blip
	if(radius==previousRadius)
		return false;
	previousRadius = radius;

	if(verbose) cout<<"no transformation x: "<< ((radius*cos(horAngle)*cos(verAngle-M_PI/2)*METERS_TO_INCHES))<<endl;
	if(verbose) cout<<"no transformation y: "<< ((radius*sin(horAngle)*cos(verAngle-M_PI/2)*METERS_TO_INCHES))<<endl;
	float translateX = ((radius*cos(horAngle)*cos(verAngle-M_PI/2)*METERS_TO_INCHES)-xOrigin);
	float translateY = ((radius*sin(horAngle)*cos(verAngle-M_PI/2)*METERS_TO_INCHES)-yOrigin);
	location[0] = translateX*cos(thetaOrigin)-translateY*sin(thetaOrigin);
	location[1] = translateX*sin(thetaOrigin)+translateY*cos(thetaOrigin);

	if(verbose) cout<< "xOrigin: " << xOrigin <<endl;
	if(verbose) cout<< "yOrigin: " << yOrigin <<endl;
	if(verbose) cout<< "thetaOrigin: " << thetaOrigin <<endl;
	return true;
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
                                 vector<double> & pHorizon) {
        /// This Function:
        ///
        /// Outputs by reference a subset of the desired path
        /// given a length of time and pulling from the current
        /// time that the program is running on
        ///
        /// Assume globals: timeStep, currentTime

	xHorizon.clear();
	yHorizon.clear();
	pHorizon.clear();

        for(double t = currentTime + timeStep; t < currentTime+horizon+timeStep ; t+= timeStep) {
                double x,y;
		int p;
                desiredPathXY(t,x,y,p);
                xHorizon.push_back(x);
                yHorizon.push_back(y);
                pHorizon.push_back(p);
		//cout << "x desired path:" << x << endl;
		//cout << "y desired path:" << y << endl;
        }
        return true;
}

/////////////////////////////////////////////////////////////////////
bool desiredPathXY(double t, double & x, double & y, int & p) {
	/// This Function:
	///
	/// Returns true if moving, false if not
	///
	/// Returns (by reference) desired x, y, theta
	/// of the path

	// Default Paint Setting
	p=0;

	// Calculated based on Field Dimensions
	double penaltyAreaWidth = 44./84.*fieldWidth;
	double penaltyAreaLength = 18./84.*fieldWidth;
	double centerRadius = 10./84.*fieldWidth;
	double penaltyRadius = centerRadius;
	double penaltyCenter = 12./84.*fieldWidth;
	double penaltyBoxWidth = 20./84.*fieldWidth;
	double penaltyBoxLength = 6./84.*fieldWidth;
	double cornerKickRadius = 1./84.*fieldWidth;
	double markLength = 1./84.*fieldWidth;


	double t0 = 2*startingDistance*12/desiredVelocity;
	double t1 = t0 + fieldLength*12/desiredVelocity;
	double t2 = t1 + cornerRadius*12/desiredVelocity;
	double t3 = t2 + 3*M_PI/2*(cornerRadius+cornerKickRadius)*12/desiredVelocity;
	double t4 = t3 + (cornerRadius+cornerKickRadius)*12/desiredVelocity;
	double t5 = t4 + M_PI/2*cornerKickRadius*12/desiredVelocity;
	double t6 = t5 + (cornerRadius+cornerKickRadius)*12/desiredVelocity;
	double t7 = t6 + 3*M_PI/2*(cornerRadius+cornerKickRadius)*12/desiredVelocity;
	double t8 = t7 + cornerRadius*12/desiredVelocity;
	double t9 = t8 + fieldWidth*12/desiredVelocity;
	double t10 = t9 + cornerRadius*12/desiredVelocity;
	double t11 = t10 + 3*M_PI/2*(cornerRadius+cornerKickRadius)*12/desiredVelocity;
	double t12 = t11 + (cornerRadius+cornerKickRadius)*12/desiredVelocity;
	double t13 = t12 + M_PI/2*cornerKickRadius*12/desiredVelocity;
	double t14 = t13 + (cornerRadius+cornerKickRadius)*12/desiredVelocity;
	double t15 = t14 + 3*M_PI/2*(cornerRadius+cornerKickRadius)*12/desiredVelocity;
	double t16 = t15 + (cornerRadius)*12/desiredVelocity;
	double t17 = t16 + fieldLength*12/desiredVelocity;
	double t18 = t17 + cornerRadius*12/desiredVelocity;
	double t19 = t18 + 3*M_PI/2*(cornerRadius+cornerKickRadius)*12/desiredVelocity;
	double t20 = t19 + (cornerKickRadius + cornerRadius)*12/desiredVelocity;
	double t21 = t20 + M_PI/2*cornerKickRadius*12/desiredVelocity;
	double t22 = t21 + (cornerKickRadius+cornerRadius)*12/desiredVelocity;
	double t23 = t22 + 3*M_PI/2*(cornerRadius+cornerKickRadius)*12/desiredVelocity;
	double t24 = t23 + cornerRadius*12/desiredVelocity;
	double t25 = t24 + fieldWidth*12/desiredVelocity;
	double t26 = t25 + cornerRadius*12/desiredVelocity;
	double t27 = t26 + 3*M_PI/2*(cornerRadius+cornerKickRadius)*12/desiredVelocity;
	double t28 = t27 + (cornerRadius+cornerKickRadius)*12/desiredVelocity;
	double t29 = t28 + M_PI/2*cornerKickRadius*12/desiredVelocity;
	double t30 = t29 + cornerRadius*12/desiredVelocity;
	double t31 = t30 + M_PI/2*cornerRadius*12/desiredVelocity;
	double t32 = t31 + (fieldLength/2-(2*cornerRadius)-cornerKickRadius)*12/desiredVelocity; 
	double t33 = t32 + M_PI/2*cornerRadius*12/desiredVelocity;
	double t34 = t33 + cornerRadius*12/desiredVelocity;
	double t35 = t34 + fieldWidth*12/desiredVelocity;
	double t36 = t35 + cornerRadius*12/desiredVelocity;
	double t37 = t36 + M_PI/2*cornerRadius*12/desiredVelocity;
	double t38 = t37 + (fieldLength/2 - penaltyAreaLength - (2*cornerRadius))*12/desiredVelocity;
	double t39 = t38 + M_PI/2*cornerRadius*12/desiredVelocity;
	double t40 = t39 + ((fieldWidth-penaltyAreaWidth)/2 + cornerRadius)*12/desiredVelocity;
	double t41 = t40 + penaltyAreaWidth*12/desiredVelocity;
	double t42 = t41 + cornerRadius*12/desiredVelocity;
	double t43 = t42 + 3*M_PI/2*cornerRadius*12/desiredVelocity;
	double t44 = t43 + cornerRadius*12/desiredVelocity;
	double t45 = t44 + penaltyAreaLength*12/desiredVelocity;
	double t46 = t45 + (penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12/desiredVelocity;
	double t47 = t46 + M_PI/2*(penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12/desiredVelocity;
	double t48 = t47 + (penaltyAreaWidth/2-penaltyBoxWidth/2-2*(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12/desiredVelocity;
	double t49 = t48 + M_PI/2*(penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12/desiredVelocity;
	double t50 = t49 + (penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12/desiredVelocity;
	double t51 = t50 + penaltyBoxLength*12/desiredVelocity;
	double t52 = t51 + cornerRadius*12/desiredVelocity;
	double t53 = t52 + 3*M_PI/2*cornerRadius*12/desiredVelocity;
	double t54 = t53 + cornerRadius*12/desiredVelocity;
	double t55 = t54 + penaltyBoxWidth*12/desiredVelocity;
	double t56 = t55 + cornerRadius*12/desiredVelocity;
	double t57 = t56 + 3*M_PI/2*cornerRadius*12/desiredVelocity;
	double t58 = t57 + cornerRadius*12/desiredVelocity;
	double t59 = t58 + penaltyBoxLength*12/desiredVelocity;
	double t60 = t59 + (penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12/desiredVelocity;
	double t61 = t60 + M_PI/2*(penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12/desiredVelocity;
	double t62 = t61 + (penaltyAreaWidth/2-penaltyBoxWidth/2-2*(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12/desiredVelocity;
	double t63 = t62 + M_PI/2*(penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12/desiredVelocity;
	double t64 = t63 + (penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12/desiredVelocity;
	double t65 = t64 + penaltyAreaLength*12/desiredVelocity;
	double t66 = t65 + cornerRadius*12/desiredVelocity;
	double t67 = t66 + M_PI*cornerRadius*12/desiredVelocity;
	double t68 = t67 + (penaltyAreaLength+cornerRadius-penaltyCenter)*12/desiredVelocity;
	double t69 = t68 + M_PI/2*cornerRadius*12/desiredVelocity;
	double t70 = t69 + (penaltyAreaWidth/2-penaltyRadius)*12/desiredVelocity;
	double t71 = t70 + M_PI/2*cornerRadius*12/desiredVelocity;
	double t72 = t71 + penaltyRadius*asin((penaltyAreaLength-penaltyCenter)/penaltyRadius)*12/desiredVelocity;
	double t73 = t72 + 2*penaltyRadius*acos((penaltyAreaLength-penaltyCenter)/penaltyRadius)*12/desiredVelocity;
	double t74 = t73 + penaltyRadius*asin((penaltyAreaLength-penaltyCenter)/penaltyRadius)*12/desiredVelocity;
	double t75 = t74 + 3*M_PI/2*cornerRadius*12/desiredVelocity;
	double t76 = t75 + M_PI/2*cornerRadius*12/desiredVelocity;
	double t77 = t76 + (fieldLength/2-penaltyCenter-2*cornerRadius)*12/desiredVelocity;
	double t78 = t77 + 2*M_PI*centerRadius*12/desiredVelocity;
	double t79 = t78 + (fieldLength/2-penaltyBoxLength)*12/desiredVelocity;
	double t80 = t79 + penaltyBoxLength*12/desiredVelocity;
	double t81 = t80 + (penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12/desiredVelocity;
	double t82 = t81 + M_PI/2*(penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12/desiredVelocity;
	double t83 = t82 + (penaltyAreaWidth/2-penaltyBoxWidth/2-2*(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12/desiredVelocity;
	double t84 = t83 + M_PI/2*(penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12/desiredVelocity;
	double t85 = t84 + (penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12/desiredVelocity;
	double t86 = t85 + penaltyAreaLength*12/desiredVelocity;
	double t87 = t86 + cornerRadius*12/desiredVelocity;
	double t88 = t87 + 3*M_PI/2*cornerRadius*12/desiredVelocity;
	double t89 = t88 + cornerRadius*12/desiredVelocity;
	double t90 = t89 + penaltyAreaWidth*12/desiredVelocity;
	double t91 = t90 + cornerRadius*12/desiredVelocity;
	double t92 = t91 + 3*M_PI/2*cornerRadius*12/desiredVelocity;
	double t93 = t92 + cornerRadius*12/desiredVelocity;
	double t94 = t93 + penaltyAreaLength*12/desiredVelocity;
	double t95 = t94 + (penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12/desiredVelocity;
	double t96 = t95 + M_PI/2*(penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12/desiredVelocity;
	double t97 = t96 + (penaltyAreaWidth/2-penaltyBoxWidth/2-2*(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12/desiredVelocity;
	double t98 = t97 + M_PI/2*(penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12/desiredVelocity;
	double t99 = t98 + (penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12/desiredVelocity;
	double t100 = t99 + penaltyBoxLength*12/desiredVelocity;
	double t101 = t100 + cornerRadius*12/desiredVelocity;
	double t102 = t101 + 3*M_PI/2*cornerRadius*12/desiredVelocity;
	double t103 = t102 + cornerRadius*12/desiredVelocity;
	double t104 = t103 + penaltyBoxWidth*12/desiredVelocity;
	double t105 = t104 + cornerRadius*12/desiredVelocity;
	double t106 = t105 + 3*M_PI/2*cornerRadius*12/desiredVelocity;
	double t107 = t106 + (penaltyRadius-(penaltyBoxLength-cornerRadius))*12/desiredVelocity;
	double t108 = t107 + 0;//(penaltyCenter-penaltyBoxLength-3*cornerRadius)*12/desiredVelocity;
	double t109 = t108 + penaltyRadius*asin((penaltyAreaLength-penaltyCenter)/penaltyRadius)*12/desiredVelocity;
	double t110 = t109 + 2*penaltyRadius*acos((penaltyAreaLength-penaltyCenter)/penaltyRadius)*12/desiredVelocity;
	double t111 = t110 + penaltyRadius*asin((penaltyAreaLength-penaltyCenter)/penaltyRadius)*12/desiredVelocity;
	double t112 = t111 + markLength/2*12/desiredVelocity;
	double t113 = t112 + M_PI/2*(penaltyBoxWidth/4)*12/desiredVelocity;
	double t114 = t113 + (centerRadius-2*(penaltyBoxWidth/4))*12/desiredVelocity;
	double t115 = t114 + M_PI/2*(penaltyBoxWidth/4)*12/desiredVelocity;
	double t116 = t115 + markLength*12/desiredVelocity;
	double t117 = t116 + (fieldLength/2-penaltyCenter-markLength)*12/desiredVelocity;
	double t118 = t117 + markLength*12/desiredVelocity;
	double t119 = t118 + (fieldLength/2-penaltyCenter-markLength)*12/desiredVelocity;
	double t120 = t119 + markLength*12/desiredVelocity;
	double t121 = t120 + startingDistance*12/desiredVelocity;

	if ( t < t0 ) {
		x=0;
		y=desiredVelocity*t*t/2/t0;
		p=0;
	}
	else if ( t < t1 ) {
		x = 0;
		y = desiredVelocity*(t-t0)+startingDistance*12;
		p=1;
	}
	else if ( t < t2 ) {
		x = 0;
		y = desiredVelocity*(t-t1)+(startingDistance+fieldLength)*12;
		p=0;
	}
	else if ( t < t3 ) {
		x = ((cornerRadius+cornerKickRadius)*12)*cos((t-t2)/(t3-t2)*3*M_PI/2)-((cornerRadius+cornerKickRadius)*12);
		y = ((cornerRadius+cornerKickRadius)*12)*sin((t-t2)/(t3-t2)*3*M_PI/2)+(startingDistance+fieldLength+cornerRadius)*12;
		p=0;
	}
	else if ( t < t4 ) {
		x = -((cornerRadius+cornerKickRadius)*12)+(desiredVelocity*(t-t3));
		y = (startingDistance+fieldLength-cornerKickRadius)*12;
		p=0;
	}
	else if ( t < t5 ) {
		x = (cornerKickRadius*12)*sin((t-t4)/(t5-t4)*M_PI/2);
		y = -(cornerKickRadius*12)*cos((t-t4)/(t5-t4)*M_PI/2)+(startingDistance+fieldLength)*12;
		p=1;
	}
	else if ( t < t6 ) {
		x = cornerKickRadius*12;
		y = (fieldLength+startingDistance)*12 + desiredVelocity*(t-t5);
		p=0;
	}
	else if ( t < t7 ) {
		x = ((cornerRadius+cornerKickRadius)*12)*cos((t-t6)/(t7-t6)*3*M_PI/2)-(cornerRadius*12);
		y = ((cornerRadius+cornerKickRadius)*12)*sin((t-t6)/(t7-t6)*3*M_PI/2)+(startingDistance+fieldLength+cornerRadius+cornerKickRadius)*12;
		p=0;
	}
	else if ( t < t8 ) {
		x = -cornerRadius*12+desiredVelocity*(t-t7);
		y = (startingDistance+fieldLength)*12;
		p=0;
	}
	else if ( t < t9 ) {
		x = desiredVelocity*(t-t8);
		y = (startingDistance+fieldLength)*12;
		p=1;
	}
	else if ( t < t10 ) {
		x = desiredVelocity*(t-t9)+fieldWidth*12;
		y = (startingDistance+fieldLength)*12;
		p=0;
	}
	else if ( t < t11 ) {
		x = (cornerRadius+cornerKickRadius)*12*sin((t-t10)/(t11-t10)*3*M_PI/2)+(fieldWidth+cornerRadius)*12;
		y = -(cornerRadius+cornerKickRadius)*12*cos((t-t10)/(t11-t10)*3*M_PI/2)+(cornerRadius+cornerKickRadius)*12+((startingDistance+fieldLength)*12);
		p=0;
	}
	else if ( t < t12 ) {
		x = (fieldWidth-cornerKickRadius)*12;
		y = (startingDistance+fieldLength+cornerRadius+cornerKickRadius)*12-(desiredVelocity*(t-t11));
		p=0;
	}
	else if ( t < t13 ) {
		x = -(cornerKickRadius*12)*cos((t-t12)/(t13-t12)*M_PI/2)+(fieldWidth*12);
		y = -(cornerKickRadius*12)*sin((t-t12)/(t13-t12)*M_PI/2)+(startingDistance+fieldLength)*12;
		p=1;
	}
	else if ( t < t14 ) {
		x = (fieldWidth*12)+desiredVelocity*(t-t13);
		y = (startingDistance+fieldLength-cornerKickRadius)*12;
		p=0;
	}
	else if ( t < t15 ) {
		x = (cornerRadius+cornerKickRadius)*12*sin((t-t14)/(t15-t14)*3*M_PI/2)+(fieldWidth+cornerRadius+cornerKickRadius)*12;
		y = -(cornerRadius+cornerKickRadius)*12*cos((t-t14)/(t15-t14)*3*M_PI/2)+(cornerRadius)*12+((startingDistance+fieldLength)*12);
 		p=0;
	}
	else if ( t < t16 ) {
		x = fieldWidth*12;
		y = (startingDistance+fieldLength+cornerRadius)*12 - desiredVelocity*(t-t15);
		p=0;
	}
	else if ( t < t17 ) {
		x = fieldWidth*12;
		y = (startingDistance+fieldLength)*12-(desiredVelocity*(t-t16));
		p=1;
	}
	else if ( t < t18 ) {
		x = fieldWidth*12;
		y = (startingDistance)*12-(desiredVelocity*(t-t17));
		p=0;
	}
	else if ( t < t19 ) {
		x = -(cornerRadius+cornerKickRadius)*12*cos((t-t18)/(t19-t18)*3*M_PI/2)+(cornerRadius+cornerKickRadius)*12+(fieldWidth*12);
		y = -(cornerRadius+cornerKickRadius)*12*sin((t-t18)/(t19-t18)*3*M_PI/2)-(cornerRadius*12)+(startingDistance*12);
		p=0;
	}
	else if ( t < t20 ) {
		x = (fieldWidth+cornerKickRadius+cornerRadius)*12-desiredVelocity*(t-t19);
		y = (startingDistance+cornerKickRadius)*12;
		p=0;
	}
	else if ( t < t21 ) {
		x = -(cornerKickRadius*12)*sin((t-t20)/(t21-t20)*M_PI/2)+(fieldWidth*12);
		y = (cornerKickRadius*12)*cos((t-t20)/(t21-t20)*M_PI/2)+(startingDistance*12);
		p=1;
	}
	else if ( t < t22 ) {
		x = (fieldWidth-cornerKickRadius)*12;
		y = (startingDistance*12)-desiredVelocity*(t-t21);
		p=0;
	}
	else if ( t < t23 ) {
		x = -(cornerKickRadius+cornerRadius)*12*cos((t-t22)/(t23-t22)*3*M_PI/2)+(fieldWidth+cornerRadius)*12;
		y = -(cornerKickRadius+cornerRadius)*12*sin((t-t22)/(t23-t22)*3*M_PI/2)+(startingDistance-cornerKickRadius-cornerRadius)*12; 
		p=0;
	}
	else if ( t < t24 ) {
		x = (fieldWidth+cornerRadius)*12-(desiredVelocity*(t-t23));
		y = startingDistance*12;
		p=0;
	}
	else if ( t < t25 ) {
		x = (fieldWidth*12)-(desiredVelocity*(t-t24));
		y = startingDistance*12;
		p=1;
	}
	else if ( t < t26 ) {
		x = -desiredVelocity*(t-t25);
		y = startingDistance*12;
		p=0;
	}
	else if ( t < t27 ) {
		x = -(cornerRadius+cornerKickRadius)*12*sin((t-t26)/(t27-t26)*3*M_PI/2)-(cornerRadius*12);
		y = (cornerRadius+cornerKickRadius)*12*cos((t-t26)/(t27-t26)*3*M_PI/2)+(startingDistance*12)-(cornerRadius+cornerKickRadius)*12;
		p=0;
	}
	else if ( t < t28 ) {
		x = cornerKickRadius*12;
		y = (startingDistance-cornerRadius-cornerKickRadius)*12 + desiredVelocity*(t-t27);
		p=0;
	}
	else if ( t < t29 ) {
		x = (cornerKickRadius*12)*cos((t-t28)/(t29-t28)*M_PI/2);
		y = (cornerKickRadius*12)*sin((t-t28)/(t29-t28)*M_PI/2)+startingDistance*12;
		p=1;
	}
	else if ( t < t30 ) {
		x = -desiredVelocity*(t-t29);
		y = (startingDistance+cornerKickRadius)*12;
		p=0;
	}
	else if ( t < t31 ) {
		x = -(cornerRadius*12)*sin((t-t30)/(t31-t30)*M_PI/2)-(cornerRadius*12);
		y = -(cornerRadius*12)*cos((t-t30)/(t31-t30)*M_PI/2)+(startingDistance+cornerKickRadius+cornerRadius)*12;
		p=0;
	}
	else if ( t < t32 ) {
		x = -(2*cornerRadius*12);
		y = ((startingDistance+cornerRadius+cornerKickRadius)*12)+desiredVelocity*(t-t31);
		p=0;
	}
	else if ( t < t33 ) {
		x = -(cornerRadius*12)*cos((t-t32)/(t33-t32)*M_PI/2)-(cornerRadius*12);
		y = (cornerRadius*12)*sin((t-t32)/(t33-t32)*M_PI/2)+(startingDistance-cornerRadius+fieldLength/2)*12;
		p=0;
	}
	else if ( t < t34 ) {
		x = -(cornerRadius*12)+desiredVelocity*(t-t33);
		y = (startingDistance+fieldLength/2)*12;
		p=0;
	}
	else if ( t < t35 ) {
		x = desiredVelocity*(t-t34);
		y = (startingDistance+fieldLength/2)*12;
		p=1;
	}
	else if ( t < t36 ) {
		x = fieldWidth*12 + (desiredVelocity*(t-t35));
		y = (startingDistance+fieldLength/2)*12;
		p=0;
	}
	else if ( t < t37 ) {
		x = (cornerRadius*12)*sin((t-t36)/(t37-t36)*M_PI/2)+(fieldWidth+cornerRadius)*12;
		y = -(cornerRadius*12)*cos((t-t36)/(t37-t36)*M_PI/2)+(startingDistance+fieldLength/2+cornerRadius)*12;
		p=0;
	}
	else if ( t < t38 ) {
		x = (fieldWidth+cornerRadius*2)*12;
		y = (startingDistance+fieldLength/2+cornerRadius)*12+desiredVelocity*(t-t37);
		p=0;
	}
	else if ( t < t39 ) {
		x = (cornerRadius*12)*cos((t-t38)/(t39-t38)*M_PI/2)+(fieldWidth+cornerRadius)*12;
		y = (cornerRadius*12)*sin((t-t38)/(t39-t38)*M_PI/2)+(fieldLength+startingDistance-penaltyAreaLength-cornerRadius)*12;
		p=0;
	}
	else if ( t < t40 ) {
		x = (cornerRadius+fieldWidth)*12 - desiredVelocity*(t-t39);
		y = (startingDistance+fieldLength-penaltyAreaLength)*12;
		p=0;
	}
	else if ( t < t41 ) {
		x = (fieldWidth/2+penaltyAreaWidth/2)*12-desiredVelocity*(t-t40);
		y = (startingDistance+fieldLength-penaltyAreaLength)*12;
		p=1;
	}
	else if ( t < t42 ) {
		x = (fieldWidth/2-penaltyAreaWidth/2)*12-desiredVelocity*(t-t41);
		y = (startingDistance+fieldLength-penaltyAreaLength)*12;
		p=0;
	}
	else if ( t < t43 ) {
		x = -(cornerRadius*12)*sin((t-t42)/(t43-t42)*3*M_PI/2)+(fieldWidth/2-penaltyAreaWidth/2-cornerRadius)*12;
		y = (cornerRadius*12)*cos((t-t42)/(t43-t42)*3*M_PI/2)+(startingDistance+fieldLength-penaltyAreaLength-cornerRadius)*12;
		p=0;
	}
	else if ( t < t44 ) {
		x = (fieldWidth/2-penaltyAreaWidth/2)*12;
		y = (startingDistance+fieldLength-penaltyAreaLength-cornerRadius)*12+desiredVelocity*(t-t43);
		p=0;
	}
	else if ( t < t45 ) {
		x = (fieldWidth/2-penaltyAreaWidth/2)*12;
		y = (startingDistance+fieldLength-penaltyAreaLength)*12+desiredVelocity*(t-t44);
		p=1;
	}
	else if ( t < t46 ) {
		x = (fieldWidth/2-penaltyAreaWidth/2)*12;
		y = (startingDistance+fieldLength)*12+desiredVelocity*(t-t45);
		p=0;
	}
	else if ( t < t47 ) {
		x = -((penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12)*cos((t-t46)/(t47-t46)*M_PI/2)+(fieldWidth/2-penaltyAreaWidth/2+(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		y = ((penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12)*sin((t-t46)/(t47-t46)*M_PI/2)+(startingDistance+fieldLength+(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		p=0;
	}
	else if ( t < t48 ) {
		x = (fieldWidth/2-penaltyAreaWidth/2+(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12+desiredVelocity*(t-t47);
		y = (fieldLength+2*(penaltyAreaWidth/2-penaltyBoxWidth/2)/2+startingDistance)*12;
		p=0;
	}
	else if ( t < t49 ) {
		x = ((penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12)*sin((t-t48)/(t49-t48)*M_PI/2)+(fieldWidth/2-penaltyBoxWidth/2-(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		y = ((penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12)*cos((t-t48)/(t49-t48)*M_PI/2)+(fieldLength+(penaltyAreaWidth/2-penaltyBoxWidth/2)/2+startingDistance)*12;
		p=0;
	}
	else if ( t < t50 ) {
		x = (fieldWidth/2-penaltyBoxWidth/2)*12;
		y = (startingDistance+fieldLength+(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12-desiredVelocity*(t-t49);
		p=0;
	}
	else if ( t < t51 ) {
		x = (fieldWidth/2-penaltyBoxWidth/2)*12;
		y = (startingDistance+fieldLength)*12-desiredVelocity*(t-t50);
		p=1;
	}
	else if ( t < t52 ) {
		x = (fieldWidth/2-penaltyBoxWidth/2)*12;
		y = (startingDistance+fieldLength-penaltyBoxLength)*12-desiredVelocity*(t-t51);
		p=0;
	}
	else if ( t < t53 ) {
		x = (cornerRadius*12)*cos((t-t52)/(t53-t52)*3*M_PI/2)+(fieldWidth/2-penaltyBoxWidth/2-cornerRadius)*12;
		y = -(cornerRadius*12)*sin((t-t52)/(t53-t52)*3*M_PI/2)+(startingDistance+fieldLength-penaltyBoxLength-cornerRadius)*12;
		p=0;
	}
	else if ( t < t54 ) {
		x = (fieldWidth/2-penaltyBoxWidth/2-cornerRadius)*12+desiredVelocity*(t-t53);
		y = (fieldLength+startingDistance-penaltyBoxLength)*12;
		p=0;
	}
	else if ( t < t55 ) {
		x = (fieldWidth/2-penaltyBoxWidth/2)*12+desiredVelocity*(t-t54);
		y = (fieldLength+startingDistance-penaltyBoxLength)*12;
		p=1;
	}
	else if ( t < t56 ) {
		x = (fieldWidth/2+penaltyBoxWidth/2)*12+desiredVelocity*(t-t55);
		y = (fieldLength+startingDistance-penaltyBoxLength)*12;
		p=0;
	}
	else if ( t < t57 ) {
		x = (cornerRadius*12)*sin((t-t56)/(t57-t56)*3*M_PI/2)+(fieldWidth/2+penaltyBoxWidth/2+cornerRadius)*12;
		y = (cornerRadius*12)*cos((t-t56)/(t57-t56)*3*M_PI/2)+(fieldLength+startingDistance-penaltyBoxLength-cornerRadius)*12;
		p=0;
	}
	else if ( t < t58 ) {
		x = (fieldWidth/2+penaltyBoxWidth/2)*12;
		y = (fieldLength+startingDistance-penaltyBoxLength-cornerRadius)*12+desiredVelocity*(t-t57);
		p=0;
	}
	else if ( t < t59 ) {
		x = (fieldWidth/2+penaltyBoxWidth/2)*12;
		y = (fieldLength+startingDistance-penaltyBoxLength)*12+desiredVelocity*(t-t58);
		p=1;
	}
	else if ( t < t60 ) {
		x = (fieldWidth/2+penaltyBoxWidth/2)*12;
		y = (fieldLength+startingDistance)*12+desiredVelocity*(t-t59);
		p=0;
	}
	else if ( t < t61 ) {
		x = -((penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12)*cos((t-t60)/(t61-t60)*M_PI/2)+(fieldWidth/2+penaltyBoxWidth/2+(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		y = ((penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12)*sin((t-t60)/(t61-t60)*M_PI/2)+(startingDistance+fieldLength+(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		p=0;
	}
	else if ( t < t62 ) {
		x = (fieldWidth/2+penaltyBoxWidth/2+(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12+desiredVelocity*(t-t61);
		y = (fieldLength+startingDistance+2*(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		p=0;
	}
	else if ( t < t63 ) {
		x = ((penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12)*sin((t-t62)/(t63-t62)*M_PI/2)+(fieldWidth/2+penaltyAreaWidth/2-(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		y = ((penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12)*cos((t-t62)/(t63-t62)*M_PI/2)+(fieldLength+startingDistance+(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		p=0;
	}
	else if ( t < t64 ) {
		x = (fieldWidth/2+penaltyAreaWidth/2)*12;
		y = (fieldLength+startingDistance+(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12-desiredVelocity*(t-t63);
		p=0;
	}
	else if ( t < t65 ) {
		x = (fieldWidth/2+penaltyAreaWidth/2)*12;
		y = (fieldLength+startingDistance)*12-desiredVelocity*(t-t64);
		p=1;
	}
	else if ( t < t66 ) {
		x = (fieldWidth/2+penaltyAreaWidth/2)*12;
		y = (fieldLength+startingDistance-penaltyAreaLength)*12-desiredVelocity*(t-t65);
		p = 0;
	}
	else if ( t < t67 ) {
		x = -(cornerRadius*12)*cos((t-t66)/(t67-t66)*M_PI)+(fieldWidth/2+penaltyAreaWidth/2+cornerRadius)*12;
		y = -(cornerRadius*12)*sin((t-t66)/(t67-t66)*M_PI)+(startingDistance+fieldLength-penaltyAreaLength-cornerRadius)*12;
		p = 0;
	}
	else if ( t < t68 ) {
		x = (fieldWidth/2+penaltyAreaWidth/2+2*cornerRadius)*12;
		y = (startingDistance+fieldLength-penaltyAreaLength-cornerRadius)*12+desiredVelocity*(t-t67);
		p=0;
	}
	else if ( t < t69 ) {
		x = (cornerRadius*12)*cos((t-t68)/(t69-t68)*M_PI/2)+(fieldWidth/2+penaltyAreaWidth/2+cornerRadius)*12;
		y = (cornerRadius*12)*sin((t-t68)/(t69-t68)*M_PI/2)+(startingDistance+fieldLength-penaltyCenter)*12;
		p=0;
	}
	else if ( t < t70 ) {
		x = (fieldWidth/2+penaltyAreaWidth/2+cornerRadius)*12-desiredVelocity*(t-t69);
		y = (startingDistance+fieldLength-penaltyCenter+cornerRadius)*12;
		p=0;
	}
	else if ( t < t71 ) {
		x = -(cornerRadius*12)*sin((t-t70)/(t71-t70)*M_PI/2)+(fieldWidth/2+penaltyRadius+cornerRadius)*12;
		y = (cornerRadius*12)*cos((t-t70)/(t71-t70)*M_PI/2)+(startingDistance+fieldLength-penaltyCenter)*12;
		p=0;
	}
	else if ( t < t72 ) {
		x = (penaltyRadius*12)*cos((t-t71)/(t72-t71)*asin((penaltyAreaLength-penaltyCenter)/penaltyRadius))+(fieldWidth/2)*12;
		y = -(penaltyRadius*12)*sin((t-t71)/(t72-t71)*asin((penaltyAreaLength-penaltyCenter)/penaltyRadius))+(startingDistance+fieldLength-penaltyCenter)*12;
		p=0;
	}
	else if ( t < t73 ) {
		x = (penaltyRadius*12)*cos((t-t72)/(t73-t72)*2*acos((penaltyAreaLength-penaltyCenter)/penaltyRadius)+asin((penaltyAreaLength-penaltyCenter)/penaltyRadius))+(fieldWidth/2)*12;
		y = -(penaltyRadius*12)*sin((t-t72)/(t73-t72)*2*acos((penaltyAreaLength-penaltyCenter)/penaltyRadius)+asin((penaltyAreaLength-penaltyCenter)/penaltyRadius))+(startingDistance+fieldLength-penaltyCenter)*12;
		p=1;
	}
	else if ( t < t74 ) {
		x = (penaltyRadius*12)*cos((t-t73)/(t74-t73)*asin((penaltyAreaLength-penaltyCenter)/penaltyRadius)+asin((penaltyAreaLength-penaltyCenter)/penaltyRadius)+2*acos((penaltyAreaLength-penaltyCenter)/penaltyRadius))+(fieldWidth/2)*12;
		y = -(penaltyRadius*12)*sin((t-t73)/(t74-t73)*asin((penaltyAreaLength-penaltyCenter)/penaltyRadius)+asin((penaltyAreaLength-penaltyCenter)/penaltyRadius)+2*acos((penaltyAreaLength-penaltyCenter)/penaltyRadius))+(startingDistance+fieldLength-penaltyCenter)*12;
		p=0;
	}
	else if ( t < t75 ) {
		x = (cornerRadius*12)*cos((t-t74)/(t75-t74)*3*M_PI/2)+(fieldWidth/2-penaltyRadius-cornerRadius)*12;
		y = (cornerRadius*12)*sin((t-t74)/(t75-t74)*3*M_PI/2)+(startingDistance+fieldLength-penaltyCenter)*12;
		p=0;
	}
	else if ( t < t76 ) {
		x = (cornerRadius*12)*sin((t-t75)/(t76-t75)*M_PI/2)+(fieldWidth/2-penaltyRadius-cornerRadius)*12;
		y = (cornerRadius*12)*cos((t-t75)/(t76-t75)*M_PI/2)+(startingDistance+fieldLength-penaltyCenter-2*cornerRadius)*12;
		p=0;
	}
	else if ( t < t77 ) {
		x = (fieldWidth/2-centerRadius)*12;
		y = (startingDistance+fieldLength-penaltyCenter-2*cornerRadius)*12-desiredVelocity*(t-t76);
		p=0;
	}
	else if ( t < t78 ) {
		x = -(centerRadius*12)*cos((t-t77)/(t78-t77)*2*M_PI)+(fieldWidth/2)*12;
		y = -(centerRadius*12)*sin((t-t77)/(t78-t77)*2*M_PI)+(startingDistance+fieldLength/2)*12;
		p=1;
	}
	else if ( t < t79 ) {
		x = (fieldWidth/2-centerRadius)*12;
		y = (startingDistance+fieldLength/2)*12-desiredVelocity*(t-t78);
		p=0;
	}
	else if ( t < t80 ) {
		x = (fieldWidth/2-centerRadius)*12;
		y = (startingDistance+penaltyBoxLength)*12-desiredVelocity*(t-t79);
		p=1;
	}
	else if ( t < t81 ) {
		x = (fieldWidth/2-centerRadius)*12;
		y = (startingDistance*12)-desiredVelocity*(t-t80);
		p=0;
	}
	else if ( t < t82 ) {
		x = ((penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12)*cos((t-t81)/(t82-t81)*M_PI/2)+(fieldWidth/2-centerRadius-(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		y = -((penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12)*sin((t-t81)/(t82-t81)*M_PI/2)+(startingDistance-(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		p=0;
	}
	else if ( t < t83 ) {
		x = (fieldWidth/2-penaltyBoxWidth/2-(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12-desiredVelocity*(t-t82);
		y = (startingDistance-2*(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		p=0;
	}
	else if ( t < t84 ) {
		x = -((penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12)*sin((t-t83)/(t84-t83)*M_PI/2)+(fieldWidth/2-penaltyAreaWidth/2+(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		y = -((penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12)*cos((t-t83)/(t84-t83)*M_PI/2)+(startingDistance-(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		p=0;
	}
	else if ( t < t85 ) {
		x = (fieldWidth/2-penaltyAreaWidth/2)*12;
		y = (startingDistance-(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12+desiredVelocity*(t-t84);
		p=0;
	}
	else if ( t < t86 ) {
		x = (fieldWidth/2-penaltyAreaWidth/2)*12;
		y = (startingDistance*12)+desiredVelocity*(t-t85);
		p=1;
	}
	else if ( t < t87 ) {
		x = (fieldWidth/2-penaltyAreaWidth/2)*12;
		y = (startingDistance+penaltyAreaLength)*12+desiredVelocity*(t-t86);
		p=0;
	}
	else if ( t < t88 ) {
		x = (cornerRadius*12)*cos((t-t87)/(t88-t87)*3*M_PI/2)+(fieldWidth/2-penaltyAreaWidth/2-cornerRadius)*12;
		y = (cornerRadius*12)*sin((t-t87)/(t88-t87)*3*M_PI/2)+(startingDistance+penaltyAreaLength+cornerRadius)*12;
		p=0;
	}
	else if ( t < t89 ) {
		x = (fieldWidth/2-penaltyAreaWidth/2-cornerRadius)*12+desiredVelocity*(t-t88);
		y = (startingDistance+penaltyAreaLength)*12;
		p=0;
	}
	else if ( t < t90 ) {
		x = (fieldWidth/2-penaltyAreaWidth/2)*12+desiredVelocity*(t-t89);
		y = (startingDistance+penaltyAreaLength)*12;
		p=1;
	}
	else if ( t < t91 ) {
		x = (fieldWidth/2+penaltyAreaWidth/2)*12+desiredVelocity*(t-t90);
		y = (startingDistance+penaltyAreaLength)*12;
		p=0;
	}
	else if ( t < t92 ) {
		x = (cornerRadius*12)*sin((t-t91)/(t92-t91)*3*M_PI/2)+(fieldWidth/2+penaltyAreaWidth/2+cornerRadius)*12;
		y = -(cornerRadius*12)*cos((t-t91)/(t92-t91)*3*M_PI/2)+(startingDistance+penaltyAreaLength+cornerRadius)*12;
		p=0;
	}
	else if ( t < t93 ) {
		x = (fieldWidth/2+penaltyAreaWidth/2)*12;
		y = (startingDistance+penaltyAreaLength+cornerRadius)*12-desiredVelocity*(t-t92);
		p=0;
	}
	else if ( t < t94 ) {
		x = (fieldWidth/2+penaltyAreaWidth/2)*12;
		y = (startingDistance+penaltyAreaLength)*12-desiredVelocity*(t-t93);
		p=1;
	}
	else if ( t < t95 ) {
		x = (fieldWidth/2+penaltyAreaWidth/2)*12;
		y = (startingDistance*12)-desiredVelocity*(t-t94);
		p=0;
	}
	else if ( t < t96 ) {
		x = ((penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12)*cos((t-t95)/(t96-t95)*M_PI/2)+(fieldWidth/2+penaltyAreaWidth/2-(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		y = -((penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12)*sin((t-t95)/(t96-t95)*M_PI/2)+(startingDistance-(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		p=0;
	}
	else if ( t < t97 ) {
		x = (fieldWidth/2+penaltyAreaWidth/2-(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12-desiredVelocity*(t-t96);
		y = (startingDistance-2*(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		p=0;
	}
	else if ( t < t98 ) {
		x = -((penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12)*sin((t-t97)/(t98-t97)*M_PI/2)+(fieldWidth/2+penaltyBoxWidth/2+(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		y = -((penaltyAreaWidth/2-penaltyBoxWidth/2)/2*12)*cos((t-t97)/(t98-t97)*M_PI/2)+(startingDistance-(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12;
		p=0;
	}
	else if ( t < t99 ) {
		x = (fieldWidth/2+penaltyBoxWidth/2)*12;
		y = (startingDistance-(penaltyAreaWidth/2-penaltyBoxWidth/2)/2)*12+desiredVelocity*(t-t98);
		p=0;
	}
	else if ( t < t100 ) {
		x = (fieldWidth/2+penaltyBoxWidth/2)*12;
		y = (startingDistance)*12+desiredVelocity*(t-t99);
		p=1;
	}
	else if ( t < t101 ) {
		x = (fieldWidth/2+penaltyBoxWidth/2)*12;
		y = (startingDistance+penaltyBoxLength)*12+desiredVelocity*(t-t100);
		p=0;
	}
	else if ( t < t102 ) {
		x = -(cornerRadius*12)*cos((t-t101)/(t102-t101)*3*M_PI/2)+(fieldWidth/2+penaltyBoxWidth/2+cornerRadius)*12;
		y = (cornerRadius*12)*sin((t-t101)/(t102-t101)*3*M_PI/2)+(startingDistance+penaltyBoxLength+cornerRadius)*12;
		p=0;
	}
	else if ( t < t103 ) {
		x = (fieldWidth/2+penaltyBoxWidth/2+cornerRadius)*12-desiredVelocity*(t-t102);
		y = (startingDistance+penaltyBoxLength)*12;
		p=0;
	}
	else if ( t < t104 ) {
		x = (fieldWidth/2+penaltyBoxWidth/2)*12-desiredVelocity*(t-t103);
		y = (startingDistance+penaltyBoxLength)*12;
		p=1;
	}
	else if ( t < t105 ) {
		x = (fieldWidth/2-penaltyBoxWidth/2)*12-desiredVelocity*(t-t104);
		y = (startingDistance+penaltyBoxLength)*12;
		p=0;
	}
	else if ( t < t106 ) {
		x = -(cornerRadius*12)*sin((t-t105)/(t106-t105)*3*M_PI/2)+(fieldWidth/2-penaltyBoxWidth/2-cornerRadius)*12;
		y = (cornerRadius*12)*cos((t-t105)/(t106-t105)*3*M_PI/2)+(startingDistance+penaltyBoxLength-cornerRadius)*12;
		p=0;
	}
	else if ( t < t107 ) {
		x = (fieldWidth/2-penaltyBoxWidth/2)*12;//(cornerRadius*12)*sin((t-t106)/(t107-t106)*M_PI/2)+(fieldWidth/2-penaltyBoxWidth/2-cornerRadius)*12;
		y = (startingDistance+penaltyBoxLength-cornerRadius)*12+desiredVelocity*(t-t106);//-(cornerRadius*12)*cos((t-t106)/(t107-t106)*M_PI/2)+(startingDistance+penaltyBoxLength+3*cornerRadius)*12;
		p=0;
	}
	else if ( t < t108 ) {
		x = (fieldWidth/2-penaltyBoxWidth/2)*12;
		y = (startingDistance+penaltyBoxLength+3*cornerRadius)*12+desiredVelocity*(t-t107);
		p=0;
	}
	else if ( t < t109 ) {
		x = -(penaltyRadius*12)*cos((t-t108)/(t109-t108)*asin((penaltyAreaLength-penaltyCenter)/penaltyRadius))+(fieldWidth/2)*12;
		y = (penaltyRadius*12)*sin((t-t108)/(t109-t108)*asin((penaltyAreaLength-penaltyCenter)/penaltyRadius))+(startingDistance+penaltyCenter)*12;
		p=0;
	}
	else if ( t < t110 ) {
		x = -(penaltyRadius*12)*cos((t-t109)/(t110-t109)*2*acos((penaltyAreaLength-penaltyCenter)/penaltyRadius)+asin((penaltyAreaLength-penaltyCenter)/penaltyRadius))+(fieldWidth/2)*12;
		y = (penaltyRadius*12)*sin((t-t109)/(t110-t109)*2*acos((penaltyAreaLength-penaltyCenter)/penaltyRadius)+asin((penaltyAreaLength-penaltyCenter)/penaltyRadius))+(startingDistance+penaltyCenter)*12;
		p=1;
	}
	else if ( t < t111 ) {
		x = -(penaltyRadius*12)*cos((t-t110)/(t111-t110)*asin((penaltyAreaLength-penaltyCenter)/penaltyRadius)+asin((penaltyAreaLength-penaltyCenter)/penaltyRadius)+2*acos((penaltyAreaLength-penaltyCenter)/penaltyRadius))+(fieldWidth/2)*12;
		y = (penaltyRadius*12)*sin((t-t110)/(t111-t110)*asin((penaltyAreaLength-penaltyCenter)/penaltyRadius)+asin((penaltyAreaLength-penaltyCenter)/penaltyRadius)+2*acos((penaltyAreaLength-penaltyCenter)/penaltyRadius))+(startingDistance+penaltyCenter)*12;
		p=0;
	}
	else if ( t < t112 ) {
		x = (fieldWidth/2+penaltyBoxWidth/2)*12;
		y = (startingDistance+penaltyCenter)*12-desiredVelocity*(t-t111);
		p=0;
	}
	else if ( t < t113 ) {
		x = ((penaltyBoxWidth/4)*12)*cos((t-t112)/(t113-t112)*M_PI/2)+(fieldWidth/2+penaltyBoxWidth/2-(penaltyBoxWidth/4))*12;
		y = -((penaltyBoxWidth/4)*12)*sin((t-t112)/(t113-t112)*M_PI/2)+(startingDistance+penaltyCenter-markLength/2)*12;
		p=0;
	}
	else if ( t < t114 ) {
		x = (fieldWidth/2+penaltyBoxWidth/2-(penaltyBoxWidth/4))*12-desiredVelocity*(t-t113);
		y = (penaltyCenter+startingDistance-markLength/2-(penaltyBoxWidth/4))*12;
		p=0;
	}
	else if ( t < t115 ) {
		x = -((penaltyBoxWidth/4)*12)*sin((t-t114)/(t115-t114)*M_PI/2)+(fieldWidth/2+(penaltyBoxWidth/4))*12;
		y = -((penaltyBoxWidth/4)*12)*cos((t-t114)/(t115-t114)*M_PI/2)+(startingDistance+penaltyCenter-markLength/2)*12;
		p=0;
	}
	else if ( t < t116 ) {
		x = (fieldWidth/2)*12;
		y = (startingDistance+penaltyCenter-markLength/2)*12+desiredVelocity*(t-t115);
		p=1;
	}
	else if ( t < t117 ) {
		x = (fieldWidth/2)*12;
		y = (startingDistance+penaltyCenter+markLength/2)*12+desiredVelocity*(t-t116);
		p=0;
	}
	else if ( t < t118 ) {
		x = (fieldWidth/2)*12;
		y = (startingDistance+fieldLength/2-markLength/2)*12+desiredVelocity*(t-t117);
		p=1;
	}
	else if ( t < t119 ) {
		x = (fieldWidth/2)*12;
		y = (startingDistance+fieldLength/2+markLength/2)*12+desiredVelocity*(t-t118);
		p=0;
	}
	else if ( t < t120 ) {
		x = (fieldWidth/2)*12;
		y = (startingDistance+fieldLength-penaltyCenter-markLength/2)*12+desiredVelocity*(t-t119);
		p=1;
	}
	else {
		x = (fieldWidth/2)*12;
		y = (startingDistance+fieldLength-penaltyCenter+markLength)*12;
		p=0;
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
double*  getOptimalVelocities(Pose*** projectedPaths, int _vNumberOfEntries, int _wNumberOfEntries, int _numPathPoints ,vector<double> xDesiredPath, vector<double> yDesiredPath, vector<double> paintDesiredPath)
{
	
	double* velocities = new double[2];
	double errorCurrent, errorMin;
	int _vMinIndex, _wMinIndex;

	//if(verbose) cout << "First y point: " << yDesiredPath[0] << endl;
	
	vector<Pose> goalPath(xDesiredPath.size());
	pathToRobotFrame(xDesiredPath, yDesiredPath, paintDesiredPath, goalPath);

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
bool pathToRobotFrame(vector<double> projectedPathX, vector<double> projectedPathY, vector<double> projectedPathPaint, vector<Pose> & newProjectedPath) {
	
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

/////////////////////////////////////////////////////////////////////////////////////
//calibrateLeica:
//drives a straight line in order to calibrate the initial theta in the Leica's frame of reference

bool calibrateLeica(char* dataString, float leicaTimeStamp)
{
	// Get the first point of the orientation calibration
	while(readPort(dataString)==0){
		cout << "Calibrating Origin" << endl;
		usleep(100000);
	}

	testData = leicaStoF(dataString);
	sphericalToPlanar(testData[2], testData[3], testData[4]);

	while(sqrt(pow((location[0]-calibrateTheta[0]),2)+pow((location[1]-calibrateTheta[1]),2))<50) {

		sendVelocityCommands(10,0);

		// Get 2nd point to calibrate theta
		cout << "Waiting for 2nd point " << endl;
		while(readPort(dataString)==0) {	
			sendVelocityCommands(10,0);
			cout << "Calibrating Theta" << endl;
			usleep(10000);
		}
		previousRadius = 0;

		testData = leicaStoF(dataString);
		sphericalToPlanar(testData[2], testData[3], testData[4]);
	}

	sendVelocityCommands(0,0);		

	thetaOrigin = atan2((location[1]-calibrateTheta[1]),(location[0]-calibrateTheta[0]));
	//cout << "First Point: " << calibrateTheta[0] << ", " << calibrateTheta[1] << " Timestamp: " << fmod(leicaTimeStamp,1.0) << endl;
	//cout << "Second Point: " << location[0] << ", " << location[1] << " Timestamp: " << fmod(testData[0],1.0) <<endl;
	thetaOrigin -= M_PI/2;
	thetaOrigin *= -1;
	cout << "thetaOrigin:" << thetaOrigin << endl;	
	// Reset the origin at the last data point from calibration.  
	xOrigin = location[0];
	yOrigin = location[1];

	return true;	
}

//////////////////////////////////////////
// Get Heading reading from IMU
/////////////////////////////////////////
void *getHeading(void *threadarg) { // myI2C *i2cptr, float & heading) {        
        while(true) {
                float scaleX=0.92;
                float scaleY=0.92; //setting scale for magnetometer
                bool dataReady = true;

                struct thread_data *args;
                args = (struct thread_data *) threadarg;

                myI2C *i2cptr = args->i2cptr;

                try {
                        i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_MODE_REG,0X00);
                }
                catch(...) {
                        dataReady = false;
                }
                usleep(67000);

                //read data from magnetometer   
                char valuemxH;
                char valuemxL;
                char valuemyH;
                char valuemyL;
		if(dataReady) {
			try {
				valuemxH=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_X_MSB);
				valuemxL=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_X_LSB);
				valuemyH=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_Y_MSB);
				valuemyL=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_Y_LSB);
			}
			catch(...) {
				dataReady = false;
			}
		}
                if(dataReady) {
                        signed int valuemxHsi=valuemxH<<8 | valuemxL;
                        signed int valuemyHsi=valuemyH<<8|valuemyL;
        
                        int valuemxHi=i2cptr->twosc2int(valuemxHsi);
                        int valuemyHi=i2cptr->twosc2int(valuemyHsi);
                        float valuemxHf=valuemxHi*scaleX;
                        float valuemyHf=valuemyHi*scaleY;       
                        //caluclate the heading angle
        
                        float heading=atan2(valuemyHf,valuemxHf);
                        float declinationAngle = 0.1614;
                        heading += declinationAngle;
			
			heading -= (headingOffset*M_PI/180);
                        if(heading < 0)
                                heading += 2*M_PI;
                        if(heading > 2*M_PI)
                                heading -= 2*M_PI;

                        // FMOD?
                        float headingDegrees = heading * 180/M_PI;
			if(calibrateTheIMU) {
				headingOffset = headingDegrees;
				calibrateTheIMU = false;
			}
                        //headingDegrees=headingDegrees+headingOffset;
                        if (headingDegrees<0)
                                headingDegrees+=360;
                        //cout <<"Heading:" << headingDegrees <<endl;                   
                        cout << "Heading: " << heading << endl;
			cout << "Degrees: " << headingDegrees << " " << currentTime << endl;

                        headingIMU = heading;
                }
                if(robotIsDone) break;
                usleep(67000);
        }
        pthread_exit(NULL);
}

///////////////////////////////////////////////////////////////////////////////
void runKalman(double rightDeltaPhi, double leftDeltaPhi)
{
	// Calculate Propogation
	F << 1, 0, 0, cos(X(2))*diffTime, -sin(X(2))*diffTime, 0,
	     0, 1, 0, sin(X(2))*diffTime, cos(X(2))*diffTime, 0,
	     0, 0, 1, 0, 0, diffTime,
	     0, 0, 0, 1, 0, 0,
	     0, 0, 0, 0, 1, 0,
	     0, 0, 0, 0, 0, 1;

	//Kalman Filter for Leica Data
	if (newLeicaData && !newIMUData) {
		// Initialize data dependent variables
		Eigen::MatrixXd H(5,6);
		Eigen::VectorXd y(5);
		Eigen::VectorXd z(5); 
		Eigen::MatrixXd K(6,5);
		
		// The measurement, including leica
		z << rightDeltaPhi/diffTime, leftDeltaPhi/diffTime, location[0], location[1], location[2];
		cout << "z = " << endl << z << endl;
        
		//Get model measurement
	        y << sqrt(pow(X(3),2)+pow(X(4),2))/wheelRadius + X(5)*botRadius/wheelRadius,
		     sqrt(pow(X(3),2)+pow(X(4),2))/wheelRadius - X(5)*botRadius/wheelRadius,
		     X(0),
		     X(1),
		     X(2);
		
		// Jacobian assumes leica is linear
		H << 0, 0, 0, -sin(X(5)*diffTime)/wheelRadius, cos(X(5)*diffTime)/wheelRadius, botRadius/wheelRadius,
		     0, 0, 0, -sin(X(5)*diffTime)/wheelRadius, cos(X(5)*diffTime)/wheelRadius, -botRadius/wheelRadius,
		     1, 0, 0, 0, 0, 0,
		     0, 1, 0, 0, 0, 0,
		     0, 0, 1, 0, 0, 0;

		// Kalman Math
        	X = F*X;
        	P = (F*P*F.transpose()) + Q;
		
		Eigen::MatrixXd temp(5,5);
		temp = ( H*P*H.transpose() ) + Ra.topLeftCorner(5,5);
        	K = (P*H.transpose())*temp.inverse();
        	
		// State and Covariance Update
		X = X + K*(z-y);
		X(2) = fmod(X(2) + 2*M_PI, 2*M_PI);
		if ( X(2)>M_PI ) X(2)-=2*M_PI;
        	int Krows = K.rows();
        	P = (Eigen::MatrixXd::Identity(Krows,Krows) - K*H) * P;

		// Print updated state
        	cout << "X = " << endl << X << endl;

		// Declare no new leica data
		newLeicaData = false;
	}
	// Kalman Filter for IMU and encoders
	else if (newIMUData && !newLeicaData){
		Eigen::MatrixXd H(3,6);
		Eigen::Vector3d y;
		Eigen::Vector3d z;
		Eigen::MatrixXd K(6,3);
		cout << "here 1" << endl;
		Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
		cout << "here 2" << endl;
		R.topLeftCorner(2,2) = Ra.topLeftCorner(2,2);
		cout << "here 3" << endl;
		R(2,2) = Ra(5,5);
		cout << "here 4" << endl;

		// The measurement
		z << rightDeltaPhi/diffTime, leftDeltaPhi/diffTime, headingIMU;
		cout << "here 5" << endl;
		cout << "z = " << endl << z << endl;

		// The model "measurement"
		y << sqrt(pow(X(3),2)+pow(X(4),2))/wheelRadius + X(5)*botRadius/wheelRadius,
		     sqrt(pow(X(3),2)+pow(X(4),2))/wheelRadius - X(5)*botRadius/wheelRadius,
		     X(2);

		// The jacobian for encoders only
		H << 0, 0, 0, -sin(X(5)*diffTime)/wheelRadius, cos(X(5)*diffTime)/wheelRadius, botRadius/wheelRadius,
		     0, 0, 0, -sin(X(5)*diffTime)/wheelRadius, cos(X(5)*diffTime)/wheelRadius, -botRadius/wheelRadius,
		     0, 0, 1, 0, 0, 0;
		
		// Kalman Maths
		X = F*X;
		P = (F*P*F.transpose()) + Q;
		
		Eigen::Matrix3d temp;
		temp = (H*P*H.transpose())+R;
		K = (P*H.transpose())*temp.inverse();
		
		// State and Covariance Update
		X = X + K*(z-y);
		X(2) = fmod(X(2) + 2*M_PI, 2*M_PI);
		if ( X(2)>M_PI ) X(2)-=2*M_PI;
		int Krows = K.rows();
		P = (Eigen::MatrixXd::Identity(Krows,Krows) - K*H) * P;

		// Print updated state
		cout << "X = " << endl << X << endl;

		// Declare no new IMU data
		newIMUData = false;
	}
	// Kalman Filter for all the data
	else if(newIMUData && newLeicaData) {
		Eigen::MatrixXd H(6,6);
		Eigen::VectorXd y(6);
		Eigen::VectorXd z(6);
		Eigen::MatrixXd K(6,6);
		Eigen::MatrixXd R(6,6); 
		R = Ra; 
	
		// The measurement (encoders only)
		z << rightDeltaPhi/diffTime, leftDeltaPhi/diffTime, location[0], location[1], location[2], headingIMU;
		cout << "z = " << endl << z << endl;

		// The model "measurement"
	        y << sqrt(pow(X(3),2)+pow(X(4),2))/wheelRadius + X(5)*botRadius/wheelRadius,
		     sqrt(pow(X(3),2)+pow(X(4),2))/wheelRadius - X(5)*botRadius/wheelRadius,
		     X(0),
		     X(1),
		     X(2),
		     X(2);
	
		// The jacobian for encoders only
		H << 0, 0, 0, -sin(X(5)*diffTime)/wheelRadius, cos(X(5)*diffTime)/wheelRadius, botRadius/wheelRadius,
		     0, 0, 0, -sin(X(5)*diffTime)/wheelRadius, cos(X(5)*diffTime)/wheelRadius, -botRadius/wheelRadius,
		     1, 0, 0, 0, 0, 0,
		     0, 1, 0, 0, 0, 0,
		     0, 0, 1, 0, 0, 0,
		     0, 0, 1, 0, 0, 0;

		// Kalman Maths
		X = F*X;
		P = (F*P*F.transpose()) + Q;
		
		Eigen::MatrixXd temp(6,6);
		temp = (H*P*H.transpose())+R;
		K = (P*H.transpose())*temp.inverse();
		
		// State and Covariance Update
		X = X + K*(z-y);
		X(2) = fmod(X(2) + 2*M_PI, 2*M_PI);
		if ( X(2)>M_PI ) X(2)-=2*M_PI;
		int Krows = K.rows();
		P = (Eigen::MatrixXd::Identity(Krows,Krows) - K*H) * P;

		// Print updated state
		cout << "X = " << endl << X << endl;
		
		// Declare no new IMU data
		newIMUData = false;

		// Declare no new leica data
		newLeicaData = false;
	}
	// Kalman Filter for encoders only
	else {
		Eigen::MatrixXd H(2,6);
		Eigen::Vector2d y;
		Eigen::Vector2d z;
		Eigen::MatrixXd K(6,2);
		Eigen::Matrix2d R; 
		R = Ra.topLeftCorner(2,2); 
	
		// The measurement (encoders only)
		z << rightDeltaPhi/diffTime, leftDeltaPhi/diffTime;
		cout << "z = " << endl << z << endl;

		// The model "measurement"
	        y << sqrt(pow(X(3),2)+pow(X(4),2))/wheelRadius + X(5)*botRadius/wheelRadius,
		     sqrt(pow(X(3),2)+pow(X(4),2))/wheelRadius - X(5)*botRadius/wheelRadius;
	
		// The jacobian for encoders only
		H << 0, 0, 0, -sin(X(5)*diffTime)/wheelRadius, cos(X(5)*diffTime)/wheelRadius, botRadius/wheelRadius,
		     0, 0, 0, -sin(X(5)*diffTime)/wheelRadius, cos(X(5)*diffTime)/wheelRadius, -botRadius/wheelRadius;

		// Kalman Maths
		X = F*X;
		P = (F*P*F.transpose()) + Q;
		
		Eigen::Matrix2d temp;
		temp = (H*P*H.transpose())+R;
		K = (P*H.transpose())*temp.inverse();
		
		// State and Covariance Update
		X = X + K*(z-y);
		X(2) = fmod(X(2) + 2*M_PI, 2*M_PI);
		if ( X(2)>M_PI ) X(2)-=2*M_PI;
		int Krows = K.rows();
		P = (Eigen::MatrixXd::Identity(Krows,Krows) - K*H) * P;

		// Print updated state
		cout << "X = " << endl << X << endl;
	}
		
        //cout << "y = " << y << endl;

	absoluteX = X(0);
	absoluteY = X(1);
	absoluteTheta = fmod(X(2) + 2*M_PI, 2*M_PI);
	//cout << "Leica X: " << location[0] << endl;
	//cout << "Leica Y: " << location[1] << endl;
	cout << "Position: " << X(0) << " " << X(1) << " " << X(2) << endl;
	cout << "Velocity: " << X(3) << " " << X(4) << " " << X(5) << endl;
	cout << "Last Leica Pose: " << location[0] << " " << location[1] << " " << location[2] << endl;
}
