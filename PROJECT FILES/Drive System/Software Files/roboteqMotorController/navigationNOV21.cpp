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
#include <stdint.h>
#include <unistd.h> /* for write() */
#include <errno.h>
#include <assert.h>
#include <sys/time.h>
#include <cstdlib>
#include <sstream>

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
string motorControllerPort = "/dev/ttyACM0";
int counts_per_revolution = 5500;
float wheelDiameter = 13.2; //wheel diameter in inches
float wheelRadius = wheelDiameter/2; //wheel radius in inches
float botRadius = 11.25; //radius from each wheel to the midpoint between wheels in inches
int stallPower = 40;

//Leica communication variables
static int fd;
int ARRAY_SIZE = 200;
char port[20] = "/dev/ttyUSB0"; /* port to connect to */

//Point File
float * xPoints = new float[100]{0,0,0,0,0,0,0,0,0,0,0,-0.48,-4.2,-11.28,-21.36,-33.48,-47.16,-61.2,-74.88,-87,-96.96,-103.92,-107.52,-108,-108,-108,-108,-108,-108,-108,-108,-108,-108,-108,-104.52,-96.96,-82.2,-64.44,-43.44,-21.24,-4.2,12.36,26.4,36.48,44.4,50.4,54.6,54.36,48,36.48,19.92,0,-13.5,-27,-40.5,-54,-67.5,-81,-94.5,-108,-119.4,-129.84,-138.6,-144.84,-148.08,-148.08,-144.84,-138.6,-129.84,-119.4,-108,-94.5,-81,-67.5,-54,-40.5,-27,-13.5,0,11.4,21.84,30.6,36.84,40.08,40.08,36.84,30.6,21.84,11.4,0,-13.5,-27,-40.5,-54,-67.5,-81,-94.5,-108,-121.5,-135};
float * yPoints = new float[100]{0,16.2,32.4,48.6,64.8,81,97.2,113.4,129.6,145.8,162,169.2,182.76,195,204.96,211.92,215.52,215.52,211.8,204.72,194.76,182.52,168.84,162,145.8,129.6,113.4,97.2,81,64.8,48.6,32.4,16.2,0,-12,-23.76,-35.4,-42.36,-45.48,-43.92,-39.12,-30.24,-16.68,0,19.56,40.56,66.36,93.12,121.56,142.92,156.84,162,162,162,162,162,162,162,162,162,160.32,155.52,148.08,138.36,127.32,115.68,104.64,94.92,87.48,82.68,81,81,81,81,81,81,81,81,81,79.32,74.52,67.08,57.36,46.32,34.68,23.64,13.92,6.48,1.68,0,0,0,0,0,0,0,0,0,0,0};
float * thetaPoints = new float[100]{0,0,0,0,0,0,0,0,0,0,0,0.134390352,0.39618974,0.657989128,0.919788516,1.181587904,1.443387291,1.705186679,1.966986067,2.228785455,2.490584843,2.75238423,3.014183618,3.141592654,3.141592654,3.141592654,3.141592654,3.141592654,3.141592654,3.141592654,3.141592654,3.141592654,3.141592654,3.141592654,3.558726345,3.865904293,4.204498168,4.454080251,4.672246408,4.890412564,5.08763477,5.335471523,5.626941508,5.831145031,5.955063408,6.051056517,6.201154832,0.105766953,0.347320521,0.663225116,1.0978121,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.857030324,2.141518992,2.42775299,2.713986987,2.998475655,3.284709652,3.56919832,3.855432318,4.141666315,4.426154983,4.71238898,4.71238898,4.71238898,4.71238898,4.71238898,4.71238898,4.71238898,4.71238898,4.71238898,4.426154983,4.141666315,3.855432318,3.56919832,3.284709652,2.998475655,2.713986987,2.42775299,2.141518992,1.857030324,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327,1.570796327};
int * paintPoints = new int[100]{1,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,-1,0,0,};

float location[2];
float prevLocation[2];
float xOrigin = 0;
float yOrigin = 0;
float thetaOrigin = 0;
double absoluteX = 0;
double absoluteY = 0;
double absoluteTheta = 0;
int prevRightEncoder;
int prevLeftEncoder;
vector<float> testData;

clock_t prevTime;
clock_t currTime;
double sumErrorX = 0;
double sumErrorY = 0;
double sumErrorTheta = 0;

bool leicaConnected = false;
bool dataOnly = false;

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

int openPort();

int readPort(char* fs);

vector<float> leicaStoF(char* recievedData);

void sphericalToPlanar(float radius, float horAngle, float verAngle);

/////////////////////////////////
//Main
/////////////////////////////////
int main(int argc, char *argv[])
{
	// If you don't want motion
	if(!dataOnly) {
		// Set up the motor controller
		device.Disconnect();
		if(initialize() == false){
		return 1;
		}

		// Turn off the motors (just in case)
		device.SetCommand(_GO, 1, 0);
		device.SetCommand(_GO, 2, 0);	
	}

	// Create variable to handle serial input
	char* full_string;
	//assign space for full string on the stack
	full_string = (char*) malloc(100*sizeof(char));

	// Leica Mode Only
	if(leicaConnected) {
		// Open leica comm port for radios
	        openPort();
		cout << "comm port opened successfully" << endl;
	
		// Check for first data
		readPort(full_string);
	
		// Don't start until we have recieved data from the Leica Tracking Station
	        while(sizeof(full_string) == 0){
			printf("no data");
			readPort(full_string);
                	//printf("Returned here- string is: %s\n",full_string);
                	usleep(1000000);
        	}

		// Display the initial data
		//cout << "received data" << endl;
	
		// Get initial robot data
		testData = leicaStoF(full_string);

		//cout << "The Data R: " << testData[2] << endl;
		//cout << "The Data Theta: " << testData[3] << endl;
		//cout << "The Data Phi: " << testData[3] << endl;

		// Convert intial robot data to planar coordinate system
		sphericalToPlanar(testData[2], testData[3], testData[4]);

		//cout << "Spherical not origin x: " << location[0] << endl;
		//cout << "Spherical not origin y: " << location[1] << endl;

		// Set robot origin to initial planar data
		xOrigin = location[0];
		yOrigin = location[1];

		// Initialize the previous Locations
		prevLocation[0] = location[0];
		prevLocation[1] = location[1];

		//cout << "xOrigin: " << xOrigin << endl;
		//cout << "yOrigin: " << yOrigin << endl;

		//readPort(full_string);
		//Get initial robot data
		//testData = leicaStoF(full_string);

		//Convert intial robot data to planar coordinate system
		//sphericalToPlanar(testData[2], testData[3], testData[4]);

		//cout << "The Data R: " << testData[2] << endl;
		//cout << "The Data Theta: " << testData[3] << endl;
		//cout << "The Data Phi: " << testData[3] << endl;

		//for(vector<float>::iterator it = testData.begin() ; it != testData.end() ; it++){
	      	//cout << *it << endl;
		//}

		//cout << "absolute X after offset: " << location[0] << endl;
		//cout << "absolute Y after offset: " << location[1] << endl;

		//sleep(1);
	}

	if(!dataOnly) {	
		// Initialize the encoders
		readAbsoluteEncoderCount(prevLeftEncoder, 2); 
		readAbsoluteEncoderCount(prevRightEncoder, 1);		
		prevTime = clock();
	}
	
	// Step through the goal points	
	for(int i=0;i<100;i++) {
		bool done = false;
		while(!done) {
			cout << "            point #: " << i << endl << endl;
			if(!dataOnly) {
				done = poseControl(getDeltaPose(),xPoints[i],yPoints[i],thetaPoints[i]);
			}
			if(leicaConnected) {
				// Get new tracking station data
				readPort(full_string);	
				testData = leicaStoF(full_string);
				sphericalToPlanar(testData[2], testData[3], testData[4]);
			}
		}
	}

	if(!dataOnly) {
		// Disconnect roboteq
		device.Disconnect();
	}
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
	
	if(leicaConnected) {
		//If tracking data is new, update the absolute position
		if(location[0]!=prevLocation[0]||location[1]!=prevLocation[1]) {
			absoluteX = location[0];
			absoluteY = location[1];
			prevLocation[0] = location[0];
			prevLocation[1] = location[1];
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
        bool end_now = false;
        int i;
        while(1){
                int l = read(fd, byte_in, ARRAY_SIZE);

                for(i = 0; i < l; i++){
                        if(byte_in[i] == '\n')
                                end_now = true;
                        else{
                                *(fs+count) = byte_in[i];
                                count++;
                        }
                }
                if(end_now)
                        break;
        }
        return count;
}



////////////////////////////////////////////////////////////////
string readPort2(void){
     
    char byte_in[100];
    int  bytes_read;
    string newDataLine;
  
    bytes_read = read(fd, byte_in, ARRAY_SIZE); /* Reads ttyO port, stores data into byte_in. */

    for (int i = 0; i < bytes_read; i++){
	    if (byte_in[i]=='\n'){
		break;
	    }
	    else if(byte_in[i] != '$' || byte_in[i] != '!'){
		newDataLine[i] = byte_in[i];
		cout << "byte_in: " << byte_in[i] << endl;
	    }
    }
    newDataLine	= newDataLine + '\0';

	cout << "received data string: " << newDataLine << endl;

    return newDataLine;
}

////////////////////////////////////////////////////////////////////
vector<float> leicaStoF(char* recievedData){

string leicaString(recievedData);
string entry;
string delimeter = " "; //CHANGE THIS TO COMMAS WHEN TESTING WITH LEICA!!!
size_t pos;
float number;
vector<float> leicaData;

	while((pos = leicaString.find(delimeter)) != string::npos){
	
          entry = leicaString.substr(0, pos).c_str();
	  istringstream(entry) >> number;
	  leicaData.push_back(number);
          leicaString.erase(0, pos + delimeter.length());
		
	}

	
	istringstream(leicaString.c_str()) >> number;
        leicaData.push_back(number);

	return leicaData;
}

///////////////////////////////////////////////////////////////////////
void sphericalToPlanar(float radius, float horAngle, float verAngle){
	//Convert to radians
	horAngle = horAngle * (M_PI/180);
	verAngle = verAngle * (M_PI/180);

	//cout << xOrigin << endl;
	//cout << yOrigin << endl;
	location[0] = (-radius*cos(horAngle)*cos(verAngle)*METERS_TO_INCHES)-xOrigin;
	location[1] = (-radius*sin(horAngle)*cos(verAngle)*METERS_TO_INCHES)-yOrigin;
	return;
}
