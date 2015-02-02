//////////////////////////////////////////////////////////////////////
// Stephen Smith - Fieldroid
///////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////
//Includes
///////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>

#include "Pose.cpp"


using namespace std;


//////////////////////////////////////////
//Global Variables
//////////////////////////////////////////

Pose*** constructLUT(double _vMin, double _vMax, const int _vNumberOfEntries, double _wMin, double _wMax, const int _wNumberOfEntries);
Pose* projectPath(double linearVelocity, double angularVelocity, double t_interval, double t_step);

double pathHorizon = 1;
double pathResolution = .1;

/////////////////////////////////
//Main
/////////////////////////////////
int main(int argc, char *argv[])
{
	Pose*** lut;
	int vLength = 1;
	int wLength = 1;
	lut = constructLUT(.4,.6,vLength,1,2,wLength);
	
	for(int i=0; i<vLength; i++) {
		for(int j=0; j<wLength; j++) {
			for(int k=0; k<pathHorizon/pathResolution; k++) {
				/*cout << "not segfaulted 3" << endl;
				Pose *** a = lut;
				cout << "not sefgaulted 4" << endl;
				Pose ** b = lut[i];
				cout << "not sefgaulted 5" << endl;
				Pose * c = lut[i][j];
				cout << "not sefgaulted 6" << endl;
				Pose d = lut[i][j][k];
				cout << "not sefgaulted 7" << endl;
				double e = lut[i][j][k].X;*/
				//cout << "(" << lut[i][j][k].X << "," << lut[i][j][k].Y << "," << lut[i][j][k].Theta << ") ";
				cout << lut[i][j][k].X << " " << lut[i][j][k].Y << " " << lut[i][j][k].Theta << endl;
			}
			//cout << endl;
		}
		//cout << endl << endl;
	}
	return 0;
}


////////////////////////////////////////////
//Function Definitions
////////////////////////////////////////////

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
	for (int i = 0; i < _vNumberOfEntries; ++i) {
		//cout << "not segfaulted 1:" << i << endl;
		LUT[i] = new Pose*[_wNumberOfEntries];
	}

	//Store projected path for each linear and angular velocity combination
    	for(int i = 0; i < _vNumberOfEntries; i++){
        	for(int j = 0; j < _wNumberOfEntries; j++){
				//cout << "not segfaulted 2:" << j << "," << i << endl;
        		LUT[i][j] = projectPath(_vMin + (_vResolution * i), _wMin + (_wResolution * j), pathHorizon, pathResolution);

        	}
    	}

    	return LUT;
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
	//Pose path[numberOfPoints];
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
			path[i].X = (_linearVelocity/_angularVelocity) * cos(_angularVelocity*t) * t;
			path[i].Y = (_linearVelocity/_angularVelocity) * sin(_angularVelocity*t) * t;
			path[i].Theta = _angularVelocity * t;
		}
	}

	return path;
}
