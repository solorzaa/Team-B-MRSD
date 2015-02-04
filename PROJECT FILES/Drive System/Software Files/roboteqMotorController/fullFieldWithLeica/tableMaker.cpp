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

Pose* projectPath(double linearVelocity, double angularVelocity, double t_interval, double t_step);

Pose*** constructLUT(double _vMin, double _vMax, const int _vNumberOfEntries, double _wMin, double _wMax, const int __wNumberOfEntries);

bool projectGoal(double horizon, vector<double> & xDesired, vector<double> & yDesired, vector<double> & thDesired);

bool desiredPathXY(double t, double & x, double & y, double & th);

bool desiredPathVW(double t, double & v, double & w);

double getPathError(Pose* projectedPath,  vector<Pose> goalPoints, int _numPathPoints);

double*  getOptimalVelocities(Pose*** projectedPaths, int _vNumberOfEntries, int _wNumberOfEntries, int _numPathPoints ,vector<double> xDesiredPath, vector<double> yDesiredPath, vector<double> thetaDesiredPath);

bool pathToRobotFrame(vector<double> projectedPathX, vector<double> projectedPathY, vector<double> projectedPathTheta, vector<Pose> & newProjectedPath);

double pathHorizon = 3;
double pathResolution = .1;
double vMin = 5;
double wMin = -1;
double vMax = 7;
double wMax = 1;
double vResolution = .2;
double wResolution = .01;
double currentTime = 2;
bool printStuff = false;
double absoluteTheta = -.3;
double absoluteX = 0;
double absoluteY = 12;
bool printThings = false;


/////////////////////////////////
//Main
/////////////////////////////////
int main(int argc, char *argv[])
{
	Pose*** lut;
	int vLength = (vMax-vMin)/vResolution;
	int wLength = (wMax-wMin)/wResolution;
	lut = constructLUT(vMin,vMax,vLength,wMin,wMax,wLength);

	vector<double> xGoal;
	vector<double> yGoal;
	vector<double> tGoal;
	projectGoal(pathHorizon, xGoal, yGoal, tGoal);

	double * optimalV;
	optimalV = getOptimalVelocities(lut, vLength, wLength, pathHorizon/pathResolution, xGoal, yGoal, tGoal);
	cout << optimalV[0] << ", " << optimalV[1] << endl;
	
/*	for(int i=0; i<vLength; i++) {
		for(int j=0; j<wLength; j++) {
			for(int k=0; k<pathHorizon/pathResolution; k++) {
				cout << lut[i][j][k].X << "," << lut[i][j][k].Y << "," << lut[i][j][k].Theta<< " ";// << endl;
			}
			cout << endl;
		}
		cout << endl << endl;
	}*/
	return 0;
}


////////////////////////////////////////////
//Function Definitions
////////////////////////////////////////////
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
	double _vResolution = (double)abs(_vMax - _vMin) / _vNumberOfEntries;
	double _wResolution = (double)abs(_wMax - _wMin) / _wNumberOfEntries;

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

        for(double t = currentTime+pathResolution; t < currentTime+horizon+pathResolution; t+= pathResolution) {
                double x,y,th;
                desiredPathXY(t,x,y,th);
                xHorizon.push_back(x);
                yHorizon.push_back(y);
                thHorizon.push_back(th);
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

		if(printThings) {
			cout << projectedPath[i].X << " vs " << goalPoints[i].X << endl;
			cout << projectedPath[i].Y << " VS " << goalPoints[i].Y << endl;
		}

		//No need to take the square root of the sum of squares because it will still be the minimum
		error += pow(xError, 2) +  pow(yError, 2);

	}

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
	
	vector<Pose> goalPath (xDesiredPath.size());
	pathToRobotFrame(xDesiredPath, yDesiredPath, thetaDesiredPath, goalPath);

	for(int i = 0; i < _vNumberOfEntries; i ++){
		for(int j = 0; j < _wNumberOfEntries; j++){
		
			//retried error for current path
			errorCurrent = getPathError(projectedPaths[i][j], goalPath, _numPathPoints);
			
			if(i==5) {
				printThings = true;
				cout << endl;
			}
			else printThings = false;
			//set this path error as the minimum if its less than the current minimum
			if(errorCurrent < errorMin || (i==0 && j==0))
			{
				errorMin = errorCurrent;
				_vMinIndex = i;
				_wMinIndex = j;
			}

		}
	}
	
	//convert the index values to the corresponding velocity commands and return them
	velocities[0] =  vMin + (_vMinIndex * vResolution);
	velocities[1] =  wMin + (_wMinIndex * wResolution);		

	return velocities;

}

// Rotates the projected goal path into the frame of the robot
bool pathToRobotFrame(vector<double> projectedPathX, vector<double> projectedPathY, vector<double> projectedPathTheta, vector<Pose> & newProjectedPath) {
	for (int i=0; i<projectedPathX.size(); i++) {
		newProjectedPath[i].X = cos(absoluteTheta)*projectedPathX[i] + sin(absoluteTheta)*projectedPathY[i];
		newProjectedPath[i].Y = -sin(absoluteTheta)*projectedPathX[i] + cos(absoluteTheta)*projectedPathY[i];
		newProjectedPath[i].X -= absoluteX;
		newProjectedPath[i].Y -= absoluteY;
		newProjectedPath[i].Theta += absoluteTheta;
	}
}
