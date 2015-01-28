//////////////////////////////////////////////////////////////////////
// Stephen Smith - Fieldroid
///////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////
//Includes
///////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;


//////////////////////////////////////////
//Global Variables
//////////////////////////////////////////

double currentTime = 34;
double timeStep = .01;

/// Need a function that finds the planned path for some horizon (ti - tf)
bool projectGoal(double horizon, vector<double> & xDesired, vector<double> & yDesired, vector<double> & thDesired);

bool desiredPathXY(double t, double & x, double & y, double & th);

bool desiredPathVW(double t, double & v, double & w);

/////////////////////////////////
//Main
/////////////////////////////////
int main(int argc, char *argv[])
{
	// Prints the whole path in .1 intervals
	for(double i = 0; i < 116.6; i+=.1) {
		double x;
		double y;
		double th;
		desiredPathXY(i,x,y,th);
		cout << x << ", " << y << ", " << th << endl;
	}
	
	// gets a subset of the path
	vector<double> xHorizon, yHorizon, thHorizon;
	projectGoal(5, xHorizon, yHorizon, thHorizon);

	// Prints the subset to the screen
	for(int i = 0; i < xHorizon.size(); i++) {
		cout << xHorizon[i] << ", " << yHorizon[i] << ", " << thHorizon[i] << endl;
	}

	return 0;
}


////////////////////////////////////////////
//Function Definitions
////////////////////////////////////////////

/////////////////////////////////////////////////////////////////

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

	for(double t = currentTime; t < currentTime+horizon; t+= timeStep) {
		double x,y,th;
		desiredPathXY(t,x,y,th);
		xHorizon.push_back(x);
		yHorizon.push_back(y);
		thHorizon.push_back(th);
	}
	return true;
}

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
		
