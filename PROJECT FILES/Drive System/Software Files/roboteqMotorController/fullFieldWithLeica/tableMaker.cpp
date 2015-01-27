//////////////////////////////////////////////////////////////////////
// Stephen Smith - Fieldroid
///////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////
//Includes
///////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <iostream>
#include <vector>

using namespace std;


//////////////////////////////////////////
//Global Variables
//////////////////////////////////////////

bool makeLUT(vector<double> & vLUT, vector<double> & wLUT);

/////////////////////////////////
//Main
/////////////////////////////////
int main(int argc, char *argv[])
{
	vector<double> vLUT;
	vector<double> wLUT;
	
	makeLUT(vLUT,wLUT);

	for(vector<double>::iterator i = vLUT.begin(); i!= vLUT.end(); ++i) {
		cout << *i << endl;
	}
	for(vector<double>::iterator i = wLUT.begin(); i!= wLUT.end(); ++i) {
		cout << *i << endl;
	}
	return 0;
}


////////////////////////////////////////////
//Function Definitions
////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
bool makeLUT(vector<double> & vLUT, vector<double> & wLUT)
{
	// Settings
	double vAverage = 0.5*12; // (0.5 ft/s)
	double vChange = 0.1*12; // (Don't differ by more than 0.1 ft/s)
	double radiusMin = 6*12; // 6ft minimum turning radius
	double vInterval = 0.001;
	double wInterval = 0.0001;
	
	// Calculated end points
	double vMin = vAverage-vChange;
	double vMax = vAverage+vChange; 
	double wMin = -vAverage/(radiusMin/2);
	double wMax = vAverage/(radiusMin/2);

	// Calculated array lengths
	int vLength = (vMax-vMin)/vInterval;
	int wLength = (wMax-wMin)/wInterval;

	//cout << vLength << " " << wLength << endl;

	// Returns
	vLUT.resize (vLength);
	wLUT.resize (wLength);

	for(vector<double>::size_type i = 0; i < vLength; i++) {
		vLUT[i] = vMin+(i*vInterval);
	}
	for(vector<double>::size_type i = 0; i < wLength; i++) {
		wLUT[i] = wMin+(wInterval*i);
	}
	
	return true;
}
