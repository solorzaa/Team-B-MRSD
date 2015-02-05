//////////////////////////////////////////////////////////////////////
// Stephen Smith - Fieldroid
///////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////
//Includes
///////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <iostream>
#include <vector>
#include <time.h>

using namespace std;


//////////////////////////////////////////
//Global Variables
//////////////////////////////////////////

/////////////////////////////////
//Main
/////////////////////////////////
int main(int argc, char *argv[])
{
	clock_t startTime = clock();
	double interval = 100;
	
	while((double) (clock()-startTime)/CLOCKS_PER_SEC < interval) {
		cout << (double) (clock()-startTime)/CLOCKS_PER_SEC << endl;
	}
	return 0;
}


////////////////////////////////////////////
//Function Definitions
////////////////////////////////////////////

/////////////////////////////////////////////////////////////////