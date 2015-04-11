#include"i2c.h"
#include <iostream>
#include<math.h>


using namespace std;
#define Task_t 10          //s3 Task Time in milli seconds
// Main code -----------------------------------------------------------------
#if 0
float compass_x_offset=0, compass_y_offset=0, compass_z_offset=0,compass_gain_fact=1,compass_x_scalled,compass_y_scalled,compass_z_scalled;
float compass_x_gainError=1,compass_y_gainError=1,compass_z_gainError=1,bearing=0;
int compass_x=0,compass_y=0,compass_z=0;
int compass_debug=0;
#endif

int main(){
	myI2C *i2cptr = new myI2C();

	float compass_x_offset=0, compass_y_offset=0, 	   compass_z_offset=0,compass_gain_fact=1,compass_x_scalled,compass_y_scalled,compass_z_scalled;
	float compass_x_gainError=1,compass_y_gainError=1,compass_z_gainError=1,bearing=0;
	int compass_x=0,compass_y=0,compass_z=0;
	int compass_debug=0;

  compass_x_offset = 242.18;
  compass_y_offset = 207.18;
  compass_z_offset = 206.38;
  
  compass_x_gainError = 1.03;
  compass_y_gainError = 1.11;
  compass_z_gainError = 1.03;
   
  
  i2cptr-> compass_init(2,compass_address);
  //compass_debug = 1;
  
  i2cptr->compass_offset_calibration(3,compass_address);

while(1){
  
  
  i2cptr->compass_scalled_reading(compass_address);
  
  //Serial.print("x = ");
  //Serial.println(compass_x_scalled);
 //Serial.print("y = ");
  //Serial.println(compass_y_scalled);
  //Serial.print("z = ");
  //Serial.println(compass_z_scalled);
  

  i2cptr->compass_heading(compass_address);
  cout <<"Heading angle = "; cout << bearing; cout<<" Degree" <<endl;
  usleep(500);
  
  
}

return 0;
}









