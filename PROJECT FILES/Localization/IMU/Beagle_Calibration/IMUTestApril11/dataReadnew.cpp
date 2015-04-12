#include"i2c.h"
#include <iostream>
#include<math.h>


using namespace std;
#define Task_t 10          //s3 Task Time in milli seconds
// Main code -----------------------------------------------------------------
#if 0
float compass_x_offset=0,compass_y_offset=0, compass_z_offset=0,compass_gain_fact=1,compass_x_scalled=0,compass_y_scalled=0,compass_z_scalled=0;
float compass_x_gainError=1,compass_y_gainError=1,compass_z_gainError=1,bearing=0;
int compass_x=0,compass_y=0,compass_z=0;
int compass_debug=1;
#endif

int main(){
	myI2C *i2cptr = new myI2C();
	
 i2cptr->compass_x_offset = 158.118;
 i2cptr->compass_y_offset = 221.43;
 i2cptr->compass_z_offset = 205.136;
  
  i2cptr-> compass_x_gainError = 1.03;
  i2cptr->compass_y_gainError = 1.13;
  i2cptr->compass_z_gainError = 1.01;
   
  //i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_MODE_REG,0X00);
 
  i2cptr-> compass_init(2,compass_address);
  i2cptr->compass_debug = 1;
 // i2cptr->compass_offset_calibration(3,compass_address);

while(1){
 //cout <<"compassx_offset " << i2cptr->compass_x_offset; 
  i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_MODE_REG,0X00);
  
  i2cptr->compass_scalled_reading(compass_address);
  
  cout <<"xs= "<< i2cptr-> compass_x_scalled <<endl;
  cout <<"ys= "<< i2cptr->compass_y_scalled <<endl;
  cout <<"zs= "<< i2cptr->compass_z_scalled <<endl;  

  i2cptr->compass_heading(compass_address);
  cout <<"Heading angle = "; cout <<  i2cptr->bearing; cout<<" Degree" <<endl;
  usleep(500000);
  
  
}

return 0;
}









