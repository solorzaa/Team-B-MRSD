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

//  i2cptr->compass_x_offset = 308.512;
// i2cptr->compass_y_offset = 266.689;
//  i2cptr->compass_z_offset = 693.773;

//  i2cptr-> compass_x_gainError = 0.9531;
//  i2cptr->compass_y_gainError = 0.770;
//  i2cptr->compass_z_gainError = 0.677;

  //i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_MODE_REG,0X00);
     i2cptr-> compass_init(2,compass_address);
  // i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,0x01,0X10);

 //2cptr->compass_debug = 1;
 //2cptr->compass_offset_calibration(3,compass_address);

while(1){
 //cout <<"compassx_offset " << i2cptr->compass_x_offset;
  i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_MODE_REG,0X00);

  i2cptr->compass_read_XYZdata(compass_address);

}
return 0;
}