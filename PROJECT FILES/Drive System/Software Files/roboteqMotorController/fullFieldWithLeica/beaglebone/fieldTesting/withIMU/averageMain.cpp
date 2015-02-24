//#include "../include/i2c.h"
#include"i2c.h"
#include <iostream>
#include<math.h>

// Define device address
// It is a better idea to include all the addresses in a header file
/*ITG3200*/
#define DEVICE_ADDR 	0x68

#define I2C_DEVICE "/dev/i2c-1"
/*......................................................................*/

// Define register address
#define CTRL_REG1		0x10
#define STATUS_REG		0x12

using namespace std;

int main(){
	// Instantiate the I2c Object
	myI2C *i2cptr = new myI2C();

	int average=10;	
	float gz[average];
	float gyroz=0;
	unsigned int i;
	float angle;
	//callibrating gyroscope and magnetometer
	float offset=i2cptr->Itg3200calibrate();
	float heading_offset=i2cptr->HMC5883Lcalibrate();
	cout <<"heading offset is:" << heading_offset <<endl;
	cout <<"offset is:" << offset <<endl;
	
	/*mag*/
	i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_CONFIG_REG_B,0X20);
	i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_MODE_REG,0X00);
	float scaleX=0.92;
	float scaleY=0.92; //setting scale for magnetometer

	while (1){
	i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_MODE_REG,0X00); 	
	usleep(67000);

	//read data from magnetometer	
	int valuemxH=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_X_MSB);
	int valuemxL=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_X_LSB);
	int valuemyH=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_Y_MSB);
	int valuemyL=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_Y_LSB);	
		valuemxH=valuemxH<<8 | valuemxL;
		valuemyH=valuemyH<<8|valuemyL;

		valuemxH=i2cptr->twosc2int(valuemxH);
		valuemyH=i2cptr->twosc2int(valuemyH);
		valuemxH=valuemxH*scaleX;
		valuemyH=valuemyH*scaleY;	
		//caluclate the heading angle
				
		float heading=atan2(valuemyH,valuemxH);
		float declinationAngle = -0.1614;
 		heading += declinationAngle;
	//	heading_offset=heading_offset*M_PI/180;
	//	heading=heading+heading_offset;
			 if(heading < 0)
      			 heading += 2*M_PI;
			 if(heading > 2*M_PI)
			 heading -= 2*M_PI;
			 //float headingDegrees = heading * 180/M_PI;
			 float headingDegrees = heading * 180/M_PI;	
                         headingDegrees=headingDegrees+heading_offset;
			 if (headingDegrees<0)
				headingDegrees+=360;
			 cout <<"Heading:" << headingDegrees <<endl;
			 usleep(100000);	
		
//	}

	/*mag*/
	//while(1)
//	{
	//read gyro data
	for (i = 0; i<average; i++)
	{
		while (!(i2cptr->Read_I2C_Byte(DEVICE_ADDR, ITG3200_INT ) & 0x01))
		;
		int valueH=i2cptr->Read_I2C_Byte(DEVICE_ADDR, ITG3200_ZRH);	
		int valueL=i2cptr->Read_I2C_Byte(DEVICE_ADDR, ITG3200_ZRL);
		valueH=valueH<<8;
		valueH|=valueL;
		angle = i2cptr->ITG3200_rot_conv(valueH);
		gz[i]=angle;
		gyroz += gz[i];
		}
	usleep(100000);
	gyroz=gyroz/(float)average;
	//offset obtained from callibration step
	gyroz=gyroz+offset;
	cout << "averaged value:" << gyroz << endl;
	
	}
	return 0;
}
