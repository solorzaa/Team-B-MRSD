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

	int average=30;	
	float gz[average];
	float gyroz=0;
	unsigned int i;
	//int k;
	float angle;
	float offset=i2cptr->Itg3200calibrate();
	cout <<"offest is:" << offset <<endl;
	/*mag*/
	i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_CONFIG_REG_B,0X20);
//	i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_MODE_REG,0X00);
	
	while (1){
	i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_MODE_REG,0X00); 		// Example send data
	float scaleX=0.92;
	float scaleY=0.92;
	int valuemxH=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_X_MSB);
	int valuemxL=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_X_LSB);
	int valuemyH=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_Y_MSB);
	int valuemyL=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_Y_LSB);	
		valuemxH=valuemxH<<8 | valuemxL;
	//	valuemxH=valuemxH*scaleX;
	        //int km= (int)valuemxH;
		cout << "hex:" << endl; cout << valuemxH;
		//cout << "magx" << km <<endl; 
		valuemyH=valuemyH<<8|valuemyL;
	//	valuemyH=valuemyH*scaleY;
		cout << "magy"<<valuemyH <<endl;
		valuemxH=i2cptr->twosc2int(valuemxH);
		valuemyH=i2cptr->twosc2int(valuemyH);
		valuemxH=valuemxH*scaleX;
		valuemyH=valuemyH*scaleY;	
		float heading=atan2(valuemyH,valuemxH);
		float declinationAngle = -0.1614;
 		heading += declinationAngle;
			 if(heading < 0)
      			 heading += 2*M_PI;
			 if(heading > 2*M_PI)
			 heading -= 2*M_PI;
			 float headingDegrees = heading * 180/M_PI;
                         cout <<"Heading:" << headingDegrees <<endl;
			 usleep(10000);	
		
	}

	/*mag*/
	while(1){
	for (i = 0; i<average; i++)
	{
		while (!(i2cptr->Read_I2C_Byte(DEVICE_ADDR, ITG3200_INT ) & 0x01))
		;
		int valueH=i2cptr->Read_I2C_Byte(DEVICE_ADDR, ITG3200_ZRH);	
		int valueL=i2cptr->Read_I2C_Byte(DEVICE_ADDR, ITG3200_ZRL);
		valueH=valueH<<8;
		valueH|=valueL;
	//	cout << "hex equv.:";
	//	cout << hex << valueH << endl <<endl;
		
	//	k=(int)valueH;
		angle = i2cptr->ITG3200_rot_conv(valueH);
		
		gz[i]=angle;
		gyroz += gz[i];
		}
	usleep(100000);
	//gyroz=(float)gyroz/(float)ITG3200_ROT_RAW_SENSITIVITY;
	gyroz=gyroz/(float)average;
	
	gyroz=gyroz+offset;
	//cout << "averaged value" <<endl;
	cout << gyroz << endl;
	
	}
	return 0;
}
