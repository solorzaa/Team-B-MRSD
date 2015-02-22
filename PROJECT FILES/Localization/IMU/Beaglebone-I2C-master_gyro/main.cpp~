#include "../include/i2c.h"
#include <iostream>

// Define device address
// It is a better idea to include all the addresses in a header file
/*ITG3200*/
#define DEVICE_ADDR 	0x68
#define I2C_DEVICE "/dev/i2c-1"

#define ITG3200_ADDR 0x68
#define ITG3200_SELF 0x0
#define ITG3200_INT 0x1a
#define ITG3200_XRH 0x1d /*2 byte Hight byte and Low byte*/
#define ITG3200_XRL 0x1e
#define ITG3200_YRH 0x1f /*2 byte Hight byte and Low byte*/
#define ITG3200_YRL 0x20
#define ITG3200_ZRH 0x21 /*2 byte Hight byte and Low byte*/
#define ITG3200_ZRL 0x22 /*2 byte Hight byte and Low byte*/
#define ITG3200_ROT_RAW_SENSITIVITY 14.375
#define MAX_BUFFER_SIZE					64
/* ......................................................................*/

// Define register address
#define CTRL_REG1		0x10
#define STATUS_REG		0x12

using namespace std;

int main(){
	// Instantiate the I2c Object
	myI2C *i2cptr = new myI2C();
			
	int average=1;	
	signed int gz[average];
	signed int gyroz = 0;
	unsigned int i;
	for (i = 0; i<average; i++)
	{
		while (!(i2cptr->Read_I2C_Byte(DEVICE_ADDR, ITG3200_INT ) & 0x01))
		;
		int valueH=i2cptr->Read_I2C_Byte(DEVICE_ADDR, ITG3200_ZRH);	
		int valueL=i2cptr->Read_I2C_Byte(DEVICE_ADDR, ITG3200_ZRL);
		valueH=valueH<<8;
		valueH|=valueL;
		int k;
		k=(int)valueH;
		int gz[i] = i2cptr->ITG3200_rot_conv(k);

		
		gyroz += gz[i];
		cout << gyroz << "in loop" <<endl;
	}
	cout << "averaged value" <<endl;
	cout << gyroz << endl;
	
	return 0;
}
