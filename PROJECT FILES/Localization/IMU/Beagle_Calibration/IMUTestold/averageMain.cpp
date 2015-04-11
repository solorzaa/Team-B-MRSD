
#include"i2c.h"
#include <iostream>
#include<math.h>
// Define device address
// It is a better idea to include all the addresses in a header file
/*ITG320/
#define DEVICE_ADDR 	0x68

#define I2C_DEVICE "/dev/i2c-1"
/.....................................................................*/

// Define register address
#define CTRL_REG1		0x10
#define STATUS_REG		0x12

using namespace std;

int main(){
	// Instantiate the I2c Object
	myI2C *i2cptr = new myI2C();

	float angle;
	float headingoffset;
	/*mag*/
	i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_CONFIG_REG_A,0x70);//Average data ,11
	i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_CONFIG_REG_B,0X10);
	i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_MODE_REG,0X00);
	float scaleX=0.92;
	float scaleY=0.92; //setting scale for magnetometer
	int flag=1;
	while (1){
	i2cptr->Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_MODE_REG,0X00); 	
	usleep(100000);
	//read data from magnetometer	

	char valuemxH=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_X_MSB);
	char valuemxL=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_X_LSB);
	char valuemzH=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_Z_MSB);// read data from the registers
	char valuemzL=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_Z_LSB);
	char valuemyH=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_Y_MSB);
	char valuemyL=i2cptr->Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_Y_LSB);	
	 signed	int valuemx=valuemxH<<8 | valuemxL;
	 signed	int valuemy=valuemyH<<8|valuemyL;
		
		int valuemax=i2cptr->twosc2int(valuemx);
		int valuemay=i2cptr->twosc2int(valuemy);
		//cout << "  actualX " << valuemax;
		//cout << "  actualY " << valuemay;	
		float valuemsx=valuemax*scaleX;
		float  valuemsy=valuemay*scaleY;
		//cout << "  scaledX " << valuemsx;
		//cout << "  scaledY " <<valuemsy;
 				
		//calculate the heading angle
				
		float heading=atan2(valuemsy, valuemsx);
		float declinationAngle = 0.16;
 		heading += declinationAngle;
			 if(heading < 0)
      			 heading += 2*M_PI;
			 if(heading > 2*M_PI)
			 heading -= 2*M_PI;
			 
			 cout << " HeadingRad " << heading;
			 float headingDegrees = heading * 180/M_PI;
                         cout <<"  HeadingDeg without offset " << headingDegrees <<endl;
			// usleep(10000); 


			  if (flag==1)
  		          {
  				headingoffset=headingDegrees;
				flag=0;
				cout << "Offset for inital setting" << headingoffset;
  			  }
  			 if (headingDegrees<headingoffset)
  			 headingDegrees=headingDegrees-headingoffset+360; 
  			 else
  			 headingDegrees=headingDegrees-headingoffset; 
  			 cout << "  HeadingDeg with offset" << headingDegrees;	
				
	}
	return 0;
}
