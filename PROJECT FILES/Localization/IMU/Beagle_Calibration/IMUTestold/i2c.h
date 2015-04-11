#ifndef _BEAGLEI2C_H_
#define _BEAGLEI2C_H_

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
#include <cstdio>


/*Magnetometer*/
 #define HMC5883L_I2C_ADDRESS                    0x1E
 #define HMC5883L_CONFIG_REG_A                   0x00
 #define HMC5883L_CONFIG_REG_B                   0x01
 #define HMC5883L_MODE_REG                       0x02
 #define HMC5883L_X_MSB			         0x03
 #define HMC5883L_X_LSB                          0x04
 #define HMC5883L_Z_MSB			         0x05
 #define HMC5883L_Z_LSB            		 0x06
 #define HMC5883L_Y_MSB               	  	 0x07
 #define HMC5883L_Y_LSB             	         0x08
 #define HMC5883L_STATUS_REG                     0x09
 #define HMC5883L_ID_REG_A     			 0x0A

/* ......................................................................*/


//added defines defination for the new code
#define compass_address 0x1E       // The I2C address of the Magnetometer
#define compass_XY_excitation 1160 // The magnetic field excitation in X and Y direction during Self Test (Calibration)
#define compass_Z_excitation 1080  // The magnetic field excitation in Z direction during Self Test (Calibration)
#define compass_rad2degree 57.3
#define compass_cal_x_offset 116   // Manually calculated offset in X direction
#define compass_cal_y_offset 225   // Manually calculated offset in Y direction
#define compass_cal_x_gain 1.1     // Stored Gain offset at room temperature
#define compass_cal_y_gain 1.12    // Stored Gain offset at room temperature

///


#define MAX_BUFFER_SIZE					64
#define ITG3200_ROT_RAW_SENSITIVITY 14.375
class myI2C {
	int g_i2cFile;
public:
	myI2C();
	~myI2C();
	
	// Public Variables
	unsigned char I2C_WR_Buf[MAX_BUFFER_SIZE];			// Contains data you want to send
	unsigned char I2C_RD_Buf[MAX_BUFFER_SIZE];			// Contains data which was read
	
	// Initialize Functions
	void i2cOpen();										// Opens i2cbus 3, done at the beginning
	void i2cClose();									// Closes i2cbus 3, done at the ending
	void i2cSetAddress(unsigned char address);					// Changes device address
	
	// Sends a single byte <Data> to <DEVICE_ADDR> on the register <Reg_ADDR>
	void Send_I2C_Byte(unsigned char DEVICE_ADDR, unsigned char Reg_ADDR, unsigned char Data);	

	// Reads and returns a single byte from <DEVICE_ADDR> on the register <Reg_ADDR>
	unsigned char Read_I2C_Byte(unsigned char DEVICE_ADDR,unsigned char Reg_ADDR);
	
	int twosc2int(int twoscomplimentdata);
 
  //added for new code
  float bearing;
  float compass_x_scalled;
  float compass_y_scalled;
  float compass_z_scalled;
  
  float compass_x_offset, compass_y_offset, compass_z_offset;
  float compass_x_gainError,compass_y_gainError,compass_z_gainError;
  
  int compass_debug;
  
  void compass_read_XYZdata(unsigned char DEVICE_ADDR);
  void compass_offset_calibration(int select,unsigned char DEVICE_ADDR);
  void compass_init(int gain, unsigned char DEVICE_ADDR);
  void compass_scalled_reading(unsigned char DEVICE_ADDR);
  void compass_heading(unsigned char DEVICE_ADDR);
 // // // // //added for new code above, remvoed the extern keyword to make work	
};
#endif /* BEAGLEI2C.H */


