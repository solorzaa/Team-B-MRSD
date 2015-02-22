#ifndef _BEAGLEI2C_H_
#define _BEAGLEI2C_H_

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
#include <cstdio>

/*ITG3200*/
#define ITG3200_ADDR 0x68
#define ITG3200_SELF 0x0
#define ITG3200_INT 0x1a
#define ITG3200_XRH 0x1d /*2 byte Hight byte and Low byte*/
#define ITG3200_XRL 0x1e
#define ITG3200_YRH 0x1f /*2 byte Hight byte and Low byte*/
#define ITG3200_YRL 0x20
#define ITG3200_ZRH 0x21 /*2 byte Hight byte and Low byte*/
#define ITG3200_ZRL 0x22 /*2 byte Hight byte and Low byte*/


/* ......................................................................*/

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
	
	// Reads multipes byte from <DEVICE_ADDR> starting from the register address <Reg_ADDR>.
	// Read the output from i2cptr->I2C_RD_Buf
	unsigned char Read_Multi_Byte(unsigned char DEVICE_ADDR, unsigned char Reg_ADDR, size_t n);
	unsigned char ReadB(unsigned char, unsigned char, unsigned char);
	int twosc2int(int twoscomplimentdata);
	float ITG3200_rot_conv(int rawdata);
	float Itg3200calibrate(void);
};
#endif /* BEAGLEI2C.H */


