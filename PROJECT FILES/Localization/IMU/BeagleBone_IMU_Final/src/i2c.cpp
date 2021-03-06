#include"i2c.h"
#include <iostream>
using namespace std;


myI2C::myI2C(){
	this->i2cOpen();
}

myI2C::~myI2C(){
	this->i2cClose();
}

// open the Linux device
void myI2C::i2cOpen()
{
	//cout << "beagle-i2c opening /dev/i2c-3... ";
	g_i2cFile = open("/dev/i2c-1", O_RDWR);
	if (g_i2cFile < 0) {
		perror("i2cOpen in myI2C::i2cOpen");
		exit(1);
	}
//	else cout << "OK"<<endl;
}

// close the Linux device
void myI2C::i2cClose()
{
	close(g_i2cFile);
}

// set the I2C slave address for all subsequent I2C device transfers
void myI2C::i2cSetAddress(unsigned char address)
{
 	//cout << "beagle-i2c setting address 0x"<< hex <<(int)address <<"... ";
	if (ioctl(g_i2cFile, I2C_SLAVE, address) < 0) {
		perror("i2cSetAddress error in myI2C::i2cSetAddress");
		exit(1);
	}
//	else cout << "OK" <<endl;
}


void myI2C::Send_I2C_Byte(unsigned char DEVICE_ADDR, unsigned char Reg_ADDR, unsigned char Data){
	i2cSetAddress(DEVICE_ADDR);
	//cout << "beagle-i2c writing 0x"<< hex << (int)Data <<" to 0x"<<hex <<(int)DEVICE_ADDR << ", reg 0x" <<hex<<(int)Reg_ADDR <<"... ";
	I2C_WR_Buf[0] = Reg_ADDR;
	I2C_WR_Buf[1] = Data;

	if(write(g_i2cFile, I2C_WR_Buf, 2) != 2) {
		perror("Write Error in myI2C::Send_I2C_Byte");
	}
//	else cout << "OK";

}


unsigned char myI2C::Read_I2C_Byte(unsigned char DEVICE_ADDR,unsigned char Reg_ADDR){
	I2C_WR_Buf[0] = Reg_ADDR;
	
	i2cSetAddress(DEVICE_ADDR);
	if(write(g_i2cFile, I2C_WR_Buf, 1) != 1) {
		perror("Write Error in myI2C::Read_I2C_Byte");
	}
	i2cSetAddress(DEVICE_ADDR);	
	if(read(g_i2cFile, I2C_RD_Buf, 1) !=1){
		perror("Read Error myI2C::Read_I2C_Byte");
	}

	return I2C_RD_Buf[0];
}

unsigned char myI2C::Read_Multi_Byte(unsigned char DEVICE_ADDR, unsigned char Reg_ADDR, size_t n){
	I2C_WR_Buf[0] = Reg_ADDR;
	
	i2cSetAddress(DEVICE_ADDR);
	ssize_t s = write(g_i2cFile, I2C_WR_Buf, 1);
	if( s != 1) {
		cout << "Wanted to write " << 1 << " byte, but instead wrote " << s << ". " <<endl;
		perror("Write Error in myI2C::Read_Multi_Byte");
	}
	i2cSetAddress(DEVICE_ADDR);	
	ssize_t t = read(g_i2cFile, I2C_RD_Buf, n);
	if( t != n)
	{
		cout << "Wanted to read " << n << " bytes, but instead got " << t << ". " <<endl;
		perror("Read Error in myI2C::Read_Multi_Byte");
	}
		
	return I2C_RD_Buf[0];
}

unsigned char myI2C::ReadB(unsigned char DEVICE_ADDR,unsigned char Reg_ADDR1,unsigned char Reg_ADDR2){
	I2C_WR_Buf[0] = Reg_ADDR1;
	
	i2cSetAddress(DEVICE_ADDR);
	if(write(g_i2cFile, I2C_WR_Buf, 1) != 1) {
		perror("Write Error in myI2C::Read_I2C_Byte");
	}
	i2cSetAddress(DEVICE_ADDR);	
	if(read(g_i2cFile, I2C_RD_Buf, 1) !=1){
		perror("Read Error myI2C::Read_I2C_Byte");
	}
	
	int temp=0;
	temp=(int)(I2C_RD_Buf[0]);
	temp=temp<<8;
	I2C_WR_Buf[0] = Reg_ADDR2;
	
	i2cSetAddress(DEVICE_ADDR);
	if(write(g_i2cFile, I2C_WR_Buf, 1) != 1) {
		perror("Write Error in myI2C::Read_I2C_Byte");
	}
	i2cSetAddress(DEVICE_ADDR);	
	if(read(g_i2cFile, I2C_RD_Buf, 1) !=1){
		perror("Read Error myI2C::Read_I2C_Byte");
	}
//	cout << "myfunc data" << endl;
//	cout <<(int)( I2C_RD_Buf[0]) << endl;
	temp |=(int)I2C_RD_Buf[0];
//	cout <<"cobineddata";
//	cout << (int) temp <<endl;
	return I2C_RD_Buf[0];
//	return temp;
}

//Changing data from 2's complement to integer value
int myI2C::twosc2int(int twoscomplimentdata)
{ int retval;
  if( twoscomplimentdata >= 32768 ) retval = twoscomplimentdata - 65536;
  else retval = twoscomplimentdata;
  return retval;
}


float myI2C::ITG3200_rot_conv(int rawdata)
{ float retval;
  int raw;

  raw=twosc2int(rawdata);
  retval = (float)raw / (float)ITG3200_ROT_RAW_SENSITIVITY;
  return retval;
}

// callibrate the IMU for zero position
float  myI2C::Itg3200calibrate(void)
{
int reads = 600;
int delay = 4000; // 4 milliseconds
int skip = 5 ; // initial samples to skip
float temp = 0;
float angle=0;
int k;	
		
		for(int i = 0; i < reads; i++){

			if (i >= skip){
			
			int valueH=Read_I2C_Byte(ITG3200_ADDR,ITG3200_ZRH);	
			int valueL=Read_I2C_Byte(ITG3200_ADDR, ITG3200_ZRL);
			valueH=valueH<<8;
			valueH|=valueL;
			k=(int)valueH;
			angle=twosc2int(k);
			temp += angle;
			}
			
			usleep(delay);
			}
			
			temp=(float)temp/(float)(ITG3200_ROT_RAW_SENSITIVITY);
			float z_offset = (-1) * (float)temp / (reads - skip);
			return z_offset;	

}

