#include"i2c.h"
#include <iostream>
#include<math.h>
using namespace std;
#include<cstdlib>
#include<time.h>
//adding for the new code
#if 0
float compass_x_offset, compass_y_offset, compass_z_offset,compass_gain_fact=1,compass_x_scalled=200,compass_y_scalled,compass_z_scalled;
float compass_x_gainError=1,compass_y_gainError=1,compass_z_gainError=1,bearing;
int compass_x=0,compass_y=0,compass_z=0;
int compass_debug=0;
#endif 
///

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
	i2copen=1; // some file is already open, its a flag
//	else cout << "OK"<<endl;
	i2cclose=0;
}

// close the Linux device
void myI2C::i2cClose()
{
	close(g_i2cFile);
	i2cclose=1;
	i2copen=0;
}

// set the I2C slave address for all subsequent I2C device transfers
void myI2C::i2cSetAddress(unsigned char address)
{

	if (i2cclose==1)
	{
	i2cOpen();
	}
	
	
 	//cout << "beagle-i2c setting address 0x"<< hex <<(int)address <<"... ";
	if (ioctl(g_i2cFile, I2C_SLAVE, address) < 0) {
		perror("i2cSetAddress error in myI2C::i2cSetAddress");
		exit(1);
	}
	
//	else cout << "OK set address" <<endl;
}






void myI2C::Send_I2C_Byte(unsigned char DEVICE_ADDR, unsigned char Reg_ADDR, unsigned char Data){
//	cout <<"Sending data";
//	i2cOpen();
	i2cSetAddress(DEVICE_ADDR);
	
	//cout << "beagle-i2c writing 0x"<< hex << (int)Data <<" to 0x"<<hex <<(int)DEVICE_ADDR << ", reg 0x" <<hex<<(int)Reg_ADDR <<"... ";
	I2C_WR_Buf[0] = Reg_ADDR;
	I2C_WR_Buf[1] = Data;

	if(write(g_i2cFile, I2C_WR_Buf, 2) != 2) {
		perror("Write Error in myI2C::Send_I2C_Byte");
	}
//	else cout << "OK";
	i2cClose();
}


unsigned char myI2C::Read_I2C_Byte(unsigned char DEVICE_ADDR,unsigned char Reg_ADDR){
//	i2cOpen();
	I2C_WR_Buf[0] = Reg_ADDR;
	
	i2cSetAddress(DEVICE_ADDR);
	if(write(g_i2cFile, I2C_WR_Buf, 1) != 1) {
		perror("Write Error in myI2C::Read_I2C_Byte");
	}
	i2cSetAddress(DEVICE_ADDR);	
	if(read(g_i2cFile, I2C_RD_Buf, 1) !=1){
		perror("Read Error myI2C::Read_I2C_Byte");
	}
	i2cClose();
	return I2C_RD_Buf[0];
}



//Changing data from 2's complement to integer value
int myI2C::twosc2int(int twoscomplimentdata)
{ int retval;
  if( twoscomplimentdata >= 32768 ) retval = twoscomplimentdata - 65536;
  else retval = twoscomplimentdata;
  return retval;
}

//adding new functions for the new code
//function one

void myI2C::compass_read_XYZdata(unsigned char DEVICE_ADDR){
//Wire.beginTransmission(compass_address);
  i2cSetAddress(DEVICE_ADDR);	
  //Wire.write(0x02);
  //Wire.write(0b10000001); 
  Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_MODE_REG,0x00); 	
	usleep(67000);  

        int valuemxH=Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_X_MSB);
	int  valuemxL=Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_X_LSB);

	int  valuemzH=Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_Z_MSB);// read data from the registers
	int  valuemzL=Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_Z_LSB);
	
	char valuemyH=Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_Y_MSB);
	char valuemyL=Read_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_Y_LSB);	
	
		int valuemx=valuemxH<<8|valuemxL;
	 	int valuemy=valuemyH<<8|valuemyL;
	        int valuemz=valuemzH<<8|valuemzL;
	compass_x=twosc2int(valuemx);
	compass_y=twosc2int(valuemy);
	compass_z=twosc2int(valuemz);
				
		
		//cout <<"not here";
		//cout << endl;
  //	cout<< "compass_x" <<compass_x<<endl;
  //  	cout<< "compass_y" <<compass_y<<endl;
 //	cout<< "compass_z" <<compass_z<<endl;
		
}



// --------------------------------------------------------------------------
// Setting the gain
// This Function updates the gain_fact variable

void myI2C::compass_init(int gain, unsigned char DEVICE_ADDR){
  //cout <<"In compass initialization works"<<endl;
  
  char gain_reg;
  //Wire.beginTransmission(compass_address);
  i2cSetAddress(DEVICE_ADDR);
  //Wire.write(0x01);

  if (gain == 0){
    gain_reg = 0b00000000;
    compass_gain_fact = 0.73;
  }
  else if (gain == 1){
    gain_reg = 0b00100000;
    compass_gain_fact= 0.92;
  }
  else if (gain == 2){
    //cout << "gain ==2";
    gain_reg = 0x40;
    compass_gain_fact= 1.22;
  }
  else if (gain == 3){
    gain_reg = 0b01100000;
    compass_gain_fact= 1.52;
  }
  else if (gain == 4){
    gain_reg = 0b10000000;
    compass_gain_fact= 2.27;
  }
  else if (gain == 5){
    gain_reg = 0b10100000;
    compass_gain_fact= 2.56;
  }
  else if (gain == 6){
    gain_reg = 0b11000000;
    compass_gain_fact= 3.03;
  }
  else if (gain == 7){
    gain_reg = 0b11100000;
    compass_gain_fact= 4.35;
  }
  Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_CONFIG_REG_B,gain_reg);  
  //Wire.write(gain_reg); // bit configuration = g2 g1 g0 0 0 0 0 0, g2 g1 g0 = 0 0 1 for 1.3 guass and 0 1 0 for 1.9 Guass
  //Wire.write(0b00000011);  // Putting the Magnetometer in idle//didnt 
  // Writing the register value 0000 0000 for continous mode
  // Writing the register value 0000 0001 for single
  // Writing the register value 0000 0011 for Idel
  //Wire.endTransmission(); didnt change
  
 //Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_MODE_REG,0x03);
 
  cout <<"Gain updated to  = ";
  cout <<compass_gain_fact;
  cout << " mG/bit" <<endl;
    
}

//added new function for new code
void myI2C::compass_scalled_reading (unsigned char DEVICE_ADDR){
  //cout<<"In scalled reading" <<endl;
  compass_read_XYZdata(DEVICE_ADDR);
 
  compass_x_scalled=compass_x*compass_gain_fact*compass_x_gainError+compass_x_offset;
  compass_y_scalled=compass_y*compass_gain_fact*compass_y_gainError+compass_y_offset;
  compass_z_scalled=compass_z*compass_gain_fact*compass_z_gainError+compass_z_offset;
   		

}



void myI2C::compass_heading(unsigned char DEVICE_ADDR){
  compass_scalled_reading(DEVICE_ADDR);
  
  if (compass_y_scalled>0){
    bearing = 90-atan(compass_x_scalled/compass_y_scalled)*compass_rad2degree;
    cout <<bearing <<"bearing"<<endl;
  }else if (compass_y_scalled<0){
    bearing = 270-atan(compass_x_scalled/compass_y_scalled)*compass_rad2degree;
  }else if (compass_y_scalled==0 & compass_x_scalled<0){
    bearing = 180;
  }else{
    bearing = 0;
  }
  
  

}



// Call Initialize before 
void myI2C::compass_offset_calibration(int select, unsigned char DEVICE_ADDR){
  // ***********************************************************
  // offset_calibration() function performs two taskes
  // 1. It calculates the diffrence in the gain of the each axis magnetometer axis, using 
  //    inbuilt self excitation function of HMC5883L (Which is useless if it is used as a compass
  //    unless you are very unlucy and got a crapy sensor or live at very High or low temperature)
  // 2. It calculates the mean of each axes magnetic field, when the Magnetometer is rotated 360 degree
  // 3. Do Both
  // ***********************************************************    
  
  
  // *****************************************************************************************
  // Gain offset estimation
  // ***************************************************************************************** 
  if (select == 1 | select == 3){ // User input in the function
 // Configuring the Control register for Positive Bais mode
  cout <<"Calibrating the Magnetometer ....... Gain" <<endl;
  //Wire.beginTransmission(compass_address);
  i2cSetAddress(DEVICE_ADDR);
  Send_I2C_Byte(compass_address,0x00,0b01110001);	
  //Wire.write(0x00);
  //Wire.write(0b01110001); // bit configuration = 0 A A DO2 DO1 DO0 MS1 MS2
  

  /*
  A A                        DO2 DO1 DO0      Sample Rate [Hz]      MS1 MS0    Measurment Mode
   0 0 = No Average            0   0   0   =   0.75                   0   0   = Normal  
   0 1 = 2 Sample average      0   0   1   =   1.5                    0   1   = Positive Bias
   1 0 = 4 Sample Average      0   1   0   =   3                      1   0   = Negative Bais
   1 1 = 8 Sample Average      0   1   1   =   7.5                    1   1   = -
   1   0   0   =   15 (Default)
   1   0   1   =   30
   1   1   0   =   75
   1   1   1   =   -
   */
  //Wire.endTransmission();//didnt change
  
  compass_read_XYZdata(DEVICE_ADDR); // Disregarding the first data

  // Reading the Positive baised Data
  while(compass_x<200 | compass_y<200 | compass_z<200){   // Making sure the data is with Positive baised
     compass_read_XYZdata(DEVICE_ADDR);
//	cout <<"***********************************************"<<endl;
  }
	 
  compass_x_scalled=compass_x*compass_gain_fact;
  compass_y_scalled=compass_y*compass_gain_fact;
  compass_z_scalled=compass_z*compass_gain_fact;
  
  
  // Offset = 1160 - Data_positive
  compass_x_gainError = (float)compass_XY_excitation/compass_x_scalled;
  compass_y_gainError = (float)compass_XY_excitation/compass_y_scalled;
  compass_z_gainError = (float)compass_Z_excitation/compass_z_scalled;

 // Positive error scaling is :
  cout <<"Positive Error : " <<endl;
  cout<<"x_gain_offset = ";
  cout<< compass_x_gainError <<endl;
  cout<<"y_gain_offset = ";
  cout<<compass_y_gainError <<endl;
  cout<<"z_gain_offset = ";
  cout<<compass_z_gainError <<endl;

  // Configuring the Control register for Negative Bais mode
  //Wire.beginTransmission(compass_address);
 // i2cSetAddress(DEVICE_ADDR);
  // Wire.write(0x00);
  //Wire.write(0b01110010); // bit configuration = 0 A A DO2 DO1 DO0 MS1 MS2
  Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_CONFIG_REG_A,0b01110010);

  /*
  A A                        DO2 DO1 DO0      Sample Rate [Hz]      MS1 MS0    Measurment Mode
   0 0 = No Average            0   0   0   =   0.75                   0   0   = Normal  
   0 1 = 2 Sample average      0   0   1   =   1.5                    0   1   = Positive Bias
   1 0 = 4 Sample Average      0   1   0   =   3                      1   0   = Negative Bais
   1 1 = 8 Sample Average      0   1   1   =   7.5                    1   1   = -
                               1   0   0   =   15 (Default)
                               1   0   1   =   30
                               1   1   0   =   75
                               1   1   1   =   -
   */
  //Wire.endTransmission(); didnt change
  
  
  compass_read_XYZdata(DEVICE_ADDR); // Disregarding the first data
  // Reading the Negative baised Data
  while(compass_x>-200 | compass_y>-200 | compass_z>-200){   // Making sure the data is with negative baised
     compass_read_XYZdata(DEVICE_ADDR);
   //   cout <<"***********************************************"<<endl;

  }
  
  compass_x_scalled=compass_x*compass_gain_fact;
  compass_y_scalled=compass_y*compass_gain_fact;
  compass_z_scalled=compass_z*compass_gain_fact;
 
  compass_x_gainError =(float) (compass_XY_excitation/fabs(compass_x_scalled));
  compass_y_gainError =(float)(compass_XY_excitation/fabs(compass_y_scalled));
  compass_z_gainError =(float)(compass_Z_excitation/fabs(compass_z_scalled));

  cout <<"Negative Error : " <<endl;
  cout<<"x_gain_offset = ";
  cout<< compass_x_gainError <<endl;
  cout<<"y_gain_offset = ";
  cout<<compass_y_gainError <<endl;
  cout<<"z_gain_offset = ";
  cout<<compass_z_gainError <<endl;


  // Taking the average of the offsets
  compass_x_gainError =(float) ((compass_XY_excitation/fabs(compass_x_scalled))+compass_x_gainError)/2;
  compass_y_gainError =(float)((compass_XY_excitation/fabs(compass_y_scalled))+compass_y_gainError)/2;
  compass_z_gainError =(float)((compass_Z_excitation/fabs(compass_z_scalled))+compass_z_gainError)/2;

  //compass_x_gainError =1.04;
  //compass_y_gainError =1.12;
  //compass_z_gainError =1.02;


  cout <<"Average error :"<<endl; 	  
  cout<<"x_gain_offset = ";
  cout<< compass_x_gainError <<endl;
  cout<<"y_gain_offset = ";
  cout<<compass_y_gainError <<endl;
  cout<<"z_gain_offset = ";
  cout<<compass_z_gainError <<endl;
  
  
  }
  
   // Configuring the Control register for normal mode
  //Wire.beginTransmission(compass_address); 
  i2cSetAddress(DEVICE_ADDR);
  //Wire.write(0x00);
  //Wire.write(0b01111000); // bit configuration = 0 A A DO2 DO1 DO0 MS1 MS2
  Send_I2C_Byte(HMC5883L_I2C_ADDRESS,HMC5883L_CONFIG_REG_A,0x70);
  /*
  A A                        DO2 DO1 DO0      Sample Rate [Hz]      MS1 MS0    Measurment Mode
   0 0 = No Average            0   0   0   =   0.75                   0   0   = Normal  
   0 1 = 2 Sample average      0   0   1   =   1.5                    0   1   = Positive Bias
   1 0 = 4 Sample Average      0   1   0   =   3                      1   0   = Negative Bais
   1 1 = 8 Sample Average      0   1   1   =   7.5                    1   1   = -
                               1   0   0   =   15 (Default)
                               1   0   1   =   30
                              1   1   0   =   75
                               1   1   1   =   -
   */
  //Wire.endTransmission();//didnt change
  
  // *****************************************************************************************
  // Offset estimation
  // *****************************************************************************************
   if (select == 2 | select == 3){// User input in the function
    cout<<"Calibrating the Magnetometer ....... Offset" <<endl;
    cout<<"Please rotate the magnetometer 2 or 3 times in complete circules with in one minute ............. "<<endl;
    
    for(int i=0;i<10;i++){   // Disregarding first few data
         compass_read_XYZdata(DEVICE_ADDR);
     }
    
    float x_max=-4000,y_max=-4000,z_max=-4000; 
    float x_min=4000,y_min=4000,z_min=4000;
    
    /*
    Debug code ------------------------------
    */
    if (compass_debug == 1){
      cout<<"Starting Debug data in "<<endl;
      usleep(1000000);
      cout<<"3" <<endl;
      usleep(1000000);
      cout<<"2" <<endl;
      usleep(1000000);
      cout<<"1" <<endl;
      usleep(1000000);
      cout<<"0"<<endl;
      cout<<  endl;
      for(int i=0;i<10;i++){   
         cout<<"*";
      }
      cout<<"*" <<endl;
      cout<<"Debug -- (Offset Calibration)"<<endl;
      for(int i=0;i<10;i++){  
         cout<<"*";
      }
      cout<<"*"<<endl;
    }
    // End Debug code
  
       time_t begin,end;
       time(&begin);
       time(&end); 
	

      //unsigned long t = millis();
      // while(millis()-t <= 30000){
      //int t; //add t such that time is around 30 seconds
       int disregardValue=0;
       while (difftime(end,begin) < 30){ 
       //usleep(67000);
       
      compass_read_XYZdata(DEVICE_ADDR);
      //cout << "t" << t <<endl;
      compass_x_scalled=(float)compass_x*compass_gain_fact*compass_x_gainError;
      compass_y_scalled=(float)compass_y*compass_gain_fact*compass_y_gainError;
      compass_z_scalled=(float)compass_z*compass_gain_fact*compass_z_gainError;
      
      if (compass_debug == 1){  //------------------ Debug Data
        cout<< "  x_scalled" << compass_x_scalled ;
       
        cout<< "  y_scalled" << compass_y_scalled ;

        cout<< "  z_scalled" << compass_z_scalled <<endl;
        }//--------------------------------- End Debug Data
      
      if (compass_x_scalled < -1000|| compass_x_scalled > 1000|| compass_y_scalled < -1000 ||compass_y_scalled >1000||compass_z_scalled <-1000 ||compass_z_scalled> 1000)
	{
	    disregardValue=1;
	}
	if (disregardValue==0)
	{
      x_max = max(x_max,compass_x_scalled);
      y_max = max(y_max,compass_y_scalled);
      z_max = max(z_max,compass_z_scalled);
  	
      
      x_min = min(x_min,compass_x_scalled);
      y_min = min(y_min,compass_y_scalled);
      z_min = min(z_min,compass_z_scalled);
   
	}
     //t=t+1;
     time(&end);	
    }
   
    cout <<	"x_max"<< x_max <<endl;
    cout <<	"y_max"<<y_max <<endl;
    cout <<	"x_min"<<x_min <<endl;
    cout <<	"y_min"<<y_min <<endl;  			 
        /*
    Debug code ------------------------------
    */
    if (compass_debug == 1){
      cout<< endl;
      for(int i=0;i<10;i++){   
         cout<<"*";
      }
      cout<<"*" <<endl;
      cout<<"End Debug -- (Offset Calibration)" <<endl;
      for(int i=0;i<10;i++){   
         cout<<"*";
      }
      cout<<"*"<<endl;
    }
    // End Debug code
  
    compass_x_offset = ((x_max-x_min)/2)-x_max;
    compass_y_offset = ((y_max-y_min)/2)-y_max;
    compass_z_offset = ((z_max-z_min)/2)-z_max;
    
    
    cout<<"Offset x  = ";
    cout<<compass_x_offset;
    cout<<"mG"<<endl;
    cout<<"Offset y  = ";
    cout<<compass_y_offset;
    cout<<"mG"<<endl;
    cout<<"Offset z  = ";
    cout<<compass_z_offset;
    cout<<"mG"<<endl;
    
  } 
 
}



