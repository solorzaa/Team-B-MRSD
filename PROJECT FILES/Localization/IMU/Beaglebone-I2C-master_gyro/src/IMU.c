#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
//#include <linux/i2c-dev.h>
#include "i2c-dev.h"
#include <fcntl.h>
#include <errno.h>

#define I2C_DEVICE "/dev/i2c-1"

/*ITG3200*/
#define ITG3200_ADDR 0x69
#define ITG3200_SELF 0x0
#define ITG3200_INT 0x1a
#define ITG3200_TH 0x1b /*2 bytes Hight byte and Low byte*/
#define ITG3200_TL 0x1c
#define ITG3200_XRH 0x1d /*2 byte Hight byte and Low byte*/
#define ITG3200_XRL 0x1e
#define ITG3200_YRH 0x1f /*2 byte Hight byte and Low byte*/
#define ITG3200_YRL 0x20
#define ITG3200_ZRH 0x21 /*2 byte Hight byte and Low byte*/
#define ITG3200_ZRL 0x22 /*2 byte Hight byte and Low byte*/
#define ITG3200_TEMP_RAW_OFFSET 13200
#define ITG3200_TEMP_RAW_SENSITIVITY 280
#define ITG3200_TEMP_OFFSET 35
#define ITG3200_ROT_RAW_SENSITIVITY 14.375

int twosc2int(int twoscomplimentdata)
{ int retval;
  if( twoscomplimentdata > 32768 ) retval = twoscomplimentdata - 65536;
  else retval = twoscomplimentdata;
  return retval;
}

float ITG3200_rot_conv(int rawdata)
{ float retval;
  int raw;

  raw=twosc2int(rawdata);
  retval = (float)raw / (float)ITG3200_ROT_RAW_SENSITIVITY;
  return retval;
}

float ITG3200_temp_conv(int rawdata)
{ float retval;
  int raw;

  raw=twosc2int(rawdata);
  retval = (float)ITG3200_TEMP_OFFSET + (((float)raw + ITG3200_TEMP_RAW_OFFSET) / ITG3200_TEMP_RAW_SENSITIVITY);
  return retval;
}

void ITG3200_read (int file, int *raw, int *reg_array,int size)
{ __s32 res;
  int i,j,k;

  for(i=0;i<size;i++)
  { k=0;
    for (j=0;j<2;j++)
    {
    if ( (res = i2c_smbus_read_byte_data(file,*(reg_array + i + j)) )<0 )
    { printf("Failed to read from the i2c bus.\n");
   exit(1);
    }
      if (j == 0) k=(int)res << 8;
      else
      { k += (int)res;
        *(raw + (i/2))=k;
      }
    }
    i++;
  }
}

main ()
{ int file;
  int i,j,k;
  float data[4]={0};

  int ITG3200_REGS[8]={ITG3200_TH,ITG3200_TL,ITG3200_XRH,ITG3200_XRL,
    ITG3200_YRH, ITG3200_YRL,ITG3200_ZRH,ITG3200_ZRL};
  int ITG3200_RAW_DATA[4];
  float ITG3200_DATA[4];

  if ((file = open(I2C_DEVICE, O_RDWR)) < 0)
  { perror("Failed to open the i2c bus");
    exit(1);
  }

  if (ioctl(file, I2C_SLAVE, ITG3200_ADDR) < 0)
  { printf("Failed to acquire bus access and/or talk to slave.\n");
    exit(1);
  }

/*Take an avarage over 10 consecuitve readings on the ITG3200*/
  for (i=0;i<10;i++)
  { ITG3200_read(file,&ITG3200_RAW_DATA[0],&ITG3200_REGS[0],sizeof(ITG3200_REGS)/sizeof(ITG3200_REGS[0]));

    data[0] += ITG3200_temp_conv(ITG3200_RAW_DATA[0]);
    data[1] += ITG3200_rot_conv(ITG3200_RAW_DATA[1]);
    data[2] += ITG3200_rot_conv(ITG3200_RAW_DATA[2]);
    data[3] += ITG3200_rot_conv(ITG3200_RAW_DATA[3]);
  }
  for(i=0;i<4;i++) data[i] /= 10; 

  printf("Temp. : %2.2f \n",data[0]);
  printf("Rot. X : %2.2f \n",data[1]);
  printf("Rot. Y : %2.2f \n",data[2]);
  printf("Rot. Z : %2.2f \n",data[3]);

  close(file);
}
