
// ########################################################################
//
//  Carnegie Mellon University
//  TeamB FielDroid

//
// ########################################################################



//#include <stdlib.h>
#include <stdint.h> /* for uint32_t */
#include <stdio.h>
#include <string.h>
#include <unistd.h> /* for write() */
#include <errno.h>
#include <assert.h>

#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <unistd.h>

#include <iostream>

#include <cstdlib>
using namespace std;


#include <sstream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <cstring>
//*fieldroid
#include<termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
int openPort ();
int readPort();
//fieldroid


///* Fieldroid
static int fd;
int ARRAY_SIZE = 200;
char port[20] = "/dev/ttyUSB1"; /* port to connect to */

int openPort(){

	
  fd = open(port, O_RDWR | O_NONBLOCK);
  if(-1 == fd){
    //if(verbosity>=1){
      fprintf(stderr, "Could not open %s: ",port);
      perror("");
    //}
    return -1;
  }

  struct termios term;
  memset(term.c_cc, 0, NCCS);
  term.c_iflag=0x0;
  term.c_oflag=0x0;
  term.c_cflag = (CS8|CREAD)| (B57600);
  term.c_lflag = 0;
  term.c_line=0;
  term.c_cc[VMIN]=1;
  term.c_cc[VTIME]=5;
  if (ioctl(fd, TCSETS, &term) != 0 ||
      ioctl(fd, TCFLSH, 2) != 0){
    perror("serialInit() ioctl failed:");
    close(fd);
    return -1;
  }

}

#if 0
int readPort(char* full_str){
 

#if 0     
    char byte_in[100];
    int  bytes_read;
    string line,entry;
    bytes_read = read(fd, byte_in, ARRAY_SIZE); /* Reads ttyO port, stores data into byte_in. */
	   	
	int rc = 0;
    for (int i = 0; i < bytes_read; i++)
	{
	 //  	printf("\n\n Bytes read are: %d\n",bytes_read);	
    	if (byte_in[i]=='\n'){
			cout << endl;
		}
          
		cout << byte_in[i];
rc++;
	}
	printf("Read characters is: %d\n",rc);
#endif
#if 0	
	printf("\n\n Bytes read are: %d\n",bytes_read);
  byte_in[bytes_read+1]='\0';
  //line=byte_in;
	printf("\n\n String is: %s\n",byte_in);
  std::string str (byte_in);

  char * cstr = new char [str.length()+1];
  std::strcpy (cstr, byte_in);
	printf("\n\nString is: %s\nn",cstr);
  // cstr now contains a c-string copy of str
	
  char * p = std::strtok (cstr," ");
  while (p!=NULL)
  {
    std::cout << *p << '\n';
    p = std::strtok(NULL," ");
  }

  delete[] cstr;
#endif
   
     return rc;
}

#endif

int readPort(char* fs){
	char byte_in[100];
	int bytes_read = 0;
	int n = 100; //this is packet size
	int count = 0;
	char last_char = 1;
	bool end_now = false;
	int i;	
	while(1){
		int l = read(fd, byte_in, ARRAY_SIZE);
		//if(l>0)
			//printf("read are: %d\n",l);
		for(i = 0; i < l; i++){
			if(byte_in[i] == '\n')
				end_now = true;
			else{
//				printf("%d\n",count);
				*(fs+count) = byte_in[i];		
	//			printf("%c\n",byte_in[i]);
				count++;			
			}
		}
		if(end_now)
			break;
	//	if(l>0)
		//	printf("read are: %d\n",l);
		//last_char = byte_in[l];		
	//	byte_in[l+1] = '\0'; 	//check here if +1 required or not
		
		//count += l;
//		strcat(fs,byte_in);
	}
	return count;	
}

//Fieldroid*/
int main(int argc, char **argv) {
       FILE *f = stdout;

	char* full_string;
	//assign space for full string on the stack
	full_string = (char*) malloc(100*sizeof(char));

	openPort();
//	usleep(1500000);
	int l = 0;
	while(1){  
		int rc = readPort(full_string);	
		printf("Returned here- string is: %s\n",full_string);
		usleep(1000000);
		l++;
	}

	return 0;
}
