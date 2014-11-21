
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


#include <iostream>

#include <cstdlib>
using namespace std;




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
char port[20] = "/dev/ttyO2"; /* port to connect to */

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

int readPort(void){
      
    char byte_in[100];
    int  bytes_read;
  
    bytes_read = read(fd, byte_in, ARRAY_SIZE); /* Reads ttyO port, stores data into byte_in. */

    for (int i = 0; i < bytes_read; i++){
	    if (byte_in[i]=='\n'){
	    cout << endl;
	    }
            cout << byte_in[i] ;
   }

    return 1;
}


//Fieldroid*/
int main(int argc, char **argv) {
       FILE *f = stdout;


openPort();
	while(1){
	  
  readPort();	

	}


	return 0;
}
