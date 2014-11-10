// ########################################################################
//
// This is an implemntaion of Leica GeoSystem's GeoCOM interface
//  - used to control a Leica Survey device over a serial port
//  - built for use of Leica GeoSystem's TCA1105 Survey Device
//
//
// ########################################################################
//
//  Carnegie Mellon University, copyright 2006
//  Joshua Anhalt, anhalt@cmu.edu
//
//  Updated 2009 by Dominic Jonak (dom@cmu.edu)
//  Updated 2011 by David Kohanbash (dkohanba@cmu.edu)
//
// ########################################################################

/*
 * TODO:
 *  replace write() with fwrite()
 *   - remove unistd.h and global int serial_port
 *  beep on error
 */

#include <stdlib.h>
#include <stdint.h> /* for uint32_t */
#include <stdio.h>
#include <string.h>
#include <unistd.h> /* for write() */
#include <errno.h>
#include <assert.h>

#include <sys/time.h>
#include <time.h>
#include <math.h>

#include "../../include/ipcMsgs.h"

#include "GeoCOM_commands.h"
#include "GeoCOM_displayInfo.h"
#include "CommUtil.h"

#include "libGeoCOM.h"
#include "libGeoCOM_win.h"


/* this will fill in the UTM position
 * unfortunately it seems to degrade the data rate
 */
#define USE_GetCoordinate 0

struct NETWORK_RESPONSE{
	uint32_t status; // 0=success; >0=error code
	uint32_t count; // the record number from Survey Instrument

  //angles
	double dHz;	// Horizontal Angle
	double dV;	// Vertical Angle (from Plumbline)
	double dDist;	// Slope Distance

  //coordinates
	double East;	// Easting
	double North;	// Northing
	double Height;	// Elevation

  //???
	double dE_Cont;	// Easting
	double dN_Cont;	// Northing
	double dH_Cont;	// Elevation

  //time of angles (dHz, dV, dDist)
  //long int anglesTime;//time in ms since the unit was powered on (not available)
  struct timeval anglesTV;//time when the data was received

  //time of the coordinate (East, North, Height)
  long int coordinateTime;//time in ms since the unit was powered on
  struct timeval coordinateTV;//time when the data was received

  //time of the ??? (dE_Cont, dN_Cont, dH_Cont)
  long int contTime;//time in ms since the unit was powered on
  //struct timeval contTV;//time when the data was received (same as coordinateTV)
};

// ########################################################################
// High-Level Commands
// ########################################################################

static void PublishMeasurement(const NETWORK_RESPONSE &nr){
  TOTAL_STATION_DATA_MSG_TYPE msg;
  msg.position.easting = nr.East;
  msg.position.northing = nr.North;
  msg.position.altitude = nr.Height;
  msg.distance = nr.dDist;
  msg.horizontal = nr.dHz;
  msg.vertical = nr.dV;
  msg.timestamp = nr.coordinateTV;
  PUBLISH_MACRO(TOTAL_STATION_DATA, msg);
}

static void PrintMeasurement(FILE *f, const NETWORK_RESPONSE &nr){
  if(0 == nr.count)
    fprintf(f, "%%timestamp count   east north altitude(m)    hor(deg) ver(deg) distance(m)\n");
  fprintf(f, "%10ld.%06ld %d   %0.3f %0.3f %0.3f      %0.2f %0.2f %0.3f\n", 
	  nr.coordinateTV.tv_sec, nr.coordinateTV.tv_usec, nr.count,
	  nr.East, nr.North, nr.Height,
	  nr.dHz * 180.0/M_PI, nr.dV * 180.0/M_PI, nr.dDist);
  fflush(f);
}

static int TakeMeasurement(NETWORK_RESPONSE &nr){
  static uint32_t count = 0;
  TMC_COORDINATE Coordinate;
  memset(&Coordinate, 0, sizeof(Coordinate));
  nr.status = 0;

  //printf("Generating Cartesian Coordinate %d\n", count);

  //	retval_Distance = TMC_DoMeasure(TMC_DEF_DIST,TMC_AUTO_INC);
  if(-1 == TMC_DoMeasure(TMC_TRK_DIST,TMC_AUTO_INC)){
    //printf("TakeMeasurement: DoMeasure failed\n");
    nr.status = 1;
    return -1;
  }

  gettimeofday(&nr.coordinateTV, NULL);
#if USE_GetCoordinate
  //printf("Valid Measurement Made\n");
  if(RC_OK != TMC_GetCoordinate(1000,Coordinate,TMC_AUTO_INC)){
    //printf("TakeMeasurement: GetCoordinate failed\n");
    nr.status += 2;
  }

  //record the timestamp of the coordinate
  nr.coordinateTime = Coordinate.CoordTime;

  //record the coordinate
  nr.East = Coordinate.dE;
  nr.North = Coordinate.dN;
  nr.Height = Coordinate.dH;

  //record the ???
  nr.contTime = Coordinate.CoordContTime;
  nr.dE_Cont = Coordinate.dE_Cont;
  nr.dN_Cont = Coordinate.dN_Cont;
  nr.dH_Cont = Coordinate.dH_Cont;
#endif

  // Get Angles
  TMC_HZ_V_ANG OnlyAngle;
  double SlopeDistance;
  if(RC_OK != TMC_GetSimpleMea(1000, OnlyAngle, SlopeDistance, TMC_AUTO_INC)){
    //printf("TakeMeasurement: GetSimpleMea failed\n");
    nr.status += 4;
  }

  //record the timestamp of the angles
  gettimeofday(&nr.anglesTV, NULL);

  //record the angles
  nr.dHz = OnlyAngle.dHz;
  nr.dV = OnlyAngle.dV;
  nr.dDist = SlopeDistance;

  nr.count = count++;

  return (nr.status?-1:0);
}


// ########################################################################
// My support functions to implment GeoCOM RPC Interface
// ########################################################################
// Evil global stuff
int serial_port=-1;
FILE* serial_port_file=NULL;

void GeoCOM_init(const char *device){
  assert(device);
  serial_port = open_serial(device, 19200, COMMUTIL_SERIAL_PARITY_NONE);
  if (serial_port == -1) {
    perror("open_serial");
    exit(2);
  }
  tcflush(serial_port, TCIOFLUSH);
  serial_port_file = fdopen(serial_port, "rw");
  if(!serial_port_file){
    perror("fd_open");
    exit(2);
  }
}

static int verbose=0;

// send an ASCII request to the device
void GeoCOM_SendRequest_ASCII(uint32_t RPC_id, 
			      const char *Parameters,
			      char *Response){
	char Request[REQUEST_LENGTH];

	assert(strlen(Parameters) + 20 < REQUEST_LENGTH);

	sprintf(Request, "\n%%R1Q,%d:%s\r\n",RPC_id,Parameters);
	if(verbose)
	  printf("Sending request %s", Request);

	if (write(serial_port, Request, strlen(Request)) != (int)strlen(Request)){
		fprintf(stderr, "ERROR: could not transmit entire message!");
		exit(-1);
	}

	/* this buffer keeps growing as necessary
	 * Though it should never grow past RESPONSE_LENGTH
	 * TODO: replace with static buffer and fgets()
	 */
	static char* retdata = NULL;
	static size_t retlen = 0;

	while(1){
	  printf("about to cal getline\n"); fflush(stdout);
	  size_t retread = getline(&retdata, &retlen, serial_port_file);

	  if(-1 == (int)retread){
	    printf("Error reading response to %s\n", Request);
	    exit(-1);
	  }

	  if(retread+1 >= RESPONSE_LENGTH){
	    printf("Error, Request %s\n", Request);
	    printf("Long response %s\n", Response);
	    exit(-1);
	  }

	  strncpy(Response, retdata, retread);
	  Response[retread] = '\0';
	  if(verbose)
	    printf("Received Response: %s", Response);

	  /* does this ever get triggered? */
	  if(Response[0] == '\r' || Response[0] == '\n'){
	    printf("Skipping empty line\n");
	    //skip empty lines
	    continue;
	  }

	  /* does this ever get triggered? */
	  //echoed commands look like "%R1Q,[RPC_id]:\n"
	  const char *Q = "%R1Q,";
	  if(!strncmp(Q, Response, strlen(Q))){
	    printf("Skipping echo\n");
	    //skip echos
	    continue;
	  }

	  break;
	}

}


int main(int argc, char **argv) {
  FILE *f = stdout;

	// Command Line Args
	if (argc != 2){
		printf("usage: %s <serial_device>\n", argv[0]);
		exit(1);
	}

	/* IPC initialization */
	if(!getenv("CENTRALHOST")){
	  printf("Warning, CENTRALHOST not set\n");
	  printf("Expected to connect to remote central\n");
	  if(-1 == setenv("CENTRALHOST", "10.0.0.2", 0)){
	    perror("Could not set CENTRALHOST");
	    return -1;
	  }
	}
	printf("Connecting to central on %s\n", getenv("CENTRALHOST"));
	IPC_connect(argv[0]);
	DEFINE_FORMAT_MACRO(TOTAL_STATION_DATA);
	CHECK_FORMAT_MACRO(TOTAL_STATION_DATA);


	// initialize the serial port
	GeoCOM_init(argv[1]);

	// print out some information
	printDeviceInfo(f);

	printf("%%Entering main work loop...\n");
	verbose=0;
	while(1){
	  //printf("\nCalling TakeMeasurement()\n");
	  NETWORK_RESPONSE nr;
	  memset(&nr, 0, sizeof(nr));
	  if(-1 == TakeMeasurement(nr))
	    fprintf(f, "%%TakeMeasurement() failed, code %d\n", nr.status);
	  PublishMeasurement(nr);
	  PrintMeasurement(f, nr);
	}

//	retval = IOS_BeepOff();
//	retval = BMM_BeepNormal();
	
	return 0;
}
