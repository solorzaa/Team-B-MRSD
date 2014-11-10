#include <stdlib.h>
#include <fcntl.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/time.h>
#include <time.h>

#include "libGeoCOM.h"
#include "libGeoCOM_win.h"


struct PARAMATER{

};
struct GeoCOM_ASCII_request{
	uint RPC_id;
	uint Transaction_id;
	uint Parameter_Count;
	PARAMATER *P;
};



