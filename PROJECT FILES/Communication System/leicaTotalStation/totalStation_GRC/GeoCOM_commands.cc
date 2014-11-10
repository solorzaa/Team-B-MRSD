#include "GeoCOM_commands.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdint.h>

// ########################################################################
// Begin re-creation of Leica's GeoCOM interface, POSIX/Linux compatible
// ########################################################################

extern void GeoCOM_SendRequest_ASCII(uint32_t RPC_id, 
				     const char *Parameters,
				     char *Response);


static char Response[RESPONSE_LENGTH];
static char Parameters[REQUEST_LENGTH];


// GeoCOM Reference Manual TPS1100 - Version 1.07
RC_TYPE BMM_BeepNormal(void){
  const uint32_t RPC_id = 11003;
  *Parameters = '\0';
  GeoCOM_SendRequest_ASCII(RPC_id, Parameters, Response);

	RC_TYPE GRC, RC;
	uint32_t Transaction_id;
	
	// Format "%R1P,0,0:RC"
	int N = sscanf(Response, "%%R1P,%hd,%u:%hd",&GRC,&Transaction_id,&RC);

	const int expect = 3;
	if(expect != N)printf("%s:%d Warning %s, only %d of %d\n", __FILE__, __LINE__, __FUNCTION__, N, expect);

	return RC;
}


// GeoCOM Reference Manual TPS1100 - Version 1.07
RC_TYPE IOS_BeepOff(void){
  const uint32_t RPC_id = 20000;
  *Parameters = '\0';
  GeoCOM_SendRequest_ASCII(RPC_id, Parameters, Response);

	RC_TYPE GRC, RC;
	uint32_t Transaction_id;
	
	// Format "%R1P,0,0:RC"
	int N = sscanf(Response, "%%R1P,%hd,%u:%hd",&GRC,&Transaction_id,&RC);

	const int expect = 3;
	if(expect != N)printf("%s:%d Warning %s, only %d of %d\n", __FILE__, __LINE__, __FUNCTION__, N, expect);

	return RC;
}


// GeoCOM Reference Manual TPS1100 - Version 1.07
RC_TYPE CSV_GetInstrumentNo(long &SerialNo){
  const uint32_t RPC_id = 5003;
  *Parameters = '\0';

  GeoCOM_SendRequest_ASCII(RPC_id, Parameters, Response);

	RC_TYPE GRC, RC;
	uint32_t Transaction_id;

	// Format "%R1P,0,0:RC, SerialNo[long]"
	int N = sscanf(Response, "%%R1P,%hd,%u:%hd,%ld",&GRC,&Transaction_id,&RC,
		       &SerialNo);

	const int expect = 4;
	if(expect != N)printf("%s:%d Warning %s, only %d of %d\n", __FILE__, __LINE__, __FUNCTION__, N, expect);

	return RC;
}


// GeoCOM Reference Manual TPS1100 - Version 1.07
RC_TYPE CSV_GetInstrumentName(char *Name){
  const uint32_t RPC_id = 5004;
  *Parameters = '\0';
  GeoCOM_SendRequest_ASCII(RPC_id, Parameters, Response);

	RC_TYPE GRC, RC;
	uint32_t Transaction_id;
	
	// Format "%R1P,0,0:RC, Name[string]"
	int N = sscanf(Response, "%%R1P,%hd,%u:%hd,%s",&GRC,&Transaction_id,&RC, Name);
	const int expect = 4;
	if(expect != N)printf("%s:%d Warning %s, only %d of %d\n", __FILE__, __LINE__, __FUNCTION__, N, expect);

	return RC;
}


// GeoCOM Reference Manual TPS1100 - Version 1.07
RC_TYPE CSV_GetDateTime (DATIME &Datime){
  const uint32_t RPC_id = 5008;
  *Parameters = '\0';
  GeoCOM_SendRequest_ASCII(RPC_id, Parameters, Response);

	RC_TYPE GRC, RC;
	uint32_t Transaction_id;
	
	// Format "%R1P,0,0:RC,Year[short],Month,Day,Hour,Minute,Second[byte]"
	int N = sscanf(Response, "%%R1P,%hd,%u:%hd,%hd,'%x','%x','%x','%x','%x'",&GRC,&Transaction_id,&RC,
		       &Datime.Date.Year,
		       (unsigned int *)&Datime.Date.Month,
		       (unsigned int *)&Datime.Date.Day,
		       (unsigned int *)&Datime.Time.Hour,
		       (unsigned int *)&Datime.Time.Minute,
		       (unsigned int *)&Datime.Time.Second);

	const int expect = 9;
	if(expect != N)printf("%s:%d Warning %s, only %d of %d\n", __FILE__, __LINE__, __FUNCTION__, N, expect);

	return RC;
}
// GeoCOM Reference Manual TPS1100 - Version 1.07
RC_TYPE CSV_GetSWVersion(short &nRelease, short &nVersion, short &nSubVersion){
  const int RPC_id = 5034;
  *Parameters = '\0';
  GeoCOM_SendRequest_ASCII(RPC_id, Parameters, Response);
	
	RC_TYPE GRC, RC;
	uint32_t Transaction_id;
	
	// Format "%R1P,0,0:RC nRelease,nVersion,nSubVersion[short]"
	int N = sscanf(Response, "%%R1P,%hd,%u:%hd,%hd,%hd,%hd",&GRC,&Transaction_id,&RC,
		       &nRelease,
		       &nVersion,
		       &nSubVersion);

	const int expect = 6;
	if(expect != N)printf("%s:%d Warning %s, only %d of %d\n", __FILE__, __LINE__, __FUNCTION__, N, expect);

	return RC;
}


// GeoCOM Reference Manual TPS1100 - Version 1.07
RC_TYPE CSV_GetDeviceConfig(TPS_DEVICE &Device){
  const uint32_t RPC_id = 5035;
  *Parameters = '\0';
  GeoCOM_SendRequest_ASCII(RPC_id, Parameters, Response);

	RC_TYPE GRC, RC;
	uint32_t Transaction_id;
	
	// Format "%R1P,0,0:RC, DevicePrecisionClass, DeviceConfigurationType [long]"
	long int eClass, Type;
	int N = sscanf(Response, "%%R1P,%hd,%u:%hd,%ld,%ld",&GRC,&Transaction_id,&RC,
		       &eClass,
		       &Type);

	const int expect = 5;
	if(expect != N)printf("%s:%d Warning %s, only %d of %d\n", __FILE__, __LINE__, __FUNCTION__, N, expect);

	Device.eClass = (TPS_DEVICE_CLASS)eClass;
	Device.Type = (TPS_DEVICE_TYPE)Type;
	return RC;
}


// GeoCOM Reference Manual TPS1100 - Version 1.07
RC_TYPE CSV_CheckPower(unsigned long &unCapacity,
		CSV_POWER_PATH &eActivePower,
		CSV_POWER_PATH &ePowerSuggest){
  const uint32_t RPC_id = 5039;
  *Parameters = '\0';
  GeoCOM_SendRequest_ASCII(RPC_id, Parameters, Response);

	RC_TYPE GRC, RC;
	uint32_t Transaction_id;
	
	long ActivePower, PowerSuggest;
	// Format "%R1P,0,0:RC, unCapacity[long], eActivePower[long], ePowerSuggest[long]"
	int N = sscanf(Response, "%%R1P,%hd,%u:%hd,%lu,%ld,%ld",&GRC,&Transaction_id,&RC,
		       &unCapacity,
		       &ActivePower,
		       &PowerSuggest);

	const int expect = 6;
	if(expect != N)printf("%s:%d Warning %s, only %d of %d\n", __FILE__, __LINE__, __FUNCTION__, N, expect);

	eActivePower = (CSV_POWER_PATH)ActivePower;
	ePowerSuggest = (CSV_POWER_PATH)PowerSuggest;
	return RC;
}


// GeoCOM Reference Manual TPS1100 - Version 1.07
RC_TYPE CSV_GetVBat(double &dVBat){
  abort();	
}


// GeoCOM Reference Manual TPS1100 - Version 1.07
RC_TYPE BAP_GetMeasPrg(BAP_USER_MEASPRG &eMeasPrg){
  const uint32_t RPC_id = 17018;
  *Parameters = '\0';
  GeoCOM_SendRequest_ASCII(RPC_id, Parameters, Response);

	RC_TYPE GRC, RC;
	uint32_t Transaction_id;

	long int MeasPrg;
	// Format "%R1P,0,0:RC, eMeasPrg[long]"
	int N = sscanf(Response, "%%R1P,%hd,%u:%hd,%ld",&GRC,&Transaction_id,&RC,
		       &MeasPrg);

	const int expect = 4;
	if(expect != N)printf("%s:%d Warning %s, only %d of %d\n", __FILE__, __LINE__, __FUNCTION__, N, expect);

	eMeasPrg = (BAP_USER_MEASPRG)MeasPrg;
	return RC;
}


// GeoCOM Reference Manual TPS1100 - Version 1.07
RC_TYPE BAP_MeasDistanceAngle(	BAP_MEASURE_PRG &DistMode,
				double &dHz,
				double &dV,
				double &dDist ){
  const uint32_t RPC_id = 17017;
  sprintf(Parameters,"%d",DistMode);
#ifdef _DEBUG
  printf("\tDistMode: %s\n",Parameters);
#endif
  GeoCOM_SendRequest_ASCII(RPC_id, Parameters, Response);

	RC_TYPE GRC, RC;
	uint32_t Transaction_id;
	
	long dist_mode;
	// Format "%R1P,0,0:RC, dHz, dV, dDist[double], DistMode[long]"
	int N = sscanf(Response, "%%R1P,%hd,%u:%hd,%lf,%lf,%lf,%ld",&GRC,&Transaction_id,&RC,
		       &dHz,
		       &dV,
		       &dDist,
		       &dist_mode);

	const int expect = 7;
	if(expect != N)printf("%s:%d Warning %s, only %d of %d\n", __FILE__, __LINE__, __FUNCTION__, N, expect);

	DistMode = (BAP_MEASURE_PRG)dist_mode;
	return RC;
	
}


// GeoCOM Reference Manual TPS1100 - Version 1.07
RC_TYPE WIR_GetRecFormat(WIR_RECFORMAT &RecFormat){
  const uint32_t RPC_id = 8011;
  *Parameters = '\0';
  GeoCOM_SendRequest_ASCII(RPC_id, Parameters, Response);

	RC_TYPE GRC, RC;
	uint32_t Transaction_id;
	
	// Format "%R1P,0,0:RC, RecFormat[short]"
	int N = sscanf(Response, "%%R1P,%hd,%u:%hd,%hd",&GRC,&Transaction_id,&RC,
		       &RecFormat);

	const int expect = 4;
	if(expect != N)printf("%s:%d Warning %s, only %d of %d\n", __FILE__, __LINE__, __FUNCTION__, N, expect);

	return RC;
}


// GeoCOM Reference Manual TPS1100 - Version 1.07
RC_TYPE TMC_GetStation(TMC_STATION &Station){
  const uint32_t RPC_id = 2009;
  *Parameters = '\0';
  GeoCOM_SendRequest_ASCII(RPC_id, Parameters, Response);

	RC_TYPE GRC, RC;
	uint32_t Transaction_id;
	
	// Format "%R1P,0,0:RC,E0[double],N0[double],H0[double],Hi[double]"
	int N = sscanf(Response, "%%R1P,%hd,%u:%hd,%lf,%lf,%lf,%lf",&GRC,&Transaction_id,&RC,
		       &Station.dE0,
		       &Station.dN0,
		       &Station.dH0,
		       &Station.dHi);

	const int expect = 7;
	if(expect != N)printf("%s:%d Warning %s, only %d of %d\n", __FILE__, __LINE__, __FUNCTION__, N, expect);

	return RC;
}


// GeoCOM Reference Manual TPS1100 - Version 1.07
RC_TYPE TMC_DoMeasure(TMC_MEASURE_PRG Command,
		      TMC_INCLINE_PRG Mode){
  const uint32_t RPC_id = 2008;
  sprintf(Parameters,"%d,%d",Command,Mode);
  GeoCOM_SendRequest_ASCII(RPC_id, Parameters, Response);

	RC_TYPE GRC, RC;
	uint32_t Transaction_id;
	
	// Format "%R1P,0,0:RC"
	int N = sscanf(Response, "%%R1P,%hd,%u:%hd",&GRC,&Transaction_id,&RC);

	const int expect = 3;
	if(expect != N)printf("%s:%d Warning %s, only %d of %d\n", __FILE__, __LINE__, __FUNCTION__, N, expect);

	return 0;
}


// GeoCOM Reference Manual TPS1100 - Version 1.07
RC_TYPE TMC_GetCoordinate(SYSTIME WaitTime,
			  TMC_COORDINATE &Coordinate,
			  TMC_INCLINE_PRG Mode){
  const uint32_t RPC_id = 2082;
  sprintf(Parameters,"%ld",WaitTime);
  GeoCOM_SendRequest_ASCII(RPC_id, Parameters, Response);

	RC_TYPE GRC, RC;
	uint32_t Transaction_id;
	
	// Format "%R1P,0,0:RC, E[double],N[double],H[double],CoordTime[long],
	// 			E-Cont[double],N-Cont[double],H-Cont[double],CoordContTime[long]"
	int N = sscanf(Response, "%%R1P,%hd,%u:%hd,%lf,%lf,%lf,%ld,%lf,%lf,%lf,%ld",&GRC,&Transaction_id,&RC,
		       &Coordinate.dE,
		       &Coordinate.dN,
		       &Coordinate.dH,
		       &Coordinate.CoordTime,
		       &Coordinate.dE_Cont,
		       &Coordinate.dN_Cont,
		       &Coordinate.dH_Cont,
		       &Coordinate.CoordContTime);

	const int expect = 11;
	if(expect != N)printf("%s:%d Warning %s, only %d of %d\n", __FILE__, __LINE__, __FUNCTION__, N, expect);

	return RC;
}


// GeoCOM Reference Manual TPS1100 - Version 1.07
RC_TYPE TMC_GetSimpleMea(SYSTIME WaitTime,
		TMC_HZ_V_ANG &OnlyAngle,
		double &SlopeDistance,
		TMC_INCLINE_PRG Mode){
  const uint32_t RPC_id = 2108;
  sprintf(Parameters,"%ld,%d",WaitTime,Mode);
  GeoCOM_SendRequest_ASCII(RPC_id, Parameters, Response);

	RC_TYPE GRC, RC;
	uint32_t Transaction_id;
	
	// Format "%R1P,0,0:RC,Hz[double],V[double],SlopeDistance[double]"
	int N = sscanf(Response, "%%R1P,%hd,%u:%hd,%lf,%lf,%lf",&GRC,&Transaction_id,&RC,
		       &OnlyAngle.dHz,
		       &OnlyAngle.dV,
		       &SlopeDistance);

	const int expect = 6;
	if(expect != N)printf("%s:%d Warning %s, only %d of %d\n", __FILE__, __LINE__, __FUNCTION__, N, expect);

	return RC;
}



// ########################################################################
// End re-creation of Leica's GeoCOM interface, POSIX/Linux compatible
// ########################################################################
