#include "GeoCOM_displayInfo.h"
#include "GeoCOM_commands.h"

#include <stdio.h>
#include <string.h>

static void printDeviceType(FILE *f, const TPS_DEVICE &Device){
  switch(Device.eClass){
  case TPS_CLASS_1100:
    fprintf(f, "%%This is a TPS1000 family device, 1 mgon, 3inch\n");
    break;
  case TPS_CLASS_1700:
    fprintf(f, "%%This is a TPS1000 family device, 0.5 mgon, 1.5inch\n");
    break;
  case TPS_CLASS_1800:
    fprintf(f, "%%This is a TPS1000 family device, 0.3 mgon, 1inch\n");
    break;
  case TPS_CLASS_5000:
    fprintf(f, "%%This is a TPS2000 family device\n");
    break;
  case TPS_CLASS_6000:
    fprintf(f, "%%This is a TPS2000 family device\n");
    break;
  case TPS_CLASS_1500:
    fprintf(f, "%%This is a TPS1000 family device\n");
    break;
  case TPS_CLASS_2003:
    fprintf(f, "%%This is a TPS2000 family device\n");
    break;
  case TPS_CLASS_5005:
    fprintf(f, "%%This is a TPS5000 family device\n");
    break;
  case TPS_CLASS_5100:
    fprintf(f, "%%This is a TPS5000 family device\n");
    break;
  case TPS_CLASS_1102:
    fprintf(f, "%%This is a TPS1100 family device, 2inch\n");
    break;
  case TPS_CLASS_1103:
    fprintf(f, "%%This is a TPS1100 family device, 3inch\n");
    break;
  case TPS_CLASS_1105:
    fprintf(f, "%%This is a TPS1100 family device, 5inch\n");
    break;
  case TPS_CLASS_1101:
    fprintf(f, "%%This is a TPS1100 family device, 1inch\n");
    break;
  default:
    fprintf(f, "%%Unknown device class:\t%d\n",Device.eClass);
    break;
  }

  int numCap = 0;
  numCap += !!(Device.Type & TPS_DEVICE_T);
  numCap += !!(Device.Type & TPS_DEVICE_TC1);
  numCap += !!(Device.Type & TPS_DEVICE_TC2);
  numCap += !!(Device.Type & TPS_DEVICE_MOT);
  numCap += !!(Device.Type & TPS_DEVICE_ATR);
  numCap += !!(Device.Type & TPS_DEVICE_EGL);
  numCap += !!(Device.Type & TPS_DEVICE_DB);
  numCap += !!(Device.Type & TPS_DEVICE_DL);
  numCap += !!(Device.Type & TPS_DEVICE_LP);
  numCap += !!(Device.Type & TPS_DEVICE_ATC);
  numCap += !!(Device.Type & TPS_DEVICE_LPNT);
  numCap += !!(Device.Type & TPS_DEVICE_RL_EXT);
  numCap += !!(Device.Type & TPS_DEVICE_PS);
  numCap += !!(Device.Type & TPS_DEVICE_SIM);

  fprintf(f, "%% %d capabilities:\n", numCap);

  if(Device.Type & TPS_DEVICE_T)
    fprintf(f, "%%\tTheodolite without built-in EDM\n");
  if(Device.Type & TPS_DEVICE_TC1)
    fprintf(f, "%%\tTachymeter (older EDM built-in)\n");
  if(Device.Type & TPS_DEVICE_TC2)
    fprintf(f, "%%\tTachymeter (newer EDM built-in) Red Laser built-in\n");
  if(Device.Type & TPS_DEVICE_MOT)
    fprintf(f, "%%\tMotorized device\n");
  if(Device.Type & TPS_DEVICE_ATR)
    fprintf(f, "%%\tAutomatic Target Recognition\n");
  if(Device.Type & TPS_DEVICE_EGL)
    fprintf(f, "%%\tElectronic Guide Light\n");
  if(Device.Type & TPS_DEVICE_DB)
    fprintf(f, "%%\treserved (Database, not GSI)\n");
  if(Device.Type & TPS_DEVICE_DL)
    fprintf(f, "%%\tDiode laser\n");
  if(Device.Type & TPS_DEVICE_LP)
    fprintf(f, "%%\tLaser plummet\n");
  if(Device.Type & TPS_DEVICE_ATC)
    fprintf(f, "%%\tAutoCollimination Lamp\n");
  if(Device.Type & TPS_DEVICE_LPNT)
    fprintf(f, "%%\tLaserpointer\n");
  if(Device.Type & TPS_DEVICE_RL_EXT)
    fprintf(f, "%%\tRed laser with extended range\n");
  if(Device.Type & TPS_DEVICE_PS)
    fprintf(f, "%%\tPowersearch is build-in\n");
  if(Device.Type & TPS_DEVICE_SIM)
    fprintf(f, "%%\truns on Simulation, no Hardware\n");

}

static void printDevicePower(FILE *f, 
			     const unsigned long &unCapacity,
			     const CSV_POWER_PATH &eActivePower, 
			     const CSV_POWER_PATH &ePowerSuggest){
  fprintf(f, "%%Device Power level:\t%ld\n", unCapacity);

  fprintf(f, "%%Active Power:\t");
  switch(eActivePower){
  case CSV_CURRENT_POWER:
    fprintf(f, "\tActual Power Source\n");
    break;
  case CSV_EXTERNAL_POWER:
    fprintf(f, "\tExternal Power Source\n");
    break;
  case CSV_INTERNAL_POWER:
    fprintf(f, "\tInternal Power Source\n");
    break;
  default:
    fprintf(f, "\tUnknown Power Source\n");
    break;
  }

  fprintf(f, "%%Suggested Power:");
  switch(ePowerSuggest){
  case CSV_CURRENT_POWER:
    fprintf(f, "\tActual Power Source\n");
    break;
  case CSV_EXTERNAL_POWER:
    fprintf(f, "\tExternal Power Source\n");
    break;
  case CSV_INTERNAL_POWER:
    fprintf(f, "\tInternal Power Source\n");
    break;
  default:
    fprintf(f, "\tUnknown Power Source\n");
    break;
  }


}


static void printRecFormat(FILE *f, 
			   const WIR_RECFORMAT &RecFormat){
  switch(RecFormat){
  case WIR_RECFORMAT_GSI:
    fprintf(f, "%%Memory Format: GSI (8bit)\n");
    break;
  case WIR_RECFORMAT_GSI16:
    fprintf(f, "%%Memory Format: GSI-16 (16bit)\n");
    break;
  default:
    fprintf(f, "%%Memory Format: Unknown\n");
    break;
  }
}

static void printStation(FILE *f,
			 const TMC_STATION &Station){
  fprintf(f, "%%Station Information:\n");
  fprintf(f, "%%\tdE0\t%f\n",Station.dE0);
  fprintf(f, "%%\tdN0\t%f\n",Station.dN0);
  fprintf(f, "%%\tdH0\t%f\n",Station.dH0);
  fprintf(f, "%%\tdHi\t%f\n",Station.dHi);
}

void printDeviceInfo(FILE *f){
  RC_TYPE retval;

  // get device serial number
  long int SerialNumber=0;
  retval = CSV_GetInstrumentNo(SerialNumber);
  if(retval)
    fprintf(f, "%%Error %d getting instrument serial number\n", retval);
  fprintf(f, "%%Serial number %ld\n", SerialNumber);

  // get device name
  char Name[1024];
  memset(Name, 0, sizeof(Name));
  retval = CSV_GetInstrumentName(Name);
  if(retval)
    fprintf(f, "%%Error %d getting instrument name\n", retval);
  fprintf(f, "%%Device Name %s\n", Name);

  // get device config
  TPS_DEVICE Device;
  memset(&Device, 0, sizeof(Device));
  retval = CSV_GetDeviceConfig(Device);
  if(retval)
    fprintf(f, "%%Error %d getting device config\n", retval);
  printDeviceType(f, Device);

  // get device software version
  short nRelease=0, nVersion=0, nSubVersion=0;
  retval = CSV_GetSWVersion(nRelease, nVersion, nSubVersion);
  if(retval)
    fprintf(f, "%%Error %d getting software version\n", retval);
  fprintf(f, "%%Software Version \t%02d.%02d.%02d\n",
	  nRelease, nVersion, nSubVersion);

  // get date/time
  DATIME Device_DateTime;
  memset(&Device_DateTime, 0, sizeof(Device_DateTime));
  retval = CSV_GetDateTime(Device_DateTime);
  if(retval)
    fprintf(f, "%%Error %d getting date/time\n", retval);
  fprintf(f, "%%Device's Date/Time \t%d-%d-%d %d:%d:%d\n",
	  Device_DateTime.Date.Year,
	  Device_DateTime.Date.Month,
	  Device_DateTime.Date.Day,
	  Device_DateTime.Time.Hour,
	  Device_DateTime.Time.Minute,
	  Device_DateTime.Time.Second);

  unsigned long unCapacity=0;
  CSV_POWER_PATH eActivePower, ePowerSuggest;
  memset(&eActivePower, 0, sizeof(eActivePower));
  memset(&ePowerSuggest, 0, sizeof(ePowerSuggest));
  retval = CSV_CheckPower(unCapacity, eActivePower, ePowerSuggest);
  if(retval)
    fprintf(f, "%%Error %d getting power information\n", retval);
  printDevicePower(f, unCapacity, eActivePower, ePowerSuggest);


  WIR_RECFORMAT RecFormat;
  memset(&RecFormat, 0, sizeof(RecFormat));
  retval = WIR_GetRecFormat(RecFormat);
  if(retval)
    fprintf(f, "%%Error %d getting format information\n", retval);
  printRecFormat(f, RecFormat);


  // get station
  TMC_STATION Station;
  memset(&Station, 0, sizeof(Station));
  retval = TMC_GetStation(Station);
  if(retval)
    fprintf(f, "%%Error %d getting station information\n", retval);
  printStation(f, Station);
  
}
