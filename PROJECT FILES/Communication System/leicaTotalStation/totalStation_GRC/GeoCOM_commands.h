#ifndef GEOCOM_COMMANDS_H
#define GEOCOM_COMMANDS_H

#include "libGeoCOM.h"
#include "libGeoCOM_win.h"

/*
 * Defines the various commands you can use with the station.
 * The important ones are the last three:
 *    DoMeasure      requests a measurements
 *    GetCoordinate  gets the 3d point of the last measurement
 *    GetSimpleMea   gets the pointing angles and distance
 */

RC_TYPE BMM_BeepNormal(void);
RC_TYPE IOS_BeepOff(void);
RC_TYPE CSV_GetInstrumentNo(long &SerialNo);
RC_TYPE CSV_GetInstrumentName(char *Name);
RC_TYPE CSV_GetDateTime (DATIME &Datime);
RC_TYPE CSV_GetSWVersion(short &nRelease, short &nVersion, short &nSubVersion);
RC_TYPE CSV_GetDeviceConfig(TPS_DEVICE &Device);
RC_TYPE CSV_CheckPower(unsigned long &unCapacity,
		       CSV_POWER_PATH &eActivePower,
		       CSV_POWER_PATH &ePowerSuggest);
RC_TYPE CSV_GetVBat(double &dVBat);
RC_TYPE BAP_GetMeasPrg(BAP_USER_MEASPRG &eMeasPrg);
RC_TYPE BAP_MeasDistanceAngle(	BAP_MEASURE_PRG &DistMode,
				double &dHz,
				double &dV,
				double &dDist );
RC_TYPE WIR_GetRecFormat(WIR_RECFORMAT &RecFormat);
RC_TYPE TMC_GetStation(TMC_STATION &Station);
RC_TYPE TMC_DoMeasure(TMC_MEASURE_PRG Command,
		      TMC_INCLINE_PRG Mode);
RC_TYPE TMC_GetCoordinate(SYSTIME WaitTime,
			  TMC_COORDINATE &Coordinate,
			  TMC_INCLINE_PRG Mode);
RC_TYPE TMC_GetSimpleMea(SYSTIME WaitTime,
			 TMC_HZ_V_ANG &OnlyAngle,
			 double &SlopeDistance,
			 TMC_INCLINE_PRG Mode);

#endif
