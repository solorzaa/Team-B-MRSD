#ifndef LIB_GEOCOM_H
#define LIB_GEOCOM_H

#define REQUEST_LENGTH 1024
#define RESPONSE_LENGTH 1024

#define MAXSTRING 1024

// unknown, this should be in Leica's header, but was only found in GeoCOM Reference Manual 1.07

enum CSV_POWER_PATH
{
	CSV_CURRENT_POWER = 0,	// Actual Power Source
	CSV_EXTERNAL_POWER = 1,	// Power source is external
	CSV_INTERNAL_POWER = 2 	// Power source is internal batter
	};

enum TPS_DEVICE_CLASS
{
	TPS_CLASS_1100 = 0, // TPS1000 family member 1 mgon, 3"
	TPS_CLASS_1700 = 1, // TPS1000 family member 0.5 mgon, 1.5"
	TPS_CLASS_1800 = 2, // TPS1000 family member 0.3 mgon, 1"
	TPS_CLASS_5000 = 3, // TPS2000 family member
	TPS_CLASS_6000 = 4, // TPS2000 family member
	TPS_CLASS_1500 = 5, // TPS1000 family member
	TPS_CLASS_2003 = 6, // TPS2000 family member
	TPS_CLASS_5005 = 7, // TPS5000 family member
	TPS_CLASS_5100 = 8, // TPS5000 family member
	TPS_CLASS_1102 = 100, // TPS1100 family member, 2"
	TPS_CLASS_1103 = 101, // TPS1100 family member, 3"
	TPS_CLASS_1105 = 102, // TPS1100 family member, 5"
	TPS_CLASS_1101 = 103 // TPS1100 family member, 1"
};

enum TPS_DEVICE_TYPE
{
	TPS_DEVICE_T = 0x00000,		// theodolite without built-in EDM
	TPS_DEVICE_TC1 = 0x00001,	// tachymeter built-in
	TPS_DEVICE_TC2 = 0x00002,	// tachymeter with red laser built-in
	TPS_DEVICE_MOT = 0x00004,	// motorized device
	TPS_DEVICE_ATR = 0x00008,	// automatic target recognition
	TPS_DEVICE_EGL = 0x00010,	// electronic guide light
	TPS_DEVICE_DB = 0x00020,	// reserved
	TPS_DEVICE_DL = 0x00040,	// diode laser
	TPS_DEVICE_LP = 0x00080,	// laser plummet
	TPS_DEVICE_ATC = 0x00100,	// autocollimination lamp
	TPS_DEVICE_LPNT= 0x00200,	// Laserpointer
	TPS_DEVICE_RL_EXT = 0x00400,	// Red laser with extended range
	TPS_DEVICE_PS = 0x00800,	// Powersearch is build-in
	TPS_DEVICE_SIM = 0x04000	// runs on simulation, not on hardware
};
struct  TPS_DEVICE
     {
	TPS_DEVICE_CLASS  eClass;   // device precision class
	TPS_DEVICE_TYPE   Type;     // device configuration
};


enum BAP_USER_MEASPRG {
	BAP_SINGLE_REF_STANDARD = 0,	// standard single IR distance with reflector
	BAP_SINGLE_REF_FAST = 1,	// fast single IR distance with reflector
	BAP_SINGLE_REF_VISIBLE = 2,	// long range distance with reflector (red laser)
	BAP_SINGLE_RLESS_VISIBLE = 3,	// single RL distance, reflector free (red laser)
	BAP_CONT_REF_STANDARD = 4,	// tracking IR distance with reflector
	BAP_CONT_REF_FAST = 5,		// fast tracking IR distance with reflector
	BAP_CONT_RLESS_VISIBLE = 6,	// fast tracking RL distance, reflector free (red)
	BAP_AVG_REF_STANDARD = 7,	// Average IR distance with reflector
	BAP_AVG_REF_VISIBLE = 8,	// Average long range dist. with reflector (red)
	BAP_AVG_RLESS_VISIBLE = 9	// Average RL distance, reflector free (red laser)
	};

#endif
