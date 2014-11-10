/* ================================================================== *
*                                                                    *
*  Copyright by Leica AG, Heerbrugg,   Switzerland 1997              *
* __________________________________________________________________ *
*                                                                    *
*  Project:        BETA II                 Division:   Geodesy D+T   *
*                                                                    *
*  File:           com_pub.hpp                           *
*                                                                    *
*  Title:          public  geocom collection header                   *
*                                                                    *
*  Author:         com_extr, file-generator Written:  16 Apr 1997    *
*  Released:       Ver. 00.00               Date:     dd MMM yyyy    *
*                                                                    *
*  Environment:    pc compaq 386, pc compaq 486, pc dell 586         *
*                  Microsoft Visual C++ (V 1.52)                     *
* __________________________________________________________________ *
*                                                                    *
*  DESCRIPTION:                                                      *
*                                                                    *
*  This is a generated header-file.                                  *
*                                                                    *
*                                                                    *
* ================================================================== */


#ifndef COM_PUBLIC_USE_HPP
#define COM_PUBLIC_USE_HPP


#ifdef  _MRI_EXTENSIONS
#define  PKD
#else
#define  PKD
#endif                

#ifndef NULL
#ifdef __cplusplus
#define NULL    0
#else 
#define NULL    ((void *)0)
#endif 
#endif

#ifdef TRUE
#undef TRUE
#endif

#ifdef FALSE
#undef FALSE
#endif


typedef unsigned char   BYTE;           // 1 byte range 0..255

typedef short           RC_TYPE;        // return code type

typedef long            SYSTIME;        // time since poweron [ms]


/* ------------------------------------------------------------------
   Return code offsets for subsystem specific codes
   with a maximum of 256 in rangesize per component

   !!! NOTE !!! The next defined constants are used in the com_extr.exe
                utility. If you change anything here you have to change
                it there too !!!
   ------------------------------------------------------------------ */
const RC_TYPE
                RCBETA  = 0x0000,   // main return codes (identical to RC_SUP!!)
                RC_ALT  = 0x2100,   // alternate user task
                RC_ANG  = 0x0100,   // angle- and inclination
                RC_ATA  = 0x0200,   // automatic target acquisition
                RC_AUS  = 0x2300,   // alternative user
                RC_AUT  = 0x2200,   // automatization
                RC_BAP  = 0x2400,   // basic applications
                RC_BAS  = 0x2700,   // GeoBasic interpreter
                RC_BMM  = 0x0900,   // basics of man machine interface
                RC_COD  = 0x2600,   // standard code function
                RC_COM  = 0x0C00,   // communication
                RC_CSV  = 0x1000,   // central services
                RC_CTL  = 0x1100,   // controlling task
                RC_DBM  = 0x0D00,   // data base management
                RC_DEL  = 0x0E00,   // dynamic event logging
                RC_DPL  = 0x1300,   // data pool
                RC_EDM  = 0x0300,   // electronic distance meter
                RC_FIL  = 0x0F00,   // file system
                RC_GMF  = 0x0400,   // geodesy mathematics & formulas
                RC_LDR  = 0x0800,   // program loader
                RC_MEM  = 0x0600,   // memory management
                RC_MMI  = 0x0B00,   // man machine interface
                RC_MOT  = 0x0700,   // motorization
                RC_SAP  = 0x2500,   // system applications
                RC_STP  = 0x1200,   // start + stop task
                RC_SUP  = 0x0000,   // supervisor task (identical to RCBETA!!)
                RC_TMC  = 0x0500,   // measurement & calc
                RC_TXT  = 0x0A00,   // text management
                RC_USR  = 0x2000,   // user task
                RC_WIR  = 0x1400,   // wi registration

                RC_APP  = 0x5000,   // offset for all applications
                RC_RES  = 0x7000;   // reserved code range 
 
/* ------------------------------------------------------------------
   Global return code values        (extended when necessary)
   ------------------------------------------------------------------ */
const RC_TYPE
   RC_OK           = RCBETA +  0,   // Function successfully completed.
   RC_UNDEFINED    = RCBETA +  1,   // Unknown error, result unspecified.
   RC_IVPARAM      = RCBETA +  2,   // Invalid parameter detected.\nResult unspecified.
   RC_IVRESULT     = RCBETA +  3,   // Invalid result.
   RC_FATAL        = RCBETA +  4,   // Fatal error.
   RC_NOT_IMPL     = RCBETA +  5,   // Not implemented yet.
   RC_TIME_OUT     = RCBETA +  6,   // Function execution timed out.\nResult unspecified.
   RC_SET_INCOMPL  = RCBETA +  7,   // Parameter setup for subsystem is incomplete.
   RC_ABORT        = RCBETA +  8,   // Function execution has been aborted.
   RC_NOMEMORY     = RCBETA +  9,   // Fatal error - not enough memory.
   RC_NOTINIT      = RCBETA + 10,   // Fatal error - subsystem not initialized.
   RC_SHUT_DOWN    = RCBETA + 12,   // Subsystem is down.
   RC_SYSBUSY      = RCBETA + 13,   // System busy/already in use of another process.\nCannot execute function.
   RC_HWFAILURE    = RCBETA + 14,   // Fatal error - hardware failure.
   RC_ABORT_APPL   = RCBETA + 15,   // Execution of application has been aborted (SHIFT-ESC).
   RC_LOW_POWER    = RCBETA + 16;   // Operation aborted - insufficient power supply level.

/* ------------------------------------------------------------------
   Enumerated   types
   ------------------------------------------------------------------ */
PKD enum BOOLE              // BOOLEan type
    {
     FALSE, TRUE
    };

PKD enum ON_OFF_TYPE        // on/off switch type
    {
     OFF, ON
    };

PKD enum INIT_MODE          // initialization modes
    {   
     INSTALLATION,      // only for the first run after installation
     FACTORY_DEFAULT,   // set all values to factory default and check
                        // all settings and calibration values
     POWER_ON,          // power on after normal off
     XPOWER_ON,         // extra power on after anormal off (without Stop)
     USER_DEFAULT,      // set values of the current user to default
     USER_CHANGED,      // current user has changed, get new user values
     RESTART            // restart after sleeping
    };   

PKD enum STOP_MODE      // stop modes
    {
     SHUT_DOWN,         // The instrument is powered-down, each subsystem
                        // is responsible for terminating its functions
                        // and internal tasks.
     SLEEP              // Instrument is in sleep state.
    };                 


/* ------------------------------------------------------------------
   Definition of the TPS Device configuration. 
   It describes build in hardware.
   ------------------------------------------------------------------ */
// Combinations are possible, 
//   i.e. TCA1700: TPS_DEVICE_TC2 | TPS_DEVICE_MOT | TPS_DEVICE_ATR
#if 0
PKD enum TPS_DEVICE_TYPE
     {
        TPS_DEVICE_T   = 0x00000,   // Theodolite without built-in EDM
        TPS_DEVICE_TC1 = 0x00001,   // taChymeter (older EDM built-in)
        TPS_DEVICE_TC2 = 0x00002,   // taChymeter (newer EDM built-in)
        TPS_DEVICE_MOT = 0x00004,   // Motorized device
        TPS_DEVICE_ATR = 0x00008,   // Automatic Target Recognition
        TPS_DEVICE_EGL = 0x00010,   // Electronic Guide Light
        TPS_DEVICE_DB  = 0x00020,   // reserved (Database, not GSI)
        TPS_DEVICE_DL  = 0x00040,   // Diode laser
        TPS_DEVICE_LP  = 0x00080,   // Laser plumbed
                                    // future...
        TPS_DEVICE_SIM = 0x04000    // runs on Simulation, no Hardware
     };
#endif
/* ------------------------------------------------------------------
   Definition of the TPS Device precision class 
   ------------------------------------------------------------------ */
#if 0
enum TPS_DEVICE_CLASS
    {                       // family --------------- accuracy
    TPS_CLASS_1100,         // TPS1000 family member, 1 mgon,   3"
    TPS_CLASS_1700,         // TPS1000 family member, 0.5 mgon, 1.5"
    TPS_CLASS_1800,         // TPS1000 family member, 0.3 mgon, 1"
    TPS_CLASS_5000,         // TPS2000 family member
    TPS_CLASS_6000,         // TPS2000 family member 
    TPS_CLASS_1500          // TPS1000 family member
    };
//                    

/* ------------------------------------------------------------------
   Structure    TPS_DEVICE   TPS device configuration type
   ------------------------------------------------------------------ */
struct  TPS_DEVICE
     {
        TPS_DEVICE_CLASS  eClass;   // device precision class
        TPS_DEVICE_TYPE   Type;     // device configuration
     };
#endif


/* ------------------------------------------------------------------
   Structure    DATE_TYPE   general date type
   ------------------------------------------------------------------ */
struct  DATE_TYPE
      {
        short   Year;       // year
        BYTE    Month;      // month in year                1..12
        BYTE    Day;        // day in month                 1..31
      };
/* ------------------------------------------------------------------
   Structure    TIME_TYPE   general time type
   ------------------------------------------------------------------ */
struct  TIME_TYPE
      {
        BYTE    Hour;       // 24 hour per day  0..23
        BYTE    Minute;     // minute           0..59
        BYTE    Second;     // seconds          0..59
      };   

/* ------------------------------------------------------------------
   Structure    DATIME      global date & time type
   ------------------------------------------------------------------ */
struct  DATIME
      {
        DATE_TYPE   Date;
        TIME_TYPE   Time;
      };


/* ================================================================== *\
   Klassenvereinbarungen
\* ================================================================== */
typedef enum
{
    AUTO_POWER_DISABLED,        // Keine Wirkung der Automatik
    AUTO_POWER_SLEEP,           // Automatik legt System schlafen
    AUTO_POWER_OFF              // Automatik schaltet System aus
} SUP_AUTO_POWER; 


extern RC_TYPE SUP_GetConfig (ON_OFF_TYPE &eLowTempOnOff,
                              SUP_AUTO_POWER &eAutoPower,
                              SYSTIME &Timeout);                             
extern RC_TYPE SUP_SetConfig (ON_OFF_TYPE eLowTempOnOff= ON,
                              SUP_AUTO_POWER eAutoPower= AUTO_POWER_SLEEP,
                              SYSTIME Timeout= 900000);
extern RC_TYPE SUP_SwitchLowTempControl (ON_OFF_TYPE eLowTempOnOff);

        
typedef enum
{
  ANG_NOT_DEF,
  ANG_TRANSM,                    // Durchlicht-System
  ANG_REFLEX                     // Reflex-System
}
ANG_INCL_DEVICE; 


typedef unsigned short  TXT_TOKEN;     // text token

                                                 
const TXT_TOKEN TXT_NIL_TOKEN = 0xFFFF;


const RC_TYPE
  TXT_OTHER_LANG        = RC_TXT+0,     // text found, but in an other language
  TXT_UNDEF_TOKEN       = RC_TXT+1,     // text not found, token is undefined
  TXT_UNDEF_LANG        = RC_TXT+2,     // language is not defined
  TXT_TOOMANY_LANG      = RC_TXT+3,     // maximal number of languages reached
  TXT_GROUP_OCC         = RC_TXT+4,     // desired text group is already in use
  TXT_INVALID_GROUP     = RC_TXT+5,     // text group is invalid
  TXT_OUT_OF_MEM        = RC_TXT+6,     // out of text memory
  TXT_MEM_ERROR         = RC_TXT+7,     // memory write / allocate error
  TXT_TRANSFER_PENDING  = RC_TXT+8,     // text transfer is alread open
  TXT_TRANSFER_ILLEGAL  = RC_TXT+9,     // text transfer is not opened
  TXT_INVALID_SIZE      = RC_TXT+10,    // illegal text data size
  TXT_ALREADY_EXIST     = RC_TXT+11;    // language already exists


// Fehlercodes
static const RC_TYPE                // Basis: 0x0800
  LDR_PENDING           = RC_LDR+0, // Transfer is already open
  LDR_PRGM_OCC          = RC_LDR+1, // Maximal number of applications reached
  LDR_TRANSFER_ILLEGAL  = RC_LDR+2, // No Transfer is open
  LDR_NOT_FOUND         = RC_LDR+3, // Function or program not found
  LDR_ALREADY_EXIST     = RC_LDR+4, // Loadable object already exists
  LDR_NOT_EXIST         = RC_LDR+5, // Can't delete. Object does not exist
  LDR_SIZE_ERROR        = RC_LDR+6, // Error in loading object
  LDR_MEM_ERROR         = RC_LDR+7, // Error at memory allocation/release
  LDR_PRGM_NOT_EXIST    = RC_LDR+8, // Can't load textobject because application does not exist
  LDR_FUNC_LEVEL_ERR    = RC_LDR+9, // Callstack limit reached
  LDR_RECURSIV_ERR      = RC_LDR+10,// Recursive calling of an loaded function
  LDR_INST_ERR          = RC_LDR+11,// Error in installation function
  LDR_FUNC_OCC          = RC_LDR+12,// Maximal number of functions reached
  LDR_RUN_ERROR         = RC_LDR+13,// Error during a loaded application program
  LDR_DEL_MENU_ERR      = RC_LDR+14,// Error during deleting of menu entries of an application
  LDR_OBJ_TYPE_ERROR    = RC_LDR+15,// Loadable object is unknown
  LDR_WRONG_SECKEY      = RC_LDR+16,// Wrong security key
  LDR_ILLEGAL_LOADADR   = RC_LDR+17,// Illegal application memory address
  LDR_IEEE_ERROR        = RC_LDR+18,// Loadable object file is not IEEE format
  LDR_WRONG_APPL_VERSION = RC_LDR+19; // Bad application version number


// Beep-Definitionen
const short BMM_BEEP_STDINTENS = 100;       // Standard-Intensität des Beep
const short BMM_BEEP_STDFREQ   = 3900;      // Standard Frequenz des Beep


                         
extern RC_TYPE BMM_BeepOn (short nIntens= BMM_BEEP_STDINTENS,
                           short nFreq= BMM_BEEP_STDFREQ);
extern RC_TYPE BMM_BeepOff (void);



// RTC-Funktionen
extern RC_TYPE CSV_SetDateTime (DATIME Datime);
extern RC_TYPE CSV_GetDateTime (DATIME &Datime);


const RC_TYPE
    BMM_XFER_PENDING           = RC_BMM+1,  // Loading process already opened
    BMM_NO_XFER_OPEN           = RC_BMM+2,  // Transfer not opened
    BMM_UNKNOWN_CHARSET        = RC_BMM+3,  // Unknown character set
    BMM_NOT_INSTALLED          = RC_BMM+4,  // Display module not present
    BMM_ALREADY_EXIST          = RC_BMM+5,  // Character set already exists
    BMM_CANT_DELETE            = RC_BMM+6,  // Character set cannot be deleted
    BMM_MEM_ERROR              = RC_BMM+7,  // Memory cannot be allocated
    BMM_CHARSET_USED           = RC_BMM+8,  // Character set still used
    BMM_CHARSET_SAVED          = RC_BMM+9,  // Charset cannot be deleted or is protected
    BMM_INVALID_ADR            = RC_BMM+10, // Attempt to copy a character block\noutside the allocated memory
    BMM_CANCELANDADR_ERROR     = RC_BMM+11, // Error during release of allocated memory
    BMM_INVALID_SIZE           = RC_BMM+12, // Number of bytes specified in header\ndoes not match the bytes read
    BMM_CANCELANDINVSIZE_ERROR = RC_BMM+13, // Allocated memory could not be released
    BMM_ALL_GROUP_OCC          = RC_BMM+14, // Max. number of character sets already loaded
    BMM_CANT_DEL_LAYERS        = RC_BMM+15, // Layer cannot be deleted
    BMM_UNKNOWN_LAYER          = RC_BMM+16, // Required layer does not exist
    BMM_INVALID_LAYERLEN       = RC_BMM+17; // Layer length exceeds maximum


extern RC_TYPE BMM_BeepShort  (void);
extern RC_TYPE BMM_BeepNormal (void);
extern RC_TYPE BMM_BeepAlarm  (void);


RC_TYPE CTL_GetUpCounter (short &nPowerOn, short &nSleepMode);

 
extern RC_TYPE CSV_GetSWVersion(short &nRelease, short &nVersion, short & nSubVersion);
extern RC_TYPE CSV_GetInstrumentNo(long &lSerialNo);
extern RC_TYPE CSV_GetInstrumentName(char *szName);


extern RC_TYPE CSV_SetUserInstrumentName(char *szName);
extern RC_TYPE CSV_GetUserInstrumentName(char *szName);

extern RC_TYPE CSV_GetDeviceConfig(TPS_DEVICE &Device);     


extern RC_TYPE CSV_GetVBat(double &dVBat);  
extern RC_TYPE CSV_GetVMem(double &dVMem);
extern RC_TYPE CSV_GetIntTemp(double &dIntTemp);


const RC_TYPE
  EDM_COMERR                = RC_EDM + 1,  // communication with edm failed
  EDM_NOSIGNAL              = RC_EDM + 2,  // no signal
  EDM_PPM_MM                = RC_EDM + 3,  // PPM and\or MM not zero
  EDM_METER_FEET            = RC_EDM + 4,  // EDM unit not set to meter
  EDM_ERR12                 = RC_EDM + 5,  // battery low
  EDM_DIL99                 = RC_EDM + 6,  // limit at 99 measurments (DIL)
  EDM_SLDR_TRANSFER_PENDING = RC_EDM + 7,  // multiple opentransfer
  EDM_SLDR_TRANSFER_ILLEGAL = RC_EDM + 8,  // no opentransfer happened
  EDM_SLDR_DATA_ERROR       = RC_EDM + 9,  // unexpected data format received
  EDM_SLDR_CHK_SUM_ERROR    = RC_EDM + 10, // checksum error in transmitted data
  EDM_SLDR_ADDR_ERROR       = RC_EDM + 11, // address out of valid range
  EDM_SLDR_INV_LOADFILE     = RC_EDM + 12,  //Firmware file has invalid format.
  EDM_SLDR_UNSUPPORTED      = RC_EDM + 13;  //Current (loaded) firmware doesn't support upload.


typedef enum
{                             /* nicht in allen EDM implementiert   */
  EDM_SINGLE_STANDARD,       /* Einzelmessung   (DIST, g)          */
  EDM_SINGLE_EXACT,          /* Einzelmessung   (GDIST, l)         */
  EDM_SINGLE_FAST,           /* Einzelmessung   (DI, j)            */
  EDM_CONT_STANDARD,         /* Rep. DIST       (TRK, h)           */
  EDM_CONT_EXACT,            /* Mittelwert DIST (DIL, i)           */
  EDM_CONT_FAST,             /* Rep. DIST       (RTRK, m)          */
  EDM_UNDEFINED              /* nicht definiert                    */

} EDM_MODE;


typedef enum
{
  EDM_LOW_BRIGHTNESS,
  EDM_MEDIUM_BRIGHTNESS,
  EDM_HIGH_BRIGHTNESS

} EDM_TRKLIGHT_BRIGHTNESS;


RC_TYPE EDM_On(ON_OFF_TYPE eOn);            

RC_TYPE EDM_Laserpointer(ON_OFF_TYPE Laser);   

RC_TYPE EDM_SetBumerang(ON_OFF_TYPE  eOn);
RC_TYPE EDM_GetBumerang(ON_OFF_TYPE& eBoomerangFilter );  


RC_TYPE EDM_SetTrkLightSwitch( ON_OFF_TYPE eOnOff );  
RC_TYPE EDM_GetTrkLightSwitch( ON_OFF_TYPE &eOnOff ); 

RC_TYPE EDM_SetTrkLightBrightness( EDM_TRKLIGHT_BRIGHTNESS eBrightness );
RC_TYPE EDM_GetTrkLightBrightness( EDM_TRKLIGHT_BRIGHTNESS &eBrightness );  


const RC_TYPE
  TMC_NO_FULL_CORRECTION = RC_TMC + 3,       // Warning: measurment without full correction
  TMC_ACCURACY_GUARANTEE = RC_TMC + 4,       // Info   : accuracy can not be guarantee

  TMC_ANGLE_OK                 = RC_TMC + 5, // Warning: only angle measurement valid
  TMC_ANGLE_NO_FULL_CORRECTION = RC_TMC + 8, // Warning: only angle measurement valid but without full correction
  TMC_ANGLE_ACCURACY_GUARANTEE = RC_TMC + 9, // Info   : only angle measurement valid but accuracy can not be guarantee

  TMC_ANGLE_ERROR       = RC_TMC + 10,       // Error  : no angle measurement

  TMC_DIST_PPM          = RC_TMC + 11,       // Error  : wrong setting of PPM or MM on EDM
  TMC_DIST_ERROR        = RC_TMC + 12,       // Error  : distance measurement not done (no aim, etc.)
  TMC_BUSY              = RC_TMC + 13,       // Error  : system is busy (no measurement done)
  TMC_SIGNAL_ERROR      = RC_TMC + 14;       // Error  : no signal on EDM (only in signal mode)



typedef enum
{
    TMC_FACE_NORMAL,      /* Die Lage entspricht den Sensorwerten (V) */
    TMC_FACE_TURN         /*                      Spiegelung der Lage */

} TMC_FACE_DEF;


typedef enum
{
    TMC_FACE_1,                             /* Lage 1 der Fernrohrs */
    TMC_FACE_2                              /* Lage 2 der Fernrohrs */

} TMC_FACE;


typedef enum
{                           /* NEIGUNGSMESSUNG        ABHÂŽNGIG SETUP */
    TMC_MEA_INC     = 0,    /* Sensor (Apriori Sigma)       ja       */
    TMC_AUTO_INC    = 1,    /* Automatismus (Sensor/Ebene)  ja       */
    TMC_PLANE_INC   = 2     /* Ebene  (Apriori Sigma)       ja       */


} TMC_INCLINE_PRG;


typedef enum              /*      TMC                        INCLINE       */
{                         /*   Messprogramm              Messmode Messzeit */
    TMC_STOP       = 0,   /* Stopt das Messprogramm        nein     nein   */
    TMC_DEF_DIST   = 1,   /* Default DIST-Messprogramm     ja       nein   */
    TMC_TRK_DIST   = 2,   /* Distanz-TRK Messprogramm      ja       nein   */
    TMC_CLEAR      = 3,   /* TMC_STOP mit Clear Data       nein     nein   */
    TMC_SIGNAL     = 4,   /* Signalmessungen (Testfkt.)    nein     nein   */


    TMC_RTRK_DIST  = 8    /* Distanz-RTRK Messprogramm     ja       nein   */


} TMC_MEASURE_PRG;


/* ================================================================== *\
   Typ Definitionen fuer Messungen
\* ================================================================== */
typedef struct
{
    double dCrossIncline;                 /*             Querneigung */
    double dLengthIncline;                /*           Laengsneigung */
    double dAccuracyIncline;              /* Genauigkeit der Neigung */
    SYSTIME InclineTime;                  /*   Zeitpunkt der Messung */

} TMC_INCLINE;

typedef struct
{
    double dHz;                   /*              Horizontalwinkel */
    double dV;                    /*                Vertikalwinkel */
    double dAngleAccuracy;        /*       Genauigkeit der Winkels */
    SYSTIME AngleTime;            /*         Zeitpunkt der Messung */

    TMC_INCLINE Incline;          /*         Dazugehoerige Neigung */
    TMC_FACE eFace;               /* Lageinformation des Fernrohrs */

} TMC_ANGLE;

typedef struct
{
    double dHz;                      /*           Horizontalwinkel */
    double dV;                       /*             Vertikalwinkel */

} TMC_HZ_V_ANG;


typedef struct
{
    double dE;              /*                       E-Koordinaten */
    double dN;              /*                       N-Koordinaten */
    double dH;              /*                       H-Koordinaten */
    SYSTIME CoordTime;      /*               Zeitpunkt der Messung */

    double dE_Cont;         /*       E-Koordinate (kontinuierlich) */
    double dN_Cont;         /*       N-Koordinate (kontinuierlich) */
    double dH_Cont;         /*       H-Koordinate (kontinuierlich) */
    SYSTIME CoordContTime;  /*               Zeitpunkt der Messung */

} TMC_COORDINATE;


typedef struct
{
    double dSignalIntensity;        /*  Signalstaerke des EDM's in % */
    SYSTIME Time;                   /* Zeitpunkt der letzten Messung */

} TMC_EDM_SIGNAL;

/* ================================================================== *\
   Typ Definitionen fuer Korrekturwerte
\* ================================================================== */

typedef struct
{
    ON_OFF_TYPE eInclineCorr;             /*         Neigungskorrektur */
    ON_OFF_TYPE eStandAxisCorr;           /*                 Stehachse */
    ON_OFF_TYPE eCollimationCorr;         /*        Kollimationsfehler */
    ON_OFF_TYPE eTiltAxisCorr;            /*                 Kippachse */

} TMC_ANG_SWITCH;


typedef struct
{
  double  dLambda;                // Wellenlaenge der Traegerwelle
  double  dPressure;              // Luftdruck
  double  dDryTemperature;        // Trockentemperatur
  double  dWetTemperature;        // Feuchtetemperatur

} TMC_ATMOS_TEMPERATURE;

typedef struct
{
  ON_OFF_TYPE eRefOn;               // Gueltigkeit der Refraktion
  double      dEarthRadius;         // Erdradius
  double      dRefractiveScale;     // Refraktionskoeffizient

} TMC_REFRACTION;


typedef struct
{
    double dHr;                     /*              Reflektorhoehe */

} TMC_HEIGHT;

typedef struct
{
    double dE0;                     /*            Standpunkt-Koordinate */
    double dN0;                     /*            Standpunkt-Koordinate */
    double dH0;                     /*            Standpunkt-Koordinate */

    double dHi;                     /*           Instrumentenhoehe */

} TMC_STATION;

typedef struct
{
    double dLengthVal;              // Aim-Offset Lenght
    double dCrossVal;               // Aim-Offset Cross
    double dHeightVal;              // Aim-Offset Height

} TMC_OFFSETDIST;



RC_TYPE TMC_SetAngSwitch( TMC_ANG_SWITCH SwCorr );
RC_TYPE TMC_GetAngSwitch( TMC_ANG_SWITCH &SwCorr );

RC_TYPE TMC_SetInclineSwitch( ON_OFF_TYPE eSwCorr );
RC_TYPE TMC_GetInclineSwitch( ON_OFF_TYPE  &eSwCorr );

RC_TYPE TMC_GetSignal( TMC_EDM_SIGNAL &Signal );

RC_TYPE TMC_QuickDist      ( TMC_HZ_V_ANG    &OnlyAngle,
                             double          &dSlopeDistance );

RC_TYPE TMC_GetSimpleMea   ( SYSTIME         WaitTime,
                             TMC_HZ_V_ANG    &OnlyAngle,
                             double          &dSlopeDistance,
                             TMC_INCLINE_PRG eMode = TMC_AUTO_INC );

RC_TYPE TMC_GetSimpleCoord ( SYSTIME         WaitTime,
                             double          &dCoordE,
                             double          &dCoordN,
                             double          &dCoordH,
                             TMC_INCLINE_PRG eMode = TMC_AUTO_INC );

RC_TYPE TMC_SetHandDist    ( double          dSlopeDistance,
                             double          dHgtOffset,
                             TMC_INCLINE_PRG eMode = TMC_AUTO_INC );

RC_TYPE TMC_GetAngle( TMC_ANGLE       &Angle,
                      TMC_INCLINE_PRG eMode = TMC_AUTO_INC );

RC_TYPE TMC_GetAngle( TMC_HZ_V_ANG    &OnlyAngle,
                      TMC_INCLINE_PRG eMode = TMC_AUTO_INC );

RC_TYPE TMC_DoMeasure( TMC_MEASURE_PRG  eCommand,
                       TMC_INCLINE_PRG  eMode = TMC_AUTO_INC );


RC_TYPE TMC_GetCoordinate( SYSTIME         WaitTime,
                           TMC_COORDINATE  &Coordinate,
                           TMC_INCLINE_PRG eMode = TMC_AUTO_INC );


RC_TYPE TMC_SetStation( TMC_STATION Station );
RC_TYPE TMC_GetStation( TMC_STATION &Station );

RC_TYPE TMC_GetPrismCorr( double &dPrismCorr );

RC_TYPE TMC_GetHeight( TMC_HEIGHT &Height );
RC_TYPE TMC_SetHeight( TMC_HEIGHT Height );

RC_TYPE TMC_SetOrientation   ( double dHzOrientation = 0.0 );
RC_TYPE TMC_SetPrismCorr( double dPrismCorr );

 
RC_TYPE TMC_GetFace( TMC_FACE &eFace );

RC_TYPE TMC_SetEdmMode( EDM_MODE eMode );
RC_TYPE TMC_GetEdmMode( EDM_MODE &eMode );

                         
RC_TYPE TMC_SetAtmCorr( TMC_ATMOS_TEMPERATURE AtmTemperature );
RC_TYPE TMC_GetAtmCorr( TMC_ATMOS_TEMPERATURE &AtmTemperature );                         

RC_TYPE TMC_SetRefractiveCorr( TMC_REFRACTION    Refractive );
RC_TYPE TMC_GetRefractiveCorr( TMC_REFRACTION    &Refractive );

RC_TYPE TMC_SetRefractiveMethod( unsigned short unMethod ); 
RC_TYPE TMC_GetRefractiveMethod( unsigned short &unMethod );

                         
RC_TYPE TMC_IfDataAzeCorrError ( BOOLE &bAzeCorrectionError );
RC_TYPE TMC_IfDataIncCorrError ( BOOLE &bIncCorrectionError );


typedef enum 
{ 
    BAP_NO_MEAS,          // no measurements
    BAP_NO_DIST,          // distance measurement, only angles
    BAP_DEF_DIST,         // default distance measurement program
    BAP_TRK_DIST,         // TRK distance measurement program and angles
    BAP_RTRK_DIST,        // RTRK distance measurement program and angles
    BAP_CLEAR_DIST,       // clear distances 
    BAP_STOP_TRK          // stop tracking
} BAP_MEASURE_PRG; 


extern RC_TYPE BAP_GetLastDisplayedError( short &nError, short &nGSIError );


extern RC_TYPE BAP_MeasDistanceAngle( BAP_MEASURE_PRG &DistMode,
                                      double &dHz,
                                      double &dV,
                                      double &dDist ); 


const short MOT_AXES = 2;     //Anzahl der Theodoliten-Achsen


// Reglerkonfigurationen:
enum MOT_MODE
    {
    MOT_POSIT,           //als "Posit"-Regler konfiguriert
    MOT_OCONST,          //als "wconstant"-Regler konfiguriert
    MOT_MANUPOS,         //zur manuellen Positionierung konfiguriert
    MOT_LOCK,            //als "Lock-In"-Regler konfiguriert
    MOT_BREAK,           //als "Brems"-Regler konfiguriert
    MOT_SERVICE,         //mit simulierten Triebe "Service"
    MOT_NONE,            //kein Regler installiert
    MOT_TERM             //terminiert den Regler-Task
    };   

// LockIn-Reglerstatus:
enum MOT_LOCK_STATUS
    {
    MOT_LOCKED_OUT,
    MOT_LOCKED_IN,
    MOT_PREDICTION    
    };

//Abbremsvarianten:
enum MOT_STOPMODE
    {
    MOT_NORMAL,          //Abremsung mit aktueller Beschleunigung
    MOT_SHUTDOWN         //Abremsung durch Unterbindung der Motorströme
    };                


struct MOT_COM_PAIR            //Datenaustauschpaar für GeoCom [HZ, V]
    {
    double  adValue[MOT_AXES];
    };                


RC_TYPE MOT_StartController     (MOT_MODE  eControlMode);
RC_TYPE MOT_StopController      (MOT_STOPMODE eMode);      

RC_TYPE MOT_SetVelocity         (MOT_COM_PAIR    RefOmega);
RC_TYPE MOT_ReadLockStatus      (MOT_LOCK_STATUS &reStatus);


//Bewegungsparameter:
struct AUT_POSTOL
    {
    double  adPosTol[MOT_AXES];     //Positioniertoleranzen [rad]
    };                

struct AUT_TIMEOUT
    {
    double  adPosTimeout[MOT_AXES]; //max. Positionierzeiten [sec]
    };         

struct AUT_DETENT
    {
    double dPositiveDetent;   //Anschlag in positiver Richtung
    double dNegativeDetent;   //Anschlag in negativer Richtung
    BOOLE bActive;            //Anschlag aktiv / inaktiv
    };         


    
enum AUT_POSMODE
    {AUT_NORMAL, AUT_PRECISE};          //Positionierungsmodi   
    
enum AUT_ATRMODE
    {AUT_POSITION, AUT_TARGET};         //Positionierungsmodi der ATR
    
enum AUT_ADJMODE
    {AUT_NORM_MODE, AUT_POINT_MODE};    //FineAdjust-Modi
    


//Hilfskonstante sur Suchfuktion für GeoCom-Aufruf mit Default-Werten
const double AUT_SEARCH_DEF = 0.04;

                                 
RC_TYPE AUT_MakePositioning     (double dHz, double dV,     //Version 4
                                 AUT_POSMODE ePOSMode, 
                                 AUT_ATRMODE eATRMode,
                                 BOOLE       bDummy);

RC_TYPE AUT_LockIn              (void);

                                 
RC_TYPE AUT_ChangeFace          (AUT_POSMODE ePOSMode,      //Version 4
                                 AUT_ATRMODE eATRMode,
                                 BOOLE       bDummy);

RC_TYPE AUT_Search              (double  dHz_Area,          //Version 2
                                 double  dV_Area, 
                                 BOOLE   bDummy);                    

 
RC_TYPE AUT_FineAdjust          (double dSrchHz,            //Version 3
                                 double dSrchV,
                                 BOOLE  bDummy);                                                               


RC_TYPE AUT_SetTol              (AUT_POSTOL  TolPar);
RC_TYPE AUT_ReadTol             (AUT_POSTOL  &rTolPar);  
RC_TYPE AUT_GetLockStatus       (ON_OFF_TYPE &reOnOff);
RC_TYPE AUT_SetLockStatus       (ON_OFF_TYPE eOnOff);
RC_TYPE AUT_GetATRStatus        (ON_OFF_TYPE &reOnOff);
RC_TYPE AUT_SetATRStatus        (ON_OFF_TYPE eOnOff); 
RC_TYPE AUT_GetFineAdjustMode   (AUT_ADJMODE &eMode);
RC_TYPE AUT_SetFineAdjustMode   (AUT_ADJMODE eMode);    
RC_TYPE AUT_SetTimeout          (AUT_TIMEOUT TimeoutPar); 
RC_TYPE AUT_ReadTimeout         (AUT_TIMEOUT &rTimeoutPar);
RC_TYPE AUT_SetDetents          (AUT_DETENT DetentHz1,
                                 AUT_DETENT DetentHz2,
                                 AUT_DETENT DetentV1,
                                 AUT_DETENT DetentV2);
RC_TYPE AUT_GetDetents          (AUT_DETENT &DetentHz1,
                                 AUT_DETENT &DetentHz2,
                                 AUT_DETENT &DetentV1,
                                 AUT_DETENT &DetentV2);
RC_TYPE AUT_EnableDetents       (BOOLE bHz1,
                                 BOOLE bHz2,
                                 BOOLE bV1,
                                 BOOLE bV2);



 
//  Target is DOS if _DOS is defined

#if         defined(_DOS)
#define     COM_CLIENT
#define     COM_DOS
#define     COM_SW_VERSION_ENABLE
#endif

//  Target is WIN-32 if WIN32 or _WIN32 is defined

#if         defined(_WIN32) || defined(WIN32) && !defined(COM_DOS)
#define     COM_CLIENT
#define     COM_WIN
#define     COM_WIN32    
#define     COM_SW_VERSION_ENABLE
#endif

//  Target is WIN-16 if _WINDOWS is defined but COM_WIN32 is not defined

#if         defined(_WINDOWS) && !defined(COM_WIN32) && !defined(COM_DOS)
#define     COM_CLIENT
#define     COM_WIN
#define     COM_WIN16
#define     COM_SW_VERSION_ENABLE
#endif

//
//  Establish export declarators
//  Visual C++ 16/32 bit uses different type-names to declare exported 
//  functions for C++ and Visual Basic programs. These type-names
//  are ignored when compiling for the TPS or for DOS.
//

#ifndef     COM_WIN
#define     COM_C_EXP
#define     COM_V_EXP
#else
#ifdef      COM_WIN16
#define     COM_C_EXP _export
#define     COM_V_EXP pascal _export
#else
#define     COM_C_EXP 
#define     COM_V_EXP WINAPI
#endif
#endif

//
//  Include files...
//

#ifdef      COM_CLIENT
#include    <windows.h>                   // Windows definitions
#undef      TRUE                          // Use beta project definitions
#undef      FALSE                         // of TRUE and FALSE 
#endif


//  To preserve backward compatability, all old return codes
//  are still defined, although many are not used.
//

const RC_TYPE
  RC_COM_ERO                    = RC_COM +  0 , //  Initiate Extended Runtime Operation (ERO).
  RC_COM_CANT_ENCODE            = RC_COM +  1 , //  Cannot encode arguments in client.
  RC_COM_CANT_DECODE            = RC_COM +  2 , //  Cannot decode results in client.
  RC_COM_CANT_SEND              = RC_COM +  3 , //  Hardware error while sending.
  RC_COM_CANT_RECV              = RC_COM +  4 , //  Hardware error while receiving.
  RC_COM_TIMEDOUT               = RC_COM +  5 , //  Request timed out.
  RC_COM_WRONG_FORMAT           = RC_COM +  6 , //  Packet format error.
  RC_COM_VER_MISMATCH           = RC_COM +  7 , //  Version mismatch between client and server.
  RC_COM_CANT_DECODE_REQ        = RC_COM +  8 , //  Cannot decode arguments in server.
  RC_COM_PROC_UNAVAIL           = RC_COM +  9 , //  Unknown RPC, procedure ID invalid.
  RC_COM_CANT_ENCODE_REP        = RC_COM + 10 , //  Cannot encode results in server.
  RC_COM_SYSTEM_ERR             = RC_COM + 11 , //  Unspecified generic system error.
  RC_COM_UNKNOWN_HOST           = RC_COM + 12 , //  (Unused error code)
  RC_COM_FAILED                 = RC_COM + 13 , //  Unspecified error.
  RC_COM_NO_BINARY              = RC_COM + 14 , //  Binary protocol not available.     
  RC_COM_INTR                   = RC_COM + 15 , //  Call interrupted.
  RC_COM_UNKNOWN_ADDR           = RC_COM + 16 , //  (Unused error code)
  RC_COM_NO_BROADCAST           = RC_COM + 17 , //  (Unused error code)
  RC_COM_REQUIRES_8DBITS        = RC_COM + 18 , //  Protocol needs 8bit encoded chararacters.
  RC_COM_UD_ERROR               = RC_COM + 19 , //  (Unused error code)
  RC_COM_LOST_REQ               = RC_COM + 20 , //  (Unused error code)
  RC_COM_TR_ID_MISMATCH         = RC_COM + 21 , //  Transacation ID mismatch error.
  RC_COM_NOT_GEOCOM             = RC_COM + 22 , //  Protocol not recognizeable.
  RC_COM_UNKNOWN_PORT           = RC_COM + 23 , //  (WIN) Invalid port address.
  RC_COM_ILLEGAL_TRPT_SELECTOR  = RC_COM + 24 , //  (Unused error code)
  RC_COM_TRPT_SELECTOR_IN_USE   = RC_COM + 25 , //  (Unused error code)
  RC_COM_INACTIVE_TRPT_SELECTOR = RC_COM + 26 , //  (Unused error code)
  RC_COM_ERO_END                = RC_COM + 27 , //  ERO is terminating.
  RC_COM_OVERRUN                = RC_COM + 28 , //  Internal error: data buffer overflow.
  RC_COM_SRVR_RX_CHECKSUM_ERROR = RC_COM + 29 , //  Invalid checksum on server side received.
  RC_COM_CLNT_RX_CHECKSUM_ERROR = RC_COM + 30 , //  Invalid checksum on client side received.
  RC_COM_PORT_NOT_AVAILABLE     = RC_COM + 31 , //  (WIN) Port not available.
  RC_COM_PORT_NOT_OPEN          = RC_COM + 32 , //  (WIN) Port not opened.
  RC_COM_NO_PARTNER             = RC_COM + 33 , //  (WIN) Unable to find TPS.
  RC_COM_ERO_NOT_STARTED        = RC_COM + 34 , //  Extended Runtime Operation could not be started.
  RC_COM_CONS_REQ               = RC_COM + 35 , //  Att to send cons reqs
  RC_COM_SRVR_IS_SLEEPING       = RC_COM + 36 ,	//  TPS has gone to sleep. Wait and try again.
  RC_COM_SRVR_IS_OFF            = RC_COM + 37 ; //  TPS has shut down. Wait and try again.

// Buffer length for error text (COM_GetErrorText)
const short   COM_MAX_ERROR_TXT_LEN = 256;      // Max. length of error text (incl. \0)

//
//  Type definitions...
//    



typedef enum                              // Communications port selector
  {                                       // Use for PC-geocom only
    COM_1   ,                             // 
    COM_2   ,                             //
    COM_3   ,                             // COM3: Windows only
    COM_4                                 // COM4: Windows only
  } COM_PORT ;                            //



typedef enum                              // Geocom format selector
  {                                       //  
    COM_ASCII   ,                         //
    COM_BINARY                            //
  } COM_FORMAT ;                          //

typedef enum                              // Used for public customer interface
  {                                       //
    COM_BAUD_38400  ,                     //
    COM_BAUD_19200  ,                     //
    COM_BAUD_9600   ,                     //
    COM_BAUD_4800   ,                     //
    COM_BAUD_2400                         //
  } COM_BAUD_RATE ;                       //

typedef enum                              //  TPS startup selector
  {                                       //  

  
    COM_TPS_REMOTE = 1                    //             remote mode of TPS
  } COM_TPS_STARTUP_MODE ;                //

typedef enum                              //  TPS startup selector
  {                                       //  
    COM_TPS_SHUT_DOWN ,                   //   shut down of TPS
    COM_TPS_SLEEP                         //   put TPS into sleep mode
  } COM_TPS_STOP_MODE ;                   //

typedef enum                              //  Current state of server
  {                                       //
    COM_TPS_OFF,                          //  TPS is off
    COM_TPS_SLEEPING,                     //  TPS is in sleep mode
    COM_TPS_READY,                        //  TPS is on & active
    COM_TPS_UNKNOWN                       //  Unknown/Not initialized
  } COM_TPS_STATUS ;                      //


 
extern    RC_TYPE COM_C_EXP COM_GetComFormat ( COM_FORMAT & ) ;
extern    RC_TYPE COM_C_EXP COM_SetComFormat ( COM_FORMAT ) ;
extern    RC_TYPE COM_C_EXP COM_SetSendDelay ( short ) ;
extern    RC_TYPE COM_C_EXP COM_NullProc ( void ) ;
extern    RC_TYPE COM_C_EXP COM_SwitchOnTPS ( COM_TPS_STARTUP_MODE ) ;
extern    RC_TYPE COM_C_EXP COM_SwitchOffTPS ( COM_TPS_STOP_MODE ) ;
extern    RC_TYPE COM_C_EXP COM_GetTimeOut ( short & ) ;
extern    RC_TYPE COM_C_EXP COM_SetTimeOut ( short ) ;
extern    RC_TYPE COM_C_EXP COM_GetSWVersion ( short &, short &, short & ) ;
extern	  RC_TYPE COM_C_EXP COM_GetTPSState ( COM_TPS_STATUS & ) ;
extern	  RC_TYPE COM_C_EXP COM_EnableSignOff ( BOOLE ) ;
extern    RC_TYPE COM_C_EXP COM_GetBinaryAvailable( BOOLE & ) ;
extern    RC_TYPE COM_C_EXP COM_SetBinaryAvailable( BOOLE ) ;

 
#ifdef COM_SERVER /*----------------------------------------------------------*/

 
#endif /* COM_SERVER ---------------------------------------------------------*/


#ifdef COM_CLIENT /*----------------------------------------------------------*/                                                               

 
extern RC_TYPE   COM_C_EXP COM_Init ( void ) ;
extern RC_TYPE   COM_C_EXP COM_End ( void ) ;
extern RC_TYPE   COM_C_EXP COM_OpenConnection ( COM_PORT, COM_BAUD_RATE &, short ) ;
extern RC_TYPE   COM_C_EXP COM_CloseConnection ( void ) ;
extern RC_TYPE   COM_C_EXP COM_GetBaudRate ( COM_BAUD_RATE & ) ;
extern RC_TYPE   COM_C_EXP COM_SetBaudRate ( COM_BAUD_RATE ) ;
extern RC_TYPE   COM_C_EXP COM_GetDoublePrecision ( short & ) ;
extern RC_TYPE   COM_C_EXP COM_SetDoublePrecision ( short ) ;
extern RC_TYPE   COM_C_EXP COM_Local ( void ) ;
extern RC_TYPE   COM_C_EXP COM_ViewError ( RC_TYPE, char * ) ;
extern RC_TYPE   COM_C_EXP COM_GetWinSWVersion ( short &, short &, short & ) ;
extern RC_TYPE   COM_C_EXP COM_UseWindow ( HWND ) ;             
extern RC_TYPE   COM_C_EXP COM_SetConnDlgFlag ( BOOLE ) ;
extern RC_TYPE   COM_C_EXP COM_GetErrorText ( RC_TYPE, char * ) ;

 
#endif /* COM_CLIENT ---------------------------------------------------------*/

#ifdef COM_DOS /*-------------------------------------------------------------*/


 
#endif  /* COM_DOS -----------------------------------------------------------*/

 
#ifdef COM_WIN  /*------------------------------------------------------------*/                                                               

 
#endif /* COM_WIN  -----------------------------------------------------------*/

#ifdef COM_SERVER /*----------------------------------------------------------*/

 
#endif  /* COM_SERVER --------------------------------------------------------*/                    

 
// recording formats
typedef short   WIR_RECFORMAT;

 
// valid data formats
const WIR_RECFORMAT WIR_RECFORMAT_GSI   = 0;  // defines recording format is GSI (standard)
const WIR_RECFORMAT WIR_RECFORMAT_GSI16 = 1;  // defines recording format is the new GSI-16



#endif


/* _________________________________________END com_pub.hpp___ */
