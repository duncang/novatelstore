/** 
 * 	\file novatel.h
 *	\author Duncan Greer
 *
 *	\brief Implements functions and data structures specific to Novatel OEM Family Receivers
 *
 *	This file contains functions and data structures specific to the Novatel OEM family of
 *	GPS receivers.  This includes CRC checks, message types, and message data structures.	
 * 
 * 	$Id: novatel.h 3510 2010-06-02 09:06:48Z greerd $
 */

#ifndef _NOVATEL_H
#define _NOVATEL_H


#include <stdint.h>

/* number of channels in the GPS receiver (max number of measurements)  */
#define NOVATEL_MAXCHANNELS	40
#define NOVATEL_MAXPRNS		40

/* message types  */
#define NOVATEL_GPSEPHEM 7
#define NOVATEL_BESTPOS	42
#define NOVATEL_RANGE	43
#define NOVATEL_IONUTC	8
#define NOVATEL_RAWEPHEM 41
#define NOVATEL_VERSION 37
#define NOVATEL_BESTXYZ 241

#define NOVATEL_INSPVA 507
#define NOVATEL_INSPVAS 508

/**
 * struct to store GPS time information
 */
typedef struct _GPSTime
{
	unsigned short usGPSWeek;  /* GPS Week */
	float fGPSSecondOfWeek; /* GPS Time of week in seconds */
} GPSTime;

/**
 * The GPSEPEHM data structure stores the ephemerides for 1 gps satellite
 */
typedef struct _GPSEPHEM
{
	GPSTime	gtTimeStamp;
	unsigned long ulPRN;
	double dTOW;
	unsigned long ulHealth;
	unsigned long ulIODE1;
	unsigned long ulIODE2;
	unsigned long ulGPSWeek;
	unsigned long ulZWeek;
	double dTOE;		/* Time of Ephemeris  */
	double dA;		/* Semi-major axis, metres */
	double dDeltaN;		/* Mean Motion Difference, radians/sec */
	double dM0;		/* mean anomoly, radians */
	double dEccentricity;	/* eccentricity */
	double dOmega;		/* argument of perigee */
	double dcuc;
	double dcus;
	double dcrc;
	double dcrs;
	double dcic;
	double dcis;
	double dInclination0;	/* inclination  */
	double dInclination_dot;	/* inclination rate */
	double dOmega0;		/* right ascention */	
	double dOmega_dot;
	unsigned long ulIODC;
	double dTOC;		/* SV clock correction, seconds */
	double dTGD;		/* estimated group delay */
	double dA_f0;		/* clock aging parameter, seconds */
	double dA_f1;		/* clock aging parameter, seconds/second */
	double dA_f2;		/* clock aging parameter, seconds/second/second */
	unsigned long ulAntiSpoofing;	/* anti-spoofing flag 0=FALSE, 1=TRUE */
	double dN;		/* corrected mean motion, radians/sec */
	double dURA;		/* User-Range Accuracy	 */
} GPSEPHEM;

/** 
 * Stores the data from a Novatel BESTPOS packet
 */
typedef struct _BESTPOS
{
	GPSTime	gtTimeStamp;	
	unsigned long ulSolutionStatus;
	unsigned long ulPositionType;
	double dLatitude;
	double dLongitude;
	double dHeight;
	float fUndulation;
	unsigned long ulDatumID;
	float fLatitudeSigma;
	float fLongitudeSigma;
	float fHeightSigma;
	char	azucBaseStationID[4];
	float fDifferentialAge;
	float fSolutionAge;
	unsigned char ucNumberObservationsTracked;
	unsigned char ucNumberL1ObservationsUsed;
	unsigned char ucNumberL1ObservationsAboveRTKMaskAngle;
	unsigned char ucNumberL2ObservationsAboveRTKMaskAngle;
} BESTPOS;


/**
 *	Stores the data from a Novatel RANGE packet
 */
typedef struct _RANGE
{
	GPSTime	gtTimeStamp;
	long lNumberObservations;
	unsigned short usPRN[NOVATEL_MAXCHANNELS];
	unsigned short usGlonassFrequency[NOVATEL_MAXCHANNELS];
	double dPseudorange[NOVATEL_MAXCHANNELS];
	float fPseudorangeSigma[NOVATEL_MAXCHANNELS];
	double dCarrierPhase[NOVATEL_MAXCHANNELS];
	float fCarrierPhaseSigma[NOVATEL_MAXCHANNELS];
	float fDoppler[NOVATEL_MAXCHANNELS];
	float fCNo[NOVATEL_MAXCHANNELS];
	float fLockTime[NOVATEL_MAXCHANNELS];
	unsigned long ulTrackingStatus[NOVATEL_MAXCHANNELS];
} RANGE;

/**
 * IONUTC Structure stores the ionospheric parameters transmitted 
 * by the GPS.
 */
typedef struct _IONUTC
{
	GPSTime gtTimeStamp;
	double a0;
	double a1;
	double a2;
	double a3;
	double b0;
	double b1;
	double b2;
	double b3;
	unsigned long ulUTCWeekNumber;
	unsigned long ulUTCReferenceTime;
	double A0;
	double A1;
	unsigned long ulUTCFutureWeekNumber;
	unsigned long ulUTCDayNumber;
	long lUTCLeapSeconds;
	long lUTCFutureLeapSeconds;
	unsigned long ulUTCDeltaT;	
} IONUTC;

typedef struct _RAWEPHEM
{
	GPSTime gtTimeStamp;
	unsigned long ulPRN;
	unsigned long ulEphemerisReferenceWeek;
	unsigned long ulEphemerisReferenceSecond;
	uint8_t Subframe1Data[30];
	uint8_t Subframe2Data[30];
	uint8_t Subframe3Data[30]; 
	
} RAWEPHEM;


typedef struct _VERSION
{
/* note that this data struct only stores info for one card - other cards are ignored */
	GPSTime gtTimeStamp;
	long ulNumberComponents;
	long ulType;
	char azcModel[16];
	char azcProductSerialNumber[16];
	char azcHardwareVersion[16];
	char azcFirmwareVersion[16];
	char azcBootCodeVersion[16];
	char azcFirmwareCompileDate[12];
	char azcFirmwareCompileTime[12];
} VERSION;


typedef struct _BESTXYZ
{
	GPSTime gtTimeStamp;
	long lPosSolutionStatus;
	long lPositionType;
	double dPosX;
	double dPosY;
	double dPosZ;
	float fPosXSigma;
	float fPosYSigma;
	float fPosZSigma;
	long lVelSolutionStatus;
	long lVelocityType;
	double dVelX;
	double dVelY;
	double dVelZ;
	float fVelXSigma;
	float fVelYSigma;
	float fVelZSigma;
	char azcBaseStationID[4];
	float fVelocityLatency;
	float fDifferentialAge;
	float fSolutionAge;
	unsigned char ucNumberObservationsTracked;
	unsigned char ucNumberL1ObservationsUsed;
	unsigned char ucNumberL1ObservationsAboveRTKMaskAngle;
	unsigned char ucNumberL2ObservationsAboveRTKMaskAngle;

} BESTXYZ;


typedef struct _INSPVA
{
	GPSTime gtTimeStamp;
	double dLatitude;
	double dLongitude;
	double dHeight;
	double dVelN;
	double dVelE;
	double dVelD;
	double dPhi;
	double dTheta;
	double dPsi;
	long INSStatus;
} INSPVA;

/**
 * \brief Novatel Function to generate 32-bit CRC
 *
 * \param i Random integer
 */
unsigned long CRC32Value(int i);

/**
 *	\brief Calculates the CRC-32 of a block of data all at once
 *	\param	ulCount	Number of bytes in the data block
 * 	\param ucBuffer	Data Block
 *
 */
unsigned long CalculateBlockCRC32(unsigned long ulCount, unsigned char *ucBuffer);




#endif


