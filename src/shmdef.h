/** 
 *	\file shmdef.h
 *	Defines the shared memory object
 *
 *	$Id: shmdef.h 1971 2008-08-07 04:28:08Z greerd $
 *
 */


#ifndef _SHMDEF_H
#define _SHMDEF_H

#include "novatel.h"

#define RANGEBUFSIZE 10  // 10 seconds of data
#define IMUBUFSIZE 1000  // 10 seconds of data

/* data structure for shared mem */
typedef struct _shm_struct
{
	int iStructInitialised;
	GPSEPHEM CurrentGPSEPHEM[NOVATEL_MAXPRNS];
	RANGE	CurrentRANGE;
	IONUTC	CurrentIONUTC;
	RAWEPHEM CurrentRAWEPHEM;
	VERSION CurrentVERSION;
	BESTXYZ CurrentBESTXYZ;	
	BESTPOS CurrentBESTPOS;

	INSPVA CurrentINSPVA;

	RANGE	RangeBuffer[RANGEBUFSIZE];
	int	iRange_irbp;  // input ring buffer pointer (read point)
	int	iRange_arbp;  // append ring buffer pointer (write point)	

	double	dLLH_LSQ[3];
	double  dECEF_LSQ[3];
	double 	dRxClock_LSQ;
	double	dDOPS[5];

} shm_struct;


#endif

