/**
 *      \mainpage
 *	This program implements a shm store for the Novatel GPS
 *
 *
 *	\file novatelstore.c - Retrieves measurements from a Novatel GPS receiver and stores in shared memory.
 *
 * 	
 *
 *
 *	Revision Information: 
 *	$Id: novatelstore.c 3510 2010-06-02 09:06:48Z greerd $
 *
 *	
 */



#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <strings.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <time.h>
#include <sys/mman.h>
#include <limits.h>

#if defined(__APPLE__) || defined(__MACH__)
#include <sys/time.h>
#endif

#include "novatel.h"
#include "shmdef.h"

#define MODULE_NAME	"[Novatel Store]"

#define log(string)	fprintf(stdout,"%s %s\n", MODULE_NAME, string)
#define err(string)	fprintf(stderr,"%s ERROR: %s\n",MODULE_NAME, string)

#define MAX_LOG_LENGTH 4096

/** max number of GPS satellites that we can store ephemeris for*/
#define MAX_PRNS	40	

/* example bestpos packet (ascii)
#BESTPOSA,COM2,0,28.0,FINESTEERING,1386,97129.000,00000020,4ca6,1810;SOL_COMPUTED,SINGLE,-27.47735832601,153.02715301736,49.0198,40.7704WGS84,1.6805,1.3796,3.8551,"",0.000,0.000,10,10,0,0,0,0,0,0*aa287bfb
*/

/** Controls whether or not the program should keep running. */
int iShutdown = 0;



/**
 *	\fn HandleSignal
 *	\brief Handle caught signals
 *	
 * 	This function sets a flag (iShutdown) to shut the program down.
 */
void HandleSignal(int signal) 
{ 
	fprintf(stdout,"Got Signal: %d\n",signal);
	iShutdown = 1; 
}

/**
 *	\fn SendString
 * 	\brief Send string to serial port
 *
 * 	\param iPort The port handler; must be open and initialised
 *	\param string The string to send (null terminated)
 */
int SendString(int iPort,char *string);


/**
 * 	\fn Usage
 *	\brief Print Usage Information
 */
void Usage(char *arg)
{

	/* options */
	/*
	    b = baudrate
	    p = port
	    l = logging 
	    f = log filename
	    t = enable tcp server
	*/
	fprintf(stdout,"Usage: %s -p [port] -b [baud] -l -f [file] -t -x\n",arg);
	fprintf(stdout,"    -b baudrate (115200)\n");
	fprintf(stdout,"    -p port (/dev/ttyS0)\n");
	fprintf(stdout,"    -l enable logging\n");
	fprintf(stdout,"    -f log filename\n");
	fprintf(stdout,"    -t enable tcp server\n");	
	fprintf(stdout,"    -v print version and quit\n");
	fprintf(stdout,"    -x use hardware flow control\n");
	fprintf(stdout,"    -u Unlink Shared Memory on close\n");
	fprintf(stdout,"    -L Use specified log file path (/data/gps)\n");
	fprintf(stdout,"    -q Quiet Mode\n");
	fprintf(stdout,"    -s Make Timestamp File\n");

}


/**
 *	
 */
int main(int argc, char *argv[])
{
	char *azcPort = "/dev/ttyS0";
	int iBaudNumber = B115200;
	int iBaudRate = 115200;
	int iParity = 0; 
	int iParityOdd = 0;
	char *azcVersion = "$Id: novatelstore.c 3510 2010-06-02 09:06:48Z greerd $";
	int iFlowControl = 0;
	int iPort;
	int iBytesRead = 0;
	unsigned char azucBuffer[MAX_LOG_LENGTH];  /* buffer storing incoming RS232 data */
	char azcLogFilePath[255];
	char azcLogFileNameWithPath[1024];
	
	int iQuietMode = 0;	  /* routes stdout to /dev/null */

	int iUseTimestampFile = 0;  /* make a timestamp file */
	FILE *fpTimestamp = NULL;
	char azcTimestampFileName[256];
	char azcTimestampFileWithPath[1024];

	int iGotVersion = 0;
	int iGotBestPos = 0;

	int iLogging = 0;	   /* logging enabled?  1=yes, default is no */
	char azcLogFileName[256];  /* log file name */
	
	FILE *fpLogFile = NULL;    /* pointer to log file stream */
	int iBytesWritten = 0;  /* used to check the log file writing */

	int iCustomLogFile = 0;

	unsigned char ucHeaderLength = 0;
	int iTotalBytes = 0;
	unsigned short usMessageLength = 0;
	unsigned short usMessageID = 0;

	unsigned long ulCRC_Calc, ulCRC_Tx;

	struct termios tPortSettings;

	int iWait1 = 0;
	int iWait2 = 0;

	
	/* shared mem stuff */
	int iUseSHM = 1;
	int iSHM_FID;
	shm_struct *shm = NULL;

	char copt;

	/** structure to store positions */
	BESTPOS	BestPosData;
	RANGE	RangeData;
	GPSEPHEM	GPSEphemeris[MAX_PRNS];

	int iAbandonPacket = 0;

	long lMeasurementNumber = 0;

	GPSTime GPSTimeStamp;
	unsigned long ulGPSMilliSeconds;
	unsigned long ulPRN;

	/* this variable controls if shared memory is unlinked on close */
	int iUnlinkSHM = 0;

	struct tm *timestruct;
	time_t tCurrentTime = 0;
	time_t tLogFileStart = 0;

	struct timespec tsCurrentTime;
	struct timeval tvCurrentTime;
	

	/* setup signal handlers */
	signal(SIGHUP,  HandleSignal);
	signal(SIGKILL, HandleSignal);
	signal(SIGTERM, HandleSignal);
	signal(SIGALRM, HandleSignal);
	signal(SIGINT, HandleSignal);

	/* get the current time to use as a timestamp in the log file name */
	tCurrentTime = time(NULL);
	timestruct = gmtime(&tCurrentTime);

	/* setting default log file path */
	sprintf(azcLogFilePath,"%s","/data/gps");


	/* read input parameters */
	while((copt = getopt(argc,argv,"p:b:lf:tvxqL:us")) && copt != -1)
	{
	  switch(copt)
	  {
	    case 'p':
		fprintf(stdout,"Option p: %s\n",optarg);	
		azcPort = optarg;
		break;

	    case 'b':
		iBaudRate = atoi(optarg);
	  	fprintf(stdout,"Option b: %d\n",iBaudRate);
		break;
	
	    case 'v':
		fprintf(stdout,"%s Version: %s\n",argv[0],azcVersion);
		return 0;
		break;

	   case 'x':
		log("Using Flow control");
		iFlowControl = 1;
		break;

	   case 'l':
		log("Logging enabled");


		strftime(azcLogFileName,sizeof(azcLogFileName),"novatel_%g%m%d%H%M%S.gps",timestruct);
		iLogging = 1;
		break;

	  case 'f':
		/* set the log file name */
		sprintf(azcLogFileName,"%s",optarg);
		iCustomLogFile = 1; /* since we're using a user specified log file name, we dont want to recycle it every hour */
		break;

	  case 'q':
		/* quiet mode */
		iQuietMode = 1;

		/* TODO: redirect stdout to /dev/null */
		

		break;

	  case 'L':
	  	/* setting log file path */
	  	sprintf(azcLogFilePath,"%s",optarg);

	  	fprintf(stdout,"%s Setting log file path to %s",MODULE_NAME,azcLogFilePath);
	  	break;

	  case 'u':
	  	/* unlink shm on close */
		iUnlinkSHM = 1;
		log("Will Unlink SHM on close");
		break;

	  case 's':
		/* make timestamp file */
		iUseTimestampFile = 1;

		break;

	  case '?':
		Usage(argv[0]);
		return 0;
		break;
	  }
	}



	fprintf(stdout,"%s Using Port: %s at %d \n",MODULE_NAME, azcPort,iBaudRate);
	
	
	/* convert baud rate to baudnum */
	if((iBaudRate == 4800) || (iBaudRate == 9600) || (iBaudRate == 19200) ||
	   (iBaudRate == 38400) || (iBaudRate == 57600) || (iBaudRate == 115200) ||
	   (iBaudRate == 230400))
	{	
		if(iBaudRate == 4800) iBaudNumber = B4800;
		if(iBaudRate == 9600) iBaudNumber = B9600;
		if(iBaudRate == 19200) iBaudNumber = B19200;
		if(iBaudRate == 38400) iBaudNumber = B38400;
		if(iBaudRate == 57600) iBaudNumber = B57600;
		if(iBaudRate == 115200) iBaudNumber = B115200;
		if(iBaudRate == 230400) iBaudNumber = B230400;

	} else
	{
	  	err("Baudrate is not supported!\n");
		return 0;
	}



	/* open port */
	iPort = open(azcPort, O_RDWR);

	if(iPort < 0)
	{
		fprintf(stderr,"Cannot open port, %s:",azcPort);
		perror("open()");
		exit(-1);
	}

	/* initialise port settings */
	bzero(&tPortSettings,sizeof(tPortSettings));

	/* configure port settings */
	tPortSettings.c_iflag &= ~(IGNPAR|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
	
	tPortSettings.c_cflag &= ~(CSIZE|PARENB);
	tPortSettings.c_cflag |= (iBaudNumber|CS8|CLOCAL|CREAD);
	if(iFlowControl) tPortSettings.c_cflag |= (CRTSCTS);  // enable hardware flow control 
	if(iParity) tPortSettings.c_cflag |= PARENB;
	if(iParity && iParityOdd) tPortSettings.c_cflag |= PARODD;

 
	tPortSettings.c_oflag &= ~(OPOST);
	tPortSettings.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
	tPortSettings.c_cc[VTIME] = 10; /* wait for 1 second before returning */
	tPortSettings.c_cc[VMIN] = 0;   /* can return with no data */
	tcflush(iPort,TCIFLUSH);
	tcsetattr(iPort,TCSANOW,&tPortSettings);

	/* open log file */
	if(iLogging == 1)
	{
		sprintf(azcLogFileNameWithPath,"%s/%s",azcLogFilePath,azcLogFileName);
	
		if((fpLogFile = fopen(azcLogFileNameWithPath,"ab")) == NULL)
		{	
			fprintf(stderr,"Error opening %s for writing: %s\n",azcLogFileNameWithPath,strerror(errno));
			close(iPort);
			return -1;
		} else
		{
			tLogFileStart = tCurrentTime;
		}
	}

	if(iLogging) fprintf(stdout,"Using log file: %s\n",azcLogFileNameWithPath);
	

	/* open timestamp file */
	if(iUseTimestampFile == 1)
	{
		strftime(azcTimestampFileName,sizeof(azcTimestampFileName),"novatel_%g%m%d%H%M%S.time",timestruct);
		sprintf(azcTimestampFileWithPath,"%s/%s",azcLogFilePath,azcTimestampFileName);

		if((fpTimestamp = fopen(azcTimestampFileWithPath,"a")) == NULL)
		{
			fprintf(stderr,"Error opening %s for writing: %s\n",azcTimestampFileWithPath,strerror(errno));
			//fclose(fpLogFile);
			//close(iPort);
			//return -1;
			/* disable timestamp file */
			iUseTimestampFile = 0;
		} else
		{

		}
	}

	log("Initialised GPS Store - Hit CTRL-C to exit");



	/* setup shared memory */
	if(iUseSHM == 1)
	{
	  iSHM_FID = shm_open("/novatel0",O_RDWR|O_CREAT,0777);
	  if(iSHM_FID == -1)
	  {
		perror("shm_open: ");
		iUseSHM = 0;
 	  } else
	  {
	    /* set shm size */
	    if( ftruncate( iSHM_FID, sizeof(*shm) ) == -1 ) 
	    {
	    	perror("shm ftruncate: ");
	    	iUseSHM = 0;
	    } else
	    {
	     	/* memory map the shm object */
	     	shm = mmap(0,sizeof(*shm),PROT_READ|PROT_WRITE,MAP_SHARED,iSHM_FID,0);
	     	if(shm == MAP_FAILED)
	     	{
		   perror("shm mmap: ");
		   iUseSHM = 0;
	     	} else
		{
		  fprintf(stdout,"%s SHM Address Is: 0x%08x\n",MODULE_NAME,(unsigned int)shm);
		}
	    }
	  }
	}
	
	// clear ring buffer
	shm->iRange_arbp = 0;
	shm->iRange_irbp = 0;


	/* configure gps */
//	SendString(iPort,"log usb3 rawimusb onnew");	
//	usleep(10000);
	SendString(iPort,"log rangeb ontime 1");
	usleep(10000);
	SendString(iPort,"log gpsephemb onchanged");
	usleep(10000);
	SendString(iPort,"log rawephemb onchanged");
	usleep(10000);
	SendString(iPort,"log ionutcb onchanged");
	usleep(10000);
	SendString(iPort,"log bestxyzb ontime 1");
	usleep(10000);
//	SendString(iPort,"log inspvab ontime 0.02");	
//	usleep(10000);
	SendString(iPort,"log versionb once");
	usleep(10000);
	SendString(iPort,"log bestposb once");
	usleep(10000);
	

	while(iShutdown == 0)
	{
		
		/* clear buffer */
		memset(azucBuffer,0,MAX_LOG_LENGTH);
		iAbandonPacket = 0;

		/* synchronoise packet - look for binary header - 0xAA 0x44 0x12 */
		do
		{
			azucBuffer[0] = azucBuffer[1];
			azucBuffer[1] = azucBuffer[2];
			iBytesRead = read(iPort,&azucBuffer[2],1);
			
			if(iShutdown == 1)
			{	
				iAbandonPacket=1;
				break;
			}




		/* if log file has been opend for > 1 hour, start a new one */
		if(iLogging == 1 && iCustomLogFile == 0)
		{
			tCurrentTime = time(NULL);
			if(difftime(tCurrentTime,tLogFileStart) > 3600.0)
			{
				/* temporarily disable logging */
				iLogging = 0;

				/* start a new log file */
				fclose(fpLogFile);
						
				timestruct = gmtime(&tCurrentTime);
				strftime(azcLogFileName,sizeof(azcLogFileName),"novatel_%g%m%d%H%M%S.gps",timestruct);
			
				sprintf(azcLogFileNameWithPath,"%s/%s",azcLogFilePath,azcLogFileName);
	
				if((fpLogFile = fopen(azcLogFileNameWithPath,"ab")) == NULL)
				{	
					fprintf(stderr,"Error opening %s for writing: %s\n",azcLogFileNameWithPath,strerror(errno));
					iLogging = 0;
				} else
				{
					tLogFileStart = tCurrentTime;
					fprintf(stdout,"%s Started New Logfile: %s\n",MODULE_NAME,azcLogFileNameWithPath);
					/* re-enable logging */
					iLogging = 1;
					
					iGotBestPos = 0;
					iGotVersion = 0;
				}
			}

		}

			

				
		} while(azucBuffer[0] != 0xAA || azucBuffer[1] != 0x44 || azucBuffer[2] != 0x12);
		
		if(iAbandonPacket == 1)
		{
			continue;
		}

		/* read header length */
		iBytesRead = read(iPort,&azucBuffer[3],1);
		if(iBytesRead == 1)
		{
			ucHeaderLength = azucBuffer[3];
			
		} else
		{
			log("Could not read header length");
			break;
		}

		/* sanity check on header length */
		if(ucHeaderLength > 28)
		{
			/* too large - try again */
			fprintf(stderr,"%s WARNING: Header Unexpectedly large (%d bytes)\n",MODULE_NAME,ucHeaderLength);
//			iAbandonPacket = 1;
//			continue;
		}

		/* we have already received the first 4 bytes */
		iTotalBytes = 4;
	
		/* read rest of header */
		while(iTotalBytes<ucHeaderLength)
		{	
			iBytesRead = read(iPort,&azucBuffer[iTotalBytes],MAX_LOG_LENGTH-2);
			iTotalBytes += iBytesRead;
			if(iTotalBytes > MAX_LOG_LENGTH)
			{
				err("Too many bytes received");
				iAbandonPacket = 1;
				break;
			}
		}

		if(iAbandonPacket == 1)
		{
			continue;
		}

		/*  read message length */
		memcpy(&usMessageLength,&azucBuffer[8],2); 

		/* receive rest of message */
		while(iTotalBytes<(ucHeaderLength + usMessageLength + 4)) /* 4 is the length of the CRC */
		{
			iBytesRead = read(iPort,&azucBuffer[iTotalBytes],MAX_LOG_LENGTH-2);
			iTotalBytes += iBytesRead;
			if(iTotalBytes > MAX_LOG_LENGTH)
			{
				err("Too many bytes received");
				iAbandonPacket = 1;
				break;
			}
		}

		if(iAbandonPacket == 1)
		{
			continue;
		}

		/* get the message ID */
		memcpy(&usMessageID,&azucBuffer[4],2);

		/* debug message */
		if (iQuietMode == 0)
		{
			fprintf(stdout,"Message type: %d received, %d bytes total\n",usMessageID, iTotalBytes);
		}
		/* perform checksum */
		ulCRC_Calc = CalculateBlockCRC32(ucHeaderLength+usMessageLength,azucBuffer);
		
		/* retrieve checksum from message */
		memcpy(&ulCRC_Tx,&azucBuffer[ucHeaderLength+usMessageLength],4);

		if(ulCRC_Tx != ulCRC_Calc)
		{
			/* we have an error */
			log("CRC Check Error");
		} else
		{	
			/* get the time stamp */
			memcpy(&GPSTimeStamp.usGPSWeek,&azucBuffer[14],2);
			memcpy(&ulGPSMilliSeconds,&azucBuffer[16],4);
			GPSTimeStamp.fGPSSecondOfWeek = ((float)ulGPSMilliSeconds) / 1000.0;

			//tCurrentTime = time(NULL);
			//timestruct = gmtime(&tCurrentTime);

#if defined(__APPLE__) || defined (__MACH__)
			gettimeofday(&tvCurrentTime,NULL);
			tsCurrentTime.tv_sec = tvCurrentTime.tv_sec;
			tsCurrentTime.tv_nsec = tvCurrentTime.tv_usec*1e3;
#else
			clock_gettime(CLOCK_REALTIME,&tsCurrentTime);
#endif			

			/* write binary message data to file */
			if(iLogging) 
			{
				iBytesWritten = fwrite(azucBuffer,
						       sizeof(unsigned char),
						       ucHeaderLength+usMessageLength+4,
						       fpLogFile);
				if(iBytesWritten < (ucHeaderLength+usMessageLength+4))
				{
					fprintf(stderr,"Error writing log: %s\n",
						strerror(errno));
				}
				
				// flush the stream to disk
				fflush(fpLogFile);
				
				if(iUseTimestampFile == 1)
				{
					
					iBytesWritten = fprintf(fpTimestamp,"%ld,%ld,%d,%f\n",
								(long)tsCurrentTime.tv_sec,
								tsCurrentTime.tv_nsec,
								GPSTimeStamp.usGPSWeek,
								GPSTimeStamp.fGPSSecondOfWeek);
								
					fflush(fpTimestamp);
				}

			}


			/* extract the message data */
			switch(usMessageID)
			{
				case NOVATEL_BESTPOS:
					iGotBestPos = 1;
					
					/* copy timestamp */
					memcpy(&BestPosData.gtTimeStamp,&GPSTimeStamp,sizeof(GPSTime));	
					
					/*extract data */

					memcpy(&BestPosData.dLatitude,&azucBuffer[ucHeaderLength+8],8);
					memcpy(&BestPosData.dLongitude,&azucBuffer[ucHeaderLength+16],8);
					memcpy(&BestPosData.dHeight,&azucBuffer[ucHeaderLength+24],8);
					memcpy(&BestPosData.fUndulation,&azucBuffer[ucHeaderLength+32],4);
					memcpy(&BestPosData.ulDatumID,&azucBuffer[ucHeaderLength+36],4);
					memcpy(&BestPosData.fLatitudeSigma,&azucBuffer[ucHeaderLength+40],4);
					memcpy(&BestPosData.fLongitudeSigma,&azucBuffer[ucHeaderLength+44],4);
					memcpy(&BestPosData.fHeightSigma,&azucBuffer[ucHeaderLength+48],4);
					memcpy(&BestPosData.azucBaseStationID[0],&azucBuffer[ucHeaderLength+52],4);
					memcpy(&BestPosData.fDifferentialAge,&azucBuffer[ucHeaderLength+56],4);
					memcpy(&BestPosData.fSolutionAge,&azucBuffer[ucHeaderLength+60],4);
					memcpy(&BestPosData.ucNumberObservationsTracked,&azucBuffer[ucHeaderLength+64],1);
					memcpy(&BestPosData.ucNumberL1ObservationsUsed,&azucBuffer[ucHeaderLength+65],1);
					memcpy(&BestPosData.ucNumberL1ObservationsAboveRTKMaskAngle,&azucBuffer[ucHeaderLength+66],1);
					memcpy(&BestPosData.ucNumberL2ObservationsAboveRTKMaskAngle,&azucBuffer[ucHeaderLength+67],1);
					
					if(iQuietMode == 0)
					{
						fprintf(stdout,"BESTPOS: %lf %lf %lf\n",BestPosData.dLatitude,
										BestPosData.dLongitude,
										BestPosData.dHeight);
					}
					
					if(iUseSHM==1)
					{
						memcpy(&shm->CurrentBESTPOS,&BestPosData,sizeof(BESTPOS));
					}
					break;

				case NOVATEL_RANGE: /* pseudorange measurements */
					/* copy timestamp */
					memcpy(&RangeData.gtTimeStamp,&GPSTimeStamp,sizeof(GPSTime));	
					
					/* get number of measurements */
					memcpy(&RangeData.lNumberObservations,&azucBuffer[ucHeaderLength],4);
					if(iQuietMode == 0) 
					{ 
						fprintf(stdout,"RANGE: %ld Measurements\n",RangeData.lNumberObservations);
					}
					
					/* limit max measurements */
					if(RangeData.lNumberObservations > NOVATEL_MAXCHANNELS)
					{
						RangeData.lNumberObservations = NOVATEL_MAXCHANNELS;
					}

					for(lMeasurementNumber=0;lMeasurementNumber<RangeData.lNumberObservations;lMeasurementNumber++)
					{
						memcpy(&RangeData.usPRN[lMeasurementNumber],
							&azucBuffer[ucHeaderLength + 4 + lMeasurementNumber*44],2);
						memcpy(&RangeData.usGlonassFrequency[lMeasurementNumber],
							&azucBuffer[ucHeaderLength + 6 + lMeasurementNumber*44],2);
						memcpy(&RangeData.dPseudorange[lMeasurementNumber],
							&azucBuffer[ucHeaderLength + 8 + lMeasurementNumber*44],8);
						memcpy(&RangeData.fPseudorangeSigma[lMeasurementNumber],
							&azucBuffer[ucHeaderLength + 16 + lMeasurementNumber*44],4);
						memcpy(&RangeData.dCarrierPhase[lMeasurementNumber],
							&azucBuffer[ucHeaderLength + 20 + lMeasurementNumber*44],8);
						memcpy(&RangeData.fCarrierPhaseSigma[lMeasurementNumber],
							&azucBuffer[ucHeaderLength + 28 + lMeasurementNumber*44],4);
						memcpy(&RangeData.fDoppler[lMeasurementNumber],
							&azucBuffer[ucHeaderLength + 32 + lMeasurementNumber*44],4);
						memcpy(&RangeData.fCNo[lMeasurementNumber],
							&azucBuffer[ucHeaderLength + 36 + lMeasurementNumber*44],4);
						memcpy(&RangeData.fLockTime[lMeasurementNumber],
							&azucBuffer[ucHeaderLength + 40 + lMeasurementNumber*44],4);
						memcpy(&RangeData.ulTrackingStatus[lMeasurementNumber],
							&azucBuffer[ucHeaderLength + 44 + lMeasurementNumber*44],4);
						

					/*	fprintf(stdout,"%d %f: RANGE: PRN:%d %lf\n",
							RangeData.gtTimeStamp.usGPSWeek,
							RangeData.gtTimeStamp.fGPSSecondOfWeek,
							RangeData.usPRN[lMeasurementNumber],
							RangeData.dPseudorange[lMeasurementNumber]); 
					*/
					
		
					}
					
					if(iUseSHM == 1)
					{
					  /* copy to shared memory */
					  memcpy(&shm->CurrentRANGE,&RangeData,sizeof(RANGE));
					  
					  /* copy to shared memory ring buffer */
					  memcpy(&shm->RangeBuffer[shm->iRange_arbp],&RangeData,sizeof(RANGE));
					  shm->iRange_arbp++;
					//  fprintf(stdout,"arbp=%d ",shm->iRange_arbp);
					  if(shm->iRange_arbp >= RANGEBUFSIZE)
					  {
					  	shm->iRange_arbp = 0;
					  	//log("Range Ring Buffer Overvlow");
					  }
					}
					break;

				case NOVATEL_GPSEPHEM:  /* satellite ephemeris */
					
					/* get the PRN */
					memcpy(&ulPRN,&azucBuffer[ucHeaderLength],4);

					/* copy timestamp */
					memcpy(&GPSEphemeris[ulPRN].gtTimeStamp,&GPSTimeStamp,sizeof(GPSTime));	
					
					/* copy ephemeris parameters */
					memcpy(&GPSEphemeris[ulPRN].ulPRN,&azucBuffer[ucHeaderLength],4);	
					memcpy(&GPSEphemeris[ulPRN].dTOW,&azucBuffer[ucHeaderLength+4],8);	
					memcpy(&GPSEphemeris[ulPRN].ulHealth,&azucBuffer[ucHeaderLength+12],4);	
					memcpy(&GPSEphemeris[ulPRN].ulIODE1,&azucBuffer[ucHeaderLength+16],4);	
					memcpy(&GPSEphemeris[ulPRN].ulIODE2,&azucBuffer[ucHeaderLength+20],4);	
					memcpy(&GPSEphemeris[ulPRN].ulGPSWeek,&azucBuffer[ucHeaderLength+24],4);	
					memcpy(&GPSEphemeris[ulPRN].ulZWeek,&azucBuffer[ucHeaderLength+28],4);	
					memcpy(&GPSEphemeris[ulPRN].dTOE,&azucBuffer[ucHeaderLength+32],8);	
					memcpy(&GPSEphemeris[ulPRN].dA,&azucBuffer[ucHeaderLength+40],8);	
					memcpy(&GPSEphemeris[ulPRN].dDeltaN,&azucBuffer[ucHeaderLength+48],8);	
					memcpy(&GPSEphemeris[ulPRN].dM0,&azucBuffer[ucHeaderLength+56],8);	
					memcpy(&GPSEphemeris[ulPRN].dEccentricity,&azucBuffer[ucHeaderLength+64],8);	
					memcpy(&GPSEphemeris[ulPRN].dOmega,&azucBuffer[ucHeaderLength+72],8);	
					memcpy(&GPSEphemeris[ulPRN].dcuc,&azucBuffer[ucHeaderLength+80],8);	
					memcpy(&GPSEphemeris[ulPRN].dcus,&azucBuffer[ucHeaderLength+88],8);	
					memcpy(&GPSEphemeris[ulPRN].dcrc,&azucBuffer[ucHeaderLength+96],8);	
					memcpy(&GPSEphemeris[ulPRN].dcrs,&azucBuffer[ucHeaderLength+104],8);	
					memcpy(&GPSEphemeris[ulPRN].dcic,&azucBuffer[ucHeaderLength+112],8);	
					memcpy(&GPSEphemeris[ulPRN].dcis,&azucBuffer[ucHeaderLength+120],8);	
					memcpy(&GPSEphemeris[ulPRN].dInclination0,&azucBuffer[ucHeaderLength+128],8);	
					memcpy(&GPSEphemeris[ulPRN].dInclination_dot,&azucBuffer[ucHeaderLength+136],8);	
					memcpy(&GPSEphemeris[ulPRN].dOmega0,&azucBuffer[ucHeaderLength+144],8);	
					memcpy(&GPSEphemeris[ulPRN].dOmega_dot,&azucBuffer[ucHeaderLength+152],8);	
					memcpy(&GPSEphemeris[ulPRN].ulIODC,&azucBuffer[ucHeaderLength+160],4);	
					memcpy(&GPSEphemeris[ulPRN].dTOC,&azucBuffer[ucHeaderLength+164],8);	
					memcpy(&GPSEphemeris[ulPRN].dTGD,&azucBuffer[ucHeaderLength+172],8);	
					memcpy(&GPSEphemeris[ulPRN].dA_f0,&azucBuffer[ucHeaderLength+180],8);	
					memcpy(&GPSEphemeris[ulPRN].dA_f1,&azucBuffer[ucHeaderLength+188],8);	
					memcpy(&GPSEphemeris[ulPRN].dA_f2,&azucBuffer[ucHeaderLength+196],8);	
					memcpy(&GPSEphemeris[ulPRN].ulAntiSpoofing,&azucBuffer[ucHeaderLength+204],4);	
					memcpy(&GPSEphemeris[ulPRN].dN,&azucBuffer[ucHeaderLength+208],8);	
					memcpy(&GPSEphemeris[ulPRN].dURA,&azucBuffer[ucHeaderLength+216],8);	
					
					fprintf(stdout,"PRN%ld Ephemeris Stored\n",GPSEphemeris[ulPRN].ulPRN);
					
					if(iUseSHM == 1)
					{
					  /* copy to shared memory */
					  memcpy(&shm->CurrentGPSEPHEM[ulPRN],&GPSEphemeris[ulPRN],sizeof(GPSEPHEM));
					}
					break;

				case NOVATEL_IONUTC:
					
					if(iUseSHM == 0)
					{
					  fprintf(stderr,"%s WARNING: Could not store IONUTC in SHM\n",MODULE_NAME);
					} else
					{
					  /* copy timestamp */
					  memcpy(&shm->CurrentIONUTC.gtTimeStamp,&GPSTimeStamp,sizeof(GPSTime));
					  
					  memcpy(&shm->CurrentIONUTC.a0,&azucBuffer[ucHeaderLength+0],8);
					  memcpy(&shm->CurrentIONUTC.a1,&azucBuffer[ucHeaderLength+8],8);
					  memcpy(&shm->CurrentIONUTC.a2,&azucBuffer[ucHeaderLength+16],8);
					  memcpy(&shm->CurrentIONUTC.a3,&azucBuffer[ucHeaderLength+24],8);
					  memcpy(&shm->CurrentIONUTC.b0,&azucBuffer[ucHeaderLength+32],8);
					  memcpy(&shm->CurrentIONUTC.b1,&azucBuffer[ucHeaderLength+40],8);
					  memcpy(&shm->CurrentIONUTC.b2,&azucBuffer[ucHeaderLength+48],8);
					  memcpy(&shm->CurrentIONUTC.b3,&azucBuffer[ucHeaderLength+56],8);
					  memcpy(&shm->CurrentIONUTC.ulUTCWeekNumber,&azucBuffer[ucHeaderLength+64],4);
					  memcpy(&shm->CurrentIONUTC.ulUTCReferenceTime,&azucBuffer[ucHeaderLength+68],4);
					  memcpy(&shm->CurrentIONUTC.A0,&azucBuffer[ucHeaderLength+72],8);
					  memcpy(&shm->CurrentIONUTC.A1,&azucBuffer[ucHeaderLength+80],8);
					  memcpy(&shm->CurrentIONUTC.ulUTCFutureWeekNumber,&azucBuffer[ucHeaderLength+88],4);
					  memcpy(&shm->CurrentIONUTC.ulUTCDayNumber,&azucBuffer[ucHeaderLength+92],4);
					  memcpy(&shm->CurrentIONUTC.lUTCLeapSeconds,&azucBuffer[ucHeaderLength+96],4);
					  memcpy(&shm->CurrentIONUTC.lUTCFutureLeapSeconds,&azucBuffer[ucHeaderLength+100],4);
					  memcpy(&shm->CurrentIONUTC.ulUTCDeltaT,&azucBuffer[ucHeaderLength+104],4);
					  						
					
					  log("Stored new IONUTC");					  	
															
					}
					  					break;

				case NOVATEL_RAWEPHEM:
					if(iUseSHM == 0)
					{
					  fprintf(stderr,"%s WARNING: Could not store RAWEPHEM in SHM\n",MODULE_NAME);
					} else
					{
					  /* copy timestamp */
					  memcpy(&shm->CurrentRAWEPHEM.gtTimeStamp,&GPSTimeStamp,sizeof(GPSTime));
					  
					  if(iQuietMode == 0) 
					  { 
					    log("Stored new RAWEPEHM");
					  }
					}	
					break;
				
				case NOVATEL_VERSION:
					iGotVersion = 1;
					if(iQuietMode == 0)
					{
						log("Got version but not decoding");
					}
					break;
					
				case NOVATEL_BESTXYZ:
					if(iQuietMode == 0)
					{
						log("Got BESTXYZ");
					}
					
					if(iUseSHM == 0)
					{
					  fprintf(stderr,"%s WARNING: Could not store BESTXYZ in SHM\n",MODULE_NAME);
					} else
					{
						/* copy timestamp */
						memcpy(&shm->CurrentBESTXYZ.gtTimeStamp,&GPSTimeStamp,sizeof(GPSTime));
					
						/* copy data */
						memcpy(&shm->CurrentBESTXYZ.lPosSolutionStatus,&azucBuffer[ucHeaderLength],4);
						memcpy(&shm->CurrentBESTXYZ.lPositionType,&azucBuffer[ucHeaderLength+4],4);
						memcpy(&shm->CurrentBESTXYZ.dPosX,&azucBuffer[ucHeaderLength+8],8);
						memcpy(&shm->CurrentBESTXYZ.dPosY,&azucBuffer[ucHeaderLength+16],8);
						memcpy(&shm->CurrentBESTXYZ.dPosZ,&azucBuffer[ucHeaderLength+24],8);
						memcpy(&shm->CurrentBESTXYZ.fPosXSigma,&azucBuffer[ucHeaderLength+32],4);
						memcpy(&shm->CurrentBESTXYZ.fPosYSigma,&azucBuffer[ucHeaderLength+36],4);
						memcpy(&shm->CurrentBESTXYZ.fPosZSigma,&azucBuffer[ucHeaderLength+40],4);
						memcpy(&shm->CurrentBESTXYZ.lVelSolutionStatus,&azucBuffer[ucHeaderLength+44],4);
						memcpy(&shm->CurrentBESTXYZ.lVelocityType,&azucBuffer[ucHeaderLength+48],4);
						memcpy(&shm->CurrentBESTXYZ.dVelX,&azucBuffer[ucHeaderLength+52],8);
						memcpy(&shm->CurrentBESTXYZ.dVelY,&azucBuffer[ucHeaderLength+60],8);
						memcpy(&shm->CurrentBESTXYZ.dVelZ,&azucBuffer[ucHeaderLength+68],8);
						memcpy(&shm->CurrentBESTXYZ.fVelXSigma,&azucBuffer[ucHeaderLength+76],4);
						memcpy(&shm->CurrentBESTXYZ.fVelYSigma,&azucBuffer[ucHeaderLength+80],4);
						memcpy(&shm->CurrentBESTXYZ.fVelZSigma,&azucBuffer[ucHeaderLength+84],4);
						shm->CurrentBESTXYZ.azcBaseStationID[0] = azucBuffer[ucHeaderLength+88];
						shm->CurrentBESTXYZ.azcBaseStationID[1] = azucBuffer[ucHeaderLength+89];
						shm->CurrentBESTXYZ.azcBaseStationID[2] = azucBuffer[ucHeaderLength+90];
						shm->CurrentBESTXYZ.azcBaseStationID[3] = azucBuffer[ucHeaderLength+91];
						memcpy(&shm->CurrentBESTXYZ.fVelocityLatency,&azucBuffer[ucHeaderLength+92],4);
						memcpy(&shm->CurrentBESTXYZ.fDifferentialAge,&azucBuffer[ucHeaderLength+96],4);
						memcpy(&shm->CurrentBESTXYZ.fSolutionAge,&azucBuffer[ucHeaderLength+100],4);
						memcpy(&shm->CurrentBESTXYZ.ucNumberObservationsTracked,&azucBuffer[ucHeaderLength+104],1);
						memcpy(&shm->CurrentBESTXYZ.ucNumberL1ObservationsUsed,&azucBuffer[ucHeaderLength+105],1);
						memcpy(&shm->CurrentBESTXYZ.ucNumberL1ObservationsAboveRTKMaskAngle,&azucBuffer[ucHeaderLength+106],1);
						memcpy(&shm->CurrentBESTXYZ.ucNumberL2ObservationsAboveRTKMaskAngle,&azucBuffer[ucHeaderLength+107],1);
				
					}
					
					break;
					
				case NOVATEL_INSPVA:
					
					//GPSTime gtTimeStamp;
					if(iUseSHM == 0)
					{
					  fprintf(stderr,"%s WARNING: Could not store INSPVA in SHM\n",MODULE_NAME);
					} else
					{
						/* copy timestamp */
						memcpy(&shm->CurrentINSPVA.gtTimeStamp,&GPSTimeStamp,sizeof(GPSTime));
					
						/* copy data */
						memcpy(&shm->CurrentINSPVA.dLatitude,&azucBuffer[ucHeaderLength+12],8);
						memcpy(&shm->CurrentINSPVA.dLongitude,&azucBuffer[ucHeaderLength+20],8);
						memcpy(&shm->CurrentINSPVA.dHeight,&azucBuffer[ucHeaderLength+28],8);
						memcpy(&shm->CurrentINSPVA.dVelN,&azucBuffer[ucHeaderLength+36],8);
						memcpy(&shm->CurrentINSPVA.dVelE,&azucBuffer[ucHeaderLength+44],8);
						memcpy(&shm->CurrentINSPVA.dVelD,&azucBuffer[ucHeaderLength+52],8);
						memcpy(&shm->CurrentINSPVA.dPhi,&azucBuffer[ucHeaderLength+60],8);
						memcpy(&shm->CurrentINSPVA.dTheta,&azucBuffer[ucHeaderLength+68],8);
						memcpy(&shm->CurrentINSPVA.dPsi,&azucBuffer[ucHeaderLength+76],8);
						memcpy(&shm->CurrentINSPVA.INSStatus,&azucBuffer[ucHeaderLength+88],4);
				
					}

					break;

				default:
					log("Unknown Message");
					break;		
			}
			
		
		}
	
		if(iGotBestPos == 0 && iWait1 > 30)
		{
			SendString(iPort,"log bestposb once");
			iWait1 = 0;
		}
		
		if(iGotVersion == 0 && iWait2 > 30)
		{
			SendString(iPort,"log versionb once");
			iWait2 = 0;
		}	
		
		iWait1++;
		iWait2++;
	
	}



	/* unlogall */
	SendString(iPort,"unlogall"); 

	/* close log file */
	if(iLogging)
	{
		log("Closing Log File");
		fclose(fpLogFile);
	}

	if(iUseTimestampFile == 1)
	{
		log("Closing Timestamp File");
		fclose(fpTimestamp);
	}
	/* shutdown */
	log("Closing GPS Port");
	close(iPort);

	/* close shm */
	if (iUnlinkSHM == 1)
	{
		log("Unlinking Shared Memory");
		shm_unlink("/novatel0");
	}
	log("Shutting Down");

	/* exit */
	return 0;
}  /* main */



int SendString(int iPort, char *string)
{
	char cWriteData[100];
	int iBytes = 0;
	int iBytesIn = 0;
	
	char azucReadBuffer[1024];

	bzero(&cWriteData,sizeof(cWriteData));
	sprintf(cWriteData,string);
	
	fprintf(stdout,"%s Tx: %s\n",MODULE_NAME, string);

	iBytes = write(iPort,string,strlen(string));

	if(iBytes < 0)
	{
		perror("write()");
	}
	/* send CRLF */
	write(iPort,"\r\n",2);

	/* check response */
//	iBytesIn = read(iPort,azucReadBuffer,1024);
//	azucReadBuffer[iBytesIn] = '\0';
//	fprintf(stdout,"Response was %d bytes: %s",iBytesIn,azucReadBuffer);	

	return iBytes;
	
	
}


