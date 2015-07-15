#include <stdio.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>

#include "shmdef.h"
#include "novatel.h"


#define MODULE_NAME	"[Novatel Client]"

#define log(string)	fprintf(stdout,"%s %s\n", MODULE_NAME, string)
#define err(string)	fprintf(stderr,"%s ERROR: %s\n", MODULE_NAME, string)

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


int main(int argc, char **argv)
{
        int iUseSHM = 1;
        int iSHM_FID;
        int iIndex = 0;        
        shm_struct *shm = NULL;        

	/* setup signal handlers */
	signal(SIGHUP,  HandleSignal);
	signal(SIGKILL, HandleSignal);
	signal(SIGTERM, HandleSignal);
	signal(SIGALRM, HandleSignal);
	signal(SIGINT, HandleSignal);


	/* setup shared memory */
	if(iUseSHM == 1)
	{
	  iSHM_FID = shm_open("/novatel0",O_RDONLY,0777);
	  if(iSHM_FID == -1)
	  {
		perror("shm_open: ");
		iUseSHM = 0;
 	  } else
	  {
	     	/* memory map the shm object */
	     	shm = mmap(0,sizeof(*shm),PROT_READ,MAP_SHARED,iSHM_FID,0);
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


	while (iShutdown == 0)
	{
	  sleep(1);
	  fprintf(stdout,"GPSTime: %d:%f: Current Observations: %ld\n",
	  shm->CurrentRANGE.gtTimeStamp.usGPSWeek,
	  shm->CurrentRANGE.gtTimeStamp.fGPSSecondOfWeek,
	  shm->CurrentRANGE.lNumberObservations);

	  fprintf(stdout,"Current Ephem:\n");
	  for(iIndex=0;iIndex<32;iIndex++)
	  {
	    if(shm->CurrentGPSEPHEM[iIndex].ulPRN != 0)
	    {
	      fprintf(stdout,"index: %d, PRN %d, TOE:%f\n",iIndex,
	                                shm->CurrentGPSEPHEM[iIndex].ulPRN,
	                                shm->CurrentGPSEPHEM[iIndex].dTOE);
            }
	  }
	  
	}

  return 0;
}

