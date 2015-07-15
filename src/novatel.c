

#include "novatel.h"



#define CRC32_POLYNOMIAL 0xEDB88320L


unsigned long CRC32Value(int i)
{
	int j;
	unsigned long ulCRC;
	
	ulCRC = i;
	for(j=8;j>0;j--)
	{
		if(ulCRC & 1)
			ulCRC = (ulCRC>>1)^CRC32_POLYNOMIAL;
		else
			ulCRC>>=1;
	}
	return ulCRC;
}


unsigned long CalculateBlockCRC32(unsigned long ulCount, unsigned char *ucBuffer)
{
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;
	
	while(ulCount-- != 0)
	{
		ulTemp1 = (ulCRC>>8)&0x00FFFFFFL;
		ulTemp2 = CRC32Value(((int)ulCRC^*ucBuffer++)&0xFF);
		ulCRC = ulTemp1^ulTemp2;
	}
	return(ulCRC);

}


