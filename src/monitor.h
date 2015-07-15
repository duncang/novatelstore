/** 
 * 	\file monitor.h
 *	\author Troy Bruggemann
 *	
 * 
 * 	$Id: monitor.h 333 2007-07-20 06:11:53Z n2523710 $
 */


/****************************************************************************
* MonitorSendMSG.c
****************************************************************************/

int MonitorSetupUDP(int iPort, char *string);
int MonitorSendMSG(int iPort, char *string);




/* Need to define this structures properly */

typedef struct _GPSMonitor
{
    int_8 iGPSLoggerStatus;
    int_8 iGPSSolutionStatus;
    float fGPSSolutionAge;

    int_8 iGPSSatellitesTracking;
    double dGPSTrackingTime;

    double dGPSLatitude;
    double dGPSLongitude;
    double dGPSHeightAMSL;
    double dGPSHeightWGS84;

    int_32 lGPSRecordsLogged_BESTXYZ;
    int_32 lGPSRecordsLogged_RAWEPHEM;
    int_32 lGPSRecordsLogged_IONUTC;
    int_32 lGPSRecordsLogged_RANGE;
    int_32 lGPSRecordsLogged_VERSION;

} GPSMonitor;


typedef struct _IMUMonitor
{
    int_8 iGPSLoggerStatus;
    int_8 iGPSSolutionStatus;
    float fGPSSolutionAge;

    int_8 iGPSSatellitesTracking;
    double dGPSTrackingTime;

    double dGPSLatitude;
    double dGPSLongitude;
    double dGPSHeightAMSL;
    double dGPSHeightWGS84;

    int_32 lGPSRecordsLogged_BESTXYZ;
    int_32 lGPSRecordsLogged_RAWEPHEM;
    int_32 lGPSRecordsLogged_IONUTC;
    int_32 lGPSRecordsLogged_RANGE;
    int_32 lGPSRecordsLogged_VERSION;

} IMUMonitor;




typedef struct _AirDataMonitor
{
    int_8 iGPSLoggerStatus;
    int_8 iGPSSolutionStatus;
    float fGPSSolutionAge;

    int_8 iGPSSatellitesTracking;
    double dGPSTrackingTime;

    double dGPSLatitude;
    double dGPSLongitude;
    double dGPSHeightAMSL;
    double dGPSHeightWGS84;

    int_32 lGPSRecordsLogged_BESTXYZ;
    int_32 lGPSRecordsLogged_RAWEPHEM;
    int_32 lGPSRecordsLogged_IONUTC;
    int_32 lGPSRecordsLogged_RANGE;
    int_32 lGPSRecordsLogged_VERSION;

} AirDataMonitor;

