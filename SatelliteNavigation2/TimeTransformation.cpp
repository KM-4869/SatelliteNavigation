
#include"MyTime.h"
#include<math.h>

/************************************************************


******************************************************************/

void MjdTimeToCommonTime(MJDTIME *MjdTime, COMMONTIME *CommonTime)
{
	double JD = MjdTime->Days+MjdTime->FracDay + 2400000.5;

	int a = int(JD + 0.5);
	int b = a + 1537;
	int c = int((b - 122.1) / 365.25);
	int d = int(365.25 * c);
	int e = int((b - d) / 30.6001);
	CommonTime->Day =int(( b - d - int(30.6001 * e) + (JD + 0.5 - a)));
	CommonTime->Month = e - 1 - 12 * int(e / 14);
	CommonTime->Year = c - 4715 - int((7 + CommonTime->Month) / 10);

	CommonTime->Hour = int(MjdTime->FracDay*24.0);
	CommonTime->Minute = int((MjdTime->FracDay * 24.0 - CommonTime->Hour) * 60);
	CommonTime->Second = ((MjdTime->FracDay * 24.0 - CommonTime->Hour) * 60 - CommonTime->Minute) * 60;

	

}


void CommonTimeToMjdTime(COMMONTIME*CommonTime ,MJDTIME*MjdTime)
{

	short y = 0;
	unsigned short m = 0;

	if (CommonTime->Month<= 2)
	{
		y = CommonTime->Year - 1;
		m = CommonTime->Month + 12;
	}
	else if (CommonTime->Month > 2)
	{
		y = CommonTime->Year;
		m = CommonTime->Month;
	}

	int intJD = floor(365.25 * y) + floor(30.6001 * (m + 1)) + CommonTime->Day + 1720981;
	double fracJD = (double(CommonTime->Hour) + double(CommonTime->Minute / 60.0) + double(CommonTime->Second / 3600.0)) / 24.0 + 0.5;
	
	MjdTime->Days = intJD - 2400000;
	MjdTime->FracDay = fracJD - 0.5;

	
}


void MjdTimeToGPSTime(MJDTIME*MjdTime, GPSTIME*GpsTime)
{
	GpsTime->Week = int((MjdTime->Days - 44244) / 7);
	GpsTime->SecOfWeek = (MjdTime->Days - 44244 - GpsTime->Week * 7) * 86400.0+MjdTime->FracDay*86400;
	
}


void GPSTimeToMjdTime(GPSTIME*GpsTime,MJDTIME*MjdTime)
{
	MjdTime->Days = 44244 + (GpsTime->Week * 7)+int(GpsTime->SecOfWeek/86400);
	MjdTime->FracDay = GpsTime->SecOfWeek / 86400.0 - int(GpsTime->SecOfWeek / 86400);

}


void CommonTimeToGPSTime(COMMONTIME*CommonTime,GPSTIME*GpsTime)
{
	MJDTIME MjdTime;

	CommonTimeToMjdTime(CommonTime,&MjdTime);

	MjdTimeToGPSTime(&MjdTime,GpsTime);

	
}

void GPSTimeToCommonTime(GPSTIME* GpsTime, COMMONTIME* CommonTime)
{
	MJDTIME MjdTime;

	GPSTimeToMjdTime(GpsTime, &MjdTime);

	MjdTimeToCommonTime(&MjdTime, CommonTime);

	
}
