#pragma once

/* 通用时间定义 */
struct COMMONTIME
{
	short Year;
	unsigned short Month;
	unsigned short Day;
	unsigned short Hour;
	unsigned short Minute;
	double Second;
};
/* 简化儒略日 */
struct MJDTIME
{
	int Days;
	double FracDay;
	MJDTIME()
	{
		Days = 0;
		FracDay = 0.0;
	}
};
/* GPS时间定义 */
struct GPSTIME
{
	unsigned short Week;
	double SecOfWeek;

	double operator -(const GPSTIME& t)
	{
		return (Week - t.Week) * 604800.0 + SecOfWeek - t.SecOfWeek;
	}

	GPSTIME()
	{
		Week = 0;
		SecOfWeek = 0.0;
	}
};

void GPSTimeToMjdTime(GPSTIME* GpsTime, MJDTIME* MjdTime);
void MjdTimeToGPSTime(MJDTIME* MjdTime, GPSTIME* GpsTime);
void CommonTimeToMjdTime(COMMONTIME* CommonTime, MJDTIME* MjdTime);
void MjdTimeToCommonTime(MJDTIME* MjdTime, COMMONTIME* CommonTime);
void CommonTimeToGPSTime(COMMONTIME* CommonTime, GPSTIME* GpsTime);
void GPSTimeToCommonTime(GPSTIME* GpsTime, COMMONTIME* CommonTime);