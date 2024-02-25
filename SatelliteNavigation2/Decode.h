#pragma once
#include<vector>
#include<iostream>
#include<fstream>
#include<string.h>
#include"Coordinate.h"
#include"MyTime.h"
#include"sockets.h"
#define POLYCRC32 0xEDB88320u /* CRC32 polynomial */
#define MaxBuffLen 10240

using namespace std;

class NonAteloem7DataLoader
{
public:
	NonAteloem7DataLoader() = default;

	void setRunMode(int runmode);

	void SetFileAddress(const string& fname);
	void SetSocketAddress(const char* serveraddress, unsigned short p);

	bool open();
	void close();

	bool ReadBuff(unsigned char* buff, int RemLen);


private:

	fstream fin;
	SOCKET sock;

	string filename;
	char ServerAddress[20];
	unsigned short port;

	int RunMode;//1为文件，2为网络
};

//万能解码函数
template<typename TYPE>//一定要紧跟在前面写。  
inline void BinaryToAnytype(TYPE* Variable, unsigned char* p)
{
	memcpy(Variable, p, sizeof(TYPE));
}

enum NAVIGATION_SYSTEM
{
	GPS, BDS, GALILEO, GLONASS, OTHER
};

struct HISTORY_SAT
{
	unsigned short prn;
	NAVIGATION_SYSTEM Navigation_System;

	//用于粗差探测
	double Last_L_gf;  //上一历元的载波相位gf组合
	double Average_MW; //之前历元的平滑MW观测值
	int n;             //迄今为止历元数

	bool update;      //检查本卫星此历元是否被更新

	HISTORY_SAT()
	{
		prn = 0;
		Navigation_System = GPS;
		Last_L_gf = 0;
		Average_MW = 0;
		n = 0;
		update = false;
		//		memset(this, 0, sizeof(HISTORY_SAT));
	}
};

struct SATELLITE_OBSERVATION
{
	unsigned short prn;

	NAVIGATION_SYSTEM Navigation_System;

	double psr[2];
	float psr_sigma[2];
	double adr[2];
	float adr_sigma[2];
	float dopp[2];
	float CNo[2];
	float locktime[2];
	bool ParityKnownFlag;

	double P_if;//伪距的if组合观测值

	//重载等号。prn与导航系统都相等时两个观测值相等
	bool operator==(SATELLITE_OBSERVATION satobs)
	{
		return (prn == satobs.prn && Navigation_System == satobs.Navigation_System);
	}

};

struct OBSERVATION_DATA
{
	unsigned long Obs_Number;
	int Sat_Number;

	GPSTIME GpsTime;
	vector<SATELLITE_OBSERVATION> SatObs;
	vector<HISTORY_SAT> History_Sat;
	OBSERVATION_DATA()
	{
		Obs_Number = 0;
		Sat_Number = 0;
	}
};

struct EPHEMERIS
{
	unsigned long prn;
	unsigned long health;

	GPSTIME toe;
	GPSTIME toc;
	double a;
	double MeanAnomaly;
	double ecc;
	double delta_n;
	double ArgumentPerigee;

	double cuc;
	double cus;
	double crc;
	double crs;
	double cic;
	double cis;

	double I0;
	double RateOfI;
	double RightAscension;
	double RateOfRightAscension;

	double af0;
	double af1;
	double af2;

	double tgd1;
	double tgd2;

	bool operator==(const int& p)
	{
		return (this->prn == p);
	}


	EPHEMERIS()
	{
		memset(this, 0, sizeof(EPHEMERIS));
	}

};

struct DECODEPOS
{
	short Satellite_Number;

	BLH Blh;
	float lat_sigma;
	float lon_sigma;
	float hgt_sigma;

	float sol_age;

	DECODEPOS()
	{
		memset(this, 0, sizeof(DECODEPOS));
	}
};

unsigned int crc32(const unsigned char* buff, int len);
//int FindFirstSyncBytePos(unsigned char* buff, int BuffLen);
GPSTIME BDST2GPST(GPSTIME& GpsTime);
GPSTIME GPST2BDST(GPSTIME& GpsTime);
unsigned short NonAteloem7DataDecode(unsigned char* buff, int* RemLen, OBSERVATION_DATA* Obs_Data, vector<EPHEMERIS>* GPSEphem, vector<EPHEMERIS>* BDSEphem, DECODEPOS* Pos);
unsigned short NonAteloem7DataDecode(unsigned char* buff, int* RemLen, OBSERVATION_DATA* Obs_Data, DECODEPOS* Pos);