#pragma once

#define f_L1 1575.42E6
#define f_L2 1227.60E6
#define f_B1I 1561.098E6
#define f_B3I 1268.520E6
#define WaveLen_L1 c_Light/f_L1
#define WaveLen_L2 c_Light/f_L2
#define WaveLen_B1I c_Light/f_B1I
#define WaveLen_B3I c_Light/f_B3I

#include"Coordinate.h"
#include"Constant.h"
#include"Coordinate.h"
#include"Matrix.h"
#include"MyTime.h"
#include"Decode.h"
#include"OptimalEstimation.h"
#include<string>
#include<math.h>
using namespace std;

//单差卫星观测值结构体
struct SD_SATELLITE_OBSERVATION
{
	unsigned short prn;

	NAVIGATION_SYSTEM Navigation_System;

	double psr[2];
	double adr[2];

	//作为判断参考星的依据
	float CNo[2];
	float locktime[2];

};

enum POSITION_STATE
{
	NOPOS, SPP, RTKFLOAT, RTKFIXED
};

//卫星位置结果结构体
struct SATPOSRES
{
	unsigned long prn;
	NAVIGATION_SYSTEM Navigation_System;
	XYZUNION SatXyz;//卫星位置

	double SatVel[3];//卫星速度

	double delta_tsv;        //卫星钟差改正
	double RateOf_delta_tsv; //卫星钟速改正

	SATPOSRES()
	{
		memset(this, 0, sizeof(SATPOSRES));
	}

	//重载等号，从单差观测数据找对应卫星时，prn号与卫星号相等时判断二者相等
	bool operator==(SD_SATELLITE_OBSERVATION sdsatobs)
	{
		return(prn == sdsatobs.prn && Navigation_System == sdsatobs.Navigation_System);
	}


};
//接收机定位结果结构体
struct STAPOSRES
{
	XYZUNION StationPos;//接收机位置
	BLH StationPosBlh;//接收机位置（经纬度）
	double StationVel[3];//接收机移动速度

	double ct_R[4];//接收机钟差（m）
	double RateOfct_R;//接收机钟速（m/s）

	//定位测速精度信息
	double Pos_sigma0;
	double Vel_sigma0;

	//SPP PDOP 协方差阵
	double PDOP;
	double Qxx[49];

	//是否定位成功及定位状态
	POSITION_STATE Pos_State;
	
	STAPOSRES()
	{
		memset(this, 0, sizeof(STAPOSRES));
	}
};



//struct ACCURACY
//{
//	double sigma0;
//	double PDOP;
//	double Qxx[49];
//
//	ACCURACY()
//	{
//		memset(this, 0, sizeof(ACCURACY));
//
//	}
//
//};


void DetectOutlier(OBSERVATION_DATA* Obs_Data);
void GPSSatPos(EPHEMERIS* GPSEphem, GPSTIME t, SATPOSRES* SatPosRes);
void BDSSatPos(EPHEMERIS* BDSEphem, GPSTIME t, SATPOSRES* SatPosRes);
void SignalSendTimeSatPos(OBSERVATION_DATA* Obs_Data, vector<EPHEMERIS>* GPSEphem, vector<EPHEMERIS>* BDSEphem, vector<SATPOSRES>* SatPosRes);
void BDS_Sat_P_if_TGDCorrection(double* P_if, double*tgd1);
bool StandardPointPositioning(OBSERVATION_DATA*Obs_Data, vector<SATPOSRES>* SatPosRes, STAPOSRES*StaPosRes);
bool StandardPointVelocity(OBSERVATION_DATA* Obs_Data, vector<SATPOSRES>* SatPosRes, STAPOSRES* StaPosRes);

void StaPosRes_Show(GPSTIME* GpsTime, STAPOSRES* StaPosRes, int SatNum);
//void StaPosRes_OutPutToFile(FILE* fp, GPSTIME* GpsTime, STAPOSRES* StaPosRes, ACCURACY* PosAccuracy, ACCURACY* VelAccuracy, int SatNum, bool PositioningSuccess);
//void StaPosResWithSat_Show(OBSERVATION_DATA* Obs_Data, STAPOSRES* StaPosRes, vector<SATPOSRES>* SatPosRes, ACCURACY* PosAccuracy, ACCURACY* VelAccuracy, bool PositioningSuccess);
//void StaPosResWithSat_OutPutToFile(FILE* fp, OBSERVATION_DATA* Obs_Data, STAPOSRES* StaPosRes, vector<SATPOSRES>* SatPosRes, ACCURACY* PosAccuracy, ACCURACY* VelAccuracy, bool PositioningSuccess);
