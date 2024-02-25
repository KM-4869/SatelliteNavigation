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

//�������ǹ۲�ֵ�ṹ��
struct SD_SATELLITE_OBSERVATION
{
	unsigned short prn;

	NAVIGATION_SYSTEM Navigation_System;

	double psr[2];
	double adr[2];

	//��Ϊ�жϲο��ǵ�����
	float CNo[2];
	float locktime[2];

};

enum POSITION_STATE
{
	NOPOS, SPP, RTKFLOAT, RTKFIXED
};

//����λ�ý���ṹ��
struct SATPOSRES
{
	unsigned long prn;
	NAVIGATION_SYSTEM Navigation_System;
	XYZUNION SatXyz;//����λ��

	double SatVel[3];//�����ٶ�

	double delta_tsv;        //�����Ӳ����
	double RateOf_delta_tsv; //�������ٸ���

	SATPOSRES()
	{
		memset(this, 0, sizeof(SATPOSRES));
	}

	//���صȺţ��ӵ���۲������Ҷ�Ӧ����ʱ��prn�������Ǻ����ʱ�ж϶������
	bool operator==(SD_SATELLITE_OBSERVATION sdsatobs)
	{
		return(prn == sdsatobs.prn && Navigation_System == sdsatobs.Navigation_System);
	}


};
//���ջ���λ����ṹ��
struct STAPOSRES
{
	XYZUNION StationPos;//���ջ�λ��
	BLH StationPosBlh;//���ջ�λ�ã���γ�ȣ�
	double StationVel[3];//���ջ��ƶ��ٶ�

	double ct_R[4];//���ջ��Ӳm��
	double RateOfct_R;//���ջ����٣�m/s��

	//��λ���پ�����Ϣ
	double Pos_sigma0;
	double Vel_sigma0;

	//SPP PDOP Э������
	double PDOP;
	double Qxx[49];

	//�Ƿ�λ�ɹ�����λ״̬
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
