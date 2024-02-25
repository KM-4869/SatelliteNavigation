#pragma once

#include"SPP.h"
#include"Matrix2.h"
#include<iomanip>//���������ʽ
#include<numeric>//accumulate

struct SD_OBSERVATION_DATA
{
	int Sat_Number;
	GPSTIME GpsTime;

	vector<SD_SATELLITE_OBSERVATION> SdSatObs;
	vector<HISTORY_SAT> History_Sat;

	SD_OBSERVATION_DATA()
	{
		Sat_Number = 0;
	}
};

struct DD_DATA
{
	unsigned short ref_prn[2];//�ο���prn��.0=GPS,1=BDS
	int ref_pos[2];//�ο����ڵ���۲�ֵ�е������š�0=GPS,1=BDS

	double n_of_FloatAmbiguity;//ģ����ά��
	double* FloatAmbiguity;//�����ģ����
	double* Q_of_FloatAmbiguity;//ģ���ȵ�Q��
	double FixedAmbiguity[100];//�̶���ģ���ȣ�0-n���Ž⣬n-2n���Ž⣩
	double ResidualAmbiguity[2], Ratio;//ģ���Ȳв�
	XYZUNION Baseline;
	bool isFixed;

	DD_DATA()
	{
		ref_prn[0] = ref_prn[1] = 0;
		ref_pos[0] = ref_pos[1] = -1;
		Ratio = 0.0;
	}
};

void SingleDifference(OBSERVATION_DATA* Rover_ObsData, OBSERVATION_DATA* Base_ObsData, SD_OBSERVATION_DATA* Sd_ObsData);
void SDDetectOutlier(SD_OBSERVATION_DATA* Obs_Data);
void SelectReferenceSatellite(SD_OBSERVATION_DATA* Sd_ObsData, DD_DATA* Dd_Data);
bool RTKFloat(SD_OBSERVATION_DATA* Sd_ObsData, vector<SATPOSRES>& Rover_SatPosRes, vector<SATPOSRES>& Base_SatPosRes, STAPOSRES* Rover_StaPosRes, STAPOSRES* Base_StaPosRes, DD_DATA* Dd_Data);
bool RTKFixed(SD_OBSERVATION_DATA* Sd_ObsData, vector<SATPOSRES>& Rover_SatPosRes, vector<SATPOSRES>& Base_SatPosRes, STAPOSRES* Rover_StaPosRes, STAPOSRES* Base_StaPosRes, DD_DATA* Dd_Data);
void RTK_Show(GPSTIME* GpsTime, STAPOSRES* StaPosRes, DD_DATA* Dd_Data, int SatNum);
void RTK_OutputToFile(fstream& fout, GPSTIME* GpsTime, STAPOSRES* StaPosRes, DD_DATA* Dd_Data, int SatNum);