#include"SPP.h"

//�����۲����ݽṹ�壬����ֲ��޳���Ľ��
void DetectOutlier(OBSERVATION_DATA* Obs_Data)
{

	//��ʼ�ֲ�̽��ǰ�Ƚ�������������ݵĸ���״̬��Ϊfalse
	for (int k = 0; k < Obs_Data->History_Sat.size(); k++)
		Obs_Data->History_Sat[k].update = false;

	for (int i = 0; i < Obs_Data->SatObs.size();)
	{
		//���ÿ�����ǵ�˫Ƶα�����λ�����Ƿ���Ч������(ͬʱ��̭����GPS�򱱶�������)
		if (abs(Obs_Data->SatObs[i].psr[0]) < 1.0E-5 || abs(Obs_Data->SatObs[i].psr[1]) < 1.0E-5 || abs(Obs_Data->SatObs[i].adr[0]) < 1.0E-5 || abs(Obs_Data->SatObs[i].adr[1]) < 1.0E-5)
		{
			Obs_Data->SatObs.erase(Obs_Data->SatObs.begin() + i);
			continue;
		}
		//ͨ���˲�������Ƕ���Ϊ�����MW��GF��ֵ

		//���㲢����ÿ�����ǵ�GF��MW���
		double L_gf = 0.0;
		double MW = 0.0;
		double L1=0.0, L2=0.0;
		double P1 = Obs_Data->SatObs[i].psr[0];
		double P2 = Obs_Data->SatObs[i].psr[1];

		switch (Obs_Data->SatObs[i].Navigation_System)
		{
		case GPS:
			L1 = Obs_Data->SatObs[i].adr[0] * WaveLen_L1;
			L2 = Obs_Data->SatObs[i].adr[1] * WaveLen_L2;
			L_gf = L1 - L2;
			MW = 1 / (f_L1 - f_L2) * (f_L1 * L1 - f_L2 * L2) - 1 / (f_L1 + f_L2) * (f_L1 * P1 + f_L2 * P2);//��λ��
			break;
		case BDS:
			L1 = Obs_Data->SatObs[i].adr[0] * WaveLen_B1I;
			L2 = Obs_Data->SatObs[i].adr[1] * WaveLen_B3I;
			L_gf = L1 - L2;
			MW = 1 / (f_B1I - f_B3I) * (f_B1I * L1 - f_B3I * L2) - 1 / (f_B1I + f_B3I) * (f_B1I * P1 + f_B3I * P2);//��λ��
			break;
		default:
			break;
		}
			
		int j = 0;
		bool find = false;
		bool gross_error = false;
		for ( j = 0; j < Obs_Data->History_Sat.size(); j++)
		{
			if (Obs_Data->SatObs[i].Navigation_System == Obs_Data->History_Sat[j].Navigation_System && Obs_Data->SatObs[i].prn == Obs_Data->History_Sat[j].prn)
			{
				find = true;

				//���㵱ǰ��Ԫÿ������GF����һ��Ԫ��ӦGF�Ĳ�ֵ
				  double dGF = L_gf - Obs_Data->History_Sat[j].Last_L_gf;
				//���㵱ǰ��Ԫÿ������MW����һ��Ԫ��ӦMWƽ��ֵ�Ĳ�ֵ
				  double dMW = MW - Obs_Data->History_Sat[j].Average_MW;

				//���dGF��dMW�Ƿ���
				if (abs(dGF) < 0.05 && abs(dMW) < 2.0)
				{
					//���ڿ��õĹ۲����ݣ�����α���IF��Ϲ۲�ֵ������SPP
					switch (Obs_Data->SatObs[i].Navigation_System)
					{
					case GPS:Obs_Data->SatObs[i].P_if = 1 / (f_L1 * f_L1 - f_L2 * f_L2) * (f_L1 * f_L1 * P1 - f_L2 * f_L2 * P2);
						break;
					case BDS:Obs_Data->SatObs[i].P_if = 1 / (f_B1I * f_B1I - f_B3I * f_B3I) * (f_B1I * f_B1I * P1 - f_B3I * f_B3I * P2);
						break;					
					default:
						break;
					}
				}
				else
				{   
					//����дֲ��������Ǵ���ʷ������ɾ������һ�γ���ʱ���������������Ǽӵ�����ĩβ
					Obs_Data->SatObs.erase(Obs_Data->SatObs.begin() + i);
					Obs_Data->History_Sat.erase(Obs_Data->History_Sat.begin() + j);
					gross_error = true;
				}
				break;
			}
		}
		//��������ǵ�MW��ϵ�ƽ��ֵ,���赱ǰ��Ԫ��GFֵ����ʶ���������Ѹ���
		//�����Ϊ�дֲ����ɾ�������ǣ����ٸ��£�������һ�����ǵĲ���
		if (gross_error == true)continue;
		
		if (find == false)
		{
			//û�ҵ�˵�����״γ��ֵ����ǣ�������prn���Լ�����ϵͳ�����������������һ������ҵ�����ȥ��prn���뵼��ϵͳ��һ����
			//�����״γ��֣������ж��Ƿ��дֲ����ҲҪɾ��
			Obs_Data->History_Sat.push_back({});
			Obs_Data->History_Sat[j].prn = Obs_Data->SatObs[i].prn;
			Obs_Data->History_Sat[j].Navigation_System = Obs_Data->SatObs[i].Navigation_System;
			Obs_Data->SatObs.erase(Obs_Data->SatObs.begin() + i);
			i--;
		}
			Obs_Data->History_Sat[j].n++;
			Obs_Data->History_Sat[j].Average_MW = (Obs_Data->History_Sat[j].n - 1.0) / Obs_Data->History_Sat[j].n * Obs_Data->History_Sat[j].Average_MW + 1.0 / Obs_Data->History_Sat[j].n * MW;
			Obs_Data->History_Sat[j].Last_L_gf = L_gf;
			Obs_Data->History_Sat[j].update = true;
			i++;
	}

	//�����Ѿ���ɸ��±���ı���Ԫ���ݣ���λgf��ϣ�ƽ��MWֵ����������ֱ�����������δ�����¹��ģ�˵���������ڱ���Ԫ�й۲�ֵȱʧ���߸���û�г��ֹ�
	//��ô����һ�γ���ʱ�����汣������ݿ��ܽ���n����Ԫ֮ǰ��
	//Ϊ�˷�ֹ����������֣���һ�����ǵ���ʷ����δ������ʱ������������ʷ����ɾ��
	//��һ�γ���ʱ�����������������Ǽ�������ĩβ��
	for (int j = 0; j < Obs_Data->History_Sat.size();)
	{
		if (Obs_Data->History_Sat[j].update == false)
			Obs_Data->History_Sat.erase(Obs_Data->History_Sat.begin() + j);
		else
			j++;
	}
}





void GPSSatPos(EPHEMERIS* GPSEphem, GPSTIME t, SATPOSRES*SatPosRes)
{
	SatPosRes->prn = GPSEphem->prn;
	SatPosRes->Navigation_System = GPS;
	double n0 = sqrt(GPS_GM) / (sqrt(GPSEphem->a) * sqrt(GPSEphem->a) * sqrt(GPSEphem->a));      //�����˶�ƽ�����ٶ�
	double tk1 = t - GPSEphem->toe;//����t����ڲο���Ԫ��ʱ��
	double tk2 = t - GPSEphem->toc;//����t�����toc��ʱ��

	double n = n0 + GPSEphem->delta_n;           //�����㶯�������ù۲�ʱ��ƽ�����ٶ�
	double M = GPSEphem->MeanAnomaly + n * tk1;   //�۲�ʱ��ƽ�����

	double E = M;      //���ƫ����ǲ����е���
	for (int i = 1; i < 20; i++)
	{
		double E0 = E;
		E = M + GPSEphem->ecc * sin(E);
		
		if (abs(E - E0) < 5.0E-10)
		{
			break;
		}
	}

	double f = atan2((sqrt(1 - GPSEphem->ecc * GPSEphem->ecc) * sin(E)), (cos(E) - GPSEphem->ecc));//����������
	
	double u0 = GPSEphem->ArgumentPerigee + f;      //���������Ǿ�

	//��������㶯��
	double delta_u = GPSEphem->cuc * cos(2 * u0) + GPSEphem->cus * sin(2 * u0);
	double delta_r = GPSEphem->crc * cos(2 * u0) + GPSEphem->crs * sin(2 * u0);
	double delta_i = GPSEphem->cic * cos(2 * u0) + GPSEphem->cis * sin(2 * u0);

	//�㶯����
	double u = u0 + delta_u;
	double r = GPSEphem->a * (1 - GPSEphem->ecc * cos(E)) + delta_r;//r0=a(1-ecosE)
	double i = GPSEphem->I0 + delta_i + GPSEphem->RateOfI * tk1;

	//����������ƽ���ϵ�λ��
	double x = r * cos(u);
	double y = r * sin(u);

	//����������ľ���
	double L = GPSEphem->RightAscension + (GPSEphem->RateOfRightAscension - GPS_we) * tk1 - GPS_we * GPSEphem->toe.SecOfWeek;
	//�����������ϵ������
	SatPosRes->SatXyz.Xyz.X = x * cos(L) - y * cos(i) * sin(L);
	SatPosRes->SatXyz.Xyz.Y = x * sin(L) + y * cos(i) * cos(L);
	SatPosRes->SatXyz.Xyz.Z = y * sin(i);

	//�����Ӳ����
	double delta_tr = F_Relativity * GPSEphem->ecc * sqrt(GPSEphem->a) * sin(E);//�����ЧӦ����
	SatPosRes->delta_tsv = GPSEphem->af0 + GPSEphem->af1 * tk2 + GPSEphem->af2 * tk2 * tk2 + delta_tr;

	//�������ٸ���
	double RateOfE = n / (1 - GPSEphem->ecc * cos(E));
	double RateOfdelta_tr = F_Relativity * GPSEphem->ecc * sqrt(GPSEphem->a) * cos(E) * RateOfE;
	SatPosRes->RateOf_delta_tsv = GPSEphem->af1 + 2 * GPSEphem->af2 * tk2 + RateOfdelta_tr;


	//�����ٶȼ���
	//double dot_Phi_k = sqrt((1 + GPSEphem->ecc) / (1 - GPSEphem->ecc)) * (cos(f / 2) / cos(E / 2)) * (cos(f / 2) / cos(E / 2)) * RateOfE;
	double dot_Phi_k = sqrt(1 - GPSEphem->ecc * GPSEphem->ecc) / (1 - cos(E) * GPSEphem->ecc) * RateOfE;
	double dot_uk = 2 * (GPSEphem->cus * cos(2 * u0) - GPSEphem->cuc * sin(2 * u0)) * dot_Phi_k + dot_Phi_k;
	double dot_rk = GPSEphem->a * GPSEphem->ecc * sin(E) * RateOfE + 2 * (GPSEphem->crs * cos(2 * u0) - GPSEphem->crc * sin(2 * u0)) * dot_Phi_k;
	double dot_Ik = GPSEphem->RateOfI + 2 * (GPSEphem->cis * cos(2 * u0) - GPSEphem->cic * sin(2 * u0)) * dot_Phi_k;
	double dot_Omega_k = GPSEphem->RateOfRightAscension - GPS_we;

	
	double dot_xk = dot_rk * cos(u) - r * dot_uk * sin(u);
	double dot_yk = dot_rk * sin(u) + r * dot_uk * cos(u);

	double dot_R[12] = { cos(L),-sin(L) * cos(i),-(x * sin(L) + y * cos(L) * cos(i)),y * sin(L) * sin(i),
						 sin(L), cos(L) * cos(i),(x * cos(L) - y * sin(L) * cos(i)),-y * cos(L) * sin(i),
						 0, sin(i) ,0, y * cos(i) };

	double xyOI[4] = { dot_xk,dot_yk,dot_Omega_k,dot_Ik };

	MatrixMultiply(3, 4, 4, 1, dot_R, xyOI, SatPosRes->SatVel);

}

void BDSSatPos(EPHEMERIS* BDSEphem, GPSTIME t, SATPOSRES* SatPosRes)
{
	t = GPST2BDST(t);
	SatPosRes->prn = BDSEphem->prn;
	SatPosRes->Navigation_System = BDS;
	double n0 = sqrt(BDS_GM) / (BDSEphem->a * BDSEphem->a * BDSEphem->a);      //�����˶�ƽ�����ٶ�
	double tk1 = t - BDSEphem->toe;//����t����ڲο���Ԫ��ʱ��
	double tk2 = t - BDSEphem->toc;//����t�����toc��ʱ��

	double n = n0 + BDSEphem->delta_n;           //�����㶯�������ù۲�ʱ��ƽ�����ٶ�
	double M = BDSEphem->MeanAnomaly + n * tk1;   //�۲�ʱ��ƽ�����

	double E = M;      //���ƫ����ǲ����е���
	for (int i = 1; i < 20; i++)
	{
		double E0 = E;
		E = M + BDSEphem->ecc * sin(E);

		if (abs(E - E0) < 5.0E-10)
		{
			break;
		}
	}

	double f = atan2((sqrt(1 - BDSEphem->ecc * BDSEphem->ecc) * sin(E)), (cos(E) - BDSEphem->ecc));//����������

	double u0 = BDSEphem->ArgumentPerigee + f;      //���������Ǿ�

	//��������㶯��
	double delta_u = BDSEphem->cuc * cos(2 * u0) + BDSEphem->cus * sin(2 * u0);
	double delta_r = BDSEphem->crc * cos(2 * u0) + BDSEphem->crs * sin(2 * u0);
	double delta_i = BDSEphem->cic * cos(2 * u0) + BDSEphem->cis * sin(2 * u0);

	//�㶯����
	double u = u0 + delta_u;
	double r = BDSEphem->a * BDSEphem->a * (1 - BDSEphem->ecc * cos(E)) + delta_r;//r0=a(1-ecosE)
	double i = BDSEphem->I0 + delta_i + BDSEphem->RateOfI * tk1;

	//����������ƽ���ϵ�λ��
	double x = r * cos(u);
	double y = r * sin(u);

	double L = 0.0;
	double X_GK=0.0, Y_GK=0.0, Z_GK=0.0;
	if ((1 <= SatPosRes->prn && SatPosRes->prn <= 5) || (59 <= SatPosRes->prn && SatPosRes->prn <= 63))
	{
		L = BDSEphem->RightAscension + BDSEphem->RateOfRightAscension * tk1 - BDS_we * BDSEphem->toe.SecOfWeek;

		  X_GK = x * cos(L) - y * cos(i) * sin(L);
		  Y_GK = x * sin(L) + y * cos(i) * cos(L);
		  Z_GK = y * sin(i);

		 double C[9];
		 double XYZ_GK[3] = { X_GK,Y_GK,Z_GK };
		 RotateMatrix(Deg2Rad(-5.0) , 0, BDS_we * (tk1), 5, C);
		 MatrixMultiply(3, 3, 3, 1, C, XYZ_GK, SatPosRes->SatXyz.xyz);

	}
	else
	{
	     L = BDSEphem->RightAscension + (BDSEphem->RateOfRightAscension - BDS_we) * tk1 - BDS_we * BDSEphem->toe.SecOfWeek;
	
		SatPosRes->SatXyz.Xyz.X = x * cos(L) - y * cos(i) * sin(L);
		SatPosRes->SatXyz.Xyz.Y = x * sin(L) + y * cos(i) * cos(L);
		SatPosRes->SatXyz.Xyz.Z = y * sin(i);

	}

	//�����Ӳ����
	double delta_tr = F_Relativity * BDSEphem->ecc * BDSEphem->a * sin(E);//�����ЧӦ����
	SatPosRes->delta_tsv = BDSEphem->af0 + BDSEphem->af1 * tk2 + BDSEphem->af2 * tk2 * tk2 + delta_tr;

	//�������ٸ���(ע�ⱱ����aΪ�볤��ƽ��������ͬ)
	double RateOfE = n / (1 -BDSEphem->ecc * cos(E));
	double RateOfdelta_tr = F_Relativity * BDSEphem->ecc * BDSEphem->a * cos(E) * RateOfE;
	SatPosRes->RateOf_delta_tsv = BDSEphem->af1 + 2 * BDSEphem->af2 * tk2 + RateOfdelta_tr;


	//�����ٶȼ���
	//double dot_Phi_k = sqrt((1 + BDSEphem->ecc) / (1 - BDSEphem->ecc)) * (cos(f / 2) / cos(E / 2)) * (cos(f / 2) / cos(E / 2)) * RateOfE;
	double dot_Phi_k = sqrt(1 - BDSEphem->ecc * BDSEphem->ecc) / (1 - cos(E) * BDSEphem->ecc) * RateOfE;
	double dot_uk = 2 * (BDSEphem->cus * cos(2 * u0) - BDSEphem->cuc * sin(2 * u0)) * dot_Phi_k + dot_Phi_k;
	double dot_rk = (BDSEphem->a) * (BDSEphem->a) * BDSEphem->ecc * sin(E) * RateOfE + 2 * (BDSEphem->crs * cos(2 * u0) - BDSEphem->crc * sin(2 * u0)) * dot_Phi_k;
	double dot_Ik = BDSEphem->RateOfI + 2 * (BDSEphem->cis * cos(2 * u0) - BDSEphem->cic * sin(2 * u0)) * dot_Phi_k;
	
	if ((1 <= SatPosRes->prn && SatPosRes->prn <= 5) || (59 <= SatPosRes->prn && SatPosRes->prn <= 63))
	{
		double dot_Omega_k = BDSEphem->RateOfRightAscension;

		double dot_xk = dot_rk * cos(u) - r * dot_uk * sin(u);
		double dot_yk = dot_rk * sin(u) + r * dot_uk * cos(u);

		double dot_R[12] = { cos(L),-sin(L) * cos(i),-(x * sin(L) + y * cos(L) * cos(i)),y * sin(L) * sin(i),
							 sin(L), cos(L) * cos(i),(x * cos(L) - y * sin(L) * cos(i)),-y * cos(L) * sin(i),
							 0, sin(i) ,0, y * cos(i) };

		double xyOI[4] = { dot_xk,dot_yk,dot_Omega_k,dot_Ik };

		double dot_XYZ_GK[3];//GEO���ض�����ϵ�µ��ٶ�

		MatrixMultiply(3, 4, 4, 1, dot_R, xyOI, dot_XYZ_GK);

		double deg_5 = Deg2Rad(-5);//-5��Ļ���

		double dot_RZRX[12] = { cos(BDS_we * tk1),sin(BDS_we * tk1) * cos(deg_5),sin(BDS_we * tk1) * sin(deg_5),-X_GK * sin(BDS_we * tk1) * BDS_we + Y_GK * cos(deg_5) * cos(BDS_we * tk1) * BDS_we + Z_GK * sin(deg_5) * cos(BDS_we * tk1) * BDS_we,
							   -sin(BDS_we * tk1),cos(BDS_we * tk1) * cos(deg_5),cos(BDS_we * tk1) * sin(deg_5),-X_GK * cos(BDS_we * tk1) * BDS_we - Y_GK * cos(deg_5) * sin(BDS_we * tk1) * BDS_we - Z_GK * sin(deg_5) * sin(BDS_we * tk1) * BDS_we,
							    0.0,-sin(deg_5),cos(deg_5),0.0 };
		double dot_XYZ_GK_1[4] = { dot_XYZ_GK[0],dot_XYZ_GK[1],dot_XYZ_GK[2],1.0 };

		MatrixMultiply(3, 4, 4, 1, dot_RZRX, dot_XYZ_GK_1, SatPosRes->SatVel);

	}
	else
	{
		double dot_Omega_k = BDSEphem->RateOfRightAscension - BDS_we;

		double dot_xk = dot_rk * cos(u) - r * dot_uk * sin(u);
		double dot_yk = dot_rk * sin(u) + r * dot_uk * cos(u);

		double dot_R[12] = { cos(L),-sin(L) * cos(i),-(x * sin(L) + y * cos(L) * cos(i)),y * sin(L) * sin(i),
							 sin(L), cos(L) * cos(i),(x * cos(L) - y * sin(L) * cos(i)),-y * cos(L) * sin(i),
							 0, sin(i) ,0, y * cos(i) };

		double xyOI[4] = { dot_xk,dot_yk,dot_Omega_k,dot_Ik };

		MatrixMultiply(3, 4, 4, 1, dot_R, xyOI, SatPosRes->SatVel);
	}

}

//�Ա���B1IB3I��˫Ƶif�����TGD����
void BDS_Sat_P_if_TGDCorrection(double*P_if,double*tgd1)
{
	*P_if += c_Light * f_B1I * f_B1I * (*tgd1) / (f_B3I * f_B3I - f_B1I * f_B1I);
}

//���ڹ۲������е����ǣ������������в��ң��ҵ���Ϊ�����һ���źŷ���ʱ������λ�ò�������Ҳ�����������������ɾ����
void SignalSendTimeSatPos(OBSERVATION_DATA* Obs_Data, vector<EPHEMERIS>* GPSEphem, vector<EPHEMERIS>* BDSEphem, vector<SATPOSRES>* SatPosRes)
{
	for (int i = 0; i < Obs_Data->SatObs.size();)
	{
		vector<EPHEMERIS>::iterator it;
		GPSTIME t_S_GPST;//�źŷ���ʱ�̵�GPS��׼ʱ��
		double tk2;//�����ӱ���ʱ��toc֮��

		//�������в��ҹ۲������е�����
		switch (Obs_Data->SatObs[i].Navigation_System)
		{
		case GPS:
			it = find(GPSEphem->begin(), GPSEphem->end(), Obs_Data->SatObs[i].prn);
			if (it != GPSEphem->end())
			{
				t_S_GPST.Week = Obs_Data->GpsTime.Week;
				t_S_GPST.SecOfWeek = Obs_Data->GpsTime.SecOfWeek - Obs_Data->SatObs[i].P_if / c_Light;//�����ӱ���ʱ
				tk2 = t_S_GPST-it->toc;
				t_S_GPST.SecOfWeek -= (it->af0 + it->af1 * tk2 + it->af2 * tk2 * tk2);//�����Ӳ�
				SatPosRes->push_back({});
				GPSSatPos(&(*it), t_S_GPST, &SatPosRes->back());//�����źŷ���ʱ������λ��
				i++;
			}
			else
				Obs_Data->SatObs.erase(Obs_Data->SatObs.begin() + i);
			break;
		case BDS:
			it = find(BDSEphem->begin(), BDSEphem->end(), Obs_Data->SatObs[i].prn);
			if (it != BDSEphem->end())
			{
				BDS_Sat_P_if_TGDCorrection(&Obs_Data->SatObs[i].P_if, &it->tgd1);
				t_S_GPST.Week = Obs_Data->GpsTime.Week;
				t_S_GPST.SecOfWeek = Obs_Data->GpsTime.SecOfWeek - Obs_Data->SatObs[i].P_if / c_Light;//�����ӱ���ʱ
				tk2 = t_S_GPST - BDST2GPST(it->toc);
				t_S_GPST.SecOfWeek -= (it->af0 + it->af1 * tk2 + it->af2 * tk2 * tk2);//�����Ӳ�
				SatPosRes->push_back({});
				BDSSatPos(&(*it), t_S_GPST, &SatPosRes->back());//�����źŷ���ʱ������λ��
				i++;
			}
			else
				Obs_Data->SatObs.erase(Obs_Data->SatObs.begin() + i);
			break;
		default:
			break;
		}
	}
}


//���������ڲ�վ����ϵ�µ����꣬������ĸ߶Ƚǣ���λ�ȣ�
double SatelliteAltitudeAngle(XYZUNION* StaSatXyz)
{
	return(Rad2Deg(atan(StaSatXyz->Xyz.Z / sqrt(StaSatXyz->Xyz.X * StaSatXyz->Xyz.X + StaSatXyz->Xyz.Y * StaSatXyz->Xyz.Y))));
}

/***********************************************************************

�������ӳٸ���ģ�͡�H0 ΪΪ�ο���ĸ߶ȣ�T0 ��p0 ��RH0�ֱ�Ϊ�ο����ϵĸ��¡���ѹ�����ʪ�ȣ�
HΪ��վ�߶ȣ�T ��p ��RH �ֱ�Ϊ��վ�ϵĸ��¡���ѹ�����ʪ�ȣ�
EΪ��������ڲ�վ�ĸ߶Ƚ�,!!��λΪ�Ƕ�!!!

***********************************************************************/
double Hopefield(const float T0, const float p0, const float RH0, const double H0, const double H, const double E)
{
	if (abs(H) > 43000.0)
		return 0.0;

	double RH = RH0 * exp(-0.0006396 * (H - H0));
	double p = p0 * pow(1 - 0.0000226 * (H - H0), 5.225);
	double T = T0 - 0.0065*(H - H0);
	double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
	double hw = 11000.0;
	double hd = 40136 + 148.72 * (T0 - 273.16);
	double Kw = 155.2E-7 * 4810 / (T * T) * e * (hw - H);
	double Kd = 155.2E-7 * (p / T) * (hd - H);

	double delta_Trop = Kd / sin(Deg2Rad(sqrt(E * E + 6.25))) + Kw / sin(Deg2Rad(sqrt(E * E + 2.25)));
	return delta_Trop;

}

//�������Ϲ۲�ֵ,�Լ�����һһ��Ӧ������λ�ý���ṹ�壨���������Ӳ���õ����ջ����꣬��λ���Ⱥͽ��ջ��Ӳ����
bool StandardPointPositioning(OBSERVATION_DATA*Obs_Data, vector<SATPOSRES>* SatPosRes, STAPOSRES* StaPosRes)
{
	//��λ����(��λ����ִ�д��������ǵ�������)
	int static Postimes = 0;
	//�������������ֵ������ǵ�һ�ζ�λ�����ֵΪ��0,0,0������������һ�ζ�λ������
	vector<double>XYZT(3, 0);
	XYZT[0] = StaPosRes->StationPos.Xyz.X;
	XYZT[1] = StaPosRes->StationPos.Xyz.Y;
	XYZT[2] = StaPosRes->StationPos.Xyz.Z;
	vector<double>Trop;
	vector<XYZUNION>SatXyz_A;//�����˵�����ת�����������λ��

	for (int IterationTime = 0; IterationTime < 10; IterationTime++)
	{
		//�����վ�Ĵ�����꣬���������������ʱҪ�õ�
		BLH StationPosBlh;
			XYZ_to_BLH(&StaPosRes->StationPos, &StationPosBlh, WGS84_a, WGS84_e2);
			
		int NavSysNum = 0;//����ϵͳ����
		int selector[4] = { 0,0,0,0 };//�жϵ���ϵͳ�������
		for (int i = 0; i < Obs_Data->SatObs.size();)
		{
			//�������ת����ֵ
			double S0 = sqrt(((*SatPosRes)[i].SatXyz.Xyz.X - XYZT[0]) * ((*SatPosRes)[i].SatXyz.Xyz.X - XYZT[0]) + ((*SatPosRes)[i].SatXyz.Xyz.Y - XYZT[1]) * ((*SatPosRes)[i].SatXyz.Xyz.Y - XYZT[1]) + ((*SatPosRes)[i].SatXyz.Xyz.Z - XYZT[2]) * ((*SatPosRes)[i].SatXyz.Xyz.Z - XYZT[2]));
			SatXyz_A.push_back({});
			SatXyz_A[i].Xyz.X = (*SatPosRes)[i].SatXyz.Xyz.X + (GPS_we * S0 / c_Light * (*SatPosRes)[i].SatXyz.Xyz.Y);
			SatXyz_A[i].Xyz.Y = (*SatPosRes)[i].SatXyz.Xyz.Y + (-GPS_we * S0 / c_Light * (*SatPosRes)[i].SatXyz.Xyz.X);
			SatXyz_A[i].Xyz.Z = (*SatPosRes)[i].SatXyz.Xyz.Z;
			//�߶ȽǼ���
			XYZUNION SatStaionXYZ;//���ǵĲ�վ����ϵ����
			EarthXYZ_to_StationXYZ(&SatXyz_A[i], &StaPosRes->StationPos, WGS84_a, WGS84_e2, &SatStaionXYZ);
			double E = SatelliteAltitudeAngle(&SatStaionXYZ);
			//�״ζ�λƫ��ϴ󣬲��Ը߶Ƚǵ͵������޳����˺�ÿ�η��ָ߶Ƚǵ���15�������������޳�
			if (E < 15.0 && Postimes != 0)
			{
				SatPosRes->erase(SatPosRes->begin() + i);
				Obs_Data->SatObs.erase(Obs_Data->SatObs.begin() + i);
				SatXyz_A.erase(SatXyz_A.begin() + i);
				continue;
			}
			//�������ӳټ���
			Trop.push_back(Hopefield(Temperature0, AirPressure0, RelativeHumidity0, SeaLevelHeight, StationPosBlh.H, E));

			//���Ҵ�ʱ�ж��ٸ��ͷֱ����ļ�������ϵͳ
			if (Obs_Data->SatObs[i].Navigation_System == GPS && selector[0] == 0)
				selector[0] = 1;
			else if (Obs_Data->SatObs[i].Navigation_System == BDS && selector[1] == 0)
				selector[1] = 1;
			else if (Obs_Data->SatObs[i].Navigation_System == GLONASS && selector[2] == 0)
				selector[2] = 1;
			else if (Obs_Data->SatObs[i].Navigation_System == GALILEO && selector[3] == 0)
				selector[3] = 1;

			i++;
		}

		//�۲���С�ڴ���δ֪��������
		if (Obs_Data->SatObs.size() < 3 + NavSysNum)
		{
			StaPosRes->Pos_State = NOPOS;
			return false;
		}

		NavSysNum = selector[0] + selector[1] + selector[2] + selector[3];
		XYZT.resize(3 + NavSysNum);
		double* B = new double[Obs_Data->SatObs.size() * (3 + NavSysNum)]();
		double* l = new double[Obs_Data->SatObs.size()]();
		for (int i = 0; i < Obs_Data->SatObs.size(); i++)
		{
			//B�����ǰ���еĸ�ֵ
			double S0 = sqrt((SatXyz_A[i].Xyz.X - XYZT[0]) * (SatXyz_A[i].Xyz.X - XYZT[0]) + (SatXyz_A[i].Xyz.Y - XYZT[1]) * (SatXyz_A[i].Xyz.Y - XYZT[1]) + (SatXyz_A[i].Xyz.Z - XYZT[2]) * (SatXyz_A[i].Xyz.Z - XYZT[2]));
			B[i * (3 + NavSysNum)] = (XYZT[0] - SatXyz_A[i].Xyz.X) / S0;
			B[i * (3 + NavSysNum) + 1] = (XYZT[1] - SatXyz_A[i].Xyz.Y) / S0;
			B[i * (3 + NavSysNum) + 2] = (XYZT[2] - SatXyz_A[i].Xyz.Z) / S0;
			//B������ݵ���ϵͳ�ڶ�Ӧλ�ø�1������λ���ڳ�ʼ��ʱ�Ѿ���0.�Ӳ������B�����е�ǰ��˳��ΪGPS>BDS>GLONASS>GALILEO���п�ȱ����ǰ��λ
			//˳��ȷ��Ҫ���ĸ�����ϵͳ�Ľ��ջ��Ӳ��ֵ
			double cdelta_tR = 0.0;//��λ��
			switch (Obs_Data->SatObs[i].Navigation_System)
			{
			case GPS:
				B[i * (3 + NavSysNum) + 2 + selector[0]] = 1.0;
				cdelta_tR = XYZT[2 + selector[0]];
				break;
			case BDS:
				B[i * (3 + NavSysNum) + 2 + selector[0] + selector[1]] = 1.0;
				cdelta_tR = XYZT[2 + selector[0] + selector[1]];
				break;
			case GLONASS:
				B[i * (3 + NavSysNum) + 2 + selector[0] + selector[1] + selector[2]] = 1.0;
				cdelta_tR = XYZT[2 + selector[0] + selector[1] + selector[2]];
				break;
			case GALILEO:
				B[i * (3 + NavSysNum) + 2 + selector[0] + selector[1] + selector[2] + selector[3]] = 1.0;
				cdelta_tR = XYZT[2 + selector[0] + selector[1] + selector[2] + selector[3]];
				break;
			}

			l[i] = Obs_Data->SatObs[i].P_if - (S0 + cdelta_tR - c_Light * (*SatPosRes)[i].delta_tsv + Trop[i]);
		}
		//������λȨ��P
		double* P = new double[Obs_Data->SatObs.size() * Obs_Data->SatObs.size()];
		CreateE(Obs_Data->SatObs.size(), P);
		//���������������x ��x=X-X0��
		double* x = new double[XYZT.size()];
		//��С�������
		LeastSquaresEstimation(Obs_Data->SatObs.size(), XYZT.size(), B, P, l, x);
		//��������
		StaPosRes->Pos_sigma0 = Sigma0(Obs_Data->SatObs.size(), XYZT.size(), B, P, l, x);

		CalculateQxx(Obs_Data->SatObs.size(), XYZT.size(), B, P, StaPosRes->Qxx);
		StaPosRes->PDOP = sqrt(StaPosRes->Qxx[0] + StaPosRes->Qxx[XYZT.size() + 1] + StaPosRes->Qxx[2 * XYZT.size() + 2]);

		//���´������
		for (int i = 0; i < XYZT.size(); i++)
			XYZT[i] += x[i];
		
		StaPosRes->StationPos.Xyz.X = XYZT[0];
		StaPosRes->StationPos.Xyz.Y = XYZT[1];
		StaPosRes->StationPos.Xyz.Z = XYZT[2];
		XYZ_to_BLH(&StaPosRes->StationPos, &StaPosRes->StationPosBlh, WGS84_a, WGS84_e2);

		delete[]B;
		delete[]P;
		delete[]l;

		if (fabs(x[0]) < 1.0E-3 && fabs(x[1]) < 1.0E-3 && fabs(x[2]) < 1.0E-3 && fabs(x[3]) < 1.0E-3)
		{
			
			delete[]x;
			//�����ĸ�ϵͳ�Ӳ�������
			for (int i = 3; i < XYZT.size() ; i++)
				StaPosRes->ct_R[i - 3] = XYZT[i];
			//�����һ�ε�����ת�����������λ�����
			for (int i = 0; i < SatPosRes->size(); i++)
				(*SatPosRes)[i].SatXyz = SatXyz_A[i];
			Postimes++;
			StaPosRes->Pos_State = SPP;
			return true;
		}
		delete[]x;
		SatXyz_A.clear();
		Trop.clear();
	}
	Postimes++;
	StaPosRes->Pos_State = NOPOS;
	return false;
}

bool StandardPointVelocity(OBSERVATION_DATA* Obs_Data, vector<SATPOSRES>* SatPosRes, STAPOSRES* StaPosRes)
{
	if (Obs_Data->SatObs.size() < 4)
		return false;

	double* B = new double[Obs_Data->SatObs.size() * 4]();
	double* l = new double[Obs_Data->SatObs.size()]();
	double x[4];

	for (int i = 0; i < Obs_Data->SatObs.size(); i++)
	{
		//���ξ���
		double S0 = sqrt(((*SatPosRes)[i].SatXyz.Xyz.X - StaPosRes->StationPos.Xyz.X) * ((*SatPosRes)[i].SatXyz.Xyz.X - StaPosRes->StationPos.Xyz.X) + ((*SatPosRes)[i].SatXyz.Xyz.Y - StaPosRes->StationPos.Xyz.Y) * ((*SatPosRes)[i].SatXyz.Xyz.Y - StaPosRes->StationPos.Xyz.Y) + ((*SatPosRes)[i].SatXyz.Xyz.Z - StaPosRes->StationPos.Xyz.Z) * ((*SatPosRes)[i].SatXyz.Xyz.Z - StaPosRes->StationPos.Xyz.Z));
		//���ξ�����
		double dot_S0 = (((*SatPosRes)[i].SatXyz.Xyz.X - StaPosRes->StationPos.Xyz.X) * (*SatPosRes)[i].SatVel[0] + ((*SatPosRes)[i].SatXyz.Xyz.Y - StaPosRes->StationPos.Xyz.Y) * (*SatPosRes)[i].SatVel[1] + ((*SatPosRes)[i].SatXyz.Xyz.Z - StaPosRes->StationPos.Xyz.Z) * (*SatPosRes)[i].SatVel[2]) / S0;

		B[i * 4] = (StaPosRes->StationPos.Xyz.X - (*SatPosRes)[i].SatXyz.Xyz.X) / S0;
		B[i * 4 + 1] = (StaPosRes->StationPos.Xyz.Y - (*SatPosRes)[i].SatXyz.Xyz.Y) / S0;
		B[i * 4 + 2] = (StaPosRes->StationPos.Xyz.Z - (*SatPosRes)[i].SatXyz.Xyz.Z) / S0;
		B[i * 4 + 3] = 1.0;
		double dopp;
		switch (Obs_Data->SatObs[i].Navigation_System)
		{
		case GPS:dopp = double(Obs_Data->SatObs[i].dopp[1]) * WaveLen_L2;
			break;
		case BDS:dopp = double(Obs_Data->SatObs[i].dopp[1]) * WaveLen_B3I;
			break;
		}
		l[i] = dopp - (dot_S0 - c_Light * (*SatPosRes)[i].RateOf_delta_tsv);

	}
	//������λȨ��P
	double* P = new double[Obs_Data->SatObs.size() * Obs_Data->SatObs.size()];
	CreateE(Obs_Data->SatObs.size(), P);
	//��С�������
	LeastSquaresEstimation(Obs_Data->SatObs.size(), 4, B, P, l, x);
	//��������
	
	StaPosRes->Vel_sigma0 = Sigma0(Obs_Data->SatObs.size(), 4, B, P, l, x);
	//������ٽ��
	StaPosRes->StationVel[0] = x[0];
	StaPosRes->StationVel[1] = x[1];
	StaPosRes->StationVel[2] = x[2];
	StaPosRes->RateOfct_R = x[3];

	delete[]B;
	delete[]P;
	delete[]l;
	return true;
}


void StaPosRes_Show(GPSTIME* GpsTime, STAPOSRES* StaPosRes, int SatNum)
{
	printf("SPP: %d %10.3f  X:%16.6f  Y:%16.6f  Z:%16.6f  B:%12.8f  L:%12.8f  H:%7.3f Vx:%8.5f Vy:%8.5f Vz:%8.5f GPS Clk:%8.4f BDS Clk:%8.4f PosSigma0=%6.3f PDOP:%6.3f  VelSigma0=%7.4f SatNum:%02d", GpsTime->Week, GpsTime->SecOfWeek, StaPosRes->StationPos.Xyz.X, StaPosRes->StationPos.Xyz.Y, StaPosRes->StationPos.Xyz.Z, StaPosRes->StationPosBlh.B, StaPosRes->StationPosBlh.L, StaPosRes->StationPosBlh.H, StaPosRes->StationVel[0], StaPosRes->StationVel[1], StaPosRes->StationVel[2], StaPosRes->ct_R[0], StaPosRes->ct_R[1], StaPosRes->Pos_sigma0, StaPosRes->PDOP, StaPosRes->Vel_sigma0, SatNum);
	if (StaPosRes->Pos_State == SPP)printf("  True\n\n");
	else printf("  False\n\n");
}

//
//void StaPosRes_OutPutToFile(FILE* fp, GPSTIME* GpsTime, STAPOSRES* StaPosRes, int SatNum)
//{
//	fprintf(fp, "SPP: %d %10.3f  X:%16.6f  Y:%16.6f  Z:%16.6f  B:%12.8f  L:%12.8f  H:%7.3f Vx:%8.5f Vy:%8.5f Vz:%8.5f GPS Clk:%8.4f BDS Clk:%8.4f PosSigma0=%6.3f PDOP:%6.3f  VelSigma0=%7.4f SatNum:%02d", GpsTime->Week, GpsTime->SecOfWeek, StaPosRes->StationPos.Xyz.X, StaPosRes->StationPos.Xyz.Y, StaPosRes->StationPos.Xyz.Z, StaPosRes->StationPosBlh.B, StaPosRes->StationPosBlh.L, StaPosRes->StationPosBlh.H, StaPosRes->StationVel[0], StaPosRes->StationVel[1], StaPosRes->StationVel[2], StaPosRes->ct_R[0], StaPosRes->ct_R[1], StaPosRes->Pos_sigma0, StaPosRes->PDOP, StaPosRes->Vel_sigma0, SatNum);
//	if (StaPosRes->PositioningSuccess)fprintf(fp,"  True\n\n");
//	else fprintf(fp,"  False\n\n");
//}

//void StaPosResWithSat_Show(OBSERVATION_DATA*Obs_Data, STAPOSRES* StaPosRes, vector<SATPOSRES>*SatPosRes, ACCURACY* PosAccuracy, ACCURACY* VelAccuracy, bool PositioningSuccess)
//{
//	for (int i = 0; i < SatPosRes->size(); i++)
//	{
//		switch ((*SatPosRes)[i].Navigation_System)
//		{
//		case GPS:printf("GPS");
//			break;
//		case BDS:printf("BDS");
//			break;
//		}
//		printf("%02d  X=%16.4f Y=%16.4f Z=%16.4f Clk=%13.6E Vx=%12.5f Vy=%12.5f Vz=%12.5f Clkd=%13.6E PIF=%16.4f \n", Obs_Data->SatObs[i].prn, (*SatPosRes)[i].SatXyz.Xyz.X, (*SatPosRes)[i].SatXyz.Xyz.Y, (*SatPosRes)[i].SatXyz.Xyz.Z, (*SatPosRes)[i].delta_tsv, (*SatPosRes)[i].SatVel[0], (*SatPosRes)[i].SatVel[1], (*SatPosRes)[i].SatVel[2], (*SatPosRes)[i].RateOf_delta_tsv, Obs_Data->SatObs[i].P_if);
//	}
//	printf("SPP: %d %10.3f  X:%16.6f  Y:%16.6f  Z:%16.6f  B:%12.8f  L:%12.8f  H:%7.3f Vx:%8.5f Vy:%8.5f Vz:%8.5f GPS Clk:%8.4f BDS Clk:%8.4f PosSigma0=%6.3f PDOP:%6.3f  VelSigma0=%7.4f SatNum:%02d", Obs_Data->GpsTime.Week, Obs_Data->GpsTime.SecOfWeek, StaPosRes->StationPos.Xyz.X, StaPosRes->StationPos.Xyz.Y, StaPosRes->StationPos.Xyz.Z, StaPosRes->StationPosBlh.B, StaPosRes->StationPosBlh.L, StaPosRes->StationPosBlh.H, StaPosRes->StationVel[0], StaPosRes->StationVel[1], StaPosRes->StationVel[2], StaPosRes->ct_R[0], StaPosRes->ct_R[1], PosAccuracy->sigma0, PosAccuracy->PDOP, VelAccuracy->sigma0, SatPosRes->size());
//	
//	if (PositioningSuccess)printf("  True\n\n");
//	else printf("  False\n\n");
//}
//
//
//void StaPosResWithSat_OutPutToFile(FILE* fp, OBSERVATION_DATA* Obs_Data, STAPOSRES* StaPosRes, vector<SATPOSRES>* SatPosRes, ACCURACY* PosAccuracy, ACCURACY* VelAccuracy, bool PositioningSuccess)
//{
//	for (int i = 0; i < SatPosRes->size(); i++)
//	{
//		switch ((*SatPosRes)[i].Navigation_System)
//		{
//		case GPS:fprintf(fp,"GPS");
//			break;
//		case BDS:fprintf(fp,"BDS");
//			break;
//		}
//		fprintf(fp,"%02d  X=%16.4f Y=%16.4f Z=%16.4f Clk=%13.6E Vx=%12.5f Vy=%12.5f Vz=%12.5f Clkd=%13.6E PIF=%16.4f \n", Obs_Data->SatObs[i].prn, (*SatPosRes)[i].SatXyz.Xyz.X, (*SatPosRes)[i].SatXyz.Xyz.Y, (*SatPosRes)[i].SatXyz.Xyz.Z, (*SatPosRes)[i].delta_tsv, (*SatPosRes)[i].SatVel[0], (*SatPosRes)[i].SatVel[1], (*SatPosRes)[i].SatVel[2], (*SatPosRes)[i].RateOf_delta_tsv, Obs_Data->SatObs[i].P_if);
//	}
//	fprintf(fp,"SPP: %d %10.3f  X:%16.6f  Y:%16.6f  Z:%16.6f  B:%12.8f  L:%12.8f  H:%7.3f Vx:%8.5f Vy:%8.5f Vz:%8.5f GPS Clk:%8.4f BDS Clk:%8.4f PosSigma0=%6.3f PDOP:%6.3f  VelSigma0=%7.4f SatNum:%02d", Obs_Data->GpsTime.Week, Obs_Data->GpsTime.SecOfWeek, StaPosRes->StationPos.Xyz.X, StaPosRes->StationPos.Xyz.Y, StaPosRes->StationPos.Xyz.Z, StaPosRes->StationPosBlh.B, StaPosRes->StationPosBlh.L, StaPosRes->StationPosBlh.H, StaPosRes->StationVel[0], StaPosRes->StationVel[1], StaPosRes->StationVel[2], StaPosRes->ct_R[0], StaPosRes->ct_R[1], PosAccuracy->sigma0, PosAccuracy->PDOP, VelAccuracy->sigma0, SatPosRes->size());
//	if (PositioningSuccess)fprintf(fp, "  True\n\n");
//	else fprintf(fp, "  False\n\n");
//}
