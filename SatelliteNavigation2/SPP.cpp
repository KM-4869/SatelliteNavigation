#include"SPP.h"

//传进观测数据结构体，输出粗差剔除后的结果
void DetectOutlier(OBSERVATION_DATA* Obs_Data)
{

	//开始粗差探测前先将保存的卫星数据的更新状态置为false
	for (int k = 0; k < Obs_Data->History_Sat.size(); k++)
		Obs_Data->History_Sat[k].update = false;

	for (int i = 0; i < Obs_Data->SatObs.size();)
	{
		//检查每颗卫星的双频伪距和相位数据是否有效和完整(同时淘汰掉非GPS或北斗的卫星)
		if (abs(Obs_Data->SatObs[i].psr[0]) < 1.0E-5 || abs(Obs_Data->SatObs[i].psr[1]) < 1.0E-5 || abs(Obs_Data->SatObs[i].adr[0]) < 1.0E-5 || abs(Obs_Data->SatObs[i].adr[1]) < 1.0E-5)
		{
			Obs_Data->SatObs.erase(Obs_Data->SatObs.begin() + i);
			continue;
		}
		//通过此步骤的卫星都将为其计算MW与GF的值

		//计算并保存每颗卫星的GF和MW组合
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
			MW = 1 / (f_L1 - f_L2) * (f_L1 * L1 - f_L2 * L2) - 1 / (f_L1 + f_L2) * (f_L1 * P1 + f_L2 * P2);//单位米
			break;
		case BDS:
			L1 = Obs_Data->SatObs[i].adr[0] * WaveLen_B1I;
			L2 = Obs_Data->SatObs[i].adr[1] * WaveLen_B3I;
			L_gf = L1 - L2;
			MW = 1 / (f_B1I - f_B3I) * (f_B1I * L1 - f_B3I * L2) - 1 / (f_B1I + f_B3I) * (f_B1I * P1 + f_B3I * P2);//单位米
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

				//计算当前历元每颗卫星GF与上一历元对应GF的差值
				  double dGF = L_gf - Obs_Data->History_Sat[j].Last_L_gf;
				//计算当前历元每颗卫星MW与上一历元对应MW平滑值的差值
				  double dMW = MW - Obs_Data->History_Sat[j].Average_MW;

				//检查dGF和dMW是否超限
				if (abs(dGF) < 0.05 && abs(dMW) < 2.0)
				{
					//对于可用的观测数据，计算伪距的IF组合观测值，用于SPP
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
					//如果有粗差，则将这颗卫星从历史数据中删除，下一次出现时它将被当做新卫星加到数组末尾
					Obs_Data->SatObs.erase(Obs_Data->SatObs.begin() + i);
					Obs_Data->History_Sat.erase(Obs_Data->History_Sat.begin() + j);
					gross_error = true;
				}
				break;
			}
		}
		//计算此卫星的MW组合的平滑值,赋予当前历元的GF值并标识卫星数据已更新
		//如果因为有粗差而被删除的卫星，则不再更新，跳到下一个卫星的查找
		if (gross_error == true)continue;
		
		if (find == false)
		{
			//没找到说明是首次出现的卫星，赋予其prn号以及导航系统，并将最大卫星数加一。如果找到则略去赋prn号与导航系统这一步。
			//由于首次出现，不能判断是否有粗差，所以也要删掉
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

	//遍历已经完成更新保存的本历元数据（相位gf组合，平滑MW值），如果发现保存中数据有未被更新过的，说明此卫星在本历元有观测值缺失或者根本没有出现过
	//那么它下一次出现时，里面保存的数据可能将是n个历元之前的
	//为了防止这种现象出现，当一颗卫星的历史数据未被更新时，它将被从历史库中删除
	//下一次出现时，它将被当做新卫星加入数组末尾。
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
	double n0 = sqrt(GPS_GM) / (sqrt(GPSEphem->a) * sqrt(GPSEphem->a) * sqrt(GPSEphem->a));      //卫星运动平均角速度
	double tk1 = t - GPSEphem->toe;//计算t相对于参考历元的时间
	double tk2 = t - GPSEphem->toc;//计算t相对于toc的时间

	double n = n0 + GPSEphem->delta_n;           //加上摄动参数，得观测时刻平均角速度
	double M = GPSEphem->MeanAnomaly + n * tk1;   //观测时刻平近点角

	double E = M;      //求解偏近点角并进行迭代
	for (int i = 1; i < 20; i++)
	{
		double E0 = E;
		E = M + GPSEphem->ecc * sin(E);
		
		if (abs(E - E0) < 5.0E-10)
		{
			break;
		}
	}

	double f = atan2((sqrt(1 - GPSEphem->ecc * GPSEphem->ecc) * sin(E)), (cos(E) - GPSEphem->ecc));//计算真近点角
	
	double u0 = GPSEphem->ArgumentPerigee + f;      //计算升交角距

	//计算改正摄动项
	double delta_u = GPSEphem->cuc * cos(2 * u0) + GPSEphem->cus * sin(2 * u0);
	double delta_r = GPSEphem->crc * cos(2 * u0) + GPSEphem->crs * sin(2 * u0);
	double delta_i = GPSEphem->cic * cos(2 * u0) + GPSEphem->cis * sin(2 * u0);

	//摄动改正
	double u = u0 + delta_u;
	double r = GPSEphem->a * (1 - GPSEphem->ecc * cos(E)) + delta_r;//r0=a(1-ecosE)
	double i = GPSEphem->I0 + delta_i + GPSEphem->RateOfI * tk1;

	//计算卫星在平面上的位置
	double x = r * cos(u);
	double y = r * sin(u);

	//计算升交点的经度
	double L = GPSEphem->RightAscension + (GPSEphem->RateOfRightAscension - GPS_we) * tk1 - GPS_we * GPSEphem->toe.SecOfWeek;
	//计算地球坐标系下坐标
	SatPosRes->SatXyz.Xyz.X = x * cos(L) - y * cos(i) * sin(L);
	SatPosRes->SatXyz.Xyz.Y = x * sin(L) + y * cos(i) * cos(L);
	SatPosRes->SatXyz.Xyz.Z = y * sin(i);

	//卫星钟差改正
	double delta_tr = F_Relativity * GPSEphem->ecc * sqrt(GPSEphem->a) * sin(E);//相对论效应改正
	SatPosRes->delta_tsv = GPSEphem->af0 + GPSEphem->af1 * tk2 + GPSEphem->af2 * tk2 * tk2 + delta_tr;

	//卫星钟速改正
	double RateOfE = n / (1 - GPSEphem->ecc * cos(E));
	double RateOfdelta_tr = F_Relativity * GPSEphem->ecc * sqrt(GPSEphem->a) * cos(E) * RateOfE;
	SatPosRes->RateOf_delta_tsv = GPSEphem->af1 + 2 * GPSEphem->af2 * tk2 + RateOfdelta_tr;


	//卫星速度计算
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
	double n0 = sqrt(BDS_GM) / (BDSEphem->a * BDSEphem->a * BDSEphem->a);      //卫星运动平均角速度
	double tk1 = t - BDSEphem->toe;//计算t相对于参考历元的时间
	double tk2 = t - BDSEphem->toc;//计算t相对于toc的时间

	double n = n0 + BDSEphem->delta_n;           //加上摄动参数，得观测时刻平均角速度
	double M = BDSEphem->MeanAnomaly + n * tk1;   //观测时刻平近点角

	double E = M;      //求解偏近点角并进行迭代
	for (int i = 1; i < 20; i++)
	{
		double E0 = E;
		E = M + BDSEphem->ecc * sin(E);

		if (abs(E - E0) < 5.0E-10)
		{
			break;
		}
	}

	double f = atan2((sqrt(1 - BDSEphem->ecc * BDSEphem->ecc) * sin(E)), (cos(E) - BDSEphem->ecc));//计算真近点角

	double u0 = BDSEphem->ArgumentPerigee + f;      //计算升交角距

	//计算改正摄动项
	double delta_u = BDSEphem->cuc * cos(2 * u0) + BDSEphem->cus * sin(2 * u0);
	double delta_r = BDSEphem->crc * cos(2 * u0) + BDSEphem->crs * sin(2 * u0);
	double delta_i = BDSEphem->cic * cos(2 * u0) + BDSEphem->cis * sin(2 * u0);

	//摄动改正
	double u = u0 + delta_u;
	double r = BDSEphem->a * BDSEphem->a * (1 - BDSEphem->ecc * cos(E)) + delta_r;//r0=a(1-ecosE)
	double i = BDSEphem->I0 + delta_i + BDSEphem->RateOfI * tk1;

	//计算卫星在平面上的位置
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

	//卫星钟差改正
	double delta_tr = F_Relativity * BDSEphem->ecc * BDSEphem->a * sin(E);//相对论效应改正
	SatPosRes->delta_tsv = BDSEphem->af0 + BDSEphem->af1 * tk2 + BDSEphem->af2 * tk2 * tk2 + delta_tr;

	//卫星钟速改正(注意北斗的a为半长轴平方根，下同)
	double RateOfE = n / (1 -BDSEphem->ecc * cos(E));
	double RateOfdelta_tr = F_Relativity * BDSEphem->ecc * BDSEphem->a * cos(E) * RateOfE;
	SatPosRes->RateOf_delta_tsv = BDSEphem->af1 + 2 * BDSEphem->af2 * tk2 + RateOfdelta_tr;


	//卫星速度计算
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

		double dot_XYZ_GK[3];//GEO在特定坐标系下的速度

		MatrixMultiply(3, 4, 4, 1, dot_R, xyOI, dot_XYZ_GK);

		double deg_5 = Deg2Rad(-5);//-5°的弧度

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

//对北斗B1IB3I的双频if组合做TGD改正
void BDS_Sat_P_if_TGDCorrection(double*P_if,double*tgd1)
{
	*P_if += c_Light * f_B1I * f_B1I * (*tgd1) / (f_B3I * f_B3I - f_B1I * f_B1I);
}

//对于观测数据中的卫星，在两组星历中查找，找到则为其计算一个信号发射时刻卫星位置并输出，找不到则把这个卫星数据删掉。
void SignalSendTimeSatPos(OBSERVATION_DATA* Obs_Data, vector<EPHEMERIS>* GPSEphem, vector<EPHEMERIS>* BDSEphem, vector<SATPOSRES>* SatPosRes)
{
	for (int i = 0; i < Obs_Data->SatObs.size();)
	{
		vector<EPHEMERIS>::iterator it;
		GPSTIME t_S_GPST;//信号发射时刻的GPS标准时间
		double tk2;//卫星钟表面时与toc之差

		//在星历中查找观测数据中的卫星
		switch (Obs_Data->SatObs[i].Navigation_System)
		{
		case GPS:
			it = find(GPSEphem->begin(), GPSEphem->end(), Obs_Data->SatObs[i].prn);
			if (it != GPSEphem->end())
			{
				t_S_GPST.Week = Obs_Data->GpsTime.Week;
				t_S_GPST.SecOfWeek = Obs_Data->GpsTime.SecOfWeek - Obs_Data->SatObs[i].P_if / c_Light;//卫星钟表面时
				tk2 = t_S_GPST-it->toc;
				t_S_GPST.SecOfWeek -= (it->af0 + it->af1 * tk2 + it->af2 * tk2 * tk2);//计算钟差
				SatPosRes->push_back({});
				GPSSatPos(&(*it), t_S_GPST, &SatPosRes->back());//计算信号发射时刻卫星位置
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
				t_S_GPST.SecOfWeek = Obs_Data->GpsTime.SecOfWeek - Obs_Data->SatObs[i].P_if / c_Light;//卫星钟表面时
				tk2 = t_S_GPST - BDST2GPST(it->toc);
				t_S_GPST.SecOfWeek -= (it->af0 + it->af1 * tk2 + it->af2 * tk2 * tk2);//计算钟差
				SatPosRes->push_back({});
				BDSSatPos(&(*it), t_S_GPST, &SatPosRes->back());//计算信号发射时刻卫星位置
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


//输入卫星在测站坐标系下的坐标，输出它的高度角（单位度）
double SatelliteAltitudeAngle(XYZUNION* StaSatXyz)
{
	return(Rad2Deg(atan(StaSatXyz->Xyz.Z / sqrt(StaSatXyz->Xyz.X * StaSatXyz->Xyz.X + StaSatXyz->Xyz.Y * StaSatXyz->Xyz.Y))));
}

/***********************************************************************

对流层延迟改正模型。H0 为为参考面的高度，T0 、p0 和RH0分别为参考面上的干温、气压和相对湿度；
H为测站高度，T 、p 和RH 分别为测站上的干温、气压和相对湿度；
E为卫星相对于测站的高度角,!!单位为角度!!!

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

//输入各组合观测值,以及与其一一对应的卫星位置结果结构体（包含卫星钟差），得到接收机坐标，定位精度和接收机钟差参数
bool StandardPointPositioning(OBSERVATION_DATA*Obs_Data, vector<SATPOSRES>* SatPosRes, STAPOSRES* StaPosRes)
{
	//定位次数(定位函数执行次数，而非迭代次数)
	int static Postimes = 0;
	//给坐标参数赋初值，如果是第一次定位，则初值为（0,0,0），否则是上一次定位的坐标
	vector<double>XYZT(3, 0);
	XYZT[0] = StaPosRes->StationPos.Xyz.X;
	XYZT[1] = StaPosRes->StationPos.Xyz.Y;
	XYZT[2] = StaPosRes->StationPos.Xyz.Z;
	vector<double>Trop;
	vector<XYZUNION>SatXyz_A;//加上了地球自转改正后的卫星位置

	for (int IterationTime = 0; IterationTime < 10; IterationTime++)
	{
		//计算测站的大地坐标，后面做对流层改正时要用到
		BLH StationPosBlh;
			XYZ_to_BLH(&StaPosRes->StationPos, &StationPosBlh, WGS84_a, WGS84_e2);
			
		int NavSysNum = 0;//导航系统个数
		int selector[4] = { 0,0,0,0 };//判断导航系统存在情况
		for (int i = 0; i < Obs_Data->SatObs.size();)
		{
			//求地球自转改正值
			double S0 = sqrt(((*SatPosRes)[i].SatXyz.Xyz.X - XYZT[0]) * ((*SatPosRes)[i].SatXyz.Xyz.X - XYZT[0]) + ((*SatPosRes)[i].SatXyz.Xyz.Y - XYZT[1]) * ((*SatPosRes)[i].SatXyz.Xyz.Y - XYZT[1]) + ((*SatPosRes)[i].SatXyz.Xyz.Z - XYZT[2]) * ((*SatPosRes)[i].SatXyz.Xyz.Z - XYZT[2]));
			SatXyz_A.push_back({});
			SatXyz_A[i].Xyz.X = (*SatPosRes)[i].SatXyz.Xyz.X + (GPS_we * S0 / c_Light * (*SatPosRes)[i].SatXyz.Xyz.Y);
			SatXyz_A[i].Xyz.Y = (*SatPosRes)[i].SatXyz.Xyz.Y + (-GPS_we * S0 / c_Light * (*SatPosRes)[i].SatXyz.Xyz.X);
			SatXyz_A[i].Xyz.Z = (*SatPosRes)[i].SatXyz.Xyz.Z;
			//高度角计算
			XYZUNION SatStaionXYZ;//卫星的测站坐标系坐标
			EarthXYZ_to_StationXYZ(&SatXyz_A[i], &StaPosRes->StationPos, WGS84_a, WGS84_e2, &SatStaionXYZ);
			double E = SatelliteAltitudeAngle(&SatStaionXYZ);
			//首次定位偏差较大，不对高度角低的卫星剔除。此后每次发现高度角低于15°的卫星则进行剔除
			if (E < 15.0 && Postimes != 0)
			{
				SatPosRes->erase(SatPosRes->begin() + i);
				Obs_Data->SatObs.erase(Obs_Data->SatObs.begin() + i);
				SatXyz_A.erase(SatXyz_A.begin() + i);
				continue;
			}
			//对流层延迟计算
			Trop.push_back(Hopefield(Temperature0, AirPressure0, RelativeHumidity0, SeaLevelHeight, StationPosBlh.H, E));

			//查找此时有多少个和分别是哪几个卫星系统
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

		//观测数小于待求未知数，返回
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
			//B矩阵的前三列的赋值
			double S0 = sqrt((SatXyz_A[i].Xyz.X - XYZT[0]) * (SatXyz_A[i].Xyz.X - XYZT[0]) + (SatXyz_A[i].Xyz.Y - XYZT[1]) * (SatXyz_A[i].Xyz.Y - XYZT[1]) + (SatXyz_A[i].Xyz.Z - XYZT[2]) * (SatXyz_A[i].Xyz.Z - XYZT[2]));
			B[i * (3 + NavSysNum)] = (XYZT[0] - SatXyz_A[i].Xyz.X) / S0;
			B[i * (3 + NavSysNum) + 1] = (XYZT[1] - SatXyz_A[i].Xyz.Y) / S0;
			B[i * (3 + NavSysNum) + 2] = (XYZT[2] - SatXyz_A[i].Xyz.Z) / S0;
			//B矩阵根据导航系统在对应位置赋1，其余位置在初始化时已经赋0.钟差参数在B矩阵中的前后顺序为GPS>BDS>GLONASS>GALILEO，有空缺则向前补位
			//顺便确定要减哪个导航系统的接收机钟差初值
			double cdelta_tR = 0.0;//单位米
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
		//创建单位权阵P
		double* P = new double[Obs_Data->SatObs.size() * Obs_Data->SatObs.size()];
		CreateE(Obs_Data->SatObs.size(), P);
		//创建待求参数矩阵x （x=X-X0）
		double* x = new double[XYZT.size()];
		//最小二乘求解
		LeastSquaresEstimation(Obs_Data->SatObs.size(), XYZT.size(), B, P, l, x);
		//精度评定
		StaPosRes->Pos_sigma0 = Sigma0(Obs_Data->SatObs.size(), XYZT.size(), B, P, l, x);

		CalculateQxx(Obs_Data->SatObs.size(), XYZT.size(), B, P, StaPosRes->Qxx);
		StaPosRes->PDOP = sqrt(StaPosRes->Qxx[0] + StaPosRes->Qxx[XYZT.size() + 1] + StaPosRes->Qxx[2 * XYZT.size() + 2]);

		//更新待求参数
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
			//把求解的各系统钟差参数输出
			for (int i = 3; i < XYZT.size() ; i++)
				StaPosRes->ct_R[i - 3] = XYZT[i];
			//把最后一次地球自转改正后的卫星位置输出
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
		//几何距离
		double S0 = sqrt(((*SatPosRes)[i].SatXyz.Xyz.X - StaPosRes->StationPos.Xyz.X) * ((*SatPosRes)[i].SatXyz.Xyz.X - StaPosRes->StationPos.Xyz.X) + ((*SatPosRes)[i].SatXyz.Xyz.Y - StaPosRes->StationPos.Xyz.Y) * ((*SatPosRes)[i].SatXyz.Xyz.Y - StaPosRes->StationPos.Xyz.Y) + ((*SatPosRes)[i].SatXyz.Xyz.Z - StaPosRes->StationPos.Xyz.Z) * ((*SatPosRes)[i].SatXyz.Xyz.Z - StaPosRes->StationPos.Xyz.Z));
		//几何距离求导
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
	//创建单位权阵P
	double* P = new double[Obs_Data->SatObs.size() * Obs_Data->SatObs.size()];
	CreateE(Obs_Data->SatObs.size(), P);
	//最小二乘求解
	LeastSquaresEstimation(Obs_Data->SatObs.size(), 4, B, P, l, x);
	//精度评定
	
	StaPosRes->Vel_sigma0 = Sigma0(Obs_Data->SatObs.size(), 4, B, P, l, x);
	//输出测速结果
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
