#include"RTK.h"

void SingleDifference(OBSERVATION_DATA* Rover_ObsData, OBSERVATION_DATA* Base_ObsData, SD_OBSERVATION_DATA* Sd_ObsData)
{
	if (Rover_ObsData->SatObs.size() == 0 || Base_ObsData->SatObs.size() == 0)
		return;

	for (auto &RoverSatObs : Rover_ObsData->SatObs)
	{
		//流动站观测值存在半周，跳过不算单差
		if (RoverSatObs.ParityKnownFlag == false)
			continue;

		//找对应的基站观测值计算单差，存在半周或者找不到则不算单差
		vector<SATELLITE_OBSERVATION>::iterator BaseSatObs;
	
		BaseSatObs = find(Base_ObsData->SatObs.begin(), Base_ObsData->SatObs.end(), RoverSatObs);
		if (BaseSatObs != Base_ObsData->SatObs.end() && BaseSatObs->ParityKnownFlag == true)
		{
			Sd_ObsData->SdSatObs.push_back({});
			Sd_ObsData->GpsTime = Rover_ObsData->GpsTime;
			Sd_ObsData->SdSatObs.back().Navigation_System = RoverSatObs.Navigation_System;
			Sd_ObsData->SdSatObs.back().prn = RoverSatObs.prn;
			for (int k = 0; k < 2; k++)
			{
				Sd_ObsData->SdSatObs.back().psr[k] = RoverSatObs.psr[k] - BaseSatObs->psr[k];
				Sd_ObsData->SdSatObs.back().adr[k] = RoverSatObs.adr[k] - BaseSatObs->adr[k];
				Sd_ObsData->SdSatObs.back().CNo[k] = (RoverSatObs.CNo[k] + BaseSatObs->CNo[k]) / 2.0;//用流动站卫星观测值CNo和基站卫星观测值的CNo的平均（事实上二者也相差不大）
				Sd_ObsData->SdSatObs.back().locktime[k] = (RoverSatObs.locktime[k] + BaseSatObs->locktime[k]) / 2.0;//同理，计算平均锁定时长，二者均为判断参考星依据	
			}
			Sd_ObsData->Sat_Number++;
		}
	}

	return;
}

//传进单差观测数据结构体，输出粗差剔除后的结果
void SDDetectOutlier(SD_OBSERVATION_DATA* Obs_Data)
{

	//开始粗差探测前先将保存的卫星数据的更新状态置为false
	for (int k = 0; k < Obs_Data->History_Sat.size(); k++)
		Obs_Data->History_Sat[k].update = false;

	for (int i = 0; i < Obs_Data->SdSatObs.size();)
	{

		//计算并保存每颗卫星的GF和MW组合
		double L_gf = 0.0;
		double MW = 0.0;
		double L1 = 0.0, L2 = 0.0;
		double P1 = Obs_Data->SdSatObs[i].psr[0];
		double P2 = Obs_Data->SdSatObs[i].psr[1];

		switch (Obs_Data->SdSatObs[i].Navigation_System)
		{
		case GPS:
			L1 = Obs_Data->SdSatObs[i].adr[0] * WaveLen_L1;
			L2 = Obs_Data->SdSatObs[i].adr[1] * WaveLen_L2;
			L_gf = L1 - L2;
			MW = 1 / (f_L1 - f_L2) * (f_L1 * L1 - f_L2 * L2) - 1 / (f_L1 + f_L2) * (f_L1 * P1 + f_L2 * P2);//单位米
			break;
		case BDS:
			L1 = Obs_Data->SdSatObs[i].adr[0] * WaveLen_B1I;
			L2 = Obs_Data->SdSatObs[i].adr[1] * WaveLen_B3I;
			L_gf = L1 - L2;
			MW = 1 / (f_B1I - f_B3I) * (f_B1I * L1 - f_B3I * L2) - 1 / (f_B1I + f_B3I) * (f_B1I * P1 + f_B3I * P2);//单位米
			break;
		default:
			break;
		}

		int j = 0;
		bool find = false;
		bool gross_error = false;
		for (j = 0; j < Obs_Data->History_Sat.size(); j++)
		{
			if (Obs_Data->SdSatObs[i].Navigation_System == Obs_Data->History_Sat[j].Navigation_System && Obs_Data->SdSatObs[i].prn == Obs_Data->History_Sat[j].prn)
			{
				find = true;

				//计算当前历元每颗卫星GF与上一历元对应GF的差值
				double dGF = L_gf - Obs_Data->History_Sat[j].Last_L_gf;
				//计算当前历元每颗卫星MW与上一历元对应MW平滑值的差值
				double dMW = MW - Obs_Data->History_Sat[j].Average_MW;

				//检查dGF和dMW是否超限
				if (abs(dGF) > 0.05 || abs(dMW) > 2.0)
				{
					//如果有粗差，则将这颗卫星从历史数据中删除，下一次出现时它将被当做新卫星加到数组末尾
					Obs_Data->SdSatObs.erase(Obs_Data->SdSatObs.begin() + i);
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
			Obs_Data->History_Sat[j].prn = Obs_Data->SdSatObs[i].prn;
			Obs_Data->History_Sat[j].Navigation_System = Obs_Data->SdSatObs[i].Navigation_System;
			Obs_Data->SdSatObs.erase(Obs_Data->SdSatObs.begin() + i);
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

	Obs_Data->Sat_Number = Obs_Data->SdSatObs.size();
}

bool CompareSdSatObsCNo(SD_SATELLITE_OBSERVATION SdSatObs1, SD_SATELLITE_OBSERVATION SdSatObs2)
{
	return ((SdSatObs1.CNo[0] + SdSatObs1.CNo[1]) < (SdSatObs2.CNo[0] + SdSatObs2.CNo[1]));
}

void SelectReferenceSatellite(SD_OBSERVATION_DATA* Sd_ObsData, DD_DATA* Dd_Data)
{
	if (Sd_ObsData->Sat_Number == 0)
		return;

	bool IsGPSRefKeep = false;
	bool IsBDSRefKeep = false;

	//参考星是否保持不变
	for (int i = 0; i < Sd_ObsData->Sat_Number; i++)
	{
		//如果在当前单差观测值中能找到之前的参考星并且满足条件
		if (Sd_ObsData->SdSatObs[i].Navigation_System == GPS && Sd_ObsData->SdSatObs[i].prn == Dd_Data->ref_prn[0]&& Sd_ObsData->SdSatObs[i].CNo[0] > 40.0 && Sd_ObsData->SdSatObs[i].CNo[1] > 30.0)
		{
				IsGPSRefKeep = true;
				Dd_Data->ref_pos[0] = i;
		}
		
		if (Sd_ObsData->SdSatObs[i].Navigation_System == BDS && Sd_ObsData->SdSatObs[i].prn == Dd_Data->ref_prn[1]&& Sd_ObsData->SdSatObs[i].CNo[0] > 40.0 && Sd_ObsData->SdSatObs[i].CNo[1] > 30.0)
		{
				IsBDSRefKeep = true;
				Dd_Data->ref_pos[1] = i;
		}
	}

	if (IsGPSRefKeep == true && IsBDSRefKeep == true)
		return;

	//Sorts the elements in the range [first,last) 按单差观测值两个频率CNo之和大小升序排列
	sort(Sd_ObsData->SdSatObs.begin(), Sd_ObsData->SdSatObs.end(), CompareSdSatObsCNo);
	
	bool FindGPSRef = IsGPSRefKeep;
	bool FindBDSRef = IsBDSRefKeep;
	for (int i = 1; i <= Sd_ObsData->Sat_Number; i++)
	{	
		auto SdSatObsFromEnd = (Sd_ObsData->SdSatObs.end() - i);

		//若某系统参考星不变还需找到它排序后的新索引值
		if (IsGPSRefKeep == true && SdSatObsFromEnd->Navigation_System == GPS && SdSatObsFromEnd->prn == Dd_Data->ref_prn[0])
			Dd_Data->ref_pos[0] = Sd_ObsData->SdSatObs.size() - i;

		if (IsBDSRefKeep == true && SdSatObsFromEnd->Navigation_System == BDS && SdSatObsFromEnd->prn == Dd_Data->ref_prn[1])
			Dd_Data->ref_pos[1] = Sd_ObsData->SdSatObs.size() - i;

		//从排序后的末尾开始依此找锁定时长大于2500s,频率1CNo>40.0,频率2CNo>30.0的GPS卫星作为参考星
		if (FindGPSRef == false && SdSatObsFromEnd->Navigation_System == GPS && SdSatObsFromEnd->locktime[0] > 2500.0 && SdSatObsFromEnd->CNo[0] > 40.0 && SdSatObsFromEnd->CNo[1] > 30.0)
		{
			Dd_Data->ref_prn[0] = SdSatObsFromEnd->prn;
			Dd_Data->ref_pos[0] = Sd_ObsData->SdSatObs.size() - i;
			FindGPSRef = true;
		}
		//从排序后的末尾开始依此找锁定时长大于2500s,频率1CNo>40.0,频率2CNo>30.0的BDS卫星作为参考星
		if (FindBDSRef == false && SdSatObsFromEnd->Navigation_System == BDS && SdSatObsFromEnd->locktime[0] > 2500.0 && SdSatObsFromEnd->CNo[0] > 40.0 && SdSatObsFromEnd->CNo[1] > 30.0)
		{
			Dd_Data->ref_prn[1] = SdSatObsFromEnd->prn;
			Dd_Data->ref_pos[1] = Sd_ObsData->SdSatObs.size() - i;
			FindBDSRef = true;
		}
	}
	
	//若没找到满足阈值条件的参考星，大概率是某导航系统缺失，将参考星索引赋为-1，prn赋为0
	if (FindGPSRef == false)
	{
		Dd_Data->ref_prn[0] = 0;
		Dd_Data->ref_pos[0] = -1;
	}

	if (FindBDSRef == false)
	{
		Dd_Data->ref_prn[1] = 0;
		Dd_Data->ref_pos[1] = -1;
	}
}

bool RTKFloat(SD_OBSERVATION_DATA* Sd_ObsData, vector<SATPOSRES>& Rover_SatPosRes, vector<SATPOSRES>& Base_SatPosRes, STAPOSRES* Rover_StaPosRes, STAPOSRES* Base_StaPosRes, DD_DATA* Dd_Data)
{
	//两个系统都没有参考星，大概率还未收到观测值，退出返回false
	if (Dd_Data->ref_pos[0] == -1 && Dd_Data->ref_pos[1] == -1)
		return false;
		
	//观测方程个数
	int m = (Sd_ObsData->Sat_Number - (Dd_Data->ref_pos[0] != -1) - (Dd_Data->ref_pos[1] != -1)) * 4;
	//未知数个数
	int n = 3 + m / 2;
	//未知数个数大于观测方程个数，返回false
	if (n > m)
		return false;

	Matrix B(m, n);
	Matrix P(m, m);
	Matrix x(n, 1);
	Matrix l(m, 1);
	Matrix X(n, 1);
	//设置待估参数的前三项初值
	X.assign(1, 1, Rover_StaPosRes->StationPos.Xyz.X);
	X.assign(2, 1, Rover_StaPosRes->StationPos.Xyz.Y);
	X.assign(3, 1, Rover_StaPosRes->StationPos.Xyz.Z);

	//基站，流动站到参考星的距离(0=GPS,1=BDS)
	double S0_Rover2RefSat[2];
	double S_Base2RefSat[2];

	double WaveLen_f1[2] = { WaveLen_L1,WaveLen_B1I };
	double WaveLen_f2[2] = { WaveLen_L2,WaveLen_B3I };
	//计算基站到所有卫星的距离（按单差观测值顺序存放）
	vector<double> S_Base2Sat(Sd_ObsData->Sat_Number);
	for (int i = 0; i < Sd_ObsData->Sat_Number; i++)
	{
		//从卫星位置数组中找与当前单差观测值对应的卫星(一定能找到，因为单差观测值就是从SPP后筛选的卫星中选出的)
		vector<SATPOSRES>::iterator BaseSat = find(Base_SatPosRes.begin(), Base_SatPosRes.end(), Sd_ObsData->SdSatObs[i]);
		S_Base2Sat[i] = BaseSat->SatXyz - Base_StaPosRes->StationPos;
	}
	//如果存在GPS参考星，获取基站到其距离
	if (Dd_Data->ref_pos[0] != -1)
		S_Base2RefSat[0] = S_Base2Sat[Dd_Data->ref_pos[0]];
	//如果存在BDS参考星，获取基站到其距离
	if (Dd_Data->ref_pos[1] != -1)
		S_Base2RefSat[1] = S_Base2Sat[Dd_Data->ref_pos[1]];

	vector<SATPOSRES>::iterator RoverRefSat[2];
	//如果存在GPS参考星，则找出
	if (Dd_Data->ref_pos[0] != -1)
		RoverRefSat[0] = find(Rover_SatPosRes.begin(), Rover_SatPosRes.end(), Sd_ObsData->SdSatObs[Dd_Data->ref_pos[0]]);
	//如果存在BDS参考星，则找出
	if (Dd_Data->ref_pos[1] != -1)
		RoverRefSat[1] = find(Rover_SatPosRes.begin(), Rover_SatPosRes.end(), Sd_ObsData->SdSatObs[Dd_Data->ref_pos[1]]);

	//双差观测值不会随迭代计算改变，事先计算存放，并为模糊度赋初值
	vector<double> P_f1(m / 4);
	vector<double> P_f2(m / 4);
	vector<double> L_f1(m / 4);
	vector<double> L_f2(m / 4);
	int h = 0;
	for (int i = 0; i < Sd_ObsData->Sat_Number; i++)
	{
		//循环到参考星则跳过
		if (i == Dd_Data->ref_pos[0] || i == Dd_Data->ref_pos[1])
			continue;
		auto& SdSatObs = Sd_ObsData->SdSatObs[i];
		//获取当前单差观测值卫星系统，后续直接根据此值选择对应系统参考星
		int s = SdSatObs.Navigation_System;

		 P_f1[h] = SdSatObs.psr[0] - Sd_ObsData->SdSatObs[Dd_Data->ref_pos[s]].psr[0];
		 P_f2[h] = SdSatObs.psr[1] - Sd_ObsData->SdSatObs[Dd_Data->ref_pos[s]].psr[1];
		 L_f1[h] = (SdSatObs.adr[0] - Sd_ObsData->SdSatObs[Dd_Data->ref_pos[s]].adr[0]) * WaveLen_f1[s];//单位米
		 L_f2[h] = (SdSatObs.adr[1] - Sd_ObsData->SdSatObs[Dd_Data->ref_pos[s]].adr[1]) * WaveLen_f2[s];//单位米

		 X.assign(3 + 2 * h + 1, 1, (L_f1[h] - P_f1[h]) / WaveLen_f1[s]);
		 X.assign(3 + 2 * h + 2, 1, (L_f2[h] - P_f2[h]) / WaveLen_f2[s]);

		 h++;
	}

	//迭代进行最小二乘求解
	for (int IterationTime = 0; IterationTime < 10; IterationTime++)
	{
		//从待估参数前三列取出流动站坐标方便后面计算
		XYZUNION RoverPos;
		RoverPos.Xyz.X = X.getelement(1, 1);
		RoverPos.Xyz.Y = X.getelement(2, 1);
		RoverPos.Xyz.Z = X.getelement(3, 1);
		//如果存在GPS参考星，计算流动站到其距离
		if (Dd_Data->ref_pos[0] != -1)
			S0_Rover2RefSat[0] = RoverRefSat[0]->SatXyz - RoverPos;
		//如果存在BDS参考星，计算流动站到其距离
		if (Dd_Data->ref_pos[1] != -1)
			S0_Rover2RefSat[1] = RoverRefSat[1]->SatXyz - RoverPos;
		
		int k = 0;
		//循环观测值构建B,P,l矩阵
		for (int i = 0; i < Sd_ObsData->Sat_Number; i++)
		{
			//循环到参考星则跳过
			if (i == Dd_Data->ref_pos[0] || i == Dd_Data->ref_pos[1])
				continue;

			auto& SdSatObs = Sd_ObsData->SdSatObs[i];
			//获取当前单差观测值卫星系统，后续直接根据此值选择对应系统参考星
			int s = SdSatObs.Navigation_System;
			//从卫星位置数组中找与当前单差观测值对应的卫星(一定能找到，因为单差观测值就是从SPP后筛选的卫星中选出的)
			vector<SATPOSRES>::iterator RoverSat = find(Rover_SatPosRes.begin(), Rover_SatPosRes.end(), SdSatObs);

			double S0_Rover2Sat = RoverSat->SatXyz - RoverPos;

			double a = (RoverPos.Xyz.X - RoverSat->SatXyz.Xyz.X) / S0_Rover2Sat - (RoverPos.Xyz.X - RoverRefSat[s]->SatXyz.Xyz.X) / S0_Rover2RefSat[s];
			double b = (RoverPos.Xyz.Y - RoverSat->SatXyz.Xyz.Y) / S0_Rover2Sat - (RoverPos.Xyz.Y - RoverRefSat[s]->SatXyz.Xyz.Y) / S0_Rover2RefSat[s];
			double c = (RoverPos.Xyz.Z - RoverSat->SatXyz.Xyz.Z) / S0_Rover2Sat - (RoverPos.Xyz.Z - RoverRefSat[s]->SatXyz.Xyz.Z) / S0_Rover2RefSat[s];
			
			B.assign(4 * k + 1, 1, a); B.assign(4 * k + 1, 2, b); B.assign(4 * k + 1, 3, c);
			B.assign(4 * k + 2, 1, a); B.assign(4 * k + 2, 2, b); B.assign(4 * k + 2, 3, c);
			B.assign(4 * k + 3, 1, a); B.assign(4 * k + 3, 2, b); B.assign(4 * k + 3, 3, c); B.assign(4 * k + 3, 3 + 2 * k + 1, WaveLen_f1[s]);
			B.assign(4 * k + 4, 1, a); B.assign(4 * k + 4, 2, b); B.assign(4 * k + 4, 3, c); B.assign(4 * k + 4, 3 + 2 * k + 2, WaveLen_f2[s]);

			//几何距离双差结果
			double S0 = S0_Rover2Sat - S_Base2Sat[i] - S0_Rover2RefSat[s] + S_Base2RefSat[s];

			l.assign(4 * k + 1, 1, P_f1[k] - S0);
			l.assign(4 * k + 2, 1, P_f2[k] - S0);
			l.assign(4 * k + 3, 1, L_f1[k] - S0 - X.getelement(3 + 2 * k + 1, 1) * WaveLen_f1[s]);
			l.assign(4 * k + 4, 1, L_f2[k] - S0 - X.getelement(3 + 2 * k + 2, 1) * WaveLen_f2[s]);

			//构建P阵
			for (int j = 0; j < m / 4; j++)
			{
				if (k == j)
				{
					P.assign(4 * k + 1, 4 * k + 1, double(m) / double(m + 1));
					P.assign(4 * k + 2, 4 * k + 2, double(m) / double(m + 1));
					P.assign(4 * k + 3, 4 * k + 3, 1000.0 * double(m) / double(m + 1));
					P.assign(4 * k + 4, 4 * k + 4, 1000.0 * double(m) / double(m + 1));
				}
				else
				{
					P.assign(4 * k + 1, 4 * j + 1, -1.0 / double(m + 1));
					P.assign(4 * k + 2, 4 * j + 2, -1.0 / double(m + 1));
					P.assign(4 * k + 3, 4 * j + 3, -1000.0 / double(m + 1));
					P.assign(4 * k + 4, 4 * j + 4, -1000.0 / double(m + 1));
				}
			}
			k++;
		}

		//最小二乘求解
		x = (B.T() * P * B).inv2() * B.T() * P * l;
		//更新待估参数
		X = X + x;
		//精度评定
		Matrix Q = (B.T() * P * B).inv2();


		if (x.getelement(1, 1) < 1.0E-3 && x.getelement(2, 1) < 1.0E-3 && x.getelement(3, 1) < 1.0E-3)
		{
			Rover_StaPosRes->StationPos.Xyz.X = X.getelement(1, 1);
			Rover_StaPosRes->StationPos.Xyz.Y = X.getelement(2, 1);
			Rover_StaPosRes->StationPos.Xyz.Z = X.getelement(3, 1);
			XYZ_to_BLH(&Rover_StaPosRes->StationPos, &Rover_StaPosRes->StationPosBlh, WGS84_a, WGS84_e2);
			Dd_Data->n_of_FloatAmbiguity = n - 3;
			Dd_Data->FloatAmbiguity = X.SubMatrix(4, 1, n - 3, 1).getdata();
			Dd_Data->Q_of_FloatAmbiguity = Q.SubMatrix(4, 4, n - 3, n - 3).getdata();
			Rover_StaPosRes->Pos_State = RTKFLOAT;
			return true;
		}
	}

	return false;
}

bool RTKFixed(SD_OBSERVATION_DATA* Sd_ObsData, vector<SATPOSRES>& Rover_SatPosRes, vector<SATPOSRES>& Base_SatPosRes, STAPOSRES* Rover_StaPosRes, STAPOSRES* Base_StaPosRes, DD_DATA* Dd_Data)
{
	//观测方程个数
	int m = (Sd_ObsData->Sat_Number - (Dd_Data->ref_pos[0] != -1) - (Dd_Data->ref_pos[1] != -1)) * 2;

	Matrix B(m, 3);
	Matrix P(m, m);
	Matrix x(3, 1);
	Matrix l(m, 1);
	Matrix X(3, 1);

	//设置待估参数的初值
	X.assign(1, 1, Rover_StaPosRes->StationPos.Xyz.X);
	X.assign(2, 1, Rover_StaPosRes->StationPos.Xyz.Y);
	X.assign(3, 1, Rover_StaPosRes->StationPos.Xyz.Z);

	//基站，流动站到参考星的距离(0=GPS,1=BDS)
	double S0_Rover2RefSat[2];
	double S_Base2RefSat[2];

	double WaveLen_f1[2] = { WaveLen_L1,WaveLen_B1I };
	double WaveLen_f2[2] = { WaveLen_L2,WaveLen_B3I };
	//计算基站到所有卫星的距离（按单差观测值顺序存放）
	vector<double> S_Base2Sat(Sd_ObsData->Sat_Number);
	for (int i = 0; i < Sd_ObsData->Sat_Number; i++)
	{
		//从卫星位置数组中找与当前单差观测值对应的卫星(一定能找到，因为单差观测值就是从SPP后筛选的卫星中选出的)
		vector<SATPOSRES>::iterator BaseSat = find(Base_SatPosRes.begin(), Base_SatPosRes.end(), Sd_ObsData->SdSatObs[i]);
		S_Base2Sat[i] = BaseSat->SatXyz - Base_StaPosRes->StationPos;
	}
	//如果存在GPS参考星，获取基站到其距离
	if (Dd_Data->ref_pos[0] != -1)
		S_Base2RefSat[0] = S_Base2Sat[Dd_Data->ref_pos[0]];
	//如果存在BDS参考星，获取基站到其距离
	if (Dd_Data->ref_pos[1] != -1)
		S_Base2RefSat[1] = S_Base2Sat[Dd_Data->ref_pos[1]];

	vector<SATPOSRES>::iterator RoverRefSat[2];
	//如果存在GPS参考星，则找出
	if (Dd_Data->ref_pos[0] != -1)
		RoverRefSat[0] = find(Rover_SatPosRes.begin(), Rover_SatPosRes.end(), Sd_ObsData->SdSatObs[Dd_Data->ref_pos[0]]);
	//如果存在BDS参考星，则找出
	if (Dd_Data->ref_pos[1] != -1)
		RoverRefSat[1] = find(Rover_SatPosRes.begin(), Rover_SatPosRes.end(), Sd_ObsData->SdSatObs[Dd_Data->ref_pos[1]]);

	//双差观测值不会随迭代计算改变，事先计算存放
	vector<double> L_f1(m / 2);
	vector<double> L_f2(m / 2);
	int h = 0;
	for (int i = 0; i < Sd_ObsData->Sat_Number; i++)
	{
		//循环到参考星则跳过
		if (i == Dd_Data->ref_pos[0] || i == Dd_Data->ref_pos[1])
			continue;
		auto& SdSatObs = Sd_ObsData->SdSatObs[i];
		//获取当前单差观测值卫星系统，后续直接根据此值选择对应系统参考星
		int s = SdSatObs.Navigation_System;

		L_f1[h] = (SdSatObs.adr[0] - Sd_ObsData->SdSatObs[Dd_Data->ref_pos[s]].adr[0]) * WaveLen_f1[s];//单位米
		L_f2[h] = (SdSatObs.adr[1] - Sd_ObsData->SdSatObs[Dd_Data->ref_pos[s]].adr[1]) * WaveLen_f2[s];//单位米

		h++;
	}


	for (int IterationTime = 0; IterationTime < 10; IterationTime++)
	{
		//取出流动站坐标方便后面计算
		XYZUNION RoverPos;
		RoverPos.Xyz.X = X.getelement(1, 1);
		RoverPos.Xyz.Y = X.getelement(2, 1);
		RoverPos.Xyz.Z = X.getelement(3, 1);
		//如果存在GPS参考星，计算流动站到其距离
		if (Dd_Data->ref_pos[0] != -1)
			S0_Rover2RefSat[0] = RoverRefSat[0]->SatXyz - RoverPos;
		//如果存在BDS参考星，计算流动站到其距离
		if (Dd_Data->ref_pos[1] != -1)
			S0_Rover2RefSat[1] = RoverRefSat[1]->SatXyz - RoverPos;

		int k = 0;
		//循环观测值构建B,P,l矩阵
		for (int i = 0; i < Sd_ObsData->Sat_Number; i++)
		{
			//循环到参考星则跳过
			if (i == Dd_Data->ref_pos[0] || i == Dd_Data->ref_pos[1])
				continue;

			auto& SdSatObs = Sd_ObsData->SdSatObs[i];
			//获取当前单差观测值卫星系统，后续直接根据此值选择对应系统参考星
			int s = SdSatObs.Navigation_System;
			//从卫星位置数组中找与当前单差观测值对应的卫星(一定能找到，因为单差观测值就是从SPP后筛选的卫星中选出的)
			vector<SATPOSRES>::iterator RoverSat = find(Rover_SatPosRes.begin(), Rover_SatPosRes.end(), SdSatObs);

			double S0_Rover2Sat = RoverSat->SatXyz - RoverPos;

			double a = (RoverPos.Xyz.X - RoverSat->SatXyz.Xyz.X) / S0_Rover2Sat - (RoverPos.Xyz.X - RoverRefSat[s]->SatXyz.Xyz.X) / S0_Rover2RefSat[s];
			double b = (RoverPos.Xyz.Y - RoverSat->SatXyz.Xyz.Y) / S0_Rover2Sat - (RoverPos.Xyz.Y - RoverRefSat[s]->SatXyz.Xyz.Y) / S0_Rover2RefSat[s];
			double c = (RoverPos.Xyz.Z - RoverSat->SatXyz.Xyz.Z) / S0_Rover2Sat - (RoverPos.Xyz.Z - RoverRefSat[s]->SatXyz.Xyz.Z) / S0_Rover2RefSat[s];

			B.assign(2 * k + 1, 1, a); B.assign(2 * k + 1, 2, b); B.assign(2 * k + 1, 3, c);
			B.assign(2 * k + 2, 1, a); B.assign(2 * k + 2, 2, b); B.assign(2 * k + 2, 3, c);

			//几何距离双差结果
			double S0 = S0_Rover2Sat - S_Base2Sat[i] - S0_Rover2RefSat[s] + S_Base2RefSat[s];

			l.assign(2 * k + 1, 1, L_f1[k] - S0 - Dd_Data->FixedAmbiguity[2 * k + 0] * WaveLen_f1[s]);
			l.assign(2 * k + 2, 1, L_f2[k] - S0 - Dd_Data->FixedAmbiguity[2 * k + 1] * WaveLen_f2[s]);

			//构建P阵
			for (int j = 0; j < m / 2; j++)
			{
				if (k == j)
				{
					P.assign(2 * k + 1, 2 * k + 1, double(m) / double(m + 1));
					P.assign(2 * k + 2, 2 * k + 2, double(m) / double(m + 1));
				}
				else
				{
					P.assign(2 * k + 1, 2 * j + 1, -1.0 / double(m + 1));
					P.assign(2 * k + 2, 2 * j + 2, -1.0 / double(m + 1));
				}
			}
			k++;

		}

		//最小二乘求解
		x = (B.T() * P * B).inv2() * B.T() * P * l;
		//更新待估参数
		X = X + x;
		//精度评定
		Matrix Q = (B.T() * P * B).inv2();

		Matrix v = B * x - l;

		Rover_StaPosRes->Pos_sigma0 = sqrt((v.T() * P * v).getelement(1, 1) / (m - 3));

		if (x.getelement(1, 1) < 1.0E-6 && x.getelement(2, 1) < 1.0E-6 && x.getelement(3, 1) < 1.0E-6)
		{
			Rover_StaPosRes->StationPos.Xyz.X = X.getelement(1, 1);
			Rover_StaPosRes->StationPos.Xyz.Y = X.getelement(2, 1);
			Rover_StaPosRes->StationPos.Xyz.Z = X.getelement(3, 1);
			XYZ_to_BLH(&Rover_StaPosRes->StationPos, &Rover_StaPosRes->StationPosBlh, WGS84_a, WGS84_e2);
			//基线向量求解
			Dd_Data->Baseline.Xyz.X = Rover_StaPosRes->StationPos.Xyz.X - Base_StaPosRes->StationPos.Xyz.X;
			Dd_Data->Baseline.Xyz.Y = Rover_StaPosRes->StationPos.Xyz.Y - Base_StaPosRes->StationPos.Xyz.Y;
			Dd_Data->Baseline.Xyz.Z = Rover_StaPosRes->StationPos.Xyz.Z - Base_StaPosRes->StationPos.Xyz.Z;
			Rover_StaPosRes->Pos_State = RTKFIXED;
			return true;
		}
	}

	return false;
}

void RTK_Show(GPSTIME* GpsTime, STAPOSRES* StaPosRes, DD_DATA* Dd_Data, int SatNum)
{
	int static sum = 0;
	sum++;
	int static fix = 0;
	cout << "RTK:";
	cout << setprecision(3) << setiosflags(ios::fixed) << setw(5) << GpsTime->Week << setw(12) << GpsTime->SecOfWeek;
	cout << setprecision(6) << " X: " << setw(16) << StaPosRes->StationPos.Xyz.X << " Y: " << setw(16) << StaPosRes->StationPos.Xyz.Y << " Z: " << setw(16) << StaPosRes->StationPos.Xyz.Z;
	cout << setprecision(10) << " B: " << setw(16) << StaPosRes->StationPosBlh.B << " L: " << setw(16) << StaPosRes->StationPosBlh.L << " H: " << setw(10) << setprecision(6) << StaPosRes->StationPosBlh.H;
	cout << setprecision(5) << " Vx: " << setw(9) << StaPosRes->StationVel[0] << " Vy: " << setw(9) << StaPosRes->StationVel[1] << " Vz: " << setw(9) << StaPosRes->StationVel[2];
	cout << " sigma: " << setw(9) << StaPosRes->Pos_sigma0;
	cout << " Ratio: " << setw(10) << Dd_Data->Ratio;
	cout << " SatNum: " << setw(4) << SatNum;
	cout << setprecision(6) << " dX: " << setw(12) << Dd_Data->Baseline.Xyz.X << " dY: " << setw(12) << Dd_Data->Baseline.Xyz.Y << " dZ: " << setw(12) << Dd_Data->Baseline.Xyz.Z;
	cout << " Fixed: "; if (Dd_Data->isFixed) { cout << "true"; }
	else { cout << "false"; }
	cout << " Pos_State: " ;
	switch (StaPosRes->Pos_State)
	{
	case NOPOS:cout << "NOPOS";
		break;
	case SPP:cout << "SPP";
		break;
	case RTKFLOAT:cout << "RTKFLOAT";
		break;
	case RTKFIXED:cout << "RTKFIXED";
		break;

	}

	if (StaPosRes->Pos_State == RTKFIXED)
		fix++;
	cout << " Fixed Rate: " << double(fix) / double(sum) * 100.0 << "%";
	cout << '\n' << '\n';

}

void RTK_OutputToFile(fstream& fout, GPSTIME* GpsTime, STAPOSRES* StaPosRes, DD_DATA* Dd_Data, int SatNum)
{
	int static sum = 0;
	sum++;
	int static fix = 0;
	fout << "RTK:";
	fout << setprecision(3) << setiosflags(ios::fixed) << setw(5) << GpsTime->Week << setw(12) << GpsTime->SecOfWeek;
	fout << setprecision(6) << " X: " << setw(16) << StaPosRes->StationPos.Xyz.X << " Y: " << setw(16) << StaPosRes->StationPos.Xyz.Y << " Z: " << setw(16) << StaPosRes->StationPos.Xyz.Z;
	fout << setprecision(10) << " B: " << setw(16) << StaPosRes->StationPosBlh.B << " L: " << setw(16) << StaPosRes->StationPosBlh.L << " H: " << setw(10) << setprecision(6) << StaPosRes->StationPosBlh.H;
	fout << setprecision(5) << " Vx: " << setw(9) << StaPosRes->StationVel[0] << " Vy: " << setw(9) << StaPosRes->StationVel[1] << " Vz: " << setw(9) << StaPosRes->StationVel[2];
	fout << " sigma: " << setw(9) << StaPosRes->Pos_sigma0;
	fout << " Ratio: " << setw(10) << Dd_Data->Ratio;
	fout << " SatNum: " << setw(4) << SatNum;
	fout << setprecision(6) << " dX: " << setw(12) << Dd_Data->Baseline.Xyz.X << " dY: " << setw(12) << Dd_Data->Baseline.Xyz.Y << " dZ: " << setw(12) << Dd_Data->Baseline.Xyz.Z;
	fout << " Fixed: "; if (Dd_Data->isFixed) { fout << "true"; }
	else { fout << "false"; }
	fout << " Pos_State: ";
	switch (StaPosRes->Pos_State)
	{
	case NOPOS:fout << "NOPOS";
		break;
	case SPP:fout << "SPP";
		break;
	case RTKFLOAT:fout << "RTKFLOAT";
		break;
	case RTKFIXED:fout << "RTKFIXED";
		break;

	}

	if (StaPosRes->Pos_State == RTKFIXED)
		fix++;
	fout << " Fixed Rate: " << double(fix) / double(sum) * 100.0 << "%";
	fout << '\n' << '\n';
}