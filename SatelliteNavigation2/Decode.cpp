#include"Decode.h"

//NonAteloem7DataLoader::NonAteloem7DataLoader()
//{
//	 filename=" ";
//	unsigned short port=0;
//
//}


void NonAteloem7DataLoader::setRunMode(int runmode)
{
	if (!(runmode == 1 || runmode == 2))
	{
		cout << "RunMode Error! Please select 1 or 2";
		return;
	}
	RunMode = runmode;
}

void NonAteloem7DataLoader::SetFileAddress(const string& fname)
{
	filename = fname;
}

void NonAteloem7DataLoader::SetSocketAddress(const char* serveraddress, unsigned short p)
{
	strcpy(ServerAddress, serveraddress);
	port = p;
}

bool NonAteloem7DataLoader::open()
{
	if (RunMode == 1)
	{
		fin.open(filename, ios::in | ios::binary);
		return fin.is_open();
	}
	else if (RunMode == 2)
	{
		return(OpenSocket(sock, ServerAddress, port));
	}

}

void NonAteloem7DataLoader::close()
{
	if (RunMode == 1)
	{
		fin.close();
	}
	else if (RunMode == 2)
	{
		CloseSocket(sock);
	}
}


bool NonAteloem7DataLoader::ReadBuff(unsigned char* buff, int RemLen)
{
	if (RunMode == 1)
	{
		if (fin.eof())
			return false;

		fin.read((char*)buff + RemLen, MaxBuffLen - RemLen);
	}
	else if (RunMode == 2)
	{
		Sleep(10);
		int read_count = recv(sock, (char*)buff + RemLen, MaxBuffLen - RemLen, 0);
		if (read_count == -1)
		{
			cout<< "No Internet,please try again\n";
			return false;
		}

		//当每次读不满缓冲区时，多读几遍，直到能读满为止
		while (read_count != MaxBuffLen - RemLen)
		{
			Sleep(100);
			int  reread_count = recv(sock, (char*)buff + RemLen + read_count, MaxBuffLen - RemLen - read_count, 0);
			if (reread_count == -1)continue;
			read_count += reread_count;
		}

	}

	return true;
}

unsigned int crc32(const unsigned char* buff, int len)
{
	int i, j;
	unsigned int crc = 0;
	for (i = 0; i < len; i++)
	{
		crc ^= buff[i];
		for (j = 0; j < 8; j++)
		{
			if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
			else crc >>= 1;
		}
	}
	return crc;
}

int FindFirstSyncBytePos(unsigned char* buff, int BuffLen)
{

	for (int i = 0; i < BuffLen - 2; i++)
	{
		if (buff[i] == 0xAA && buff[i + 1] == 0x44 && buff[i + 2] == 0x12)
			return i;
	}

	return BuffLen - 1;
}

void RangeDecode(unsigned char* buff, OBSERVATION_DATA* Obs_Data)
{
	
	//读取头长度（一般为28）
	int HeadLen = 0;
	memcpy(&HeadLen, buff + 3, 1);
	//获取观测值个数
	BinaryToAnytype(&Obs_Data->Obs_Number, buff + HeadLen);
	for (int i = 0; i < Obs_Data->Obs_Number; i++)
	{
		//对每条观测值循环，获取此条观测值的prn号与导航系统
		unsigned short prn;
		NAVIGATION_SYSTEM Navigation_System;
		unsigned long ch_tr_status;
		BinaryToAnytype(&prn, buff + HeadLen + i * 44 + 4);
		BinaryToAnytype(&ch_tr_status, buff + HeadLen + i * 44 + 44);
		switch ((ch_tr_status >> 16) & 7)
		{
		case 0:Navigation_System = GPS;
			break;
		case 1:Navigation_System = GLONASS;
			break;
		case 3:Navigation_System = GALILEO;
			break;
		case 4:Navigation_System = BDS;
			break;
		default:Navigation_System = OTHER;
			break;
		}

		int j = 0;
		bool repeat = false;
		//遍历已有的卫星，判断此条观测值的prn号与导航系统与之前已有的卫星是否重复
		for (j = 0; j < Obs_Data->Sat_Number; j++)
		{

			if (prn == Obs_Data->SatObs[j].prn && Navigation_System == Obs_Data->SatObs[j].Navigation_System)
			{
				repeat = true;
				break;
			}

		}
		//如果不是重复卫星则卫星数加一
		if (repeat == false)
		{
			Obs_Data->Sat_Number++;
			Obs_Data->SatObs.push_back({});
		}
		//对第j颗卫星赋prn号和导航系统，j可以是之前已有的卫星，也可以是新卫星
		Obs_Data->SatObs[j].prn = prn;
		Obs_Data->SatObs[j].Navigation_System = Navigation_System;
		Obs_Data->SatObs[j].ParityKnownFlag = (ch_tr_status >> 11) & 1;
		unsigned short SignalType = (ch_tr_status >> 21) & 31;

		//对第j颗卫星赋值。只对GPS卫星与BDS卫星赋予伪距，相位及其精度，多普勒，载噪比的值。
		int k = 2;
		if (Obs_Data->SatObs[j].Navigation_System == GPS)
		{
			switch (SignalType)
			{
				//0 = L1C/A
			case 0:k = 0; break;
				//9 = L2P (Y), semi-codeless
			case 9:k = 1; break;

			default: break;
			}
		}
		else if (Obs_Data->SatObs[j].Navigation_System == BDS)
		{
			switch (SignalType)
			{
				//0 = B1 (I) with D1 data
				//4 = B1 (I) with D2 data
			case 0:
			case 4:k = 0; break;
				//2 = B3 (I) with D1 data
				//6 = B3 (I) with D2 data
			case 2:
			case 6:k = 1; break;
			default: break;
			}
		}
		if (k == 2)continue;
		BinaryToAnytype(&Obs_Data->SatObs[j].dopp[k], buff + HeadLen + i * 44 + 32);
		Obs_Data->SatObs[j].dopp[k] = -Obs_Data->SatObs[j].dopp[k];
		BinaryToAnytype(&Obs_Data->SatObs[j].CNo[k], buff + HeadLen + i * 44 + 36);
		BinaryToAnytype(&Obs_Data->SatObs[j].locktime[k], buff + HeadLen + i * 44 + 40);
		BinaryToAnytype(&Obs_Data->SatObs[j].psr[k], buff + HeadLen + i * 44 + 8);
		BinaryToAnytype(&Obs_Data->SatObs[j].adr[k], buff + HeadLen + i * 44 + 20);//(单位周)
		Obs_Data->SatObs[j].adr[k] = -Obs_Data->SatObs[j].adr[k];
		BinaryToAnytype(&Obs_Data->SatObs[j].psr_sigma[k], buff + HeadLen + i * 44 + 16);
		BinaryToAnytype(&Obs_Data->SatObs[j].adr_sigma[k], buff + HeadLen + i * 44 + 28);
	}
}


void GPSEphemDecode(unsigned char* buff, EPHEMERIS* GPSEphem)
{
	int HeadLen = 0;
	memcpy(&HeadLen, buff + 3, 1);
	
	BinaryToAnytype(&GPSEphem->prn, buff + HeadLen);
	BinaryToAnytype(&GPSEphem->health, buff + HeadLen + 12);
	BinaryToAnytype(&GPSEphem->toe.Week, buff + HeadLen + 24);//GPSTIME结构体中的week为ushort
	BinaryToAnytype(&GPSEphem->toc.Week, buff + HeadLen + 24);
	BinaryToAnytype(&GPSEphem->toe.SecOfWeek, buff + HeadLen + 32);
	BinaryToAnytype(&GPSEphem->toc.SecOfWeek, buff + HeadLen + 164);
	BinaryToAnytype(&GPSEphem->a, buff + HeadLen + 40);//半长轴长 A
	BinaryToAnytype(&GPSEphem->delta_n, buff + HeadLen + 48);
	BinaryToAnytype(&GPSEphem->MeanAnomaly, buff + HeadLen + 56);
	BinaryToAnytype(&GPSEphem->ecc, buff + HeadLen + 64);
	BinaryToAnytype(&GPSEphem->ArgumentPerigee, buff + HeadLen + 72);
	BinaryToAnytype(&GPSEphem->cuc, buff + HeadLen + 80);
	BinaryToAnytype(&GPSEphem->cus, buff + HeadLen + 88);
	BinaryToAnytype(&GPSEphem->crc, buff + HeadLen + 96);
	BinaryToAnytype(&GPSEphem->crs, buff + HeadLen + 104);
	BinaryToAnytype(&GPSEphem->cic, buff + HeadLen + 112);
	BinaryToAnytype(&GPSEphem->cis, buff + HeadLen + 120);
	BinaryToAnytype(&GPSEphem->I0, buff + HeadLen + 128);
	BinaryToAnytype(&GPSEphem->RateOfI, buff + HeadLen + 136);
	BinaryToAnytype(&GPSEphem->RightAscension, buff + HeadLen + 144);
	BinaryToAnytype(&GPSEphem->RateOfRightAscension, buff + HeadLen + 152);
	BinaryToAnytype(&GPSEphem->af0, buff + HeadLen + 180);
	BinaryToAnytype(&GPSEphem->af1, buff + HeadLen + 188);
	BinaryToAnytype(&GPSEphem->af2, buff + HeadLen + 196);
	BinaryToAnytype(&GPSEphem->tgd1, buff + HeadLen + 172);

}

void BDSEphemDecode(unsigned char* buff, EPHEMERIS* BDSEphem)
{
	int HeadLen = 0;
	memcpy(&HeadLen, buff + 3, 1);

	BinaryToAnytype(&BDSEphem->prn, buff + HeadLen);
	BinaryToAnytype(&BDSEphem->health, buff + HeadLen + 16);
	BinaryToAnytype(&BDSEphem->toe.Week, buff + HeadLen + 4);
	BinaryToAnytype(&BDSEphem->toc.Week, buff + HeadLen + 4);
	unsigned long BDS_toe_SecOfWeek;                 //星历中的toe为ulong类型
	unsigned long BDS_toc_SecOfWeek;                 //星历中的toc为ulong类型
	memcpy(&BDS_toe_SecOfWeek, buff + HeadLen + 72, 4);
	memcpy(&BDS_toc_SecOfWeek, buff + HeadLen + 40, 4);
	BDSEphem->toe.SecOfWeek = double(BDS_toe_SecOfWeek);
	BDSEphem->toc.SecOfWeek = double(BDS_toc_SecOfWeek);
	BinaryToAnytype(&BDSEphem->a, buff + HeadLen + 76);//半长轴的平方根 RootA
	BinaryToAnytype(&BDSEphem->delta_n, buff + HeadLen + 100);
	BinaryToAnytype(&BDSEphem->MeanAnomaly, buff + HeadLen + 108);
	BinaryToAnytype(&BDSEphem->ecc, buff + HeadLen + 84);
	BinaryToAnytype(&BDSEphem->ArgumentPerigee, buff + HeadLen + 92);
	BinaryToAnytype(&BDSEphem->cuc, buff + HeadLen + 148);
	BinaryToAnytype(&BDSEphem->cus, buff + HeadLen + 156);
	BinaryToAnytype(&BDSEphem->crc, buff + HeadLen + 164);
	BinaryToAnytype(&BDSEphem->crs, buff + HeadLen + 172);
	BinaryToAnytype(&BDSEphem->cic, buff + HeadLen + 180);
	BinaryToAnytype(&BDSEphem->cis, buff + HeadLen + 188);
	BinaryToAnytype(&BDSEphem->I0, buff + HeadLen + 132);
	BinaryToAnytype(&BDSEphem->RateOfI, buff + HeadLen + 140);
	BinaryToAnytype(&BDSEphem->RightAscension, buff + HeadLen + 116);
	BinaryToAnytype(&BDSEphem->RateOfRightAscension, buff + HeadLen + 124);
	BinaryToAnytype(&BDSEphem->af0, buff + HeadLen + 44);
	BinaryToAnytype(&BDSEphem->af1, buff + HeadLen + 52);
	BinaryToAnytype(&BDSEphem->af2, buff + HeadLen + 60);
	BinaryToAnytype(&BDSEphem->tgd1, buff + HeadLen + 20);
	BinaryToAnytype(&BDSEphem->tgd2, buff + HeadLen + 28);


}
//用于将星历中解出的BDS时转为GPS时
GPSTIME BDST2GPST(GPSTIME& GpsTime)
{
	GPSTIME T;
	T.Week = GpsTime.Week + 1356;
	T.SecOfWeek = GpsTime.SecOfWeek + 14.0;
	return T;
}
//用于将GPS时转为BDS时
GPSTIME GPST2BDST(GPSTIME& GpsTime)
{
	GPSTIME T;
	T.Week = GpsTime.Week - 1356;
	T.SecOfWeek = GpsTime.SecOfWeek - 14.0;
	return T;
}

void PositionDecode(unsigned char* buff, DECODEPOS* Pos)
{
	int HeadLen = 0;
	memcpy(&HeadLen, buff + 3, 1);


	BinaryToAnytype(&Pos->Blh.B, buff + HeadLen + 8);
	BinaryToAnytype(&Pos->Blh.L, buff + HeadLen + 16);
	BinaryToAnytype(&Pos->Blh.H, buff + HeadLen + 24);

	BinaryToAnytype(&Pos->lat_sigma, buff + HeadLen + 40);
	BinaryToAnytype(&Pos->lon_sigma, buff + HeadLen + 44);
	BinaryToAnytype(&Pos->hgt_sigma, buff + HeadLen + 48);

	BinaryToAnytype(&Pos->sol_age, buff + HeadLen + 60);
	memcpy(&(Pos->Satellite_Number), buff + HeadLen + 65, 1);

}

unsigned short NonAteloem7DataDecode(unsigned char* buff, int* RemLen, OBSERVATION_DATA* Obs_Data, vector<EPHEMERIS>* GPSEphem, vector<EPHEMERIS>* BDSEphem, DECODEPOS* Pos)
{

	//读取头长度(一般为28)，信息长度，并计算每条数据的总长度
	int HeadLen = 0;
	memcpy(&HeadLen, buff + 3, 1);
	unsigned short MesLen;
	BinaryToAnytype(&MesLen, buff + 8);
	int TotalLen = HeadLen + MesLen + 4;

	//进行crc校验
	unsigned int crc;
	BinaryToAnytype(&crc, buff + HeadLen + MesLen);
	//校验失败则返回0并进行移位
	if (crc != crc32(buff, HeadLen + MesLen))
	{
		TotalLen = FindFirstSyncBytePos(buff + 3, MaxBuffLen - 3) + 3;/*如果收到数据完整则不需要此步骤，但网络接收数据似乎会有缺失，导致字节数与解出的MesLen长度不一致。*/
		*RemLen = MaxBuffLen - TotalLen;                              /*若数据发生缺失或增添，则其长度不再等于解出的长度，而是等于此条的消息同步字节和下一个同步字节之间的距离*/
		for (int i = 0; i < *RemLen; i++)
		{
			buff[i] = buff[i + TotalLen];
		}
		return 0;
	}

	//获取GPS周及GPS周秒
	BinaryToAnytype(&Obs_Data->GpsTime.Week, buff + 14);
	long GpsTime_SecOfWeek;                      //周内秒定义为long，单位为毫秒
	memcpy(&GpsTime_SecOfWeek, buff + 16, 4);     //拷贝字节
	Obs_Data->GpsTime.SecOfWeek = double(GpsTime_SecOfWeek) * 1.0E-3;//强转double并化为秒

	//根据Message ID来区分数据类型并进行解码
	unsigned short MesID;
	BinaryToAnytype(&MesID, buff + 4);

	//cout << "Rover  " << Obs_Data->GpsTime.SecOfWeek << "  " << MesID << endl;

	//设置布尔变量用来控制解出的单个星历是否编成连续的一组
	//static bool GEphContinuous = false;
	//static bool BEphContinuous = false;
	EPHEMERIS Ephemcur;
	vector<EPHEMERIS>::iterator Ephempre;
	

	switch (MesID)
	{
		//观测数据解码
	case 43:RangeDecode(buff, Obs_Data);
		break;
		//GPS星历解码
	case 7:
		GPSEphemDecode(buff, &Ephemcur);
		if (Ephemcur.health == 1 || abs(Ephemcur.toe - Obs_Data->GpsTime) > 7500.0)//解出的星历若不健康或者星历过期，则不要
			break;
		Ephempre = find(GPSEphem->begin(), GPSEphem->end(), Ephemcur.prn);
		if (Ephempre != GPSEphem->end())
			*Ephempre = Ephemcur;
		else
			GPSEphem->push_back(Ephemcur);
		break;
		//BDS星历解码
	case 1696:
		BDSEphemDecode(buff, &Ephemcur);
		if (Ephemcur.health == 1 || abs(BDST2GPST(Ephemcur.toe) - Obs_Data->GpsTime) > 3900.0)//解出的星历若不健康或者星历过期，则不要
			break;
		Ephempre = find(BDSEphem->begin(), BDSEphem->end(), Ephemcur.prn);
		if (Ephempre != BDSEphem->end())
			*Ephempre = Ephemcur;
		else
			BDSEphem->push_back(Ephemcur);
		break;
		//位置解码
	case 42:PositionDecode(buff, Pos);
		break;
	default:
		break;
	}

	//若不再解出星历，则说明该组星历已经解完，下次解到星历时先清空再从零开始添加新星历
	//if (MesID != 7)GEphContinuous = false;
	//if (MesID != 1696)BEphContinuous = false;

	//解码完成后前移TotalLen个字节，即MaxBuffLen-RemLen个字节
	*RemLen = MaxBuffLen - TotalLen;
	for (int i = 0; i < *RemLen; i++)
	{
		buff[i] = buff[i + TotalLen];
	}

	return MesID;

}

//不解星历（只送大脑）！
unsigned short NonAteloem7DataDecode(unsigned char* buff, int* RemLen, OBSERVATION_DATA* Obs_Data, DECODEPOS* Pos)
{

	//读取头长度(一般为28)，信息长度，并计算每条数据的总长度
	int HeadLen = 0;
	memcpy(&HeadLen, buff + 3, 1);
	unsigned short MesLen;
	BinaryToAnytype(&MesLen, buff + 8);
	int TotalLen = HeadLen + MesLen + 4;

	//进行crc校验
	unsigned int crc;
	BinaryToAnytype(&crc, buff + HeadLen + MesLen);
	//校验失败则返回0并进行移位
	if (crc != crc32(buff, HeadLen + MesLen))
	{
		TotalLen = FindFirstSyncBytePos(buff + 3, MaxBuffLen - 3) + 3;/*如果收到数据完整则不需要此步骤，但网络接收数据似乎会有缺失，导致字节数与解出的MesLen长度不一致。*/
		*RemLen = MaxBuffLen - TotalLen;                              /*若数据发生缺失或增添，则其长度不再等于解出的长度，而是等于此条的消息同步字节和下一个同步字节之间的距离*/
		for (int i = 0; i < *RemLen; i++)
		{
			buff[i] = buff[i + TotalLen];
		}
		return 0;
	}

	//获取GPS周及GPS周秒
	BinaryToAnytype(&Obs_Data->GpsTime.Week, buff + 14);
	long GpsTime_SecOfWeek;                      //周内秒定义为long，单位为毫秒
	memcpy(&GpsTime_SecOfWeek, buff + 16, 4);     //拷贝字节
	Obs_Data->GpsTime.SecOfWeek = double(GpsTime_SecOfWeek) * 1.0E-3;//强转double并化为秒

	//根据Message ID来区分数据类型并进行解码
	unsigned short MesID;
	BinaryToAnytype(&MesID, buff + 4);

	//cout << "Base   " << Obs_Data->GpsTime.SecOfWeek << "  " << MesID << endl;

	switch (MesID)
	{
		//观测数据解码
	case 43:RangeDecode(buff, Obs_Data);
		break;
		//位置解码
	case 42:PositionDecode(buff, Pos);
		break;
	default:
		break;
	}

	//解码完成后前移TotalLen个字节，即MaxBuffLen-RemLen个字节
	*RemLen = MaxBuffLen - TotalLen;
	for (int i = 0; i < *RemLen; i++)
	{
		buff[i] = buff[i + TotalLen];
	}

	return MesID;

}