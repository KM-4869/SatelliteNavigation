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

		//��ÿ�ζ�����������ʱ��������飬ֱ���ܶ���Ϊֹ
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
	
	//��ȡͷ���ȣ�һ��Ϊ28��
	int HeadLen = 0;
	memcpy(&HeadLen, buff + 3, 1);
	//��ȡ�۲�ֵ����
	BinaryToAnytype(&Obs_Data->Obs_Number, buff + HeadLen);
	for (int i = 0; i < Obs_Data->Obs_Number; i++)
	{
		//��ÿ���۲�ֵѭ������ȡ�����۲�ֵ��prn���뵼��ϵͳ
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
		//�������е����ǣ��жϴ����۲�ֵ��prn���뵼��ϵͳ��֮ǰ���е������Ƿ��ظ�
		for (j = 0; j < Obs_Data->Sat_Number; j++)
		{

			if (prn == Obs_Data->SatObs[j].prn && Navigation_System == Obs_Data->SatObs[j].Navigation_System)
			{
				repeat = true;
				break;
			}

		}
		//��������ظ���������������һ
		if (repeat == false)
		{
			Obs_Data->Sat_Number++;
			Obs_Data->SatObs.push_back({});
		}
		//�Ե�j�����Ǹ�prn�ź͵���ϵͳ��j������֮ǰ���е����ǣ�Ҳ������������
		Obs_Data->SatObs[j].prn = prn;
		Obs_Data->SatObs[j].Navigation_System = Navigation_System;
		Obs_Data->SatObs[j].ParityKnownFlag = (ch_tr_status >> 11) & 1;
		unsigned short SignalType = (ch_tr_status >> 21) & 31;

		//�Ե�j�����Ǹ�ֵ��ֻ��GPS������BDS���Ǹ���α�࣬��λ���侫�ȣ������գ�����ȵ�ֵ��
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
		BinaryToAnytype(&Obs_Data->SatObs[j].adr[k], buff + HeadLen + i * 44 + 20);//(��λ��)
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
	BinaryToAnytype(&GPSEphem->toe.Week, buff + HeadLen + 24);//GPSTIME�ṹ���е�weekΪushort
	BinaryToAnytype(&GPSEphem->toc.Week, buff + HeadLen + 24);
	BinaryToAnytype(&GPSEphem->toe.SecOfWeek, buff + HeadLen + 32);
	BinaryToAnytype(&GPSEphem->toc.SecOfWeek, buff + HeadLen + 164);
	BinaryToAnytype(&GPSEphem->a, buff + HeadLen + 40);//�볤�᳤ A
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
	unsigned long BDS_toe_SecOfWeek;                 //�����е�toeΪulong����
	unsigned long BDS_toc_SecOfWeek;                 //�����е�tocΪulong����
	memcpy(&BDS_toe_SecOfWeek, buff + HeadLen + 72, 4);
	memcpy(&BDS_toc_SecOfWeek, buff + HeadLen + 40, 4);
	BDSEphem->toe.SecOfWeek = double(BDS_toe_SecOfWeek);
	BDSEphem->toc.SecOfWeek = double(BDS_toc_SecOfWeek);
	BinaryToAnytype(&BDSEphem->a, buff + HeadLen + 76);//�볤���ƽ���� RootA
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
//���ڽ������н����BDSʱתΪGPSʱ
GPSTIME BDST2GPST(GPSTIME& GpsTime)
{
	GPSTIME T;
	T.Week = GpsTime.Week + 1356;
	T.SecOfWeek = GpsTime.SecOfWeek + 14.0;
	return T;
}
//���ڽ�GPSʱתΪBDSʱ
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

	//��ȡͷ����(һ��Ϊ28)����Ϣ���ȣ�������ÿ�����ݵ��ܳ���
	int HeadLen = 0;
	memcpy(&HeadLen, buff + 3, 1);
	unsigned short MesLen;
	BinaryToAnytype(&MesLen, buff + 8);
	int TotalLen = HeadLen + MesLen + 4;

	//����crcУ��
	unsigned int crc;
	BinaryToAnytype(&crc, buff + HeadLen + MesLen);
	//У��ʧ���򷵻�0��������λ
	if (crc != crc32(buff, HeadLen + MesLen))
	{
		TotalLen = FindFirstSyncBytePos(buff + 3, MaxBuffLen - 3) + 3;/*����յ�������������Ҫ�˲��裬��������������ƺ�����ȱʧ�������ֽ���������MesLen���Ȳ�һ�¡�*/
		*RemLen = MaxBuffLen - TotalLen;                              /*�����ݷ���ȱʧ���������䳤�Ȳ��ٵ��ڽ���ĳ��ȣ����ǵ��ڴ�������Ϣͬ���ֽں���һ��ͬ���ֽ�֮��ľ���*/
		for (int i = 0; i < *RemLen; i++)
		{
			buff[i] = buff[i + TotalLen];
		}
		return 0;
	}

	//��ȡGPS�ܼ�GPS����
	BinaryToAnytype(&Obs_Data->GpsTime.Week, buff + 14);
	long GpsTime_SecOfWeek;                      //�����붨��Ϊlong����λΪ����
	memcpy(&GpsTime_SecOfWeek, buff + 16, 4);     //�����ֽ�
	Obs_Data->GpsTime.SecOfWeek = double(GpsTime_SecOfWeek) * 1.0E-3;//ǿתdouble����Ϊ��

	//����Message ID�������������Ͳ����н���
	unsigned short MesID;
	BinaryToAnytype(&MesID, buff + 4);

	//cout << "Rover  " << Obs_Data->GpsTime.SecOfWeek << "  " << MesID << endl;

	//���ò��������������ƽ���ĵ��������Ƿ���������һ��
	//static bool GEphContinuous = false;
	//static bool BEphContinuous = false;
	EPHEMERIS Ephemcur;
	vector<EPHEMERIS>::iterator Ephempre;
	

	switch (MesID)
	{
		//�۲����ݽ���
	case 43:RangeDecode(buff, Obs_Data);
		break;
		//GPS��������
	case 7:
		GPSEphemDecode(buff, &Ephemcur);
		if (Ephemcur.health == 1 || abs(Ephemcur.toe - Obs_Data->GpsTime) > 7500.0)//����������������������������ڣ���Ҫ
			break;
		Ephempre = find(GPSEphem->begin(), GPSEphem->end(), Ephemcur.prn);
		if (Ephempre != GPSEphem->end())
			*Ephempre = Ephemcur;
		else
			GPSEphem->push_back(Ephemcur);
		break;
		//BDS��������
	case 1696:
		BDSEphemDecode(buff, &Ephemcur);
		if (Ephemcur.health == 1 || abs(BDST2GPST(Ephemcur.toe) - Obs_Data->GpsTime) > 3900.0)//����������������������������ڣ���Ҫ
			break;
		Ephempre = find(BDSEphem->begin(), BDSEphem->end(), Ephemcur.prn);
		if (Ephempre != BDSEphem->end())
			*Ephempre = Ephemcur;
		else
			BDSEphem->push_back(Ephemcur);
		break;
		//λ�ý���
	case 42:PositionDecode(buff, Pos);
		break;
	default:
		break;
	}

	//�����ٽ����������˵�����������Ѿ����꣬�´ν⵽����ʱ������ٴ��㿪ʼ���������
	//if (MesID != 7)GEphContinuous = false;
	//if (MesID != 1696)BEphContinuous = false;

	//������ɺ�ǰ��TotalLen���ֽڣ���MaxBuffLen-RemLen���ֽ�
	*RemLen = MaxBuffLen - TotalLen;
	for (int i = 0; i < *RemLen; i++)
	{
		buff[i] = buff[i + TotalLen];
	}

	return MesID;

}

//����������ֻ�ʹ��ԣ���
unsigned short NonAteloem7DataDecode(unsigned char* buff, int* RemLen, OBSERVATION_DATA* Obs_Data, DECODEPOS* Pos)
{

	//��ȡͷ����(һ��Ϊ28)����Ϣ���ȣ�������ÿ�����ݵ��ܳ���
	int HeadLen = 0;
	memcpy(&HeadLen, buff + 3, 1);
	unsigned short MesLen;
	BinaryToAnytype(&MesLen, buff + 8);
	int TotalLen = HeadLen + MesLen + 4;

	//����crcУ��
	unsigned int crc;
	BinaryToAnytype(&crc, buff + HeadLen + MesLen);
	//У��ʧ���򷵻�0��������λ
	if (crc != crc32(buff, HeadLen + MesLen))
	{
		TotalLen = FindFirstSyncBytePos(buff + 3, MaxBuffLen - 3) + 3;/*����յ�������������Ҫ�˲��裬��������������ƺ�����ȱʧ�������ֽ���������MesLen���Ȳ�һ�¡�*/
		*RemLen = MaxBuffLen - TotalLen;                              /*�����ݷ���ȱʧ���������䳤�Ȳ��ٵ��ڽ���ĳ��ȣ����ǵ��ڴ�������Ϣͬ���ֽں���һ��ͬ���ֽ�֮��ľ���*/
		for (int i = 0; i < *RemLen; i++)
		{
			buff[i] = buff[i + TotalLen];
		}
		return 0;
	}

	//��ȡGPS�ܼ�GPS����
	BinaryToAnytype(&Obs_Data->GpsTime.Week, buff + 14);
	long GpsTime_SecOfWeek;                      //�����붨��Ϊlong����λΪ����
	memcpy(&GpsTime_SecOfWeek, buff + 16, 4);     //�����ֽ�
	Obs_Data->GpsTime.SecOfWeek = double(GpsTime_SecOfWeek) * 1.0E-3;//ǿתdouble����Ϊ��

	//����Message ID�������������Ͳ����н���
	unsigned short MesID;
	BinaryToAnytype(&MesID, buff + 4);

	//cout << "Base   " << Obs_Data->GpsTime.SecOfWeek << "  " << MesID << endl;

	switch (MesID)
	{
		//�۲����ݽ���
	case 43:RangeDecode(buff, Obs_Data);
		break;
		//λ�ý���
	case 42:PositionDecode(buff, Pos);
		break;
	default:
		break;
	}

	//������ɺ�ǰ��TotalLen���ֽڣ���MaxBuffLen-RemLen���ֽ�
	*RemLen = MaxBuffLen - TotalLen;
	for (int i = 0; i < *RemLen; i++)
	{
		buff[i] = buff[i + TotalLen];
	}

	return MesID;

}