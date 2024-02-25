#pragma once

#define Pi 3.14159265358979323
#define c_Light 2.99792458E8

#define GPS_GM 3.986005E14
#define GPS_we 7.2921151467E-5 //����ÿ��
#define BDS_GM 3.986004418E14
#define BDS_we 7.2921150E-5 //����ÿ��
#define F_Relativity -4.442807633E-10 //�����ЧӦӰ�쳣��

#define Deg2Rad(x) ((x)*Pi/180.0)
#define Rad2Deg(x) ((x)*180.0/Pi)

#define WGS84_a 6378137.0
#define CGCS2000_a 6378137.0
#define WGS84_e2 0.00669437999013
#define CGCS2000_e2 0.00669438002244

//��׼����Ԫ��
#define SeaLevelHeight 0
#define Temperature0 (15+273.16)
#define AirPressure0 1013.25
#define RelativeHumidity0 0.5
