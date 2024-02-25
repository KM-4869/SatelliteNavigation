#include"Coordinate.h"
#include"MyVector.h"
#include"Constant.h"
#include<math.h>
#include"Matrix.h"

int XYZ_to_BLH(XYZUNION* XyzUnion, BLH* Blh, const double a, const double e2)
{


	double e12 = e2 / (1.0 - e2);
	double c = sqrt(e12 + 1) * a;
	double tanB = 0;

	Blh->L = atan2(XyzUnion->Xyz.Y, XyzUnion->Xyz.X) * 180 / Pi;

	if (XyzUnion->Xyz.Z <= fabs(1.0E-8) &&XyzUnion->Xyz.X<=fabs(1.0E-8)&& XyzUnion->Xyz.Y <= fabs(1.0E-8))
	{
		return 0;
	}

	tanB = XyzUnion->Xyz.Z / sqrt(XyzUnion->Xyz.X * XyzUnion->Xyz.X + XyzUnion->Xyz.Y * XyzUnion->Xyz.Y);

	for (int i = 0; i <= 20; i++)
	{
		double t = tanB;
		tanB = XyzUnion->Xyz.Z / sqrt(XyzUnion->Xyz.X * XyzUnion->Xyz.X + XyzUnion->Xyz.Y * XyzUnion->Xyz.Y) + (c * e2 * tanB) / (sqrt(XyzUnion->Xyz.X * XyzUnion->Xyz.X + XyzUnion->Xyz.Y * XyzUnion->Xyz.Y) * sqrt(1 + e12 + tanB * tanB));

		if (fabs(t - tanB) < 1.0E-6)
		{
			break;
		}
	}

	Blh->B = atan(tanB) * 180 / Pi;

	double cosB = cos(Blh->B * Pi / 180);
	double sinB = sin(Blh->B * Pi / 180);
	double W = sqrt(1 - e2 * sinB * sinB);
	double N = a / W;
	if (sinB == 0)
	{
		Blh->H = sqrt(XyzUnion->Xyz.X * XyzUnion->Xyz.X + XyzUnion->Xyz.Y * XyzUnion->Xyz.Y) - a;
	}
	else
	{
		Blh->H =XyzUnion->Xyz.Z/sinB  - N * (1 - e2);
	}


	return 1;


}





void BLH_to_XYZ(BLH *Blh,XYZUNION*XyzUnion,const double a,const double e2)
{
	
	double cosB = cos(Blh->B * Pi / 180.0);
	double sinB = sin(Blh->B * Pi / 180.0);
	double cosL = cos(Blh->L * Pi / 180.0);
	double sinL = sin(Blh->L * Pi / 180.0);
	double W = sqrt(1 - e2 * sinB * sinB);
	double N = a / W;

	XyzUnion->Xyz.X = (N + Blh->H) * cosB * cosL;
	XyzUnion->Xyz.Y = (N + Blh->H) * cosB * sinL;
	XyzUnion->Xyz.Z = (N * (1 - e2) + Blh->H) * sinB;

}

void StationXYZ_to_EarthXYZ(XYZUNION* StationXYZ, XYZUNION* StationPos, double a, double e2, XYZUNION* EarthXYZ)
{
	//如果测站在地心处，则地球坐标系与测站坐标系重合
	if (fabs(StationPos->Xyz.X) < 1.0E-5 && fabs(StationPos->Xyz.Y) < 1.0E-5 && fabs(StationPos->Xyz.Z) < 1.0E-5)
	{
		*EarthXYZ = *StationXYZ;
		return;
	}

	BLH blh;
	XYZ_to_BLH(StationPos, &blh, a, e2);

	//按y轴反向，y轴先转（90°-B），z轴转（180°-L）的顺序构造旋转矩阵C与反向矩阵Py
	double Py[9];
	double C[9];
	double CPy[9];
	double CPySXYZ[3];
	CreateE(3, Py);
	Py[4] = -Py[4];
	RotateMatrix(0, Pi / 2 - Deg2Rad(blh.B), Pi - Deg2Rad(blh.L), 6, C);

	MatrixMultiply(3, 3, 3, 3, C, Py, CPy);
	MatrixMultiply(3, 3, 3, 1, CPy, StationXYZ->xyz, CPySXYZ);
	MatrixAddition(3, 1, StationPos->xyz, CPySXYZ, EarthXYZ->xyz);
}


void EarthXYZ_to_StationXYZ(XYZUNION* EarthXYZ, XYZUNION* StationPos, double a, double e2, XYZUNION* StationXYZ)
{

	//如果测站在地心处，则地球坐标系与测站坐标系重合
	if (fabs(StationPos->Xyz.X) < 1.0E-5 && fabs(StationPos->Xyz.Y) < 1.0E-5 && fabs(StationPos->Xyz.Z) < 1.0E-5)
	{
		*StationXYZ = *EarthXYZ;
		return;
	}

	BLH blh;
	XYZ_to_BLH(StationPos, &blh, a, e2);

	//将上一函数的正交矩阵拿来求转置
	double Py[9];
	double C[9];
	double CPy[9];
	double CPyT[9];
	double EXYZ_SPos[3];
	CreateE(3, Py);
	Py[4] = -Py[4];
	RotateMatrix(0, Pi / 2 - Deg2Rad(blh.B), Pi - Deg2Rad(blh.L), 6, C);

	MatrixMultiply(3, 3, 3, 3, C, Py, CPy);
	MatrixT(3, 3, CPy, CPyT);

	MatrixSubstraction(3, 1, EarthXYZ->xyz, StationPos->xyz, EXYZ_SPos);
	MatrixMultiply(3, 3, 3, 1, CPyT, EXYZ_SPos, StationXYZ->xyz);


}
//子午圈半径,经度单位为度
double RadiusOfMeridianCircle( double B, const double a, const double e2)
{
	return(a * (1 - e2) / sqrt(pow(1 - e2 * sin(Deg2Rad(B)) * sin(Deg2Rad(B)), 3)));
}
//卯酉圈半径
double RadiusOfUnitaryCircle( double B, const double a, const double e2)
{
	return(a / sqrt(1 - e2 * sin(Deg2Rad(B)) * sin(Deg2Rad(B))));
}

//采用GRS80地球椭球模型计算的正常重力
double gravity(double B, double H)
{
	double g0 = 9.7803267715 * (1 + 0.0052790414 * pow(sin(Deg2Rad(B)), 2) + 0.0000232718 * pow(sin(Deg2Rad(B)), 4));
	double g = g0 - (3.087691089E-6 - 4.397731E-9 * pow(sin(Deg2Rad(B)), 2)) * H + 0.721E-12 * H * H;
	return g;
}

//在小范围内绘制平面轨迹图经纬度转站心坐标系
void BLH_to_NE(const BLH& Station_BLH, const double a, const double e2, const BLH& Blh, double&N, double&E)
{

	double RM = RadiusOfMeridianCircle(Blh.B, a, e2);
	double RN = RadiusOfUnitaryCircle(Blh.B, a, e2);

	N = Deg2Rad(Blh.B - Station_BLH.B) * (RM + Blh.H);
	E = Deg2Rad(Blh.L - Station_BLH.L) * (RN + Blh.H) * cos(Deg2Rad(Blh.B));

}
