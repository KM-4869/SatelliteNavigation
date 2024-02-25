#pragma once
#include<math.h>
struct XYZ 
{
	double X;
	double Y;
	double Z;
};
union XYZUNION
{
	XYZ Xyz;
	double xyz[3];

	XYZUNION()
	{
		Xyz.X = 0.0;
		Xyz.Y = 0.0;
		Xyz.Z = 0.0;
	}

	double operator-(XYZUNION& xyzu)
	{
		return sqrt((xyz[0] - xyzu.xyz[0]) * (xyz[0] - xyzu.xyz[0]) + (xyz[1] - xyzu.xyz[1]) * (xyz[1] - xyzu.xyz[1]) + (xyz[2] - xyzu.xyz[2]) * (xyz[2] - xyzu.xyz[2]));
	}

};

struct BLH 
{
	double B;
	double L;
	double H;
	BLH()
	{
		B = 0.0;
		L = 0.0;
		H = 0.0;
	}
};


void BLH_to_XYZ(BLH* Blh, XYZUNION* XyzUnion, const double a, const double e2);
int XYZ_to_BLH(XYZUNION* XyzUnion, BLH* Blh, const double a, const double e2);
void StationXYZ_to_EarthXYZ(XYZUNION* StationXYZ, XYZUNION* StationPos, double a, double e2, XYZUNION* EarthXYZ);
void EarthXYZ_to_StationXYZ(XYZUNION* EarthXYZ, XYZUNION* StationPos, double a, double e2, XYZUNION* StationXYZ);
double RadiusOfMeridianCircle(const double B, const double a, const double e2);
double RadiusOfUnitaryCircle(const double B, const double a, const double e2);
double gravity(double B, double H);
void BLH_to_NE(const BLH& Station_BLH, const double a, const double e2, const BLH& Blh, double& N, double& E);