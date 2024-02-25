#include<math.h>
#include"Matrix.h"
/****************************************************************************

根据间接平差原理V=Bx-l，输入系数矩阵B，权阵P，常数项矩阵l，得x = (B'*P*B)\(B'*P*l).

n为观测方程个数，t为未知数个数,B为n行t列，x为t行1列，l为n行1列

NBB=B'*P*B  为t行t列；    P为n行n列；    W=B'*P*l  为t行1列

*****************************************************************************/
void LeastSquaresEstimation(const int n, const int t, const double B[], const double P[], const double l[], double x[])
{

	double* NBB = new double[t * t];
	double* W = new double[t * 1];

	double* BT = new double[t * n];

	MatrixT(n, t, B, BT);

	double* BTP = new double[t * n];

	MatrixMultiply(t, n, n, n, BT, P, BTP);

	MatrixMultiply(t, n, n, t, BTP, B, NBB);
	MatrixMultiply(t, n, n, 1, BTP, l, W);

	double* NBB_1 = new double[t * t];

	Matrix_inv(t, NBB, NBB_1);

	MatrixMultiply(t, t, t, 1, NBB_1, W, x);

	delete[]NBB;
	delete[]W;
	delete[]BT;
	delete[]BTP;
	delete[]NBB_1;

}

double Sigma0(const int n, const int t, const double B[], const double P[], const double l[], double x[])
{
	double* v = new double[n];
	double* Bx = new double[n];

	MatrixMultiply(n, t, t, 1, B, x, Bx);
	MatrixSubstraction(n, 1, Bx, l, v);

	double* vTP = new double[n];
	double vTPv;

	MatrixMultiply(1, n, n, n, v, P, vTP);
	MatrixMultiply(1, n, n, 1, vTP, v, &vTPv);

	delete[]v;
	delete[]Bx;
	delete[]vTP;

	return(sqrt(vTPv / (n - t)));
	
}
//Qxx=(B'PB)^-1
void CalculateQxx(const int n, const int t, const double B[], const double P[], double Qxx[])
{
	double* BT = new double[t * n];
	double* BTP = new double[t * n];
	double* BTPB = new double[t * t];

	MatrixT(n, t, B, BT);
	MatrixMultiply(t, n, n, n, BT, P, BTP);
	MatrixMultiply(t, n, n, t, BTP, B, BTPB);

	Matrix_inv(t, BTPB, Qxx);

	delete[]BT;
	delete[]BTP;
	delete[]BTPB;
}