#include<math.h>
#include"Matrix.h"
/****************************************************************************

���ݼ��ƽ��ԭ��V=Bx-l������ϵ������B��Ȩ��P�����������l����x = (B'*P*B)\(B'*P*l).

nΪ�۲ⷽ�̸�����tΪδ֪������,BΪn��t�У�xΪt��1�У�lΪn��1��

NBB=B'*P*B  Ϊt��t�У�    PΪn��n�У�    W=B'*P*l  Ϊt��1��

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