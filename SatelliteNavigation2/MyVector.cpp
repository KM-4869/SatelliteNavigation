
#include<math.h>
#include"Matrix.h"
/*******************************************

nά��A������nά��B�������

************************************************/

void VectorAddition(const int n, const double A[], const double B[], double C[])
{
	MatrixAddition(1, n, A, B, C);
}




/*******************************************

nά��A������nά��B�������

************************************************/

void VectorSubstraction(const int n, const double A[], const double B[], double C[])
{
	MatrixSubstraction(1, n, A, B, C);
	
}






/************************************************************

�����ĵ��

����1��nά����A,1��nά����B,���������C

�����������ֱ𿴳�1��n�еľ�����n��1�еľ����ٽ��þ���˷������õ�

******************************************************************/

double VectorDotProduct(int n, const double A[], const double B[])
{
	double T[1] = { 0 };
	MatrixMultiply(1, n, n, 1, A, B, T);
	return T[0];
}




/************************************************************

������ά����A����ά����B�����A���B�Ľ������C.��A��B=C

******************************************************************/

void VectorCrossProduct(const double A[], const double B[], double C[])
{
	C[0] = A[1] * B[2] - A[2] * B[1];
	C[1] = -(A[0] * B[2] - A[2] * B[0]);
	C[2] = A[0] * B[1] - A[1] * B[0];

}


/************************************************************

��������A��ά��������A��������A��ģ��

******************************************************************/

double VectorNorm(int n, const double A[])
{
	double result = sqrt(VectorDotProduct(n, A, A));
	return result;

}

/************************************************************

��������������

******************************************************************/

void VectorScalarMultiply(const double C,const int n, const double A[], double B[])
{
	MatrixScalarMultiply(C, 1, n, A, B);
}






/************************************************************

��������λ��

******************************************************************/

void VectorToUnit(int n, const double A[], double B[])
{
	
	VectorScalarMultiply(1.0 / VectorNorm(n, A), n, A, B);

}