
#include<math.h>
#include"Matrix.h"
/*******************************************

n维的A向量与n维的B向量相加

************************************************/

void VectorAddition(const int n, const double A[], const double B[], double C[])
{
	MatrixAddition(1, n, A, B, C);
}




/*******************************************

n维的A向量与n维的B向量相减

************************************************/

void VectorSubstraction(const int n, const double A[], const double B[], double C[])
{
	MatrixSubstraction(1, n, A, B, C);
	
}






/************************************************************

向量的点积

输入1行n维向量A,1行n维向量B,运算得向量C

把两个向量分别看成1行n列的矩阵与n行1列的矩阵再借用矩阵乘法函数得到

******************************************************************/

double VectorDotProduct(int n, const double A[], const double B[])
{
	double T[1] = { 0 };
	MatrixMultiply(1, n, n, 1, A, B, T);
	return T[0];
}




/************************************************************

输入三维向量A与三维向量B，输出A叉乘B的结果向量C.即A×B=C

******************************************************************/

void VectorCrossProduct(const double A[], const double B[], double C[])
{
	C[0] = A[1] * B[2] - A[2] * B[1];
	C[1] = -(A[0] * B[2] - A[2] * B[0]);
	C[2] = A[0] * B[1] - A[1] * B[0];

}


/************************************************************

输入向量A的维度与向量A，得向量A的模长

******************************************************************/

double VectorNorm(int n, const double A[])
{
	double result = sqrt(VectorDotProduct(n, A, A));
	return result;

}

/************************************************************

向量的数乘运算

******************************************************************/

void VectorScalarMultiply(const double C,const int n, const double A[], double B[])
{
	MatrixScalarMultiply(C, 1, n, A, B);
}






/************************************************************

将向量单位化

******************************************************************/

void VectorToUnit(int n, const double A[], double B[])
{
	
	VectorScalarMultiply(1.0 / VectorNorm(n, A), n, A, B);

}