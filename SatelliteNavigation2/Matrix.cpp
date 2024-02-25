
#include<stdlib.h>
#include<math.h>

/************************************************************

矩阵相乘函数

输入m1*n1矩阵A,m2*n2矩阵B,运算得m1*n2矩阵C

当矩阵维数不一致导致无法相乘时返回值为0，正常则返回值为1

******************************************************************/

int MatrixMultiply(const int m1, const int n1, const int m2, const int n2, const double A[], const double B[],  double C[])
{
	int i, j, k;
	double sum;

	//判断两矩阵维数是否能进行相乘
	if (n1 != m2)
	{
		return 0;
	}


	//矩阵相乘
	for (i = 0; i < m1; i++)
	{
		for (j = 0; j < n2; j++)
		{
			sum = 0;
			for (k = 0; k < n1; k++)
			{
				sum = sum + A[i * n1 + k] * B[k * n2 + j];
			}
			C[i * n2 + j] = sum;
		}
	}
	return 1;
}



/*******************************************

m*n的A矩阵与m*n的B矩阵相加

************************************************/
void MatrixAddition(const int m, const int n, const double A[], const double B[], double C[])
{
	//判断矩阵维数是否能进行加法

	for (int i = 0; i < m * n; i++)
	{
		C[i] = A[i] + B[i];
	}
	

}


/*******************************************

m*n的A矩阵减去m*n的B矩阵

************************************************/

void MatrixSubstraction(const int m, const int n, const double A[], const double B[], double C[])
{


	for (int i = 0; i < m * n; i++)
	{
		C[i] = A[i] - B[i];
	}
	

}



/*******************************************

由m*n的矩阵A经过旋转得到n*m的矩阵B

************************************************/

int MatrixT(const int m, const int n, const double A[], double B[])
{
	if (sizeof(A) == sizeof(B))
	{
		for (int i = 0; i < m * n; i++)
		{
			B[i] = A[(i % m) * n + (i / m)];
		}
		return 1;
	}
	else
	{
		return 0;
	}
}



/************************************************************

求n阶方阵A的行列式,返回值为行列式结果

核心思想是将行列式进行降阶，将高阶（3阶及以上）降为2阶，然后用公式计算。

利用此函数进行迭代，一次降一阶，直至2阶

******************************************************************/

double Matrix_det(const int n, const double A[])
{
	double Result = 0;

	if (n > 2)
	{
		//每次都按第一行展开，且只需要展开第一行。定义任意高阶行列式首行某列元素的列序号。此元素的余子式赋值时将跳过本列
		int row1_coln;

		for (row1_coln = 0; row1_coln < n; row1_coln++)
		{

			//定义余子式数组
			double* Cofactor = new double[(n - 1) * (n - 1)];
			int i = 0;

			//赋值该元素的余子式数组。从第一行展开，故余子式用到的为二行及以后的数值。
			for (int row = 1; row < n; row++)
			{
				for (int col = 0; col < n; col++)
				{
					//跳过该元素所在列
					if (row1_coln == col)
					{
						continue;
					}
					else
					{
						Cofactor[i++] = A[row * n + col];
					}
				}
			}

			//解决代数余子式的符号。某元素的行数i，列数j，符号为（-1）^(i+j)次方
			int sign = 1;
			if (row1_coln % 2 == 1)
			{
				sign = -1;
			}

			//行列式的值为按第一行展开后的结果，及第一行的元素乘他们的代数余子式。
			Result = Result + A[row1_coln] * sign * Matrix_det(n - 1, Cofactor);
			delete[]Cofactor;
		}
	}
	//二阶及一阶行列式的直接求解
	else
	{

		switch (n)
		{
		case 1:Result = A[0];
			break;
		case 2:Result = A[0] * A[3] - A[1] * A[2];
			break;
		}


	}
	return Result;
}


/************************************************************

输入n阶的矩阵A，得到其伴随矩阵B

******************************************************************/
void MatrixAdjugate(const int n, const double A[], double B[])
{

	//定义一个过渡矩阵，此矩阵转置即可得伴随矩阵
	double* T = new double[n * n];


	//遍历对A中的每个元素，求出它们的余子式数组
	for (int A_row = 0; A_row < n; A_row++)
	{
		for (int A_col = 0; A_col < n; A_col++)
		{
			double* Cofactor = new double[(n - 1) * (n - 1)];
			int i = 0;

			for (int row = 0; row < n; row++)
			{
				if (row == A_row)
				{
					continue;
				}
				for (int col = 0; col < n; col++)
				{
					if (col == A_col)
					{
						continue;
					}
					//给余子式数组赋值
					Cofactor[i++] = A[row * n + col];
				}
			}

			//余子式符号判断
			int sign = 1;
			if ((A_row + A_col) % 2 == 1)
			{
				sign = -1;
			}

			//利用求行列式函数，将每一代数余子式值求出（注意余子式本身就是数值，而不是矩阵）
			T[A_row * n + A_col] = sign * Matrix_det(n - 1, Cofactor);
			delete[]Cofactor;
		}
	}

	//利用转置矩阵，将矩阵T转置得B。
	MatrixT(n, n, T, B);
	delete[]T;
	
}



/************************************************************

输入n阶矩阵A，如果行列式不为0，则输出逆矩阵B

公式为A^(-1)=A的伴随/|A|

******************************************************************/

int Matrix_inv(const int n, const double A[], double B[])
{
	double det = Matrix_det(n, A);
	if (det == 0)
	{
		return 0;
	}
	
	MatrixAdjugate(n, A, B);

		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				B[i * n + j] = B[i * n + j] / det;
			}
		}

		return 1;
	
}



/************************************************************

输入常数C,m*n维的矩阵A，得到矩阵数乘结果C*A=B

******************************************************************/

void MatrixScalarMultiply(const double C, const int m, const int n, const double A[], double B[])
{
	for (int i = 0; i < m * n; i++)
	{
		B[i] = C * A[i];
	}

	
}

/************************************************************

输入维数n，创造一个n维单位阵

******************************************************************/

void CreateE(int n, double E[])
{
	for(int i=0;i<n;i++)
		for (int j = 0; j < n; j++)
		{
			if (i == j)
				E[i * n + j] = 1;
			else
				E[i * n + j] = 0;
		}
}

/*******************************************************************************

在三维空间内输入三个旋转角 和矩阵排列顺序，得到一个旋转矩阵C。order由1-6分别对应

  XYZ   XZY   YXZ   YZX   ZXY   ZYX   

  ！！矩阵排列顺序与轴的旋转顺序相反！！（如旋转顺序为Z轴->X轴->Y轴，则选择order=3）

********************************************************************************/

void RotateMatrix(double x_angle,double y_angle,double z_angle,int order,double C[])
{

	double X[9] = { 1,0,0,0,cos(x_angle),sin(x_angle),0,-sin(x_angle),cos(x_angle) };
	double Y[9] = { cos(y_angle),0,-sin(y_angle),0,1,0,sin(y_angle),0,cos(y_angle) };
	double Z[9] = { cos(z_angle),sin(z_angle),0,-sin(z_angle),cos(z_angle),0,0,0,1 };

	double T[9];

	switch (order)
	{
	case 1:MatrixMultiply(3, 3, 3, 3, X, Y, T);
		MatrixMultiply(3, 3, 3, 3, T, Z, C);
		break;
	case 2:MatrixMultiply(3, 3, 3, 3, Y, X, T);
		MatrixMultiply(3, 3, 3, 3, T, Z, C);
		break;
	case 3:MatrixMultiply(3, 3, 3, 3, Y, X, T);
		MatrixMultiply(3, 3, 3, 3, T, Z, C);
		break;
	case 4:MatrixMultiply(3, 3, 3, 3, Y, Z, T);
		MatrixMultiply(3, 3, 3, 3, T, X, C);
		break;
	case 5:MatrixMultiply(3, 3, 3, 3, Z, X, T);
		MatrixMultiply(3, 3, 3, 3, T, Y, C);
		break;
	case 6:MatrixMultiply(3, 3, 3, 3, Z, Y, T);
		MatrixMultiply(3, 3, 3, 3, T, X, C);
		break;
	default:
		break;
	}
	


}