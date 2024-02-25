
#include<stdlib.h>
#include<math.h>

/************************************************************

������˺���

����m1*n1����A,m2*n2����B,�����m1*n2����C

������ά����һ�µ����޷����ʱ����ֵΪ0�������򷵻�ֵΪ1

******************************************************************/

int MatrixMultiply(const int m1, const int n1, const int m2, const int n2, const double A[], const double B[],  double C[])
{
	int i, j, k;
	double sum;

	//�ж�������ά���Ƿ��ܽ������
	if (n1 != m2)
	{
		return 0;
	}


	//�������
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

m*n��A������m*n��B�������

************************************************/
void MatrixAddition(const int m, const int n, const double A[], const double B[], double C[])
{
	//�жϾ���ά���Ƿ��ܽ��мӷ�

	for (int i = 0; i < m * n; i++)
	{
		C[i] = A[i] + B[i];
	}
	

}


/*******************************************

m*n��A�����ȥm*n��B����

************************************************/

void MatrixSubstraction(const int m, const int n, const double A[], const double B[], double C[])
{


	for (int i = 0; i < m * n; i++)
	{
		C[i] = A[i] - B[i];
	}
	

}



/*******************************************

��m*n�ľ���A������ת�õ�n*m�ľ���B

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

��n�׷���A������ʽ,����ֵΪ����ʽ���

����˼���ǽ�����ʽ���н��ף����߽ף�3�׼����ϣ���Ϊ2�ף�Ȼ���ù�ʽ���㡣

���ô˺������е�����һ�ν�һ�ף�ֱ��2��

******************************************************************/

double Matrix_det(const int n, const double A[])
{
	double Result = 0;

	if (n > 2)
	{
		//ÿ�ζ�����һ��չ������ֻ��Ҫչ����һ�С���������߽�����ʽ����ĳ��Ԫ�ص�����š���Ԫ�ص�����ʽ��ֵʱ����������
		int row1_coln;

		for (row1_coln = 0; row1_coln < n; row1_coln++)
		{

			//��������ʽ����
			double* Cofactor = new double[(n - 1) * (n - 1)];
			int i = 0;

			//��ֵ��Ԫ�ص�����ʽ���顣�ӵ�һ��չ����������ʽ�õ���Ϊ���м��Ժ����ֵ��
			for (int row = 1; row < n; row++)
			{
				for (int col = 0; col < n; col++)
				{
					//������Ԫ��������
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

			//�����������ʽ�ķ��š�ĳԪ�ص�����i������j������Ϊ��-1��^(i+j)�η�
			int sign = 1;
			if (row1_coln % 2 == 1)
			{
				sign = -1;
			}

			//����ʽ��ֵΪ����һ��չ����Ľ��������һ�е�Ԫ�س����ǵĴ�������ʽ��
			Result = Result + A[row1_coln] * sign * Matrix_det(n - 1, Cofactor);
			delete[]Cofactor;
		}
	}
	//���׼�һ������ʽ��ֱ�����
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

����n�׵ľ���A���õ���������B

******************************************************************/
void MatrixAdjugate(const int n, const double A[], double B[])
{

	//����һ�����ɾ��󣬴˾���ת�ü��ɵð������
	double* T = new double[n * n];


	//������A�е�ÿ��Ԫ�أ�������ǵ�����ʽ����
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
					//������ʽ���鸳ֵ
					Cofactor[i++] = A[row * n + col];
				}
			}

			//����ʽ�����ж�
			int sign = 1;
			if ((A_row + A_col) % 2 == 1)
			{
				sign = -1;
			}

			//����������ʽ��������ÿһ��������ʽֵ�����ע������ʽ���������ֵ�������Ǿ���
			T[A_row * n + A_col] = sign * Matrix_det(n - 1, Cofactor);
			delete[]Cofactor;
		}
	}

	//����ת�þ��󣬽�����Tת�õ�B��
	MatrixT(n, n, T, B);
	delete[]T;
	
}



/************************************************************

����n�׾���A���������ʽ��Ϊ0������������B

��ʽΪA^(-1)=A�İ���/|A|

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

���볣��C,m*nά�ľ���A���õ��������˽��C*A=B

******************************************************************/

void MatrixScalarMultiply(const double C, const int m, const int n, const double A[], double B[])
{
	for (int i = 0; i < m * n; i++)
	{
		B[i] = C * A[i];
	}

	
}

/************************************************************

����ά��n������һ��nά��λ��

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

����ά�ռ�������������ת�� �;�������˳�򣬵õ�һ����ת����C��order��1-6�ֱ��Ӧ

  XYZ   XZY   YXZ   YZX   ZXY   ZYX   

  ������������˳���������ת˳���෴����������ת˳��ΪZ��->X��->Y�ᣬ��ѡ��order=3��

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