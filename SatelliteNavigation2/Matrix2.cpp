#include"Matrix2.h"
#include<iostream>
void Matrix::Initialize()
{
	M = new double[MatrixRows * MatrixCols];
}


Matrix::Matrix(int rows, int cols, double value)
{
	MatrixRows = rows;
	MatrixCols = cols;
	Initialize();

	for (int i = 0; i < MatrixRows * MatrixCols; i++)
			M[i] = value;
}

Matrix::Matrix(int rows, int cols, char delimiter, ...)
{
	MatrixRows = rows;
	MatrixCols = cols;
	Initialize();

	va_list p;//ָ�������ָ��
	va_start(p, delimiter);//��ʼ��ָ�������ָ�룬�ڶ��������ǿɱ������ǰһ��������
	
	for (int i = 0; i < MatrixRows * MatrixCols; i++)
	{
		M[i] = va_arg(p, double);//��ȡ�ɱ�������ڶ�������Ϊ�ɱ���������͡�
	}

	va_end(p);
}

Matrix::~Matrix()
{
	if (M != NULL)
	{
		delete[]M;
		M = NULL;
	}
}
//��д���ƹ��캯���������ں�������ֵΪ����ʱ�Ὣָ��̬�ռ��ָ��ֱ����ȣ�����Σ�ա�
Matrix::Matrix(const Matrix& m)
{
	 MatrixRows = m.MatrixRows;
	 MatrixCols = m.MatrixCols;
	 M = new double[m.MatrixRows * m.MatrixCols];
	for (int i = 0; i < m.MatrixRows * m.MatrixCols; i++)
		M[i] = m.M[i];
}

Matrix& Matrix::operator=(const Matrix& m)
{
	if (this == &m)
		return *this;

	if (MatrixRows != m.MatrixRows || MatrixCols != m.MatrixCols)
	{
		delete[]M;
		MatrixRows = m.MatrixRows;
		MatrixCols = m.MatrixCols;
		Initialize();
	}

	for (int i = 0; i < MatrixRows*MatrixCols; i++)
			M[i] = m.M[i];

	return *this;

}

Matrix& Matrix::operator=(double Array[])
{
	for (int i = 0; i < MatrixRows*MatrixCols; i++)
			M[i] = Array[i];

	return *this;
}

void Matrix::assign(int i, int j, double value)
{
	if (i > MatrixRows || j > MatrixCols || i < 1 || j < 1)
	{
		printf("The index is over the Matrix size");
		return ;
	}

	M[(i - 1) * MatrixCols + (j - 1)] = value;
}

Matrix Matrix::operator+(const Matrix& m)const
{
	Matrix sumM(MatrixRows,MatrixCols);

	if (MatrixRows != m.MatrixRows || MatrixCols != m.MatrixCols)
	{
		printf("The number of rows or columns in matrix addition is inconsistent\n");
		return sumM;
	}

	for (int i = 0; i < MatrixRows * MatrixCols; i++)
		sumM.M[i] = M[i] + m.M[i];

	return sumM;
}

Matrix Matrix::operator-(const Matrix& m)const
{
	Matrix subM(MatrixRows, MatrixCols);

	if (MatrixRows != m.MatrixRows || MatrixCols != m.MatrixCols)
	{
		printf("The number of rows or columns in matrix subtraction is inconsistent\n");
		return subM;
	}

	for (int i = 0; i < MatrixRows * MatrixCols; i++)
		subM.M[i] = M[i] - m.M[i];

	return subM;
}

Matrix Matrix::operator*(const Matrix& m)const
{
	Matrix mulM(MatrixRows, m.MatrixCols);

	if (MatrixCols != m.MatrixRows)
	{
		printf("The number of rows or columns in matrix multiply is inconsistent\n");
		return mulM;
	}

	for (int i = 0; i < MatrixRows; i++)
		for (int j = 0; j < m.MatrixCols; j++)
			for (int k = 0; k < MatrixCols; k++)
				mulM.M[i * m.MatrixCols + j] += M[i * MatrixCols + k] * m.M[m.MatrixCols * k + j];

	return mulM;
}

Matrix operator*(const double C, const Matrix& m)
{
	Matrix mulM(m.MatrixRows, m.MatrixCols);

	for (int i = 0; i < m.MatrixRows * m.MatrixCols; i++)
		mulM.M[i] = C * m.M[i];

	return mulM;
}

Matrix operator*(const Matrix& m, const double C)
{
	Matrix mulM(m.MatrixRows, m.MatrixCols);

	for (int i = 0; i < m.MatrixRows * m.MatrixCols; i++)
		mulM.M[i] = C * m.M[i];

	return mulM;
}

Matrix Matrix::SubMatrix(int topleft_row, int topleft_col, int submatrixrow, int submatrixcol)const
{
	Matrix SubM(submatrixrow, submatrixcol);

	if (topleft_row + submatrixrow - 1 > MatrixRows || topleft_col + submatrixcol - 1 > MatrixCols)
	{
		printf("The SubMatrix is out of the original matrix\n");
		return SubM;
	}

	for (int i = 0; i < submatrixrow; i++)
		for (int j = 0; j < submatrixcol; j++)
		{
			SubM.M[i * submatrixcol + j] = M[(i + topleft_row - 1) * MatrixCols + (j + topleft_col - 1)];
		}

	return SubM;
}

Matrix Matrix::T()const
{
	Matrix mT(MatrixCols, MatrixRows);

	for (int i = 0; i < mT.MatrixRows; i++)
		for (int j = 0; j < mT.MatrixCols; j++)
			mT.M[i * mT.MatrixCols + j] = M[j * MatrixCols + i];

	return mT;

}

double Matrix::det()const
{
	if (MatrixRows != MatrixCols)
	{
		printf("Only square matrices can find the determinant\n");
		return -1;
	}

	double Result = 0.0;
	//���׼�һ������ʽ��ֱ�����
	if (MatrixRows <= 2)
	{
		switch (MatrixRows)
		{
		case 1:Result = M[0];
			break;
		case 2:Result = M[0] * M[3] - M[1] * M[2];
			break;
		}
		return Result;
	}
	
	//ÿ�ζ�����һ��չ������ֻ��Ҫչ����һ�С���������߽�����ʽ����ĳ��Ԫ�ص�����š���Ԫ�ص�����ʽ��ֵʱ����������
	for (int FirstRow_COL = 0; FirstRow_COL < MatrixCols; FirstRow_COL++)
	{
		//��������ʽ����
		Matrix Cofactor(MatrixRows - 1, MatrixCols - 1);
		int pos = 0;
		//��ֵ��Ԫ�ص�����ʽ���顣�ӵ�һ��չ����������ʽ�õ���Ϊ���м��Ժ����ֵ��
		for (int row = 1; row < MatrixRows; row++)
		{
			for (int col = 0; col < MatrixCols; col++)
			{
				if (col == FirstRow_COL)
					continue;

				Cofactor.M[pos++] = M[row * MatrixCols + col];
			}
		}

		//�����������ʽ�ķ��š�ĳԪ�ص�����i������j������Ϊ��-1��^(i+j)�η�
		int sign = 1;
		if (FirstRow_COL % 2 == 1)
			sign = -1;
		
		Result += M[FirstRow_COL] * sign * Cofactor.det();

	}

	return Result;
}

Matrix Matrix::A()const
{
	Matrix A(MatrixRows, MatrixCols);

	if (MatrixRows != MatrixCols)
	{
		printf("Only square matrices can find the adjugate matrix\n");
		return A;
	}
	//һ��Ԫ�صİ������Ϊ1
	if (MatrixRows == 1)
	{
		A.M[0] = 1.0;
		return A;
	}

	//���������е�ÿ��Ԫ�أ�������ǵ�����ʽ����
	for (int ROW = 0; ROW < MatrixRows; ROW++)
	{
		for (int COL = 0; COL < MatrixCols; COL++)
		{
			Matrix Cofactor(MatrixRows - 1, MatrixCols - 1);
			int pos = 0;

			for (int row = 0; row < MatrixRows; row++)
			{
				if (row == ROW)
					continue;

				for (int col = 0; col < MatrixCols; col++)
				{
					if (col == COL)
						continue;
					//������ʽ���鸳ֵ
					Cofactor.M[pos++] = M[row * MatrixCols + col];
				}
			}

			//����ʽ�����ж�
			int sign = 1;
			if ((ROW + COL) % 2 == 1)
			{
				sign = -1;
			}
			//����������ʽ��������ÿһԪ�ش�������ʽֵ�����ע������ʽ���������ֵ�������Ǿ���
			A.M[COL * MatrixCols + ROW] = sign * Cofactor.det();
		}
	}

	return A;
}

Matrix Matrix::inv()const
{
	double det = this->det();
	if (det == 0)
	{
		printf("You can't invert the matrix\n");
		return *this;
	}

	Matrix invM(MatrixRows, MatrixCols);

	invM = this->A()*(1.0/det);

	return invM;
}

Matrix Matrix::inv2()const
{
	if (MatrixRows != MatrixCols)
	{
		printf("Only square matrices can find the inverse matrix\n");
		return *this;
	}

	//����һ��ͬά����λ��
	Matrix I(MatrixRows, MatrixCols);
	I.ToE(MatrixRows);

	//����һ�˾���ĸ��ƾ����Դ�Ϊ���������б任����ԭ�����ڲ�Ԫ�ز���
	Matrix T = *this;

	for (int i = 0; i < T.MatrixRows; i++)
	{
		int j;
		for ( j = i; j < T.MatrixRows; j++)
		{	//���ڴ���ֵ�ж�Ϊ��j��i��Ԫ�ز�Ϊ0
			if (fabs(T.M[j * MatrixCols + i]) > 10E-12)
			{
				break;
			}
		}
		//û���ҵ�ͬ�з�0Ԫ�أ���˹��Ԫ��������δ֪�������̶�⣬����ʽΪ0����������
		if (j == T.MatrixRows)
		{
			printf("You can't invert the matrix\n");
			return *this;
		}
		//˵����i��i�е�Ԫ��Ϊ0���޷�ֱ�ӱ任����Ҫ���ҵ��ĵ�j��i�в�Ϊ0��Ԫ�أ���ʹi����j�е���
		if (j != i)
		{
			I.ExChangeT((i + 1), (j + 1));
			T.ExChangeT((i + 1), (j + 1));
		}
		//��i��i��Ԫ�ػ�Ϊ1
		I.MulT((i + 1), (1.0 / T.M[i * MatrixCols + i]));
		T.MulT((i + 1), (1.0 / T.M[i * MatrixCols + i]));

		//��ʱi��i��Ԫ�ز�Ϊ0���������н��������任
		for (int k = 0; k < MatrixRows; k++)
		{
			if (k == i)
				continue;

			I.AddT((i + 1), -T.M[k * MatrixCols + i], (k + 1));
			T.AddT((i + 1), -T.M[k * MatrixCols + i], (k + 1));

		}

	}

	return I;
}

void Matrix::ToE(int n)
{
	//��ԭ���ľ���ɾ��
	delete[]M;
	MatrixRows = n;
	MatrixCols = n;
	Initialize();

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			if (i == j)
				M[i * n + j] = 1.0;
			else
				M[i * n + j] = 0.0;
		}
	}
}

void Matrix::ToZero()
{
	for (int i = 0; i < MatrixRows * MatrixCols; i++)
	{
		M[i] = 0.0;
	}
	return;
}

void Matrix::Show()const
{
	for (int i = 0; i < MatrixRows; i++)
	{
		printf("��%d��:  ", i + 1);
		for (int j = 0; j < MatrixCols; j++)
		{
			printf("%15.6f", M[i * MatrixCols + j]);
		}
		printf("\n");
	}
	printf("\n");
}

int Matrix::getrow()const
{
	return MatrixRows;
}

int Matrix::getcol()const
{
	return MatrixCols;
}

int Matrix::getsize()const
{
	return MatrixRows * MatrixCols;
}

double Matrix::getelement(int row, int col)const
{
	if (row > MatrixRows || col > MatrixCols|| row < 1 || col < 1 )
	{
		printf("The index is over the Matrix size\n");
		return -1;
	}

	return M[(row - 1) * MatrixCols + col - 1];
}

Matrix RotationMatrix(double x_angle, double y_angle, double z_angle, int order)
{
	Matrix X(3, 3);
	X.M[0] = 1.0; X.M[1] = 0.0; X.M[2] = 0.0;
	X.M[3] = 0.0;X.M[4] = cos(x_angle); X.M[5] = sin(x_angle);
	X.M[6] = 0.0; X.M[7] = -sin(x_angle); X.M[8] = cos(x_angle);

	Matrix Y(3, 3);
	Y.M[0] = cos(y_angle); Y.M[1] = 0.0; Y.M[2] = -sin(y_angle);
	Y.M[3] = 0.0; Y.M[4] = 1.0; Y.M[5] = 0.0;
	Y.M[6] = sin(y_angle); Y.M[7] = 0.0; Y.M[8] = cos(y_angle);

	Matrix Z(3, 3);
	Z.M[0] = cos(z_angle); Z.M[1] = sin(z_angle); Z.M[2] = 0.0;
	Z.M[3] = -sin(z_angle); Z.M[4] = cos(z_angle); Z.M[5] = 0.0;
	Z.M[6] = 0.0; Z.M[7] = 0.0; Z.M[8] = 1.0;

	switch (order)
	{
	case 1:return(X * Y * Z);
	case 2:return(X * Z * Y);
	case 3:return(Y * X * Z);
	case 4:return(Y * Z * X);
	case 5:return(Z * X * Y);
	case 6:return(Z * Y * X);
	case 7:return(X * Y * X);
	case 8:return(X * Z * X);
	case 9:return(Y * X * Y);
	case 10:return(Y * Z * Y);
	case 11:return(Z * X * Z);
	case 12:return(Z * Y * Z);
	default:
		printf("The order should be chosen from 1~12");
		return X;
		break;
	}
}

Matrix Matrix::operator,(const Matrix& m)const
{
	Matrix BigM(MatrixRows, MatrixCols + m.MatrixCols);

	if (MatrixRows != m.MatrixRows)
	{
		printf("The number of rows in matrix subtraction is inconsistent\n");
		return BigM;
	}

	for (int i = 0; i < BigM.MatrixRows; i++)
		for (int j = 0; j < BigM.MatrixCols; j++)
		{
			if (j <= MatrixCols - 1)
			{
				BigM.M[i * BigM.MatrixCols + j] = M[i * MatrixCols + j];
			}
			else
			{
				BigM.M[i * BigM.MatrixCols + j] = m.M[i * m.MatrixCols + j - MatrixCols];
			}
		}

	return BigM;

}


Matrix Matrix::operator&(const Matrix& m)const
{
	Matrix BigM(MatrixRows + m.MatrixRows, MatrixCols);

	if (MatrixCols != m.MatrixCols)
	{
		printf("The number of cols in matrix subtraction is inconsistent\n");
		return BigM;
	}

	for (int i = 0; i < BigM.MatrixRows; i++)
		for (int j = 0; j < BigM.MatrixCols; j++)
		{
			if (i <= MatrixRows - 1)
			{
				BigM.M[i * BigM.MatrixCols + j] = M[i * MatrixCols + j];
			}
			else
			{
				BigM.M[i * BigM.MatrixCols + j] = m.M[i * m.MatrixCols + j - MatrixCols * MatrixRows];
			}
		}

	return BigM;

}

void Matrix::MulT(int row, double c)
{
	if (row > MatrixRows)
	{
		printf("The index is out of Matrix's row\n");
		return;
	}

	for (int i = 0; i < MatrixCols; i++)
	{
		M[(row - 1) * MatrixCols + i] *= c;
	}

}

void Matrix::AddT(int irow, double c, int jrow)
{
	if (irow > MatrixRows|| jrow > MatrixRows)
	{
		printf("The index is out of Matrix's row\n");
		return;
	}

	for (int i = 0; i < MatrixCols; i++)
	{
		M[(jrow - 1) * MatrixCols + i] += M[(irow - 1) * MatrixCols + i] * c;
	}

}

void Matrix::ExChangeT(int irow, int jrow)
{
	if (irow > MatrixRows || jrow > MatrixRows)
	{
		printf("The index is out of Matrix's row\n");
		return;
	}

	for (int i = 0; i < MatrixCols; i++)
	{
		double t;
		t = M[(irow - 1) * MatrixCols + i];
		M[(irow - 1) * MatrixCols + i] = M[(jrow - 1) * MatrixCols + i];
		M[(jrow - 1) * MatrixCols + i] = t;
	}

}

void Matrix::RandomMatrix(double min, double max)
{
	std::default_random_engine e;//���������

	std::uniform_real_distribution<double> u(min, max);//�����������������min��maxƽ���ֲ���ʵ��

	e.seed(time(0));

	for (int i = 0; i < MatrixRows; i++)
		for (int j = 0; j < MatrixCols; j++)
		{
			M[i * MatrixCols + j] = u(e);
		}
}

double* Matrix::getdata()const
{
	double* CopyM = new double[MatrixRows * MatrixCols];

	for (int i = 0; i < MatrixRows * MatrixCols; i++)
	{
		CopyM[i] = M[i];
	}

	return CopyM;
}