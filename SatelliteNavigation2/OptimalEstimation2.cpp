#include"OptimalEstimation2.h"
void LeastSquaresEstimation(const Matrix& B, const Matrix& P, const Matrix& l, Matrix& x)
{
	x = (B.T() * P * B).inv() * (B.T() * P * l);
}

void LeastSquaresEstimation(const Matrix& B, const Matrix& l, Matrix& x)
{
	x = (B.T() * B).inv() * (B.T() * l);
}

double Sigma0(const Matrix& B, const Matrix& P, const Matrix& l, const Matrix& x)
{
	Matrix v(B.getrow(), 1);
	v = B * x - l;

	return sqrt((v.T() * P * v).getelement(1, 1) / (B.getrow() - B.getcol()));
}

void CalculateQxx(const Matrix& B, const Matrix& P, Matrix& Qxx)
{
	Qxx = (B.T() * P * B).inv();
}