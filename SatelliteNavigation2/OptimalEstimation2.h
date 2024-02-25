#pragma once
#include"Matrix2.h"
#include<math.h>
void LeastSquaresEstimation(const Matrix& B, const Matrix& P, const Matrix& l, Matrix& x);
void LeastSquaresEstimation(const Matrix& B, const Matrix& l, Matrix& x);
double Sigma0(const Matrix& B, const Matrix& P, const Matrix& l, const Matrix& x);
void CalculateQxx(const Matrix& B, const Matrix& P, Matrix& Qxx);
