#pragma once
void LeastSquaresEstimation(const int n, const int t, const double B[], const double P[], const double l[], double x[]);
double Sigma0(const int n, const int t, const double B[], const double P[], const double l[], double x[]);
void CalculateQxx(const int n, const int t, const double B[], const double P[], double Qxx[]);