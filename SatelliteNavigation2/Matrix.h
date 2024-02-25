#pragma once

int MatrixMultiply(const int m1, const int n1, const int m2, const int n2, const double A[], const double B[], double C[]);
void MatrixAddition(const int m, const int n, const double A[], const double B[], double C[]);
void MatrixSubstraction(const int m, const int n, const double A[], const double B[], double C[]);
int MatrixT(const int m, const int n, const double A[], double B[]);
double Matrix_det(const int n, const double A[]);
void MatrixAdjugate(const int n, const double A[], double B[]);
int Matrix_inv(const int n, const double A[], double B[]);
void MatrixScalarMultiply(const double C, const int m, const int n, const double A[], double B[]);
void CreateE(int n, double E[]);
void RotateMatrix(double x_angle, double y_angle, double z_angle, int order, double C[]);