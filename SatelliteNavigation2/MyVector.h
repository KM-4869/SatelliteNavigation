#pragma once

void VectorAddition(const int n, const double A[], const double B[], double C[]);
void VectorSubstraction(const int n, const double A[], const double B[], double C[]);
double VectorDotProduct(int n, const double A[], const double B[]);
void VectorCrossProduct(const double A[], const double B[], double C[]);
double VectorNorm(int n, const double A[]);
void VectorScalarMultiply(const double C, const int n, const double A[], double B[]);
void VectorToUnit(int n, const double A[], double B[]);