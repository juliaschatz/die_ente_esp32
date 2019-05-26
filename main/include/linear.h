#pragma once

typedef struct {
    float* data;
    int length;
} Vector;

typedef struct {
    float* data;
    int rows;
    int cols;
} Matrix;

Vector matVecMul(Matrix A, Vector x);
Vector vectorSub(Vector a, Vector b);
float matAt(Matrix A, int row, int col);
void vectorScale(Vector * a, float scale);
void vectorAddScalar(Vector * a, float b);
void printVec(Vector * a);