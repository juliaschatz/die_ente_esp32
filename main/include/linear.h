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