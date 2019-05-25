#include "linear.h"

Vector matVecMul(Matrix A, Vector x) {
    if (A.cols != x.length) {
        Vector v = {.data = 0, .length = 0};
        return v;
    }
    Vector b = {.data = malloc(sizeof(float) * A.rows), .length = A.rows};
    for (int row = 0; row < A.rows; row++) {
        for (int col = 0; col < A.cols; col++) {
            b.data[row] += x.data[col] * matAt(A, row, col);
        }
    }
    return b;
}

Vector vectorSub(Vector a, Vector b) {
    if (a.length != b.length) {
        Vector v = {.data = 0, .length = 0};
        return v;
    }
    Vector v = {.data = malloc(sizeof(float) * a.length), .length = a.length};
    for (int i = 0; i < a.length; i++) {
        v.data[i] = a.data[i] - b.data[i];
    }
}

float matAt(Matrix A, int row, int col) {
    return A.data[row * A.cols + col];
}