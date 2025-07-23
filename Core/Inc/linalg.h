#ifndef LINALG_H
#define LINALG_H

#include <stdbool.h>
#include <math.h>

void matrix_multiply(const float *A, int rowsA, int colsA, const float *B, int rowsB, int colsB, float *C);

void matrix_transpose(const float *A, int rowsA, int colsA, float *AT);

void matrix_add(const float *A, const float *B, int rows, int cols, float *C);

void matrix_subtract(const float *A, const float *B, int rows, int cols, float *C);

void matrix_scalar_multiply(const float *A, int rows, int cols, float scalar, float *C);

void matrix_multiply_vector(const float *A, int rowsA, int colsA, const float *x, float *y);

void matrix_set_identity(float *matrix, int size);

bool matrix_inverse_3x3(const float *A, float *A_inv);

#endif // LINALG_H