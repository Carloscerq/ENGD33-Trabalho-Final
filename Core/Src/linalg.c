
#include "linalg.h"
#include <string.h>
#include <stdio.h>

/*
Legenda:
- A: Matriz de entrada
- B: Matriz de entrada
- C: Matriz de saída
- A_T: Transposta da matriz A
- rowsA: Número de linhas da matriz A
- colsA: Número de colunas da matriz A
- rowsB: Número de linhas da matriz B
- colsB: Número de colunas da matriz B
*/

void matrix_multiply(const float *A, int rowsA, int colsA, const float *B, int rowsB, int colsB, float *C)
{
    memset(C, 0, sizeof(float) * rowsA * colsB);
    for (int i = 0; i < rowsA; i++)
    {
        for (int j = 0; j < colsB; j++)
        {
            for (int k = 0; k < colsA; k++)
            {
                C[i * colsB + j] += A[i * colsA + k] * B[k * colsB + j];
            }
        }
    }
}

void matrix_transpose(const float *A, int rowsA, int colsA, float *AT)
{
    for (int i = 0; i < rowsA; i++)
    {
        for (int j = 0; j < colsA; j++)
        {
            AT[j * rowsA + i] = A[i * colsA + j];
        }
    }
}

void matrix_add(const float *A, const float *B, int rows, int cols, float *C)
{
    for (int i = 0; i < rows * cols; i++)
    {
        C[i] = A[i] + B[i];
    }
}

void matrix_subtract(const float *A, const float *B, int rows, int cols, float *C)
{
    for (int i = 0; i < rows * cols; i++)
    {
        C[i] = A[i] - B[i];
    }
}

void matrix_scalar_multiply(const float *A, int rows, int cols, float scalar, float *C)
{
    for (int i = 0; i < rows * cols; i++)
    {
        C[i] = A[i] * scalar;
    }
}

void matrix_multiply_vector(const float *A, int rowsA, int colsA, const float *x, float *y)
{
    memset(y, 0, sizeof(float) * rowsA);
    for (int i = 0; i < rowsA; i++)
    {
        for (int j = 0; j < colsA; j++)
        {
            y[i] += A[i * colsA + j] * x[j];
        }
    }
}

void matrix_set_identity(float *matrix, int size)
{
    memset(matrix, 0, sizeof(float) * size * size);
    for (int i = 0; i < size; i++)
    {
        matrix[i * size + i] = 1.0f;
    }
}

bool matrix_inverse_3x3(const float *A, float *A_inv)
{
    float det =
        A[0 * 3 + 0] * (A[1 * 3 + 1] * A[2 * 3 + 2] - A[1 * 3 + 2] * A[2 * 3 + 1]) -
        A[0 * 3 + 1] * (A[1 * 3 + 0] * A[2 * 3 + 2] - A[1 * 3 + 2] * A[2 * 3 + 0]) +
        A[0 * 3 + 2] * (A[1 * 3 + 0] * A[2 * 3 + 1] - A[1 * 3 + 1] * A[2 * 3 + 0]);

    if (fabsf(det) < 1e-6) // Verificacao de matriz singular
    {
        return false;
    }

    float inv_det = 1.0f / det;

    A_inv[0 * 3 + 0] = (A[1 * 3 + 1] * A[2 * 3 + 2] - A[1 * 3 + 2] * A[2 * 3 + 1]) * inv_det;
    A_inv[0 * 3 + 1] = (A[0 * 3 + 2] * A[2 * 3 + 1] - A[0 * 3 + 1] * A[2 * 3 + 2]) * inv_det;
    A_inv[0 * 3 + 2] = (A[0 * 3 + 1] * A[1 * 3 + 2] - A[0 * 3 + 2] * A[1 * 3 + 1]) * inv_det;

    A_inv[1 * 3 + 0] = (A[1 * 3 + 2] * A[2 * 3 + 0] - A[1 * 3 + 0] * A[2 * 3 + 2]) * inv_det;
    A_inv[1 * 3 + 1] = (A[0 * 3 + 0] * A[2 * 3 + 2] - A[0 * 3 + 2] * A[2 * 3 + 0]) * inv_det;
    A_inv[1 * 3 + 2] = (A[0 * 3 + 2] * A[1 * 3 + 0] - A[0 * 3 + 0] * A[1 * 3 + 2]) * inv_det;

    A_inv[2 * 3 + 0] = (A[1 * 3 + 0] * A[2 * 3 + 1] - A[1 * 3 + 1] * A[2 * 3 + 0]) * inv_det;
    A_inv[2 * 3 + 1] = (A[0 * 3 + 1] * A[2 * 3 + 0] - A[0 * 3 + 0] * A[2 * 3 + 1]) * inv_det;
    A_inv[2 * 3 + 2] = (A[0 * 3 + 0] * A[1 * 3 + 1] - A[0 * 3 + 1] * A[1 * 3 + 0]) * inv_det;

    return true;
}