#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>
#include <math.h>
#include "linalg.h"

#define N_STATES 6
#define N_MEASUREMENTS 3

typedef struct
{
    float states_vector[N_STATES];
} State;

typedef struct
{
    float measurements_vector[N_MEASUREMENTS];
} Measurement;

typedef struct
{
    float P[N_STATES][N_STATES];             // Matriz de covariância
    float Q[N_STATES][N_STATES];             // Matriz de covariância do ruído do processo
    float R[N_MEASUREMENTS][N_MEASUREMENTS]; // Matriz de covariância da medição
    float A[N_STATES][N_STATES];             // Matriz de transição de estado
    float H[N_MEASUREMENTS][N_STATES];       // Matriz de observação
} KalmanMatrices;

typedef struct
{
    float states_matrix[N_STATES][N_STATES];
    float states_vector[N_STATES];
    float measurements_matrix[N_MEASUREMENTS][N_MEASUREMENTS];
    float measurements_vector[N_MEASUREMENTS];
    float states_measurements[N_STATES][N_MEASUREMENTS];
    float measurements_states[N_MEASUREMENTS][N_STATES];

} KalmanTmpMatrices;

typedef struct
{
    float x, y, omega;
} SensorData;

extern KalmanMatrices kalman_matrices;
extern KalmanTmpMatrices kalman_tmp_matrices;
extern bool kalman_initialized;
void kalman_filter_init(void);
void kalman_filter_predict(float dt);
void kalman_filter_update(SensorData *data);

#endif // KALMAN_H