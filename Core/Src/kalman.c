#include "kalman.h"
#include <string.h>

State kalman_state;
KalmanMatrices kalman_matrices;
KalmanTmpMatrices kalman_tmp_matrices;
bool kalman_initialized = false;

float start_noise = 10.0f; // Adequado verificar no datasheet
void covariance_init(void)
{
    matrix_set_identity((float *)kalman_matrices.P, N_STATES);
    kalman_matrices.P[0][0] = start_noise; // x
    kalman_matrices.P[1][1] = start_noise; // y
    kalman_matrices.P[2][2] = start_noise; // theta
    kalman_matrices.P[3][3] = start_noise; // vx
    kalman_matrices.P[4][4] = start_noise; // vy
    kalman_matrices.P[5][5] = start_noise; // omega
}

float start_process_noise = 1.0f;
void process_noise_covariance_init(void)
{
    memset(kalman_matrices.Q, 0, sizeof(kalman_matrices.Q));
    kalman_matrices.Q[0][0] = start_process_noise; // Ruído x
    kalman_matrices.Q[1][1] = start_process_noise; // Ruído y
    kalman_matrices.Q[2][2] = start_process_noise; // Ruído theta
    kalman_matrices.Q[3][3] = start_process_noise; // Ruído vx
    kalman_matrices.Q[4][4] = start_process_noise; // Ruído vy
    kalman_matrices.Q[5][5] = start_process_noise; // Ruído omega
}

float measure_noise = 0.1f; // Adequado verificar no datasheet
void measurement_noise_covariance_init(void)
{
    memset(kalman_matrices.R, 0, sizeof(kalman_matrices.R));
    kalman_matrices.R[0][0] = measure_noise; // Ruído X
    kalman_matrices.R[1][1] = measure_noise; // Ruído Y
    kalman_matrices.R[2][2] = measure_noise; // Ruído Omega
}

void measurement_matrix_init(void)
{
    // H = [ 1 0 0 0 0 0 ]
    //     [ 0 1 0 0 0 0 ]
    //     [ 0 0 0 0 0 1 ]
    memset(kalman_matrices.H, 0, sizeof(kalman_matrices.H));
    kalman_matrices.H[0][0] = 1.0f; // X
    kalman_matrices.H[1][1] = 1.0f; // Y
    kalman_matrices.H[2][5] = 1.0f; // Omega
}

void kalman_filter_init(void)
{
    memset(kalman_state.states_vector, 0, sizeof(kalman_state.states_vector));

    covariance_init();
    process_noise_covariance_init();
    measurement_noise_covariance_init();
    measurement_matrix_init();

    kalman_initialized = true;
}

void kalman_filter_predict(float dt)
{
    if (!kalman_initialized)
        return;

    // A = [ 1 0 0 dt 0  0  ]
    //     [ 0 1 0 0  dt 0  ]
    //     [ 0 0 1 0  0  dt ]
    //     [ 0 0 0 1  0  0  ]
    //     [ 0 0 0 0  1  0  ]
    //     [ 0 0 0 0  0  1  ]
    matrix_set_identity((float *)kalman_matrices.A, N_STATES);
    kalman_matrices.A[0][3] = dt; // x = x + vx*dt
    kalman_matrices.A[1][4] = dt; // y = y + vy*dt
    kalman_matrices.A[2][5] = dt; // theta = theta + omega*dt

    // (X_k|k-1 = A * X_k-1)
    matrix_multiply_vector(
        (float *)kalman_matrices.A, N_STATES, N_STATES, // A
        (float *)kalman_state.states_vector,            // X_k-1
        (float *)kalman_tmp_matrices.states_vector);    // X_k|k-1

    // x = X_k|k-1
    memcpy((float *)kalman_state.states_vector,        // X_k-1 (Destino)
           (float *)kalman_tmp_matrices.states_vector, // X_k|k-1 (Origem)
           sizeof(kalman_state.states_vector));

    //  (P_k|k-1 = A * P_k-1 * A^T + Q)

    // A^T
    float A_transposed[N_STATES][N_STATES]; // Buffer local para a transposta
    matrix_transpose(
        (float *)kalman_matrices.A, N_STATES, N_STATES, // A
        (float *)A_transposed);                         // A^T

    //  A * P
    matrix_multiply((float *)kalman_matrices.A, N_STATES, N_STATES, // A
                    (float *)kalman_matrices.P, N_STATES, N_STATES, // P
                    (float *)kalman_tmp_matrices.states_matrix);    // A * P

    // (A * P) * A^T
    matrix_multiply((float *)kalman_tmp_matrices.states_matrix, N_STATES, N_STATES, // A * P
                    (float *)A_transposed, N_STATES, N_STATES,                      // A^T
                    (float *)kalman_tmp_matrices.states_matrix);                    // (A * P) * A^T

    // P = (A * P * A^T) + Q
    matrix_add((float *)kalman_tmp_matrices.states_matrix,     // (A * P * A^T)
               (float *)kalman_matrices.Q, N_STATES, N_STATES, // Q
               (float *)kalman_matrices.P);                    // P
}

void kalman_filter_update(SensorData *data)
{
    if (!kalman_initialized)
        return;

    //****************************************************************************************** */
    // Inovacao — z_k - H * X_k|k-1

    // H * X_k|k-1
    matrix_multiply_vector((float *)kalman_matrices.H, N_MEASUREMENTS, N_STATES,
                           (float *)kalman_state.states_vector,
                           (float *)kalman_tmp_matrices.measurements_vector);

    // Medicao (z_k)
    float actual_measurement[N_MEASUREMENTS];
    actual_measurement[0] = data->x;
    actual_measurement[1] = data->y;
    actual_measurement[2] = data->omega;

    // Calculo Inovacao z_k - H * X_k|k-1
    float innovation[N_MEASUREMENTS];
    for (int i = 0; i < N_MEASUREMENTS; i++)
    {
        innovation[i] = actual_measurement[i] - kalman_tmp_matrices.measurements_vector[i];
    }
    //****************************************************************************************** */
    //****************************************************************************************** */
    // Ganho de Kalman — (K_k)= P_k|k-1 * H^T * (H * P_k|k-1 * H^T + R)^-1

    // H * P_k|k-1 * H^T + R)
    // H^T
    float H_transposed[N_STATES][N_MEASUREMENTS];
    matrix_transpose((float *)kalman_matrices.H, N_MEASUREMENTS, N_STATES, // H
                     (float *)H_transposed);                               // H^T

    // H * P_k|k-1
    matrix_multiply((float *)kalman_matrices.H, N_MEASUREMENTS, N_STATES, // H
                    (float *)kalman_matrices.P, N_STATES, N_STATES,       // P_k|k-1
                    (float *)kalman_tmp_matrices.measurements_states);

    // (H * P_k|k-1) * H^T
    matrix_multiply(
        (float *)kalman_tmp_matrices.measurements_states, N_MEASUREMENTS, N_STATES, // H * P_k|k-1
        (float *)H_transposed, N_STATES, N_MEASUREMENTS,                            // H^T
        (float *)kalman_tmp_matrices.measurements_matrix);

    // (H * P_k|k-1 * H^T + R)
    matrix_add(
        (float *)kalman_tmp_matrices.measurements_matrix,           // (H * P_k|k-1) * H^T
        (float *)kalman_matrices.R, N_MEASUREMENTS, N_MEASUREMENTS, // R
        (float *)kalman_tmp_matrices.measurements_matrix);

    // (H * P_k|k-1 * H^T + R)^-1
    if (!matrix_inverse_3x3((float *)kalman_tmp_matrices.measurements_matrix,
                            (float *)kalman_tmp_matrices.measurements_matrix))
    {
        return;
    }

    // P_k|k-1 * H^T
    matrix_multiply(
        (float *)kalman_matrices.P, N_STATES, N_STATES,  // P_k|k-1
        (float *)H_transposed, N_STATES, N_MEASUREMENTS, // H^T
        (float *)kalman_tmp_matrices.states_measurements);

    // (K_k)= P_k|k-1 * H^T * (H * P_k|k-1 * H^T + R)^-1
    matrix_multiply(
        (float *)kalman_tmp_matrices.states_measurements, N_STATES, N_MEASUREMENTS,       // P_k|k-1 * H^T
        (float *)kalman_tmp_matrices.measurements_matrix, N_MEASUREMENTS, N_MEASUREMENTS, // (H * P_k|k-1 * H^T + R)^-1
        (float *)kalman_tmp_matrices.states_measurements);

    //****************************************************************************************** */
    //****************************************************************************************** */

    // Atualizacao — (X_k = X_k|k-1 + K_k * ( z_k - H * X_k|k-1))

    // K_k * ( z_k - H * X_k|k-1)
    matrix_multiply_vector(
        (float *)kalman_tmp_matrices.states_measurements, N_STATES, N_MEASUREMENTS, // K_k
        innovation,                                                                 // z_k - H * X_k|k-1
        (float *)kalman_tmp_matrices.states_vector);

    //  X_k|k-1 + (K_k * ( z_k - H * X_k|k-1))
    for (int i = 0; i < N_STATES; i++)
    {
        kalman_state.states_vector[i] += kalman_tmp_matrices.states_vector[i];
    }

    //****************************************************************************************** */
    //****************************************************************************************** */
    // Atualizar a Covariancia (P_k = (I - K_k * H) * P_k|k-1)

    // I
    float I_matrix[N_STATES][N_STATES];
    matrix_set_identity((float *)I_matrix, N_STATES);

    // K_k * H
    matrix_multiply((float *)kalman_tmp_matrices.states_measurements, N_STATES, N_MEASUREMENTS, // K_k
                    (float *)kalman_matrices.H, N_MEASUREMENTS, N_STATES,                       // H
                    (float *)kalman_tmp_matrices.states_matrix);

    // I - (K_k * H)
    matrix_subtract(
        (float *)I_matrix,                                              // I
        (float *)kalman_tmp_matrices.states_matrix, N_STATES, N_STATES, // K_k * H
        (float *)kalman_tmp_matrices.states_matrix);

    // P_k = (I - K_k * H) * P_k|k-1
    matrix_multiply((float *)kalman_tmp_matrices.states_matrix, N_STATES, N_STATES, // I - K_k * H
                    (float *)kalman_matrices.P, N_STATES, N_STATES,                 // P_k|k-1
                    (float *)kalman_matrices.P);
}