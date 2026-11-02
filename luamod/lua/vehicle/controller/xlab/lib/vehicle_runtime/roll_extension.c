#include "roll_extension.h"
#include "vehicle_export_config.h"
#include <math.h>
static void fill(rf_real *x, int n, rf_real value) {
    if (x)
        for (int i = 0; i < n; ++i)
            x[i] = value;
}
int rf_roll_dynamics(const rf_query *q, const vehicle_model_parameters *p, const rf_command *u,
                     rf_real out[15]) {
    fill(out, 15, NAN);
    if (!q || !u || !out || !isfinite(u->steering_rate))
        return RF_INVALID_INPUT;
    vehicle_control control = {q->steering_angle_rad, u->torque, q->brake};
    int status = vehicle_evaluate(&q->vehicle, p, &control, out);
    if (status)
        return status;
    out[14] = u->steering_rate;
    return RF_OK;
}
int rf_roll_linearize(const rf_query *q, const vehicle_model_parameters *p, rf_real torque,
                      rf_real A[225], rf_real B[30], rf_real *M) {
    fill(A, 225, NAN);
    fill(B, 30, NAN);
    fill(M, 15 * VEHICLE_MODEL_PARAMETER_COUNT, NAN);
    if (!q || !A || !B || (VEHICLE_MODEL_PARAMETER_COUNT && !M))
        return RF_INVALID_INPUT;
    vehicle_control control = {q->steering_angle_rad, torque, q->brake};
    rf_real a[196], b[42], m[14 * VEHICLE_MODEL_PARAMETER_COUNT + 1];
    int status = vehicle_jacobians(&q->vehicle, p, &control, a, b, m);
    if (status)
        return status;
    fill(A, 225, 0);
    fill(B, 30, 0);
    fill(M, 15 * VEHICLE_MODEL_PARAMETER_COUNT, 0);
    for (int col = 0; col < 14; ++col)
        for (int row = 0; row < 14; ++row)
            A[row + 15 * col] = a[row + 14 * col];
    for (int row = 0; row < 14; ++row) {
        A[row + 15 * 14] = b[row];
        B[row + 15] = b[row + 14];
    }
    B[14] = 1;
    for (int col = 0; col < VEHICLE_MODEL_PARAMETER_COUNT; ++col)
        for (int row = 0; row < 14; ++row)
            M[row + 15 * col] = m[row + 14 * col];
    return RF_OK;
}
