#include "vehicle_runtime.h"
#include "model_generated.h"
#include "projection.h"
#include "vehicle_export_config.h"
#include <math.h>
#ifdef CR_FLOAT32
#define fabs fabsf
#endif

static void fill(vehicle_real *out, int n, vehicle_real value) {
    if (out)
        for (int i = 0; i < n; ++i)
            out[i] = value;
}
static int finite_array(const vehicle_real *x, int n) {
    for (int i = 0; i < n; ++i)
        if (!isfinite(x[i]))
            return 0;
    return 1;
}
/* Explicit packing avoids struct-to-array casts and padding assumptions. */
static void pack_state(const vehicle_state *s, vehicle_real x[14]) {
    for (int i = 0; i < 3; ++i)
        x[i] = s->position_world_m[i];
    for (int i = 0; i < 4; ++i)
        x[3 + i] = s->quaternion_xyzw[i];
    for (int i = 0; i < 3; ++i)
        x[7 + i] = s->velocity_body_m_s[i];
    for (int i = 0; i < 3; ++i)
        x[10 + i] = s->angular_velocity_body_rad_s[i];
    x[13] = s->rear_peripheral_speed_m_s;
}
static int pack_inputs(const vehicle_query *q, const vehicle_model_parameters *p,
                       const vehicle_control *u, vehicle_real state[16], vehicle_real control[3]) {
    if (!q || q->valid != 1 || !p || p->count != VEHICLE_MODEL_PARAMETER_COUNT ||
        (p->count && (!p->values || !finite_array(p->values, p->count))) || !u)
        return 0;
    pack_state(&q->state, state);
    state[14] = q->uvZ[0];
    state[15] = q->uvZ[1];
    control[0] = u->steering_angle_rad;
    control[1] = u->rear_torque_Nm;
    control[2] = u->brake;
    return finite_array(state, 16) && finite_array(q->patch, 27) && finite_array(control, 3);
}
/* Scatter the generated packed output into an already-zeroed dense buffer.
 * CasADi uses either its compact dense marker or ordinary CSC metadata. */
static int scatter(const vehicle_real *packed, const casadi_int *sp, int rows, int cols,
                   vehicle_real *dense, int leading) {
    if (!sp || sp[0] != rows || sp[1] != cols)
        return 0;
    if (sp[2] == 1) {
        for (int col = 0; col < cols; ++col)
            for (int row = 0; row < rows; ++row) {
                vehicle_real value = packed[row + rows * col];
                if (!isfinite(value))
                    return 0;
                dense[row + leading * col] = value;
            }
    } else {
        const casadi_int *colptr = sp + 2;
        const casadi_int *rowidx = sp + 3 + cols;
        if (colptr[0] != 0 || colptr[cols] < 0 || colptr[cols] > rows * cols)
            return 0;
        for (int col = 0; col < cols; ++col) {
            if (colptr[col] < 0 || colptr[col] > colptr[col + 1])
                return 0;
            for (casadi_int k = colptr[col]; k < colptr[col + 1]; ++k) {
                if (rowidx[k] < 0 || rowidx[k] >= rows || !isfinite(packed[k]))
                    return 0;
                dense[rowidx[k] + leading * col] = packed[k];
            }
        }
    }
    return 1;
}

int vehicle_evaluate(const vehicle_query *q, const vehicle_model_parameters *p,
                     const vehicle_control *u, vehicle_real out[14]) {
    fill(out, 14, NAN);
    vehicle_real state[16], control[3];
    if (!out || !pack_inputs(q, p, u, state, control))
        return VEHICLE_INVALID_INPUT;
    const casadi_real *args[vehicle_dynamics_SZ_ARG + 1] = {state, q->patch, control, p->values};
    vehicle_real packed[14];
    casadi_real *res[vehicle_dynamics_SZ_RES + 1] = {packed};
    casadi_int iw[vehicle_dynamics_SZ_IW + 1];
    casadi_real work[vehicle_dynamics_SZ_W + 1];
    if (vehicle_dynamics(args, res, iw, work, 0))
        return VEHICLE_MODEL_FAILED;
    fill(out, 14, 0);
    if (!scatter(packed, vehicle_dynamics_sparsity_out(0), 14, 1, out, 14)) {
        fill(out, 14, NAN);
        return VEHICLE_MODEL_FAILED;
    }
    return VEHICLE_OK;
}
int vehicle_jacobians(const vehicle_query *q, const vehicle_model_parameters *p,
                      const vehicle_control *u, vehicle_real A[196], vehicle_real B[42],
                      vehicle_real *M) {
    fill(A, 196, NAN);
    fill(B, 42, NAN);
    fill(M, 14 * VEHICLE_MODEL_PARAMETER_COUNT, NAN);
    vehicle_real state[16], control[3];
    if (!A || !B || (VEHICLE_MODEL_PARAMETER_COUNT && !M) || !pack_inputs(q, p, u, state, control))
        return VEHICLE_INVALID_INPUT;
    const casadi_real *args[vehicle_linearize_SZ_ARG + 1] = {state, q->patch, control, p->values};
    vehicle_real packed_A[196], packed_B[42], packed_M[14 * VEHICLE_MODEL_PARAMETER_COUNT + 1];
    casadi_real *res[vehicle_linearize_SZ_RES + 1] = {packed_A, packed_B, packed_M};
    casadi_int iw[vehicle_linearize_SZ_IW + 1];
    casadi_real work[vehicle_linearize_SZ_W + 1];
    if (vehicle_linearize(args, res, iw, work, 0))
        return VEHICLE_MODEL_FAILED;
    fill(A, 196, 0);
    fill(B, 42, 0);
    fill(M, 14 * VEHICLE_MODEL_PARAMETER_COUNT, 0);
    if (!scatter(packed_A, vehicle_linearize_sparsity_out(0), 14, 14, A, 14) ||
        !scatter(packed_B, vehicle_linearize_sparsity_out(1), 14, 3, B, 14) ||
        !scatter(packed_M, vehicle_linearize_sparsity_out(2), 14, VEHICLE_MODEL_PARAMETER_COUNT, M,
                 14)) {
        fill(A, 196, NAN);
        fill(B, 42, NAN);
        fill(M, 14 * VEHICLE_MODEL_PARAMETER_COUNT, NAN);
        return VEHICLE_MODEL_FAILED;
    }
    return VEHICLE_OK;
}
