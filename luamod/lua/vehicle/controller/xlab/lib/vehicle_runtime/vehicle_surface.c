/* Shared surface/state validation and projection. No vehicle dynamics,
 * Jacobian or roll graph is linked by this translation unit. */
#include "projection.h"
#include "vehicle_export_config.h"
#include "vehicle_runtime.h"
#include <math.h>
#ifdef CR_FLOAT32
#define fabs fabsf
#endif
static int finite_array(const vehicle_real *x, int n) {
    for (int i = 0; i < n; ++i)
        if (!isfinite(x[i]))
            return 0;
    return 1;
}
int vehicle_prepare(const vehicle_state *state, const vehicle_surface *surface, vehicle_query *q) {
    if (!q)
        return VEHICLE_INVALID_INPUT;
    *q = (vehicle_query){.residual_m = NAN, .projection_status = -1};
    if (!state || !surface)
        return VEHICLE_INVALID_INPUT;
    if (!finite_array(state->position_world_m, 3) || !finite_array(state->quaternion_xyzw, 4) ||
        !finite_array(state->velocity_body_m_s, 3) ||
        !finite_array(state->angular_velocity_body_rad_s, 3) ||
        !isfinite(state->rear_peripheral_speed_m_s) || !finite_array(surface->patch, 27))
        return VEHICLE_INVALID_INPUT;
    vehicle_real norm = 0;
    for (int i = 0; i < 4; ++i)
        norm += state->quaternion_xyzw[i] * state->quaternion_xyzw[i];
    if (fabs(norm - 1) > (vehicle_real)1e-3)
        return VEHICLE_INVALID_INPUT;
    const vehicle_real *r = surface->patch + 3;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            vehicle_real dot = 0;
            for (int k = 0; k < 3; ++k)
                dot += r[3 * i + k] * r[3 * j + k];
            if (fabs(dot - (i == j ? 1 : 0)) > (vehicle_real)1e-3)
                return VEHICLE_INVALID_INPUT;
        }
    }
    vehicle_real det = r[0] * (r[4] * r[8] - r[7] * r[5]) - r[3] * (r[1] * r[8] - r[7] * r[2]) +
                       r[6] * (r[1] * r[5] - r[4] * r[2]);
    if (det <= 0)
        return VEHICLE_INVALID_INPUT;
    q->projection_status = cr_project(
        state->position_world_m, surface->patch, surface->seed, surface->bounds,
        surface->tolerance_m, surface->max_iterations, q->uvZ, &q->iterations, &q->residual_m);
    if (q->projection_status != CR_PROJECT_OK)
        return VEHICLE_PROJECTION_FAILED;
    for (int i = 0; i < 3; ++i)
        if (q->uvZ[i] < surface->bounds[2 * i] || q->uvZ[i] > surface->bounds[2 * i + 1])
            q->projection_status = CR_PROJECT_EXTRAPOLATED;

    q->state = *state;
    for (int i = 0; i < 27; ++i)
        q->patch[i] = surface->patch[i];
    q->valid = 1;
    return VEHICLE_OK;
}

int vehicle_model_parameter_count(void) { return VEHICLE_MODEL_PARAMETER_COUNT; }
