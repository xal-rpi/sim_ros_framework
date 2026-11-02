#ifndef SHARED_ROLL_EXTENSION_H
#define SHARED_ROLL_EXTENSION_H
#include "roll_filter.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Augment canonical vehicle dynamics with steering_dot=command.steering_rate.
 * Brake is held at query.brake. Same state order plus steering angle; outputs
 * 15, A15x15/B15x2/M15xD. These Jacobians hold brake fixed; it is not a decision.
 * dense column-major. M may be NULL iff D=0. No allocation or retained pointers.
 * Failed calls mark provided outputs NaN. No aliasing inputs and outputs. */
int rf_roll_dynamics(const rf_query *, const vehicle_model_parameters *, const rf_command *,
                     rf_real derivative[15]);
int rf_roll_linearize(const rf_query *, const vehicle_model_parameters *, rf_real torque,
                      rf_real A[225], rf_real B[30], rf_real *M);
#ifdef __cplusplus
}
#endif
#endif
