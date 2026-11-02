#ifndef CARTESIAN_ROLL_PROJECTION_H
#define CARTESIAN_ROLL_PROJECTION_H

/* geometry_generated.h selects casadi_real (float or double) for this build.
 * Do not mix precision variants in one process under the same symbol names.
 * This header and implementation require only C99 and the generated geometry.
 */
#include "geometry_generated.h"

#ifdef __cplusplus
extern "C" {
#endif

enum cr_projection_status {
    CR_PROJECT_OK = 0,
    CR_PROJECT_BAD_INPUT = 1,
    CR_PROJECT_SINGULAR = 2,
    CR_PROJECT_NO_CONVERGENCE = 3,
    CR_PROJECT_EXTRAPOLATED = 4 /* vehicle_query diagnostic; projection succeeded */
};

/* Orthogonal projection onto X(u,v)+Z*n(u,v).
 *
 * position: world xyz [m]. patch: origin[3], R_WP[9] column-major, H coefficients[15].
 * seed: warm-start [u,v,Z] in metres; no persistent global state is used.
 * bounds: [u_min,u_max,v_min,v_max,Z_min,Z_max], nominal fit/offset domain.
 * Validate their shape/order, but continue the polynomial outside this domain.
 * cr_project returns CR_PROJECT_OK for any finite, regular converged solution;
 * vehicle_prepare separately marks outside-domain queries as extrapolated.
 * tolerance_m: Cartesian residual tolerance; caller chooses for its precision.
 * max_iterations: 1..24, bounding Newton work. All scratch is fixed-size stack.
 *
 * Caller must validate patch rotation/fit quality and maintain coverage. A zero
 * return only establishes a local projection residual/regularity check, not map
 * accuracy, global uniqueness, contact, or safety. Failed outputs are unusable.
 */
int cr_project(const casadi_real position[3], const casadi_real patch[27],
               const casadi_real seed[3], const casadi_real bounds[6], casadi_real tolerance_m,
               int max_iterations, casadi_real uvZ[3], int *iterations, casadi_real *residual_m);

#ifdef __cplusplus
}
#endif
#endif
