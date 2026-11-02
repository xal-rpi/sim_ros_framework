#include "projection.h"
#include <math.h>
#ifdef CR_FLOAT32
#define cr_sqrt sqrtf
#define cr_fmax fmaxf
#else
#define cr_sqrt sqrt
#define cr_fmax fmax
#endif

static casadi_real dot3(const casadi_real a[3], const casadi_real b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void cross3(const casadi_real a[3], const casadi_real b[3], casadi_real c[3]) {
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

int cr_project(const casadi_real position[3], const casadi_real patch[27],
               const casadi_real seed[3], const casadi_real bounds[6], casadi_real tolerance_m,
               int max_iterations, casadi_real uvZ[3], int *iterations, casadi_real *residual_m) {
    casadi_real q[3], point[3], jac[9], residual[3], bc[3], ca[3], ab[3];
    casadi_real work[patch_offset_SZ_W + 1];
    casadi_int iwork[patch_offset_SZ_IW + 1];
    const casadi_real *args[patch_offset_SZ_ARG] = {q, patch};
    casadi_real *results[patch_offset_SZ_RES] = {point, jac};
    if (!position || !patch || !seed || !bounds || !uvZ || !iterations || !residual_m ||
        !isfinite(tolerance_m) || tolerance_m <= 0 || max_iterations < 1 || max_iterations > 24)
        return CR_PROJECT_BAD_INPUT;
    for (int i = 0; i < 27; ++i)
        if (!isfinite(patch[i]))
            return CR_PROJECT_BAD_INPUT;
    for (int i = 0; i < 6; ++i)
        if (!isfinite(bounds[i]))
            return CR_PROJECT_BAD_INPUT;
    for (int i = 0; i < 3; ++i) {
        if (!isfinite(position[i]) || !isfinite(seed[i]) || bounds[2 * i] > bounds[2 * i + 1])
            return CR_PROJECT_BAD_INPUT;
        q[i] = seed[i];
    }
    *iterations = 0;
    for (int it = 0; it <= max_iterations; ++it) {
        if (patch_offset(args, results, iwork, work, 0))
            return CR_PROJECT_BAD_INPUT;
        for (int i = 0; i < 3; ++i)
            residual[i] = point[i] - position[i];
        *residual_m = cr_sqrt(dot3(residual, residual));
        if (!isfinite(*residual_m))
            return CR_PROJECT_BAD_INPUT;
        *iterations = it;
        /* Check chart regularity even when the initial seed already fits. */
        cross3(jac + 3, jac + 6, bc);
        cross3(jac + 6, jac, ca);
        cross3(jac, jac + 3, ab);
        casadi_real det = dot3(jac, bc);
        /* Use the largest column scale, not the product of column norms: a focal
         * point can collapse one column without making the remaining columns
         * parallel. The product would hide precisely that ill-conditioning. */
        casadi_real largest = cr_sqrt(
            cr_fmax(dot3(jac, jac), cr_fmax(dot3(jac + 3, jac + 3), dot3(jac + 6, jac + 6))));
        casadi_real scale = largest * largest * largest;
        if (!isfinite(det) || det <= (casadi_real)1e-8 * scale || scale == 0)
            return CR_PROJECT_SINGULAR;
        if (*residual_m <= tolerance_m) {
            for (int i = 0; i < 3; ++i)
                uvZ[i] = q[i];
            return CR_PROJECT_OK;
        }
        if (it == max_iterations)
            break;
        /* CasADi matrices are column-major. Cramer's row products give the inverse
         * action without heap allocation or a generic linear algebra dependency.
         */
        q[0] -= dot3(bc, residual) / det;
        q[1] -= dot3(ca, residual) / det;
        q[2] -= dot3(ab, residual) / det;
    }
    return CR_PROJECT_NO_CONVERGENCE;
}
