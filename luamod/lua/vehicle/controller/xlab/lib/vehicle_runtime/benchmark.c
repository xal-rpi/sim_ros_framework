/* Host-only timing driver. The library itself has no POSIX dependency. */
#define _POSIX_C_SOURCE 200809L
#include "barrier_generated.h"
#include "projection.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#ifndef CR_MODEL_PARAMETER_COUNT
#define CR_MODEL_PARAMETER_COUNT 0
#endif

static double now_ns(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec * 1e9 + t.tv_nsec;
}
static int order(const void *a, const void *b) {
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}
int main(int argc, char **argv) {
    if (argc < 2 || argc > 3)
        return 2;
    const int full = argc == 3;
    FILE *f = fopen(argv[1], "rb");
    if (!f)
        return 3;
    fseek(f, 0, SEEK_END);
    long bytes = ftell(f);
    rewind(f);
    const int width = 17 + 27 + 10 + 2 + CR_MODEL_PARAMETER_COUNT;
    size_t n = (size_t)bytes / (width * sizeof(casadi_real));
    if (!n)
        return 4;
    casadi_real *data = malloc((size_t)bytes);
    if (!data || fread(data, 1, (size_t)bytes, f) != (size_t)bytes)
        return 5;
    fclose(f);
    casadi_real output[10], work[roll_barrier_SZ_W + 1];
    casadi_int iwork[roll_barrier_SZ_IW + 1];
    const casadi_real *args[roll_barrier_SZ_ARG];
    casadi_real *results[roll_barrier_SZ_RES] = {output};
    const size_t count = 100 * n;
    double *times = malloc(count * sizeof(double));
    if (!times)
        return 6;
    for (size_t j = 0; j < count + n; ++j) {
        casadi_real *s = data + (j % n) * width;
        args[0] = s;
        args[1] = s + 17;
        args[2] = s + 44;
        args[3] = s + 54;
#if CR_MODEL_PARAMETER_COUNT > 0
        args[4] = s + 56;
#endif
        double begin = now_ns();
        if (full) {
            casadi_real seed[3] = {s[15], s[16], (casadi_real).65};
            casadi_real bounds[6] = {-4, 4, -3, 3, (casadi_real).1, (casadi_real)1.5};
            casadi_real uvz[3], residual, terms[3];
            int iterations;
            casadi_real tolerance =
                (casadi_real)(sizeof(casadi_real) == 4 ? 2e-5 : 2e-10);
            if (cr_project(s, s + 17, seed, bounds, tolerance, 12, uvz, &iterations,
                           &residual))
                return 8;
            casadi_real evaluated[17];
            for (int k = 0; k < 15; ++k)
                evaluated[k] = s[k];
            evaluated[15] = uvz[0];
            evaluated[16] = uvz[1];
            const casadi_real *kin_args[] = {
                evaluated,
                s + 17,
                s + 54
#if CR_MODEL_PARAMETER_COUNT > 0
                ,
                s + 56
#endif
            };
            casadi_real *kin_results[1] = {terms};
            if (roll_terms(kin_args, kin_results, iwork, work, 0))
                return 9;
            if (!isfinite(terms[2]))
                return 10;
            /* This driver has no observer. The recorded bias remains the input.
             * Keep evaluated alive until the barrier call in this scope completes. */
            const casadi_real *full_args[] = {
                evaluated,
                s + 17,
                s + 44,
                s + 54
#if CR_MODEL_PARAMETER_COUNT > 0
                ,
                s + 56
#endif
            };
            int code = roll_barrier(full_args, results, iwork, work, 0);
            if (code || !isfinite(output[6]))
                return 7;
        } else {
            int code = roll_barrier(args, results, iwork, work, 0);
            if (code || !isfinite(output[6]))
                return 7;
        }
        double elapsed = now_ns() - begin;
        if (j >= n)
            times[j - n] = elapsed / 1000;
    }
    qsort(times, count, sizeof(double), order);
    printf("{\"calls\":%zu,\"median_us\":%.6f,\"p95_us\":%.6f,\"max_us\":%.6f}\n",
           count, times[count / 2], times[(size_t)(.95 * count)], times[count - 1]);
    free(times);
    free(data);
    return 0;
}
