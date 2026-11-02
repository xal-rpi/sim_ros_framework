/* Host-only cache instrumentation and batch timings. Not linked into the filter
 * library or PX4. GNU ld --wrap counts actual coefficient evaluations. */
#define _POSIX_C_SOURCE 199309L
#include "roll_filter.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <time.h>

static unsigned expm1_calls;
float __real_expm1f(float value);
float __wrap_expm1f(float value) {
    ++expm1_calls;
    return __real_expm1f(value);
}

static double now_s(void) {
    struct timespec time;
    assert(clock_gettime(CLOCK_MONOTONIC, &time) == 0);
    return time.tv_sec + time.tv_nsec * 1e-9;
}

int main(void) {
    rf_observer observer;
    rf_observer_reset(&observer);
    assert(rf_observer_update(&observer, 0, 1, 0, 10, 0.05f) == RF_OBSERVER_INITIALIZED);
    for (int i = 0; i < 1000; ++i)
        assert(rf_observer_update(&observer, 0.005f, 1, 0, 10, 0.05f) == RF_OBSERVER_UPDATED);
    assert(expm1_calls == 1);
    assert(rf_observer_update(&observer, 0.01f, 1, 0, 10, 0.05f) == RF_OBSERVER_UPDATED);
    assert(expm1_calls == 2);
    assert(rf_observer_update(&observer, 0.01f, 1, 0, 20, 0.05f) == RF_OBSERVER_GAIN_RESET);
    assert(expm1_calls == 2);
    assert(rf_observer_update(&observer, 0.01f, 1, 0, 20, 0.05f) == RF_OBSERVER_UPDATED);
    assert(expm1_calls == 3 && fabsf(observer.bias) < 1e-6f);

    const int iterations = 200000;
    double start = now_s();
    for (int i = 0; i < iterations; ++i)
        assert(rf_observer_update(&observer, 0.01f, 0.2f, 0.1f, 20, 0.05f) == RF_OBSERVER_UPDATED);
    double fixed_ns = (now_s() - start) * 1e9 / iterations;
    unsigned before = expm1_calls;
    start = now_s();
    for (int i = 0; i < iterations; ++i)
        assert(rf_observer_update(&observer, i % 2 ? 0.005f : 0.006f, 0.2f, 0.1f, 20, 0.05f) ==
               RF_OBSERVER_UPDATED);
    double variable_ns = (now_s() - start) * 1e9 / iterations;
    assert(expm1_calls - before == (unsigned)iterations);

    rf_row row = {-0.2f, 1, 0.0001f};
    rf_command nominal = {-0.2f, 500}, filtered;
    rf_bounds bounds = {{-0.9f, -1500}, {0.9f, 1500}, {1, 500}};
    rf_qp_diagnostics diagnostics;
    start = now_s();
    for (int i = 0; i < iterations; ++i)
        assert(rf_solve_qp(&row, &nominal, &bounds, &filtered, &diagnostics) == RF_OK);
    double qp_ns = (now_s() - start) * 1e9 / iterations;
    assert(diagnostics.margin >= -diagnostics.roundoff_tolerance);
    printf("{\"fixed_interval_expm1_calls_per_1000\":1,"
           "\"cache_invalidation\":\"passed\",\"iterations\":%d,"
           "\"observer_cached_mean_ns\":%.3f,\"observer_variable_dt_mean_ns\":%.3f,"
           "\"qp_active_mean_ns\":%.3f}\n",
           iterations, fixed_ns, variable_ns, qp_ns);
    return 0;
}
