#ifndef CARTESIAN_ROLL_FILTER_H
#define CARTESIAN_ROLL_FILTER_H

/* Build with CR_FLOAT32 exactly when the generated model uses float. The build
 * writes a precision-specific roll_filter_ffi.h for ffi.cdef; do not preprocess
 * this header in Lua or pass structs by value across the FFI boundary.
 */
#include "vehicle_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

/* FFI_BEGIN */
typedef vehicle_real rf_real;
enum rf_status {
    RF_OK = 0,
    RF_INFEASIBLE = 1,
    RF_INVALID_INPUT = 2,
    RF_PROJECTION_FAILED = 3,
    RF_MODEL_FAILED = 4,
    RF_NUMERICAL_FAILURE = 5
};

/* Physical state, surface and parameters belong to vehicle_runtime.h. */
typedef struct {
    vehicle_query vehicle;
    rf_real steering_angle_rad;
    rf_real brake;           /* supplied service-brake input [0,1], not a decision */
    rf_real brake_rate_per_s; /* supplied continuous rate; zero for a held input */
} rf_query;

typedef struct {
    rf_real phi;       /* surface-relative angle [rad] */
    rf_real nu;        /* geometric angle derivative [rad/s], generally != p */
    rf_real model_acc; /* [rad/s^2], steering rate=0 and torque=0 */
} rf_measurement;

typedef struct {
    rf_real steering_angle_rad;
    rf_real cost;          /* load-weighted squared front slip angle [rad^2] */
    rf_real front_load_N;  /* model-predicted sum, not measured contact force */
    int evaluations;
} rf_alignment;

typedef struct {
    rf_real tau_s;
    rf_real phi_limit_rad;
    rf_real alpha_per_s;
    rf_real P[6]; /* symmetric SPD: 11,12,13,22,23,33; dimensionless */
} rf_settings;

typedef struct {
    rf_real value; /* frozen p-dot disturbance [rad/s^2] */
    int valid;
    int apply; /* application toggle; does not alter any observer state */
} rf_bias;

typedef struct {
    rf_real steering_rate; /* rad/s */
    rf_real torque;        /* rear wheel torque, Nm */
} rf_command;

typedef struct {
    rf_command low;
    rf_command high;
    /* Positive physical scales: objective=sum(((u-u_nom)/scale)^2).
     * These are policy weights, not state normalization or speed weights. */
    rf_command scale;
} rf_bounds;

typedef struct {
    /* b + g_rate*steering_rate + g_torque*torque >= 0. No torque anchor. */
    rf_real b;
    rf_real g_rate;
    rf_real g_torque;
} rf_row;

typedef struct {
    rf_real nominal_margin; /* at the caller's unmodified nominal command */
    rf_real margin;
    rf_real maximum_margin;
    rf_real scaled_objective;
    rf_real row_scale;          /* normalization applied to the row, not a margin */
    rf_real roundoff_tolerance; /* physical row units; not a robustness bound */
    int intervened;
    int status;
} rf_qp_diagnostics;

typedef struct {
    rf_measurement measurement;
    rf_real corrected_acc;
    rf_real jerk_zero;
    rf_real h;
    rf_real score;
    rf_row row;
    rf_qp_diagnostics qp;
    int bias_applied;
    int status;
} rf_diagnostics;

/* Observer state is caller-owned and independent of filter/application toggles.
 * No timestamp or platform clock is stored. The adapter computes dt from its
 * integer timestamps before converting that short interval to rf_real seconds.
 * Coefficients are cached for exactly repeated gain/dt, without time quantization.
 * No implicit domain/speed gate. */
enum rf_observer_status {
    RF_OBSERVER_UPDATED = 0,
    RF_OBSERVER_INITIALIZED = 1,
    RF_OBSERVER_GAP_RESET = 2,
    RF_OBSERVER_DUPLICATE = 3,
    RF_OBSERVER_REGRESSED = 4,
    RF_OBSERVER_INVALID = 5,
    RF_OBSERVER_GAIN_RESET = 6
};
typedef struct {
    rf_real xi;
    rf_real q;
    rf_real bias;
    rf_real gain;
    rf_real cached_dt_s;
    rf_real one_minus_decay;
    rf_real ramp;
    int initialized;
    int valid;
    int coefficients_valid;
} rf_observer;

int rf_abi_version(void);
int rf_scalar_bytes(void);
int rf_parameter_count(void);

/* All functions use caller-owned buffers, allocate no heap, retain no pointers
 * and have no global mutable state. Do not alias input and output objects.
 * Failed commands/measurements are NaN and must not be executed/consumed.
 * RF_INFEASIBLE alone returns a bounded max-margin command, NOT a safe backup.
 */
/* Reuse a successfully prepared vehicle snapshot; no second projection.
 * Copies the snapshot and adds steering [rad] and service brake [0,1].
 * brake_rate_per_s is an exogenous continuous rate, not a control variable.
 * Held commands use zero. Brake steps require evaluation at the post-step value;
 * the continuous row does not certify membership across such a score jump. */
int rf_prepare(const vehicle_query *vehicle, rf_real steering_angle_rad,
               rf_real brake, rf_real brake_rate_per_s, rf_query *query);
int rf_measure(const rf_query *query, const vehicle_model_parameters *parameters,
               rf_measurement *measurement);
/* Same prepared surface/model context as the barrier; no second projection.
 * Supplied brake is used; front torque is zero. Geometry, hub motion and
 * loads independent of trial steering are evaluated once per call. Evaluate
 * exposes the scalar objective; target searches the supplied steering interval
 * (53 samples plus bounded refinement to 1e-5 rad). This is a model alignment
 * policy, not a force-zeroing or global-optimality certificate. With <=1 N front
 * load, target retains current steering projected into the interval. No heap,
 * retained search state, hard-coded vehicle bounds or platform I/O.
 */
int rf_alignment_evaluate(const rf_query *query, const vehicle_model_parameters *parameters,
                          rf_real steering_angle_rad, rf_alignment *alignment);
int rf_alignment_target(const rf_query *query, const vehicle_model_parameters *parameters,
                        rf_real steering_low_rad, rf_real steering_high_rad,
                        rf_alignment *alignment);
int rf_solve_qp(const rf_row *row, const rf_command *nominal, const rf_bounds *bounds,
                rf_command *filtered, rf_qp_diagnostics *diagnostics);
int rf_filter_step(const rf_query *query, const vehicle_model_parameters *parameters,
                   const rf_settings *settings, const rf_bias *bias, const rf_bounds *bounds,
                   const rf_command *nominal, rf_command *filtered, rf_diagnostics *diagnostics);

/* Reset explicitly after a model/frame discontinuity; routine patch changes do
 * not reset automatically. gain>0 [1/s], max_gap_s>0 [s], dt_s finite [s]. A zero
 * interval is allowed only for the first sample; subsequent zero/negative
 * intervals reject the sample. Gaps and gain changes reinitialize with zero bias
 * and an invalid estimate for that call. Nonfinite/zero/negative rejected samples
 * do not change xi/q/bias or the coefficient cache, but mark the estimate invalid.
 */
void rf_observer_reset(rf_observer *observer);
int rf_observer_update(rf_observer *observer, rf_real dt_s, rf_real nu, rf_real model_acc,
                       rf_real gain, rf_real max_gap_s);
/* FFI_END */

#ifdef __cplusplus
}
#endif
#endif
