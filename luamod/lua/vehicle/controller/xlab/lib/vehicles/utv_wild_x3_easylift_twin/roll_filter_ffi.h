typedef float vehicle_real;

enum vehicle_status {
    VEHICLE_OK = 0,
    VEHICLE_INVALID_INPUT = 2,
    VEHICLE_PROJECTION_FAILED = 3,
    VEHICLE_MODEL_FAILED = 4
};

typedef struct {
    vehicle_real position_world_m[3];
    vehicle_real quaternion_xyzw[4]; /* Hamilton, body FLU to world; unit length */
    vehicle_real velocity_body_m_s[3];
    vehicle_real angular_velocity_body_rad_s[3]; /* p, q, r */
    vehicle_real rear_peripheral_speed_m_s;
} vehicle_state;

typedef struct {
    vehicle_real steering_angle_rad;
    vehicle_real rear_torque_Nm;
    vehicle_real brake; /* calibrated model brake input, dimensionless */
} vehicle_control;

typedef struct {
    /* origin3, proper patch-to-world rotation9 column-major, quartic height15.
     * Coefficients: 1,u,v,u²,uv,v²,...,u⁴,u³v,u²v²,uv³,v⁴. Coordinates in m.
     * Owner qualifies fit accuracy, coverage, revision and model validity. */
    vehicle_real patch[27];
    vehicle_real seed[3];   /* normal-offset projection seed u,v,Z [m] */
    vehicle_real bounds[6]; /* nominal u/v/Z domain [m]; extrapolation is allowed */
    vehicle_real tolerance_m;
    int max_iterations; /* 1..24 */
} vehicle_surface;

typedef struct {
    /* Export manifest order; finite values, exact declared count, no defaults.
     * values may be NULL only for count=0. Owner enforces physical ranges and
     * independent parameter-context validity. Current inputs are constant physical
     * values, not spatial-field coefficients; no terrain frame/domain is implied. */
    const vehicle_real *values;
    int count;
} vehicle_model_parameters;

typedef struct {
    /* Read-only prepared snapshot. Owns copies; never retains input pointers.
     * Reprepare after any state or patch change; do not edit fields directly. */
    vehicle_state state;
    vehicle_real patch[27];
    vehicle_real uvZ[3];
    vehicle_real residual_m;
    int iterations;
    int projection_status; /* 0 inside, 4 extrapolated; both can have valid=1 */
    int valid;
} vehicle_query;

/* Shapes/order of derivative and Jacobian rows/columns:
 * [position3, quaternion_xyzw4, velocity_body3, angular_velocity_body3,
 *  rear_peripheral_speed1]. Derivatives have the corresponding units per second.
 * A=df/dstate14, B=df/d[steering_angle,rear_torque,brake], M=df/dparametersD.
 * Dense column-major; M may be NULL iff D=0. Parameters/patch are held fixed;
 * A includes implicit motion of the projected footpoint with world position.
 * Quaternion derivatives are ambient-coordinate derivatives, not a reduced
 * three-angle attitude Jacobian. No automatic quaternion renormalization.
 *
 * No heap/global mutable state. Do not alias inputs and outputs. Failed prepare
 * sets valid=0; failed evaluation fills provided output buffers with NaN.
 * Evaluation requires a successful prepare and validated parameters/control.
 */
int vehicle_model_parameter_count(void);
int vehicle_prepare(const vehicle_state *state, const vehicle_surface *surface,
                    vehicle_query *query);
int vehicle_evaluate(const vehicle_query *query, const vehicle_model_parameters *parameters,
                     const vehicle_control *control, vehicle_real derivative[14]);
int vehicle_jacobians(const vehicle_query *query, const vehicle_model_parameters *parameters,
                      const vehicle_control *control, vehicle_real A[196], vehicle_real B[42],
                      vehicle_real *M);


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

enum ve_status {
    VE_OK = 0,
    VE_NO_CONTEXT = 1,
    VE_BAD_PACKET = 2,
    VE_WRONG_IDENTITY = 3,
    VE_OLD_SNAPSHOT = 4,
    VE_OUTSIDE_PARAMETERS = 5,
    VE_CHANGED_REVISION = 6,
    VE_INVALID_STATE = 7,
    VE_PROJECTION_FAILED = 8
};
typedef struct {
    uint32_t frame_id;
    uint32_t epoch_id; /* explicitly configured session; never auto-adopt */
    vehicle_real projection_tolerance_m;
    int projection_max_iterations;
} ve_config;

typedef struct {
    vehicle_surface surface;
    vehicle_real parameters[1];
    vehicle_real parameter_bounds_xyz[6];
    uint64_t state_time_us;
    uint64_t geometry_source_time_us;
    uint64_t parameter_source_time_us;
    uint32_t revision;
    uint32_t geometry_revision;
    uint32_t parameter_revision;
    unsigned int flags;
    int valid;
} ve_context;

/* Caller owns all storage; no heap, socket, global state or retained pointers.
 * Reset once per configured frame/epoch. Receive validates bytes into temporary
 * storage, then replaces the complete context. Serialize calls in one consumer.
 * Bad/old packets leave the accepted context unchanged. Repeated identical
 * snapshots are accepted without changing source/assembly times. No
 * receive-time timeout or delivery lease: caller supervises producer health
 * separately. Wire size and parameter layout are fixed by the selected vehicle
 * export.
 */
int ve_abi_version(void); /* Environment API version, independent of rf_abi_version. */
size_t ve_packet_size(void);
size_t ve_context_size(void);
int ve_scalar_bytes(void);
int ve_parameter_count(void);
void ve_reset(ve_context *context);
int ve_receive(ve_context *context, const ve_config *config, const unsigned char *bytes,
               size_t length);
/* Check independent parameter XYZ bounds and surface projection. Surface domain
 * bounds are diagnostic: a regular projection outside them remains usable and
 * has query.projection_status=4, valid=1. There is no
 * absolute clock or freshness policy in model preparation. Packet timestamps
 * are retained as metadata for a consumer/supervisor that needs an age policy.
 * On failure query.valid=0. Parameter storage is owned by context; callers pass
 * {context.parameters, vehicle_model_parameter_count()} during the same call
 * chain.
 */
int ve_prepare(const ve_context *context, const ve_config *config, const vehicle_state *state,
               vehicle_query *query);
