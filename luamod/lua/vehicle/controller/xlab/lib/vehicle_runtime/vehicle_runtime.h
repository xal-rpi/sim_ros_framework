#ifndef VEHICLE_RUNTIME_H
#define VEHICLE_RUNTIME_H

/* Public C99 API. Build all translation units with the same CR_FLOAT32 setting.
 * No generated headers, CasADi types, roll filter, platform or heap dependency
 * leaks through this header. All pointers are borrowed only for the call.
 */
#ifdef CR_FLOAT32
typedef float vehicle_real;
#else
typedef double vehicle_real;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* FFI_BEGIN */
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

/* FFI_END */
#ifdef __cplusplus
}
#endif
#endif
