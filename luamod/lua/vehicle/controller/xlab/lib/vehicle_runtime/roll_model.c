/* Dense, pointer-only ABI around the selected generated model. This file owns
 * no observer or execution policy. Brake is supplied context, not a decision. */
#include "barrier_generated.h"
#include "roll_filter.h"
#include "vehicle_export_config.h"
#include <math.h>

#ifdef CR_FLOAT32
#define fabs fabsf
#define fmin fminf
#define fmax fmaxf
#endif

static int finite_array(const rf_real *values, int count) {
    for (int i = 0; i < count; ++i)
        if (!isfinite(values[i]))
            return 0;
    return 1;
}

int rf_parameter_count(void) { return vehicle_model_parameter_count(); }

static int valid_parameters(const vehicle_model_parameters *p) {
    return p && p->count == rf_parameter_count() &&
           (p->count == 0 || (p->values && finite_array(p->values, p->count)));
}

static int valid_query(const rf_query *q) {
    return q && q->vehicle.valid == 1 && isfinite(q->steering_angle_rad)
        && isfinite(q->brake) && q->brake >= 0 && q->brake <= 1
        && isfinite(q->brake_rate_per_s);
}
static void pack_roll(const rf_query *q, rf_real z[17]) {
    const vehicle_state *s = &q->vehicle.state;
    for (int i = 0; i < 3; ++i)
        z[i] = s->position_world_m[i];
    for (int i = 0; i < 4; ++i)
        z[3 + i] = s->quaternion_xyzw[i];
    for (int i = 0; i < 3; ++i)
        z[7 + i] = s->velocity_body_m_s[i];
    for (int i = 0; i < 3; ++i)
        z[10 + i] = s->angular_velocity_body_rad_s[i];
    z[13] = s->rear_peripheral_speed_m_s;
    z[14] = q->steering_angle_rad;
    z[15] = q->vehicle.uvZ[0];
    z[16] = q->vehicle.uvZ[1];
}
int rf_prepare(const vehicle_query *v, rf_real steering, rf_real brake,
               rf_real brake_rate_per_s, rf_query *q) {
    if (!q)
        return RF_INVALID_INPUT;
    *q = (rf_query){0};
    if (!v || v->valid != 1 || !isfinite(steering) || !isfinite(brake)
        || brake < 0 || brake > 1 || !isfinite(brake_rate_per_s))
        return RF_INVALID_INPUT;
    q->vehicle = *v;
    q->steering_angle_rad = steering;
    q->brake = brake;
    q->brake_rate_per_s = brake_rate_per_s;
    return RF_OK;
}

int rf_measure(const rf_query *q, const vehicle_model_parameters *p, rf_measurement *m) {
    if (m)
        m->phi = m->nu = m->model_acc = NAN;
    if (!m || !valid_query(q) || !valid_parameters(p))
        return RF_INVALID_INPUT;
    /* Reserve the optional parameter slot even in parameter-free builds. */
    rf_real z[17];
    pack_roll(q, z);
    if (!finite_array(z, 17) || !finite_array(q->vehicle.patch, 27))
        return RF_INVALID_INPUT;
    const casadi_real brake[2] = {q->brake, q->brake_rate_per_s};
    const casadi_real *args[roll_terms_SZ_ARG + 1] = {z, q->vehicle.patch, brake};
    if (p->count)
        args[3] = p->values;
    casadi_real output[3];
    casadi_real *results[roll_terms_SZ_RES] = {output};
    casadi_int iw[roll_terms_SZ_IW + 1];
    casadi_real work[roll_terms_SZ_W + 1];
    if (roll_terms(args, results, iw, work, 0) || !finite_array(output, 3))
        return RF_MODEL_FAILED;
    m->phi = output[0];
    m->nu = output[1];
    m->model_acc = output[2];
    return RF_OK;
}

static int alignment_prepare(const rf_query *q, const vehicle_model_parameters *p,
                             rf_real *context, rf_alignment *out) {
    if (out)
        *out = (rf_alignment){.steering_angle_rad=NAN, .cost=NAN, .front_load_N=NAN};
    if (!out || !valid_query(q) || !valid_parameters(p))
        return RF_INVALID_INPUT;
    rf_real z[17];
    pack_roll(q, z);
    if (!finite_array(z, 17) || !finite_array(q->vehicle.patch, 27))
        return RF_INVALID_INPUT;
    const casadi_real brake[2] = {q->brake, q->brake_rate_per_s};
    const casadi_real *args[roll_alignment_context_SZ_ARG + 1] = {z, q->vehicle.patch, brake};
    if (p->count)
        args[3] = p->values;
    casadi_real *results[roll_alignment_context_SZ_RES] = {context};
    casadi_int iw[roll_alignment_context_SZ_IW + 1];
    casadi_real work[roll_alignment_context_SZ_W + 1];
    return roll_alignment_context(args, results, iw, work, 0)
        || !finite_array(context, ROLL_ALIGNMENT_CONTEXT_SIZE) ? RF_MODEL_FAILED : RF_OK;
}

static int alignment_value(const rf_real *context, rf_real angle, rf_alignment *out) {
    const casadi_real *args[roll_alignment_cost_SZ_ARG] = {context, &angle};
    casadi_real value[2], *results[roll_alignment_cost_SZ_RES] = {value};
    casadi_int iw[roll_alignment_cost_SZ_IW + 1];
    casadi_real work[roll_alignment_cost_SZ_W + 1];
    ++out->evaluations;
    if (roll_alignment_cost(args, results, iw, work, 0) || !finite_array(value, 2)) {
        out->steering_angle_rad = out->cost = out->front_load_N = NAN;
        return RF_MODEL_FAILED;
    }
    out->steering_angle_rad = angle;
    out->cost = value[0];
    out->front_load_N = value[1];
    return RF_OK;
}

int rf_alignment_evaluate(const rf_query *q, const vehicle_model_parameters *p,
                          rf_real angle, rf_alignment *out) {
    rf_real context[ROLL_ALIGNMENT_CONTEXT_SIZE];
    int status = alignment_prepare(q, p, context, out);
    if (status != RF_OK)
        return status;
    if (!isfinite(angle))
        return RF_INVALID_INPUT;
    return alignment_value(context, angle, out);
}

int rf_alignment_target(const rf_query *q, const vehicle_model_parameters *p,
                        rf_real low, rf_real high, rf_alignment *out) {
    rf_real context[ROLL_ALIGNMENT_CONTEXT_SIZE];
    int status = alignment_prepare(q, p, context, out);
    if (status != RF_OK)
        return status;
    if (!isfinite(low) || !isfinite(high) || low > high || !isfinite(high-low))
        return RF_INVALID_INPUT;
    status = alignment_value(context, fmin(high, fmax(low, q->steering_angle_rad)), out);
    if (status != RF_OK || out->front_load_N <= (rf_real)1 || low == high)
        return status;
    rf_alignment best = *out;
    if (high-low <= (rf_real)1e-5) {
        if (alignment_value(context,low,out) != RF_OK) return RF_MODEL_FAILED;
        if (out->cost < best.cost) best=*out;
        if (alignment_value(context,high,out) != RF_OK) return RF_MODEL_FAILED;
        if (out->cost < best.cost) best=*out;
        best.evaluations=out->evaluations;
        *out=best;
        return RF_OK;
    }
    int best_index = -1;
    const rf_real spacing = (high-low)/(rf_real)52;
    for (int i=0; i<=52; ++i) {
        status = alignment_value(context, i==52 ? high : low+spacing*(rf_real)i, out);
        if (status != RF_OK)
            return status;
        if (out->cost < best.cost) { best=*out; best_index=i; }
    }
    /* Include the initial angle as an incumbent; refine its cell if it won. */
    if (best_index < 0)
        best_index = (int)((best.steering_angle_rad-low)/spacing);
    rf_real lo = best_index>0 ? low+spacing*(rf_real)(best_index-1) : low;
    rf_real hi = best_index<52 ? low+spacing*(rf_real)(best_index+1) : high;
    const rf_real golden = (rf_real)0.6180339887498948482;
    rf_real a=hi-golden*(hi-lo), b=lo+golden*(hi-lo);
    if (alignment_value(context,a,out) != RF_OK) return RF_MODEL_FAILED;
    rf_real fa=out->cost;
    if (fa<best.cost) best=*out;
    if (alignment_value(context,b,out) != RF_OK) return RF_MODEL_FAILED;
    rf_real fb=out->cost;
    if (fb<best.cost) best=*out;
    for (int i=0; i<32 && hi-lo>(rf_real)1e-5; ++i) {
        rf_real angle;
        int left = fa<fb;
        if (left) { hi=b; b=a; fb=fa; a=hi-golden*(hi-lo); angle=a; }
        else { lo=a; a=b; fa=fb; b=lo+golden*(hi-lo); angle=b; }
        if (alignment_value(context,angle,out) != RF_OK) return RF_MODEL_FAILED;
        if (left) fa=out->cost; else fb=out->cost;
        if (out->cost<best.cost) best=*out;
    }
    best.evaluations = out->evaluations;
    *out = best;
    return RF_OK;
}

static int valid_settings(const rf_settings *s) {
    if (!s || !isfinite(s->tau_s) || s->tau_s <= 0 || !isfinite(s->phi_limit_rad) ||
        s->phi_limit_rad <= 0 || s->phi_limit_rad >= (rf_real)1.5707963267948966 ||
        !isfinite(s->alpha_per_s) || s->alpha_per_s <= 0 || !finite_array(s->P, 6))
        return 0;
    /* Cholesky pivots test SPD without squaring the overall matrix scale. */
    const rf_real *p = s->P;
    if (p[0] <= 0)
        return 0;
    rf_real d2 = p[3] - p[1] * (p[1] / p[0]);
    rf_real off = p[4] - p[1] * (p[2] / p[0]);
    if (!isfinite(d2) || d2 <= 0)
        return 0;
    rf_real d3 = p[5] - p[2] * (p[2] / p[0]) - off * (off / d2);
    return isfinite(d3) && d3 > 0;
}

int rf_filter_step(const rf_query *q, const vehicle_model_parameters *p, const rf_settings *s,
                   const rf_bias *bias, const rf_bounds *bounds, const rf_command *nominal,
                   rf_command *filtered, rf_diagnostics *d) {
    if (filtered)
        filtered->steering_rate = filtered->torque = NAN;
    if (d) {
        *d = (rf_diagnostics){.measurement = {NAN, NAN, NAN},
                              .corrected_acc = NAN,
                              .jerk_zero = NAN,
                              .h = NAN,
                              .score = NAN,
                              .row = {NAN, NAN, NAN},
                              .qp = {.status = RF_INVALID_INPUT},
                              .status = RF_INVALID_INPUT};
    }
    if (!filtered || !d || !valid_query(q) || !valid_parameters(p) || !valid_settings(s) || !bias ||
        (bias->valid != 0 && bias->valid != 1) || (bias->apply != 0 && bias->apply != 1))
        return RF_INVALID_INPUT;
    d->bias_applied = bias->valid && bias->apply;
    if (d->bias_applied && !isfinite(bias->value))
        return RF_INVALID_INPUT;
    casadi_real settings[10] = {d->bias_applied ? bias->value : 0, s->tau_s, s->phi_limit_rad,
                                s->alpha_per_s};
    for (int i = 0; i < 6; ++i)
        settings[i + 4] = s->P[i];
    rf_real z[17];
    pack_roll(q, z);
    if (!finite_array(z, 17) || !finite_array(q->vehicle.patch, 27))
        return RF_INVALID_INPUT;
    const casadi_real brake[2] = {q->brake, q->brake_rate_per_s};
    const casadi_real *args[roll_barrier_SZ_ARG + 1] = {z, q->vehicle.patch, settings, brake};
    if (p->count)
        args[4] = p->values;
    casadi_real output[10];
    casadi_real *results[roll_barrier_SZ_RES] = {output};
    casadi_int iw[roll_barrier_SZ_IW + 1];
    casadi_real work[roll_barrier_SZ_W + 1];
    if (roll_barrier(args, results, iw, work, 0) || !finite_array(output, 10)) {
        d->status = RF_MODEL_FAILED;
        return d->status;
    }
    d->measurement.phi = output[0];
    d->measurement.nu = output[1];
    d->measurement.model_acc = output[2];
    d->corrected_acc = output[3];
    d->jerk_zero = output[4];
    d->h = output[5];
    d->row.b = output[6];
    d->row.g_rate = output[7];
    d->row.g_torque = output[8];
    d->score = output[9];
    d->status = rf_solve_qp(&d->row, nominal, bounds, filtered, &d->qp);
    return d->status;
}
