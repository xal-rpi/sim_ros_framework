#include "roll_filter.h"
#include <float.h>
#include <math.h>

#ifdef CR_FLOAT32
#define RF_EPS FLT_EPSILON
#define rf_expm1 expm1f
#define rf_fma fmaf
#define fabs fabsf
#define fmax fmaxf
#else
#define RF_EPS DBL_EPSILON
#define rf_expm1 expm1
#define rf_fma fma
#endif

static rf_real clip(rf_real x, rf_real low, rf_real high) {
    return x < low ? low : x > high ? high : x;
}

static rf_real margin(const rf_row *row, rf_real x, rf_real t) {
    return rf_fma(row->g_rate, x, rf_fma(row->g_torque, t, row->b));
}

static rf_real cost(const rf_command *u, const rf_command *nominal, const rf_bounds *bounds) {
    rf_real x = (u->steering_rate - nominal->steering_rate) / bounds->scale.steering_rate;
    rf_real t = (u->torque - nominal->torque) / bounds->scale.torque;
    return x * x + t * t;
}

/* Retain product roundoff when two large contributions nearly cancel. */
static rf_real dot2(rf_real a, rf_real b, rf_real c, rf_real d) {
    rf_real product = c * d;
    return rf_fma(a, b, product) + rf_fma(c, d, -product);
}

static int finish(int status, const rf_row *row, const rf_command *nominal, const rf_bounds *bounds,
                  rf_command *u, rf_qp_diagnostics *d) {
    d->margin = margin(row, u->steering_rate, u->torque);
    d->scaled_objective = cost(u, nominal, bounds);
    d->intervened = u->steering_rate != nominal->steering_rate || u->torque != nominal->torque;
    if (!isfinite(d->margin) || !isfinite(d->scaled_objective)) {
        u->steering_rate = u->torque = NAN;
        status = RF_NUMERICAL_FAILURE;
    }
    d->status = status;
    return status;
}

int rf_abi_version(void) { return 4; }
int rf_scalar_bytes(void) { return (int)sizeof(rf_real); }

int rf_solve_qp(const rf_row *row, const rf_command *nominal, const rf_bounds *bounds,
                rf_command *filtered, rf_qp_diagnostics *d) {
    if (filtered)
        filtered->steering_rate = filtered->torque = NAN;
    if (d) {
        *d = (rf_qp_diagnostics){.nominal_margin = NAN,
                                 .margin = NAN,
                                 .maximum_margin = NAN,
                                 .scaled_objective = NAN,
                                 .row_scale = NAN,
                                 .roundoff_tolerance = NAN,
                                 .status = RF_INVALID_INPUT};
    }
    if (!row || !nominal || !bounds || !filtered || !d)
        return RF_INVALID_INPUT;
    rf_real values[] = {row->b,
                        row->g_rate,
                        row->g_torque,
                        nominal->steering_rate,
                        nominal->torque,
                        bounds->low.steering_rate,
                        bounds->low.torque,
                        bounds->high.steering_rate,
                        bounds->high.torque,
                        bounds->scale.steering_rate,
                        bounds->scale.torque};
    for (unsigned i = 0; i < sizeof(values) / sizeof(values[0]); ++i)
        if (!isfinite(values[i]))
            return RF_INVALID_INPUT;
    if (bounds->low.steering_rate > bounds->high.steering_rate ||
        bounds->low.torque > bounds->high.torque || bounds->scale.steering_rate <= 0 ||
        bounds->scale.torque <= 0)
        return RF_INVALID_INPUT;

    rf_command boxed = {
        clip(nominal->steering_rate, bounds->low.steering_rate, bounds->high.steering_rate),
        clip(nominal->torque, bounds->low.torque, bounds->high.torque)};
    rf_command maximum = {row->g_rate > 0   ? bounds->high.steering_rate
                          : row->g_rate < 0 ? bounds->low.steering_rate
                                            : boxed.steering_rate,
                          row->g_torque > 0   ? bounds->high.torque
                          : row->g_torque < 0 ? bounds->low.torque
                                              : boxed.torque};
    d->nominal_margin = margin(row, nominal->steering_rate, nominal->torque);
    d->maximum_margin = margin(row, maximum.steering_rate, maximum.torque);
    /* Scale around the boxed nominal point in dimensionless decision coordinates.
     * Reconstructing a boundary point relative to a distant nominal loses digits.
     * Row normalization does not change feasibility or the minimizing command. */
    rf_real a = row->g_rate * bounds->scale.steering_rate;
    rf_real c = row->g_torque * bounds->scale.torque;
    rf_real raw_a = a, raw_c = c;
    rf_real b = margin(row, boxed.steering_rate, boxed.torque);
    rf_real normalization = fmax(fabs(a), fmax(fabs(c), fabs(b)));
    rf_real physical_scale =
        fabs(row->b) +
        fabs(row->g_rate) *
            fmax(fabs(bounds->low.steering_rate), fabs(bounds->high.steering_rate)) +
        fabs(row->g_torque) * fmax(fabs(bounds->low.torque), fabs(bounds->high.torque));
    d->row_scale = normalization;
    d->roundoff_tolerance = 16 * RF_EPS * physical_scale;
    if (!isfinite(normalization) || !isfinite(physical_scale) || !isfinite(d->nominal_margin) ||
        !isfinite(d->maximum_margin)) {
        d->status = RF_NUMERICAL_FAILURE;
        return d->status;
    }
    if (d->maximum_margin < 0) {
        *filtered = maximum;
        return finish(RF_INFEASIBLE, row, nominal, bounds, filtered, d);
    }
    if (margin(row, boxed.steering_rate, boxed.torque) >= 0) {
        *filtered = boxed;
        return finish(RF_OK, row, nominal, bounds, filtered, d);
    }
    /* The box projection violates the row, so the optimum is on its boundary.
     * Enumerate the orthogonal projection and all four edge intersections. */
    if (normalization == 0) {
        d->status = RF_NUMERICAL_FAILURE;
        return d->status;
    }
    a /= normalization;
    c /= normalization;
    b /= normalization;
    rf_real lo[2] = {(bounds->low.steering_rate - boxed.steering_rate) /
                         bounds->scale.steering_rate,
                     (bounds->low.torque - boxed.torque) / bounds->scale.torque};
    rf_real hi[2] = {(bounds->high.steering_rate - boxed.steering_rate) /
                         bounds->scale.steering_rate,
                     (bounds->high.torque - boxed.torque) / bounds->scale.torque};
    rf_real candidates[5][2];
    int count = 0;
    /* Use raw coefficients in cancelling products. Normalizing each first
     * could rotate the tangent by roundoff amplified by a distant nominal. */
    rf_real norm_scale = fmax(fabs(raw_a), fabs(raw_c));
    rf_real desired_tangent = dot2(-raw_c, nominal->steering_rate / bounds->scale.steering_rate,
                                   raw_a, nominal->torque / bounds->scale.torque) /
                              norm_scale;
    rf_real nx = raw_a / norm_scale, nt = raw_c / norm_scale;
    rf_real norm_squared = nx * nx + nt * nt;
    rf_real offset = row->b / norm_scale;
    if (!isfinite(desired_tangent) || !isfinite(offset)) {
        d->status = RF_NUMERICAL_FAILURE;
        return d->status;
    }
    candidates[count][0] = rf_fma(-nt, desired_tangent, -offset * nx) / norm_squared -
                           boxed.steering_rate / bounds->scale.steering_rate;
    candidates[count++][1] = rf_fma(nx, desired_tangent, -offset * nt) / norm_squared -
                             boxed.torque / bounds->scale.torque;
    if (c != 0) {
        for (int i = 0; i < 2; ++i) {
            candidates[count][0] = i ? hi[0] : lo[0];
            candidates[count][1] = -(b + a * candidates[count][0]) / c;
            ++count;
        }
    }
    if (a != 0) {
        for (int i = 0; i < 2; ++i) {
            candidates[count][1] = i ? hi[1] : lo[1];
            candidates[count][0] = -(b + c * candidates[count][1]) / a;
            ++count;
        }
    }
    *filtered = maximum;
    int found = 0;
    rf_real best_tangent = 0;
    for (int i = 0; i < count; ++i) {
        rf_real x = candidates[i][0], t = candidates[i][1];
        rf_real ex = 16 * RF_EPS * fmax(1, fmax(fabs(lo[0]), fabs(hi[0])));
        rf_real et = 16 * RF_EPS * fmax(1, fmax(fabs(lo[1]), fabs(hi[1])));
        if (!isfinite(x) || !isfinite(t) || x < lo[0] - ex || x > hi[0] + ex || t < lo[1] - et ||
            t > hi[1] + et)
            continue;
        rf_command u = {
            clip(boxed.steering_rate + bounds->scale.steering_rate * x, bounds->low.steering_rate,
                 bounds->high.steering_rate),
            clip(boxed.torque + bounds->scale.torque * t, bounds->low.torque, bounds->high.torque)};
        rf_real m = margin(row, u.steering_rate, u.torque);
        rf_real tangent = dot2(-raw_c, u.steering_rate / bounds->scale.steering_rate, raw_a,
                               u.torque / bounds->scale.torque) /
                          norm_scale;
        /* On the active row the normal cost is constant. Compare tangent
         * distances without subtracting huge squared objective values. */
        rf_real difference =
            (tangent - best_tangent) * (tangent + best_tangent - 2 * desired_tangent);
        if (m >= -d->roundoff_tolerance && (!found || difference < 0)) {
            *filtered = u;
            best_tangent = tangent;
            found = 1;
        }
    }
    if (!found) {
        filtered->steering_rate = filtered->torque = NAN;
        d->status = RF_NUMERICAL_FAILURE;
        return d->status;
    }
    return finish(RF_OK, row, nominal, bounds, filtered, d);
}

void rf_observer_reset(rf_observer *observer) {
    if (observer)
        *observer = (rf_observer){0};
}

int rf_observer_update(rf_observer *o, rf_real dt_s, rf_real nu, rf_real model_acc, rf_real gain,
                       rf_real max_gap_s) {
    if (!o)
        return RF_OBSERVER_INVALID;
    o->valid = 0;
    if (!isfinite(dt_s) || !isfinite(nu) || !isfinite(model_acc) || !isfinite(gain) || gain <= 0 ||
        !isfinite(max_gap_s) || max_gap_s <= 0)
        return RF_OBSERVER_INVALID;
    rf_real q = model_acc + gain * nu;
    if (!isfinite(q))
        return RF_OBSERVER_INVALID;
    if (dt_s < 0)
        return RF_OBSERVER_REGRESSED;
    if (o->initialized && dt_s == 0)
        return RF_OBSERVER_DUPLICATE;
    if (!o->initialized || gain != o->gain || dt_s > max_gap_s) {
        int status = !o->initialized   ? RF_OBSERVER_INITIALIZED
                     : gain != o->gain ? RF_OBSERVER_GAIN_RESET
                                       : RF_OBSERVER_GAP_RESET;
        o->xi = gain * nu;
        o->q = q;
        o->bias = 0;
        o->gain = gain;
        o->initialized = 1;
        o->coefficients_valid = 0;
        return status;
    }
    if (!o->coefficients_valid || dt_s != o->cached_dt_s) {
        rf_real z = gain * dt_s;
        if (!isfinite(z))
            return RF_OBSERVER_INVALID;
        o->one_minus_decay = -rf_expm1(-z);
        o->ramp = z < (rf_real)1e-3 ? z / 2 - z * z / 6 + z * z * z / 24 - z * z * z * z / 120
                                    : 1 - o->one_minus_decay / z;
        o->cached_dt_s = dt_s;
        o->coefficients_valid = 1;
    }
    /* Exact first-order-held integration, written as increments: exp(-z) is
     * unnecessary because decay = 1 - one_minus_decay. */
    rf_real xi = o->xi + o->one_minus_decay * (o->q - o->xi) + o->ramp * (q - o->q);
    rf_real bias = gain * nu - xi;
    if (!isfinite(xi) || !isfinite(bias))
        return RF_OBSERVER_INVALID;
    o->xi = xi;
    o->bias = bias;
    o->q = q;
    o->valid = 1;
    return RF_OBSERVER_UPDATED;
}
