/* Auto-generated rwd_struct: T = g(x) + Σ I_k(u) F_k(x).
 *
 * One MLP eval on (ω_e, boost, ω_w). Inverse bisects the mix (12 steps).
 * drivetrain_inverse_throttle_knots is a 9-knot scan of the same polynomial.
 *
 * Wheel ODE (axle-effective, Convention B). State v = ω R_kin [m/s]:
 *
 *   J_eff · v̇ = R_kin · (T_drive + R · (−F_tire + F_brake))
 *
 *   T_drive  = drivetrain_forward_torque(ω_e, u, boost, ω_w)
 *   ω_w      = v / R_kin
 *   F_brake  = −brake · brake_coeff_rear     (rear axle, N)
 *   T_brake  = R · brake · brake_coeff_rear  (rear hub, N·m)
 *   R        = drivetrain_r           contact / brake lever
 *   R_kin    = drivetrain_r_kin       kinematics / features
 *   J_eff    = drivetrain_j_eff       = J / (corr_phys · wheel_corr_scale)
 *
 * F_tire is the contact model (not in this file).
 *
 * CVT occupancy (estimator only; invert does not use these):
 *   catalog: /home/franck/Documents/xal_rpi/xal_vehicle_models/xal_vehicle_models/generated/identification/easylift_confirmation_v2/catalogs/torque
 *   drivetrain_we_band(ω_w) → ω_e ∈ [i_lo(ω_w), i_hi(ω_w)] · ω_w
 *   drivetrain_ww_band(ω_e) → ω_w ∈ [ω_e/i_hi(ω_e), ω_e/i_lo(ω_e)]
 * i(ω_w) and i(ω_e) are percentile slices of the same catalog, not inverses.
 * Rails are not very good below / above these speeds (clutch / sparse tails):
 *   ω_w < 9.07 rad/s or ω_w > 84.79 rad/s
 *   ω_e < 189.75 rad/s or ω_e > 715.95 rad/s
 */
#include "utv_canam_r322_lift_wheel_torque_identified_20261028.h"
#include <math.h>

#ifndef DRIVETRAIN_U_EPS
#define DRIVETRAIN_U_EPS 0.0010000f
#define DRIVETRAIN_T_DEN 0.0000010f
#endif

static float drivetrain_relu(float x) { return x > 0.0f ? x : 0.0f; }

static float drivetrain_softplus(float x) {
    float ax = x < 0.0f ? -x : x;
    float pos = x > 0.0f ? x : 0.0f;
    return pos + logf(1.0f + expf(-ax));
}

const float drivetrain_j_eff = 14.592795193331822f;
const float drivetrain_r = 0.32401257f;
const float drivetrain_r_kin = 0.32469766999999999f;
const float drivetrain_brake_coeff_rear = 6621.3603784869529f;
const float drivetrain_ww_ok_lo = 9.0658608518199646f;
const float drivetrain_ww_ok_hi = 84.786276023749053f;
const float drivetrain_we_ok_lo = 189.75497436523438f;
const float drivetrain_we_ok_hi = 715.95283508300781f;
const int drivetrain_n_u = 9;
const int drivetrain_n_bisect = 12;

static const float drivetrain_feat_mean[3] = {
    0.0f, 0.0f, 0.0f,
};

static const float drivetrain_feat_std[3] = {
    300.0f, 10.0f, 15.39894026f,
};

static const float drivetrain_binom[5] = {
    1.0f, 4.0f, 6.0f, 4.0f, 1.0f,
};

static const float drivetrain_u_knots[9] = {
    0.0f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f,
    1.0f,
};

static const float drivetrain_w0[18] = {
    0.012540142051875591f, 0.39302030205726624f, -0.018435316160321236f, 0.011289073154330254f, 0.16741490364074707f, -0.024728668853640556f, 0.014516964554786682f, 0.60970789194107056f,
    -0.36668309569358826f, -0.01292608305811882f, -0.83761453628540039f, -0.018331019207835197f, -0.1364181786775589f, 0.27359664440155029f, 0.76389008760452271f, -0.025407927110791206f,
    0.87183994054794312f, 0.0045668715611100197f,
};
static const float drivetrain_b0[6] = {
    -0.014093143865466118f, -0.069400377571582794f, 0.050834681838750839f, -0.02718120813369751f, -0.87144887447357178f, -0.063091568648815155f,
};
enum { drivetrain_in0 = 3, drivetrain_out0 = 6 };

static const float drivetrain_w1[36] = {
    0.14404475688934326f, -0.084516771137714386f, -0.071828804910182953f, 0.096329711377620697f, 0.12448489665985107f, -0.019532788544893265f, -0.083359584212303162f, 0.01115243136882782f,
    0.48429363965988159f, 0.3039538562297821f, 0.82878053188323975f, 0.039729580283164978f, 2.0346829891204834f, -0.45140543580055237f, -0.075185582041740417f, 0.45316818356513977f,
    -0.76428282260894775f, -0.0041925995610654354f, 0.076095826923847198f, 0.070881471037864685f, 0.0031740006525069475f, 0.10513769835233688f, 0.054402682930231094f, -0.019475588575005531f,
    -1.0123189687728882f, 0.66565126180648804f, 1.2211829423904419f, -0.73702734708786011f, 0.66667419672012329f, -0.0092925000935792923f, 0.023583386093378067f, 0.042845826596021652f,
    0.064433380961418152f, 0.06758301705121994f, 0.029942542314529419f, 0.061171282082796097f,
};
static const float drivetrain_b1[6] = {
    0.52573108673095703f, -0.53251850605010986f, -0.65189248323440552f, -0.11735312640666962f, 0.082417070865631104f, -0.16841709613800049f,
};
enum { drivetrain_in1 = 6, drivetrain_out1 = 6 };

static const float drivetrain_w2[30] = {
    -0.10802637785673141f, -0.033283255994319916f, -1.6090751886367798f, -1.8440309762954712f, -0.092369921505451202f, 0.058904554694890976f, 0.12546779215335846f, 0.9830964207649231f,
    -1.8262723684310913f, -3.3466448783874512f, 0.029679568484425545f, -0.54256367683410645f, -0.84563863277435303f, 0.71438515186309814f, -0.27344596385955811f, -0.15636730194091797f,
    -0.038104083389043808f, -2.2242140769958496f, -1.3080383539199829f, -0.93801885843276978f, -0.16883395612239838f, 1.014814019203186f, 1.9009543657302856f, 1.2948610782623291f,
    0.79723471403121948f, -0.012828805483877659f, 0.071079008281230927f, 0.03266124427318573f, 0.15182539820671082f, -0.078120291233062744f,
};
static const float drivetrain_b2[5] = {
    -0.068940714001655579f, 0.27827119827270508f, 0.03667878732085228f, -0.81088924407958984f, 0.83431780338287354f,
};
enum { drivetrain_in2 = 6, drivetrain_out2 = 5 };

static void drivetrain_psi(float u, float *out) {
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;
    float um = 1.0f - u;
    float b[5];
    for (int j = 0; j <= drivetrain_k; ++j) {
        float uj = 1.0f;
        float umj = 1.0f;
        for (int t = 0; t < j; ++t) uj *= u;
        for (int t = 0; t < drivetrain_k - j; ++t) umj *= um;
        b[j] = drivetrain_binom[j] * uj * umj;
    }
    float run = b[drivetrain_k];
    out[drivetrain_k - 1] = run;
    for (int j = drivetrain_k - 1; j >= 1; --j) {
        run += b[j];
        out[j - 1] = run;
    }
}

float drivetrain_mix(float g, const float *F, float throttle_cmd) {
    float psi[drivetrain_k];
    drivetrain_psi(throttle_cmd, psi);
    float t = g;
    for (int k = 0; k < drivetrain_k; ++k) t += psi[k] * F[k];
    return t;
}

int drivetrain_heads(float engine_speed_rads, float boost_pressure, float rear_wheelspeed_rads,
                   float *g_out, float *F_out) {
    if (!isfinite(engine_speed_rads) || !isfinite(rear_wheelspeed_rads) || !isfinite(boost_pressure)) {
        return DRIVETRAIN_INV_INVALID;
    }
    float raw[drivetrain_n_in];
    for (int i = 0; i < drivetrain_n_in; ++i) raw[i] = 0.0f;
    raw[0] = engine_speed_rads;
    raw[1] = boost_pressure;
    raw[2] = rear_wheelspeed_rads;
    float x[drivetrain_n_in];
    for (int i = 0; i < drivetrain_n_in; ++i) {
        x[i] = (raw[i] - drivetrain_feat_mean[i]) / drivetrain_feat_std[i];
    }
    float h0[6];
    for (int j = 0; j < 6; ++j) {
        float z = drivetrain_b0[j];
        for (int kk = 0; kk < 3; ++kk) {
            z += x[kk] * drivetrain_w0[kk * 6 + j];
        }
        h0[j] = drivetrain_relu(z);
    }
    float h1[6];
    for (int j = 0; j < 6; ++j) {
        float z = drivetrain_b1[j];
        for (int kk = 0; kk < 6; ++kk) {
            z += h0[kk] * drivetrain_w1[kk * 6 + j];
        }
        h1[j] = drivetrain_relu(z);
    }
    float h2[5];
    for (int j = 0; j < 5; ++j) {
        float z = drivetrain_b2[j];
        for (int kk = 0; kk < 6; ++kk) {
            z += h1[kk] * drivetrain_w2[kk * 5 + j];
        }
        h2[j] = z;
    }
    if (g_out) *g_out = h2[0] * 1000.0f;
    if (F_out) {
        for (int k = 0; k < drivetrain_k; ++k) {
            F_out[k] = drivetrain_softplus(h2[k + 1]) * 1000.0f;
        }
    }
    return DRIVETRAIN_INV_OK;
}

float drivetrain_forward_torque(float engine_speed_rads, float throttle_cmd,
                              float boost_pressure, float rear_wheelspeed_rads) {
    float g = 0.0f, F[drivetrain_k];
    if (drivetrain_heads(engine_speed_rads, boost_pressure, rear_wheelspeed_rads, &g, F)
            != DRIVETRAIN_INV_OK) {
        return 0.0f;
    }
    return drivetrain_mix(g, F, throttle_cmd);
}

static int drivetrain_invert_from_heads(float g, const float *F, float torque_cmd,
                                      float *u_out, float *t_hat, int use_knots) {
    if (!isfinite(torque_cmd) || !isfinite(g)) {
        if (u_out) *u_out = 0.0f;
        if (t_hat) *t_hat = 0.0f;
        return DRIVETRAIN_INV_INVALID;
    }
    float t1 = g;
    for (int k = 0; k < drivetrain_k; ++k) {
        if (!isfinite(F[k])) {
            if (u_out) *u_out = 0.0f;
            if (t_hat) *t_hat = 0.0f;
            return DRIVETRAIN_INV_INVALID;
        }
        t1 += F[k];
    }
    if (t1 <= g + DRIVETRAIN_U_EPS) {
        if (u_out) *u_out = 0.0f;
        if (t_hat) *t_hat = g;
        return DRIVETRAIN_INV_BAD_ENDPOINTS;
    }
    if (torque_cmd <= g) {
        if (u_out) *u_out = 0.0f;
        if (t_hat) *t_hat = g;
        return DRIVETRAIN_INV_SAT_COAST;
    }
    if (torque_cmd >= t1) {
        if (u_out) *u_out = 1.0f;
        if (t_hat) *t_hat = t1;
        return DRIVETRAIN_INV_SAT_FULL;
    }
    if (use_knots) {
        float t_slice[9];
        for (int i = 0; i < 9; ++i) {
            t_slice[i] = drivetrain_mix(g, F, drivetrain_u_knots[i]);
        }
        for (int i = 0; i < 9 - 1; ++i) {
            float t0 = t_slice[i], t1b = t_slice[i + 1];
            if (t1b <= t0 + DRIVETRAIN_U_EPS) continue;
            if (torque_cmd < t0 || torque_cmd > t1b) continue;
            float den = t1b - t0;
            float a = (den < DRIVETRAIN_T_DEN && den > -DRIVETRAIN_T_DEN)
                ? 0.0f : (torque_cmd - t0) / den;
            float u = drivetrain_u_knots[i] + a * (drivetrain_u_knots[i + 1] - drivetrain_u_knots[i]);
            if (u < 0.0f) u = 0.0f;
            if (u > 1.0f) u = 1.0f;
            if (u_out) *u_out = u;
            if (t_hat) *t_hat = torque_cmd;
            return DRIVETRAIN_INV_OK;
        }
        if (u_out) *u_out = 0.0f;
        if (t_hat) *t_hat = g;
        return DRIVETRAIN_INV_NO_SAFE_BRANCH;
    }
    float lo = 0.0f, hi = 1.0f;
    for (int it = 0; it < drivetrain_n_bisect; ++it) {
        float mid = 0.5f * (lo + hi);
        if (drivetrain_mix(g, F, mid) < torque_cmd) lo = mid;
        else hi = mid;
    }
    if (u_out) *u_out = 0.5f * (lo + hi);
    if (t_hat) *t_hat = torque_cmd;
    return DRIVETRAIN_INV_OK;
}

int drivetrain_inverse_throttle_ex(float engine_speed_rads, float boost_pressure,
                                 float rear_wheelspeed_rads, float torque_cmd,
                                 float *u_out, float *t_hat) {
    float g = 0.0f, F[drivetrain_k];
    int st = drivetrain_heads(engine_speed_rads, boost_pressure, rear_wheelspeed_rads, &g, F);
    if (st != DRIVETRAIN_INV_OK) {
        if (u_out) *u_out = 0.0f;
        if (t_hat) *t_hat = 0.0f;
        return st;
    }
    return drivetrain_invert_from_heads(g, F, torque_cmd, u_out, t_hat, 0);
}

float drivetrain_inverse_throttle(float engine_speed_rads, float boost_pressure,
                                float rear_wheelspeed_rads, float torque_cmd) {
    float u = 0.0f, that = 0.0f;
    drivetrain_inverse_throttle_ex(engine_speed_rads, boost_pressure, rear_wheelspeed_rads,
                                 torque_cmd, &u, &that);
    return u;
}

int drivetrain_inverse_throttle_knots_ex(float engine_speed_rads, float boost_pressure,
                                       float rear_wheelspeed_rads, float torque_cmd,
                                       float *u_out, float *t_hat) {
    float g = 0.0f, F[drivetrain_k];
    int st = drivetrain_heads(engine_speed_rads, boost_pressure, rear_wheelspeed_rads, &g, F);
    if (st != DRIVETRAIN_INV_OK) {
        if (u_out) *u_out = 0.0f;
        if (t_hat) *t_hat = 0.0f;
        return st;
    }
    return drivetrain_invert_from_heads(g, F, torque_cmd, u_out, t_hat, 1);
}

float drivetrain_inverse_throttle_knots(float engine_speed_rads, float boost_pressure,
                                      float rear_wheelspeed_rads, float torque_cmd) {
    float u = 0.0f, that = 0.0f;
    drivetrain_inverse_throttle_knots_ex(engine_speed_rads, boost_pressure, rear_wheelspeed_rads,
                                       torque_cmd, &u, &that);
    return u;
}

static float drivetrain_interp_tab(float x, const float *xs, const float *ys, int n) {
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n - 1]) return ys[n - 1];
    int k = 0;
    while (k + 1 < n && xs[k + 1] < x) ++k;
    float a = (xs[k + 1] - xs[k]);
    if (a < 1.0e-12f && a > -1.0e-12f) return ys[k];
    return ys[k] + (x - xs[k]) / a * (ys[k + 1] - ys[k]);
}

static const float drivetrain_i_ww[16] = {
    5.0076972192674063f, 9.0658608518199646f, 13.611719814373167f, 19.044546128769742f, 24.619771843395291f, 29.619117101511673f, 34.982287903874479f, 41.449259472474772f,
    48.061237864924884f, 53.536510260295287f, 58.126483401431628f, 63.664654109210133f, 70.498251026693936f, 77.075896016253807f, 84.786276023749053f, 103.58069415600821f,
};
static const float drivetrain_i_lo_ww[16] = {
    20.754847578134644f, 13.146635259649775f, 11.948978705684969f, 12.389610926058868f, 11.211750208881583f, 10.362736980485954f, 9.5032336275488252f, 8.7559389570624493f,
    8.2422671373119005f, 7.9348385154340564f, 7.6874808697921377f, 7.4038526680665671f, 7.2007669360989368f, 7.1255131203778275f, 7.1287245458688293f, 7.0140427570344865f,
};
static const float drivetrain_i_hi_ww[16] = {
    71.548591689172824f, 33.25461267993429f, 25.551063235649533f, 25.702142667588063f, 25.141528152625238f, 24.505292235208316f, 23.523860620922449f, 21.214575329140303f,
    18.143268043246415f, 15.605814372365401f, 13.937877921496266f, 12.534719703339432f, 10.595552241777749f, 8.7329239066282334f, 7.5912422243656579f, 7.2329912586045451f,
};
enum { drivetrain_n_i_ww = 16 };

int drivetrain_we_band(float rear_wheelspeed_rads, float *we_lo, float *we_hi) {
    if (!isfinite(rear_wheelspeed_rads) || rear_wheelspeed_rads < 1.0e-3f) {
        if (we_lo) *we_lo = 0.0f;
        if (we_hi) *we_hi = 0.0f;
        return DRIVETRAIN_INV_INVALID;
    }
    float ilo = drivetrain_interp_tab(rear_wheelspeed_rads, drivetrain_i_ww, drivetrain_i_lo_ww, drivetrain_n_i_ww);
    float ihi = drivetrain_interp_tab(rear_wheelspeed_rads, drivetrain_i_ww, drivetrain_i_hi_ww, drivetrain_n_i_ww);
    if (ihi < ilo) {
        if (we_lo) *we_lo = 0.0f;
        if (we_hi) *we_hi = 0.0f;
        return DRIVETRAIN_INV_BAD_ENDPOINTS;
    }
    if (we_lo) *we_lo = ilo * rear_wheelspeed_rads;
    if (we_hi) *we_hi = ihi * rear_wheelspeed_rads;
    return DRIVETRAIN_INV_OK;
}

static const float drivetrain_i_we[16] = {
    144.12832641601562f, 189.75497436523438f, 260.13682556152344f, 316.46784210205078f, 376.21977233886719f, 430.29872131347656f, 466.88058471679688f, 490.43301391601562f,
    515.50057983398438f, 541.18818664550781f, 565.39134216308594f, 587.51698303222656f, 614.52243041992188f, 653.82879638671875f, 715.95283508300781f, 798.99650573730469f,
};
static const float drivetrain_i_lo_we[16] = {
    12.040747291559903f, 12.958573958067081f, 11.777093249539014f, 9.8021492903876268f, 8.4707779367031097f, 7.7592362305531868f, 7.5268448490059665f, 7.3446358274182657f,
    7.210322974002799f, 7.1429214845844182f, 7.1279067384756667f, 7.1327132698698241f, 7.1319603638768543f, 7.1272605504791819f, 7.122466283091315f, 7.0224818207809552f,
};
static const float drivetrain_i_hi_we[16] = {
    45.940040904502617f, 71.543494667405866f, 55.453029597697281f, 25.541015046525786f, 25.420504076607614f, 25.402581677774482f, 25.48140313837235f, 25.525510312858998f,
    25.562716883862045f, 25.408381691392194f, 25.111448339714251f, 25.027981392215985f, 24.912168639045177f, 24.72292081571155f, 24.480515515698261f, 23.674184666559068f,
};
enum { drivetrain_n_i_we = 16 };

int drivetrain_ww_band(float engine_speed_rads, float *ww_lo, float *ww_hi) {
    if (!isfinite(engine_speed_rads) || engine_speed_rads < 1.0e-3f) {
        if (ww_lo) *ww_lo = 0.0f;
        if (ww_hi) *ww_hi = 0.0f;
        return DRIVETRAIN_INV_INVALID;
    }
    float ilo = drivetrain_interp_tab(engine_speed_rads, drivetrain_i_we, drivetrain_i_lo_we, drivetrain_n_i_we);
    float ihi = drivetrain_interp_tab(engine_speed_rads, drivetrain_i_we, drivetrain_i_hi_we, drivetrain_n_i_we);
    if (!(ilo > 0.0f) || ihi < ilo) {
        if (ww_lo) *ww_lo = 0.0f;
        if (ww_hi) *ww_hi = 0.0f;
        return DRIVETRAIN_INV_BAD_ENDPOINTS;
    }
    if (ww_lo) *ww_lo = engine_speed_rads / ihi;
    if (ww_hi) *ww_hi = engine_speed_rads / ilo;
    return DRIVETRAIN_INV_OK;
}

