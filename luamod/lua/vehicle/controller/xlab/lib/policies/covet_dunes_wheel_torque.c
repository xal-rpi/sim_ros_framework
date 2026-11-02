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
 *   catalog: /home/franck/Documents/xal_rpi/xal_vehicle_models/xal_vehicle_models/generated/identification/covet_dunes_v1/catalogs/torque
 *   drivetrain_we_band(ω_w) → ω_e ∈ [i_lo(ω_w), i_hi(ω_w)] · ω_w
 *   drivetrain_ww_band(ω_e) → ω_w ∈ [ω_e/i_hi(ω_e), ω_e/i_lo(ω_e)]
 * i(ω_w) and i(ω_e) are percentile slices of the same catalog, not inverses.
 * Rails are not very good below / above these speeds (clutch / sparse tails):
 *   ω_w < 11.05 rad/s or ω_w > 158.70 rad/s
 *   ω_e < 211.06 rad/s or ω_e > 840.03 rad/s
 */
#include "covet_dunes_wheel_torque.h"
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

const float drivetrain_j_eff = 12.229862261632256f;
const float drivetrain_r = 0.42702329999999999f;
const float drivetrain_r_kin = 0.42728621999999999f;
const float drivetrain_brake_coeff_rear = 2299.2719266125832f;
const float drivetrain_ww_ok_lo = 11.054231475530854f;
const float drivetrain_ww_ok_hi = 158.69767963849225f;
const float drivetrain_we_ok_lo = 211.06219148635864f;
const float drivetrain_we_ok_hi = 840.03359031677246f;
const int drivetrain_n_u = 9;
const int drivetrain_n_bisect = 12;

static const float drivetrain_feat_mean[3] = {
    0.0f, 0.0f, 0.0f,
};

static const float drivetrain_feat_std[3] = {
    300.0f, 10.0f, 11.701758229999999f,
};

static const float drivetrain_binom[5] = {
    1.0f, 4.0f, 6.0f, 4.0f, 1.0f,
};

static const float drivetrain_u_knots[9] = {
    0.0f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f,
    1.0f,
};

static const float drivetrain_w0[18] = {
    0.16424733400344849f, 0.22625848650932312f, 0.44912829995155334f, -0.025514958426356316f, 0.62214928865432739f, 0.93722224235534668f, 0.4716397225856781f, 0.70783400535583496f,
    0.048597265034914017f, -0.0050516948103904724f, 0.72450250387191772f, 0.32083070278167725f, 0.036244023591279984f, 0.28874024748802185f, -0.30681267380714417f, -0.003168557770550251f,
    -0.40409520268440247f, -0.18096745014190674f,
};
static const float drivetrain_b0[6] = {
    0.027948986738920212f, 0.096680961549282074f, 0.0066395420581102371f, -0.090311706066131592f, 0.339103102684021f, 0.32026022672653198f,
};
enum { drivetrain_in0 = 3, drivetrain_out0 = 6 };

static const float drivetrain_w1[36] = {
    0.3524107038974762f, -0.074953578412532806f, -0.61797487735748291f, 0.17343959212303162f, 0.64230829477310181f, -0.021920584142208099f, -0.078213073313236237f, 1.1113595962524414f,
    0.78599518537521362f, 0.18181334435939789f, -0.073867723345756531f, 0.0052762054838240147f, 1.9946922063827515f, -1.2475742101669312f, -0.25668445229530334f, -0.26304420828819275f,
    -0.66817867755889893f, 0.080725200474262238f, 0.1048692986369133f, -0.073651619255542755f, -0.022617869079113007f, 0.10216972231864929f, 0.10248930752277374f, 0.026528310030698776f,
    1.3620873689651489f, 0.21356973052024841f, -0.043656680732965469f, 1.0920565128326416f, -0.65982335805892944f, -0.028732985258102417f, 0.64331597089767456f, 0.3854900598526001f,
    -0.21703584492206573f, -0.068993009626865387f, 0.2785058319568634f, 0.044331274926662445f,
};
static const float drivetrain_b1[6] = {
    0.43320181965827942f, 0.087654002010822296f, -0.2163909375667572f, -0.087508268654346466f, -0.17555657029151917f, -0.16867436468601227f,
};
enum { drivetrain_in1 = 6, drivetrain_out1 = 6 };

static const float drivetrain_w2[30] = {
    0.71066826581954956f, 1.2288119792938232f, -0.97844672203063965f, -1.1892809867858887f, -0.20652531087398529f, -0.93731838464736938f, -0.23479916155338287f, -0.083577312529087067f,
    -0.29580089449882507f, 0.10679176449775696f, 1.8869513273239136f, -0.097366265952587128f, -0.00071261165430769324f, -0.41396695375442505f, -0.61675727367401123f, -1.1815962791442871f,
    0.52174961566925049f, 1.525769829750061f, 1.7747412919998169f, -1.0151885747909546f, 0.74150162935256958f, 0.57722568511962891f, -0.29214748740196228f, 1.1254874467849731f,
    0.56219649314880371f, 0.02936539426445961f, 0.10102861374616623f, 0.1009930819272995f, 0.08146512508392334f, 0.00026909474399872124f,
};
static const float drivetrain_b2[5] = {
    0.070296868681907654f, 0.47525635361671448f, -0.80927532911300659f, -1.464582085609436f, 0.44330957531929016f,
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
    4.4574894235451668f, 11.054231475530854f, 20.030468938732419f, 31.081044368124179f, 41.732391388373252f, 50.162199284063746f, 58.079060237699373f, 66.517354222551049f,
    77.546563773546296f, 91.694115081131756f, 106.75250941526771f, 121.27787903342205f, 133.95219454194279f, 145.54564923054369f, 158.69767963849225f, 180.38955420368887f,
};
static const float drivetrain_i_lo_ww[16] = {
    22.676960741348672f, 17.793702767375887f, 11.302690446215474f, 8.1013544873866632f, 6.3492369028169682f, 6.1073591574922315f, 4.9969300824652541f, 4.9887549821745667f,
    4.9837983398162899f, 4.9725006369268332f, 4.9562488478215014f, 4.947088923833932f, 4.9210540449710498f, 4.8649315752071418f, 4.8792120402485351f, 4.8890385175643498f,
};
static const float drivetrain_i_hi_ww[16] = {
    68.577527347422276f, 28.255629961668124f, 19.930109949500213f, 18.729152155626924f, 18.678980953897284f, 18.393159036201212f, 12.19876516770754f, 11.452759905554004f,
    11.45206342149684f, 8.311644844274845f, 8.1076447863342711f, 8.0918034465368045f, 6.3299522810070803f, 6.315285417911479f, 4.9599110164518985f, 4.9408738439961644f,
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
    162.51193475723267f, 211.06219148635864f, 276.21411323547363f, 309.64734649658203f, 335.37545299530029f, 363.80491542816162f, 398.04447364807129f, 436.21117210388184f,
    485.51799583435059f, 543.17558479309082f, 604.35421943664551f, 668.26788139343262f, 729.38223075866699f, 785.47170066833496f, 840.03359031677246f, 945.70651817321777f,
};
static const float drivetrain_i_lo_we[16] = {
    18.134392214869109f, 11.384454795827047f, 4.9979738554975244f, 4.9956107437987542f, 4.9917637328396935f, 4.9887663661084387f, 4.9842486965289297f, 4.97714096065575f,
    4.969898003206568f, 4.9557816418130134f, 4.9464385728053859f, 4.8908363960295613f, 4.896619150477111f, 4.8871259368296993f, 4.9035631794747374f, 4.8952479150277464f,
};
static const float drivetrain_i_hi_we[16] = {
    67.15821040207345f, 63.892954560434802f, 28.035620662651596f, 18.932495222433996f, 18.51706534677967f, 18.542779900942104f, 18.574092570771885f, 18.575992815006945f,
    18.729418438696086f, 18.546025013134244f, 18.530742574856692f, 18.521197920250952f, 18.466252067126121f, 18.34397611952523f, 16.648850288505884f, 18.422659582993859f,
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

