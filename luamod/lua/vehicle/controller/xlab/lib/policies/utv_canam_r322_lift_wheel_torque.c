/* Vendored from utv_canam_r322_lift_wheel_torque_and_brake/nn/drivetrain_struct.c
 * rwd_struct: T = g(x) + Σ I_k(u) F_k(x).
 * Stock export uses math.h (isfinite, expf, logf) → NEEDED libm.so.6.
 * Local replacements keep BeamNG ffi.load libm-free.
 */
#include "utv_canam_r322_lift_wheel_torque.h"

#ifndef DRIVETRAIN_U_EPS
#define DRIVETRAIN_U_EPS 0.0010000f
#define DRIVETRAIN_T_DEN 0.0000010f
#endif

static int drivetrain_isfinite(float x) { return x == x && (x - x) == 0.0f; }

static float drivetrain_relu(float x) { return x > 0.0f ? x : 0.0f; }

static float drivetrain_expf(float x) {
    const float ln2 = 0.69314718056f;
    const float inv_ln2 = 1.44269504089f;
    float y = x * inv_ln2;
    int k = (int)(y + (y >= 0.0f ? 0.5f : -0.5f));
    float f = x - (float)k * ln2;
    float p = 1.0f + f * (1.0f + f * (0.5f + f * (0.166666667f + f * 0.041666667f)));
    union { float f; unsigned u; } s;
    s.f = p;
    int exp = (int)((s.u >> 23) & 255) + k;
    if (exp <= 0) return 0.0f;
    if (exp >= 255) return 3.402823466e+38f;
    s.u = (s.u & 0x807fffffu) | ((unsigned)exp << 23);
    return s.f;
}

static float drivetrain_logf(float x) {
    if (x <= 0.0f) return -1.0e30f;
    union { float f; unsigned u; } v;
    v.f = x;
    int e = (int)((v.u >> 23) & 255) - 127;
    v.u = (v.u & 0x7fffffu) | (127u << 23);
    float z = v.f - 1.0f;
    float y = z / (2.0f + z);
    float y2 = y * y;
    float p = y * (2.0f + y2 * (0.6666667f + y2 * (0.4f + y2 * 0.2857143f)));
    return p + (float)e * 0.69314718056f;
}

static float drivetrain_softplus(float x) {
    float ax = x < 0.0f ? -x : x;
    float pos = x > 0.0f ? x : 0.0f;
    return pos + drivetrain_logf(1.0f + drivetrain_expf(-ax));
}

const float drivetrain_j_eff = 27.727867903099202f;
const float drivetrain_r = 0.32400000000000001f;
const float drivetrain_r_kin = 0.32100000000000001f;
const float drivetrain_brake_coeff_rear = 8816.9743249445801f;
const float drivetrain_ww_ok_lo = 11.42558246572441f;
const float drivetrain_ww_ok_hi = 61.768498561835365f;
const float drivetrain_we_ok_lo = 252.21786594390869f;
const float drivetrain_we_ok_hi = 674.10664367675781f;
const int drivetrain_n_u = 9;
const int drivetrain_n_bisect = 12;

static const float drivetrain_feat_mean[3] = {
    0.0f, 0.0f, 0.0f,
};

static const float drivetrain_feat_std[3] = {
    300.0f, 10.0f, 15.0f,
};

static const float drivetrain_binom[5] = {
    1.0f, 4.0f, 6.0f, 4.0f, 1.0f,
};

static const float drivetrain_u_knots[9] = {
    0.0f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f,
    1.0f,
};

static const float drivetrain_w0[18] = {
    -0.020091604441404343f, 0.47305464744567871f, -0.35998255014419556f, 0.0015599189791828394f, 0.30383506417274475f, 0.41501414775848389f, -0.025245971977710724f, 0.38932028412818909f,
    0.089507430791854858f, -0.0019077187171205878f, 0.69000184535980225f, 0.29808905720710754f, -0.024438466876745224f, 0.042070705443620682f, 0.6675105094909668f, -0.01759670116007328f,
    -0.26560324430465698f, -0.5089794397354126f,
};
static const float drivetrain_b0[6] = {
    -0.037684410810470581f, 0.33832892775535583f, 0.43111827969551086f, -0.030932201072573662f, 0.16312897205352783f, 0.46272233128547668f,
};
enum { drivetrain_in0 = 3, drivetrain_out0 = 6 };

static const float drivetrain_w1[36] = {
    0.10301966965198517f, 0.055844109505414963f, 0.043415199965238571f, -0.0010918100597336888f, 0.067759789526462555f, -0.036191616207361221f, -0.14979255199432373f, -0.00085384392878040671f,
    0.0082929488271474838f, 0.59850120544433594f, 0.83150702714920044f, 0.31361481547355652f, 0.57269799709320068f, 0.00013422456686384976f, -0.0068931677378714085f, -0.18799595534801483f,
    -0.68773263692855835f, -0.79202759265899658f, 0.01871175691485405f, 0.040068384259939194f, -0.0011510980548337102f, 0.053992193192243576f, 0.046543031930923462f, 0.016297386959195137f,
    0.50579792261123657f, -0.0043313181959092617f, -0.029842451214790344f, 0.76199555397033691f, 0.64488720893859863f, -0.29160094261169434f, 0.86986392736434937f, 0.0056561091914772987f,
    -0.014271561987698078f, -0.81702506542205811f, 0.43113803863525391f, 0.71849089860916138f,
};
static const float drivetrain_b1[6] = {
    0.4730563759803772f, -0.067067347466945648f, -0.071272224187850952f, 0.036172471940517426f, 0.3198859691619873f, -0.0031214186456054449f,
};
enum { drivetrain_in1 = 6, drivetrain_out1 = 6 };

static const float drivetrain_w2[30] = {
    -0.24954785406589508f, -0.51622682809829712f, -0.12640021741390228f, -2.0453307628631592f, -0.46094170212745667f, 0.041860513389110565f, -0.029900155961513519f, 0.019875718280673027f,
    0.018949706107378006f, 0.060121417045593262f, 0.031278137117624283f, 0.068217255175113678f, 0.020952781662344933f, -0.076040059328079224f, 0.023875581100583076f, -0.58163851499557495f,
    0.20290905237197876f, 1.610446572303772f, 2.5259206295013428f, 0.41215500235557556f, -0.090800255537033081f, 1.223607063293457f, -1.1018048524856567f, -2.0196654796600342f,
    0.37196686863899231f, 2.5603194236755371f, 0.39204537868499756f, -1.4827797412872314f, 1.3286343812942505f, 2.7296733856201172f,
};
static const float drivetrain_b2[5] = {
    0.056866984814405441f, 0.0020840123761445284f, -0.84338897466659546f, -2.9273452758789062f, 0.28625431656837463f,
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
    if (!drivetrain_isfinite(engine_speed_rads) || !drivetrain_isfinite(rear_wheelspeed_rads) || !drivetrain_isfinite(boost_pressure)) {
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
    if (!drivetrain_isfinite(torque_cmd) || !drivetrain_isfinite(g)) {
        if (u_out) *u_out = 0.0f;
        if (t_hat) *t_hat = 0.0f;
        return DRIVETRAIN_INV_INVALID;
    }
    float t1 = g;
    for (int k = 0; k < drivetrain_k; ++k) {
        if (!drivetrain_isfinite(F[k])) {
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
    8.1415903809657344f, 11.42558246572441f, 14.402711781385904f, 17.367174135190304f, 19.876084827188389f, 21.983129259581879f, 23.855990524232574f, 25.974321588177546f,
    28.936445434516834f, 32.825373507734398f, 37.936340147089737f, 43.577791838631079f, 49.223679062733396f, 55.203168748695163f, 61.768498561835365f, 73.355277193669593f,
};
static const float drivetrain_i_lo_ww[16] = {
    14.556337182559517f, 13.210315976562294f, 15.106199758699027f, 14.314113054282277f, 13.719949137257881f, 13.214983603977174f, 12.788469188958661f, 12.282141746221544f,
    11.488292093410537f, 10.832598700745706f, 10.13764574981707f, 9.4332563740794306f, 8.9498698673909249f, 8.5657887205466281f, 8.254891560230261f, 7.1950745139857757f,
};
static const float drivetrain_i_hi_ww[16] = {
    36.29044228396787f, 25.633444332992028f, 25.380665006809686f, 25.401336502284522f, 25.715810221043498f, 24.984832266385595f, 24.772691804632327f, 24.534538446123314f,
    24.152114381939118f, 23.484952227014677f, 22.106218345892167f, 19.895446350641937f, 17.319335664081571f, 14.660361729549043f, 12.21393479269463f, 10.058933096535029f,
};
enum { drivetrain_n_i_ww = 16 };

int drivetrain_we_band(float rear_wheelspeed_rads, float *we_lo, float *we_hi) {
    if (!drivetrain_isfinite(rear_wheelspeed_rads) || rear_wheelspeed_rads < 1.0e-3f) {
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
    187.56913089752197f, 252.21786594390869f, 299.38342761993408f, 349.46601581573486f, 392.81360149383545f, 428.85383701324463f, 454.68682098388672f, 471.22190284729004f,
    486.03573513031006f, 502.456223487854f, 520.96635246276855f, 544.14992141723633f, 575.11264038085938f, 612.67105865478516f, 674.10664367675781f, 774.95176696777344f,
};
static const float drivetrain_i_lo_we[16] = {
    13.210324264560219f, 14.040975689875063f, 12.429208979769504f, 10.898380190051737f, 10.063047297953116f, 9.2369573181718252f, 8.9517259444391026f, 8.7783229142572861f,
    8.5889170333585199f, 8.4192629385736097f, 7.526677319898841f, 7.3026619999322735f, 7.2164114232795002f, 7.3531147328496038f, 9.1517585684112301f, 12.165203454159011f,
};
static const float drivetrain_i_hi_we[16] = {
    35.54744779156853f, 36.164628758135493f, 25.557751943626432f, 25.324969698770868f, 25.075415932768923f, 25.208532655896395f, 25.124046561146756f, 25.230034680171258f,
    25.637342548325691f, 25.20759139539301f, 25.121575599851589f, 24.92763308836405f, 24.827301284519177f, 24.638778325594291f, 24.397750822131556f, 23.709805326025702f,
};
enum { drivetrain_n_i_we = 16 };

int drivetrain_ww_band(float engine_speed_rads, float *ww_lo, float *ww_hi) {
    if (!drivetrain_isfinite(engine_speed_rads) || engine_speed_rads < 1.0e-3f) {
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

