/* Vendored from utv_wild_v2_wheel_torque_struct/nn/drivetrain_struct.c
 * rwd_struct: T = g(x) + Σ I_k(u) F_k(x).
 * Stock export uses math.h (isfinite, expf, logf) → NEEDED libm.so.6.
 * Local replacements keep BeamNG ffi.load libm-free.
 */
#include "utv_wild_v2_wheel_torque.h"

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

const float drivetrain_r_kin = 0.38664999999999999f;
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
    0.66000264883041382f, 0.17263412475585938f, 0.42213475704193115f, -0.052052810788154602f, 0.17085996270179749f, 0.28002837300300598f, 0.46287795901298523f, 0.75335323810577393f,
    0.16464903950691223f, 0.0019185841083526611f, -0.18283633887767792f, 0.28781673312187195f, -0.33054342865943909f, -0.036474239081144333f, -0.40212225914001465f, -0.011312364600598812f,
    -0.51181107759475708f, 0.46875742077827454f,
};
static const float drivetrain_b0[6] = {
    0.2859804630279541f, 0.21925532817840576f, 0.45973116159439087f, -0.046936240047216415f, 0.22456546127796173f, 0.064095482230186462f,
};
enum { drivetrain_in0 = 3, drivetrain_out0 = 6 };

static const float drivetrain_w1[36] = {
    0.61919260025024414f, 0.0018516059499233961f, 0.41343817114830017f, 0.022840417921543121f, 0.24719633162021637f, 0.6607433557510376f, 0.81278634071350098f, 0.037894546985626221f,
    -0.45937865972518921f, 0.022620731964707375f, 1.0099785327911377f, -0.84013783931732178f, 0.5817265510559082f, -0.069718383252620697f, -0.39397570490837097f, -0.039429578930139542f,
    -0.3864082396030426f, -0.85901373624801636f, 0.032959945499897003f, 0.06570744514465332f, 0.036856025457382202f, 0.097848758101463318f, 0.033854003995656967f, 0.02442435547709465f,
    1.5356732606887817f, 0.032155372202396393f, 0.77946501970291138f, 0.023094687610864639f, -1.389555811882019f, -0.078371115028858185f, -0.44899243116378784f, -0.03185427188873291f,
    0.17166782915592194f, -0.012909692712128162f, 0.51784485578536987f, 0.39401975274085999f,
};
static const float drivetrain_b1[6] = {
    0.46161925792694092f, -0.11697977036237717f, 0.048964362591505051f, -0.049444872885942459f, 0.22297555208206177f, -0.14499832689762115f,
};
enum { drivetrain_in1 = 6, drivetrain_out1 = 6 };

static const float drivetrain_w2[30] = {
    -0.16559277474880219f, 1.5646886825561523f, 0.069090269505977631f, -1.8834706544876099f, -0.53512120246887207f, 0.077000856399536133f, -0.057340733706951141f, 0.0024990621022880077f,
    0.11817635595798492f, 0.0091813821345567703f, 0.91292387247085571f, -0.047727085649967194f, 0.024085616692900658f, -3.2226347923278809f, 0.4061950147151947f, 0.040917690843343735f,
    -0.039170645177364349f, 0.025295892730355263f, 0.083330631256103516f, 0.028894009068608284f, -0.43076243996620178f, -0.312287837266922f, 0.28897294402122498f, -1.6647040843963623f,
    -0.67083084583282471f, -0.26290410757064819f, -0.28860267996788025f, 1.4264075756072998f, -1.1078206300735474f, 3.5057821273803711f,
};
static const float drivetrain_b2[5] = {
    -0.20524957776069641f, 0.26395314931869507f, -0.0382070392370224f, -2.6249833106994629f, -0.74137645959854126f,
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
    if (!drivetrain_isfinite(engine_speed_rads) || !drivetrain_isfinite(rear_wheelspeed_rads)
        || !drivetrain_isfinite(boost_pressure)) {
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
