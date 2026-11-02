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
 *   catalog: /home/franck/Documents/xal_rpi/xal_vehicle_models/xal_vehicle_models/generated/identification/covet_turbo_rally_v1/catalogs/torque
 *   drivetrain_we_band(ω_w) → ω_e ∈ [i_lo(ω_w), i_hi(ω_w)] · ω_w
 *   drivetrain_ww_band(ω_e) → ω_w ∈ [ω_e/i_hi(ω_e), ω_e/i_lo(ω_e)]
 * i(ω_w) and i(ω_e) are percentile slices of the same catalog, not inverses.
 * Rails are not very good below / above these speeds (clutch / sparse tails):
 *   ω_w < 20.93 rad/s or ω_w > 187.46 rad/s
 *   ω_e < 262.23 rad/s or ω_e > 778.95 rad/s
 */
#include "covet_turbo_rally_wheel_torque.h"
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

const float drivetrain_j_eff = 11.782561362507836f;
const float drivetrain_r = 0.30321938999999998f;
const float drivetrain_r_kin = 0.30471935f;
const float drivetrain_brake_coeff_rear = 5023.4254478250878f;
const float drivetrain_ww_ok_lo = 20.927550931598326f;
const float drivetrain_ww_ok_hi = 187.46163413496089f;
const float drivetrain_we_ok_lo = 262.23132228851318f;
const float drivetrain_we_ok_hi = 778.95263290405273f;
const int drivetrain_n_u = 9;
const int drivetrain_n_bisect = 12;

static const float drivetrain_feat_mean[3] = {
    0.0f, 0.0f, 0.0f,
};

static const float drivetrain_feat_std[3] = {
    300.0f, 10.0f, 16.4085413f,
};

static const float drivetrain_binom[5] = {
    1.0f, 4.0f, 6.0f, 4.0f, 1.0f,
};

static const float drivetrain_u_knots[9] = {
    0.0f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f,
    1.0f,
};

static const float drivetrain_w0[18] = {
    -0.61943215131759644f, 0.58190971612930298f, -0.28478333353996277f, -0.02233424223959446f, 0.24300301074981689f, -0.0071269366890192032f, -1.3248239755630493f, -0.28198805451393127f,
    -0.28627613186836243f, 0.071770109236240387f, 0.79774528741836548f, 0.047410868108272552f, 0.89641541242599487f, -0.11229315400123596f, 0.3297639787197113f, -0.074902698397636414f,
    0.27496647834777832f, -0.014048748649656773f,
};
static const float drivetrain_b0[6] = {
    -0.05245145782828331f, -0.11690336465835571f, -0.49332818388938904f, -0.09185800701379776f, 0.065601371228694916f, -0.055684834718704224f,
};
enum { drivetrain_in0 = 3, drivetrain_out0 = 6 };

static const float drivetrain_w1[36] = {
    0.90491247177124023f, 0.79161494970321655f, -0.34266394376754761f, 0.46443954110145569f, 0.41693073511123657f, 0.73064589500427246f, 0.85932821035385132f, -0.55415451526641846f,
    -0.50597500801086426f, 0.48781833052635193f, -0.0040525095537304878f, -0.82818871736526489f, -0.23095157742500305f, -0.97139155864715576f, 0.39358976483345032f, 0.28695213794708252f,
    0.24629087746143341f, -0.88249069452285767f, 0.017229180783033371f, -0.083794757723808289f, -0.0041315965354442596f, 0.22476288676261902f, 0.039857063442468643f, -0.011561678722500801f,
    0.44312208890914917f, 0.2905954122543335f, 0.57785236835479736f, -0.69297939538955688f, -0.54722863435745239f, 0.12829038500785828f, -0.0043663876131176949f, 0.0064937230199575424f,
    0.095087163150310516f, 0.065518751740455627f, 0.042979627847671509f, 0.06631450355052948f,
};
static const float drivetrain_b1[6] = {
    0.39755496382713318f, -0.18865916132926941f, 0.0058670239523053169f, -0.051299594342708588f, -0.18962973356246948f, -0.50794357061386108f,
};
enum { drivetrain_in1 = 6, drivetrain_out1 = 6 };

static const float drivetrain_w2[30] = {
    -0.38774818181991577f, 0.51262027025222778f, -0.46253684163093567f, -0.47983482480049133f, -1.2011256217956543f, -0.085558421909809113f, -1.9777309894561768f, -1.2175326347351074f,
    -2.677764892578125f, 1.5197309255599976f, 0.58015376329421997f, 0.41637676954269409f, 0.75935840606689453f, -0.3363126814365387f, 0.88513189554214478f, 1.6835709810256958f,
    -0.0034758634865283966f, -0.75510084629058838f, -0.79973900318145752f, 1.2979862689971924f, -1.7261805534362793f, 0.36691489815711975f, 0.80367940664291382f, 1.0461176633834839f,
    -1.5783710479736328f, 0.79800504446029663f, 0.94311559200286865f, 0.92018812894821167f, 1.2073501348495483f, 0.17802824079990387f,
};
static const float drivetrain_b2[5] = {
    -0.0097257569432258606f, 1.0506764650344849f, -0.39733409881591797f, 0.10135888308286667f, -0.9528314471244812f,
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
    6.6878833136134048f, 20.927550931598326f, 35.734664335474946f, 49.437325688088194f, 61.496752410124415f, 70.547963927837401f, 79.053910710367063f, 88.442046455883627f,
    99.766698051442134f, 113.30708858130865f, 127.69361023859008f, 141.3735375894592f, 154.49483959756708f, 169.71600175899755f, 187.46163413496089f, 201.70292138723985f,
};
static const float drivetrain_i_lo_ww[16] = {
    14.775681170652788f, 9.3479327869270286f, 6.6212389027097709f, 6.5505374337930462f, 4.100069746972995f, 4.0982952264391272f, 4.0787303177414183f, 4.0920751104479054f,
    4.0864040939742861f, 4.0791508362795197f, 4.0717449290746206f, 4.0672732865363397f, 4.0590708306168306f, 4.0519108026598918f, 4.0364127830509169f, 3.9777750240156662f,
};
static const float drivetrain_i_hi_ww[16] = {
    61.474808173063622f, 21.349735471163747f, 16.363593692399618f, 15.313716127642541f, 9.4363293670741868f, 10.110148715469508f, 9.5694683260843174f, 9.4063310708461749f,
    6.676492151340474f, 6.6733017000355748f, 6.662865644493837f, 5.2088001764711223f, 5.2010552880451169f, 4.0840493458882099f, 4.0748289440981065f, 4.1140374286485466f,
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
    200.53370571136475f, 262.23132228851318f, 293.57477569580078f, 323.81760406494141f, 355.07467269897461f, 387.21044540405273f, 421.24820899963379f, 460.38437080383301f,
    504.0240478515625f, 550.28806304931641f, 595.25425338745117f, 636.71387100219727f, 681.28450393676758f, 729.28354263305664f, 778.95263290405273f, 831.40668869018555f,
};
static const float drivetrain_i_lo_we[16] = {
    4.1020167866172175f, 4.0999639458178594f, 4.0978010284332091f, 4.0831158762499182f, 4.0934746289868738f, 4.0906080181310935f, 4.0846431700309473f, 4.0806314194905555f,
    4.0743588633876451f, 4.0693143692848359f, 4.065157746378766f, 4.0588594266045241f, 4.0540318696189939f, 4.0494988811819166f, 3.9902886755184288f, 4.0051430147923863f,
};
static const float drivetrain_i_hi_we[16] = {
    56.795870472488545f, 57.877067308997532f, 23.408865536153108f, 15.275433263660181f, 15.284315715577108f, 15.231739020541507f, 15.229278720553596f, 15.225330066323483f,
    15.220825943337005f, 15.254735320928827f, 15.216915858202334f, 15.247752697620818f, 15.228100341210858f, 15.168798698911303f, 15.263964267828772f, 15.193820471524051f,
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

