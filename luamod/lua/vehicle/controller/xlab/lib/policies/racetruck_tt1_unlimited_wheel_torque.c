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
 *   catalog: /home/franck/Documents/xal_rpi/xal_vehicle_models/xal_vehicle_models/generated/identification/racetruck_tt1_unlimited_v1/catalogs/torque
 *   drivetrain_we_band(ω_w) → ω_e ∈ [i_lo(ω_w), i_hi(ω_w)] · ω_w
 *   drivetrain_ww_band(ω_e) → ω_w ∈ [ω_e/i_hi(ω_e), ω_e/i_lo(ω_e)]
 * i(ω_w) and i(ω_e) are percentile slices of the same catalog, not inverses.
 * Rails are not very good below / above these speeds (clutch / sparse tails):
 *   ω_w < 5.50 rad/s or ω_w > 106.86 rad/s
 *   ω_e < 77.51 rad/s or ω_e > 634.76 rad/s
 */
#include "racetruck_tt1_unlimited_wheel_torque.h"
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

const float drivetrain_j_eff = 93.024076967151771f;
const float drivetrain_r = 0.50380312000000005f;
const float drivetrain_r_kin = 0.51292771000000004f;
const float drivetrain_brake_coeff_rear = 8310.6996944401035f;
const float drivetrain_ww_ok_lo = 5.4965188355883292f;
const float drivetrain_ww_ok_hi = 106.86192373164377f;
const float drivetrain_we_ok_lo = 77.513960361480713f;
const float drivetrain_we_ok_hi = 634.75881576538086f;
const int drivetrain_n_u = 9;
const int drivetrain_n_bisect = 12;

static const float drivetrain_feat_mean[2] = {
    0.0f, 0.0f,
};

static const float drivetrain_feat_std[2] = {
    300.0f, 9.7479623100000001f,
};

static const float drivetrain_binom[5] = {
    1.0f, 4.0f, 6.0f, 4.0f, 1.0f,
};

static const float drivetrain_u_knots[9] = {
    0.0f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f,
    1.0f,
};

static const float drivetrain_w0[12] = {
    -0.40417921543121338f, 1.108728289604187f, 0.79797685146331787f, 0.31180694699287415f, 1.0739952325820923f, 0.6568608283996582f, 1.1225036382675171f, -0.058142431080341339f,
    -0.26073431968688965f, -0.095598980784416199f, -0.42793324589729309f, -0.088523656129837036f,
};
static const float drivetrain_b0[6] = {
    0.27674826979637146f, 0.9345855712890625f, 0.54267722368240356f, 0.37568205595016479f, 0.21872794628143311f, 0.26387539505958557f,
};
enum { drivetrain_in0 = 2, drivetrain_out0 = 6 };

static const float drivetrain_w1[36] = {
    -0.25988665223121643f, -0.080454133450984955f, -0.84543067216873169f, -0.1377323716878891f, 0.42829740047454834f, -0.19957448542118073f, 0.65995758771896362f, 0.28677991032600403f,
    0.29802599549293518f, 1.1340528726577759f, 0.31897220015525818f, 1.0676778554916382f, 0.62335187196731567f, -0.71244871616363525f, -0.199082151055336f, -0.52916908264160156f,
    0.52496755123138428f, 0.41934871673583984f, 0.40007907152175903f, 0.56596314907073975f, -0.4803084135055542f, -0.01698092557489872f, 0.34010446071624756f, -1.0991969108581543f,
    0.71660053730010986f, 0.046379692852497101f, 1.2943804264068604f, -1.1678332090377808f, 0.25911709666252136f, 1.0539345741271973f, 0.76923513412475586f, 0.28173205256462097f,
    0.4402930736541748f, 0.13587513566017151f, 0.85579997301101685f, 0.18704918026924133f,
};
static const float drivetrain_b1[6] = {
    0.6769559383392334f, -0.10583846271038055f, 0.3508528470993042f, 0.2676127552986145f, 0.10816352814435959f, 0.73377656936645508f,
};
enum { drivetrain_in1 = 6, drivetrain_out1 = 6 };

static const float drivetrain_w2[30] = {
    -0.18335461616516113f, 0.19266700744628906f, -0.52919125556945801f, 0.90352797508239746f, -0.002244958421215415f, -0.15995950996875763f, -0.45048943161964417f, -0.37094816565513611f,
    0.61252057552337646f, 0.74472248554229736f, 0.29666626453399658f, 1.7374711036682129f, -0.13328228890895844f, -0.019352490082383156f, -0.4523082971572876f, -0.41337877511978149f,
    0.76656961441040039f, -0.11182095110416412f, -1.3234167098999023f, 0.97526133060455322f, -0.34311085939407349f, -0.1061074286699295f, 0.23699451982975006f, 0.24008293449878693f,
    -0.49944412708282471f, 0.46227255463600159f, 1.6257579326629639f, -0.3269859254360199f, 0.1049034371972084f, 0.40584272146224976f,
};
static const float drivetrain_b2[5] = {
    0.2540658712387085f, 1.1667499542236328f, -0.16163469851016998f, 0.26659256219863892f, 0.23555710911750793f,
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
    if (!isfinite(engine_speed_rads) || !isfinite(rear_wheelspeed_rads)) {
        return DRIVETRAIN_INV_INVALID;
    }
    float raw[drivetrain_n_in];
    for (int i = 0; i < drivetrain_n_in; ++i) raw[i] = 0.0f;
    raw[0] = engine_speed_rads;
    raw[1] = rear_wheelspeed_rads;
    float x[drivetrain_n_in];
    for (int i = 0; i < drivetrain_n_in; ++i) {
        x[i] = (raw[i] - drivetrain_feat_mean[i]) / drivetrain_feat_std[i];
    }
    float h0[6];
    for (int j = 0; j < 6; ++j) {
        float z = drivetrain_b0[j];
        for (int kk = 0; kk < 2; ++kk) {
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
    3.1671107978358437f, 5.4965188355883292f, 10.076937103908799f, 16.933417491179846f, 23.088041424584272f, 28.99127238190913f, 34.857547550775216f, 42.067697725933378f,
    51.313861621446343f, 60.664257706372858f, 70.574911367239295f, 80.646894429542385f, 90.160860808622871f, 98.322041483710763f, 106.86192373164377f, 120.86476078237993f,
};
static const float drivetrain_i_lo_ww[16] = {
    17.412247712920937f, 11.894224869699404f, 5.8405604037539689f, 3.8822328411020757f, 3.0305228498981287f, 2.4470562830372673f, 2.10565819084949f, 4.4219859725940491f,
    5.3015345707118824f, 5.3997422001588182f, 5.4365258507606082f, 5.4487759691702422f, 5.442032367888082f, 5.4490777339442884f, 5.4496837032973389f, 5.448804666533559f,
};
static const float drivetrain_i_hi_ww[16] = {
    137.48168727649434f, 69.898450732925681f, 51.371856774028323f, 29.638488587322673f, 22.565280035080328f, 19.217473684360748f, 16.37744540876341f, 14.614749802998686f,
    12.583081820735506f, 11.71110604206107f, 11.313723716496451f, 8.2177832744541774f, 7.8396746130421322f, 6.2251052309826109f, 6.053123242582088f, 5.8844961461161773f,
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
    76.411112308502197f, 77.513960361480713f, 78.106554985046387f, 167.97679042816162f, 287.48349952697754f, 338.65267753601074f, 381.85443687438965f, 423.7407398223877f,
    460.16229248046875f, 491.41610717773438f, 519.2676887512207f, 544.54188919067383f, 569.24822616577148f, 596.06912612915039f, 634.75881576538086f, 723.75275039672852f,
};
static const float drivetrain_i_lo_we[16] = {
    2.1763275120612575f, 2.1247535209019852f, 2.1120325962348954f, 2.2671549288109603f, 5.3225041391063552f, 5.4134211026494583f, 5.4381779783150934f, 5.4486473841630083f,
    5.4511134514704489f, 5.4461985844596725f, 5.4418186611880666f, 5.4527875788161939f, 5.4545852644079922f, 5.4496840006752434f, 5.4488045671860181f, 5.6512750243981511f,
};
static const float drivetrain_i_hi_we[16] = {
    22.032457786057186f, 21.793049625775222f, 21.598881255752346f, 100.17185453910621f, 137.48294418604792f, 81.036173083997241f, 50.844801678880842f, 35.631533372416307f,
    27.022374870633747f, 22.265474554873144f, 19.65117667928963f, 18.481764072919244f, 14.602361495300087f, 12.889669188564341f, 12.230636896405718f, 11.730429201881604f,
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

