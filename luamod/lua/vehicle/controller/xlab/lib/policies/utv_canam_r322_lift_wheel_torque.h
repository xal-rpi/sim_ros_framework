/* Vendored from xal_vehicle_models generated/analysis/torque_table/
 * utv_canam_r322_lift_wheel_torque_and_brake/nn/drivetrain_struct.{c,h} — rwd_struct.
 * T = g(x) + Σ I_k(u) F_k(x). Stem include only.
 */
#ifndef DRIVETRAIN_STRUCT_H
#define DRIVETRAIN_STRUCT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DRIVETRAIN_INV_OK = 0,
    DRIVETRAIN_INV_SAT_COAST = 1,
    DRIVETRAIN_INV_SAT_FULL = 2,
    DRIVETRAIN_INV_NO_SAFE_BRANCH = 3,
    DRIVETRAIN_INV_BAD_ENDPOINTS = 4,
    DRIVETRAIN_INV_OUT_OF_DOMAIN = 5,
    DRIVETRAIN_INV_INVALID = 6
} drivetrain_inv_status;

enum { drivetrain_k = 4, drivetrain_n_in = 3, drivetrain_n_out = 5 };

int drivetrain_heads(float engine_speed_rads, float boost_pressure, float rear_wheelspeed_rads,
                   float *g_out, float *F_out);
float drivetrain_mix(float g, const float *F, float throttle_cmd);
float drivetrain_forward_torque(float engine_speed_rads, float throttle_cmd,
                              float boost_pressure, float rear_wheelspeed_rads);
float drivetrain_inverse_throttle(float engine_speed_rads, float boost_pressure,
                                float rear_wheelspeed_rads, float torque_cmd);
int drivetrain_inverse_throttle_ex(float engine_speed_rads, float boost_pressure,
                                 float rear_wheelspeed_rads, float torque_cmd,
                                 float *u_out, float *t_hat);
float drivetrain_inverse_throttle_knots(float engine_speed_rads, float boost_pressure,
                                      float rear_wheelspeed_rads, float torque_cmd);
int drivetrain_inverse_throttle_knots_ex(float engine_speed_rads, float boost_pressure,
                                       float rear_wheelspeed_rads, float torque_cmd,
                                       float *u_out, float *t_hat);
int drivetrain_ww_band(float engine_speed_rads, float *ww_lo, float *ww_hi);
int drivetrain_we_band(float rear_wheelspeed_rads, float *we_lo, float *we_hi);

extern const float drivetrain_ww_ok_lo;
extern const float drivetrain_ww_ok_hi;
extern const float drivetrain_we_ok_lo;
extern const float drivetrain_we_ok_hi;
extern const float drivetrain_j_eff;
extern const float drivetrain_r;
extern const float drivetrain_r_kin;
extern const float drivetrain_brake_coeff_rear;
extern const int drivetrain_n_u;
extern const int drivetrain_n_bisect;

#ifdef __cplusplus
}
#endif

#endif /* DRIVETRAIN_STRUCT_H */
