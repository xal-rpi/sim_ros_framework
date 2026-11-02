/* XLE1 snapshot decoding. Wire scalars are little-endian float32;
 * native storage can be float32 or float64. No alignment/host-endian
 * assumptions. */
#include "vehicle_environment.h"
#include <math.h>
#ifdef CR_FLOAT32
#define VE_ABS fabsf
#else
#define VE_ABS fabs
#endif
#define VE_PARAMETER_COUNT VEHICLE_MODEL_PARAMETER_COUNT
#define VE_SNAPSHOT_SIZE (216u + 4u * VE_PARAMETER_COUNT)

static uint16_t u16(const unsigned char *p) {
    return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}
static uint32_t u32(const unsigned char *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static uint64_t u64(const unsigned char *p) {
    return (uint64_t)u32(p) | (uint64_t)u32(p + 4) << 32;
}
static vehicle_real real32(const unsigned char *p) {
    union {
        uint32_t bits;
        float value;
    } x = {.bits = u32(p)};
    return (vehicle_real)x.value;
}
static int magic(const unsigned char *p, char a, char b, char c, char d) {
    return p[0] == (unsigned char)a && p[1] == (unsigned char)b && p[2] == (unsigned char)c &&
           p[3] == (unsigned char)d;
}
static int config_valid(const ve_config *c) {
    return c && isfinite(c->projection_tolerance_m) && c->projection_tolerance_m > 0 &&
           c->projection_max_iterations > 0 && c->projection_max_iterations <= 24;
}
static int same(const vehicle_real *a, const vehicle_real *b, int count) {
    for (int i = 0; i < count; ++i)
        if (a[i] != b[i])
            return 0;
    return 1;
}
int ve_abi_version(void) { return 1; }
size_t ve_packet_size(void) { return VE_SNAPSHOT_SIZE; }
size_t ve_context_size(void) { return sizeof(ve_context); }
int ve_scalar_bytes(void) { return (int)sizeof(vehicle_real); }
int ve_parameter_count(void) { return VE_PARAMETER_COUNT; }
void ve_reset(ve_context *c) {
    if (c)
        *c = (ve_context){0};
}

int ve_receive(ve_context *c, const ve_config *cfg, const unsigned char *p, size_t length) {
    if (!c || !config_valid(cfg) || !p || length != VE_SNAPSHOT_SIZE)
        return VE_BAD_PACKET;
    if (!magic(p, 'X', 'L', 'E', '1') || u16(p + 4) != 1u || (u16(p + 6) & ~7u))
        return VE_BAD_PACKET;
    if (u32(p + 8) != cfg->frame_id || u32(p + 12) != cfg->epoch_id ||
        u64(p + 16) != VEHICLE_PARAMETER_LAYOUT_ID)
        return VE_WRONG_IDENTITY;
    ve_context candidate = {0};
    candidate.valid = 1;
    candidate.flags = u16(p + 6);
    candidate.revision = u32(p + 24);
    candidate.geometry_revision = u32(p + 28);
    candidate.parameter_revision = u32(p + 32);
    candidate.state_time_us = u64(p + 36);
    candidate.geometry_source_time_us = u64(p + 44);
    candidate.parameter_source_time_us = u64(p + 52);
    if ((candidate.geometry_source_time_us > candidate.state_time_us) ||
        (candidate.parameter_source_time_us > candidate.state_time_us) ||
        (!(candidate.flags & 1u) && candidate.geometry_source_time_us) ||
        (!(candidate.flags & 2u) && candidate.parameter_source_time_us))
        return VE_BAD_PACKET;
    for (unsigned int i = 0; i < 39u + VE_PARAMETER_COUNT; ++i) {
        vehicle_real value = real32(p + 60u + 4u * i);
        if (!isfinite(value))
            return VE_BAD_PACKET;
        if (i < 27u)
            candidate.surface.patch[i] = value;
        else if (i < 33u)
            candidate.surface.bounds[i - 27u] = value;
        else if (i < 39u)
            candidate.parameter_bounds_xyz[i - 33u] = value;
        else
            candidate.parameters[i - 39u] = value;
    }
    for (int i = 0; i < 6; i += 2) {
        if (candidate.surface.bounds[i] >= candidate.surface.bounds[i + 1])
            return VE_BAD_PACKET;
        if ((candidate.flags & 4u) &&
            candidate.parameter_bounds_xyz[i] > candidate.parameter_bounds_xyz[i + 1])
            return VE_BAD_PACKET;
        if (!(candidate.flags & 4u) &&
            (candidate.parameter_bounds_xyz[i] || candidate.parameter_bounds_xyz[i + 1]))
            return VE_BAD_PACKET;
    }
    const vehicle_real *r = candidate.surface.patch + 3;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            vehicle_real dot = 0;
            for (int k = 0; k < 3; ++k)
                dot += r[3 * i + k] * r[3 * j + k];
            if (VE_ABS(dot - (i == j ? 1 : 0)) > (vehicle_real)1e-6)
                return VE_BAD_PACKET;
        }
    vehicle_real det = r[0] * (r[4] * r[8] - r[7] * r[5]) - r[3] * (r[1] * r[8] - r[7] * r[2]) +
                       r[6] * (r[1] * r[5] - r[4] * r[2]);
    if (VE_ABS(det - 1) > (vehicle_real)1e-6)
        return VE_BAD_PACKET;
    if (c->valid) {
        if (candidate.revision < c->revision ||
            candidate.geometry_revision < c->geometry_revision ||
            candidate.parameter_revision < c->parameter_revision ||
            candidate.state_time_us < c->state_time_us)
            return VE_OLD_SNAPSHOT;
        int geometry_same = same(candidate.surface.patch, c->surface.patch, 27) &&
                            same(candidate.surface.bounds, c->surface.bounds, 6) &&
                            ((candidate.flags ^ c->flags) & 1u) == 0u &&
                            candidate.geometry_source_time_us == c->geometry_source_time_us;
        int parameters_same = same(candidate.parameters, c->parameters, VE_PARAMETER_COUNT) &&
                              same(candidate.parameter_bounds_xyz, c->parameter_bounds_xyz, 6) &&
                              ((candidate.flags ^ c->flags) & 6u) == 0u &&
                              candidate.parameter_source_time_us == c->parameter_source_time_us;
        if ((candidate.geometry_revision == c->geometry_revision && !geometry_same) ||
            (candidate.parameter_revision == c->parameter_revision && !parameters_same) ||
            (candidate.revision == c->revision &&
             (candidate.geometry_revision != c->geometry_revision ||
              candidate.parameter_revision != c->parameter_revision ||
              candidate.state_time_us != c->state_time_us)))
            return VE_CHANGED_REVISION;
    }
    candidate.surface.tolerance_m = cfg->projection_tolerance_m;
    candidate.surface.max_iterations = cfg->projection_max_iterations;
    /* Initial normal-offset seed is geometric, not a vehicle height assumption.
     */
    candidate.surface.seed[2] = 0;
    *c = candidate;
    return VE_OK;
}

int ve_prepare(const ve_context *c, const ve_config *cfg, const vehicle_state *state,
               vehicle_query *q) {
    if (!q)
        return VE_INVALID_STATE;
    *q = (vehicle_query){0};
    if (!c || !config_valid(cfg))
        return VE_BAD_PACKET;
    if (!c->valid)
        return VE_NO_CONTEXT;
    if (!state)
        return VE_INVALID_STATE;
    if (c->flags & 4u)
        for (int i = 0; i < 3; ++i)
            if (state->position_world_m[i] < c->parameter_bounds_xyz[2 * i] ||
                state->position_world_m[i] > c->parameter_bounds_xyz[2 * i + 1])
                return VE_OUTSIDE_PARAMETERS;
    int projected = vehicle_prepare(state, &c->surface, q);
    return projected == VEHICLE_OK                  ? VE_OK
           : projected == VEHICLE_PROJECTION_FAILED ? VE_PROJECTION_FAILED
                                                    : VE_INVALID_STATE;
}
