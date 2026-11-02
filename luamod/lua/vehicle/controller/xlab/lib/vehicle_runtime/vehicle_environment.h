#ifndef VEHICLE_ENVIRONMENT_H
#define VEHICLE_ENVIRONMENT_H

#include "vehicle_export_config.h"
#include "vehicle_runtime.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/* FFI_BEGIN */
enum ve_status {
    VE_OK = 0,
    VE_NO_CONTEXT = 1,
    VE_BAD_PACKET = 2,
    VE_WRONG_IDENTITY = 3,
    VE_OLD_SNAPSHOT = 4,
    VE_OUTSIDE_PARAMETERS = 5,
    VE_CHANGED_REVISION = 6,
    VE_INVALID_STATE = 7,
    VE_PROJECTION_FAILED = 8
};
typedef struct {
    uint32_t frame_id;
    uint32_t epoch_id; /* explicitly configured session; never auto-adopt */
    vehicle_real projection_tolerance_m;
    int projection_max_iterations;
} ve_config;

typedef struct {
    vehicle_surface surface;
    vehicle_real parameters[VEHICLE_MODEL_PARAMETER_STORAGE];
    vehicle_real parameter_bounds_xyz[6];
    uint64_t state_time_us;
    uint64_t geometry_source_time_us;
    uint64_t parameter_source_time_us;
    uint32_t revision;
    uint32_t geometry_revision;
    uint32_t parameter_revision;
    unsigned int flags;
    int valid;
} ve_context;

/* Caller owns all storage; no heap, socket, global state or retained pointers.
 * Reset once per configured frame/epoch. Receive validates bytes into temporary
 * storage, then replaces the complete context. Serialize calls in one consumer.
 * Bad/old packets leave the accepted context unchanged. Repeated identical
 * snapshots are accepted without changing source/assembly times. No
 * receive-time timeout or delivery lease: caller supervises producer health
 * separately. Wire size and parameter layout are fixed by the selected vehicle
 * export.
 */
int ve_abi_version(void); /* Environment API version, independent of rf_abi_version. */
size_t ve_packet_size(void);
size_t ve_context_size(void);
int ve_scalar_bytes(void);
int ve_parameter_count(void);
void ve_reset(ve_context *context);
int ve_receive(ve_context *context, const ve_config *config, const unsigned char *bytes,
               size_t length);
/* Check independent parameter XYZ bounds and surface projection. Surface domain
 * bounds are diagnostic: a regular projection outside them remains usable and
 * has query.projection_status=4, valid=1. There is no
 * absolute clock or freshness policy in model preparation. Packet timestamps
 * are retained as metadata for a consumer/supervisor that needs an age policy.
 * On failure query.valid=0. Parameter storage is owned by context; callers pass
 * {context.parameters, vehicle_model_parameter_count()} during the same call
 * chain.
 */
int ve_prepare(const ve_context *context, const ve_config *config, const vehicle_state *state,
               vehicle_query *query);
/* FFI_END */
#ifdef __cplusplus
}
#endif
#endif
