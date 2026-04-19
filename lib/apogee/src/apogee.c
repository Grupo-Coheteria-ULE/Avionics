/**
 * @file apogee.c
 * @brief Apogee detection state machine implementation.
 *
 * State machine logic:
 * - IDLE: Wait until altitude > MIN_ALT (100m) to arm
 * - ASCENT: Monitor velocity. If velocity < 0 (descending = pressure increasing)
 *           for WINDOW samples, transition to DETECTED
 * - DETECTED: Terminal state, parachute deployed
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE
 * ---------------------------------------------------------------------------
 */

#include "apogee.h"
#include <string.h>

void apogee_init(apogee_ctx_t *ctx)
{
    if (!ctx) return;

    memset(ctx, 0, sizeof(*ctx));
    ctx->state = APOGEE_IDLE;
    ctx->pos_count = 0;
}

bool apogee_update(apogee_ctx_t *ctx, uint32_t dt_ms, int32_t altitude_mm, int32_t velocity_mm_s)
{
    if (!ctx) return false;

    ctx->last_altitude_mm = altitude_mm;
    ctx->last_velocity_mm_s = velocity_mm_s;

    switch (ctx->state) {
        case APOGEE_IDLE:
            if (altitude_mm > APOGEE_MIN_ALT_MM) {
                ctx->state = APOGEE_ASCENT;
                ctx->pos_count = 0;
            }
            break;

        case APOGEE_ASCENT:
            if (velocity_mm_s < 0) {
                ctx->pos_count++;
                if (ctx->pos_count >= APOGEE_WINDOW_SAMPLES) {
                    ctx->state = APOGEE_DETECTED;
                    return true;
                }
            } else {
                ctx->pos_count = 0;
            }
            break;

        case APOGEE_DETECTED:
        default:
            break;
    }

    return false;
}

apogee_state_t apogee_get_state(const apogee_ctx_t *ctx)
{
    return (ctx) ? ctx->state : APOGEE_IDLE;
}