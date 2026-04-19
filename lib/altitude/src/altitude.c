/**
 * @file altitude.c
 * @brief Altitude calculation implementation.
 *
 * Barometric formula: h = 44330 * (1 - p/p0)^0.1903 meters
 * Converted to millimeters for integer precision.
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE
 * ---------------------------------------------------------------------------
 */

#include "altitude.h"
#include <math.h>
#include <string.h>

void altitude_init(altitude_ctx_t *ctx, float pressure_pa)
{
    if (!ctx) return;

    memset(ctx, 0, sizeof(*ctx));
    ctx->base_pressure_pa = pressure_pa;
    ctx->last_altitude_mm = 0;
    ctx->last_timestamp_ms = 0;
    ctx->initialized = true;
}

int32_t altitude_update(altitude_ctx_t *ctx, float pressure_pa, uint32_t now_ms)
{
    if (!ctx || !ctx->initialized) return 0;

    if (ctx->base_pressure_pa <= 0.0f) return 0;

    float ratio = pressure_pa / ctx->base_pressure_pa;

    if (ratio <= 0.0f) return ctx->last_altitude_mm;

    float exponent = powf(ratio, ALTITUDE_EXPONENT);
    float altitude_m = ALTITUDE_SCALE_M * (1.0f - exponent);

    int32_t new_altitude_mm = (int32_t)(altitude_m * 1000.0f);

    int32_t velocity_mm_s = 0;
    if (ctx->last_timestamp_ms > 0 && now_ms > ctx->last_timestamp_ms) {
        int32_t dt_ms = (int32_t)(now_ms - ctx->last_timestamp_ms);
        if (dt_ms >= 10) {
            velocity_mm_s = ((new_altitude_mm - ctx->last_altitude_mm) * 1000) / dt_ms;
        }
    }

    ctx->last_altitude_mm = new_altitude_mm;
    ctx->last_velocity_mm_s = velocity_mm_s;
    ctx->last_timestamp_ms = now_ms;

    return velocity_mm_s;
}

int32_t altitude_get(const altitude_ctx_t *ctx)
{
    return (ctx) ? ctx->last_altitude_mm : 0;
}

int32_t altitude_get_velocity(const altitude_ctx_t *ctx)
{
    return (ctx) ? ctx->last_velocity_mm_s : 0;
}