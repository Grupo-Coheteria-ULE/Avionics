/**
 * @file altitude.h
 * @brief Altitude calculation from barometric pressure.
 *
 * Uses barometric formula to compute relative altitude from pressure,
 * and calculates vertical velocity from altitude changes.
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE
 * ---------------------------------------------------------------------------
 */

#ifndef LIB_ALTITUDE_ALTITUDE_H_
#define LIB_ALTITUDE_ALTITUDE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define ALTITUDE_EXPONENT      0.1903f
#define ALTITUDE_SCALE_M     44330.0f
#define ALTITUDE_SCALE_MM   44330000.0f

typedef struct {
    float base_pressure_pa;
    int32_t last_altitude_mm;
    int32_t last_velocity_mm_s;
    uint32_t last_timestamp_ms;
    bool initialized;
} altitude_ctx_t;

/**
 * @brief Initializes altitude context with baseline pressure.
 * @param ctx Pointer to altitude context
 * @param pressure_pa Baseline pressure in Pascals
 */
void altitude_init(altitude_ctx_t *ctx, float pressure_pa);

/**
 * @brief Updates altitude and computes vertical velocity.
 * @param ctx Pointer to altitude context
 * @param pressure_pa Current pressure in Pascals
 * @param now_ms Current timestamp in milliseconds
 * @return Current altitude in millimeters (relative to base)
 */
int32_t altitude_update(altitude_ctx_t *ctx, float pressure_pa, uint32_t now_ms);

/**
 * @brief Returns the last computed altitude.
 * @param ctx Pointer to altitude context
 * @return Last altitude in millimeters
 */
int32_t altitude_get(const altitude_ctx_t *ctx);

/**
 * @brief Returns the last computed vertical velocity.
 * @param ctx Pointer to altitude context
 * @return Vertical velocity in mm/s
 */
int32_t altitude_get_velocity(const altitude_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* LIB_ALTITUDE_ALTITUDE_H_ */