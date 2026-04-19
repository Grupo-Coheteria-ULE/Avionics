/**
 * @file apogee.h
 * @brief Apogee detection state machine.
 *
 * Detects rocket apogee by monitoring altitude and vertical velocity.
 * Requires sustained negative velocity (descending) for a window
 * of samples before declaring apogee.
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE
 * ---------------------------------------------------------------------------
 */

#ifndef LIB_APOGEE_APOGEE_H_
#define LIB_APOGEE_APOGEE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define APOGEE_MIN_ALT_MM        100000
#define APOGEE_WINDOW_SAMPLES    10

typedef enum {
    APOGEE_IDLE,
    APOGEE_ASCENT,
    APOGEE_DETECTED
} apogee_state_t;

typedef struct {
    apogee_state_t state;
    uint8_t pos_count;
    int32_t last_altitude_mm;
    int32_t last_velocity_mm_s;
} apogee_ctx_t;

/**
 * @brief Initializes apogee detection context.
 * @param ctx Pointer to apogee context
 */
void apogee_init(apogee_ctx_t *ctx);

/**
 * @brief Updates apogee detection with new sensor data.
 *
 * Monitors altitude and velocity to detect descent phase.
 * Must have sustained positive vertical velocity (pressure increase)
 * for APOGEE_WINDOW_SAMPLES consecutive samples.
 *
 * @param ctx Pointer to apogee context
 * @param dt_ms Time delta since last update (milliseconds)
 * @param altitude_mm Current altitude in millimeters
 * @param velocity_mm_s Current vertical velocity in mm/s
 * @return true if apogee detected in this update
 */
bool apogee_update(apogee_ctx_t *ctx, uint32_t dt_ms, int32_t altitude_mm, int32_t velocity_mm_s);

/**
 * @brief Returns current apogee state.
 * @param ctx Pointer to apogee context
 * @return Current state (IDLE, ASCENT, or DETECTED)
 */
apogee_state_t apogee_get_state(const apogee_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* LIB_APOGEE_APOGEE_H_ */