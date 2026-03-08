#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
  APP_STATE_IDLE = 0,
  APP_STATE_ASCENT,
  APP_STATE_APOGEE
} app_state_t;

enum {
  FLAG_BARO_OK   = (1u << 0),
  FLAG_SD_OK     = (1u << 1),
  FLAG_LOG_OK    = (1u << 2),
  FLAG_APOGEE    = (1u << 3),
  FLAG_DROPPED   = (1u << 4),
  FLAG_IMU_OK    = (1u << 5),
};

/* -------------------- Event bus -------------------- */
typedef enum {
  EVT_NONE = 0,
  EVT_APOGEE_DETECTED,
  EVT_BARO_FAIL,
  EVT_SD_FAIL,
  EVT_IMU_FAIL,
} app_event_id_t;

typedef struct {
  app_event_id_t id;
  uint32_t t_ms;
  int32_t  v;     /* generic payload (e.g. alt_mm or driver status) */
} app_event_t;

/* -------------------- Telemetry snapshot --------------------
 * The snapshot is intentionally "flat" and log-friendly:
 *  - fast to copy
 *  - easy to serialize
 *  - easy to extend with additional sensors
 *
 * For future growth, keep adding new sensor blocks instead of overloading the
 * existing fields. The current primary navigation chain is still barometric.
 * -------------------------------------------------------------------------- */
typedef struct {
  uint32_t t_ms;

  /* Primary barometric solution */
  int32_t p_centi_mbar;
  int32_t t_centi_c;
  int32_t alt_mm;
  int32_t vel_mm_s;

  /* Primary IMU snapshot */
  int32_t ax_ug;
  int32_t ay_ug;
  int32_t az_ug;

  int32_t gx_udps;
  int32_t gy_udps;
  int32_t gz_udps;

  int32_t imu_t_centi_c;

  uint32_t flags;
  uint16_t state;
  uint16_t reserved;

  uint32_t dropped_bytes;
} telemetry_t;
