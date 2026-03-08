#pragma once
#include <stdbool.h>
#include "lsm6dsl.h"

/* ============================================================================
 * Build selection (CubeIDE recommended)
 * Define ONE of these in Project Properties -> Preprocessor:
 *   - APP_DIAG_BUILD
 *   - APP_FLIGHT_BUILD
 * If none is defined, FLIGHT is assumed.
 * ========================================================================== */
#if !defined(APP_DIAG_BUILD) && !defined(APP_FLIGHT_BUILD)
#define APP_FLIGHT_BUILD
#endif

/* ============================================================================
 * Feature toggles (single place to enable/disable stuff)
 *
 * The app keeps transport- and sensor-specific code inside dedicated modules
 * (baro, imu, log, apogee, ...). That makes it easier to scale the system in
 * the future with additional sensor instances or new subsystems such as GPS,
 * LoRa, actuators, and redundant sensing chains.
 * ========================================================================== */
#if defined(APP_DIAG_BUILD)
  #define APP_USE_BARO        1
  #define APP_USE_IMU         1
  #define APP_USE_SDLOG       0   /* keep DIAG simple by default */
  #define APP_USE_APOGEE      0   /* optional in DIAG */
  #define APP_USE_UART_DIAG   1
#elif defined(APP_FLIGHT_BUILD)
  #define APP_USE_BARO        1
  #define APP_USE_IMU         1
  #define APP_USE_SDLOG       1
  #define APP_USE_APOGEE      1
  #define APP_USE_UART_DIAG   0
#endif

/* -------------------- Planned scalability --------------------
 * These counts are intentionally kept as configuration macros so the app can
 * grow toward multiple barometers / IMUs without redesigning the public types.
 * The current implementation uses the first instance as the primary source.
 * -------------------------------------------------------------------------- */
#define APP_BARO_COUNT      1u
#define APP_IMU_COUNT       1u

/* -------------------- Task rates -------------------- */
#define APP_BARO_HZ         50u
#define APP_IMU_HZ          100u
#define APP_APOGEE_HZ       50u
#define APP_LOG_HZ          50u
#define APP_FLUSH_HZ        200u
#define APP_HEALTH_HZ       10u

/* DIAG: print rate (UART) */
#define APP_DIAG_PRINT_HZ   5u

/* -------------------- Event queue -------------------- */
#define APP_EVENT_Q_LEN     8u

/* -------------------- IMU configuration -------------------- */
#define APP_IMU_ADDR8             ((uint16_t)(LSM6DSL_ADDR7_SA0_0 << 1))
#define APP_IMU_ACCEL_ODR         LSM6DSL_XL_ODR_104HZ
#define APP_IMU_ACCEL_FS          LSM6DSL_XL_FS_16G
#define APP_IMU_GYRO_ODR          LSM6DSL_G_ODR_104HZ
#define APP_IMU_GYRO_FS           LSM6DSL_G_FS_1000DPS
#define APP_IMU_GYRO_FS_125       false

/* The app keeps IMU integration lightweight for now:
 *  - accel + gyro + temperature are sampled and logged
 *  - the flight state machine remains baro-driven
 * This is the safest first step and leaves room for later sensor fusion.
 */

/* -------------------- SDLOG RAW -------------------- */
#define LOG_VERSION        2u

#define LOG_PAGE_BYTES     (32768u)   /* multiple of 512 */
#define LOG_NUM_BUFFERS    (4u)

#define LOG_USE_CRC        (true)
#define LOG_WRAP           (true)
#define LOG_USE_PREERASE   (true)
#define LOG_META_PERIOD_PAGES (16u)

/* Adjust to your REAL RAW layout */
#define LOG_META_LBA       (2048u)
#define LOG_DATA_LBA       (2050u)
#define LOG_END_LBA        (2050u + 200000u)

/* -------------------- Apogee thresholds -------------------- */
#define APOGEE_MIN_ALT_ARM_MM           (5000)   /* 5 m */
#define APOGEE_ASCENT_VEL_ARM_MM_S      (2000)   /* 2 m/s */
#define APOGEE_NEG_VEL_DETECT_MM_S      (-500)   /* -0.5 m/s sustained -> apogee */
#define APOGEE_HOLD_MS                  (200)
#define APOGEE_FILTER_ALPHA_Q15         (3277)   /* ~0.10 in Q15 */

/* -------------------- Baro baseline -------------------- */
#define BARO_BASELINE_WARMUP_MS         (1500u)
