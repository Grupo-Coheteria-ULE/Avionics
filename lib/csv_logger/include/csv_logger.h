/**
 * @file csv_logger.h
 * @brief CSV formatter and serial debug output for sensor data.
 *
 * Formats sensor samples as CSV lines:
 *   t;ax;ay;az;wx;wy;wz;T;p
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE
 * ---------------------------------------------------------------------------
 */

#ifndef LIB_CSV_LOGGER_CSV_LOGGER_H_
#define LIB_CSV_LOGGER_CSV_LOGGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "imu.h"

/** @brief Maximum length of a formatted CSV line. */
#define CSV_LINE_MAX_LEN    128

/**
 * @brief Sensor data sample with all fields for CSV output.
 */
typedef struct
{
    uint32_t timestamp_ms;      /**< Timestamp in milliseconds */
    imu_raw_t imu;              /**< IMU data (ax,ay,az,wx,wy,wz) */
    float temperature;          /**< Temperature [deg C] (from barometer) */
    float pressure;             /**< Barometer pressure [Pa] */
} csv_sample_t;

/**
 * @brief Formats a sample into a CSV line string.
 *
 * Format: t;ax;ay;az;wx;wy;wz;T;p\n
 *
 * @param sample Pointer to sensor data
 * @param buf Output buffer (must be >= CSV_LINE_MAX_LEN)
 * @return Number of characters written (excluding null terminator)
 */
int csv_format_line(const csv_sample_t *sample, char *buf);

/**
 * @brief Initializes the serial debug backend.
 *
 * Adds a short delay before enabling the peripheral.
 */
void csv_debug_init(void);

/**
 * @brief Sends a formatted CSV line over serial debug (blocking).
 *
 * Does nothing if CONFIG_SERIAL_DEBUG is not defined.
 *
 * @param sample Pointer to sensor data
 */
void csv_debug_print(const csv_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* LIB_CSV_LOGGER_CSV_LOGGER_H_ */
