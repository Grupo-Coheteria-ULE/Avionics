/**
 * @file imu.h
 * @brief Generic IMU driver with calibration offset storage in flash.
 *
 * Provides sensor reading, calibration, and flash-backed offset persistence.
 * The register-level SPI/I2C communication is left as a stub for the user to
 * implement for their specific IMU (e.g. ICM-42688-P, MPU-6500, BMI270).
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE
 * ---------------------------------------------------------------------------
 */

#ifndef LIB_IMU_IMU_H_
#define LIB_IMU_IMU_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* --------------------------------------------------------------------------
 * Data types
 * -------------------------------------------------------------------------- */

/** @brief Raw sensor data from the IMU. */
typedef struct
{
    float ax, ay, az;       /**< Linear acceleration [m/s^2 or g] */
    float wx, wy, wz;       /**< Angular velocity [rad/s or deg/s] */
    float temperature;      /**< IMU temperature [deg C] */
} imu_raw_t;

/** @brief Calibration offsets stored in flash. */
typedef struct
{
    uint32_t magic;         /**< Validation magic: 0x43414C49 */
    float acc_off[3];       /**< Accelerometer offsets (x, y, z) */
    float gyro_off[3];      /**< Gyroscope offsets (x, y, z) */
} imu_cal_t;

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

/**
 * @brief Initializes the IMU hardware.
 *
 * Configures the sensor registers (range, ODR, filters, etc.)
 * and attempts to load calibration offsets from flash.
 *
 * @return 0 on success, negative on error
 */
int imu_init(void);

/**
 * @brief Reads a single sample from the IMU.
 *
 * Applies stored calibration offsets automatically.
 *
 * @param out Pointer to output data structure
 * @return 0 on success, negative on error
 */
int imu_read(imu_raw_t *out);

/**
 * @brief Runs a calibration routine (blocking).
 *
 * Takes N samples while the sensor is stationary and computes
 * average offsets. Accelerometer Z is corrected to 1g.
 * Result is stored in flash automatically.
 *
 * @param num_samples Number of samples to average (e.g. 1000)
 * @return 0 on success, negative on error
 */
int imu_calibrate(uint32_t num_samples);

/**
 * @brief Loads calibration offsets from flash.
 *
 * @return 0 if valid offsets found, -1 if no calibration data
 */
int imu_load_cal(void);

/**
 * @brief Saves current calibration offsets to flash.
 *
 * @return 0 on success, negative on error
 */
int imu_save_cal(void);

/**
 * @brief Returns pointer to current calibration data.
 */
const imu_cal_t *imu_get_cal(void);

#ifdef __cplusplus
}
#endif

#endif /* LIB_IMU_IMU_H_ */
