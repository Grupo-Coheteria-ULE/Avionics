/**
 * @file config.h
 * @brief Compile-time configuration for AVI flight software.
 *
 * Enable/disable features by commenting/uncommenting macros.
 * To switch between debug and normal operation mode, just toggle
 * CONFIG_SERIAL_DEBUG and CONFIG_IMU_CALIBRATE.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Feature flags  (comment/uncomment to enable/disable)
 * -------------------------------------------------------------------------- */

/** @brief Run IMU calibration on startup and store offsets in flash. */
#define CONFIG_IMU_CALIBRATE

/** @brief Enable debug output (required for any serial output). */
#define CONFIG_SERIAL_DEBUG

/** @brief Use USB CDC (Virtual COM Port) instead of hardware UART. */
#define CONFIG_CDC

/* --------------------------------------------------------------------------
 * Flash storage for IMU calibration offsets
 * -------------------------------------------------------------------------- */

/**
 * STM32F411 flash sectors 0-3: 16KB each (64KB total)
 * Sector 4: 64KB
 * Sector 5: 128KB   <-- used for calibration storage
 *
 * Base address: 0x08000000
 * Sector 5 base: 0x08020000
 */
#define CONFIG_FLASH_CALIB_ADDR     0x08020000UL
#define CONFIG_FLASH_CALIB_MAGIC    0x43414C49UL   /* "CALI" */

/* --------------------------------------------------------------------------
 * Serial debug
 * -------------------------------------------------------------------------- */

#define CONFIG_SERIAL_BAUDRATE      115200UL

/* --------------------------------------------------------------------------
 * IMU calibration defaults (applied if no flash data found)
 * -------------------------------------------------------------------------- */

#define CONFIG_IMU_DEFAULT_ACC_OFF_X    0.0f
#define CONFIG_IMU_DEFAULT_ACC_OFF_Y    0.0f
#define CONFIG_IMU_DEFAULT_ACC_OFF_Z    0.0f
#define CONFIG_IMU_DEFAULT_GYRO_OFF_X   0.0f
#define CONFIG_IMU_DEFAULT_GYRO_OFF_Y   0.0f
#define CONFIG_IMU_DEFAULT_GYRO_OFF_Z   0.0f

/* --------------------------------------------------------------------------
 * Logging
 * -------------------------------------------------------------------------- */

/** @brief CSV separator character. */
#define CONFIG_CSV_SEP              ';'

/** @brief SD log page size in bytes (must be multiple of 512). */
#define CONFIG_SD_PAGE_BYTES        4096u

/** @brief Number of RAM page buffers for SD logger. */
#define CONFIG_SD_NUM_BUFFERS       4u

/** @brief MS5837 local atmospheric pressure for calibration [Pa].
 *  Set to your actual local pressure (e.g. 92000 for 800m altitude).
 *  On first boot, the raw pressure is measured and a correction factor is computed. */
#define CONFIG_BARO_LOCAL_PRESSURE  92000.0f

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */
