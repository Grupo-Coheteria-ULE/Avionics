/**
 * @file ms5837.h
 * @brief MS5837-30BA barometer driver (I2C).
 *
 * Measures pressure (Pa) and temperature (deg C).
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE
 * ---------------------------------------------------------------------------
 */

#ifndef LIB_MS5837_MS5837_H_
#define LIB_MS5837_MS5837_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** @brief Barometer data. */
typedef struct
{
    float pressure;         /**< Pressure [Pa] */
    float temperature;      /**< Temperature [deg C] */
} ms5837_data_t;

/** @brief MS5837 driver instance. */
typedef struct
{
    void *i2c_bus;          /**< Opaque I2C bus (osal_i2c_t *) */
    uint16_t prom[7];       /**< PROM calibration coefficients (public for debug) */
    uint32_t last_D1;       /**< Last raw pressure ADC value */
    uint32_t last_D2;       /**< Last raw temperature ADC value */
    bool initialized;
} ms5837_t;

/**
 * @brief Initializes the MS5837 (reset + read PROM).
 * @param dev Driver instance
 * @param i2c_bus Pointer to osal_i2c_t
 * @return 0 on success, -1 on error
 */
int ms5837_init(ms5837_t *dev, void *i2c_bus);

/**
 * @brief Reads pressure and temperature.
 * @param dev Driver instance
 * @param out Output data
 * @return 0 on success, -1 on error
 */
int ms5837_read(ms5837_t *dev, ms5837_data_t *out);

#ifdef __cplusplus
}
#endif

#endif /* LIB_MS5837_MS5837_H_ */
