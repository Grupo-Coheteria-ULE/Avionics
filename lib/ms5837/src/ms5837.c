/**
 * @file ms5837.c
 * @brief MS5837-30BA barometer driver (I2C).
 *
 * Protocol based on BlueRobotics_MS5837_Library.
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE
 * ---------------------------------------------------------------------------
 */

#include "ms5837.h"
#include "osal.h"
#include "stm32f4xx_hal.h"
#include "config.h"
#include <string.h>

/* --------------------------------------------------------------------------
 * MS5837 commands (BlueRobotics uses OSR=8192)
 * -------------------------------------------------------------------------- */

#define MS5837_ADDR             0x76
#define MS5837_RESET            0x1E
#define MS5837_ADC_READ         0x00
#define MS5837_PROM_BASE        0xA0
#define MS5837_PROM_WORDS       7
#define MS5837_CONVERT_D1       0x4A    /* Pressure, OSR=8192 */
#define MS5837_CONVERT_D2       0x5A    /* Temperature, OSR=8192 */

/* Conversion time for OSR=8192: ~17.6 ms, use 20ms (BlueRobotics value) */
#define MS5837_CONV_DELAY_MS    20

/* --------------------------------------------------------------------------
 * Low-level helpers
 * -------------------------------------------------------------------------- */

static int ms5837_cmd(ms5837_t *dev, uint8_t cmd)
{
    osal_i2c_t *bus = (osal_i2c_t *)dev->i2c_bus;
    return (osal_i2c_write(bus, osal_i2c_addr8(MS5837_ADDR), &cmd, 1) == OSAL_OK) ? 0 : -1;
}

/**
 * BlueRobotics protocol:
 *   1. beginTransmission(addr) + write(0x00) + endTransmission()  → HAL_I2C_Master_Transmit
 *   2. requestFrom(addr, 3)                                       → HAL_I2C_Master_Receive
 * Two SEPARATE I2C transactions with STOP between them.
 */
static int ms5837_read_adc(ms5837_t *dev, uint32_t *value)
{
    osal_i2c_t *bus = (osal_i2c_t *)dev->i2c_bus;
    uint8_t cmd = MS5837_ADC_READ;
    uint8_t buf[3] = {0};

    /* Step 1: Send ADC read command (0x00) — transaction with STOP */
    if (HAL_I2C_Master_Transmit(bus->hi2c, osal_i2c_addr8(MS5837_ADDR),
                                &cmd, 1, 100) != HAL_OK)
        return -1;

    /* Small delay before reading */
    osal_delay_ms(1);

    /* Step 2: Read 3 bytes — separate transaction with STOP */
    if (HAL_I2C_Master_Receive(bus->hi2c, osal_i2c_addr8(MS5837_ADDR),
                               buf, 3, 100) != HAL_OK)
        return -1;

    *value = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    return 0;
}

/* --------------------------------------------------------------------------
 * Compensation with auto-calibration
 * -------------------------------------------------------------------------- */

static float s_pressure_offset = 0.0f;
static bool  s_calibrated = false;

static void ms5837_compensate(uint16_t *C, uint32_t D1, uint32_t D2,
                               float *pressure, float *temperature)
{
    int32_t dT = (int32_t)D2 - (int32_t)C[5] * 256L;
    int64_t SENS = (int64_t)C[1] * 32768L + ((int64_t)C[3] * dT) / 256L;
    int64_t OFF  = (int64_t)C[2] * 65536L + ((int64_t)C[4] * dT) / 128L;
    int32_t P    = (D1 * SENS / 2097152L - OFF) / 8192L;

    int32_t TEMP = 2000L + (int64_t)dT * C[6] / 8388608LL;

    /* Second order compensation */
    int32_t Ti = 0, OFFi = 0, SENSi = 0;

    if ((TEMP / 100) < 20)
    {
        Ti    = (3L * (int64_t)dT * dT) / 8589934592LL;
        OFFi  = (3L * (TEMP - 2000L) * (TEMP - 2000L)) / 2L;
        SENSi = (5L * (TEMP - 2000L) * (TEMP - 2000L)) / 8L;
        if ((TEMP / 100) < -15)
        {
            OFFi  = OFFi  + 7L * (TEMP + 1500L) * (TEMP + 1500L);
            SENSi = SENSi + 4L * (TEMP + 1500L) * (TEMP + 1500L);
        }
    }
    else if ((TEMP / 100) >= 20)
    {
        Ti    = 2L * ((int64_t)dT * dT) / 137438953472LL;
        OFFi  = (1L * (TEMP - 2000L) * (TEMP - 2000L)) / 16L;
        SENSi = 0;
    }

    int64_t OFF2  = OFF  - OFFi;
    int64_t SENS2 = SENS - SENSi;

    TEMP = TEMP - Ti;
    P = ((D1 * SENS2) / 2097152L - OFF2) / 8192L;

    /* Raw pressure in Pa */
    float raw_p = (float)P / 10.0f * 100.0f;

    /* Auto-calibrate: compute fixed offset on first valid read */
    if (!s_calibrated && raw_p > 0.0f)
    {
        s_pressure_offset = raw_p - CONFIG_BARO_LOCAL_PRESSURE;
        s_calibrated = true;
    }

    *pressure    = raw_p - s_pressure_offset;
    *temperature = (float)TEMP / 100.0f;
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

int ms5837_init(ms5837_t *dev, void *i2c_bus)
{
    if ((dev == NULL) || (i2c_bus == NULL))
        return -1;

    dev->i2c_bus = i2c_bus;
    dev->initialized = false;
    memset(dev->prom, 0, sizeof(dev->prom));

    /* Reset */
    if (ms5837_cmd(dev, MS5837_RESET) != 0)
        return -1;
    osal_delay_ms(10);

    /* Read PROM (7 words) — BlueRobotics uses write+endTransmission then requestFrom */
    for (uint8_t i = 0; i < MS5837_PROM_WORDS; i++)
    {
        uint8_t addr = MS5837_PROM_BASE + (i << 1);
        uint8_t buf[2] = {0};

        osal_i2c_t *bus = (osal_i2c_t *)dev->i2c_bus;

        /* Send PROM address — transaction with STOP */
        if (HAL_I2C_Master_Transmit(bus->hi2c, osal_i2c_addr8(MS5837_ADDR),
                                    &addr, 1, 100) != HAL_OK)
            return -2;

        /* Read 2 bytes — separate transaction with STOP */
        if (HAL_I2C_Master_Receive(bus->hi2c, osal_i2c_addr8(MS5837_ADDR),
                                   buf, 2, 100) != HAL_OK)
            return -2;

        dev->prom[i] = ((uint16_t)buf[0] << 8) | buf[1];
    }

    if (dev->prom[1] == 0)
        return -3;

    dev->initialized = true;
    return 0;
}

int ms5837_read(ms5837_t *dev, ms5837_data_t *out)
{
    if ((dev == NULL) || (out == NULL) || !dev->initialized)
        return -1;

    uint32_t D1, D2;

    /* Convert D1 (pressure) — BlueRobotics protocol */
    if (ms5837_cmd(dev, MS5837_CONVERT_D1) != 0)
        return -1;
    osal_delay_ms(MS5837_CONV_DELAY_MS);
    if (ms5837_read_adc(dev, &D1) != 0)
        return -2;

    /* Convert D2 (temperature) */
    if (ms5837_cmd(dev, MS5837_CONVERT_D2) != 0)
        return -3;
    osal_delay_ms(MS5837_CONV_DELAY_MS);
    if (ms5837_read_adc(dev, &D2) != 0)
        return -4;

    if ((D1 == 0) && (D2 == 0))
        return -5;

    dev->last_D1 = D1;
    dev->last_D2 = D2;

    ms5837_compensate(dev->prom, D1, D2, &out->pressure, &out->temperature);

    return 0;
}
