/**
 * @file imu.c
 * @brief LSM6DSO IMU driver with flash-backed calibration.
 *
 * Registers and conversion factors for ST LSM6DSO 6-axis IMU.
 * Communication via SPI1 (CS on PB12).
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE
 * ---------------------------------------------------------------------------
 */

#include "imu.h"
#include "config.h"
#include "main.h"
#include "osal.h"

#include <string.h>

/* --------------------------------------------------------------------------
 * LSM6DSO Register Map
 * -------------------------------------------------------------------------- */

#define LSM6DSO_WHO_AM_I        0x0F
#define LSM6DSO_WHO_AM_I_VAL    0x6C

#define LSM6DSO_CTRL1_XL        0x10    /* Accel ODR and full-scale */
#define LSM6DSO_CTRL2_G         0x11    /* Gyro ODR and full-scale */
#define LSM6DSO_CTRL3_C         0x12    /* IF_INC, SW_RESET, BDU */
#define LSM6DSO_CTRL4_C         0x13
#define LSM6DSO_CTRL5_C         0x14
#define LSM6DSO_CTRL6_G         0x15    /* Gyro full-scale (alt) */
#define LSM6DSO_CTRL7_G         0x16
#define LSM6DSO_CTRL8_XL        0x17
#define LSM6DSO_CTRL9_XL        0x18

#define LSM6DSO_STATUS_REG      0x1E

#define LSM6DSO_OUT_TEMP_L      0x20
#define LSM6DSO_OUT_TEMP_H      0x21
#define LSM6DSO_OUTX_L_G        0x22
#define LSM6DSO_OUTX_H_G        0x23
#define LSM6DSO_OUTY_L_G        0x24
#define LSM6DSO_OUTY_H_G        0x25
#define LSM6DSO_OUTZ_L_G        0x26
#define LSM6DSO_OUTZ_H_G        0x27
#define LSM6DSO_OUTX_L_A        0x28
#define LSM6DSO_OUTX_H_A        0x29
#define LSM6DSO_OUTY_L_A        0x2A
#define LSM6DSO_OUTY_H_A        0x2B
#define LSM6DSO_OUTZ_L_A        0x2C
#define LSM6DSO_OUTZ_H_A        0x2D

/* --------------------------------------------------------------------------
 * Sensitivity factors (LSB per physical unit)
 * -------------------------------------------------------------------------- */

/* Accel +/-4g: 0.122 mg/LSB -> 0.000122 g/LSB */
#define ACC_SENSITIVITY_4G      0.000122f
/* Gyro +/-500 dps: 17.50 mdps/LSB -> 0.0175 dps/LSB */
#define GYRO_SENSITIVITY_500DPS 0.0175f
/* Temp: raw / 256 + 25 */
#define TEMP_SENSITIVITY_INV    0.00390625f  /* 1/256 */
#define TEMP_OFFSET             25.0f

/* --------------------------------------------------------------------------
 * Private state
 * -------------------------------------------------------------------------- */

static imu_cal_t s_cal;
static bool s_cal_loaded = false;

extern SPI_HandleTypeDef hspi1;

static osal_spi_t s_spi_imu;

/* --------------------------------------------------------------------------
 * Chip-select callbacks (CS_IMU on PB12)
 * -------------------------------------------------------------------------- */

static void imu_cs_select(void *user)
{
    (void)user;
    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
}

static void imu_cs_deselect(void *user)
{
    (void)user;
    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);
}

/* --------------------------------------------------------------------------
 * Low-level SPI helpers for LSM6DSO
 * -------------------------------------------------------------------------- */

/**
 * @brief SPI protocol for LSM6DSO:
 *   Read:  send [reg | 0x80], then clock in data bytes.
 *   Write: send [reg & 0x7F], then send data bytes.
 */

static int imu_read_reg(uint8_t reg, uint8_t *val)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0x00 };
    uint8_t rx[2] = { 0 };

    imu_cs_select(NULL);
    if (osal_spi_txrx(&s_spi_imu, tx, rx, 2) != OSAL_OK)
    {
        imu_cs_deselect(NULL);
        return -1;
    }
    imu_cs_deselect(NULL);

    *val = rx[1];
    return 0;
}

static int imu_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t cmd = (uint8_t)(reg | 0x80);
    uint8_t dummy = 0x00;

    imu_cs_select(NULL);

    /* Send register address with read bit, discard first MISO byte */
    if (osal_spi_txrx(&s_spi_imu, &cmd, buf, 1) != OSAL_OK)
    {
        imu_cs_deselect(NULL);
        return -1;
    }

    /* Clock in data bytes */
    for (uint8_t i = 0; i < len; i++)
    {
        if (osal_spi_txrx(&s_spi_imu, &dummy, &buf[i], 1) != OSAL_OK)
        {
            imu_cs_deselect(NULL);
            return -1;
        }
    }

    imu_cs_deselect(NULL);
    return 0;
}

static int imu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t data = val;
    return (osal_spi_reg_write(&s_spi_imu, reg, &data, 1) == OSAL_OK) ? 0 : -1;
}

/* --------------------------------------------------------------------------
 * Configuration
 * -------------------------------------------------------------------------- */

static int imu_who_am_i(void)
{
    uint8_t who = 0;
    if (imu_read_reg(LSM6DSO_WHO_AM_I, &who) != 0)
        return -1;
    return (who == LSM6DSO_WHO_AM_I_VAL) ? 0 : -1;
}

static int imu_configure(void)
{
    if (imu_write_reg(LSM6DSO_CTRL3_C, 0x01) != 0) return -1;
    osal_delay_ms(20);
    if (imu_write_reg(LSM6DSO_CTRL3_C, 0x44) != 0) return -1;
    if (imu_write_reg(LSM6DSO_CTRL1_XL, 0x48) != 0) return -1;
    if (imu_write_reg(LSM6DSO_CTRL2_G, 0x44) != 0) return -1;
    osal_delay_ms(20);
    return 0;
}

/* --------------------------------------------------------------------------
 * Raw data reading
 * -------------------------------------------------------------------------- */

static int imu_read_raw(imu_raw_t *out)
{
    uint8_t buf[14];

    /*
     * Burst read from OUT_TEMP_L (0x20) through OUTZ_H_A (0x2D).
     * With IF_INC enabled, a single burst read of 14 bytes starting
     * at OUT_TEMP_L gives: TEMP(2), GYRO(6), ACCEL(6).
     *
     *   [0] TEMP_L, [1] TEMP_H
     *   [2] GYRO_X_L, [3] GYRO_X_H
     *   [4] GYRO_Y_L, [5] GYRO_Y_H
     *   [6] GYRO_Z_L, [7] GYRO_Z_H
     *   [8] ACCEL_X_L, [9] ACCEL_X_H
     *   [10] ACCEL_Y_L, [11] ACCEL_Y_H
     *   [12] ACCEL_Z_L, [13] ACCEL_Z_H
     */
    if (imu_read_regs(LSM6DSO_OUT_TEMP_L, buf, 14) != 0)
        return -1;

    /* Temperature */
    int16_t raw_temp = (int16_t)((buf[1] << 8) | buf[0]);

    /* Gyroscope */
    int16_t raw_wx = (int16_t)((buf[3]  << 8) | buf[2]);
    int16_t raw_wy = (int16_t)((buf[5]  << 8) | buf[4]);
    int16_t raw_wz = (int16_t)((buf[7]  << 8) | buf[6]);

    /* Accelerometer */
    int16_t raw_ax = (int16_t)((buf[9]  << 8) | buf[8]);
    int16_t raw_ay = (int16_t)((buf[11] << 8) | buf[10]);
    int16_t raw_az = (int16_t)((buf[13] << 8) | buf[12]);

    /* Convert to physical units */
    out->temperature = (float)raw_temp * TEMP_SENSITIVITY_INV + TEMP_OFFSET;

    out->wx = (float)raw_wx * GYRO_SENSITIVITY_500DPS;
    out->wy = (float)raw_wy * GYRO_SENSITIVITY_500DPS;
    out->wz = (float)raw_wz * GYRO_SENSITIVITY_500DPS;

    out->ax = (float)raw_ax * ACC_SENSITIVITY_4G;
    out->ay = (float)raw_ay * ACC_SENSITIVITY_4G;
    out->az = (float)raw_az * ACC_SENSITIVITY_4G;

    return 0;
}

/* --------------------------------------------------------------------------
 * Flash storage
 * -------------------------------------------------------------------------- */

static int flash_write_cal(const imu_cal_t *cal)
{
    HAL_StatusTypeDef st;

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Sector = FLASH_SECTOR_5,
        .NbSectors = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    uint32_t sector_error = 0;

    st = HAL_FLASHEx_Erase(&erase, &sector_error);
    if (st != HAL_OK)
    {
        HAL_FLASH_Lock();
        return -1;
    }

    const uint32_t *src = (const uint32_t *)cal;
    uint32_t addr = CONFIG_FLASH_CALIB_ADDR;
    uint32_t words = (sizeof(imu_cal_t) + 3) / 4;

    for (uint32_t i = 0; i < words; i++)
    {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]);
        if (st != HAL_OK)
        {
            HAL_FLASH_Lock();
            return -1;
        }
        addr += 4;
    }

    HAL_FLASH_Lock();
    return 0;
}

static int flash_read_cal(imu_cal_t *cal)
{
    memcpy(cal, (const void *)CONFIG_FLASH_CALIB_ADDR, sizeof(imu_cal_t));
    return (cal->magic == CONFIG_FLASH_CALIB_MAGIC) ? 0 : -1;
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

int imu_init(void)
{
    osal_spi_init(&s_spi_imu,
                  &hspi1,
                  100,
                  imu_cs_select,
                  imu_cs_deselect,
                  NULL);

    imu_cs_deselect(NULL);
    osal_delay_ms(10); /* Power stabilization */

    if (imu_who_am_i() != 0)
        return -1;

    if (imu_configure() != 0)
        return -1;

    /* Try to load calibration from flash */
    s_cal_loaded = (imu_load_cal() == 0);

    if (!s_cal_loaded)
    {
        memset(&s_cal, 0, sizeof(s_cal));
        s_cal.magic = CONFIG_FLASH_CALIB_MAGIC;
        s_cal.acc_off[0]  = CONFIG_IMU_DEFAULT_ACC_OFF_X;
        s_cal.acc_off[1]  = CONFIG_IMU_DEFAULT_ACC_OFF_Y;
        s_cal.acc_off[2]  = CONFIG_IMU_DEFAULT_ACC_OFF_Z;
        s_cal.gyro_off[0] = CONFIG_IMU_DEFAULT_GYRO_OFF_X;
        s_cal.gyro_off[1] = CONFIG_IMU_DEFAULT_GYRO_OFF_Y;
        s_cal.gyro_off[2] = CONFIG_IMU_DEFAULT_GYRO_OFF_Z;
    }

    return 0;
}

int imu_read(imu_raw_t *out)
{
    if (out == NULL)
        return -1;

    if (imu_read_raw(out) != 0)
        return -1;

    /* Apply calibration offsets */
    out->ax -= s_cal.acc_off[0];
    out->ay -= s_cal.acc_off[1];
    out->az -= s_cal.acc_off[2];

    out->wx -= s_cal.gyro_off[0];
    out->wy -= s_cal.gyro_off[1];
    out->wz -= s_cal.gyro_off[2];

    return 0;
}

int imu_calibrate(uint32_t num_samples)
{
    if (num_samples == 0)
        return -1;

    imu_raw_t sample;
    double sum_ax = 0, sum_ay = 0, sum_az = 0;
    double sum_wx = 0, sum_wy = 0, sum_wz = 0;

    /* Read raw (uncalibrated) samples */
    for (uint32_t i = 0; i < num_samples; i++)
    {
        if (imu_read_raw(&sample) != 0)
            return -1;

        sum_ax += sample.ax;
        sum_ay += sample.ay;
        sum_az += sample.az;

        sum_wx += sample.wx;
        sum_wy += sample.wy;
        sum_wz += sample.wz;

        osal_delay_ms(10);
    }

    s_cal.magic = CONFIG_FLASH_CALIB_MAGIC;
    s_cal.acc_off[0]  = (float)(sum_ax / num_samples);
    s_cal.acc_off[1]  = (float)(sum_ay / num_samples);
    s_cal.acc_off[2]  = (float)((sum_az / num_samples) - 1.0); /* Remove 1g from Z */

    s_cal.gyro_off[0] = (float)(sum_wx / num_samples);
    s_cal.gyro_off[1] = (float)(sum_wy / num_samples);
    s_cal.gyro_off[2] = (float)(sum_wz / num_samples);

    s_cal_loaded = true;

    return imu_save_cal();
}

int imu_load_cal(void)
{
    return flash_read_cal(&s_cal);
}

int imu_save_cal(void)
{
    return flash_write_cal(&s_cal);
}

const imu_cal_t *imu_get_cal(void)
{
    return &s_cal;
}
