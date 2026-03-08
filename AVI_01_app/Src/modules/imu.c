#include "modules/imu.h"
#include <string.h>

bool imu_init(imu_t *imu, osal_i2c_t *i2c, uint16_t addr8)
{
  if (!imu || !i2c) return false;
  memset(imu, 0, sizeof(*imu));

  lsm6dsl_status_t st = lsm6dsl_init_i2c(&imu->dev, i2c, addr8);
  if (st != LSM6DSL_OK) {
    imu->last_status = (int32_t)st;
    return false;
  }

  st = lsm6dsl_set_accel(&imu->dev, APP_IMU_ACCEL_ODR, APP_IMU_ACCEL_FS);
  if (st != LSM6DSL_OK) {
    imu->last_status = (int32_t)st;
    return false;
  }

  st = lsm6dsl_set_gyro(&imu->dev, APP_IMU_GYRO_ODR, APP_IMU_GYRO_FS, APP_IMU_GYRO_FS_125);
  if (st != LSM6DSL_OK) {
    imu->last_status = (int32_t)st;
    return false;
  }

  imu->ready = true;
  imu->last_status = (int32_t)LSM6DSL_OK;
  return true;
}

bool imu_update(imu_t *imu, imu_sample_t *out)
{
  if (!imu || !out || !imu->ready) return false;

  bool xl_ready = false;
  bool g_ready = false;
  bool t_ready = false;

  lsm6dsl_status_t st = lsm6dsl_status(&imu->dev, &xl_ready, &g_ready, &t_ready);
  if (st != LSM6DSL_OK) {
    imu->last_status = (int32_t)st;
    return false;
  }

  int16_t a_raw[3] = {0,0,0};
  int16_t g_raw[3] = {0,0,0};
  int16_t t_raw = 0;

  if (xl_ready) {
    st = lsm6dsl_read_accel_raw(&imu->dev, a_raw);
    if (st != LSM6DSL_OK) {
      imu->last_status = (int32_t)st;
      return false;
    }
  }

  if (g_ready) {
    st = lsm6dsl_read_gyro_raw(&imu->dev, g_raw);
    if (st != LSM6DSL_OK) {
      imu->last_status = (int32_t)st;
      return false;
    }
  }

  if (t_ready) {
    st = lsm6dsl_read_temp_raw(&imu->dev, &t_raw);
    if (st != LSM6DSL_OK) {
      imu->last_status = (int32_t)st;
      return false;
    }
  }

  out->ax_ug = lsm6dsl_accel_raw_to_ug(&imu->dev, a_raw[0]);
  out->ay_ug = lsm6dsl_accel_raw_to_ug(&imu->dev, a_raw[1]);
  out->az_ug = lsm6dsl_accel_raw_to_ug(&imu->dev, a_raw[2]);

  out->gx_udps = lsm6dsl_gyro_raw_to_udps(&imu->dev, g_raw[0]);
  out->gy_udps = lsm6dsl_gyro_raw_to_udps(&imu->dev, g_raw[1]);
  out->gz_udps = lsm6dsl_gyro_raw_to_udps(&imu->dev, g_raw[2]);

  out->t_centi_c = lsm6dsl_temp_raw_to_centi_c(t_raw);

  imu->last_status = (int32_t)LSM6DSL_OK;
  return true;
}
