#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "app_cfg.h"
#include "osal.h"
#include "lsm6dsl.h"

typedef struct {
  int32_t ax_ug;
  int32_t ay_ug;
  int32_t az_ug;

  int32_t gx_udps;
  int32_t gy_udps;
  int32_t gz_udps;

  int32_t t_centi_c;
} imu_sample_t;

typedef struct {
  lsm6dsl_t dev;
  bool ready;
  int32_t last_status;
} imu_t;

bool imu_init(imu_t *imu, osal_i2c_t *i2c, uint16_t addr8);
bool imu_update(imu_t *imu, imu_sample_t *out);
