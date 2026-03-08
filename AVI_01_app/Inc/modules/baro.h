/**
 * @file baro.h
 * @brief Barometer processing module interface.
 *
 * The barometer module wraps the MS5837 driver and adds application-specific
 * processing:
 *  - warm-up and baseline pressure estimation
 *  - relative altitude computation
 *  - filtered vertical velocity estimation
 *
 * Output units are aligned with the rest of the app:
 *  - pressure    : centi-mbar
 *  - temperature : centi-degree Celsius
 *  - altitude    : millimetres
 *  - velocity    : millimetres per second
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE (Grupo de Cohetería ULE)
 * Version      : v1.0
 * ---------------------------------------------------------------------------
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "app_cfg.h"
#include "osal.h"
#include "ms5837.h"

typedef struct {
  ms5837_t dev;
  bool ready;

  /* Baseline averaging state. */
  uint32_t warmup_start_ms;
  int64_t  p_sum;
  uint32_t p_count;
  int32_t  p0_centi_mbar;
  bool     baseline_ready;

  /* Filtered outputs and derivative state. */
  int32_t alt_mm_f;
  int32_t vel_mm_s_f;

  int32_t alt_mm_prev;
  uint32_t t_prev_ms;
} baro_t;

bool baro_init(baro_t *b, osal_i2c_t *i2c, uint16_t addr8);

bool baro_update(baro_t *b, uint32_t now_ms,
                 int32_t *p_centi_mbar, int32_t *t_centi_c,
                 int32_t *alt_mm, int32_t *vel_mm_s,
                 bool *baseline_ready);
