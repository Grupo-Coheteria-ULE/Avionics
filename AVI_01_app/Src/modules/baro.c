/**
 * @file baro.c
 * @brief Barometer processing module implementation.
 *
 * This module uses the MS5837 pressure sensor as a source of atmospheric data,
 * then derives relative altitude and vertical speed with a light filtering
 * layer adapted to the application.
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE (Grupo de Cohetería ULE)
 * Version      : v1.0
 * ---------------------------------------------------------------------------
 */

#include "modules/baro.h"
#include "physics/filters.h"
#include "physics/atmos.h"
#include <string.h>

static int32_t clamp_dt_ms(uint32_t dt_ms)
{
  if (dt_ms < 1u) return 1;
  if (dt_ms > 200u) return 200;
  return (int32_t)dt_ms;
}

bool baro_init(baro_t *b, osal_i2c_t *i2c, uint16_t addr8)
{
  if (!b || !i2c) return false;
  memset(b, 0, sizeof(*b));

  /* Bring up the low-level MS5837 driver using the configured OSR. */
  if (ms5837_init(&b->dev, i2c, addr8, MS5837_OSR_4096) != MS5837_OK)
    return false;

  /* Start the sensor so conversions can be requested afterwards. */
  if (ms5837_begin(&b->dev) != MS5837_OK)
    return false;

  b->ready = true;
  b->baseline_ready = false;
  return true;
}

bool baro_update(baro_t *b, uint32_t now_ms,
                 int32_t *p_centi_mbar, int32_t *t_centi_c,
                 int32_t *alt_mm, int32_t *vel_mm_s,
                 bool *baseline_ready)
{
  if (!b || !b->ready) return false;

  /* Acquire one fresh sensor sample. */
  if (ms5837_read(&b->dev) != MS5837_OK)
    return false;

  int32_t p = b->dev.pressure_centi_mbar;
  int32_t t = b->dev.temperature_centi_c;

  /* Start warm-up timing on the first successful read. */
  if (b->warmup_start_ms == 0) {
    b->warmup_start_ms = now_ms;
    b->p_sum = 0;
    b->p_count = 0;
  }

  /* Average pressure during warm-up to build the altitude reference. */
  if (!b->baseline_ready) {
    b->p_sum += p;
    b->p_count++;

    if ((now_ms - b->warmup_start_ms) >= BARO_BASELINE_WARMUP_MS && b->p_count > 0) {
      b->p0_centi_mbar = (int32_t)(b->p_sum / (int64_t)b->p_count);
      b->baseline_ready = true;

      /* Reset derivative state to avoid start-up spikes. */
      b->alt_mm_f = 0;
      b->vel_mm_s_f = 0;
      b->alt_mm_prev = 0;
      b->t_prev_ms = now_ms;
    }
  }

  if (p_centi_mbar) *p_centi_mbar = p;
  if (t_centi_c)    *t_centi_c = t;
  if (baseline_ready) *baseline_ready = b->baseline_ready;

  /* Sensor is healthy even while the baseline is still warming up. */
  if (!b->baseline_ready) {
    if (alt_mm) *alt_mm = 0;
    if (vel_mm_s) *vel_mm_s = 0;
    return true;
  }

  /* Convert pressure into relative altitude and smooth the result. */
  int32_t alt_raw = atmos_alt_mm_from_p_centi_mbar(p, b->p0_centi_mbar);
  b->alt_mm_f = iir1_q15(b->alt_mm_f, alt_raw, APOGEE_FILTER_ALPHA_Q15);

  /* Derive vertical speed from filtered altitude and smooth it as well. */
  if (b->t_prev_ms == 0) b->t_prev_ms = now_ms;
  uint32_t dt_u = now_ms - b->t_prev_ms;
  int32_t dt = clamp_dt_ms(dt_u);

  int32_t vel_raw = (int32_t)(((int64_t)(b->alt_mm_f - b->alt_mm_prev) * 1000) / dt);
  b->vel_mm_s_f = iir1_q15(b->vel_mm_s_f, vel_raw, APOGEE_FILTER_ALPHA_Q15);

  b->alt_mm_prev = b->alt_mm_f;
  b->t_prev_ms = now_ms;

  if (alt_mm)   *alt_mm = b->alt_mm_f;
  if (vel_mm_s) *vel_mm_s = b->vel_mm_s_f;

  return true;
}
