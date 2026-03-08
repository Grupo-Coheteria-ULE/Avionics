/**
 * @file atmos.c
 * @brief Atmospheric helper implementation.
 *
 * The conversion implemented here uses a simple ISA barometric relation to
 * estimate altitude from pressure ratio. It is intended for relative altitude
 * estimation inside the flight application, not for high-accuracy modelling.
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE (Grupo de Cohetería ULE)
 * Version      : v1.0
 * ---------------------------------------------------------------------------
 */

#include "physics/atmos.h"
#include <math.h>

int32_t atmos_alt_mm_from_p_centi_mbar(int32_t p_centi_mbar, int32_t p0_centi_mbar)
{
  /* Reject invalid or uninitialized pressures. */
  if (p_centi_mbar <= 0 || p0_centi_mbar <= 0) return 0;

  /* Convert fixed-point centi-mbar values into floating-point mbar. */
  float p  = p_centi_mbar / 100.0f;
  float p0 = p0_centi_mbar / 100.0f;

  float ratio = p / p0;
  if (ratio <= 0.0f) ratio = 0.000001f;

  /* Standard barometric formula, result expressed in metres. */
  float h_m = 44330.0f * (1.0f - powf(ratio, 0.1903f));

  /* Convert metres into millimetres for the application fixed-point units. */
  return (int32_t)(h_m * 1000.0f);
}
