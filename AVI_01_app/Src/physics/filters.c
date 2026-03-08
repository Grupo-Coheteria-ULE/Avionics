/**
 * @file filters.c
 * @brief Digital filter helper implementation.
 *
 * This module implements a compact first-order IIR filter using Q15 arithmetic
 * to avoid floating-point cost in the main signal path.
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE (Grupo de Cohetería ULE)
 * Version      : v1.0
 * ---------------------------------------------------------------------------
 */

#include "physics/filters.h"

int32_t iir1_q15(int32_t y_prev, int32_t x, uint16_t alpha_q15)
{
  /* Filter the difference between current input and previous output. */
  int32_t err = x - y_prev;
  int32_t delta = (int32_t)(((int64_t)err * (int64_t)alpha_q15) >> 15);
  return y_prev + delta;
}
