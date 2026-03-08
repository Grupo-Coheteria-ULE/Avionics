/**
 * @file filters.h
 * @brief Digital filter helper functions.
 *
 * This header exposes a compact first-order IIR filter implemented in Q15
 * fixed-point arithmetic.
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE (Grupo de Cohetería ULE)
 * Version      : v1.0
 * ---------------------------------------------------------------------------
 */

#pragma once
#include <stdint.h>

int32_t iir1_q15(int32_t y_prev, int32_t x, uint16_t alpha_q15);
