/**
 * @file atmos.h
 * @brief Atmospheric helper functions.
 *
 * This module currently provides a simple International Standard Atmosphere
 * approximation to convert pressure into relative altitude.
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE (Grupo de Cohetería ULE)
 * Version      : v1.0
 * ---------------------------------------------------------------------------
 */

#pragma once
#include <stdint.h>

int32_t atmos_alt_mm_from_p_centi_mbar(int32_t p_centi_mbar, int32_t p0_centi_mbar);
