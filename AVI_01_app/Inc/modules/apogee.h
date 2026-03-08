/**
 * @file apogee.h
 * @brief Apogee detection state machine interface.
 *
 * This module implements a compact finite-state machine that analyses filtered
 * altitude and vertical velocity to determine when apogee has been reached.
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
#include "app_types.h"

typedef struct {
  app_state_t state;
  uint32_t hold_ms;
} apogee_t;

void apogee_init(apogee_t *a);

bool apogee_update(apogee_t *a,
                   uint32_t dt_ms,
                   int32_t alt_mm,
                   int32_t vel_mm_s,
                   app_state_t *out_state,
                   bool *out_apogee_event);
