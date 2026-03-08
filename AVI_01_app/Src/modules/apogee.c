/**
 * @file apogee.c
 * @brief Apogee detection state machine implementation.
 *
 * Detection is deliberately conservative:
 *  - the detector arms only after altitude and ascent speed exceed thresholds
 *  - apogee is declared only after sustained negative vertical velocity
 *  - once detected, the state remains latched
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE (Grupo de Cohetería ULE)
 * Version      : v1.0
 * ---------------------------------------------------------------------------
 */

#include "modules/apogee.h"
#include <string.h>

void apogee_init(apogee_t *a)
{
  memset(a, 0, sizeof(*a));
  a->state = APP_STATE_IDLE;
}

bool apogee_update(apogee_t *a,
                   uint32_t dt_ms,
                   int32_t alt_mm,
                   int32_t vel_mm_s,
                   app_state_t *out_state,
                   bool *out_apogee_event)
{
  if (!a) return false;
  if (out_apogee_event) *out_apogee_event = false;

  switch (a->state) {
    case APP_STATE_IDLE:
      /* Arm only after a meaningful ascent has started. */
      if (alt_mm > APOGEE_MIN_ALT_ARM_MM && vel_mm_s > APOGEE_ASCENT_VEL_ARM_MM_S) {
        a->state = APP_STATE_ASCENT;
        a->hold_ms = 0;
      }
      break;

    case APP_STATE_ASCENT:
      /* Require sustained negative speed to avoid transient false positives. */
      if (vel_mm_s < APOGEE_NEG_VEL_DETECT_MM_S) {
        a->hold_ms += dt_ms;
        if (a->hold_ms >= APOGEE_HOLD_MS) {
          a->state = APP_STATE_APOGEE;
          if (out_apogee_event) *out_apogee_event = true;
        }
      } else {
        a->hold_ms = 0;
      }
      break;

    case APP_STATE_APOGEE:
    default:
      /* Latched terminal state. */
      break;
  }

  if (out_state) *out_state = a->state;
  return true;
}
