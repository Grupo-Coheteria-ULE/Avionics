/**
 * @file app_tasks.c
 * @brief Cooperative periodic task scheduler implementation.
 *
 * This module provides two helper functions:
 *  - tasks_init(): arms all tasks relative to a common timestamp
 *  - tasks_run_due(): dispatches every expired task once and advances its slot
 *
 * Time comparison uses signed subtraction so the scheduler remains robust
 * across the 32-bit millisecond tick wraparound.
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE (Grupo de Cohetería ULE)
 * Version      : v1.0
 * ---------------------------------------------------------------------------
 */

#include "app_tasks.h"

void tasks_init(task_t *tasks, uint32_t count, uint32_t now_ms)
{
  for (uint32_t i = 0; i < count; i++) {
    /* First release of each task is one period after initialization time. */
    tasks[i].next_ms = now_ms + tasks[i].period_ms;
  }
}

void tasks_run_due(task_t *tasks, uint32_t count, uint32_t now_ms)
{
  for (uint32_t i = 0; i < count; i++) {
    /*
     * Signed subtraction is used to tolerate wraparound of the system tick.
     * If now_ms has reached or passed next_ms, the task is due.
     */
    if ((int32_t)(now_ms - tasks[i].next_ms) >= 0) {
      /* Advance the task schedule before running the callback. */
      tasks[i].next_ms += tasks[i].period_ms;
      tasks[i].fn(tasks[i].ctx);
    }
  }
}
