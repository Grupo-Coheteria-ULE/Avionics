/**
 * @file app_tasks.h
 * @brief Minimal cooperative task scheduler interface.
 *
 * The scheduler manages a fixed array of periodic tasks. Each task stores its
 * period, next activation time, callback, and opaque context pointer.
 *
 * The implementation is intentionally lightweight and suitable for bare-metal
 * loops where tasks are dispatched cooperatively from app_loop().
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE (Grupo de Cohetería ULE)
 * Version      : v1.0
 * ---------------------------------------------------------------------------
 */

#pragma once
#include <stdint.h>

typedef void (*task_fn_t)(void *ctx);

typedef struct {
  uint32_t period_ms; /**< Execution period in milliseconds */
  uint32_t next_ms;   /**< Next scheduled activation timestamp */
  task_fn_t fn;       /**< Task function */
  void *ctx;          /**< User context pointer */
} task_t;

void tasks_init(task_t *tasks, uint32_t count, uint32_t now_ms);
void tasks_run_due(task_t *tasks, uint32_t count, uint32_t now_ms);
