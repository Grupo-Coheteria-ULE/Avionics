/**
 * @file log.h
 * @brief Logging system wrapper interface.
 *
 * This module connects the application data model with the raw SD logging
 * backend. It defines the serialized frame formats used for telemetry and
 * events, and exposes a compact API used by the application tasks.
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

#include "osal.h"
#include "sd_spi.h"
#include "sdlog_raw.h"

typedef enum {
  LOG_FRAME_TLM   = 1,
  LOG_FRAME_EVENT = 2
} log_frame_type_t;

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  ver;
  uint16_t size;
} log_hdr_t;

typedef struct __attribute__((packed)) {
  log_hdr_t h;
  telemetry_t tel;
} log_tlm_frame_t;

typedef struct __attribute__((packed)) {
  log_hdr_t h;
  app_event_t ev;
} log_event_frame_t;

typedef struct {
  sd_spi_t sd;
  sdlog_t  log;
  uint8_t  buf[LOG_NUM_BUFFERS * LOG_PAGE_BYTES];

  bool sd_ok;
  bool log_ok;
} logsys_t;

bool logsys_init(logsys_t *l, osal_spi_t *spi_bus);
void logsys_append_telemetry(logsys_t *l, const telemetry_t *tel);
void logsys_append_event(logsys_t *l, const app_event_t *ev);
void logsys_poll(logsys_t *l, uint32_t max_pages);
uint32_t logsys_dropped_bytes(const logsys_t *l);
