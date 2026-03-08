/**
 * @file log.c
 * @brief Logging system wrapper implementation.
 *
 * The logger writes two application-level frame types into the raw SD logging
 * backend:
 *  - telemetry snapshots
 *  - asynchronous event records
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE (Grupo de Cohetería ULE)
 * Version      : v1.0
 * ---------------------------------------------------------------------------
 */

#include "modules/log.h"
#include <string.h>

bool logsys_init(logsys_t *l, osal_spi_t *spi_bus)
{
  if (!l || !spi_bus) return false;
  memset(l, 0, sizeof(*l));

  /* Bind the SD SPI transport to the shared OSAL SPI bus. */
  sd_spi_attach(&l->sd, spi_bus, 200);

  /* Initialize the card before creating the higher-level logger. */
  if (sd_spi_init(&l->sd) != SD_SPI_OK) {
    l->sd_ok = false;
    l->log_ok = false;
    return false;
  }
  l->sd_ok = true;

  sdlog_cfg_t cfg = {
    .meta_lba = LOG_META_LBA,
    .data_lba = LOG_DATA_LBA,
    .end_lba  = LOG_END_LBA,

    .page_bytes = LOG_PAGE_BYTES,
    .buffers = l->buf,
    .num_buffers = LOG_NUM_BUFFERS,

    .wrap = LOG_WRAP,
    .use_crc = LOG_USE_CRC,
    .use_preerase = LOG_USE_PREERASE,
    .meta_period_pages = LOG_META_PERIOD_PAGES
  };

  if (sdlog_init(&l->log, &l->sd, &cfg) != SDLOG_OK) {
    l->log_ok = false;
    return false;
  }

  l->log_ok = true;
  return true;
}

void logsys_append_telemetry(logsys_t *l, const telemetry_t *tel)
{
  if (!l || !tel || !l->log_ok) return;

  /* Serialize telemetry into a self-describing frame. */
  log_tlm_frame_t fr;
  fr.h.type = LOG_FRAME_TLM;
  fr.h.ver  = (uint8_t)LOG_VERSION;
  fr.h.size = (uint16_t)sizeof(fr.tel);
  fr.tel = *tel;

  (void)sdlog_append(&l->log, &fr, (uint32_t)sizeof(fr), (uint64_t)tel->t_ms);
}

void logsys_append_event(logsys_t *l, const app_event_t *ev)
{
  if (!l || !ev || !l->log_ok) return;

  /* Serialize asynchronous event information into its own frame type. */
  log_event_frame_t fr;
  fr.h.type = LOG_FRAME_EVENT;
  fr.h.ver  = (uint8_t)LOG_VERSION;
  fr.h.size = (uint16_t)sizeof(fr.ev);
  fr.ev = *ev;

  (void)sdlog_append(&l->log, &fr, (uint32_t)sizeof(fr), (uint64_t)ev->t_ms);
}

void logsys_poll(logsys_t *l, uint32_t max_pages)
{
  if (!l || !l->log_ok) return;

  /* Let the raw logger flush a bounded amount of work per scheduler tick. */
  (void)sdlog_poll(&l->log, max_pages, NULL);
}

uint32_t logsys_dropped_bytes(const logsys_t *l)
{
  if (!l) return 0;
  return sdlog_get_dropped_bytes(&l->log);
}
