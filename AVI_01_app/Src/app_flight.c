#include "app.h"
#include "app_cfg.h"
#include "app_tasks.h"

#include "modules/baro.h"
#include "modules/imu.h"
#include "modules/apogee.h"
#include "modules/log.h"

#include "osal.h"
#include "main.h"

#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

__weak void bsp_sd_cs_select(void *user)   { (void)user; }
__weak void bsp_sd_cs_deselect(void *user) { (void)user; }

typedef struct {
  app_event_t q[APP_EVENT_Q_LEN];
  uint8_t head, tail, count;
} eventq_t;

static void eventq_init(eventq_t *q) { memset(q, 0, sizeof(*q)); }
static bool eventq_push(eventq_t *q, const app_event_t *e)
{
  if (q->count >= APP_EVENT_Q_LEN) return false;
  q->q[q->tail] = *e;
  q->tail = (uint8_t)((q->tail + 1u) % APP_EVENT_Q_LEN);
  q->count++;
  return true;
}
static bool eventq_pop(eventq_t *q, app_event_t *out)
{
  if (q->count == 0) return false;
  *out = q->q[q->head];
  q->head = (uint8_t)((q->head + 1u) % APP_EVENT_Q_LEN);
  q->count--;
  return true;
}

typedef struct {
  telemetry_t tel;

  osal_i2c_t i2c;
  osal_spi_t spi;

#if APP_USE_BARO
  baro_t baro;
#endif
#if APP_USE_IMU
  imu_t imu;
#endif
#if APP_USE_APOGEE
  apogee_t apogee;
#endif
#if APP_USE_SDLOG
  logsys_t logsys;
#endif

  eventq_t evq;

  task_t tasks[6];
  uint32_t n_tasks;

  bool prev_baro_ok;
  bool prev_imu_ok;
  bool prev_sd_ok;
} app_flight_t;

static app_flight_t g;

static void task_baro(void *ctx);
static void task_imu(void *ctx);
static void task_apogee(void *ctx);
static void task_log(void *ctx);
static void task_flush(void *ctx);
static void task_health(void *ctx);

void app_init(void)
{
  memset(&g, 0, sizeof(g));
  eventq_init(&g.evq);
  g.tel.state = (uint16_t)APP_STATE_IDLE;

  osal_i2c_init(&g.i2c, &hi2c1, 50);
  osal_spi_init(&g.spi, &hspi1, 200, bsp_sd_cs_select, bsp_sd_cs_deselect, NULL);

#if APP_USE_BARO
  if (baro_init(&g.baro, &g.i2c, MS5837_I2C_ADDR)) {
    g.tel.flags |= FLAG_BARO_OK;
    g.prev_baro_ok = true;
  }
#endif

#if APP_USE_IMU
  if (imu_init(&g.imu, &g.i2c, APP_IMU_ADDR8)) {
    g.tel.flags |= FLAG_IMU_OK;
    g.prev_imu_ok = true;
  } else {
    app_event_t ev = { .id = EVT_IMU_FAIL, .t_ms = osal_ticks_ms(), .v = g.imu.last_status };
    (void)eventq_push(&g.evq, &ev);
  }
#endif

#if APP_USE_SDLOG
  if (logsys_init(&g.logsys, &g.spi)) {
    g.tel.flags |= FLAG_SD_OK;
    g.tel.flags |= FLAG_LOG_OK;
    g.prev_sd_ok = true;
  } else {
    app_event_t ev = { .id = EVT_SD_FAIL, .t_ms = osal_ticks_ms(), .v = 0 };
    (void)eventq_push(&g.evq, &ev);
  }
#endif

#if APP_USE_APOGEE
  apogee_init(&g.apogee);
#endif

  g.n_tasks = 0;
#if APP_USE_BARO
  g.tasks[g.n_tasks++] = (task_t){ .period_ms = 1000u/APP_BARO_HZ, .fn = task_baro, .ctx = &g };
#endif
#if APP_USE_IMU
  g.tasks[g.n_tasks++] = (task_t){ .period_ms = 1000u/APP_IMU_HZ, .fn = task_imu, .ctx = &g };
#endif
#if APP_USE_APOGEE
  g.tasks[g.n_tasks++] = (task_t){ .period_ms = 1000u/APP_APOGEE_HZ, .fn = task_apogee, .ctx = &g };
#endif
#if APP_USE_SDLOG
  g.tasks[g.n_tasks++] = (task_t){ .period_ms = 1000u/APP_LOG_HZ, .fn = task_log, .ctx = &g };
  g.tasks[g.n_tasks++] = (task_t){ .period_ms = 1000u/APP_FLUSH_HZ, .fn = task_flush, .ctx = &g };
#endif
  g.tasks[g.n_tasks++] = (task_t){ .period_ms = 1000u/APP_HEALTH_HZ, .fn = task_health, .ctx = &g };

  tasks_init(g.tasks, g.n_tasks, osal_ticks_ms());
}

void app_loop(void)
{
  uint32_t now = osal_ticks_ms();
  g.tel.t_ms = now;
  tasks_run_due(g.tasks, g.n_tasks, now);
}

const telemetry_t* app_get_telemetry(void) { return &g.tel; }

static void task_baro(void *ctx)
{
  app_flight_t *a = (app_flight_t*)ctx;
  int32_t p=0,t=0,alt=0,vel=0;
  bool baseline_ready=false;

  if (baro_update(&a->baro, a->tel.t_ms, &p, &t, &alt, &vel, &baseline_ready)) {
    a->tel.flags |= FLAG_BARO_OK;
    a->tel.p_centi_mbar = p;
    a->tel.t_centi_c = t;
    a->tel.alt_mm = alt;
    a->tel.vel_mm_s = vel;
  } else {
    a->tel.flags &= ~FLAG_BARO_OK;
  }
}

static void task_imu(void *ctx)
{
  app_flight_t *a = (app_flight_t*)ctx;
  imu_sample_t s;

  if (imu_update(&a->imu, &s)) {
    a->tel.flags |= FLAG_IMU_OK;
    a->tel.ax_ug = s.ax_ug;
    a->tel.ay_ug = s.ay_ug;
    a->tel.az_ug = s.az_ug;
    a->tel.gx_udps = s.gx_udps;
    a->tel.gy_udps = s.gy_udps;
    a->tel.gz_udps = s.gz_udps;
    a->tel.imu_t_centi_c = s.t_centi_c;
  } else {
    a->tel.flags &= ~FLAG_IMU_OK;
  }
}

static void task_apogee(void *ctx)
{
  app_flight_t *a = (app_flight_t*)ctx;
  static uint32_t last_ms = 0;

  uint32_t now = a->tel.t_ms;
  uint32_t dt = (last_ms == 0) ? (1000u/APP_APOGEE_HZ) : (now - last_ms);
  last_ms = now;

  bool ap_event = false;
  app_state_t st;

  (void)apogee_update(&a->apogee, dt, a->tel.alt_mm, a->tel.vel_mm_s, &st, &ap_event);
  a->tel.state = (uint16_t)st;
  if (st == APP_STATE_APOGEE) a->tel.flags |= FLAG_APOGEE;

  if (ap_event) {
    app_event_t ev = { .id = EVT_APOGEE_DETECTED, .t_ms = now, .v = a->tel.alt_mm };
    (void)eventq_push(&a->evq, &ev);
  }
}

static void task_log(void *ctx)
{
  app_flight_t *a = (app_flight_t*)ctx;

  a->tel.dropped_bytes = logsys_dropped_bytes(&a->logsys);
  if (a->tel.dropped_bytes) a->tel.flags |= FLAG_DROPPED;

  logsys_append_telemetry(&a->logsys, &a->tel);

  app_event_t ev;
  if (eventq_pop(&a->evq, &ev)) logsys_append_event(&a->logsys, &ev);
}

static void task_flush(void *ctx)
{
  app_flight_t *a = (app_flight_t*)ctx;
  logsys_poll(&a->logsys, 1);
}

static void task_health(void *ctx)
{
  app_flight_t *a = (app_flight_t*)ctx;

  bool baro_ok = (a->tel.flags & FLAG_BARO_OK) != 0;
  bool imu_ok = (a->tel.flags & FLAG_IMU_OK) != 0;
  bool sd_ok = a->logsys.sd_ok && a->logsys.log_ok;

  if (sd_ok) a->tel.flags |= (FLAG_SD_OK | FLAG_LOG_OK);
  else a->tel.flags &= ~(FLAG_SD_OK | FLAG_LOG_OK);

  if (a->prev_baro_ok && !baro_ok) {
    app_event_t ev = { .id = EVT_BARO_FAIL, .t_ms = a->tel.t_ms, .v = 0 };
    (void)eventq_push(&a->evq, &ev);
  }
  if (a->prev_imu_ok && !imu_ok) {
    app_event_t ev = { .id = EVT_IMU_FAIL, .t_ms = a->tel.t_ms, .v = a->imu.last_status };
    (void)eventq_push(&a->evq, &ev);
  }
  if (a->prev_sd_ok && !sd_ok) {
    app_event_t ev = { .id = EVT_SD_FAIL, .t_ms = a->tel.t_ms, .v = 0 };
    (void)eventq_push(&a->evq, &ev);
  }

  a->prev_baro_ok = baro_ok;
  a->prev_imu_ok = imu_ok;
  a->prev_sd_ok = sd_ok;
}
