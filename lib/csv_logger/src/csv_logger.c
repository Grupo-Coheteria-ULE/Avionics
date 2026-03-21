/**
 * @file csv_logger.c
 * @brief CSV formatting and serial debug implementation.
 *
 * Supports two debug backends:
 *   - Hardware UART (default, USART2 on PA2/PA3)
 *   - USB CDC Virtual COM Port (when CONFIG_CDC is defined)
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE
 * ---------------------------------------------------------------------------
 */

#include "csv_logger.h"
#include "config.h"
#include "osal.h"

#include <stdio.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * Serial debug support
 * -------------------------------------------------------------------------- */

#ifdef CONFIG_SERIAL_DEBUG

#ifndef CONFIG_CDC

/* --- Hardware UART backend (USART2) --- */

extern UART_HandleTypeDef huart2;

static osal_uart_t s_uart_debug;
static bool s_uart_ready = false;

void csv_debug_init(void)
{
    osal_delay_ms(100);
    osal_uart_init(&s_uart_debug, &huart2, 100);
    s_uart_ready = true;
}

static void debug_send(const uint8_t *data, uint16_t len)
{
    if (s_uart_ready)
    {
        osal_uart_write(&s_uart_debug, data, len);
    }
}

#else /* CONFIG_CDC */

/* --- USB CDC backend --- */
/* Requires USB Device middleware (usbd_cdc_if.c) to be included in the project.
 * The HAL_PCD and USBD handles must be initialized before calling csv_debug_init().
 *
 * Expected external function:
 *   extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
 */

extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

static bool s_usb_ready = false;

void csv_debug_init(void)
{
    osal_delay_ms(100);
    s_usb_ready = true;
}

static void debug_send(const uint8_t *data, uint16_t len)
{
    if (s_usb_ready)
    {
        CDC_Transmit_FS((uint8_t *)data, len);
    }
}

#endif /* CONFIG_CDC */

#else /* !CONFIG_SERIAL_DEBUG */

void csv_debug_init(void) {}

static void debug_send(const uint8_t *data, uint16_t len)
{
    (void)data;
    (void)len;
}

#endif /* CONFIG_SERIAL_DEBUG */

/* --------------------------------------------------------------------------
 * CSV formatting
 * -------------------------------------------------------------------------- */

int csv_format_line(const csv_sample_t *sample, char *buf)
{
    if ((sample == NULL) || (buf == NULL))
        return 0;

    /*
     * Format: t;ax;ay;az;wx;wy;wz;T;p
     * Use uint32_t for timestamp (newlib-nano doesn't support %llu).
     */
    int n = snprintf(buf, CSV_LINE_MAX_LEN,
        "%lu"   "%c"   /* t */
        "%.4f"  "%c"   /* ax */
        "%.4f"  "%c"   /* ay */
        "%.4f"  "%c"   /* az */
        "%.4f"  "%c"   /* wx */
        "%.4f"  "%c"   /* wy */
        "%.4f"  "%c"   /* wz */
        "%.2f"  "%c"   /* T */
        "%.2f"  "\r\n", /* p */
        (unsigned long)sample->timestamp_ms,
        CONFIG_CSV_SEP,
        (double)sample->imu.ax, CONFIG_CSV_SEP,
        (double)sample->imu.ay, CONFIG_CSV_SEP,
        (double)sample->imu.az, CONFIG_CSV_SEP,
        (double)sample->imu.wx, CONFIG_CSV_SEP,
        (double)sample->imu.wy, CONFIG_CSV_SEP,
        (double)sample->imu.wz, CONFIG_CSV_SEP,
        (double)sample->temperature, CONFIG_CSV_SEP,
        (double)sample->pressure);

    return (n > 0) ? n : 0;
}

void csv_debug_print(const csv_sample_t *sample)
{
#ifdef CONFIG_SERIAL_DEBUG
    char line[CSV_LINE_MAX_LEN];
    int len = csv_format_line(sample, line);

    if (len > 0)
    {
        debug_send((const uint8_t *)line, (uint16_t)len);
    }
#else
    (void)sample;
#endif
}
