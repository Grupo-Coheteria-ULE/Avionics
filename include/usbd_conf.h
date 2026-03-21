/**
 * @file usbd_conf.h
 * @brief USB Device configuration for STM32F411.
 */

#ifndef __USBD_CONF_H
#define __USBD_CONF_H

#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <string.h>

#define USBD_MAX_NUM_INTERFACES     1U
#define USBD_MAX_NUM_CONFIGURATION  1U
#define USBD_MAX_STR_DESC_SIZ       512U
#define USBD_SUPPORT_USER_STRING    0U
#define USBD_DEBUG_LEVEL            0U
#define USBD_LPM_ENABLED            0U
#define USBD_SELF_POWERED           1U

#define USBD_malloc                 malloc
#define USBD_free                   free
#define USBD_memset                 memset
#define USBD_memcpy                 memcpy

#endif /* __USBD_CONF_H */
