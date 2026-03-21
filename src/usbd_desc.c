/**
 * @file usbd_desc.c
 * @brief USB Device + CDC descriptors.
 */

#include "usbd_core.h"
#include "usbd_cdc.h"

#define USBD_VID    0x0483
#define USBD_PID    0x5740

/* Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
{
    0x12, USB_DESC_TYPE_DEVICE,
    0x00, 0x02,                     /* bcdUSB 2.0 */
    0x02, 0x00, 0x00,              /* class, subclass, protocol */
    USB_MAX_EP0_SIZE,
    LOBYTE(USBD_VID), HIBYTE(USBD_VID),
    LOBYTE(USBD_PID), HIBYTE(USBD_PID),
    0x00, 0x02,                     /* bcdDevice */
    USBD_IDX_MFC_STR, USBD_IDX_PRODUCT_STR, USBD_IDX_SERIAL_STR,
    0x01
};

/* Configuration Descriptor (67 bytes) */
__ALIGN_BEGIN uint8_t USBD_CDC_CfgFSDesc[67] __ALIGN_END =
{
    /* Configuration */
    0x09, USB_DESC_TYPE_CONFIGURATION, 0x43, 0x00, 0x02, 0x01, 0x00, 0xC0, 0x32,
    /* Interface 0: CDC Communication */
    0x09, USB_DESC_TYPE_INTERFACE, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
    /* Header */
    0x05, 0x24, 0x00, 0x10, 0x01,
    /* Call Management */
    0x05, 0x24, 0x01, 0x00, 0x01,
    /* ACM */
    0x04, 0x24, 0x02, 0x02,
    /* Union */
    0x05, 0x24, 0x06, 0x00, 0x01,
    /* Notification EP */
    0x07, USB_DESC_TYPE_ENDPOINT, CDC_CMD_EP, 0x03, 0x08, 0x00, 0x10,
    /* Interface 1: CDC Data */
    0x09, USB_DESC_TYPE_INTERFACE, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,
    /* Data OUT EP */
    0x07, USB_DESC_TYPE_ENDPOINT, CDC_OUT_EP, 0x02, 0x40, 0x00, 0x00,
    /* Data IN EP */
    0x07, USB_DESC_TYPE_ENDPOINT, CDC_IN_EP, 0x02, 0x40, 0x00, 0x00
};

/* Language ID */
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
    USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING, 0x09, 0x04
};

/* String descriptor buffer */
static uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ];

/* String descriptor callbacks */
uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_DeviceDesc);
    return USBD_DeviceDesc;
}

uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_LangIDDesc);
    return USBD_LangIDDesc;
}

uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)"GC_ULE", USBD_StrDesc, length);
    return USBD_StrDesc;
}

uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)"AVI Flight Computer", USBD_StrDesc, length);
    return USBD_StrDesc;
}

uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    char s[25];
    snprintf(s, sizeof(s), "%08lX%08lX%08lX",
             (unsigned long)HAL_GetUIDw0(),
             (unsigned long)HAL_GetUIDw1(),
             (unsigned long)HAL_GetUIDw2());
    USBD_GetString((uint8_t *)s, USBD_StrDesc, length);
    return USBD_StrDesc;
}

uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)"CDC Config", USBD_StrDesc, length);
    return USBD_StrDesc;
}

uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)"CDC Interface", USBD_StrDesc, length);
    return USBD_StrDesc;
}

USBD_DescriptorsTypeDef FS_Desc =
{
    USBD_FS_DeviceDescriptor,
    USBD_FS_LangIDStrDescriptor,
    USBD_FS_ManufacturerStrDescriptor,
    USBD_FS_ProductStrDescriptor,
    USBD_FS_SerialStrDescriptor,
    USBD_FS_ConfigStrDescriptor,
    USBD_FS_InterfaceStrDescriptor
};
