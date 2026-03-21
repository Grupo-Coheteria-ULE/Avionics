/**
 * @file usbd_cdc_if.c
 * @brief CDC interface implementation for USB Virtual COM Port.
 *
 * Based on STM32Libs/black_pill_cdc working example for STM32F411CE Black Pill.
 * Key fixes: TxState check, line coding handling, transmit complete callback.
 */

#include "usbd_cdc.h"
#include <string.h>

/* --------------------------------------------------------------------------
 * Private variables
 * -------------------------------------------------------------------------- */

extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t UserRxBufferFS[CDC_DATA_FS_MAX_PACKET_SIZE];
static uint8_t UserTxBufferFS[CDC_DATA_FS_MAX_PACKET_SIZE];

/* Buffer for SET/GET_LINE_CODING (Windows needs this for enumeration) */
static uint8_t LineCodingBuf[7] = {0, 0, 0, 0, 0, 0, 0};

/* --------------------------------------------------------------------------
 * CDC callbacks
 * -------------------------------------------------------------------------- */

static int8_t CDC_Init_FS(void)
{
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    return (USBD_OK);
}

static int8_t CDC_DeInit_FS(void)
{
    return (USBD_OK);
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    (void)length;

    switch (cmd)
    {
        case CDC_SEND_ENCAPSULATED_COMMAND:
            break;

        case CDC_GET_ENCAPSULATED_RESPONSE:
            break;

        case CDC_SET_COMM_FEATURE:
            break;

        case CDC_GET_COMM_FEATURE:
            break;

        case CDC_CLEAR_COMM_FEATURE:
            break;

        case CDC_SET_LINE_CODING:
            memcpy(LineCodingBuf, pbuf, 7);
            break;

        case CDC_GET_LINE_CODING:
            memcpy(pbuf, LineCodingBuf, 7);
            break;

        case CDC_SET_CONTROL_LINE_STATE:
            break;

        case CDC_SEND_BREAK:
            break;

        default:
            break;
    }

    return (USBD_OK);
}

static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
    (void)Len;
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return (USBD_OK);
}

static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
    (void)Buf;
    (void)Len;
    (void)epnum;
    return (USBD_OK);
}

/* --------------------------------------------------------------------------
 * Interface operations structure
 * -------------------------------------------------------------------------- */

static USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
    CDC_Init_FS,
    CDC_DeInit_FS,
    CDC_Control_FS,
    CDC_Receive_FS,
    CDC_TransmitCplt_FS
};

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
    uint8_t result = USBD_OK;

    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc == NULL)
        return USBD_BUSY;

    if (hcdc->TxState != 0)
    {
        return USBD_BUSY;
    }

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
    result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);

    return result;
}

uint8_t CDC_Interface_Init(void)
{
    return USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);
}
