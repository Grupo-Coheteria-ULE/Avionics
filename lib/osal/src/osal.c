/**
 * @file osal.c
 * @brief Implementation of the Operating System Abstraction Layer (OSAL).
 *
 * This module wraps a subset of STM32 HAL services into a simplified API
 * used by the rest of the avionics software.
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE (Grupo de Cohetería ULE)
 * Author       : Manuel SdA. R.
 * Date         : 2026-03-06
 * Version      : v1.0
 * ---------------------------------------------------------------------------
 */

#include "osal.h"


/* -------------------------------------------------------------------------- */
/* Private Constants                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief SPI read bit used by many register-based devices.
 */
#define OSAL_SPI_READ    0x80u

/**
 * @brief SPI auto-increment bit used by many register-based devices.
 */
#define OSAL_SPI_AUTOINC 0x40u


/* -------------------------------------------------------------------------- */
/* Private Helper Functions                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Converts an STM32 HAL status code into an OSAL status code.
 *
 * This keeps higher-level modules independent from HAL-specific return values.
 *
 * @param st HAL status value returned by a HAL function
 * @return Equivalent OSAL status code
 */
static osal_status_t osal_from_hal(HAL_StatusTypeDef st)
{
    if (st == HAL_OK)
    {
        return OSAL_OK;
    }

    if (st == HAL_TIMEOUT)
    {
        return OSAL_TIMEOUT;
    }

    return OSAL_ERR;
}


/* -------------------------------------------------------------------------- */
/* Time Utilities                                                             */
/* -------------------------------------------------------------------------- */

void osal_delay_ms(uint32_t ms)
{
    /* Delegate the blocking delay directly to the HAL. */
    HAL_Delay(ms);
}

uint32_t osal_ticks_ms(void)
{
    /* Return the millisecond system tick maintained by the HAL. */
    return HAL_GetTick();
}


/* -------------------------------------------------------------------------- */
/* I2C Functions                                                              */
/* -------------------------------------------------------------------------- */

void osal_i2c_init(osal_i2c_t *bus, I2C_HandleTypeDef *hi2c, uint32_t timeout_ms)
{
    /* Validate pointers before touching the wrapper object. */
    if ((bus == NULL) || (hi2c == NULL))
    {
        return;
    }

    /* Store the HAL handle and the default timeout for later transactions. */
    bus->hi2c = hi2c;
    bus->timeout_ms = timeout_ms;
}

osal_status_t osal_i2c_write(osal_i2c_t *bus, uint16_t addr8, const uint8_t *data, uint16_t len)
{
    /* The wrapper, HAL handle, and user data buffer must all be valid. */
    if ((bus == NULL) || (bus->hi2c == NULL) || (data == NULL))
    {
        return OSAL_E_NULL;
    }

    /* A transfer with zero length is considered an invalid parameter. */
    if (len == 0u)
    {
        return OSAL_E_PARAM;
    }

    /* Send raw bytes to the target I2C device using a blocking HAL call. */
    return osal_from_hal(
        HAL_I2C_Master_Transmit(bus->hi2c, addr8, (uint8_t *)data, len, bus->timeout_ms)
    );
}

osal_status_t osal_i2c_read(osal_i2c_t *bus, uint16_t addr8, uint8_t *data, uint16_t len)
{
    /* Validate wrapper object, HAL handle, and destination buffer. */
    if ((bus == NULL) || (bus->hi2c == NULL) || (data == NULL))
    {
        return OSAL_E_NULL;
    }

    /* Reject empty transfers because they are not meaningful. */
    if (len == 0u)
    {
        return OSAL_E_PARAM;
    }

    /* Read raw bytes from the target I2C device. */
    return osal_from_hal(
        HAL_I2C_Master_Receive(bus->hi2c, addr8, data, len, bus->timeout_ms)
    );
}

osal_status_t osal_i2c_reg_read(osal_i2c_t *bus, uint16_t addr8, uint8_t reg, uint8_t *data, uint16_t len)
{
    /* Validate wrapper object, HAL handle, and destination buffer. */
    if ((bus == NULL) || (bus->hi2c == NULL) || (data == NULL))
    {
        return OSAL_E_NULL;
    }

    /* Reading zero bytes from a register is an invalid request. */
    if (len == 0u)
    {
        return OSAL_E_PARAM;
    }

    /* Perform an 8-bit register-addressed memory read through I2C. */
    return osal_from_hal(
        HAL_I2C_Mem_Read(bus->hi2c, addr8, reg, I2C_MEMADD_SIZE_8BIT, data, len, bus->timeout_ms)
    );
}

osal_status_t osal_i2c_reg_write(osal_i2c_t *bus, uint16_t addr8, uint8_t reg, const uint8_t *data, uint16_t len)
{
    /* Validate wrapper object, HAL handle, and source buffer. */
    if ((bus == NULL) || (bus->hi2c == NULL) || (data == NULL))
    {
        return OSAL_E_NULL;
    }

    /* Writing zero bytes to a register is invalid. */
    if (len == 0u)
    {
        return OSAL_E_PARAM;
    }

    /* Perform an 8-bit register-addressed memory write through I2C. */
    return osal_from_hal(
        HAL_I2C_Mem_Write(bus->hi2c, addr8, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len, bus->timeout_ms)
    );
}

osal_status_t osal_i2c_is_ready(osal_i2c_t *bus, uint16_t addr8, uint32_t trials)
{
    /* The wrapper and HAL handle must be valid. */
    if ((bus == NULL) || (bus->hi2c == NULL))
    {
        return OSAL_E_NULL;
    }

    /* It makes no sense to test readiness zero times. */
    if (trials == 0u)
    {
        return OSAL_E_PARAM;
    }

    /* Ask the HAL to poll the device and check whether it acknowledges. */
    return osal_from_hal(
        HAL_I2C_IsDeviceReady(bus->hi2c, addr8, trials, bus->timeout_ms)
    );
}


/* -------------------------------------------------------------------------- */
/* SPI Functions                                                              */
/* -------------------------------------------------------------------------- */

void osal_spi_init(osal_spi_t *bus,
                   void *hspi,
                   uint32_t timeout_ms,
                   void (*cs_select)(void *user),
                   void (*cs_deselect)(void *user),
                   void *cs_user)
{
    /* Only the wrapper object itself is mandatory at init time. */
    if (bus == NULL)
    {
        return;
    }

    /* Store all SPI-related resources inside the abstraction object. */
    bus->hspi = hspi;
    bus->timeout_ms = timeout_ms;
    bus->cs_select = cs_select;
    bus->cs_deselect = cs_deselect;
    bus->cs_user = cs_user;
}

#if defined(HAL_SPI_MODULE_ENABLED)

osal_status_t osal_spi_txrx(osal_spi_t *bus, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    /* Validate wrapper object, HAL handle, and both data buffers. */
    if ((bus == NULL) || (bus->hspi == NULL) || (tx == NULL) || (rx == NULL))
    {
        return OSAL_E_NULL;
    }

    /* Full-duplex transfer must move at least one byte. */
    if (len == 0u)
    {
        return OSAL_E_PARAM;
    }

    /* Recover the real HAL SPI handle from the opaque pointer. */
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)bus->hspi;

    /* Perform a blocking transmit/receive transaction. */
    return osal_from_hal(
        HAL_SPI_TransmitReceive(hspi, (uint8_t *)tx, rx, len, bus->timeout_ms)
    );
}

osal_status_t osal_spi_reg_read(osal_spi_t *bus, uint8_t reg, uint8_t *data, uint16_t len)
{
    /* Validate everything required for a register-based SPI read. */
    if ((bus == NULL) || (bus->hspi == NULL) ||
        (bus->cs_select == NULL) || (bus->cs_deselect == NULL) ||
        (data == NULL))
    {
        return OSAL_E_NULL;
    }

    /* At least one byte must be requested. */
    if (len == 0u)
    {
        return OSAL_E_PARAM;
    }

    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)bus->hspi;

    /*
     * Build the command byte:
     * - set the read bit
     * - set auto-increment if more than one byte will be read
     */
    uint8_t cmd = (uint8_t)(reg | OSAL_SPI_READ | ((len > 1u) ? OSAL_SPI_AUTOINC : 0u));

    /* Assert chip select before starting the SPI frame. */
    bus->cs_select(bus->cs_user);

    /* First send the register command. */
    HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, &cmd, 1u, bus->timeout_ms);

    /* If the command phase succeeded, receive the requested payload bytes. */
    if (st == HAL_OK)
    {
        st = HAL_SPI_Receive(hspi, data, len, bus->timeout_ms);
    }

    /* Always release chip select after the transaction. */
    bus->cs_deselect(bus->cs_user);

    /* Convert the HAL result into the project-wide OSAL status. */
    return osal_from_hal(st);
}

osal_status_t osal_spi_reg_write(osal_spi_t *bus, uint8_t reg, const uint8_t *data, uint16_t len)
{
    /* Validate everything required for a register-based SPI write. */
    if ((bus == NULL) || (bus->hspi == NULL) ||
        (bus->cs_select == NULL) || (bus->cs_deselect == NULL) ||
        (data == NULL))
    {
        return OSAL_E_NULL;
    }

    /* At least one byte must be written. */
    if (len == 0u)
    {
        return OSAL_E_PARAM;
    }

    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)bus->hspi;

    /*
     * Build the command byte:
     * - write mode is implicit
     * - enable auto-increment if more than one byte will be written
     */
    uint8_t cmd = (uint8_t)(reg | ((len > 1u) ? OSAL_SPI_AUTOINC : 0u));

    /* Assert chip select before sending the command and data. */
    bus->cs_select(bus->cs_user);

    /* Send the register address/command first. */
    HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, &cmd, 1u, bus->timeout_ms);

    /* If the command phase succeeded, send the data payload. */
    if (st == HAL_OK)
    {
        st = HAL_SPI_Transmit(hspi, (uint8_t *)data, len, bus->timeout_ms);
    }

    /* Release chip select to finish the SPI frame. */
    bus->cs_deselect(bus->cs_user);

    /* Return the final translated status code. */
    return osal_from_hal(st);
}

#else

osal_status_t osal_spi_txrx(osal_spi_t *bus, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    (void)bus;
    (void)tx;
    (void)rx;
    (void)len;

    /* SPI support is not enabled in this build configuration. */
    return OSAL_E_PARAM;
}

osal_status_t osal_spi_reg_read(osal_spi_t *bus, uint8_t reg, uint8_t *data, uint16_t len)
{
    (void)bus;
    (void)reg;
    (void)data;
    (void)len;

    /* SPI support is not enabled in this build configuration. */
    return OSAL_E_PARAM;
}

osal_status_t osal_spi_reg_write(osal_spi_t *bus, uint8_t reg, const uint8_t *data, uint16_t len)
{
    (void)bus;
    (void)reg;
    (void)data;
    (void)len;

    /* SPI support is not enabled in this build configuration. */
    return OSAL_E_PARAM;
}

#endif


/* -------------------------------------------------------------------------- */
/* UART Functions                                                             */
/* -------------------------------------------------------------------------- */

void osal_uart_init(osal_uart_t *bus, void *huart, uint32_t timeout_ms)
{
    /* Validate wrapper pointer before writing into it. */
    if (bus == NULL)
    {
        return;
    }

    /* Store UART HAL handle and default timeout. */
    bus->huart = huart;
    bus->timeout_ms = timeout_ms;
}

#if defined(HAL_UART_MODULE_ENABLED)

osal_status_t osal_uart_write(osal_uart_t *bus, const uint8_t *data, uint16_t len)
{
    /* Validate wrapper object, HAL handle, and source buffer. */
    if ((bus == NULL) || (bus->huart == NULL) || (data == NULL))
    {
        return OSAL_E_NULL;
    }

    /* Zero-length transmissions are invalid. */
    if (len == 0u)
    {
        return OSAL_E_PARAM;
    }

    /* Recover the real UART HAL handle and perform a blocking transmit. */
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)bus->huart;

    return osal_from_hal(
        HAL_UART_Transmit(huart, (uint8_t *)data, len, bus->timeout_ms)
    );
}

osal_status_t osal_uart_read(osal_uart_t *bus, uint8_t *data, uint16_t len)
{
    /* Validate wrapper object, HAL handle, and destination buffer. */
    if ((bus == NULL) || (bus->huart == NULL) || (data == NULL))
    {
        return OSAL_E_NULL;
    }

    /* Zero-length receptions are invalid. */
    if (len == 0u)
    {
        return OSAL_E_PARAM;
    }

    /* Recover the real UART HAL handle and perform a blocking receive. */
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)bus->huart;

    return osal_from_hal(
        HAL_UART_Receive(huart, data, len, bus->timeout_ms)
    );
}

osal_status_t osal_uart_write_it(osal_uart_t *bus, const uint8_t *data, uint16_t len)
{
    /* Validate wrapper object, HAL handle, and source buffer. */
    if ((bus == NULL) || (bus->huart == NULL) || (data == NULL))
    {
        return OSAL_E_NULL;
    }

    /* Interrupt-driven transmission must also send at least one byte. */
    if (len == 0u)
    {
        return OSAL_E_PARAM;
    }

    /*
     * Cast away const because the HAL interrupt API expects uint8_t*.
     * The user must keep the buffer valid until the transfer completes.
     */
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)bus->huart;

    return osal_from_hal(
        HAL_UART_Transmit_IT(huart, (uint8_t *)data, len)
    );
}

osal_status_t osal_uart_read_it(osal_uart_t *bus, uint8_t *data, uint16_t len)
{
    /* Validate wrapper object, HAL handle, and destination buffer. */
    if ((bus == NULL) || (bus->huart == NULL) || (data == NULL))
    {
        return OSAL_E_NULL;
    }

    /* Interrupt-driven reception must also request at least one byte. */
    if (len == 0u)
    {
        return OSAL_E_PARAM;
    }

    /* Start the UART interrupt-driven receive operation. */
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)bus->huart;

    return osal_from_hal(
        HAL_UART_Receive_IT(huart, data, len)
    );
}

#else

osal_status_t osal_uart_write(osal_uart_t *bus, const uint8_t *data, uint16_t len)
{
    (void)bus;
    (void)data;
    (void)len;

    /* UART support is not enabled in this build configuration. */
    return OSAL_E_PARAM;
}

osal_status_t osal_uart_read(osal_uart_t *bus, uint8_t *data, uint16_t len)
{
    (void)bus;
    (void)data;
    (void)len;

    /* UART support is not enabled in this build configuration. */
    return OSAL_E_PARAM;
}

osal_status_t osal_uart_write_it(osal_uart_t *bus, const uint8_t *data, uint16_t len)
{
    (void)bus;
    (void)data;
    (void)len;

    /* UART support is not enabled in this build configuration. */
    return OSAL_E_PARAM;
}

osal_status_t osal_uart_read_it(osal_uart_t *bus, uint8_t *data, uint16_t len)
{
    (void)bus;
    (void)data;
    (void)len;

    /* UART support is not enabled in this build configuration. */
    return OSAL_E_PARAM;
}

#endif