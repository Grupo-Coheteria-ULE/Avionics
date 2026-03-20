/**
 * @file osal.h
 * @brief Operating System Abstraction Layer (OSAL) public interface.
 *
 * This module provides a thin abstraction layer over the STM32 HAL services
 * used by the avionics software. Its main objective is to reduce direct
 * dependencies between high-level modules and the HAL, improving readability,
 * portability, and maintainability.
 *
 * The OSAL module currently provides:
 *  - Basic time utilities
 *  - I2C wrapper functions
 *  - SPI wrapper functions
 *  - UART wrapper functions
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE (Grupo de Cohetería ULE)
 * Author       : Manuel SdA. R.
 * Date         : 2026-03-06
 * Version      : v1.0
 *
 * Notes:
 *  - Designed for STM32CubeIDE projects using STM32 HAL.
 *  - Most operations are blocking unless otherwise stated.
 *  - SPI and UART HAL handles are stored as opaque pointers so that this
 *    module can still compile even if those peripherals are not enabled.
 * ---------------------------------------------------------------------------
 */

#ifndef INC_OSAL_H_
#define INC_OSAL_H_

#pragma once

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Status Codes                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Generic OSAL status codes.
 *
 * These values provide a common error model for the project and hide the
 * original STM32 HAL return values from upper software layers.
 */
typedef enum
{
    OSAL_OK      = 0,   /**< Operation completed successfully */
    OSAL_ERR     = -1,  /**< Generic error */
    OSAL_TIMEOUT = -2,  /**< Timeout occurred during operation */

    OSAL_E_NULL  = -3,  /**< Null pointer or uninitialized handle */
    OSAL_E_PARAM = -4   /**< Invalid parameter */
} osal_status_t;


/* -------------------------------------------------------------------------- */
/* I2C Abstraction                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief I2C wrapper object.
 *
 * Stores the HAL I2C handle and a default timeout used by blocking transfers.
 */
typedef struct
{
    I2C_HandleTypeDef *hi2c; /**< Pointer to STM32 HAL I2C handle */
    uint32_t timeout_ms;     /**< Timeout for blocking operations */
} osal_i2c_t;


/**
 * @brief Converts a 7-bit I2C address to the 8-bit HAL-compatible format.
 *
 * STM32 HAL I2C functions usually expect the slave address left-shifted
 * by one bit. This helper performs that conversion.
 *
 * @param addr7 7-bit slave address
 * @return 8-bit address suitable for HAL I2C APIs
 */
static inline uint16_t osal_i2c_addr8(uint8_t addr7)
{
    return (uint16_t)(addr7 << 1);
}


/* -------------------------------------------------------------------------- */
/* Time Utilities                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Blocks execution for a given number of milliseconds.
 *
 * @param ms Delay duration in milliseconds
 */
void osal_delay_ms(uint32_t ms);

/**
 * @brief Returns the current system tick in milliseconds.
 *
 * @return Millisecond tick count since startup
 */
uint32_t osal_ticks_ms(void);


/* -------------------------------------------------------------------------- */
/* I2C Public API                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initializes an OSAL I2C wrapper.
 *
 * @param bus Pointer to OSAL I2C object
 * @param hi2c Pointer to STM32 HAL I2C handle
 * @param timeout_ms Timeout for blocking transfers in milliseconds
 */
void osal_i2c_init(osal_i2c_t *bus, I2C_HandleTypeDef *hi2c, uint32_t timeout_ms);

/**
 * @brief Writes raw bytes to an I2C device.
 *
 * @param bus Pointer to initialized I2C wrapper
 * @param addr8 8-bit I2C device address
 * @param data Pointer to transmit buffer
 * @param len Number of bytes to transmit
 * @return OSAL status code
 */
osal_status_t osal_i2c_write(osal_i2c_t *bus, uint16_t addr8, const uint8_t *data, uint16_t len);

/**
 * @brief Reads raw bytes from an I2C device.
 *
 * @param bus Pointer to initialized I2C wrapper
 * @param addr8 8-bit I2C device address
 * @param data Pointer to receive buffer
 * @param len Number of bytes to read
 * @return OSAL status code
 */
osal_status_t osal_i2c_read(osal_i2c_t *bus, uint16_t addr8, uint8_t *data, uint16_t len);

/**
 * @brief Reads bytes from a device register through I2C.
 *
 * @param bus Pointer to initialized I2C wrapper
 * @param addr8 8-bit I2C device address
 * @param reg Register address
 * @param data Pointer to receive buffer
 * @param len Number of bytes to read
 * @return OSAL status code
 */
osal_status_t osal_i2c_reg_read(osal_i2c_t *bus, uint16_t addr8, uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief Writes bytes to a device register through I2C.
 *
 * @param bus Pointer to initialized I2C wrapper
 * @param addr8 8-bit I2C device address
 * @param reg Register address
 * @param data Pointer to transmit buffer
 * @param len Number of bytes to write
 * @return OSAL status code
 */
osal_status_t osal_i2c_reg_write(osal_i2c_t *bus, uint16_t addr8, uint8_t reg, const uint8_t *data, uint16_t len);

/**
 * @brief Checks if an I2C device responds on the bus.
 *
 * @param bus Pointer to initialized I2C wrapper
 * @param addr8 8-bit I2C device address
 * @param trials Number of readiness attempts
 * @return OSAL status code
 */
osal_status_t osal_i2c_is_ready(osal_i2c_t *bus, uint16_t addr8, uint32_t trials);


/* -------------------------------------------------------------------------- */
/* SPI Abstraction                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief SPI wrapper object.
 *
 * The SPI handle is stored as an opaque pointer so the project can compile
 * even if SPI support is not enabled in the current build.
 *
 * The chip-select callbacks are supplied by the application.
 */
typedef struct
{
    void *hspi;                          /**< SPI_HandleTypeDef* when enabled */
    uint32_t timeout_ms;                 /**< Timeout for blocking transfers */

    void (*cs_select)(void *user);       /**< Assert chip-select line */
    void (*cs_deselect)(void *user);     /**< Release chip-select line */
    void *cs_user;                       /**< User context for callbacks */
} osal_spi_t;

/**
 * @brief Initializes an OSAL SPI wrapper.
 *
 * @param bus Pointer to OSAL SPI object
 * @param hspi Pointer to STM32 HAL SPI handle (stored as opaque pointer)
 * @param timeout_ms Timeout for blocking transfers in milliseconds
 * @param cs_select Callback used to assert CS
 * @param cs_deselect Callback used to release CS
 * @param cs_user User context passed to CS callbacks
 */
void osal_spi_init(osal_spi_t *bus,
                   void *hspi,
                   uint32_t timeout_ms,
                   void (*cs_select)(void *user),
                   void (*cs_deselect)(void *user),
                   void *cs_user);

/**
 * @brief Performs a full-duplex SPI transfer.
 *
 * @param bus Pointer to initialized SPI wrapper
 * @param tx Pointer to transmit buffer
 * @param rx Pointer to receive buffer
 * @param len Number of bytes to transfer
 * @return OSAL status code
 */
osal_status_t osal_spi_txrx(osal_spi_t *bus,
                            const uint8_t *tx,
                            uint8_t *rx,
                            uint16_t len);

/**
 * @brief Reads one or more registers from an SPI device.
 *
 * This helper assumes a common register-based SPI protocol where:
 *  - bit 7 enables read mode
 *  - bit 6 enables auto-increment for multi-byte transfers
 *
 * @param bus Pointer to initialized SPI wrapper
 * @param reg Register address
 * @param data Pointer to receive buffer
 * @param len Number of bytes to read
 * @return OSAL status code
 */
osal_status_t osal_spi_reg_read(osal_spi_t *bus,
                                uint8_t reg,
                                uint8_t *data,
                                uint16_t len);

/**
 * @brief Writes one or more registers to an SPI device.
 *
 * This helper assumes a common register-based SPI protocol where bit 6
 * enables auto-increment for multi-byte transfers.
 *
 * @param bus Pointer to initialized SPI wrapper
 * @param reg Register address
 * @param data Pointer to transmit buffer
 * @param len Number of bytes to write
 * @return OSAL status code
 */
osal_status_t osal_spi_reg_write(osal_spi_t *bus,
                                 uint8_t reg,
                                 const uint8_t *data,
                                 uint16_t len);


/* -------------------------------------------------------------------------- */
/* UART Abstraction                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief UART wrapper object.
 *
 * The UART handle is stored as an opaque pointer so the code can compile
 * even if UART support is not enabled in the current build.
 */
typedef struct
{
    void *huart;          /**< UART_HandleTypeDef* when enabled */
    uint32_t timeout_ms;  /**< Timeout for blocking transfers */
} osal_uart_t;

/**
 * @brief Initializes an OSAL UART wrapper.
 *
 * @param bus Pointer to OSAL UART object
 * @param huart Pointer to STM32 HAL UART handle (opaque pointer)
 * @param timeout_ms Timeout for blocking operations in milliseconds
 */
void osal_uart_init(osal_uart_t *bus, void *huart, uint32_t timeout_ms);

/**
 * @brief Transmits data through UART using a blocking call.
 *
 * @param bus Pointer to initialized UART wrapper
 * @param data Pointer to transmit buffer
 * @param len Number of bytes to transmit
 * @return OSAL status code
 */
osal_status_t osal_uart_write(osal_uart_t *bus,
                              const uint8_t *data,
                              uint16_t len);

/**
 * @brief Receives data through UART using a blocking call.
 *
 * @param bus Pointer to initialized UART wrapper
 * @param data Pointer to receive buffer
 * @param len Number of bytes to receive
 * @return OSAL status code
 */
osal_status_t osal_uart_read(osal_uart_t *bus,
                             uint8_t *data,
                             uint16_t len);

/**
 * @brief Starts a non-blocking UART transmission using interrupts.
 *
 * The provided buffer must remain valid until the transmission is completed.
 *
 * @param bus Pointer to initialized UART wrapper
 * @param data Pointer to transmit buffer
 * @param len Number of bytes to transmit
 * @return OSAL status code
 */
osal_status_t osal_uart_write_it(osal_uart_t *bus,
                                 const uint8_t *data,
                                 uint16_t len);

/**
 * @brief Starts a non-blocking UART reception using interrupts.
 *
 * The provided buffer must remain valid until the reception is completed.
 *
 * @param bus Pointer to initialized UART wrapper
 * @param data Pointer to receive buffer
 * @param len Number of bytes to receive
 * @return OSAL status code
 */
osal_status_t osal_uart_read_it(osal_uart_t *bus,
                                uint8_t *data,
                                uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* INC_OSAL_H_ */