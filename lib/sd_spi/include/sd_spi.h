/**
 * @file sd_spi.h
 * @brief SD card driver for SPI mode.
 *
 * This module provides block-level access to an SD card operating in SPI mode.
 * It supports card initialization, sector reads, sector writes, optional
 * pre-erase hints for multi-block writes, and basic card capacity discovery.
 *
 * The driver is intended to sit above the OSAL SPI abstraction and provide
 * a simple 512-byte logical block interface to upper layers such as raw
 * data loggers or filesystem adapters.
 *
 * Supported features:
 *  - SD card initialization in SPI mode
 *  - SDSC and SDHC card type detection
 *  - Single-block and multi-block read
 *  - Single-block and multi-block write
 *  - Sector count extraction from CSD register
 *  - Optional ACMD23 pre-erase hint before multi-block write
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE (Grupo de Cohetería ULE)
 * Author       : Manuel SdA. R.
 * Date         : 2026-03-06
 * Version      : v1.0
 *
 * Notes:
 *  - The SPI bus must run at low speed during card initialization
 *    (typically 100-400 kHz).
 *  - After successful initialization, the SPI clock can be increased.
 *  - This module exposes a logical sector interface of 512 bytes per sector.
 * ---------------------------------------------------------------------------
 */

#ifndef INC_SD_SPI_H_
#define INC_SD_SPI_H_

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "osal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Public Constants                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Logical sector size used by the driver.
 *
 * SD cards expose a 512-byte logical sector interface once properly
 * initialized in SPI mode.
 */
#define SD_SPI_SECTOR_SIZE 512u


/* -------------------------------------------------------------------------- */
/* Public Types                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief SD SPI driver status codes.
 */
typedef enum
{
    SD_SPI_OK       = 0,   /**< Operation completed successfully */
    SD_SPI_ERR      = -1,  /**< Generic driver error */
    SD_SPI_TIMEOUT  = -2,  /**< Timeout while waiting for card response */
    SD_SPI_PARAM    = -3,  /**< Invalid parameter or invalid state */
    SD_SPI_NOCARD   = -4,  /**< No card detected (reserved / optional use) */
    SD_SPI_CRC_ERR  = -5   /**< CRC error (reserved / optional use) */
} sd_spi_status_t;

/**
 * @brief Supported SD card types.
 */
typedef enum
{
    SD_SPI_CARD_UNKNOWN = 0, /**< Card type not yet identified */
    SD_SPI_CARD_SDSC    = 1, /**< Standard Capacity SD card */
    SD_SPI_CARD_SDHC    = 2  /**< High Capacity SD card */
} sd_spi_card_t;

/**
 * @brief SD card driver object.
 *
 * This structure stores the OSAL SPI bus reference, runtime state, detected
 * card type, discovered logical sector count, and configuration flags.
 */
typedef struct
{
    osal_spi_t *bus;         /**< Pointer to the SPI bus abstraction */
    uint32_t timeout_ms;     /**< Timeout used by command and data operations */

    sd_spi_card_t card_type; /**< Detected card type */
    uint32_t sector_count;   /**< Number of logical 512-byte sectors */
    bool initialized;        /**< True once initialization completed successfully */

    bool use_crc;            /**< Reserved flag for future CRC support */
} sd_spi_t;


/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Attaches the SD driver to an SPI bus and resets runtime state.
 *
 * This function does not communicate with the card. It only initializes the
 * software driver object and stores the SPI bus reference.
 *
 * @param sd Pointer to SD driver object
 * @param bus Pointer to initialized OSAL SPI bus
 * @param timeout_ms Operation timeout in milliseconds; if zero, a default
 *        timeout is applied
 */
void sd_spi_attach(sd_spi_t *sd, osal_spi_t *bus, uint32_t timeout_ms);

/**
 * @brief Initializes the SD card in SPI mode.
 *
 * This routine sends the standard startup sequence used by SPI-mode SD cards:
 * idle clocks, CMD0, CMD8, ACMD41 loop, OCR read, optional block length setup,
 * and capacity detection from the CSD register.
 *
 * @param sd Pointer to SD driver object
 * @return SD driver status code
 */
sd_spi_status_t sd_spi_init(sd_spi_t *sd);

/**
 * @brief Reads one or more 512-byte logical sectors.
 *
 * @param sd Pointer to initialized SD driver
 * @param lba Logical block address of the first sector to read
 * @param buf Destination buffer
 * @param count Number of sectors to read
 * @return SD driver status code
 */
sd_spi_status_t sd_spi_read(sd_spi_t *sd, uint32_t lba, void *buf, uint32_t count);

/**
 * @brief Writes one or more 512-byte logical sectors.
 *
 * @param sd Pointer to initialized SD driver
 * @param lba Logical block address of the first sector to write
 * @param buf Source buffer
 * @param count Number of sectors to write
 * @return SD driver status code
 */
sd_spi_status_t sd_spi_write(sd_spi_t *sd, uint32_t lba, const void *buf, uint32_t count);

/**
 * @brief Sends a pre-erase hint before a multi-block write.
 *
 * Some cards may reduce latency variation if they know in advance how many
 * blocks will be written.
 *
 * @param sd Pointer to initialized SD driver
 * @param nblocks Number of blocks expected in the upcoming multi-block write
 * @return SD driver status code
 */
sd_spi_status_t sd_spi_preerase(sd_spi_t *sd, uint32_t nblocks);

/**
 * @brief Returns the detected logical sector count.
 *
 * @param sd Pointer to SD driver object
 * @return Number of 512-byte logical sectors, or 0 on invalid pointer
 */
uint32_t sd_spi_get_sector_count(const sd_spi_t *sd);

/**
 * @brief Returns the detected card type.
 *
 * @param sd Pointer to SD driver object
 * @return Detected card type, or SD_SPI_CARD_UNKNOWN on invalid pointer
 */
sd_spi_card_t sd_spi_get_card_type(const sd_spi_t *sd);

/**
 * @brief Converts a driver status code into a short string.
 *
 * @param st Status code
 * @return Constant string describing the status
 */
const char *sd_spi_strerror(sd_spi_status_t st);

#ifdef __cplusplus
}
#endif

#endif /* INC_SD_SPI_H_ */