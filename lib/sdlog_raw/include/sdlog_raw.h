/**
 * @file sdlog_raw.h
 * @brief Deterministic raw page logger for SD cards.
 *
 * This module implements a fixed-page raw logger designed for embedded
 * high-rate acquisition systems such as avionics. The critical acquisition
 * path only copies payload data into RAM buffers, while SD card writes are
 * deferred to a lower-priority context through sdlog_poll() or sdlog_flush().
 *
 * Main features:
 *  - Fixed-size on-disk pages
 *  - Multi-buffer RAM staging
 *  - Optional wrap-around logging area
 *  - Optional superblock-based resume after reset
 *  - Optional payload/header CRC generation
 *  - Optional ACMD23 pre-erase hint before large writes
 *
 * Disk layout:
 *  - Optional superblock copies at meta_lba and meta_lba + 1
 *  - Data pages in the interval [data_lba, end_lba)
 *  - Each page has a fixed size page_bytes, which must be a multiple of 512
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE (Grupo de Cohetería ULE)
 * Author       : Manuel SdA. R.
 * Date         : 2026-03-06
 * Version      : v1.0
 * ---------------------------------------------------------------------------
 */

#ifndef INC_SDLOG_RAW_H_
#define INC_SDLOG_RAW_H_

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "sd_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Public Constants                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Magic value stored in each page header ("SDLG").
 */
#define SDLOG_MAGIC_PAGE 0x53444C47u

/**
 * @brief Magic value stored in the optional superblock ("SBLG").
 */
#define SDLOG_MAGIC_SB   0x53424C47u

/**
 * @brief Current on-disk format version.
 */
#define SDLOG_VERSION    1u

/**
 * @brief Maximum number of RAM page buffers supported by the logger object.
 */
#ifndef SDLOG_MAX_BUFFERS
#define SDLOG_MAX_BUFFERS 16u
#endif

/**
 * @brief Fixed page header size in bytes.
 *
 * Keep this constant stable if you want offline extractors and parsers
 * to remain compatible with previously generated logs.
 */
#define SDLOG_PAGE_HDR_BYTES 32u


/* -------------------------------------------------------------------------- */
/* Public Types                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Logger status codes.
 */
typedef enum
{
    SDLOG_OK    = 0,   /**< Operation completed successfully */
    SDLOG_ERR   = -1,  /**< Generic logger or storage error */
    SDLOG_FULL  = -2,  /**< No free RAM buffers or no free disk space */
    SDLOG_PARAM = -3   /**< Invalid parameter */
} sdlog_status_t;

/**
 * @brief Logger configuration structure.
 *
 * This configuration is copied into the logger during sdlog_init().
 */
typedef struct
{
    /**
     * @brief Optional superblock base LBA.
     *
     * If different from zero, sectors meta_lba and meta_lba + 1 are used as
     * alternating superblock copies so logging can resume after reset.
     */
    uint32_t meta_lba;

    /**
     * @brief First LBA of the data area.
     */
    uint32_t data_lba;

    /**
     * @brief End of the data area (exclusive).
     */
    uint32_t end_lba;

    /**
     * @brief Fixed page size in bytes.
     *
     * Must be a non-zero multiple of 512 bytes.
     * A large value such as 32768 bytes is typically recommended.
     */
    uint32_t page_bytes;

    /**
     * @brief Pointer to external RAM buffer storage.
     *
     * The storage must contain num_buffers * page_bytes bytes.
     */
    void *buffers;

    /**
     * @brief Number of RAM page buffers.
     *
     * Valid range: 2 .. SDLOG_MAX_BUFFERS
     */
    uint32_t num_buffers;

    /**
     * @brief Enables wrap-around when the data area becomes full.
     *
     * If false, writes stop with SDLOG_FULL once the storage region ends.
     * If true, writing restarts at data_lba.
     */
    bool wrap;

    /**
     * @brief Enables CRC32 generation for page payloads and headers.
     */
    bool use_crc;

    /**
     * @brief Enables ACMD23 pre-erase hint before large writes.
     */
    bool use_preerase;

    /**
     * @brief Number of written pages between superblock commits.
     *
     * Only used if meta_lba is non-zero.
     * If zero, periodic metadata commits are disabled.
     */
    uint32_t meta_period_pages;
} sdlog_cfg_t;

/**
 * @brief Runtime logger object.
 */
typedef struct
{
    sd_spi_t *sd;           /**< Attached SD card driver */
    sdlog_cfg_t cfg;        /**< Copy of the user configuration */

    uint32_t page_sectors;  /**< Page size expressed in 512-byte sectors */
    uint32_t payload_cap;   /**< Usable payload bytes per page */

    /* Persistent committed disk pointers */
    uint32_t write_lba;     /**< Next committed free LBA on disk */
    uint32_t seq_next;      /**< Next committed page sequence number */

    /* Superblock generation */
    uint32_t gen;           /**< Superblock generation counter */

    /*
     * Per-buffer state:
     * 0 = free
     * 1 = currently being filled
     * 2 = ready to be written to SD
     */
    uint8_t  state[SDLOG_MAX_BUFFERS];

    uint32_t head;          /**< Oldest ready buffer */
    uint32_t tail;          /**< Buffer currently being filled or next free slot */
    uint32_t ready_count;   /**< Number of ready pages waiting for disk write */

    /* Current open page state */
    bool     page_open;     /**< True while a page is actively being filled */
    uint32_t fill_used;     /**< Payload bytes already copied into current page */
    uint64_t fill_t0;       /**< Timestamp associated with current page */
    uint32_t fill_seq;      /**< Sequence number associated with current page */

    /* Metadata and diagnostics */
    uint32_t pages_written_since_meta; /**< Pages written since last metadata commit */
    uint32_t dropped_bytes;            /**< Bytes dropped due to lack of free buffers */
} sdlog_t;


/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initializes the raw logger.
 *
 * This function validates the configuration, clears the runtime state,
 * attaches the SD driver, computes internal derived values, and optionally
 * attempts to resume from the latest valid superblock.
 *
 * The SD card driver must already be initialized with sd_spi_init().
 *
 * @param log Pointer to logger object
 * @param sd Pointer to initialized SD SPI driver
 * @param cfg Pointer to logger configuration
 * @return Logger status code
 */
sdlog_status_t sdlog_init(sdlog_t *log, sd_spi_t *sd, const sdlog_cfg_t *cfg);

/**
 * @brief Appends payload bytes to the logger.
 *
 * The function only copies data into RAM staging pages. It does not write to
 * the SD card directly. This makes it suitable for time-critical sampling code.
 *
 * If no free RAM buffer is available, the function returns SDLOG_FULL and
 * increments dropped_bytes by the number of bytes that could not be queued.
 *
 * @param log Pointer to logger object
 * @param data Pointer to payload bytes to append
 * @param len Number of bytes to append
 * @param t0 Timestamp associated with the page if a new page is opened
 * @return Logger status code
 */
sdlog_status_t sdlog_append(sdlog_t *log, const void *data, uint32_t len, uint64_t t0);

/**
 * @brief Attempts to write closed pages to the SD card.
 *
 * This function should be called from a non-critical loop or background task.
 * It writes up to max_pages already-closed pages from RAM to disk.
 *
 * @param log Pointer to logger object
 * @param max_pages Maximum number of pages to write in this call
 * @param pages_written Optional pointer to store the number of pages written
 * @return Logger status code
 */
sdlog_status_t sdlog_poll(sdlog_t *log, uint32_t max_pages, uint32_t *pages_written);

/**
 * @brief Forces the current partial page to close and writes pending pages.
 *
 * Useful before shutdown or before intentionally ending a logging session.
 *
 * @param log Pointer to logger object
 * @param max_pages Maximum number of pages to write in this call
 * @return Logger status code
 */
sdlog_status_t sdlog_flush(sdlog_t *log, uint32_t max_pages);

/**
 * @brief Returns the accumulated number of dropped payload bytes.
 *
 * @param log Pointer to logger object
 * @return Total dropped bytes, or 0 on invalid pointer
 */
uint32_t sdlog_get_dropped_bytes(const sdlog_t *log);

#ifdef __cplusplus
}
#endif

#endif /* INC_SDLOG_RAW_H_ */