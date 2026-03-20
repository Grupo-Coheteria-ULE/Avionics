/**
 * @file sdlog_raw.c
 * @brief Deterministic raw fixed-page logger implementation.
 *
 * This module implements a raw page logger on top of the sd_spi driver.
 * Its main purpose is to decouple high-rate data acquisition from SD card
 * write latency by staging complete pages in RAM and committing them later.
 *
 * On-disk page format at offset 0:
 *   uint32_t magic           = SDLOG_MAGIC_PAGE ('SDLG')
 *   uint16_t version         = 1
 *   uint16_t hdr_bytes       = 32
 *   uint32_t seq
 *   uint64_t t0
 *   uint32_t payload_len
 *   uint32_t payload_crc32
 *   uint32_t header_crc32    // CRC32 of the first 28 header bytes
 *
 * Payload starts at byte offset 32.
 *
 * Optional superblock copies at meta_lba and meta_lba + 1 store the latest
 * committed write pointer and sequence number so the logger can resume after
 * reset or power interruption.
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE (Grupo de Cohetería ULE)
 * Author       : Manuel SdA. R.
 * Date         : 2026-03-06
 * Version      : v1.0
 * ---------------------------------------------------------------------------
 */

#include "sdlog_raw.h"
#include <string.h>

#define SDLOG_SECTOR_BYTES 512u


/* -------------------------------------------------------------------------- */
/* Private CRC32 Support                                                      */
/* -------------------------------------------------------------------------- */

/*
 * CRC32 implementation compatible with the common IEEE / zlib-style polynomial.
 * The table is initialized lazily on first use.
 */
static uint32_t s_crc_table[256];
static int s_crc_inited = 0;

/**
 * @brief Initializes the CRC32 lookup table.
 */
static void crc32_init(void)
{
    for (uint32_t i = 0u; i < 256u; i++)
    {
        uint32_t c = i;

        for (uint32_t k = 0u; k < 8u; k++)
        {
            c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }

        s_crc_table[i] = c;
    }

    s_crc_inited = 1;
}

/**
 * @brief Computes the CRC32 of a memory block.
 *
 * @param data Pointer to input buffer
 * @param len Number of bytes
 * @return CRC32 value
 */
static uint32_t crc32_ieee(const void *data, uint32_t len)
{
    if (!s_crc_inited)
    {
        crc32_init();
    }

    const uint8_t *p = (const uint8_t *)data;
    uint32_t c = 0xFFFFFFFFu;

    for (uint32_t i = 0u; i < len; i++)
    {
        c = s_crc_table[(c ^ p[i]) & 0xFFu] ^ (c >> 8);
    }

    return c ^ 0xFFFFFFFFu;
}


/* -------------------------------------------------------------------------- */
/* Private On-Disk Structures                                                 */
/* -------------------------------------------------------------------------- */

#pragma pack(push, 1)

/**
 * @brief Header stored at the beginning of every fixed-size log page.
 */
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t hdr_bytes;
    uint32_t seq;
    uint64_t t0;
    uint32_t payload_len;
    uint32_t payload_crc32;
    uint32_t header_crc32;
} sdlog_page_hdr_t;

/**
 * @brief Optional superblock stored on disk to resume logging state.
 *
 * Two alternating copies are maintained so that the newest valid generation
 * can be selected during startup.
 */
typedef struct
{
    uint32_t magic;             /* SDLOG_MAGIC_SB */
    uint16_t version;           /* SDLOG_VERSION */
    uint16_t reserved0;         /* Reserved for future use */
    uint32_t gen;               /* Monotonic generation counter */
    uint32_t write_lba;         /* Next committed LBA */
    uint32_t seq_next;          /* Next sequence number */
    uint32_t data_lba;          /* Data area start */
    uint32_t end_lba;           /* Data area end (exclusive) */
    uint32_t page_bytes;        /* Fixed page size */
    uint32_t meta_period_pages; /* Metadata commit cadence */
    uint32_t flags;             /* bit0 wrap, bit1 use_crc, bit2 use_preerase */
    uint32_t dropped_bytes;     /* Accumulated dropped bytes */
    uint8_t  pad[468];          /* Pad to exactly 512 bytes */
} sdlog_sb_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(sdlog_sb_t) == SDLOG_SECTOR_BYTES, "sdlog_sb_t must be exactly 512 bytes");
#else
typedef char sdlog_sb_size_must_be_512[(sizeof(sdlog_sb_t) == SDLOG_SECTOR_BYTES) ? 1 : -1];
#endif

#pragma pack(pop)


/* -------------------------------------------------------------------------- */
/* Private Helper Functions                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Returns the pointer to the beginning of a RAM page buffer.
 *
 * @param log Pointer to logger object
 * @param bi Buffer index
 * @return Pointer to the page buffer base
 */
static uint8_t *buf_ptr(sdlog_t *log, uint32_t bi)
{
    return (uint8_t *)log->cfg.buffers + (bi * log->cfg.page_bytes);
}

/**
 * @brief Builds the page header at the beginning of a buffer.
 *
 * This function computes optional CRCs and writes the final header over the
 * first bytes of the target page.
 *
 * @param log Pointer to logger object
 * @param bi Buffer index
 * @param payload_len Number of valid payload bytes in the page
 * @param t0 Timestamp associated with the page
 * @param seq Page sequence number
 */
static void page_build(sdlog_t *log, uint32_t bi, uint32_t payload_len, uint64_t t0, uint32_t seq)
{
    uint8_t *p = buf_ptr(log, bi);
    sdlog_page_hdr_t hdr;

    /* Fill the fixed metadata fields first. */
    hdr.magic = SDLOG_MAGIC_PAGE;
    hdr.version = SDLOG_VERSION;
    hdr.hdr_bytes = SDLOG_PAGE_HDR_BYTES;
    hdr.seq = seq;
    hdr.t0 = t0;
    hdr.payload_len = payload_len;

    /*
     * If CRC support is enabled, compute a CRC over the payload section that
     * starts immediately after the page header. Otherwise store zero.
     */
    hdr.payload_crc32 = log->cfg.use_crc
        ? crc32_ieee(p + SDLOG_PAGE_HDR_BYTES, payload_len)
        : 0u;

    /*
     * The header CRC covers the first 28 bytes, that is, every header field
     * except the CRC field itself.
     */
    hdr.header_crc32 = log->cfg.use_crc
        ? crc32_ieee(&hdr, 28u)
        : 0u;

    /* Copy the final header into the beginning of the page buffer. */
    memcpy(p, &hdr, sizeof(hdr));
}

/**
 * @brief Writes one 512-byte superblock sector to disk.
 *
 * @param log Pointer to logger object
 * @param lba Destination logical block address
 * @param sb Pointer to superblock image
 * @return Logger status code
 */
static sdlog_status_t sb_write(sdlog_t *log, uint32_t lba, const sdlog_sb_t *sb)
{
    sd_spi_status_t st = sd_spi_write(log->sd, lba, (const uint8_t *)sb, 1u);
    return (st == SD_SPI_OK) ? SDLOG_OK : SDLOG_ERR;
}

/**
 * @brief Reads one 512-byte superblock sector from disk.
 *
 * @param log Pointer to logger object
 * @param lba Source logical block address
 * @param sb Pointer to destination superblock image
 * @return Logger status code
 */
static sdlog_status_t sb_read(sdlog_t *log, uint32_t lba, sdlog_sb_t *sb)
{
    sd_spi_status_t st = sd_spi_read(log->sd, lba, (uint8_t *)sb, 1u);
    return (st == SD_SPI_OK) ? SDLOG_OK : SDLOG_ERR;
}

/**
 * @brief Fills a superblock image from the current runtime logger state.
 *
 * @param log Pointer to logger object
 * @param sb Pointer to superblock image to populate
 */
static void sb_fill_from_log(sdlog_t *log, sdlog_sb_t *sb)
{
    memset(sb, 0, sizeof(*sb));

    sb->magic = SDLOG_MAGIC_SB;
    sb->version = SDLOG_VERSION;
    sb->gen = log->gen;
    sb->write_lba = log->write_lba;
    sb->seq_next = log->seq_next;
    sb->data_lba = log->cfg.data_lba;
    sb->end_lba = log->cfg.end_lba;
    sb->page_bytes = log->cfg.page_bytes;
    sb->meta_period_pages = log->cfg.meta_period_pages;

    sb->flags =
        (log->cfg.wrap ? 1u : 0u) |
        (log->cfg.use_crc ? 2u : 0u) |
        (log->cfg.use_preerase ? 4u : 0u);

    sb->dropped_bytes = log->dropped_bytes;
}

/**
 * @brief Validates the superblock integrity.
 *
 * At the moment, this function only returns true. It is intentionally left
 * as the future extension point for adding CRC validation to the superblock.
 *
 * @param log Pointer to logger object
 * @param sb Pointer to superblock to validate
 * @return true if valid
 * @return false if invalid
 */
static bool sb_crc_ok(sdlog_t *log, const sdlog_sb_t *sb)
{
    (void)log;
    (void)sb;

    /* Reserved for future superblock CRC validation. */
    return true;
}

/**
 * @brief Closes the currently open page and marks it ready for disk write.
 *
 * If no page is open, the function succeeds without doing anything.
 *
 * @param log Pointer to logger object
 * @return Logger status code
 */
static sdlog_status_t close_page(sdlog_t *log)
{
    if (!log->page_open)
    {
        return SDLOG_OK;
    }

    uint32_t bi = log->tail;

    /* Finalize the page header now that payload length is known. */
    page_build(log, bi, log->fill_used, log->fill_t0, log->fill_seq);

    /* Mark the page as ready so sdlog_poll() can write it later. */
    log->state[bi] = 2u;
    log->ready_count++;

    /* Advance tail to the next buffer slot. */
    log->tail = (log->tail + 1u) % log->cfg.num_buffers;

    /* Clear the "currently filling" state. */
    log->page_open = false;
    log->fill_used = 0u;
    log->fill_t0 = 0u;
    log->fill_seq = 0u;

    return SDLOG_OK;
}

/**
 * @brief Writes the latest committed state into one alternating superblock.
 *
 * The destination alternates between meta_lba and meta_lba + 1 based on the
 * current generation counter.
 *
 * @param log Pointer to logger object
 * @return Logger status code
 */
static sdlog_status_t meta_commit(sdlog_t *log)
{
    if (log->cfg.meta_lba == 0u)
    {
        return SDLOG_OK;
    }

    sdlog_sb_t sb;
    sb_fill_from_log(log, &sb);

    /* Alternate between the two metadata slots to improve robustness. */
    uint32_t lba = log->cfg.meta_lba + (log->gen & 1u);

    sdlog_status_t st = sb_write(log, lba, &sb);
    if (st == SDLOG_OK)
    {
        log->gen++;
    }

    return st;
}


/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

sdlog_status_t sdlog_init(sdlog_t *log, sd_spi_t *sd, const sdlog_cfg_t *cfg)
{
    if ((log == NULL) || (sd == NULL) || (cfg == NULL))
    {
        return SDLOG_PARAM;
    }

    /* Validate external RAM buffer configuration. */
    if ((cfg->buffers == NULL) ||
        (cfg->num_buffers < 2u) ||
        (cfg->num_buffers > SDLOG_MAX_BUFFERS))
    {
        return SDLOG_PARAM;
    }

    /* Page size must be a positive multiple of one SD logical sector. */
    if ((cfg->page_bytes == 0u) ||
        ((cfg->page_bytes % SDLOG_SECTOR_BYTES) != 0u))
    {
        return SDLOG_PARAM;
    }

    /* Data area must be valid and non-empty. */
    if ((cfg->data_lba == 0u) || (cfg->end_lba <= cfg->data_lba))
    {
        return SDLOG_PARAM;
    }

    /*
     * Ensure at least one full page fits into the configured data region.
     * Without this check, writes could fail immediately or wrap endlessly.
     */
    if ((cfg->end_lba - cfg->data_lba) < (cfg->page_bytes / SDLOG_SECTOR_BYTES))
    {
        return SDLOG_PARAM;
    }

    /* Reset the full logger state to a known starting point. */
    memset(log, 0, sizeof(*log));
    log->sd = sd;
    log->cfg = *cfg;

    /* Precompute commonly used derived values. */
    log->page_sectors = cfg->page_bytes / SDLOG_SECTOR_BYTES;
    log->payload_cap = cfg->page_bytes - SDLOG_PAGE_HDR_BYTES;

    /*
     * Payload capacity must remain positive.
     * This also protects against pathological page size settings.
     */
    if (log->payload_cap == 0u)
    {
        return SDLOG_PARAM;
    }

    /* Default starting point if no valid superblock is found. */
    log->write_lba = cfg->data_lba;
    log->seq_next = 0u;
    log->gen = 1u;

    /* Initialize all RAM buffer states as free. */
    for (uint32_t i = 0u; i < cfg->num_buffers; i++)
    {
        log->state[i] = 0u;
    }

    log->head = 0u;
    log->tail = 0u;
    log->ready_count = 0u;

    log->page_open = false;
    log->fill_used = 0u;
    log->fill_t0 = 0u;
    log->fill_seq = 0u;

    log->pages_written_since_meta = 0u;
    log->dropped_bytes = 0u;

    /*
     * If metadata sectors are configured, try to resume from the newest valid
     * superblock copy.
     */
    if (cfg->meta_lba != 0u)
    {
        sdlog_sb_t sb0, sb1;

        bool ok0 = (sb_read(log, cfg->meta_lba, &sb0) == SDLOG_OK) &&
                   (sb0.magic == SDLOG_MAGIC_SB) &&
                   (sb0.version == SDLOG_VERSION) &&
                   sb_crc_ok(log, &sb0);

        bool ok1 = (sb_read(log, cfg->meta_lba + 1u, &sb1) == SDLOG_OK) &&
                   (sb1.magic == SDLOG_MAGIC_SB) &&
                   (sb1.version == SDLOG_VERSION) &&
                   sb_crc_ok(log, &sb1);

        const sdlog_sb_t *best = NULL;

        if (ok0 && ok1)
        {
            /* Choose the newest generation if both copies are valid. */
            best = (sb1.gen >= sb0.gen) ? &sb1 : &sb0;
        }
        else if (ok0)
        {
            best = &sb0;
        }
        else if (ok1)
        {
            best = &sb1;
        }

        if (best != NULL)
        {
            /*
             * Resume committed pointers from disk. Generation is advanced so the
             * next metadata write lands as a strictly newer copy.
             */
            log->gen = best->gen + 1u;
            log->write_lba = best->write_lba;
            log->seq_next = best->seq_next;
            log->dropped_bytes = best->dropped_bytes;
        }
    }

    return SDLOG_OK;
}

sdlog_status_t sdlog_append(sdlog_t *log, const void *data, uint32_t len, uint64_t t0)
{
    if ((log == NULL) || (data == NULL) || (len == 0u))
    {
        return SDLOG_PARAM;
    }

    const uint8_t *p = (const uint8_t *)data;

    while (len > 0u)
    {
        /*
         * If no page is currently open, try to open one in the current tail
         * slot. If that slot is not free, the producer has outrun the writer.
         */
        if (!log->page_open)
        {
            if (log->state[log->tail] != 0u)
            {
                /* Account for all bytes that could not be queued. */
                log->dropped_bytes += len;
                return SDLOG_FULL;
            }

            /* Reserve the tail buffer as the currently filling page. */
            log->state[log->tail] = 1u;
            log->page_open = true;
            log->fill_used = 0u;
            log->fill_t0 = t0;
            log->fill_seq = log->seq_next++;
        }

        /* Compute how many bytes still fit in the open page payload. */
        uint32_t cap = log->payload_cap - log->fill_used;
        uint32_t take = (len < cap) ? len : cap;

        /* Copy the next payload chunk immediately after the page header area. */
        uint8_t *buf = buf_ptr(log, log->tail);
        memcpy(buf + SDLOG_PAGE_HDR_BYTES + log->fill_used, p, take);

        /* Advance producer pointers. */
        log->fill_used += take;
        p += take;
        len -= take;

        /*
         * If the payload area is full, seal the page so it becomes eligible
         * for deferred SD write during sdlog_poll().
         */
        if (log->fill_used == log->payload_cap)
        {
            sdlog_status_t st = close_page(log);
            if (st != SDLOG_OK)
            {
                return st;
            }
        }
    }

    return SDLOG_OK;
}

sdlog_status_t sdlog_poll(sdlog_t *log, uint32_t max_pages, uint32_t *pages_written)
{
    if (log == NULL)
    {
        return SDLOG_PARAM;
    }

    if (pages_written != NULL)
    {
        *pages_written = 0u;
    }

    if (max_pages == 0u)
    {
        return SDLOG_OK;
    }

    for (uint32_t i = 0u; i < max_pages; i++)
    {
        if (log->ready_count == 0u)
        {
            /* No closed pages are pending in RAM. */
            break;
        }

        uint32_t bi = log->head;

        /*
         * Defensive recovery path:
         * if head points to a non-ready slot, free it and continue.
         */
        if (log->state[bi] != 2u)
        {
            log->state[bi] = 0u;
            log->head = (log->head + 1u) % log->cfg.num_buffers;
            log->ready_count--;
            continue;
        }

        /*
         * Ensure enough space remains for the whole fixed-size page.
         * If wrap is disabled, report the disk region as full.
         */
        if ((log->write_lba + log->page_sectors) > log->cfg.end_lba)
        {
            if (!log->cfg.wrap)
            {
                return SDLOG_FULL;
            }

            /* Wrap back to the start of the configured data area. */
            log->write_lba = log->cfg.data_lba;
        }

        /*
         * Optionally hint the card about the upcoming write size.
         * The logger ignores failures here because this is only an optimization.
         */
        if (log->cfg.use_preerase)
        {
            (void)sd_spi_preerase(log->sd, log->page_sectors);
        }

        /* Commit the fully built page buffer to the SD card. */
        sd_spi_status_t wr = sd_spi_write(log->sd,
                                          log->write_lba,
                                          buf_ptr(log, bi),
                                          log->page_sectors);
        if (wr != SD_SPI_OK)
        {
            return SDLOG_ERR;
        }

        /* Advance the committed disk write pointer past the written page. */
        log->write_lba += log->page_sectors;

        /* Release the buffer so the producer can reuse it. */
        log->state[bi] = 0u;
        log->head = (log->head + 1u) % log->cfg.num_buffers;
        log->ready_count--;

        /* Update accounting and optional output count. */
        log->pages_written_since_meta++;

        if (pages_written != NULL)
        {
            (*pages_written)++;
        }

        /*
         * Periodically commit metadata so a future restart can resume near the
         * latest committed position.
         */
        if ((log->cfg.meta_lba != 0u) && (log->cfg.meta_period_pages != 0u))
        {
            if (log->pages_written_since_meta >= log->cfg.meta_period_pages)
            {
                (void)meta_commit(log);
                log->pages_written_since_meta = 0u;
            }
        }
    }

    return SDLOG_OK;
}

sdlog_status_t sdlog_flush(sdlog_t *log, uint32_t max_pages)
{
    if (log == NULL)
    {
        return SDLOG_PARAM;
    }

    /* Seal the current partial page, if one exists. */
    sdlog_status_t st = close_page(log);
    if (st != SDLOG_OK)
    {
        return st;
    }

    /*
     * Try to write pending pages to the card. The actual number written
     * depends on max_pages and on how many were ready.
     */
    uint32_t dummy = 0u;
    st = sdlog_poll(log, max_pages, &dummy);
    if (st != SDLOG_OK)
    {
        return st;
    }

    /* Perform a final metadata commit so resume pointers are up to date. */
    (void)meta_commit(log);

    return SDLOG_OK;
}

uint32_t sdlog_get_dropped_bytes(const sdlog_t *log)
{
    return (log != NULL) ? log->dropped_bytes : 0u;
}
