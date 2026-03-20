/**
 * @file sd_spi.c
 * @brief SPI-mode SD card driver implementation.
 *
 * This module implements block-level SD card access using the SPI protocol.
 * It handles card startup, command framing, response parsing, data packet
 * transfers, and capacity extraction from the CSD register.
 *
 * ---------------------------------------------------------------------------
 * Project      : Avionics Software
 * Organization : GC_ULE (Grupo de Cohetería ULE)
 * Author       : Manuel SdA. R.
 * Date         : 2026-03-06
 * Version      : v1.0
 * ---------------------------------------------------------------------------
 */

#include "sd_spi.h"
#include <string.h>


/* -------------------------------------------------------------------------- */
/* Private SD SPI Constants                                                   */
/* -------------------------------------------------------------------------- */

/* Data tokens used during read/write transfers. */
#define SD_TOKEN_START_BLOCK      0xFEu
#define SD_TOKEN_MULTI_WRITE      0xFCu
#define SD_TOKEN_STOP_TRAN        0xFDu

/* R1 response bits. */
#define R1_IDLE_STATE             0x01u
#define R1_ILLEGAL_COMMAND        0x04u

/* SD commands used by this driver. */
#define CMD0                      0u
#define CMD8                      8u
#define CMD9                      9u
#define CMD12                     12u
#define CMD16                     16u
#define CMD17                     17u
#define CMD18                     18u
#define CMD24                     24u
#define CMD25                     25u
#define CMD55                     55u
#define CMD58                     58u
#define ACMD41                    41u
#define ACMD23                    23u


/* -------------------------------------------------------------------------- */
/* Private Helper Functions                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Maps an OSAL status code into an SD SPI driver status code.
 *
 * @param st OSAL status code
 * @return Equivalent SD SPI status code
 */
static sd_spi_status_t map_osal(osal_status_t st)
{
    if (st == OSAL_OK)
    {
        return SD_SPI_OK;
    }

    if (st == OSAL_TIMEOUT)
    {
        return SD_SPI_TIMEOUT;
    }

    return SD_SPI_ERR;
}

/**
 * @brief Asserts the card chip-select line.
 *
 * @param sd Pointer to SD driver object
 */
static inline void cs_select(sd_spi_t *sd)
{
    if ((sd != NULL) && (sd->bus != NULL) && (sd->bus->cs_select != NULL))
    {
        sd->bus->cs_select(sd->bus->cs_user);
    }
}

/**
 * @brief Releases the card chip-select line.
 *
 * @param sd Pointer to SD driver object
 */
static inline void cs_deselect(sd_spi_t *sd)
{
    if ((sd != NULL) && (sd->bus != NULL) && (sd->bus->cs_deselect != NULL))
    {
        sd->bus->cs_deselect(sd->bus->cs_user);
    }
}

/**
 * @brief Performs a raw SPI transfer through the OSAL layer.
 *
 * @param sd Pointer to SD driver object
 * @param tx Transmit buffer
 * @param rx Receive buffer
 * @param len Number of bytes to transfer
 * @return SD SPI status code
 */
static sd_spi_status_t spi_txrx(sd_spi_t *sd, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if ((sd == NULL) || (sd->bus == NULL))
    {
        return SD_SPI_PARAM;
    }

    return map_osal(osal_spi_txrx(sd->bus, tx, rx, len));
}

/**
 * @brief Transfers a single byte and optionally returns the received byte.
 *
 * @param sd Pointer to SD driver object
 * @param txb Byte to transmit
 * @param rxb Pointer to store received byte, or NULL to ignore it
 * @return SD SPI status code
 */
static sd_spi_status_t spi_xfer_byte(sd_spi_t *sd, uint8_t txb, uint8_t *rxb)
{
    uint8_t tx = txb;
    uint8_t rx = 0xFFu;

    sd_spi_status_t st = spi_txrx(sd, &tx, &rx, 1u);
    if (st != SD_SPI_OK)
    {
        return st;
    }

    if (rxb != NULL)
    {
        *rxb = rx;
    }

    return SD_SPI_OK;
}

/**
 * @brief Reads bytes from SPI by transmitting dummy 0xFF bytes.
 *
 * @param sd Pointer to SD driver object
 * @param dst Destination buffer
 * @param len Number of bytes to read
 * @return SD SPI status code
 */
static sd_spi_status_t spi_read(sd_spi_t *sd, uint8_t *dst, uint32_t len)
{
    static const uint8_t ff[32] =
    {
        0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,
        0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,
        0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,
        0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu
    };

    if ((dst == NULL) && (len != 0u))
    {
        return SD_SPI_PARAM;
    }

    /*
     * Read in chunks so we do not need a large temporary buffer.
     * Every received byte is clocked out by sending a dummy 0xFF byte.
     */
    while (len > 0u)
    {
        uint16_t chunk = (len > sizeof(ff)) ? (uint16_t)sizeof(ff) : (uint16_t)len;

        sd_spi_status_t st = spi_txrx(sd, ff, dst, chunk);
        if (st != SD_SPI_OK)
        {
            return st;
        }

        dst += chunk;
        len -= chunk;
    }

    return SD_SPI_OK;
}

/**
 * @brief Writes bytes over SPI while discarding received bytes.
 *
 * @param sd Pointer to SD driver object
 * @param src Source buffer
 * @param len Number of bytes to write
 * @return SD SPI status code
 */
static sd_spi_status_t spi_write(sd_spi_t *sd, const uint8_t *src, uint32_t len)
{
    uint8_t rx_dummy[32];

    if ((src == NULL) && (len != 0u))
    {
        return SD_SPI_PARAM;
    }

    /*
     * Write in chunks and ignore the simultaneously received bytes.
     * SPI is always full-duplex, even when we conceptually "just write".
     */
    while (len > 0u)
    {
        uint16_t chunk = (len > sizeof(rx_dummy)) ? (uint16_t)sizeof(rx_dummy) : (uint16_t)len;

        sd_spi_status_t st = spi_txrx(sd, src, rx_dummy, chunk);
        if (st != SD_SPI_OK)
        {
            return st;
        }

        src += chunk;
        len -= chunk;
    }

    return SD_SPI_OK;
}

/**
 * @brief Sends idle clocks with chip-select released.
 *
 * SD cards require at least 74 clock cycles with CS high before the first
 * command in SPI mode. This function sends 80 clocks using ten 0xFF bytes.
 *
 * @param sd Pointer to SD driver object
 */
static void sd_idle_clocks(sd_spi_t *sd)
{
    uint8_t dump[10];

    /* Keep the card deselected while providing the required startup clocks. */
    cs_deselect(sd);

    /* Ignore received bytes; only the generated clocks matter here. */
    (void)spi_read(sd, dump, sizeof(dump));
}

/**
 * @brief Waits until the card is no longer busy.
 *
 * During internal programming operations, the card may hold MISO low and
 * return bytes different from 0xFF. The card is considered ready again when
 * it returns 0xFF.
 *
 * @param sd Pointer to SD driver object
 * @param timeout_ms Timeout in milliseconds
 * @return SD SPI status code
 */
static sd_spi_status_t sd_wait_not_busy(sd_spi_t *sd, uint32_t timeout_ms)
{
    uint32_t t0 = osal_ticks_ms();
    uint8_t b = 0u;

    do
    {
        sd_spi_status_t st = spi_xfer_byte(sd, 0xFFu, &b);
        if (st != SD_SPI_OK)
        {
            return st;
        }

        if (b == 0xFFu)
        {
            return SD_SPI_OK;
        }
    }
    while ((osal_ticks_ms() - t0) < timeout_ms);

    return SD_SPI_TIMEOUT;
}

/**
 * @brief Reads an R1 response byte from the card.
 *
 * After a command is sent, the card may output one or more 0xFF bytes before
 * the actual response. This function polls until a non-0xFF byte is received
 * or the timeout expires.
 *
 * @param sd Pointer to SD driver object
 * @param r1_out Pointer to store the R1 response
 * @param timeout_ms Timeout in milliseconds
 * @return SD SPI status code
 */
static sd_spi_status_t sd_read_r1(sd_spi_t *sd, uint8_t *r1_out, uint32_t timeout_ms)
{
    uint32_t t0 = osal_ticks_ms();
    uint8_t r1 = 0xFFu;

    do
    {
        sd_spi_status_t st = spi_xfer_byte(sd, 0xFFu, &r1);
        if (st != SD_SPI_OK)
        {
            return st;
        }

        if (r1 != 0xFFu)
        {
            if (r1_out != NULL)
            {
                *r1_out = r1;
            }
            return SD_SPI_OK;
        }
    }
    while ((osal_ticks_ms() - t0) < timeout_ms);

    return SD_SPI_TIMEOUT;
}

/**
 * @brief Sends a standard SD SPI command frame and reads its R1 response.
 *
 * SPI command frame format:
 * [0x40 | cmd][arg byte 3][arg byte 2][arg byte 1][arg byte 0][crc]
 *
 * CRC is only mandatory for selected commands during startup, but the stop
 * bit must always be set in SPI mode.
 *
 * @param sd Pointer to SD driver object
 * @param cmd Command index
 * @param arg Command argument
 * @param crc CRC byte; bit 0 is forced to 1
 * @param r1_out Pointer to store R1 response
 * @return SD SPI status code
 */
static sd_spi_status_t sd_send_cmd(sd_spi_t *sd,
                                   uint8_t cmd,
                                   uint32_t arg,
                                   uint8_t crc,
                                   uint8_t *r1_out)
{
    uint8_t frame[6];
    uint8_t dump = 0u;

    /* Build the 6-byte SPI command frame. */
    frame[0] = (uint8_t)(0x40u | cmd);
    frame[1] = (uint8_t)(arg >> 24);
    frame[2] = (uint8_t)(arg >> 16);
    frame[3] = (uint8_t)(arg >> 8);
    frame[4] = (uint8_t)(arg);
    frame[5] = (uint8_t)(crc | 0x01u);

    /* Provide at least one idle byte before issuing the command. */
    sd_spi_status_t st = spi_xfer_byte(sd, 0xFFu, &dump);
    if (st != SD_SPI_OK)
    {
        return st;
    }

    /* Send the command frame bytes. */
    st = spi_write(sd, frame, sizeof(frame));
    if (st != SD_SPI_OK)
    {
        return st;
    }

    /* Wait for the first valid R1 response byte. */
    return sd_read_r1(sd, r1_out, sd->timeout_ms);
}

/**
 * @brief Sends an application-specific command sequence.
 *
 * ACMD commands are issued as CMD55 followed by the actual ACMD.
 *
 * @param sd Pointer to SD driver object
 * @param acmd Application command index
 * @param arg Command argument
 * @param r1_out Pointer to store final R1 response
 * @return SD SPI status code
 */
static sd_spi_status_t sd_send_acmd(sd_spi_t *sd, uint8_t acmd, uint32_t arg, uint8_t *r1_out)
{
    uint8_t r1 = 0u;

    sd_spi_status_t st = sd_send_cmd(sd, CMD55, 0u, 0x01u, &r1);
    if (st != SD_SPI_OK)
    {
        return st;
    }

    st = sd_send_cmd(sd, acmd, arg, 0x01u, &r1);
    if (st != SD_SPI_OK)
    {
        return st;
    }

    if (r1_out != NULL)
    {
        *r1_out = r1;
    }

    return SD_SPI_OK;
}

/**
 * @brief Reads a data packet after a read command.
 *
 * The function waits for the start token, reads the payload, and then discards
 * the two trailing CRC bytes.
 *
 * @param sd Pointer to SD driver object
 * @param buf Destination buffer
 * @param len Payload length in bytes
 * @param timeout_ms Timeout in milliseconds
 * @return SD SPI status code
 */
static sd_spi_status_t sd_read_data_packet(sd_spi_t *sd,
                                           uint8_t *buf,
                                           uint32_t len,
                                           uint32_t timeout_ms)
{
    uint32_t t0 = osal_ticks_ms();
    uint8_t token = 0xFFu;

    if ((buf == NULL) && (len != 0u))
    {
        return SD_SPI_PARAM;
    }

    /*
     * Wait for the data start token.
     * If the card returns a low nibble error token instead, abort immediately.
     */
    do
    {
        sd_spi_status_t st = spi_xfer_byte(sd, 0xFFu, &token);
        if (st != SD_SPI_OK)
        {
            return st;
        }

        if (token == SD_TOKEN_START_BLOCK)
        {
            break;
        }

        if ((token & 0xF0u) == 0x00u)
        {
            return SD_SPI_ERR;
        }
    }
    while ((osal_ticks_ms() - t0) < timeout_ms);

    if (token != SD_TOKEN_START_BLOCK)
    {
        return SD_SPI_TIMEOUT;
    }

    /* Read the payload itself. */
    sd_spi_status_t st = spi_read(sd, buf, len);
    if (st != SD_SPI_OK)
    {
        return st;
    }

    /* The current driver ignores data CRC and discards the two CRC bytes. */
    uint8_t crc[2];
    return spi_read(sd, crc, 2u);
}

/**
 * @brief Writes one data packet and checks the card data response token.
 *
 * The transmitted sequence is:
 * [data token][payload][dummy CRC(2)]
 *
 * @param sd Pointer to SD driver object
 * @param token Data token to transmit
 * @param buf Payload buffer
 * @param len Payload length in bytes
 * @return SD SPI status code
 */
static sd_spi_status_t sd_write_data_packet(sd_spi_t *sd,
                                            uint8_t token,
                                            const uint8_t *buf,
                                            uint32_t len)
{
    uint8_t resp = 0u;

    if ((buf == NULL) && (len != 0u))
    {
        return SD_SPI_PARAM;
    }

    /* Send the token announcing the beginning of the data packet. */
    sd_spi_status_t st = spi_xfer_byte(sd, token, &resp);
    if (st != SD_SPI_OK)
    {
        return st;
    }

    /* Send the payload bytes. */
    st = spi_write(sd, buf, len);
    if (st != SD_SPI_OK)
    {
        return st;
    }

    /* Send dummy CRC bytes because CRC is not actively used in this driver. */
    {
        uint8_t crc[2] = { 0xFFu, 0xFFu };
        st = spi_write(sd, crc, 2u);
        if (st != SD_SPI_OK)
        {
            return st;
        }
    }

    /* Read the data response token returned by the card. */
    st = spi_xfer_byte(sd, 0xFFu, &resp);
    if (st != SD_SPI_OK)
    {
        return st;
    }

    /*
     * Accepted pattern is xxx0_101.
     * Any other status means the card rejected the data block.
     */
    if ((resp & 0x1Fu) != 0x05u)
    {
        return SD_SPI_ERR;
    }

    /* Wait until the card finishes internal programming. */
    return sd_wait_not_busy(sd, sd->timeout_ms);
}

/**
 * @brief Extracts a bit-field from the 16-byte CSD register.
 *
 * Bit numbering follows the SD specification:
 * bit 127 is csd[0] bit 7, and bit 0 is csd[15] bit 0.
 *
 * @param csd 16-byte CSD buffer
 * @param msb Most significant bit index
 * @param lsb Least significant bit index
 * @return Extracted value
 */
static uint32_t csd_get_bits(const uint8_t csd[16], uint32_t msb, uint32_t lsb)
{
    uint32_t val = 0u;

    /*
     * Extract one bit at a time from the specified range and accumulate
     * the result from MSB to LSB.
     */
    for (uint32_t b = msb; b >= lsb; b--)
    {
        uint32_t byte = (127u - b) / 8u;
        uint32_t bit  = (127u - b) % 8u;
        uint32_t bitv = (csd[byte] >> (7u - bit)) & 1u;

        val = (val << 1) | bitv;

        /* Prevent unsigned wraparound when the loop reaches lsb. */
        if (b == lsb)
        {
            break;
        }
    }

    return val;
}

/**
 * @brief Reads the 16-byte CSD register from the card.
 *
 * @param sd Pointer to SD driver object
 * @param csd Destination buffer for the CSD register
 * @return SD SPI status code
 */
static sd_spi_status_t sd_read_csd(sd_spi_t *sd, uint8_t csd[16])
{
    uint8_t r1 = 0u;

    cs_select(sd);

    /* CMD9 requests the CSD register. */
    sd_spi_status_t st = sd_send_cmd(sd, CMD9, 0u, 0x01u, &r1);
    if (st != SD_SPI_OK)
    {
        cs_deselect(sd);
        return st;
    }

    if (r1 != 0x00u)
    {
        cs_deselect(sd);
        return SD_SPI_ERR;
    }

    /* Read the 16-byte CSD payload plus discarded CRC. */
    st = sd_read_data_packet(sd, csd, 16u, sd->timeout_ms);

    cs_deselect(sd);

    /* Provide one trailing idle byte after the command transaction. */
    (void)spi_xfer_byte(sd, 0xFFu, NULL);

    return st;
}

/**
 * @brief Updates the logical sector count by decoding the CSD register.
 *
 * Supports both CSD version 1.0 (SDSC) and CSD version 2.0 (SDHC/SDXC-style
 * layout, though this driver currently only labels high-capacity cards as
 * SDHC for simplicity).
 *
 * @param sd Pointer to SD driver object
 * @return SD SPI status code
 */
static sd_spi_status_t sd_update_capacity(sd_spi_t *sd)
{
    uint8_t csd[16];

    sd_spi_status_t st = sd_read_csd(sd, csd);
    if (st != SD_SPI_OK)
    {
        return st;
    }

    uint8_t csd_structure = (uint8_t)csd_get_bits(csd, 127u, 126u);

    if (csd_structure == 1u)
    {
        /*
         * CSD v2.0:
         * sector_count = (C_SIZE + 1) * 1024
         */
        uint32_t c_size = csd_get_bits(csd, 69u, 48u);
        sd->sector_count = (c_size + 1u) * 1024u;
        return SD_SPI_OK;
    }

    if (csd_structure == 0u)
    {
        /*
         * CSD v1.0:
         * capacity = BLOCKNR * BLOCK_LEN
         * sector_count = capacity / 512
         */
        uint32_t c_size      = csd_get_bits(csd, 73u, 62u);
        uint32_t c_mult      = csd_get_bits(csd, 49u, 47u);
        uint32_t read_bl_len = csd_get_bits(csd, 83u, 80u);

        uint32_t block_len = 1u << read_bl_len;
        uint32_t mult      = 1u << (c_mult + 2u);
        uint32_t blocknr   = (c_size + 1u) * mult;
        uint64_t capacity  = (uint64_t)blocknr * (uint64_t)block_len;

        sd->sector_count = (uint32_t)(capacity / SD_SPI_SECTOR_SIZE);
        return SD_SPI_OK;
    }

    return SD_SPI_ERR;
}

/**
 * @brief Converts a logical block address into the command argument format.
 *
 * SDHC cards use block addressing directly. SDSC cards expect a byte address,
 * so the logical block number must be multiplied by 512.
 *
 * @param sd Pointer to SD driver object
 * @param lba Logical block address
 * @return Command argument value
 */
static inline uint32_t lba_to_arg(const sd_spi_t *sd, uint32_t lba)
{
    return (sd->card_type == SD_SPI_CARD_SDHC) ? lba : (lba * SD_SPI_SECTOR_SIZE);
}


/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void sd_spi_attach(sd_spi_t *sd, osal_spi_t *bus, uint32_t timeout_ms)
{
    if (sd == NULL)
    {
        return;
    }

    /* Reset the full software object so all fields start from a known state. */
    memset(sd, 0, sizeof(*sd));

    /* Store bus reference and runtime configuration. */
    sd->bus = bus;
    sd->timeout_ms = (timeout_ms == 0u) ? 2000u : timeout_ms;

    /* Explicit initialization for readability. */
    sd->card_type = SD_SPI_CARD_UNKNOWN;
    sd->sector_count = 0u;
    sd->initialized = false;
    sd->use_crc = false;
}

sd_spi_status_t sd_spi_init(sd_spi_t *sd)
{
    uint8_t r1 = 0u;
    sd_spi_status_t st;
    bool v2 = false;

    if ((sd == NULL) || (sd->bus == NULL) ||
        (sd->bus->cs_select == NULL) || (sd->bus->cs_deselect == NULL))
    {
        return SD_SPI_PARAM;
    }

    /* Reset runtime identification fields before attempting a new init. */
    sd->initialized = false;
    sd->card_type = SD_SPI_CARD_UNKNOWN;
    sd->sector_count = 0u;

    /* Send initial clocks with CS high so the card can enter SPI idle state. */
    sd_idle_clocks(sd);

    /*
     * CMD0 puts the card into idle state.
     * A valid card should answer with R1_IDLE_STATE.
     */
    cs_select(sd);
    st = sd_send_cmd(sd, CMD0, 0u, 0x95u, &r1);
    cs_deselect(sd);
    (void)spi_xfer_byte(sd, 0xFFu, NULL);

    if (st != SD_SPI_OK)
    {
        return st;
    }

    if (r1 != R1_IDLE_STATE)
    {
        return SD_SPI_ERR;
    }

    /*
     * CMD8 checks whether the card supports the v2 voltage/pattern command.
     * If accepted, we read the trailing R7 response and verify the echo.
     */
    {
        uint8_t r7[4] = { 0u, 0u, 0u, 0u };

        cs_select(sd);
        st = sd_send_cmd(sd, CMD8, 0x000001AAu, 0x87u, &r1);
        if (st != SD_SPI_OK)
        {
            cs_deselect(sd);
            return st;
        }

        if (r1 == R1_IDLE_STATE)
        {
            st = spi_read(sd, r7, 4u);
            cs_deselect(sd);
            (void)spi_xfer_byte(sd, 0xFFu, NULL);

            if (st != SD_SPI_OK)
            {
                return st;
            }

            /* Check the echoed voltage/pattern field. */
            {
                uint32_t echo = ((uint32_t)r7[2] << 8) | r7[3];
                if (echo != 0x01AAu)
                {
                    return SD_SPI_ERR;
                }
            }

            v2 = true;
        }
        else
        {
            /*
             * If CMD8 is illegal, the card is likely an older SDSC card.
             * Any other response is treated as an unexpected error.
             */
            cs_deselect(sd);
            (void)spi_xfer_byte(sd, 0xFFu, NULL);

            if ((r1 & R1_ILLEGAL_COMMAND) == 0u)
            {
                return SD_SPI_ERR;
            }
        }
    }

    /*
     * Repeatedly issue ACMD41 until the card leaves idle state.
     * For v2 cards, set HCS in the argument to request high-capacity support.
     */
    {
        uint32_t t0 = osal_ticks_ms();

        do
        {
            uint32_t arg = v2 ? 0x40000000u : 0x00000000u;

            cs_select(sd);
            st = sd_send_acmd(sd, ACMD41, arg, &r1);
            cs_deselect(sd);
            (void)spi_xfer_byte(sd, 0xFFu, NULL);

            if (st != SD_SPI_OK)
            {
                return st;
            }

            if (r1 == 0x00u)
            {
                break;
            }
        }
        while ((osal_ticks_ms() - t0) < sd->timeout_ms);
    }

    if (r1 != 0x00u)
    {
        return SD_SPI_TIMEOUT;
    }

    /*
     * For v2 cards, CMD58 reads the OCR register so we can detect CCS.
     * CCS = 1 means block-addressed high-capacity card.
     */
    if (v2)
    {
        uint8_t ocr[4];

        cs_select(sd);
        st = sd_send_cmd(sd, CMD58, 0u, 0x01u, &r1);
        if (st != SD_SPI_OK)
        {
            cs_deselect(sd);
            return st;
        }

        if (r1 != 0x00u)
        {
            cs_deselect(sd);
            return SD_SPI_ERR;
        }

        st = spi_read(sd, ocr, 4u);
        cs_deselect(sd);
        (void)spi_xfer_byte(sd, 0xFFu, NULL);

        if (st != SD_SPI_OK)
        {
            return st;
        }

        sd->card_type = ((ocr[0] & 0x40u) != 0u) ? SD_SPI_CARD_SDHC : SD_SPI_CARD_SDSC;
    }
    else
    {
        sd->card_type = SD_SPI_CARD_SDSC;
    }

    /*
     * SDSC cards use byte addressing and require block length to be set to 512.
     * SDHC cards always use fixed 512-byte block addressing.
     */
    if (sd->card_type == SD_SPI_CARD_SDSC)
    {
        cs_select(sd);
        st = sd_send_cmd(sd, CMD16, SD_SPI_SECTOR_SIZE, 0x01u, &r1);
        cs_deselect(sd);
        (void)spi_xfer_byte(sd, 0xFFu, NULL);

        if (st != SD_SPI_OK)
        {
            return st;
        }

        if (r1 != 0x00u)
        {
            return SD_SPI_ERR;
        }
    }

    /* Read and decode the CSD register to discover total logical capacity. */
    st = sd_update_capacity(sd);
    if (st != SD_SPI_OK)
    {
        return st;
    }

    sd->initialized = true;
    return SD_SPI_OK;
}

sd_spi_status_t sd_spi_preerase(sd_spi_t *sd, uint32_t nblocks)
{
    uint8_t r1 = 0u;

    if ((sd == NULL) || !sd->initialized)
    {
        return SD_SPI_PARAM;
    }

    /* Pre-erase hint with zero blocks is simply treated as a no-op. */
    if (nblocks == 0u)
    {
        return SD_SPI_OK;
    }

    /*
     * ACMD23 informs the card about the upcoming multi-block write length.
     * Not all cards benefit from this, but many do.
     */
    cs_select(sd);
    sd_spi_status_t st = sd_send_acmd(sd, ACMD23, nblocks, &r1);
    cs_deselect(sd);
    (void)spi_xfer_byte(sd, 0xFFu, NULL);

    if (st != SD_SPI_OK)
    {
        return st;
    }

    return (r1 == 0x00u) ? SD_SPI_OK : SD_SPI_ERR;
}

sd_spi_status_t sd_spi_read(sd_spi_t *sd, uint32_t lba, void *buf, uint32_t count)
{
    uint8_t *p = (uint8_t *)buf;
    uint8_t r1 = 0u;

    if ((sd == NULL) || !sd->initialized || (buf == NULL) || (count == 0u))
    {
        return SD_SPI_PARAM;
    }

    /*
     * Single-block read path:
     * CMD17 -> wait for data token -> read 512-byte sector.
     */
    if (count == 1u)
    {
        cs_select(sd);

        sd_spi_status_t st = sd_send_cmd(sd, CMD17, lba_to_arg(sd, lba), 0x01u, &r1);
        if (st != SD_SPI_OK)
        {
            cs_deselect(sd);
            return st;
        }

        if (r1 != 0x00u)
        {
            cs_deselect(sd);
            return SD_SPI_ERR;
        }

        st = sd_read_data_packet(sd, p, SD_SPI_SECTOR_SIZE, sd->timeout_ms);

        cs_deselect(sd);
        (void)spi_xfer_byte(sd, 0xFFu, NULL);

        return st;
    }

    /*
     * Multi-block read path:
     * CMD18 -> read N data packets -> CMD12 stop transmission.
     */
    cs_select(sd);

    sd_spi_status_t st = sd_send_cmd(sd, CMD18, lba_to_arg(sd, lba), 0x01u, &r1);
    if (st != SD_SPI_OK)
    {
        cs_deselect(sd);
        return st;
    }

    if (r1 != 0x00u)
    {
        cs_deselect(sd);
        return SD_SPI_ERR;
    }

    for (uint32_t i = 0u; i < count; i++)
    {
        st = sd_read_data_packet(sd, p, SD_SPI_SECTOR_SIZE, sd->timeout_ms);
        if (st != SD_SPI_OK)
        {
            break;
        }

        p += SD_SPI_SECTOR_SIZE;
    }

    /*
     * CMD12 stops a multi-block read stream.
     * Many cards return one extra stuff byte before the R1 response.
     */
    {
        uint8_t dump = 0u;
        (void)spi_xfer_byte(sd, 0xFFu, &dump);
        (void)sd_send_cmd(sd, CMD12, 0u, 0x01u, &r1);
        (void)sd_wait_not_busy(sd, sd->timeout_ms);
    }

    cs_deselect(sd);
    (void)spi_xfer_byte(sd, 0xFFu, NULL);

    return st;
}

sd_spi_status_t sd_spi_write(sd_spi_t *sd, uint32_t lba, const void *buf, uint32_t count)
{
    const uint8_t *p = (const uint8_t *)buf;
    uint8_t r1 = 0u;

    if ((sd == NULL) || !sd->initialized || (buf == NULL) || (count == 0u))
    {
        return SD_SPI_PARAM;
    }

    /*
     * Single-block write path:
     * CMD24 -> transmit one data packet -> wait for card ready.
     */
    if (count == 1u)
    {
        cs_select(sd);

        sd_spi_status_t st = sd_send_cmd(sd, CMD24, lba_to_arg(sd, lba), 0x01u, &r1);
        if (st != SD_SPI_OK)
        {
            cs_deselect(sd);
            return st;
        }

        if (r1 != 0x00u)
        {
            cs_deselect(sd);
            return SD_SPI_ERR;
        }

        st = sd_write_data_packet(sd, SD_TOKEN_START_BLOCK, p, SD_SPI_SECTOR_SIZE);

        cs_deselect(sd);
        (void)spi_xfer_byte(sd, 0xFFu, NULL);

        return st;
    }

    /*
     * Multi-block write path:
     * CMD25 -> transmit N data packets with multi-write token -> stop token.
     */
    cs_select(sd);

    sd_spi_status_t st = sd_send_cmd(sd, CMD25, lba_to_arg(sd, lba), 0x01u, &r1);
    if (st != SD_SPI_OK)
    {
        cs_deselect(sd);
        return st;
    }

    if (r1 != 0x00u)
    {
        cs_deselect(sd);
        return SD_SPI_ERR;
    }

    for (uint32_t i = 0u; i < count; i++)
    {
        st = sd_write_data_packet(sd, SD_TOKEN_MULTI_WRITE, p, SD_SPI_SECTOR_SIZE);
        if (st != SD_SPI_OK)
        {
            break;
        }

        p += SD_SPI_SECTOR_SIZE;
    }

    /* End the multi-block write stream with the stop transmission token. */
    {
        uint8_t resp = 0u;
        (void)spi_xfer_byte(sd, SD_TOKEN_STOP_TRAN, &resp);
        (void)sd_wait_not_busy(sd, sd->timeout_ms);
    }

    cs_deselect(sd);
    (void)spi_xfer_byte(sd, 0xFFu, NULL);

    return st;
}

uint32_t sd_spi_get_sector_count(const sd_spi_t *sd)
{
    return (sd != NULL) ? sd->sector_count : 0u;
}

sd_spi_card_t sd_spi_get_card_type(const sd_spi_t *sd)
{
    return (sd != NULL) ? sd->card_type : SD_SPI_CARD_UNKNOWN;
}

const char *sd_spi_strerror(sd_spi_status_t st)
{
    switch (st)
    {
        case SD_SPI_OK:      return "OK";
        case SD_SPI_ERR:     return "ERR";
        case SD_SPI_TIMEOUT: return "TIMEOUT";
        case SD_SPI_PARAM:   return "PARAM";
        case SD_SPI_NOCARD:  return "NOCARD";
        case SD_SPI_CRC_ERR: return "CRC";
        default:             return "?";
    }
}